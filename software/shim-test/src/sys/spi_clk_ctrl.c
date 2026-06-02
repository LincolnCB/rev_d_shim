#include <stdio.h>
#include <stdlib.h>
#include "spi_clk_ctrl.h"
#include "map_memory.h"

#define SPI_CLK_RESET_KEY        ((uint32_t)0x0000000A)
#define SPI_CLK_LOCKED_MASK      ((uint32_t)0x1)

#define SPI_CLK_FB_DIV_MASK      ((uint32_t)0x000000FF)
#define SPI_CLK_FB_MULT_WHOLE_MASK ((uint32_t)0x0000FF00)
#define SPI_CLK_FB_MULT_FRAC_MASK  ((uint32_t)0x03FF0000)
#define SPI_CLK_FB_MULT_WHOLE_SHIFT 8
#define SPI_CLK_FB_MULT_FRAC_SHIFT  16

#define SPI_CLK_DIV_WHOLE_MASK   ((uint32_t)0x000000FF)
#define SPI_CLK_DIV_FRAC_MASK    ((uint32_t)0x0003FF00)
#define SPI_CLK_DIV_FRAC_SHIFT   8

#define SPI_CLK_ENABLE_LOAD_DEFAULT ((uint32_t)0x1)
#define SPI_CLK_ENABLE_LOAD_USER    ((uint32_t)0x3)

#define SPI_CLK_PHASE_MIN        (-360000)
#define SPI_CLK_PHASE_MAX        (360000)
#define SPI_CLK_DUTY_MAX         ((uint32_t)100000)

static uint32_t encode_whole_frac_8_10(double value, const char *name) {
  if (value < 0.0) {
    fprintf(stderr, "Invalid %s: %.6f. Must be non-negative.\n", name, value);
    exit(EXIT_FAILURE);
  }

  uint32_t whole = (uint32_t)value;
  double frac_float = (value - (double)whole) * 1000.0;
  uint32_t frac = (uint32_t)(frac_float + 0.5);

  // Fractional field is constrained to 1/8 increments => 125 milli-units.
  frac = ((frac + 62) / 125) * 125;

  if (frac >= 1000) {
    whole += 1;
    frac = 0;
  }

  if (whole > 0xFF) {
    fprintf(stderr, "Invalid %s: %.6f. Whole part exceeds 8 bits.\n", name, value);
    exit(EXIT_FAILURE);
  }

  return ((whole & 0xFF) << 10) | (frac & 0x3FF);
}

static double decode_whole_frac_8_10(uint32_t whole, uint32_t frac) {
  return (double)whole + ((double)frac / 1000.0);
}

// Function to create SPI clock control structure
struct spi_clk_ctrl_t create_spi_clk_ctrl(bool verbose) {
  struct spi_clk_ctrl_t spi_clk_ctrl;

  // Map SPI clock control base address
  volatile uint32_t *spi_clk_ptr = map_32bit_memory(SPI_CLK_BASE, SPI_CLK_WORDCOUNT, "SPI Clock Ctrl", verbose);
  if (spi_clk_ptr == NULL) {
    fprintf(stderr, "Failed to map SPI clock control memory region.\n");
    exit(EXIT_FAILURE);
  }

  // Initialize the SPI clock control structure with the mapped memory addresses
  spi_clk_ctrl.reset = spi_clk_ptr + (SPI_CLK_RESET_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.status = spi_clk_ptr + (SPI_CLK_STATUS_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.fb_cfg = spi_clk_ptr + (SPI_CLK_FB_CFG_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.fb_phase = spi_clk_ptr + (SPI_CLK_FB_PHASE_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.div = spi_clk_ptr + (SPI_CLK_DIV_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.phase = spi_clk_ptr + (SPI_CLK_PHASE_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.duty = spi_clk_ptr + (SPI_CLK_CFG_4_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.enable = spi_clk_ptr + (SPI_CLK_ENABLE_OFFSET / sizeof(uint32_t));

  return spi_clk_ctrl;
}

void spi_clk_ctrl_reset(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->reset) = SPI_CLK_RESET_KEY;
  if (verbose) {
    printf("SPI clock reset requested (key=0x%08X).\n", SPI_CLK_RESET_KEY);
  }
}

bool spi_clk_ctrl_locked(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  bool locked = ((*(spi_clk_ctrl->status)) & SPI_CLK_LOCKED_MASK) != 0;
  if (verbose) {
    printf("SPI clock locked: %s\n", locked ? "true" : "false");
  }
  return locked;
}

uint8_t spi_clk_ctrl_get_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint8_t clk_fb_div = (uint8_t)(*(spi_clk_ctrl->fb_cfg) & SPI_CLK_FB_DIV_MASK);
  if (verbose) {
    printf("SPI clock clk_fb_div: %u\n", clk_fb_div);
  }
  return clk_fb_div;
}

void spi_clk_ctrl_set_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, uint8_t clk_fb_div, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  reg = (reg & ~SPI_CLK_FB_DIV_MASK) | ((uint32_t)clk_fb_div & SPI_CLK_FB_DIV_MASK);
  *(spi_clk_ctrl->fb_cfg) = reg;

  if (verbose) {
    printf("SPI clock clk_fb_div set to %u (fb_cfg=0x%08X).\n", clk_fb_div, reg);
  }
}

double spi_clk_ctrl_get_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  uint32_t whole = (reg & SPI_CLK_FB_MULT_WHOLE_MASK) >> SPI_CLK_FB_MULT_WHOLE_SHIFT;
  uint32_t frac = (reg & SPI_CLK_FB_MULT_FRAC_MASK) >> SPI_CLK_FB_MULT_FRAC_SHIFT;
  double clk_fb_mult = decode_whole_frac_8_10(whole, frac);

  if (verbose) {
    printf("SPI clock clk_fb_mult: %.3f (whole=%u, frac=%u).\n", clk_fb_mult, whole, frac);
  }

  return clk_fb_mult;
}

void spi_clk_ctrl_set_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, double clk_fb_mult, bool verbose) {
  uint32_t encoded = encode_whole_frac_8_10(clk_fb_mult, "clk_fb_mult");
  uint32_t whole = (encoded >> 10) & 0xFF;
  uint32_t frac = encoded & 0x3FF;

  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  reg &= ~(SPI_CLK_FB_MULT_WHOLE_MASK | SPI_CLK_FB_MULT_FRAC_MASK);
  reg |= (whole << SPI_CLK_FB_MULT_WHOLE_SHIFT) | (frac << SPI_CLK_FB_MULT_FRAC_SHIFT);
  *(spi_clk_ctrl->fb_cfg) = reg;

  if (verbose) {
    printf("SPI clock clk_fb_mult set to %.3f (whole=%u, frac=%u, fb_cfg=0x%08X).\n", clk_fb_mult, whole, frac, reg);
  }
}

int32_t spi_clk_ctrl_get_clk_fb_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  int32_t clk_fb_phase = (int32_t)(*(spi_clk_ctrl->fb_phase));
  if (verbose) {
    printf("SPI clock clk_fb_phase: %d mdeg\n", clk_fb_phase);
  }
  return clk_fb_phase;
}

void spi_clk_ctrl_set_clk_fb_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, int32_t clk_fb_phase, bool verbose) {
  if ((clk_fb_phase < SPI_CLK_PHASE_MIN) || (clk_fb_phase > SPI_CLK_PHASE_MAX)) {
    fprintf(stderr, "Invalid clk_fb_phase: %d. Must be in [%d, %d] mdeg.\n", clk_fb_phase, SPI_CLK_PHASE_MIN, SPI_CLK_PHASE_MAX);
    exit(EXIT_FAILURE);
  }

  *(spi_clk_ctrl->fb_phase) = (uint32_t)clk_fb_phase;
  if (verbose) {
    printf("SPI clock clk_fb_phase set to %d mdeg.\n", clk_fb_phase);
  }
}

double spi_clk_ctrl_get_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->div);
  uint32_t whole = reg & SPI_CLK_DIV_WHOLE_MASK;
  uint32_t frac = (reg & SPI_CLK_DIV_FRAC_MASK) >> SPI_CLK_DIV_FRAC_SHIFT;
  double clk_div = decode_whole_frac_8_10(whole, frac);

  if (verbose) {
    printf("SPI clock clk_div: %.3f (whole=%u, frac=%u).\n", clk_div, whole, frac);
  }

  return clk_div;
}

void spi_clk_ctrl_set_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, double clk_div, bool verbose) {
  uint32_t encoded = encode_whole_frac_8_10(clk_div, "clk_div");
  uint32_t whole = (encoded >> 10) & 0xFF;
  uint32_t frac = encoded & 0x3FF;

  uint32_t reg = *(spi_clk_ctrl->div);
  reg &= ~(SPI_CLK_DIV_WHOLE_MASK | SPI_CLK_DIV_FRAC_MASK);
  reg |= (whole & SPI_CLK_DIV_WHOLE_MASK) | (frac << SPI_CLK_DIV_FRAC_SHIFT);
  *(spi_clk_ctrl->div) = reg;

  if (verbose) {
    printf("SPI clock clk_div set to %.3f (whole=%u, frac=%u, div=0x%08X).\n", clk_div, whole, frac, reg);
  }
}

int32_t spi_clk_ctrl_get_clk_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  int32_t clk_phase = (int32_t)(*(spi_clk_ctrl->phase));
  if (verbose) {
    printf("SPI clock clk_phase: %d mdeg\n", clk_phase);
  }
  return clk_phase;
}

void spi_clk_ctrl_set_clk_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, int32_t clk_phase, bool verbose) {
  if ((clk_phase < SPI_CLK_PHASE_MIN) || (clk_phase > SPI_CLK_PHASE_MAX)) {
    fprintf(stderr, "Invalid clk_phase: %d. Must be in [%d, %d] mdeg.\n", clk_phase, SPI_CLK_PHASE_MIN, SPI_CLK_PHASE_MAX);
    exit(EXIT_FAILURE);
  }

  *(spi_clk_ctrl->phase) = (uint32_t)clk_phase;
  if (verbose) {
    printf("SPI clock clk_phase set to %d mdeg.\n", clk_phase);
  }
}

uint32_t spi_clk_ctrl_get_clk_duty(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t clk_duty = *(spi_clk_ctrl->duty);
  if (verbose) {
    printf("SPI clock clk_duty: %u m%%\n", clk_duty);
  }
  return clk_duty;
}

void spi_clk_ctrl_set_clk_duty(struct spi_clk_ctrl_t *spi_clk_ctrl, uint32_t clk_duty, bool verbose) {
  if (clk_duty > SPI_CLK_DUTY_MAX) {
    fprintf(stderr, "Invalid clk_duty: %u. Must be in [0, %u] m%%.\n", clk_duty, SPI_CLK_DUTY_MAX);
    exit(EXIT_FAILURE);
  }

  *(spi_clk_ctrl->duty) = clk_duty;
  if (verbose) {
    printf("SPI clock clk_duty set to %u m%%.\n", clk_duty);
  }
}

void spi_clk_ctrl_load_default(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->enable) = SPI_CLK_ENABLE_LOAD_DEFAULT;
  if (verbose) {
    printf("SPI clock load_default requested (enable=0x%08X).\n", SPI_CLK_ENABLE_LOAD_DEFAULT);
  }
}

void spi_clk_ctrl_load_user(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->enable) = SPI_CLK_ENABLE_LOAD_USER;
  if (verbose) {
    printf("SPI clock load_user requested (enable=0x%08X).\n", SPI_CLK_ENABLE_LOAD_USER);
  }
}
