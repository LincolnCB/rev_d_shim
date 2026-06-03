#include <stdio.h>
#include <stdlib.h>
#include "spi_clk_ctrl.h"
#include "sys_sts.h"
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
#define SPI_CLK_FB_FREQ_MIN_HZ   (600000000.0)
#define SPI_CLK_FB_FREQ_MAX_HZ   (1200000000.0)

#define CLK_FB_MULT_WHOLE(encoded) (((encoded) & SPI_CLK_FB_MULT_WHOLE_MASK) >> SPI_CLK_FB_MULT_WHOLE_SHIFT)
#define CLK_FB_MULT_FRAC(encoded)  (((encoded) & SPI_CLK_FB_MULT_FRAC_MASK) >> SPI_CLK_FB_MULT_FRAC_SHIFT)
#define CLK_DIV_WHOLE(encoded)     ((encoded) & SPI_CLK_DIV_WHOLE_MASK)
#define CLK_DIV_FRAC(encoded)      (((encoded) & SPI_CLK_DIV_FRAC_MASK) >> SPI_CLK_DIV_FRAC_SHIFT)

// Encode a double value into aligned bits for feedback multiplier
static int32_t encode_clk_fb_mult(double value) {
  if (value < 0.0) {
    return -1;
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
    return -1;
  }

  return (int32_t)(((whole << SPI_CLK_FB_MULT_WHOLE_SHIFT) & SPI_CLK_FB_MULT_WHOLE_MASK) |
                   ((frac << SPI_CLK_FB_MULT_FRAC_SHIFT) & SPI_CLK_FB_MULT_FRAC_MASK));
}

// Encode a double value into aligned bits for clock divider
static int32_t encode_clk_div(double value) {
  if (value < 0.0) {
    return -1;
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
    return -1;
  }

  return (int32_t)((whole & SPI_CLK_DIV_WHOLE_MASK) |
                   ((frac << SPI_CLK_DIV_FRAC_SHIFT) & SPI_CLK_DIV_FRAC_MASK));
}

// Validate that the resulting feedback clock frequency is within the valid range of 600 to 1200 MHz
static int validate_clk_fb_settings(uint32_t source_clk_freq_hz, double clk_fb_mult, uint8_t clk_fb_div, bool verbose) {
  if (source_clk_freq_hz == 0) {
    if (verbose) {
      fprintf(stderr, "Invalid source clock frequency: 0 Hz. Cannot validate feedback clock settings.\n");
    }
    return -1;
  }

  if (clk_fb_div == 0) {
    if (verbose) {
      fprintf(stderr, "Invalid clk_fb_div: 0. Divider must be between 1 and 255.\n");
    }
    return -1;
  }

  double fb_freq_hz = ((double)source_clk_freq_hz * clk_fb_mult) / (double)clk_fb_div;
  if ((fb_freq_hz < SPI_CLK_FB_FREQ_MIN_HZ) || (fb_freq_hz > SPI_CLK_FB_FREQ_MAX_HZ)) {
    if (verbose) {
      fprintf(stderr,
              "Invalid SPI feedback clock configuration: %.3f MHz source * %.3f / %u = %.3f MHz; required range is 600.000 to 1200.000 MHz.\n",
              ((double)source_clk_freq_hz) / 1000000.0,
              clk_fb_mult,
              clk_fb_div,
              fb_freq_hz / 1000000.0);
    }
    return -1;
  }

  return 0;
}

// Write feedback clock configuration to hardware register (assuming values are already validated)
static int write_clk_fb_cfg(struct spi_clk_ctrl_t *spi_clk_ctrl, uint8_t clk_fb_div, uint32_t clk_fb_mult_encoded, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  reg &= ~(SPI_CLK_FB_DIV_MASK | SPI_CLK_FB_MULT_WHOLE_MASK | SPI_CLK_FB_MULT_FRAC_MASK);
  reg |= ((uint32_t)clk_fb_div & SPI_CLK_FB_DIV_MASK);
  reg |= clk_fb_mult_encoded & (SPI_CLK_FB_MULT_WHOLE_MASK | SPI_CLK_FB_MULT_FRAC_MASK);
  *(spi_clk_ctrl->fb_cfg) = reg;

  if (verbose) {
    printf("SPI clock feedback config set (clk_fb_mult=%.3f, clk_fb_div=%u, fb_cfg=0x%08X).\n",
           (double)CLK_FB_MULT_WHOLE(clk_fb_mult_encoded) + (double)CLK_FB_MULT_FRAC(clk_fb_mult_encoded) / 1000.0,
           clk_fb_div,
           reg);
  }

  return 0;
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

// Reset the SPI clock
void spi_clk_ctrl_reset(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->reset) = SPI_CLK_RESET_KEY;
  if (verbose) {
    printf("SPI clock reset requested (key=0x%08X).\n", SPI_CLK_RESET_KEY);
  }
}

// Check if the SPI clock is locked
bool spi_clk_ctrl_locked(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  bool locked = ((*(spi_clk_ctrl->status)) & SPI_CLK_LOCKED_MASK) != 0;
  if (verbose) {
    printf("SPI clock locked: %s\n", locked ? "true" : "false");
  }
  return locked;
}

// Get feedback clock divider
uint8_t spi_clk_ctrl_get_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint8_t clk_fb_div = (uint8_t)(*(spi_clk_ctrl->fb_cfg) & SPI_CLK_FB_DIV_MASK);
  if (verbose) {
    printf("SPI clock clk_fb_div: %u\n", clk_fb_div);
  }
  return clk_fb_div;
}

// Set feedback clock divider with validation
int spi_clk_ctrl_set_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, uint8_t clk_fb_div, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  uint32_t clk_fb_mult_encoded = reg & (SPI_CLK_FB_MULT_WHOLE_MASK | SPI_CLK_FB_MULT_FRAC_MASK);
  double clk_fb_mult = (double)CLK_FB_MULT_WHOLE(clk_fb_mult_encoded) + (double)CLK_FB_MULT_FRAC(clk_fb_mult_encoded) / 1000.0;
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(spi_clk_ctrl, clk_fb_div, clk_fb_mult_encoded, verbose);
}

// Get feedback clock multiplier
double spi_clk_ctrl_get_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->fb_cfg);
  uint32_t whole = CLK_FB_MULT_WHOLE(reg);
  uint32_t frac = CLK_FB_MULT_FRAC(reg);
  double clk_fb_mult = (double)whole + (double)frac / 1000.0;

  if (verbose) {
    printf("SPI clock clk_fb_mult: %.3f (whole=%u, frac=%u).\n", clk_fb_mult, whole, frac);
  }

  return clk_fb_mult;
}

// Set feedback clock multiplier with validation
int spi_clk_ctrl_set_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, bool verbose) {
  int32_t encoded = encode_clk_fb_mult(clk_fb_mult);
  if (encoded < 0) {
    return -1;
  }

  double quantized_clk_fb_mult = (double)CLK_FB_MULT_WHOLE((uint32_t)encoded) + (double)CLK_FB_MULT_FRAC((uint32_t)encoded) / 1000.0;
  uint8_t clk_fb_div = spi_clk_ctrl_get_clk_fb_div(spi_clk_ctrl, false);
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, quantized_clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(spi_clk_ctrl, clk_fb_div, (uint32_t)encoded, verbose);
}

// Set feedback clock multiplier and divider together with validation
int spi_clk_ctrl_set_clk_fb(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, uint8_t clk_fb_div, bool verbose) {
  int32_t encoded = encode_clk_fb_mult(clk_fb_mult);
  if (encoded < 0) {
    return -1;
  }

  double quantized_clk_fb_mult = (double)CLK_FB_MULT_WHOLE((uint32_t)encoded) + (double)CLK_FB_MULT_FRAC((uint32_t)encoded) / 1000.0;
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, quantized_clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(spi_clk_ctrl, clk_fb_div, (uint32_t)encoded, verbose);
}

// Get feedback clock phase
int32_t spi_clk_ctrl_get_clk_fb_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  int32_t clk_fb_phase = (int32_t)(*(spi_clk_ctrl->fb_phase));
  if (verbose) {
    printf("SPI clock clk_fb_phase: %d mdeg\n", clk_fb_phase);
  }
  return clk_fb_phase;
}

// Set feedback clock phase with validation
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

// Get output clock divider
double spi_clk_ctrl_get_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t reg = *(spi_clk_ctrl->div);
  uint32_t whole = CLK_DIV_WHOLE(reg);
  uint32_t frac = CLK_DIV_FRAC(reg);
  double clk_div = (double)whole + (double)frac / 1000.0;

  if (verbose) {
    printf("SPI clock clk_div: %.3f (whole=%u, frac=%u).\n", clk_div, whole, frac);
  }

  return clk_div;
}

// Set output clock divider with validation
void spi_clk_ctrl_set_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, double clk_div, bool verbose) {
  int32_t encoded = encode_clk_div(clk_div);
  if (encoded < 0) {
    return;
  }

  uint32_t reg = *(spi_clk_ctrl->div);
  reg &= ~(SPI_CLK_DIV_WHOLE_MASK | SPI_CLK_DIV_FRAC_MASK);
  reg |= ((uint32_t)encoded & (SPI_CLK_DIV_WHOLE_MASK | SPI_CLK_DIV_FRAC_MASK));
  *(spi_clk_ctrl->div) = reg;

  if (verbose) {
    uint32_t whole = reg & SPI_CLK_DIV_WHOLE_MASK;
    uint32_t frac = (reg & SPI_CLK_DIV_FRAC_MASK) >> SPI_CLK_DIV_FRAC_SHIFT;
    printf("SPI clock clk_div set to %.3f (whole=%u, frac=%u, div=0x%08X).\n", clk_div, whole, frac, reg);
  }
}

// Get output clock phase
int32_t spi_clk_ctrl_get_clk_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  int32_t clk_phase = (int32_t)(*(spi_clk_ctrl->phase));
  if (verbose) {
    printf("SPI clock clk_phase: %d mdeg\n", clk_phase);
  }
  return clk_phase;
}

// Set output clock phase with validation
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

// Get output clock duty cycle
uint32_t spi_clk_ctrl_get_clk_duty(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  uint32_t clk_duty = *(spi_clk_ctrl->duty);
  if (verbose) {
    printf("SPI clock clk_duty: %u m%%\n", clk_duty);
  }
  return clk_duty;
}

// Set output clock duty cycle with validation
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

// Initiate reconfiguration of the SPI clock with the default settings
void spi_clk_ctrl_load_default(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->enable) = SPI_CLK_ENABLE_LOAD_DEFAULT;
  if (verbose) {
    printf("SPI clock load_default requested (enable=0x%08X).\n", SPI_CLK_ENABLE_LOAD_DEFAULT);
  }
}

// Initiate reconfiguration of the SPI clock with the user-provided settings in registers
void spi_clk_ctrl_load_user(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose) {
  *(spi_clk_ctrl->enable) = SPI_CLK_ENABLE_LOAD_USER;
  if (verbose) {
    printf("SPI clock load_user requested (enable=0x%08X).\n", SPI_CLK_ENABLE_LOAD_USER);
  }
}
