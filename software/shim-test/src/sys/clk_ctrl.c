#include <stdio.h>
#include <stdlib.h>
#include "clk_ctrl.h"
#include "sys_sts.h"
#include "map_memory.h"

#define CLK_RESET_KEY        ((uint32_t)0x0000000A)
#define CLK_LOCKED_MASK      ((uint32_t)0x1)

#define CLK_FB_DIV_MASK      ((uint32_t)0x000000FF)
#define CLK_FB_MULT_WHOLE_MASK ((uint32_t)0x0000FF00)
#define CLK_FB_MULT_FRAC_MASK  ((uint32_t)0x03FF0000)
#define CLK_FB_MULT_WHOLE_SHIFT 8
#define CLK_FB_MULT_FRAC_SHIFT  16

#define CLK_DIV_WHOLE_MASK   ((uint32_t)0x000000FF)
#define CLK_DIV_FRAC_MASK    ((uint32_t)0x0003FF00)
#define CLK_DIV_FRAC_SHIFT   8

#define CLK_ENABLE_LOAD_DEFAULT ((uint32_t)0x1)
#define CLK_ENABLE_LOAD_USER    ((uint32_t)0x3)

#define CLK_PHASE_MIN        (-360000)
#define CLK_PHASE_MAX        (360000)
#define CLK_DUTY_MAX         ((uint32_t)100000)
#define CLK_FB_FREQ_MIN_HZ   (600000000.0)
#define CLK_FB_FREQ_MAX_HZ   (1200000000.0)

#define CLK_MAX_FREQ_HZ      (100000000.0) // 100 MHz max frequency for SPI system

#define CLK_FB_MULT_WHOLE(encoded) (((encoded) & CLK_FB_MULT_WHOLE_MASK) >> CLK_FB_MULT_WHOLE_SHIFT)
#define CLK_FB_MULT_FRAC(encoded)  (((encoded) & CLK_FB_MULT_FRAC_MASK) >> CLK_FB_MULT_FRAC_SHIFT)
#define CLK_DIV_WHOLE(encoded)     ((encoded) & CLK_DIV_WHOLE_MASK)
#define CLK_DIV_FRAC(encoded)      (((encoded) & CLK_DIV_FRAC_MASK) >> CLK_DIV_FRAC_SHIFT)

// Simple ceil/floor functions for positive double values without including math.h
static double pos_ceil(double x) {
  if (x < 0.0) {
    return 0.0;
  }
  uint32_t xi = (uint32_t)x;
  return (x > (double)xi) ? (double)(xi + 1) : (double)xi;
}

static double pos_floor(double x) {
  if (x < 0.0) {
    return 0.0;
  }
  return (double)(uint32_t)x;
}

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

  return (int32_t)(((whole << CLK_FB_MULT_WHOLE_SHIFT) & CLK_FB_MULT_WHOLE_MASK) |
                   ((frac << CLK_FB_MULT_FRAC_SHIFT) & CLK_FB_MULT_FRAC_MASK));
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

  return (int32_t)((whole & CLK_DIV_WHOLE_MASK) |
                   ((frac << CLK_DIV_FRAC_SHIFT) & CLK_DIV_FRAC_MASK));
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
  if ((fb_freq_hz < CLK_FB_FREQ_MIN_HZ) || (fb_freq_hz > CLK_FB_FREQ_MAX_HZ)) {
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
static int write_clk_fb_cfg(struct clk_ctrl_t *clk_ctrl, uint8_t clk_fb_div, uint32_t clk_fb_mult_encoded, bool verbose) {
  uint32_t reg = *(clk_ctrl->fb_cfg);
  reg &= ~(CLK_FB_DIV_MASK | CLK_FB_MULT_WHOLE_MASK | CLK_FB_MULT_FRAC_MASK);
  reg |= ((uint32_t)clk_fb_div & CLK_FB_DIV_MASK);
  reg |= clk_fb_mult_encoded & (CLK_FB_MULT_WHOLE_MASK | CLK_FB_MULT_FRAC_MASK);
  *(clk_ctrl->fb_cfg) = reg;

  if (verbose) {
    printf("SPI clock feedback config set (clk_fb_mult=%.3f, clk_fb_div=%u, fb_cfg=0x%08X).\n",
           (double)CLK_FB_MULT_WHOLE(clk_fb_mult_encoded) + (double)CLK_FB_MULT_FRAC(clk_fb_mult_encoded) / 1000.0,
           clk_fb_div,
           reg);
  }

  return 0;
}

// Function to create SPI clock control structure
struct clk_ctrl_t create_clk_ctrl(bool verbose) {
  struct clk_ctrl_t clk_ctrl;

  // Map SPI clock control base address
  volatile uint32_t *clk_ptr = map_32bit_memory(CLK_BASE, CLK_WORDCOUNT, "SPI Clock Ctrl", verbose);
  if (clk_ptr == NULL) {
    fprintf(stderr, "Failed to map SPI clock control memory region.\n");
    exit(EXIT_FAILURE);
  }

  // Initialize the SPI clock control structure with the mapped memory addresses
  clk_ctrl.reset = clk_ptr + (CLK_RESET_OFFSET / sizeof(uint32_t));
  clk_ctrl.status = clk_ptr + (CLK_STATUS_OFFSET / sizeof(uint32_t));
  clk_ctrl.fb_cfg = clk_ptr + (CLK_FB_CFG_OFFSET / sizeof(uint32_t));
  clk_ctrl.fb_phase = clk_ptr + (CLK_FB_PHASE_OFFSET / sizeof(uint32_t));
  clk_ctrl.div = clk_ptr + (CLK_DIV_OFFSET / sizeof(uint32_t));
  clk_ctrl.phase = clk_ptr + (CLK_PHASE_OFFSET / sizeof(uint32_t));
  clk_ctrl.duty = clk_ptr + (CLK_CFG_4_OFFSET / sizeof(uint32_t));
  clk_ctrl.enable = clk_ptr + (CLK_ENABLE_OFFSET / sizeof(uint32_t));

  return clk_ctrl;
}

// Reset the SPI clock
void clk_ctrl_reset(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  *(clk_ctrl->reset) = CLK_RESET_KEY;
  if (verbose) {
    printf("SPI clock reset requested (key=0x%08X).\n", CLK_RESET_KEY);
  }
}

// Check if the SPI clock is locked
bool clk_ctrl_locked(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  bool locked = ((*(clk_ctrl->status)) & CLK_LOCKED_MASK) != 0;
  if (verbose) {
    printf("SPI clock locked: %s\n", locked ? "true" : "false");
  }
  return locked;
}

// Get feedback clock divider
uint8_t clk_ctrl_get_clk_fb_div(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  uint8_t clk_fb_div = (uint8_t)(*(clk_ctrl->fb_cfg) & CLK_FB_DIV_MASK);
  if (verbose) {
    printf("SPI clock clk_fb_div: %u\n", clk_fb_div);
  }
  return clk_fb_div;
}

// Set feedback clock divider with validation
int clk_ctrl_set_clk_fb_div(struct clk_ctrl_t *clk_ctrl, struct sys_sts_t *sys_sts, uint8_t clk_fb_div, bool verbose) {
  uint32_t reg = *(clk_ctrl->fb_cfg);
  uint32_t clk_fb_mult_encoded = reg & (CLK_FB_MULT_WHOLE_MASK | CLK_FB_MULT_FRAC_MASK);
  double clk_fb_mult = (double)CLK_FB_MULT_WHOLE(clk_fb_mult_encoded) + (double)CLK_FB_MULT_FRAC(clk_fb_mult_encoded) / 1000.0;
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(clk_ctrl, clk_fb_div, clk_fb_mult_encoded, verbose);
}

// Get feedback clock multiplier
double clk_ctrl_get_clk_fb_mult(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  uint32_t reg = *(clk_ctrl->fb_cfg);
  uint32_t whole = CLK_FB_MULT_WHOLE(reg);
  uint32_t frac = CLK_FB_MULT_FRAC(reg);
  double clk_fb_mult = (double)whole + (double)frac / 1000.0;

  if (verbose) {
    printf("SPI clock clk_fb_mult: %.3f (whole=%u, frac=%u).\n", clk_fb_mult, whole, frac);
  }

  return clk_fb_mult;
}

// Set feedback clock multiplier with validation
int clk_ctrl_set_clk_fb_mult(struct clk_ctrl_t *clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, bool verbose) {
  int32_t encoded = encode_clk_fb_mult(clk_fb_mult);
  if (encoded < 0) {
    return -1;
  }

  double quantized_clk_fb_mult = (double)CLK_FB_MULT_WHOLE((uint32_t)encoded) + (double)CLK_FB_MULT_FRAC((uint32_t)encoded) / 1000.0;
  uint8_t clk_fb_div = clk_ctrl_get_clk_fb_div(clk_ctrl, false);
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, quantized_clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(clk_ctrl, clk_fb_div, (uint32_t)encoded, verbose);
}

// Set feedback clock multiplier and divider together with validation
int clk_ctrl_set_clk_fb(struct clk_ctrl_t *clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, uint8_t clk_fb_div, bool verbose) {
  int32_t encoded = encode_clk_fb_mult(clk_fb_mult);
  if (encoded < 0) {
    return -1;
  }

  double quantized_clk_fb_mult = (double)CLK_FB_MULT_WHOLE((uint32_t)encoded) + (double)CLK_FB_MULT_FRAC((uint32_t)encoded) / 1000.0;
  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);

  if (validate_clk_fb_settings(source_clk_freq_hz, quantized_clk_fb_mult, clk_fb_div, verbose) != 0) {
    return -1;
  }

  return write_clk_fb_cfg(clk_ctrl, clk_fb_div, (uint32_t)encoded, verbose);
}

// Get feedback clock phase
int32_t clk_ctrl_get_clk_fb_phase(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  int32_t clk_fb_phase = (int32_t)(*(clk_ctrl->fb_phase));
  if (verbose) {
    printf("SPI clock clk_fb_phase: %d mdeg\n", clk_fb_phase);
  }
  return clk_fb_phase;
}

// Set feedback clock phase with validation
void clk_ctrl_set_clk_fb_phase(struct clk_ctrl_t *clk_ctrl, int32_t clk_fb_phase, bool verbose) {
  if ((clk_fb_phase < CLK_PHASE_MIN) || (clk_fb_phase > CLK_PHASE_MAX)) {
    fprintf(stderr, "Invalid clk_fb_phase: %d. Must be in [%d, %d] mdeg.\n", clk_fb_phase, CLK_PHASE_MIN, CLK_PHASE_MAX);
    exit(EXIT_FAILURE);
  }

  *(clk_ctrl->fb_phase) = (uint32_t)clk_fb_phase;
  if (verbose) {
    printf("SPI clock clk_fb_phase set to %d mdeg.\n", clk_fb_phase);
  }
}

// Get output clock divider
double clk_ctrl_get_clk_div(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  uint32_t reg = *(clk_ctrl->div);
  uint32_t whole = CLK_DIV_WHOLE(reg);
  uint32_t frac = CLK_DIV_FRAC(reg);
  double clk_div = (double)whole + (double)frac / 1000.0;

  if (verbose) {
    printf("SPI clock clk_div: %.3f (whole=%u, frac=%u).\n", clk_div, whole, frac);
  }

  return clk_div;
}

// Set output clock divider with validation
void clk_ctrl_set_clk_div(struct clk_ctrl_t *clk_ctrl, double clk_div, bool verbose) {
  int32_t encoded = encode_clk_div(clk_div);
  if (encoded < 0) {
    return;
  }

  uint32_t reg = *(clk_ctrl->div);
  reg &= ~(CLK_DIV_WHOLE_MASK | CLK_DIV_FRAC_MASK);
  reg |= ((uint32_t)encoded & (CLK_DIV_WHOLE_MASK | CLK_DIV_FRAC_MASK));
  *(clk_ctrl->div) = reg;

  if (verbose) {
    uint32_t whole = reg & CLK_DIV_WHOLE_MASK;
    uint32_t frac = (reg & CLK_DIV_FRAC_MASK) >> CLK_DIV_FRAC_SHIFT;
    printf("SPI clock clk_div set to %.3f (whole=%u, frac=%u, div=0x%08X).\n", clk_div, whole, frac, reg);
  }
}

// Get output clock phase
int32_t clk_ctrl_get_clk_phase(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  int32_t clk_phase = (int32_t)(*(clk_ctrl->phase));
  if (verbose) {
    printf("SPI clock clk_phase: %d mdeg\n", clk_phase);
  }
  return clk_phase;
}

// Set output clock phase with validation
void clk_ctrl_set_clk_phase(struct clk_ctrl_t *clk_ctrl, int32_t clk_phase, bool verbose) {
  if ((clk_phase < CLK_PHASE_MIN) || (clk_phase > CLK_PHASE_MAX)) {
    fprintf(stderr, "Invalid clk_phase: %d. Must be in [%d, %d] mdeg.\n", clk_phase, CLK_PHASE_MIN, CLK_PHASE_MAX);
    exit(EXIT_FAILURE);
  }

  *(clk_ctrl->phase) = (uint32_t)clk_phase;
  if (verbose) {
    printf("SPI clock clk_phase set to %d mdeg.\n", clk_phase);
  }
}

// Get output clock duty cycle
uint32_t clk_ctrl_get_clk_duty(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  uint32_t clk_duty = *(clk_ctrl->duty);
  if (verbose) {
    printf("SPI clock clk_duty: %u m%%\n", clk_duty);
  }
  return clk_duty;
}

// Load default clock settings
void clk_ctrl_load_default(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  *(clk_ctrl->enable) = CLK_ENABLE_LOAD_DEFAULT;
  if (verbose) {
    printf("SPI clock load_default triggered (enable=0x%08X).\n", CLK_ENABLE_LOAD_DEFAULT);
  }
}

// Load user clock settings
void clk_ctrl_load_user(struct clk_ctrl_t *clk_ctrl, bool verbose) {
  *(clk_ctrl->enable) = CLK_ENABLE_LOAD_USER;
  if (verbose) {
    printf("SPI clock load_user triggered (enable=0x%08X).\n", CLK_ENABLE_LOAD_USER);
  }
}

// Intelligently set the SPI clock to a target frequency given the source clock frequency
int clk_ctrl_set_target_freq(struct clk_ctrl_t *clk_ctrl, struct sys_sts_t *sys_sts, double target_freq_hz, bool verbose) {
  if (target_freq_hz <= 0.0 || target_freq_hz > CLK_MAX_FREQ_HZ) {
    fprintf(stderr, "Invalid target frequency: %.6f Hz. Must be in (0, %.0f] Hz.\n", target_freq_hz, CLK_MAX_FREQ_HZ);
    return -1;
  }

  uint32_t source_clk_freq_hz = sys_sts_get_source_clk_freq_hz(sys_sts, verbose);
  if (source_clk_freq_hz == 0) {
    fprintf(stderr, "Invalid source clock frequency: 0 Hz. Cannot set target frequency.\n");
    return -1;
  }

  // Check target is within the achievable output frequency range.
  // freq_min = VCO_MIN / clk_div_max = VCO_MIN / (2047/8)
  // freq_max = VCO_MAX / clk_div_min = VCO_MAX / (1/8)
  double freq_min = CLK_FB_FREQ_MIN_HZ / (2047.0 / 8.0);
  double freq_max = CLK_FB_FREQ_MAX_HZ / (1.0   / 8.0);
  if (target_freq_hz < freq_min || target_freq_hz > freq_max) {
    fprintf(stderr, "Target %.6f MHz is outside the achievable range (%.6f-%.0f MHz).\n",
            target_freq_hz / 1e6, freq_min / 1e6, freq_max / 1e6);
    return -1;
  }

  double target_ratio = target_freq_hz / (double)source_clk_freq_hz;

  double best_error_hz = target_freq_hz;
  uint32_t best_clk_fb_mult_8 = 0;
  uint8_t  best_clk_fb_div    = 0;
  uint32_t best_clk_div_8     = 0;

  // Brute-force over fb_div. For each fb_div, use continued fractions to find
  // convergents and semiconvergents of (target_ratio * fb_div), then scale by k
  // to satisfy hardware bounds. All k give the same ratio n/d and thus the same
  // frequency, so any valid k is equally good.
  if (verbose) {
    printf("Searching for best clk_fb_mult and clk_div to achieve target frequency %.6f MHz (ratio=%.6f) with source clock %.6f MHz...\n",
           target_freq_hz / 1e6, target_ratio, ((double)source_clk_freq_hz) / 1e6);
  }
  for (uint32_t fb_div = 1; fb_div <= 255; fb_div++) {
    if (verbose && (fb_div / 10 * 10 == fb_div)) {
      printf("  Checking fb_div=%u...\n", fb_div);
    }

    // Search for integer pairs (n, d) with n/d ~= target_ratio_fb.
    // fb_mult_8 = k*n, clk_div_8 = k*d for some positive integer k.
    double target_ratio_fb = target_ratio * (double)fb_div;

    // VCO limits in units of fb_mult_8 (fb_mult_8 = fb_mult * 8)
    uint32_t fb_mult_8_min = (uint32_t)pos_ceil(
        (CLK_FB_FREQ_MIN_HZ * (double)fb_div * 8.0) / (double)source_clk_freq_hz);
    uint32_t fb_mult_8_max = (uint32_t)pos_floor(
        (CLK_FB_FREQ_MAX_HZ * (double)fb_div * 8.0) / (double)source_clk_freq_hz);
    if (fb_mult_8_max > 2047) fb_mult_8_max = 2047;
    if (fb_mult_8_max < fb_mult_8_min) continue;

    // clk_div_8 bounds implied by fb_mult_8 VCO bounds via ratio, capped at hw limit
    uint32_t clk_div_8_min = (uint32_t)pos_ceil((double)fb_mult_8_min / target_ratio_fb);
    uint32_t clk_div_8_max = (uint32_t)pos_floor((double)fb_mult_8_max / target_ratio_fb);
    if (clk_div_8_max > 2047) clk_div_8_max = 2047;

    // Continued fraction recurrence: p[i] = a[i]*p[i-1] + p[i-2], same for q.
    uint32_t p_prev = 0, q_prev = 1;
    uint32_t p_curr = 1, q_curr = 0;

    double remainder = target_ratio_fb;

    for (int i = 0; i < 20; i++) {
      if (remainder < 0.0) break;

      uint32_t a    = (uint32_t)pos_floor(remainder);
      double   frac = remainder - (double)a;

      uint32_t p_next = a * p_curr + p_prev;
      uint32_t q_next = a * q_curr + q_prev;

      // Check convergents and semiconvergents: j*p_curr + p_prev / j*q_curr + q_prev
      // for j in [ceil(a/2), a]. j < ceil(a/2) are worse than the previous convergent.
      // On i==0, p_curr/q_curr is not yet a valid fraction, so start j at a.
      uint32_t j_start = (i == 0) ? a : (a + 1) / 2;

      for (uint32_t j = j_start; j <= a; j++) {
        uint32_t n = j * p_curr + p_prev;
        uint32_t d = j * q_curr + q_prev;

        if (n == 0 || d == 0) continue;

        // Intersect two independent k constraints:
        //   (1) fb_mult_8 in VCO range:  fb_mult_8_min <= k*n <= fb_mult_8_max
        //   (2) clk_div_8 in ratio range: clk_div_8_min <= k*d <= clk_div_8_max
        uint32_t k_min_n = (fb_mult_8_min + n - 1) / n;   // ceil(fb_mult_8_min / n)
        uint32_t k_min_d = (clk_div_8_min + d - 1) / d;   // ceil(clk_div_8_min / d)
        uint32_t k_max_n = fb_mult_8_max / n;
        uint32_t k_max_d = (clk_div_8_max > 0) ? clk_div_8_max / d : 0;

        uint32_t k_min = (k_min_n > k_min_d) ? k_min_n : k_min_d;
        uint32_t k_max = (k_max_n < k_max_d) ? k_max_n : k_max_d;

        if (k_min > k_max) continue;

        // Pick k that centers fb_mult_8 within the VCO range, clamped to valid range.
        uint32_t k_center = (k_min_n + k_max_n) / 2;
        uint32_t k_best = k_center;
        if (k_best < k_min) k_best = k_min;
        if (k_best > k_max) k_best = k_max;

        uint32_t fb_mult_8 = k_best * n;
        uint32_t clk_div_8 = k_best * d;

        double achieved_freq = (double)source_clk_freq_hz
                               * ((double)fb_mult_8 / 8.0)
                               / (double)fb_div
                               / ((double)clk_div_8 / 8.0);
        double error_hz = achieved_freq - target_freq_hz;
        if (error_hz < 0.0) error_hz = -error_hz;

        if (error_hz < best_error_hz && achieved_freq < CLK_MAX_FREQ_HZ) {
          best_error_hz       = error_hz;
          best_clk_fb_mult_8  = fb_mult_8;
          best_clk_fb_div     = (uint8_t)fb_div;
          best_clk_div_8      = clk_div_8;
        }
      }

      p_prev = p_curr; q_prev = q_curr;
      p_curr = p_next; q_curr = q_next;

      if (q_curr > 2047) break;  // no valid k exists for any deeper convergent
      if (frac < 1e-9)  break;  // target is nearly rational, expansion complete
      remainder = 1.0 / frac;
    }
  }

  if (best_clk_fb_div == 0) {
    fprintf(stderr, "No feasible configuration found for target %.6f MHz. "
            "Target may be outside the range achievable given VCO bounds (%.0f-%.0f MHz) and hardware limits.\n",
            target_freq_hz / 1e6, CLK_FB_FREQ_MIN_HZ / 1e6, CLK_FB_FREQ_MAX_HZ / 1e6);
    return -1;
  }

  double best_clk_fb_mult = (double)best_clk_fb_mult_8 / 8.0;
  double best_clk_div     = (double)best_clk_div_8     / 8.0;

  if (verbose) {
    double vco_freq = (double)source_clk_freq_hz * best_clk_fb_mult / (double)best_clk_fb_div;
    double final_freq = vco_freq / best_clk_div;
    printf("Best configuration found:\n");
    printf("  clk_fb_mult: %.3f\n",           best_clk_fb_mult);
    printf("  clk_fb_div:  %u\n",             best_clk_fb_div);
    printf("  clk_div:     %.3f\n",           best_clk_div);
    printf("  Achieved:    %.6f MHz (error: %.3f Hz)\n", final_freq / 1e6, best_error_hz);
    printf("  VCO freq:    %.3f MHz (bounds: %.0f-%.0f MHz)\n",
           vco_freq / 1e6, CLK_FB_FREQ_MIN_HZ / 1e6, CLK_FB_FREQ_MAX_HZ / 1e6);
  }

  clk_ctrl_set_clk_fb(clk_ctrl, sys_sts, best_clk_fb_mult, best_clk_fb_div, verbose);
  clk_ctrl_set_clk_div(clk_ctrl, best_clk_div, verbose);
  clk_ctrl_load_user(clk_ctrl, verbose);

  return 0;
}
