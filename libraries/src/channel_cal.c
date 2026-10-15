#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "channel_cal.h"
#include "hw_config.h"

// Fraction of the fitted offset folded back into the cal value each correction pass. Below 1
// so the correction converges rather than ringing.
#define CAL_FRAC_STEP 0.9

// Upper bound on ramp points, so the per-channel work arrays can be fixed-size.
#define CAL_MAX_SAMPLES_PER_RAMP 64

// PIO data-readback poll: retry this many times, sleeping this long between tries.
#define CAL_PIO_POLL_TRIES 100
#define CAL_PIO_POLL_US    100

// Linearity verdict for a channel's fitted ramp.
typedef enum {
  LINEARITY_LINEAR = 0,     // Good linearity, calibration successful
  LINEARITY_ZERO = 1,       // Slope constant near zero (likely disconnected)
  LINEARITY_NONLINEAR = 2   // Slope outside acceptable range (likely oscillations)
} linearity_status_t;

// ---- PIO transport primitives. The DMA split is scaffolded behind cal_transport_t in the
// public API but not implemented yet; its primitives will slot in alongside these. ----

// Write a DAC code to (board, channel). Returns 0 on success, negative on error.
static int xport_dac_write(const channel_cal_ctx_t *ctx, int board, int channel, int16_t val, bool verbose) {
  dac_cmd_dac_wr_ch(ctx->dac, (uint8_t)board, (uint8_t)channel, val, verbose);
  return 0;
}

// Set the calibration value for (board, channel). Returns 0 on success, negative on error.
static int xport_set_cal(const channel_cal_ctx_t *ctx, int board, int channel, int16_t cal, bool verbose) {
  dac_cmd_set_cal(ctx->dac, (uint8_t)board, (uint8_t)channel, cal, verbose);
  return 0;
}

// Issue one single-channel ADC read on (board, channel) and return the signed sample in *out.
// Returns 0 on success, negative on timeout.
static int xport_adc_read(const channel_cal_ctx_t *ctx, int board, int channel, int16_t *out, bool verbose) {
  adc_cmd_adc_rd_ch(ctx->adc, (uint8_t)board, (uint8_t)channel, 0, verbose);
  for (int tries = 0; tries < CAL_PIO_POLL_TRIES; tries++) {
    if (FIFO_STS_WORD_COUNT(sys_sts_get_adc_data_fifo_status(ctx->sts, (uint8_t)board, false)) > 0) {
      *out = (int16_t)(adc_read_word(ctx->adc, (uint8_t)board) & 0xFFFF);
      return 0;
    }
    usleep(CAL_PIO_POLL_US);
  }
  return -1; // timeout -- no data in FIFO
}

// ---- public API ----

channel_cal_params_t channel_cal_default_params(void) {
  channel_cal_params_t p = {
    .samples_per_ramp    = CHANNEL_CAL_DEFAULT_SAMPLES,
    .abs_scale_per_ramp  = CHANNEL_CAL_DEFAULT_SCALE,
    .averages_per_sample = CHANNEL_CAL_DEFAULT_AVERAGES,
    .correction_iters    = CHANNEL_CAL_DEFAULT_ITERS,
  };
  return p;
}

int channel_cal_run_default(const channel_cal_ctx_t *ctx, cal_transport_t xport,
                            int ch_start, int ch_count, bool verbose) {
  return channel_cal_run(ctx, xport, ch_start, ch_count, channel_cal_default_params(), verbose);
}

int channel_cal_run(const channel_cal_ctx_t *ctx, cal_transport_t xport,
                    int ch_start, int ch_count,
                    channel_cal_params_t params, bool verbose) {
  if (ctx == NULL || ctx->dac == NULL || ctx->adc == NULL || ctx->sts == NULL) {
    fprintf(stderr, "Error: channel_cal_run called with an incomplete context.\n");
    return -1;
  }
  if (xport == CAL_XPORT_DMA) {
    fprintf(stderr, "Error: DMA calibration transport is not implemented yet (use PIO).\n");
    return -1;
  }
  const int n_points = params.samples_per_ramp;
  if (n_points < 2 || n_points > CAL_MAX_SAMPLES_PER_RAMP) {
    fprintf(stderr, "Error: samples_per_ramp %d out of range (2-%d).\n", n_points, CAL_MAX_SAMPLES_PER_RAMP);
    return -1;
  }
  if (params.averages_per_sample < 1 || params.correction_iters < 1) {
    fprintf(stderr, "Error: averages_per_sample and correction_iters must be >= 1.\n");
    return -1;
  }
  if (params.abs_scale_per_ramp <= 0.0) {
    fprintf(stderr, "Error: abs_scale_per_ramp must be positive.\n");
    return -1;
  }
  if (HW_STS_STATE(sys_sts_get_hw_status(ctx->sts, verbose)) != S_RUNNING) {
    fprintf(stderr, "Error: cannot run calibration because hardware is not running.\n");
    return -1;
  }

  // DAC ramp points: n evenly spaced codes spanning [-scale, +scale].
  int16_t dac_points[CAL_MAX_SAMPLES_PER_RAMP];
  for (int i = 0; i < n_points; i++) {
    double v = -params.abs_scale_per_ramp + (2.0 * params.abs_scale_per_ramp) * i / (n_points - 1);
    dac_points[i] = (int16_t)(v >= 0 ? v + 0.5 : v - 0.5);
  }

  bool any_calibration_failed = false;

  for (int ch = ch_start; ch < ch_start + ch_count; ch++) {
    int board = ch / 8;
    int channel = ch % 8;
    int16_t current_cal_value = 0;

    printf("Ch %02d : ", ch);
    fflush(stdout);

    if (verbose) {
      printf("\n  Starting calibration for channel %d (board %d, channel %d)\n", ch, board, channel);
    }

    bool calibration_failed = false;
    linearity_status_t cal_sts = LINEARITY_LINEAR;
    int completed_iterations = 0;

    for (int iter = 0; iter < params.correction_iters && !calibration_failed && (cal_sts == LINEARITY_LINEAR); iter++) {
      double dac_vals[CAL_MAX_SAMPLES_PER_RAMP];
      double avg_adc_vals[CAL_MAX_SAMPLES_PER_RAMP];

      // Sweep the ramp, averaging the ADC reading at each point.
      for (int i = 0; i < n_points; i++) {
        int16_t dac_val = dac_points[i];
        dac_vals[i] = (double)dac_val;

        if (verbose) {
          printf("    Testing DAC value %d (%d/%d), averaging %d samples...\n",
                 dac_val, i + 1, n_points, params.averages_per_sample);
        }

        if (xport_dac_write(ctx, board, channel, dac_val, verbose) != 0) {
          calibration_failed = true;
          break;
        }
        // Let the DAC output and coil current settle before the first ADC read.
        usleep(CAL_SETTLE_DELAY);

        double sum_adc = 0.0;
        for (int avg = 0; avg < params.averages_per_sample; avg++) {
          int16_t adc_reading;
          if (xport_adc_read(ctx, board, channel, &adc_reading, verbose) != 0) {
            if (verbose) printf("      ADC read failed (no data)\n");
            calibration_failed = true;
            break;
          }
          if (verbose && avg < 3) {
            printf("      Sample %d: ADC signed=%d\n", avg + 1, adc_reading);
          }
          sum_adc += (double)adc_reading;
        }
        if (calibration_failed) break;

        avg_adc_vals[i] = sum_adc / params.averages_per_sample;
        if (verbose) {
          printf("    DAC=%d -> ADC_avg=%.2f\n", dac_val, avg_adc_vals[i]);
        }
      }
      if (calibration_failed) break;

      // Linear regression y = mx + b over the ramp.
      double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
      for (int i = 0; i < n_points; i++) {
        sum_x += dac_vals[i];
        sum_y += avg_adc_vals[i];
        sum_xy += dac_vals[i] * avg_adc_vals[i];
        sum_x2 += dac_vals[i] * dac_vals[i];
      }

      // Variance of the averaged readings, to tell a disconnected channel (flat) from an
      // oscillating one (flat slope but noisy) when the slope is near zero.
      double mean_y = sum_y / n_points;
      double variance_y = 0.0;
      for (int i = 0; i < n_points; i++) {
        double diff = avg_adc_vals[i] - mean_y;
        variance_y += diff * diff;
      }
      variance_y /= n_points;

      double denominator = n_points * sum_x2 - sum_x * sum_x;
      double slope, intercept;
      bool division_by_zero = false;
      if (denominator == 0) {
        division_by_zero = true;
        slope = 0;
        intercept = sum_y / n_points;
      } else {
        slope = (n_points * sum_xy - sum_x * sum_y) / denominator;
        intercept = (sum_y - slope * sum_x) / n_points;
      }

      if (division_by_zero) {
        cal_sts = LINEARITY_NONLINEAR;
      } else if (slope > -0.02 && slope < 0.02 && variance_y < 10000.0) {
        cal_sts = LINEARITY_ZERO;
      } else if (slope > 0.95 && slope < 1.05) {
        cal_sts = LINEARITY_LINEAR;
      } else {
        cal_sts = LINEARITY_NONLINEAR;
      }

      if (verbose) {
        printf("  Iteration %d: Current cal=%d, Slope=%.3f, Intercept=%.2f, Variance=%.2f\n",
               iter + 1, current_cal_value, slope, intercept, variance_y);
        fflush(stdout);
      }

      // Fold a fraction of the fitted offset back into the cal value.
      current_cal_value = current_cal_value - (int16_t)(CAL_FRAC_STEP * (intercept >= 0 ? intercept + 0.5 : intercept - 0.5));
      if (verbose) {
        printf("    Updated cal value to %d\n", current_cal_value);
        fflush(stdout);
      }
      if (current_cal_value < -4095) {
        if (verbose) printf("    Calibration value clamped to -4095\n");
        current_cal_value = -4095;
      }
      if (current_cal_value > 4095) {
        if (verbose) printf("    Calibration value clamped to 4095\n");
        current_cal_value = 4095;
      }

      if (cal_sts == LINEARITY_LINEAR) {
        if (xport_set_cal(ctx, board, channel, current_cal_value, verbose) != 0) {
          calibration_failed = true;
          break;
        }
      } else if (verbose) {
        printf("    Skipping DAC calibration update due to non-linear or zero slope\n");
        fflush(stdout);
      }

      double offset_amps = intercept / 32767.0 * HW_MAX_ABS_AMPS;
      if (!verbose) {
        if (division_by_zero) {
          printf("%+.3f A ( inf.) | ", offset_amps);
        } else if (slope < -9.99) {
          printf("%+.3f A ( neg.) | ", offset_amps);
        } else if (slope < 0) {
          printf("%+.3f A (%.2f) | ", offset_amps, slope);
        } else if (slope > 9.99) {
          printf("%+.3f A (10.0+) | ", offset_amps);
        } else {
          printf("%+.3f A (%.3f) | ", offset_amps, slope);
        }
      }

      completed_iterations++;
      fflush(stdout);
    }

    // Zero the channel to finalize, then let the DAC settle.
    xport_dac_write(ctx, board, channel, 0, verbose);
    usleep(1000);

    // Pad skipped iterations so the per-channel columns stay aligned.
    for (int i = completed_iterations; i < params.correction_iters; i++) {
      if (verbose) printf("  -- Skipped iteration number %d", i + 1);
      else printf("---------------- | "); // dashes match the "+x.xxx A (x.xxx)" field width
    }

    // A read failure can mean the hardware halted mid-run; if so, stop here.
    if (calibration_failed) {
      uint32_t hw_status = sys_sts_get_hw_status(ctx->sts, verbose);
      if (HW_STS_STATE(hw_status) == S_HALTED) {
        printf("\nHardware status shows system is HALTED. Aborting channel calibration.\n");
        print_hw_status(hw_status, verbose);
        return -1;
      }
    }

    if (verbose) {
      if (calibration_failed) {
        printf(" Calibration FAILED (code bug)");
      } else {
        switch (cal_sts) {
          case LINEARITY_LINEAR:    printf(" Calibration OK"); break;
          case LINEARITY_ZERO:      printf(" Poor linearity (check connections)"); break;
          case LINEARITY_NONLINEAR: printf(" Nonlinear (possible oscillations)"); break;
          default:                  printf(" Unknown calibration status"); break;
        }
      }
    } else {
      if (calibration_failed) {
        any_calibration_failed = true;
        printf("-F- |");
      } else {
        switch (cal_sts) {
          case LINEARITY_LINEAR:    printf("--- |"); break;
          case LINEARITY_ZERO:      printf("-X- |"); break;
          case LINEARITY_NONLINEAR: printf("~E~ |"); break;
          default:                  printf("??? |"); break;
        }
      }
    }
    printf("\n");
  }

  if (any_calibration_failed) {
    fprintf(stderr, "Error: One or more channels failed calibration.\n");
    return -1;
  }
  return 0;
}
