#include <stdio.h>
#include <stdbool.h>

#include "waveform_hw.h"

// Linearity status for channel calibration
typedef enum {
  LINEARITY_LINEAR = 0,     // Good linearity, calibration successful
  LINEARITY_ZERO = 1,       // Slope is constant near zero (likely disconnected)
  LINEARITY_NONLINEAR = 2   // Slope outside acceptable range (likely oscillations)
} linearity_status_t;

// Initialize and validate hardware control structure for a given channel count. Exits on failure
hw_t hw_init(uint32_t channel_count, bool verbose) {
  hw_t hw;

  if (channel_count < 1 || channel_count > HW_MAX_CHANNELS) {
    fprintf(stderr, "Error: channel_count %u is out of supported range (1-%u).\n", channel_count, HW_MAX_CHANNELS);
    exit(1);
  }

  hw.channel_count = channel_count;
  hw.verbose = verbose;

  // Initialize all hardware control structures
  hw.sys_ctrl     = create_sys_ctrl(verbose);
  hw.clk_ctrl     = create_clk_ctrl(verbose);
  hw.sys_sts      = create_sys_sts(verbose);
  hw.dac_ctrl     = create_dac_ctrl(verbose);
  hw.adc_ctrl     = create_adc_ctrl(verbose);
  hw.trigger_ctrl = create_trigger_ctrl(verbose);

  // Check that enough FIFOs are configured in hardware for the requested channel count
  for (uint32_t i = 0; i < ((channel_count - 1) / 8 + 1); i++) {
    if (verbose) {
      printf("Checking FIFO connections for board %u...\n", i);
    }
    if (!FIFO_PRESENT(sys_sts_get_dac_cmd_fifo_status(&hw.sys_sts, i, verbose))) {
      fprintf(stderr, "Error: DAC command FIFO for board %u is not present. Channel count may be too high for hardware configuration.\n", i);
      exit(1);
    }
    if (!FIFO_PRESENT(sys_sts_get_dac_data_fifo_status(&hw.sys_sts, i, verbose))) {
      fprintf(stderr, "Error: DAC data FIFO for board %u is not present. Channel count may be too high for hardware configuration.\n", i);
      exit(1);
    }
    if (!FIFO_PRESENT(sys_sts_get_adc_cmd_fifo_status(&hw.sys_sts, i, verbose))) {
      fprintf(stderr, "Error: ADC command FIFO for board %u is not present. Channel count may be too high for hardware configuration.\n", i);
      exit(1);
    }
    if (!FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(&hw.sys_sts, i, verbose))) {
      fprintf(stderr, "Error: ADC data FIFO for board %u is not present. Channel count may be too high for hardware configuration.\n", i);
      exit(1);
    }
  }

  return hw;
}

// Clear DAC buffers
int hw_clear_dac_buffers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t dac_buf_reset_mask = 0;
  uint32_t board_count = (hw->channel_count - 1) / 8 + 1;
  for (uint32_t board = 0; board < board_count; board++) {
    dac_buf_reset_mask |= (0x1 << 2 * board); // dac buffer reset bit for board
  }
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, dac_buf_reset_mask, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, dac_buf_reset_mask, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process buffer resets
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process buffer resets
  
  // If powered on, send cancel commands to all DAC boards to clear any running commands
  if (hw_running(hw)) {
    if (hw_halted(hw)) {
      fprintf(stderr, "Error: hardware is halted while trying to clear DAC buffers.\n");
      if (!(hw->verbose)) {
        uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
        print_hw_status(hw_status, hw->verbose);
      }
      return -1;
    }
    for (uint32_t board = 0; board < board_count; board++) {
      dac_cmd_cancel(&hw->dac_ctrl, board, hw->verbose);
    }
    HW_SLEEP; // Sleep to allow hardware to process cancel commands
  }

  // Check that all DAC FIFOs are empty
  for (uint32_t board = 0; board < board_count; board++) {
    if (!FIFO_STS_EMPTY(sys_sts_get_dac_cmd_fifo_status(&hw->sys_sts, board, hw->verbose))) {
      fprintf(stderr, "Error: DAC command FIFO for board %u is not empty after buffer reset.\n", board);
      return -1;
    }
    if (!FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(&hw->sys_sts, board, hw->verbose))) {
      fprintf(stderr, "Error: DAC data FIFO for board %u is not empty after buffer reset.\n", board);
      return -1;
    }
  }

  return 0;
}

// Clear ADC buffers
int hw_clear_adc_buffers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t adc_buf_reset_mask = 0;
  uint32_t board_count = (hw->channel_count - 1) / 8 + 1;
  for (uint32_t board = 0; board < board_count; board++) {
    adc_buf_reset_mask |= (0x2 << 2 * board); // adc buffer reset bit for board
  }
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, adc_buf_reset_mask, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, adc_buf_reset_mask, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process buffer resets
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
	HW_SLEEP; // Sleep to allow hardware to process buffer resets

	// If powered on, send cancel commands to all ADC boards to clear any running commands
	if (hw_running(hw)) {
		for (uint32_t board = 0; board < board_count; board++) {
			adc_cmd_cancel(&hw->adc_ctrl, board, hw->verbose);
		}
		HW_SLEEP; // Sleep to allow hardware to process cancel commands
	}

  // Check that all ADC FIFOs are empty
  for (uint32_t board = 0; board < board_count; board++) {
    if (!FIFO_STS_EMPTY(sys_sts_get_adc_cmd_fifo_status(&hw->sys_sts, board, hw->verbose))) {
      fprintf(stderr, "Error: ADC command FIFO for board %u is not empty after buffer reset.\n", board);
      return -1;
    }
    if (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(&hw->sys_sts, board, hw->verbose))) {
      fprintf(stderr, "Error: ADC data FIFO for board %u is not empty after buffer reset.\n", board);
      return -1;
    }
  }

  return 0;
}

// Clear trigger buffers
int hw_clear_trigger_buffers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, 0x10000, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, 0x10000, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process buffer resets
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
	HW_SLEEP; // Sleep to allow hardware to process buffer resets

	// If powered on, send cancel command to trigger controller to clear any running commands
	if (hw_running(hw)) {
		trigger_cmd_cancel(&hw->trigger_ctrl, hw->verbose);
		HW_SLEEP; // Sleep to allow hardware to process cancel command
	}

  // Check that trigger FIFOs are empty
  if (!FIFO_STS_EMPTY(sys_sts_get_trig_cmd_fifo_status(&hw->sys_sts, hw->verbose))) {
    fprintf(stderr, "Error: Trigger command FIFO is not empty after buffer reset.\n");
    return -1;
  }
  if (!FIFO_STS_EMPTY(sys_sts_get_trig_data_fifo_status(&hw->sys_sts, hw->verbose))) {
    fprintf(stderr, "Error: Trigger data FIFO is not empty after buffer reset.\n");
    return -1;
  }
  return 0;
}

// Set the SPI clock frequency to the specified value in MHz. Returns 0 on success, non-zero on failure
int hw_set_spi_clock(hw_t *hw, double clk_mhz) {
  if (hw == NULL) {
    return -1;
  }
  if (clk_mhz <= 0) {
    fprintf(stderr, "Error: clk_mhz must be positive.\n");
    return -1;
  }
  uint32_t clk_hz = (uint32_t)(clk_mhz * 1e6);
  if (hw->verbose) {
    printf("Configuring SPI clock to " PRIu32 "MHz...\n", clk_hz);
  }
  if (clk_ctrl_set_target_freq(&hw->clk_ctrl, &hw->sys_sts, clk_hz, hw->verbose) != 0) {
    fprintf(stderr, "Error: failed to set SPI clock frequency.\n");
    return -1;
  }
  // Sleep to allow hardware to stabilize after clock change
  HW_SLEEP;
  // Verify that clock frequency is within 100 kHz of target
  uint32_t clk_freq = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  if (hw->verbose) {
    printf("SPI clock frequency set to %u Hz\n", clk_freq);
  }
  if ((float) clk_freq < 0.99 * (float) clk_hz || (float) clk_freq > 1.01 * (float) clk_hz) {
    fprintf(stderr, "Error: SPI clock frequency %u Hz is out of 1\% range.\n", clk_freq);
    return -1;
  }
  return 0;
}

// Power on the hardware. Returns 0 on success, non-zero on failure
int hw_power_on(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  if (hw->verbose) {
    printf("Powering on hardware...\n");
  }
  if (hw->verbose) {
    printf("Turning control board on...\n");
  }
  sys_ctrl_turn_ctrl_on(&hw->sys_ctrl, hw->verbose);
  // Wait 200ms for necessary power-on pulse durations
  usleep(200000);
  uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
  if (HW_STS_STATE(hw_status) == S_HALTED) {
    fprintf(stderr, "Error: hardware is in HALTED state after power on. Status: 0x%08X\n", hw_status);
    print_hw_status(hw_status, hw->verbose);
    return -1;
  } else if (HW_STS_STATE(hw_status) != S_WAIT_FOR_POW_EN) {
    fprintf(stderr, "Error: hardware is in unexpected state after power on. Status: 0x%08X\n", hw_status);
    print_hw_status(hw_status, hw->verbose);
    return -1;
  }
  if (hw->verbose) {
    printf("Turning power amp on...\n");
  }
  sys_ctrl_turn_pow_on(&hw->sys_ctrl, hw->verbose);
  // Wait 200ms for necessary power-on pulse durations
  usleep(200000);
  hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
  if (HW_STS_STATE(hw_status) != S_RUNNING) {
    fprintf(stderr, "Error: hardware failed to enter RUNNING state after power on. Status: 0x%08X\n", hw_status);
    print_hw_status(hw_status, hw->verbose);
    return -1;
  }
  if (hw->verbose) {
    printf("Hardware powered on successfully.\n");
  }

  return 0;
}

// Check that the hardware is running
bool hw_running(hw_t *hw) {
  if (hw == NULL) {
    return false;
  }
  uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
	if (hw->verbose) {
		printf("Checking if hardware is running. Status: 0x%08X\n", hw_status);
		print_hw_status(hw_status, hw->verbose);
	}
  return HW_STS_STATE(hw_status) == S_RUNNING;
}

// Check if the hardware is halted
bool hw_halted(hw_t *hw) {
  if (hw == NULL) {
    return false;
  }
  uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
	if (hw->verbose) {
		printf("Checking if hardware is halted. Status: 0x%08X\n", hw_status);
		print_hw_status(hw_status, hw->verbose);
	}
  return HW_STS_STATE(hw_status) == S_HALTED;
}

// Run channel calibration routine
int hw_calibrate(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    fprintf(stderr, "Error: cannot run calibration because hardware is not running.\n");
    return -1;
  }
  uint32_t clk_freq = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  double clk_mhz = (double)clk_freq / 1e6;
  if (clk_freq > 10e6) {
    printf("Skipping calibration because SPI clock frequency is too high (%.3f Hz). Calibration requires <= 10 MHz.\n", clk_mhz);
    return 0;
  }
  hw_clear_dac_buffers(hw);
  hw_clear_adc_buffers(hw);
  hw_reset_triggers(hw);

  // Calibration constants
  const int dac_values[] = {-3000, -1500, 0, 1500, 3000};
  const int num_dac_values = 5;
  const int average_count = 15;
  const double frac_step = 0.9;
  const int calibration_iterations = 4;
  const int delay_ms = 0.3; // 0.3 ms delay
  
  // Overall calibration status
  bool any_calibration_failed = false;

  // Iterate through all channels to calibrate
  for (int ch = 0; ch < (int)hw->channel_count; ch++) {
    int board, channel;
    int16_t current_cal_value = 0;
    board = ch / 8;
    channel = ch % 8;

    printf("Ch %02d : ", ch);
    fflush(stdout);

    if (hw->verbose) {
      printf("\n  Starting calibration for channel %d (board %d, channel %d)\n", ch, board, channel);
    }

    // Perform calibration iterations
    bool calibration_failed = false;
    linearity_status_t cal_sts = LINEARITY_LINEAR;
    int completed_iterations = 0;

    for (int iter = 0; iter < calibration_iterations && !calibration_failed && (cal_sts == LINEARITY_LINEAR); iter++) {
      // Arrays to store DAC values and corresponding averaged ADC readings
      double dac_vals[num_dac_values];
      double avg_adc_vals[num_dac_values];

      // Test each DAC value
      for (int i = 0; i < num_dac_values; i++) {
        int16_t dac_val = dac_values[i];
        dac_vals[i] = (double)dac_val;

        if (hw->verbose) {
          printf("    Testing DAC value %d (%d/%d), averaging %d samples...\n", dac_val, i+1, num_dac_values, average_count);
        }

        // Write DAC value
        dac_cmd_dac_wr_ch(&hw->dac_ctrl, (uint8_t)board, (uint8_t)channel, dac_val, hw->verbose);

        // Perform multiple reads and average them
        double sum_adc = 0.0;

        for (int avg = 0; avg < average_count; avg++) {
          usleep(delay_ms * 1000); // Wait fixed delay

          // Read ADC value
          adc_cmd_adc_rd_ch(&hw->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, hw->verbose);

          // Wait for ADC data to be available
          int adc_tries = 0;
          uint32_t adc_data_fifo_status;
          while (adc_tries < 100) {
            adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(&hw->sys_sts, (uint8_t)board, false);
            if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) > 0) break;
            usleep(100); // 0.1ms
            adc_tries++;
          }

          if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) == 0) {
            if (hw->verbose) {
              printf("      ADC data timeout (no data in FIFO after %d tries)\n", adc_tries);
            }
            calibration_failed = true;
            break;
          }

          uint32_t adc_word = adc_read_word(&hw->adc_ctrl, (uint8_t)board);
          uint16_t adc_reading_raw = (uint16_t)(adc_word & 0xFFFF);
          int16_t adc_reading = adc_reading_raw < 32768 ? adc_reading_raw : adc_reading_raw - 65536; // Convert to signed
          double adc_value = (double)adc_reading;

          if (hw->verbose && avg < 3) {  // Only show first few readings to avoid spam
            printf("      Sample %d: ADC raw=0x%08X, signed=%d, double=%.1f\n",
                   avg+1, adc_word, adc_reading, adc_value);
          }

          sum_adc += adc_value;
        }

        if (calibration_failed) break;

        avg_adc_vals[i] = sum_adc / average_count;

        if (hw->verbose) {
          printf("    DAC=%d -> ADC_avg=%.2f\n", dac_val, avg_adc_vals[i]);
        }
      }

      if (calibration_failed) break;

      // Perform linear regression: y = mx + b
      // Calculate slope (m) and intercept (b)
      double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
      for (int i = 0; i < num_dac_values; i++) {
        sum_x += dac_vals[i];
        sum_y += avg_adc_vals[i];
        sum_xy += dac_vals[i] * avg_adc_vals[i];
        sum_x2 += dac_vals[i] * dac_vals[i];
      }

      // Calculate the variance in y samples (averaged) to detect oscillations even with 0 slope
      double mean_y = sum_y / num_dac_values;
      double variance_y = 0.0;
      for (int i = 0; i < num_dac_values; i++) {
        double diff = avg_adc_vals[i] - mean_y;
        variance_y += diff * diff;
      }
      variance_y /= num_dac_values;

      // Check for division by zero before calculating slope
      double denominator = num_dac_values * sum_x2 - sum_x * sum_x;
      double slope, intercept;
      bool division_by_zero = false;

      if (denominator == 0) {
        division_by_zero = true;
        slope = 0; // Set to 0 to avoid using uninitialized value
        intercept = sum_y / num_dac_values; // Simple average for intercept
      } else {
        slope = (num_dac_values * sum_xy - sum_x * sum_y) / denominator;
        intercept = (sum_y - slope * sum_x) / num_dac_values;
      }

      // Check for division by zero (infinite slope)
      if (division_by_zero) {
        cal_sts = LINEARITY_NONLINEAR;
      }
      // Check if the slope is close to zero AND variance is low (disconnected)
      else if (slope > -0.02 && slope < 0.02 && variance_y < 10000.0) {
        cal_sts = LINEARITY_ZERO;
      }
      // Check if slope is inside acceptable range
      else if (slope > 0.95 && slope < 1.05) {
        cal_sts = LINEARITY_LINEAR;
      }
      // Otherwise, non-linear (likely oscillations)
      else {
        cal_sts = LINEARITY_NONLINEAR;
      }

      // If verbose, print the updates to the calibration value
      if (hw->verbose) {
        printf("  Iteration %d: Current cal=%d, Slope=%.3f, Intercept=%.2f, Variance=%.2f\n",
               iter + 1, current_cal_value, slope, intercept, variance_y);
        fflush(stdout);
      }

      // Update calibration value: subtract frac_step * intercept from current cal value
      current_cal_value = current_cal_value - (int16_t)(frac_step * (intercept >= 0 ? intercept + 0.5 : intercept - 0.5));

      // Print update info if verbose
      if (hw->verbose) {
        printf("    Updated cal value to %d\n", current_cal_value);
        fflush(stdout);
      }

      // Clamp calibration value to valid range
      if (current_cal_value < -4095) {
        if (hw->verbose) {
          printf("    Calibration value clamped to -4095\n");
          fflush(stdout);
        }
        current_cal_value = -4095;
      }
      if (current_cal_value > 4095) {
        if (hw->verbose) {
          printf("    Calibration value clamped to 4095\n");
          fflush(stdout);
        }
        current_cal_value = 4095;
      }

      // Set new calibration value if linearity is still linear
      if (cal_sts == LINEARITY_LINEAR) {
        dac_cmd_set_cal(&hw->dac_ctrl, (uint8_t)board, (uint8_t)channel, current_cal_value, hw->verbose);
      } else if (hw->verbose) {
        printf("    Skipping DAC calibration update due to non-linear or zero slope\n");
        fflush(stdout);
      }

      // Convert offset and slope to amps (range ±HW_MAX_ABS_AMPS for ±32767)
      double offset_amps = intercept / 32767.0 * HW_MAX_ABS_AMPS;

      // If NOT verbose, print this iteration's results with special slope formatting
      if (!hw->verbose) {
        if (division_by_zero) {
          printf("%+.3f A ( inf.) | ", offset_amps);
        } else if (slope < -9.99) {
          printf("%+.3f A (neg.) | ", offset_amps);
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

    // Zero the channel to finalize
    dac_cmd_dac_wr_ch(&hw->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, hw->verbose);
    usleep(1000); // 1ms to let DAC settle

    // Print spaces for skipped iterations to maintain column alignment
    for (int i = completed_iterations; i < calibration_iterations; i++) {
      if (hw->verbose) printf("  -- Skipped iteration number %d", i + 1);
      else printf("----------------- | "); // 18 spaces to match the format above
    }

    // Check hardware status when calibration fails - if system is halted, abort calibration
    if (calibration_failed) {
      printf("Reading hardware status register...\n");
      uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
      if (HW_STS_STATE(hw_status) == S_HALTED) {
        printf("Hardware status shows system is HALTED. Aborting channel calibration.\n");
        print_hw_status(hw_status, hw->verbose);
        return -1;
      }
    }

    // Print final status
    if (hw->verbose) {
      if (calibration_failed) {
        printf(" Calibration FAILED (code bug)");
      } else {
        switch (cal_sts) {
          case LINEARITY_LINEAR:
            printf(" Calibration OK");
            break;
          case LINEARITY_ZERO:
            printf(" Poor linearity (check connections)");
            break;
          case LINEARITY_NONLINEAR:
            printf(" Nonlinear (possible oscillations)");
            break;
          default:
            printf(" Unknown calibration status");
            break;
        }
      }
    }
    else {
      if (calibration_failed) {
        any_calibration_failed = true;
        printf("-F- |");
      } else {
        switch (cal_sts) {
          case LINEARITY_LINEAR:
            printf("--- |");
            break;
          case LINEARITY_ZERO:
            printf("-X- |");
            break;
          case LINEARITY_NONLINEAR:
            printf("~E~ |");
            break;
          default:
            printf("??? |");
            break;
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

// Set trigger lockout in ms
int hw_set_trigger_lockout(hw_t *hw, double trigger_lockout_ms) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    fprintf(stderr, "Error: cannot set trigger lockout because hardware is not running.\n");
    return -1;
  }
  hw_reset_triggers(hw);
  uint32_t spi_clk_freq_hz = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  // Convert lockout time from ms to number of SPI clock cycles
  double lockout_cycles_double = (trigger_lockout_ms / 1000.0) * (double)spi_clk_freq_hz;
  if (lockout_cycles_double > 268435455.0) {
    double max_lockout_ms = (268435455.0 / (double)spi_clk_freq_hz) * 1000.0;
    fprintf(stderr, "Error: trigger_lockout_ms value %.3f is too high for the current SPI clock frequency of %u Hz. Maximum trigger_lockout_ms is %.3f ms.\n",
            trigger_lockout_ms, spi_clk_freq_hz, max_lockout_ms);
    return -1;
  }
  uint32_t lockout_cycles = (uint32_t)(lockout_cycles_double + 0.5); // Round to nearest integer
  trigger_cmd_set_lockout(&hw->trigger_ctrl, lockout_cycles, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process command
  // Check that the trigger buffer is empty
  uint32_t trig_cmd_fifo_sts = sys_sts_get_trig_cmd_fifo_status(&hw->sys_sts, hw->verbose);
  if (!FIFO_STS_EMPTY(trig_cmd_fifo_sts)) {
    fprintf(stderr, "Error: trigger command FIFO is not empty after setting trigger lockout. Status: 0x%08X\n", trig_cmd_fifo_sts);
    return -1;
  }
  return 0;
}

// Validate the minimum file delay against minimum DAC delay.
bool hw_dac_timing_valid(hw_t *hw, double min_dt) {
  if (hw == NULL) {
    return false;
  }
  uint32_t spi_clk_freq_hz = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  // Round down to nearest integer number of SPI clock cycles for minimum DAC delay
  uint32_t min_dac_delay_cycles = (uint32_t)((min_dt / 1000.0) * (double)spi_clk_freq_hz);
  // Get the minimum DAC delay from hardware (in SPI clock cycles)
  uint32_t min_dac_delay_hw = sys_sts_get_dac_min_delay_time(&hw->sys_sts, hw->verbose);
  if (min_dac_delay_cycles < min_dac_delay_hw) {
    if (hw->verbose) {
      fprintf(stderr, "Error: minimum DAC delay %.3f ms (%.0f SPI clock cycles) is less than hardware minimum DAC delay %.3f ms (%u SPI clock cycles).\n",
             min_dt, (double)min_dac_delay_cycles, (double)min_dac_delay_hw / (double)spi_clk_freq_hz * 1000.0, min_dac_delay_hw);
      return false;
    }
  }
}

// Validate the minimum file delay against minimum ADC delay.
bool hw_adc_timing_valid(hw_t *hw, double min_dt) {
  if (hw == NULL) {
    return false;
  }
  uint32_t spi_clk_freq_hz = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  // Round down to nearest integer number of SPI clock cycles for minimum ADC delay
  uint32_t min_adc_delay_cycles = (uint32_t)((min_dt / 1000.0) * (double)spi_clk_freq_hz);
  // Get the minimum ADC delay from hardware (in SPI clock cycles)
  uint32_t min_adc_delay_hw = sys_sts_get_adc_min_delay_time(&hw->sys_sts, hw->verbose);
  if (min_adc_delay_cycles < min_adc_delay_hw) {
    if (hw->verbose) {
      fprintf(stderr, "Error: minimum ADC delay %.3f ms (%.0f SPI clock cycles) is less than hardware minimum ADC delay %.3f ms (%u SPI clock cycles).\n",
             min_dt, (double)min_adc_delay_cycles, (double)min_adc_delay_hw / (double)spi_clk_freq_hz * 1000.0, min_adc_delay_hw);
      return false;
    }
  }
}

// Start the expected count of triggers.
int hw_start_triggers(hw_t *hw, uint32_t expected_triggers) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    fprintf(stderr, "Error: cannot start triggers because hardware is not running.\n");
    return -1;
  }
  // Clear, not reset, to keep trigger count
  hw_clear_trigger_buffers(hw);
  // Expect the specified number of triggers
  trigger_cmd_expect_ext(&hw->trigger_ctrl, expected_triggers, false, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process command
  // Check that the trigger buffer is empty
  uint32_t trig_cmd_fifo_sts = sys_sts_get_trig_cmd_fifo_status(&hw->sys_sts, hw->verbose);
  if (!FIFO_STS_EMPTY(trig_cmd_fifo_sts)) {
    fprintf(stderr, "Error: trigger command FIFO is not empty after starting triggers. Status: 0x%08X\n", trig_cmd_fifo_sts);
    return -1;
  }
  return 0;
}

// Get trigger count
uint32_t hw_get_trigger_count(hw_t *hw) {
  if (hw == NULL) {
    return 0;
  }
  return sys_sts_get_trig_count(&hw->sys_sts, hw->verbose);
}

// Reset trigger counter
int hw_reset_triggers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    return 0;
  }
  hw_clear_trigger_buffers(hw);
  trigger_cmd_reset_count(&hw->trigger_ctrl, hw->verbose);
  HW_SLEEP; // Sleep to allow hardware to process reset command
  // Check that trigger counter is reset to 0 and the trigger buffer is empty
  uint32_t trig_count = sys_sts_get_trig_count(&hw->sys_sts, hw->verbose);
  uint32_t trig_cmd_fifo_sts = sys_sts_get_trig_cmd_fifo_status(&hw->sys_sts, hw->verbose);
  if (trig_count != 0) {
    fprintf(stderr, "Error: trigger counter is %u after reset, expected 0.\n", trig_count);
    return -1;
  }
  if (!FIFO_STS_EMPTY(trig_cmd_fifo_sts)) {
    fprintf(stderr, "Error: trigger command FIFO is not empty after trigger reset. Status: 0x%08X\n", trig_cmd_fifo_sts);
    return -1;
  }
  return 0;
}

// Power off hardware
void hw_power_off(hw_t *hw) {
  if (hw == NULL) {
    return;
  }
  if (hw->verbose) {
    printf("Powering off hardware...\n");
  }
  sys_ctrl_turn_off(&hw->sys_ctrl, hw->verbose);
  if (hw->verbose) {
    printf("Hardware powered off.\n");
  }
}
