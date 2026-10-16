#include <stdio.h>
#include <stdlib.h>

#include "waveform_hw.h"

// Initialize and validate hardware control structure for a given channel count. Exits on failure
hw_t hw_init(uint32_t channel_count, double clk_MHz, bool verbose) {
  hw_t hw;

  if (channel_count < 1 || channel_count > HW_MAX_CHANNELS) {
    fprintf(stderr, "Error: channel_count %u is out of supported range (1-%u).\n", channel_count, HW_MAX_CHANNELS);
    exit(1);
  }

  hw.channel_count = channel_count;
  hw.board_count = (channel_count - 1) / 8 + 1;
  hw.verbose = verbose;

  // Initialize all hardware control structures
  hw.sys_ctrl     = create_sys_ctrl(verbose);
  hw.clk_ctrl     = create_clk_ctrl(verbose);
  hw.sys_sts      = create_sys_sts(verbose);
  hw.dac_ctrl     = create_dac_ctrl(verbose);
  hw.adc_ctrl     = create_adc_ctrl(verbose);
  hw.trigger_ctrl = create_trigger_ctrl(verbose);

  // Check that enough FIFOs are configured in hardware for the requested channel count
  for (uint32_t i = 0; i < hw.board_count; i++) {
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

  // Clear all buffers (DAC, ADC, and trigger)
  if (hw_clear_dac_buffers(&hw) != 0) {
    fprintf(stderr, "Error: failed to clear DAC buffers.\n");
    exit(1);
  }
  if (hw_clear_adc_buffers(&hw) != 0) {
    fprintf(stderr, "Error: failed to clear ADC buffers.\n");
    exit(1);
  }
  if (hw_clear_trigger_buffers(&hw) != 0) {
    fprintf(stderr, "Error: failed to clear trigger buffers.\n");
    exit(1);
  }

  // Set the clock frequency for the hardware
  if (hw_set_spi_clock(&hw, clk_MHz) != 0) {
    fprintf(stderr, "Error: failed to set clock frequency to %.3f MHz.\n", clk_MHz);
    exit(1);
  }

  return hw;
}

// Clear DAC buffers
int hw_clear_dac_buffers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t dac_buf_reset_mask = 0;
  uint32_t board_count = hw->board_count;
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
  uint32_t board_count = hw->board_count;
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

// Pulse buf_reset on the DMA-driven FIFOs (dac_cmd + adc_data) across active boards. The DAC
// command FIFO reset uses the even per-board bit, the ADC data FIFO reset the odd one (the
// same bit assignment hw_clear_dac_buffers / hw_clear_adc_buffers use). This clears data left
// in the DAC FIFO when a run is halted mid-sequence and the latched bridge faults (mode_viol,
// over/underflow) that live in the FIFO reset domain. Assert only after the MCDMA is halted.
int hw_reset_dma_buffers(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t cmd_mask = 0, data_mask = 0;
  for (uint32_t board = 0; board < hw->board_count; board++) {
    cmd_mask  |= (0x1u << (2 * board));  // dac_cmd FIFO (the MM2S sink)
    data_mask |= (0x2u << (2 * board));  // adc_data FIFO (the S2MM source)
  }
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, cmd_mask, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, data_mask, hw->verbose);
  HW_SLEEP;
  sys_ctrl_set_cmd_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  sys_ctrl_set_data_buf_reset(&hw->sys_ctrl, 0, hw->verbose);
  HW_SLEEP;
  return 0;
}

// Set the SPI clock frequency to the specified value in MHz. Returns 0 on success, non-zero on failure
int hw_set_spi_clock(hw_t *hw, double clk_MHz) {
  if (hw == NULL) {
    return -1;
  }
  if (clk_MHz <= 0) {
    fprintf(stderr, "Error: clk_MHz must be positive.\n");
    return -1;
  }
  uint32_t clk_hz = (uint32_t)(clk_MHz * 1e6);
  if (hw->verbose) {
    printf("Configuring SPI clock to %" PRIu32 "MHz...\n", clk_hz);
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
    fprintf(stderr, "Error: SPI clock frequency %u Hz is out of 1%% range.\n", clk_freq);
    return -1;
  }
  hw->spi_clk_hz = clk_freq;
  // Cache the hardware minimum command delays (in SPI clock cycles). These are
  // hardware constants exposed in the status register and available without
  // powering on; they are used while streaming to split oversized delays into
  // no-op delay commands without leaving a residual that is "too short".
  hw->dac_min_delay = sys_sts_get_dac_min_delay_time(&hw->sys_sts, hw->verbose);
  hw->adc_min_delay = sys_sts_get_adc_min_delay_time(&hw->sys_sts, hw->verbose);
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

// Build a channel_cal context from the hw_t handles (PIO transport; no DMA mover attached).
channel_cal_ctx_t hw_channel_cal_ctx(hw_t *hw) {
  channel_cal_ctx_t ctx = {
    .dac = &hw->dac_ctrl,
    .adc = &hw->adc_ctrl,
    .sts = &hw->sys_sts,
    .dma = NULL,
  };
  return ctx;
}

// Run channel calibration over the PIO FIFO bridge, or over the MCDMA when the boards are in DMA
// mode. Calibration needs clean ADC reads, so it is skipped above 10 MHz.
int hw_calibrate(hw_t *hw, bool use_dma) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    fprintf(stderr, "Error: cannot run calibration because hardware is not running.\n");
    return -1;
  }
  uint32_t clk_freq = sys_sts_get_clk_freq_hz(&hw->sys_sts, hw->verbose);
  // 1% margin so a requested 10 MHz that measures slightly high still calibrates
  if (clk_freq > 10.1e6) {
    fprintf(stderr, "Warning: skipping channel calibration -- SPI clock is %.3f MHz, and calibration needs the clean ADC reads only available at <= 10 MHz.\n", (double)clk_freq / 1e6);
    return 0;
  }
  channel_cal_ctx_t ctx = hw_channel_cal_ctx(hw);
  if (!use_dma) {
    hw_clear_dac_buffers(hw);
    hw_clear_adc_buffers(hw);
    hw_reset_triggers(hw);
    return channel_cal_run_default(&ctx, CAL_XPORT_PIO, 0, (int)hw->channel_count, hw->verbose);
  }

  struct dma_ctrl_t dma = create_dma_ctrl(hw->verbose);
  if (!dma.ok) {
    fprintf(stderr, "Error: MCDMA/u-dma-buf unavailable for DMA calibration.\n");
    return -1;
  }
  // hw_clear_dac_buffers would send a PIO cancel into the DMA-owned DAC FIFO (a mode_viol), so
  // reset the DMA-driven FIFOs by buf_reset instead; the ADC command lane stays PIO.
  hw_reset_dma_buffers(hw);
  hw_clear_adc_buffers(hw);
  hw_reset_triggers(hw);
  ctx.dma = &dma;
  int rc = channel_cal_run_default(&ctx, CAL_XPORT_DMA, 0, (int)hw->channel_count, hw->verbose);
  // Leave the MCDMA idle and the FIFOs clean for the run that follows.
  dma_wave_halt(&dma, hw->verbose);
  hw_reset_dma_buffers(hw);
  destroy_dma_ctrl(&dma);
  return rc;
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
bool hw_dac_timing_valid(hw_t *hw, uint32_t min_delay_cycles) {
  if (hw == NULL) {
    return false;
  }
  // Read the hardware minimum fresh: it is frequency-dependent and recomputed at power-on.
  uint32_t min_dac_delay_hw = sys_sts_get_dac_min_delay_time(&hw->sys_sts, hw->verbose);
  if (min_delay_cycles < min_dac_delay_hw) {
    fprintf(stderr,
      "Error: DAC timing invalid -- smallest delay %u SPI cycles is below the hardware minimum %u cycles at %u Hz.\n",
      min_delay_cycles, min_dac_delay_hw, hw->spi_clk_hz);
    return false;
  }
  return true;
}

// Validate the minimum file delay against minimum ADC delay.
bool hw_adc_timing_valid(hw_t *hw, uint32_t min_delay_cycles) {
  if (hw == NULL) {
    return false;
  }
  // Read the hardware minimum fresh: it is frequency-dependent and recomputed at power-on.
  uint32_t min_adc_delay_hw = sys_sts_get_adc_min_delay_time(&hw->sys_sts, hw->verbose);
  if (min_delay_cycles < min_adc_delay_hw) {
    fprintf(stderr,
      "Error: ADC timing invalid -- smallest delay %u SPI cycles is below the hardware minimum %u cycles at %u Hz.\n",
      min_delay_cycles, min_adc_delay_hw, hw->spi_clk_hz);
    return false;
  }
  return true;
}

// Convert a timestamp in seconds to SPI clock cycles, rounding to nearest.
uint32_t hw_time_to_cycles(double t_sec, uint32_t spi_clk_hz) {
  if (t_sec <= 0.0) {
    return 0;
  }
  return (uint32_t)(t_sec * (double)spi_clk_hz + 0.5);
}

// Start the expected count of triggers.
int hw_start_triggers(hw_t *hw, uint32_t expected_triggers) {
  if (hw == NULL) {
    return -1;
  }
  if (!hw_running(hw)) {
    uint32_t hw_status = sys_sts_get_hw_status(&hw->sys_sts, hw->verbose);
    fprintf(stderr, "Error: cannot start triggers because hardware is not running. Status: 0x%08X\n", hw_status);
    print_hw_status(hw_status, true);
    return -1;
  }
  // Clear, not reset, to keep trigger count
  hw_clear_trigger_buffers(hw);
  // Expect the specified number of triggers, logging each trigger's timepoint to
  // the trigger data FIFO so the trigger data stream has samples to read.
  trigger_cmd_expect_ext(&hw->trigger_ctrl, expected_triggers, true, hw->verbose);
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

// Power off hardware. waveform is one-shot, so on exit a halted system is left halted -- its
// fault diagnostics stay visible to `status`, and the next run's startup off clears it. A
// running system is still turned off so the tool never exits while driving.
void hw_power_off(hw_t *hw) {
  if (hw == NULL) {
    return;
  }
  if (hw_halted(hw)) {
    printf("Hardware is halted; leaving it halted so the fault stays visible to `status`.\n");
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

// Send a DAC noop command for a single trigger wait to all active boards (assumes not last)
int hw_dac_noop_trig(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint32_t board = 0; board < board_count; board++) {
    dac_cmd_noop(&hw->dac_ctrl, board, DAC_TRIGGER_WAIT, DAC_CONTINUE, DAC_NO_LDAC, 1, hw->verbose);
  }
  return 0;
}

// Send a DAC noop command for a delay to all active boards (assumes not last)
int hw_dac_noop_delay(hw_t *hw, uint32_t delay_clks) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint32_t board = 0; board < board_count; board++) {
    dac_cmd_noop(&hw->dac_ctrl, board, DAC_DELAY_WAIT, DAC_CONTINUE, DAC_NO_LDAC, delay_clks, hw->verbose);
  }
  return 0;
}

// Get the available DAC sample command space (minimum across all active boards)
// Each DAC sample command occupies 5 command FIFO words (1 command word + 4 data words)
int hw_get_dac_sample_cmd_space(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  uint32_t min_free = DAC_CMD_FIFO_WORDCOUNT;
  for (uint32_t board = 0; board < board_count; board++) {
    uint32_t used = FIFO_STS_WORD_COUNT(sys_sts_get_dac_cmd_fifo_status(&hw->sys_sts, board, hw->verbose));
    uint32_t free_words = (used >= DAC_CMD_FIFO_WORDCOUNT) ? 0 : (DAC_CMD_FIFO_WORDCOUNT - used);
    if (free_words < min_free) {
      min_free = free_words;
    }
  }
  return (int)(min_free / 5);
}

// Get the available ADC command space (minimum across all active boards)
int hw_get_adc_cmd_space(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  uint32_t min_free = ADC_CMD_FIFO_WORDCOUNT;
  for (uint32_t board = 0; board < board_count; board++) {
    uint32_t used = FIFO_STS_WORD_COUNT(sys_sts_get_adc_cmd_fifo_status(&hw->sys_sts, board, hw->verbose));
    uint32_t free_words = (used >= ADC_CMD_FIFO_WORDCOUNT) ? 0 : (ADC_CMD_FIFO_WORDCOUNT - used);
    if (free_words < min_free) {
      min_free = free_words;
    }
  }
  return (int)min_free;
}

// Get the available ADC sample count (minimum across all active boards)
// Each ADC sample occupies 4 data FIFO words (8 channels as 4 pairs)
int hw_get_adc_sample_count(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  uint32_t min_words = FIFO_STS_WORD_COUNT(sys_sts_get_adc_data_fifo_status(&hw->sys_sts, 0, hw->verbose));
  for (uint32_t board = 1; board < board_count; board++) {
    uint32_t words = FIFO_STS_WORD_COUNT(sys_sts_get_adc_data_fifo_status(&hw->sys_sts, board, hw->verbose));
    if (words < min_words) {
      min_words = words;
    }
  }
  return (int)(min_words / 4);
}

// Get the available trigger sample words (only one buffer)
// Each trigger sample is 2 words, so it's just the words divided by 2 (rounded down)
// This indicates how many individual trigger samples can be read
int hw_get_trigger_sample_count(hw_t *hw) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t words = FIFO_STS_WORD_COUNT(sys_sts_get_trig_data_fifo_status(&hw->sys_sts, hw->verbose));
  return (int)(words / 2);
}

// Send a DAC command with a delay in clock cycles to all active boards
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
// Indicate whether this is the last DAC command in a sequence to control the continue flag
int hw_set_dacs_delay(hw_t *hw, const double *amps, uint32_t delay_clks, bool last) {
  if (hw == NULL || amps == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    int16_t dac_values[8] = {0}; // Default to 0 for unused channels
    for (uint8_t ch_in_board = 0; ch_in_board < 8; ch_in_board++) {
      uint32_t ch = board * 8 + ch_in_board;
      if (ch < hw->channel_count) {
        double value_amps = amps[ch];
        if (value_amps < -HW_MAX_ABS_AMPS || value_amps > HW_MAX_ABS_AMPS) {
          fprintf(stderr, "Error: value_amps %.3f for channel %u is out of range (valid range is -%.1f to %.1f amps).\n",
                  value_amps, ch, HW_MAX_ABS_AMPS, HW_MAX_ABS_AMPS);
          return -1;
        }
        dac_values[ch_in_board] = (int16_t)((value_amps / HW_MAX_ABS_AMPS) * 32767.0);
      }
    }
    // Write all channels, then delay for delay_clks cycles (LDAC latches all channels together)
    dac_cmd_dac_wr(&hw->dac_ctrl, board, dac_values, DAC_DELAY_WAIT, !last, DAC_LDAC, delay_clks, hw->verbose);
  }
  return 0;
}

// Send a DAC command with a single trigger wait to all active boards
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
// Indicate whether this is the last DAC command in a sequence to control the continue flag
int hw_set_dacs_trig(hw_t *hw, const double *amps, bool last) {
  if (hw == NULL || amps == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    int16_t dac_values[8] = {0}; // Default to 0 for unused channels
    for (uint8_t ch_in_board = 0; ch_in_board < 8; ch_in_board++) {
      uint32_t ch = board * 8 + ch_in_board;
      if (ch < hw->channel_count) {
        double value_amps = amps[ch];
        if (value_amps < -HW_MAX_ABS_AMPS || value_amps > HW_MAX_ABS_AMPS) {
          fprintf(stderr, "Error: value_amps %.3f for channel %u is out of range (valid range is -%.1f to %.1f amps).\n",
                  value_amps, ch, HW_MAX_ABS_AMPS, HW_MAX_ABS_AMPS);
          return -1;
        }
        dac_values[ch_in_board] = (int16_t)((value_amps / HW_MAX_ABS_AMPS) * 32767.0);
      }
    }
    // Write all channels, then wait for a single trigger (LDAC latches all channels together)
    dac_cmd_dac_wr(&hw->dac_ctrl, board, dac_values, DAC_TRIGGER_WAIT, !last, DAC_LDAC, 1, hw->verbose);
  }
  return 0;
}

// Send an ADC no-op single trigger wait to all active boards (assumes not last)
int hw_adc_noop_trig(hw_t *hw, uint32_t repeat_count) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    adc_cmd_noop_repeat(&hw->adc_ctrl, board, ADC_TRIGGER_WAIT, ADC_CONTINUE, 1, repeat_count, hw->verbose);
  }
  return 0;
}

// Send an ADC no-op delay command to all active boards (assumes not last)
int hw_adc_noop_delay(hw_t *hw, uint32_t delay_clks, uint32_t repeat_count) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    adc_cmd_noop_repeat(&hw->adc_ctrl, board, ADC_DELAY_WAIT, ADC_CONTINUE, delay_clks, repeat_count, hw->verbose);
  }
  return 0;
}

// Send an ADC read command with a single trigger wait afterwards to all active boards
// Indicate whether this is the last ADC read command in a sequence to control the continue flag
// If last is true, the trigger value will be 0 instead of 1 to immediately finish once read
int hw_adc_read_trig(hw_t *hw, bool last, uint32_t repeat_count) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    adc_cmd_adc_rd(&hw->adc_ctrl, board, ADC_TRIGGER_WAIT, !last, last ? 0 : 1, repeat_count, hw->verbose);
  }
  return 0;
}

// Send an ADC read command with a delay wait afterwards to all active boards
// Indicate whether this is the last ADC read command in a sequence to control the continue flag
int hw_adc_read_delay(hw_t *hw, uint32_t delay_clks, bool last, uint32_t repeat_count) {
  if (hw == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;
  for (uint8_t board = 0; board < board_count; board++) {
    adc_cmd_adc_rd(&hw->adc_ctrl, board, ADC_DELAY_WAIT, !last, delay_clks, repeat_count, hw->verbose);
  }
  return 0;
}

// Read a single sample of 8 ADC channels (4 ADC data words) from all active boards, convert to amps
// Fill these into the provided buffer indexed by channel number (HW_MAX_CHANNELS in length)
// This will only fill in channels corresponding to active boards
// Assume data is available before running
int hw_read_adc_data(hw_t *hw, double *amps) {
  if (hw == NULL || amps == NULL) {
    return -1;
  }
  uint32_t board_count = hw->board_count;

  // Read 4 pairs (8 channels) per active board and convert to amps
  for (uint32_t board = 0; board < board_count; board++) {
    for (uint32_t pair = 0; pair < 4; pair++) {
      uint32_t data_word = adc_read_word(&hw->adc_ctrl, board);
      uint16_t raw = (uint16_t)(data_word & 0xFFFF);
      int16_t adc_val_lo = (raw <= 32767) ? (int16_t)raw : (int16_t)(raw - 65536);
      raw = (uint16_t)((data_word >> 16) & 0xFFFF);
      int16_t adc_val_hi = (raw <= 32767) ? (int16_t)raw : (int16_t)(raw - 65536);

      uint32_t ch_lo = board * 8 + pair * 2;
      uint32_t ch_hi = ch_lo + 1;
      if (ch_lo < hw->channel_count) {
        amps[ch_lo] = HW_MAX_ABS_AMPS * ((double)adc_val_lo) / 32767.0;
      }
      if (ch_hi < hw->channel_count) {
        amps[ch_hi] = HW_MAX_ABS_AMPS * ((double)adc_val_hi) / 32767.0;
      }
    }
  }

  return 0;
}

// Read a single sample of trigger timepoint data (2 trigger data words) from the trigger buffer
// Concatenate these and use clock frequency to convert into a trigger time in seconds
// Assume sample is available before running
int hw_read_trigger_data(hw_t *hw, double *trigger_time) {
  if (hw == NULL || trigger_time == NULL) {
    return -1;
  }

  uint64_t trigger_data = trigger_read(&hw->trigger_ctrl);
  *trigger_time = ((double)trigger_data) / (double)hw->spi_clk_hz;
  return 0;
}

int hw_read_trigger_cycles(hw_t *hw, uint64_t *cycles) {
  if (hw == NULL || cycles == NULL) {
    return -1;
  }
  *cycles = trigger_read(&hw->trigger_ctrl);
  return 0;
}
