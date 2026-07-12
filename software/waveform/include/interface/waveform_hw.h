#ifndef WAVEFORM_HW_H
#define WAVEFORM_HW_H

#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>

#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "clk_ctrl.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

#define HW_SLEEP usleep(1000) // 1 ms sleep for hardware timing
#define HW_MAX_CHANNELS 64 // Maximum number of channels supported by hardware
#define HW_MAX_ABS_AMPS 5.0 // Maximum absolute current in amps for DAC channels

// Aggregates all hardware control structures needed for boot and operation
typedef struct {
  struct sys_ctrl_t     sys_ctrl;
  struct clk_ctrl_t     clk_ctrl;
  struct sys_sts_t      sys_sts;
  struct dac_ctrl_t     dac_ctrl;
  struct adc_ctrl_t     adc_ctrl;
  struct trigger_ctrl_t trigger_ctrl;
  uint32_t              channel_count;
  bool                  verbose;
} hw_t;

// Initialize and validate hardware control structure for a given channel count. Exits on failure
// Returns a populated hw_t
hw_t hw_init(uint32_t channel_count, bool verbose);

// Set the SPI clock frequency to the specified value in MHz. Returns 0 on success, non-zero on failure
int hw_set_spi_clock(hw_t *hw, double clk_mhz);

// Clear DAC buffers
int hw_clear_dac_buffers(hw_t *hw);

// Clear ADC buffers
int hw_clear_adc_buffers(hw_t *hw);

// Clear trigger buffers
int hw_clear_trigger_buffers(hw_t *hw);

// Power on the hardware.
int hw_power_on(hw_t *hw);

// Check if the hardware is running.
bool hw_running(hw_t *hw);

// Check if the hardware is halted.
bool hw_halted(hw_t *hw);

// Run calibration routine for connected channels.
int hw_calibrate(hw_t *hw);

// Validate the timing for DAC and ADC delays (calculated after power-on).
bool hw_dac_timing_valid(hw_t *hw, double min_dt);
bool hw_adc_timing_valid(hw_t *hw, double min_dt);

// Set the lockout for the triggers
int hw_set_trigger_lockout(hw_t *hw, double lockout_ms);

// Start the expected count of triggers
int hw_start_triggers(hw_t *hw, uint32_t expected_triggers);

// Get trigger count
uint32_t hw_get_trigger_count(hw_t *hw);

// Reset trigger counter
int hw_reset_triggers(hw_t *hw);

// Power off the hardware.
void hw_power_off(hw_t *hw);

#endif // WAVEFORM_HW_H
