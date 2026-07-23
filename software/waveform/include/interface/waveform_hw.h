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
  uint32_t              board_count;    // number of active 8-channel boards, derived from channel_count
  uint32_t              spi_clk_hz;    // Current SPI clock frequency in Hz
  bool                  verbose;
} hw_t;

// Initialize and validate hardware control structure for a given channel count. Exits on failure
// Returns a populated hw_t
hw_t hw_init(uint32_t channel_count, double clk_MHz, bool verbose);

// Set the SPI clock frequency to the specified value in MHz. Returns 0 on success, non-zero on failure
int hw_set_spi_clock(hw_t *hw, double clk_MHz);

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

// Get the available DAC sample command space (minimum across all active boards)
// This is the available cmd words divided by 5 (rounded down)
// This indicates how many individual DAC sample commands can be sent
int hw_get_dac_sample_cmd_space(hw_t *hw);

// Get the available ADC command space (minimum across all active boards)
int hw_get_adc_cmd_space(hw_t *hw);

// Get the available ADC sample words (minimum across all active boards)
// This is the available data words divided by 4 (rounded down)
// This indicates how many individual ADC samples can be read
int hw_get_adc_sample_count(hw_t *hw);

// Get the available trigger sample words (only one buffer)
// Each trigger sample is 2 words, so it's just the words divided by 2 (rounded down)
// This indicates how many individual trigger samples can be read
int hw_get_trigger_sample_count(hw_t *hw);

// Send a DAC noop command for a single trigger wait to all active boards (assumes not last)
int hw_dac_noop_trig(hw_t *hw);

// Send a DAC noop command for a delay to all active boards (assumes not last)
int hw_dac_noop_delay(hw_t *hw, uint32_t delay_clks);

// Send a DAC command with a delay in clock cycles
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
// There are 8 channels per active board, and all need to be included
// Zero out unused channels within an active board, and skip non-active boards
// Indicate whether this is the last DAC command in a sequence to control the continue flag
int hw_set_dacs_delay(hw_t *hw, const double *amps, uint32_t delay_clks, bool last);

// Send a DAC command with a single trigger wait
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
// There are 8 channels per active board, and all need to be included
// Zero out unused channels within an active board, and skip non-active boards
// Indicate whether this is the last DAC command in a sequence to control the continue flag
int hw_set_dacs_trig(hw_t *hw, const double *amps, bool last);

// Send an ADC no-op single trigger wait to start to all active boards (assumes not last)
int hw_adc_noop_trig(hw_t *hw);

// Send an ADC no-op delay command to all active boards (assumes not last)
int hw_adc_noop_delay(hw_t *hw, uint32_t delay_clks);

// Send an ADC read command with a single trigger wait afterwards to all active boards
// Indicate whether this is the last ADC read command in a sequence to control the continue flag
int hw_adc_read_trig(hw_t *hw, bool last);

// Send an ADC read command with a delay wait afterwards to all active boards
// Indicate whether this is the last ADC read command in a sequence to control the continue flag
int hw_adc_read_delay(hw_t* hw, uint32_t delay_clks, bool last);

// Read a single sample of 8 ADC channels (4 ADC data words) from all active boards, convert to amps
// Fill these into the provided buffer indexed by channel number (HW_MAX_CHANNELS in length)
// This will only fill in channels corresponding to active boards
int hw_read_adc_data(hw_t *hw, double *amps);

#endif // WAVEFORM_HW_H
