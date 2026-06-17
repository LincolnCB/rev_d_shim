#ifndef BOOT_H
#define BOOT_H

#include <stdint.h>
#include <stdbool.h>

#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "clk_ctrl.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

#define HW_SLEEP usleep(1000) // 1 ms sleep for hardware timing
#define HW_MAX_CHANNELS 64 // Maximum number of channels supported by hardware

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

// Configure clock. Returns 0 on success, non-zero on failure
int hw_set_clk(hw_t *hw);

// Power on hardware. Returns 0 on success, non-zero on failure
int hw_power_on(hw_t *hw);

// Power off hardware. Safe to call from a signal handler via a flag
void hw_power_off(hw_t *hw);

// Check that the hardware is running
bool hw_running(hw_t *hw);

// Check that the hardware is halted
bool hw_halted(hw_t *hw);

// Clear DAC buffers
int hw_clear_dac_buffers(hw_t *hw);

// Clear ADC buffers
int hw_clear_adc_buffers(hw_t *hw);

// Clear trigger buffers
int hw_clear_trigger_buffers(hw_t *hw);

// Start external triggers
int hw_start_triggers(hw_t *hw);

// Force n triggers
int hw_force_trigger(hw_t *hw, uint32_t n);

// Reset trigger counter
int hw_reset_triggers(hw_t *hw);

// Set trigger lockout in ms
int hw_set_trigger_lockout(hw_t *hw, double trigger_lockout_ms);

// Run channel calibration routine
int hw_calibrate(hw_t *hw);

// Get trigger count
uint32_t hw_get_trigger_count(hw_t *hw);

// Send zero command to all DAC channels
int hw_zero_dacs(hw_t *hw);

// Send ADC read ch command for a specific channel and return the result in amps
int hw_read_adc_channel(hw_t *hw, uint32_t channel, double *adc_value_amps);

// Send ADC read commmand to all boards and return results to a provided buffer
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
int hw_read_adcs(hw_t *hw, double *adc_values_amps);

// Send DAC set ch command for a specific channel with a value in amps
int hw_set_dac_channel(hw_t *hw, uint32_t channel, double amps);

// Send DAC set all channels command with values in amps 
// (buffer is HW_MAX_CHANNELS in length and indexed by channel number)
int hw_set_dacs(hw_t *hw, double *amps);

#endif // BOOT_H
