#ifndef SYS_CTRL_H
#define SYS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Mapped Memory Definitions ////////////////////
// AXI interface addresses
// Addresses are defined in the hardware design Tcl file

// System control and configuration register
#define SYS_CTRL_BASE             (uint32_t) 0x40000000
#define SYS_CTRL_WORDCOUNT        (uint32_t) 10 // Size in 32-bit words
// 32-bit offsets within the system control and configuration register
#define CTRL_ENABLE_OFFSET        (uint32_t) 0
#define POWER_ENABLE_OFFSET       (uint32_t) 1
#define CMD_BUF_RESET_OFFSET      (uint32_t) 2
#define DATA_BUF_RESET_OFFSET     (uint32_t) 3
#define THRESHOLD_VALUE_OFFSET    (uint32_t) 4
#define THRESHOLD_WINDOW_OFFSET   (uint32_t) 5
#define THRESHOLD_ENABLE_OFFSET   (uint32_t) 6
#define BOOT_TEST_SKIP_OFFSET     (uint32_t) 7
#define DEBUG_OFFSET              (uint32_t) 8
#define DAC_CAL_INIT_OFFSET       (uint32_t) 9
#define DO_DAC_PRE_DELAY_OFFSET   (uint32_t) 10

//////////////////////////////////////////////////////////////////

// System control structure
struct sys_ctrl_t {
  volatile uint32_t *ctrl_en;          // Ctrl enable
  volatile uint32_t *pow_en;           // Power enable
  volatile uint32_t *cmd_buf_reset;    // Command buffer reset
  volatile uint32_t *data_buf_reset;   // Data buffer reset
  volatile uint32_t *thresh_val;       // Threshold average
  volatile uint32_t *thresh_window;    // Threshold window
  volatile uint32_t *thresh_en;        // Threshold enable
  volatile uint32_t *boot_test_skip;   // Boot test skip
  volatile uint32_t *debug;            // Debug
  volatile uint32_t *dac_cal_init;     // DAC calibration init
  volatile uint32_t *do_dac_pre_delay; // Do DAC pre-delay
};

// Create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose);
// Turn the control board on
void sys_ctrl_turn_ctrl_on(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Turn the power board on
void sys_ctrl_turn_pow_on(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Set the command buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_cmd_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);
// Set the data buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_data_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);
// Set the threshold average register to a 32-bit value
void sys_ctrl_set_thresh_val(struct sys_ctrl_t *sys_ctrl, uint32_t value, bool verbose);
// Set the threshold window register to a 32-bit value
void sys_ctrl_set_thresh_window(struct sys_ctrl_t *sys_ctrl, uint32_t value, bool verbose);
// Set the threshold enable register to a 32-bit value
void sys_ctrl_set_thresh_en(struct sys_ctrl_t *sys_ctrl, uint32_t value, bool verbose);
// Set the boot_test_skip register to a 16-bit value
void sys_ctrl_set_boot_test_skip(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the debug register to a 16-bit value
void sys_ctrl_set_debug(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the DAC calibration init register to a 16-bit signed value
void sys_ctrl_set_dac_cal_init(struct sys_ctrl_t *sys_ctrl, int16_t value, bool verbose);
// Toggle the DAC pre-delay bit in the do_dac_pre_delay register
void sys_ctrl_toggle_dac_pre_delay(struct sys_ctrl_t *sys_ctrl, bool verbose);


#endif // SYS_CTRL_H
