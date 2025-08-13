#ifndef SYS_CTRL_H
#define SYS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Mapped Memory Definitions ////////////////////
// AXI interface addresses
// Addresses are defined in the hardware design Tcl file

// System control and configuration register
#define SYS_CTRL_BASE                       (uint32_t) 0x40000000
#define SYS_CTRL_WORDCOUNT                  (uint32_t) 8 // Size in 32-bit words
// 32-bit offsets within the system control and configuration register 
#define SYSTEM_ENABLE_OFFSET                (uint32_t) 0
#define CMD_BUF_RESET_OFFSET                (uint32_t) 1
#define DATA_BUF_RESET_OFFSET               (uint32_t) 2
#define INTEGRATOR_THRESHOLD_AVERAGE_OFFSET (uint32_t) 3
#define INTEGRATOR_WINDOW_OFFSET            (uint32_t) 4
#define INTEGRATOR_ENABLE_OFFSET            (uint32_t) 5
#define BOOT_TEST_SKIP_OFFSET               (uint32_t) 6
#define BOOT_TEST_DEBUG_OFFSET              (uint32_t) 7

//////////////////////////////////////////////////////////////////

// System control structure
struct sys_ctrl_t {
  volatile uint32_t *system_enable;                // System enable
  volatile uint32_t *cmd_buf_reset;                // Command buffer reset
  volatile uint32_t *data_buf_reset;               // Data buffer reset
  volatile uint32_t *integrator_threshold_average; // Integrator threshold average
  volatile uint32_t *integrator_window;            // Integrator window
  volatile uint32_t *integrator_enable;            // Integrator enable
  volatile uint32_t *boot_test_skip;               // Boot test skip
  volatile uint32_t *boot_test_debug;              // Boot test debug
};

// Create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose);
// Turn the system on
void sys_ctrl_turn_on(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Set the boot_test_skip register to a 16-bit value
void sys_ctrl_set_boot_test_skip(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the boot_test_debug register to a 16-bit value
void sys_ctrl_set_boot_test_debug(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the command buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_cmd_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);
// Set the data buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_data_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);


#endif // SYS_CTRL_H
