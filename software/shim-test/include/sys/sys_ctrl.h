#ifndef SYS_CTRL_H
#define SYS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Mapped Memory Definitions ////////////////////
// AXI interface addresses
// Addresses are defined in the hardware design Tcl file

// System control and configuration register
#define SYS_CTRL_BASE                  (uint32_t) 0x40000000
#define SYS_CTRL_WORDCOUNT             (uint32_t) 10 // Size in 32-bit words
// 32-bit offsets within the system control and configuration register 
#define SYSTEM_ENABLE_OFFSET           (uint32_t) 0
#define CMD_BUF_RESET_OFFSET           (uint32_t) 1
#define DATA_BUF_RESET_OFFSET          (uint32_t) 2
#define INTEG_THRESHOLD_AVERAGE_OFFSET (uint32_t) 3
#define INTEG_WINDOW_OFFSET            (uint32_t) 4
#define INTEG_ENABLE_OFFSET            (uint32_t) 5
#define BOOT_TEST_SKIP_OFFSET          (uint32_t) 6
#define DEBUG_OFFSET                   (uint32_t) 7
#define MOSI_SCK_POL_OFFSET            (uint32_t) 8
#define MISO_SCK_POL_OFFSET            (uint32_t) 9

//////////////////////////////////////////////////////////////////

// System control structure
struct sys_ctrl_t {
  volatile uint32_t *system_enable;           // System enable
  volatile uint32_t *cmd_buf_reset;           // Command buffer reset
  volatile uint32_t *data_buf_reset;          // Data buffer reset
  volatile uint32_t *integ_threshold_average; // Integrator threshold average
  volatile uint32_t *integ_window;            // Integrator window
  volatile uint32_t *integ_enable;            // Integrator enable
  volatile uint32_t *boot_test_skip;          // Boot test skip
  volatile uint32_t *debug;                   // Debug
  volatile uint32_t *mosi_sck_pol;            // MOSI SCK polarity
  volatile uint32_t *miso_sck_pol;            // MISO SCK polarity
};

// Create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose);
// Turn the system on
void sys_ctrl_turn_on(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Set the boot_test_skip register to a 16-bit value
void sys_ctrl_set_boot_test_skip(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the debug register to a 16-bit value
void sys_ctrl_set_debug(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose);
// Set the command buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_cmd_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);
// Set the data buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_data_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose);
// Invert the MOSI SCK polarity register
void sys_ctrl_invert_mosi_sck(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Invert the MISO SCK polarity register
void sys_ctrl_invert_miso_sck(struct sys_ctrl_t *sys_ctrl, bool verbose);


#endif // SYS_CTRL_H
