#ifndef SYS_CTRL_H
#define SYS_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Mapped Memory Definitions ////////////////////
// AXI interface addresses
// Addresses are defined in the hardware design Tcl file

// System control and configuration register
#define SYS_CTRL_BASE                       (uint32_t) 0x40000000
#define SYS_CTRL_WORDCOUNT                  (uint32_t) 5 // Size in 32-bit words
// 32-bit offsets within the system control and configuration register 
#define INTEGRATOR_THRESHOLD_AVERAGE_OFFSET (uint32_t) 0
#define INTEGRATOR_WINDOW_OFFSET            (uint32_t) 1
#define INTEGRATOR_ENABLE_OFFSET            (uint32_t) 2
#define BUFFER_RESET_OFFSET                 (uint32_t) 3
#define SYSTEM_ENABLE_OFFSET                (uint32_t) 4

//////////////////////////////////////////////////////////////////

// System control structure
struct sys_ctrl_t {
  volatile uint32_t *integrator_threshold_average; // Integrator threshold average
  volatile uint32_t *integrator_window;            // Integrator window
  volatile uint32_t *integrator_enable;            // Integrator enable
  volatile uint32_t *buffer_reset;                 // Buffer reset
  volatile uint32_t *system_enable;                // System enable
};

// Function to create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose);
// Function to turn the system on
void sys_ctrl_turn_on(struct sys_ctrl_t *sys_ctrl, bool verbose);
// Function to turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose);


#endif // SYS_CTRL_H
