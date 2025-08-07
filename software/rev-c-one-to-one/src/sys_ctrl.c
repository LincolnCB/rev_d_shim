#include <inttypes.h> // For PRIx32 format specifier
#include <stdio.h> // For printf and perror functions
#include <stdlib.h> // For exit function and NULL definition etc.
#include <sys/mman.h> // For mmap function
#include "sys_ctrl.h"    // Include the header for sys_ctrl structure
#include "map_memory.h"  // Include the header for map_memory function

// Function to create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose) {
  struct sys_ctrl_t sys_ctrl;
  volatile uint32_t *sys_ctrl_ptr = map_32bit_memory(SYS_CTRL_BASE, SYS_CTRL_WORDCOUNT, "System Ctrl", verbose);
  if (sys_ctrl_ptr == NULL) {
    fprintf(stderr, "Failed to map system control memory region.\n");
    exit(EXIT_FAILURE);
  }

  // Initialize the system control structure with the mapped memory addresses
  sys_ctrl.integrator_threshold_average = sys_ctrl_ptr + INTEGRATOR_THRESHOLD_AVERAGE_OFFSET;
  sys_ctrl.integrator_window = sys_ctrl_ptr + INTEGRATOR_WINDOW_OFFSET;
  sys_ctrl.integrator_enable = sys_ctrl_ptr + INTEGRATOR_ENABLE_OFFSET;
  sys_ctrl.buffer_reset = sys_ctrl_ptr + BUFFER_RESET_OFFSET;
  sys_ctrl.system_enable = sys_ctrl_ptr + SYSTEM_ENABLE_OFFSET;
  
  return sys_ctrl;
}

// Function to turn the system on
void sys_ctrl_turn_on(struct sys_ctrl_t *sys_ctrl, bool verbose) {
  if (verbose) {
    printf("Turning on the system...\n");
  }
  *(sys_ctrl->system_enable) = 1; // Set the system enable register to 1
}

// Function to turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose) {
  if (verbose) {
    printf("Turning off the system...\n");
  }
  *(sys_ctrl->system_enable) = 0; // Set the system enable register to 0
}
