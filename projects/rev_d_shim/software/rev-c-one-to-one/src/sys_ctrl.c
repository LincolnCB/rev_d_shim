#include <inttypes.h> // For PRIx32 format specifier
#include <stdio.h> // For printf and perror functions
#include <stdlib.h> // For exit function and NULL definition etc.
#include <sys/mman.h> // For mmap function
#include "sys_ctrl.h"    // Include the header for sys_ctrl structure
#include "map_memory.h"  // Include the header for map_memory function

// Create a system control structure
struct sys_ctrl_t create_sys_ctrl(bool verbose) {
  struct sys_ctrl_t sys_ctrl;
  volatile uint32_t *sys_ctrl_ptr = map_32bit_memory(SYS_CTRL_BASE, SYS_CTRL_WORDCOUNT, "System Ctrl", verbose);
  if (sys_ctrl_ptr == NULL) {
    fprintf(stderr, "Failed to map system control memory region.\n");
    exit(EXIT_FAILURE);
  }

  // Initialize the system control structure with the mapped memory addresses
  sys_ctrl.system_enable                = sys_ctrl_ptr + SYSTEM_ENABLE_OFFSET;
  sys_ctrl.cmd_buf_reset                = sys_ctrl_ptr + CMD_BUF_RESET_OFFSET;
  sys_ctrl.data_buf_reset               = sys_ctrl_ptr + DATA_BUF_RESET_OFFSET;
  sys_ctrl.integrator_threshold_average = sys_ctrl_ptr + INTEGRATOR_THRESHOLD_AVERAGE_OFFSET;
  sys_ctrl.integrator_window            = sys_ctrl_ptr + INTEGRATOR_WINDOW_OFFSET;
  sys_ctrl.integrator_enable            = sys_ctrl_ptr + INTEGRATOR_ENABLE_OFFSET;
  sys_ctrl.boot_test_skip               = sys_ctrl_ptr + BOOT_TEST_SKIP_OFFSET;
  sys_ctrl.boot_test_debug              = sys_ctrl_ptr + BOOT_TEST_DEBUG_OFFSET;
  
  return sys_ctrl;
}

// Turn the system on
void sys_ctrl_turn_on(struct sys_ctrl_t *sys_ctrl, bool verbose) {
  if (verbose) {
    printf("Turning on the system...\n");
  }
  *(sys_ctrl->system_enable) = 1; // Set the system enable register to 1
}

// Turn the system off
void sys_ctrl_turn_off(struct sys_ctrl_t *sys_ctrl, bool verbose) {
  if (verbose) {
    printf("Turning off the system...\n");
  }
  *(sys_ctrl->system_enable) = 0; // Set the system enable register to 0
}

// Set the boot_test_skip register to a 16-bit value
void sys_ctrl_set_boot_test_skip(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose) {
  if (verbose) {
    printf("Setting boot_test_skip to 0x%" PRIx32 "\n", value);
  }
  // Write the 16-bit value to the boot_test_skip register
  *(sys_ctrl->boot_test_skip) = (uint32_t)value;
  if (verbose) {
    printf("boot_test_skip set to 0x%" PRIx32 "\n", *(sys_ctrl->boot_test_skip));
  }
}

// Set the boot_test_debug register to a 16-bit value
void sys_ctrl_set_boot_test_debug(struct sys_ctrl_t *sys_ctrl, uint16_t value, bool verbose) {
  if (verbose) {
    printf("Setting boot_test_debug to 0x%" PRIx32 "\n", value);
  }
  // Write the 16-bit value to the boot_test_debug register
  *(sys_ctrl->boot_test_debug) = (uint32_t)value;
  if (verbose) {
    printf("boot_test_debug set to 0x%" PRIx32 "\n", *(sys_ctrl->boot_test_debug));
  }
}

// Set the command buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_cmd_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose) {
  if (mask > 0x1FFFF) {
    fprintf(stderr, "Invalid command buffer reset mask: 0x%" PRIx32 ". Must be a 17-bit value.\n", mask);
    exit(EXIT_FAILURE);
  }
  if (verbose) {
    printf("Setting cmd_buf_reset to 0x%" PRIx32 "\n", mask);
  }
  // Write the 17-bit mask to the cmd_buf_reset register
  *(sys_ctrl->cmd_buf_reset) = mask & 0x1FFFF; // Mask to 17 bits
  if (verbose) {
    printf("cmd_buf_reset set to 0x%" PRIx32 "\n", *(sys_ctrl->cmd_buf_reset));
  }
}

// Set the data buffer reset register (1 = reset) to a 17-bit mask
void sys_ctrl_set_data_buf_reset(struct sys_ctrl_t *sys_ctrl, uint32_t mask, bool verbose) {
  if (mask > 0x1FFFF) {
    fprintf(stderr, "Invalid data buffer reset mask: 0x%" PRIx32 ". Must be a 17-bit value.\n", mask);
    exit(EXIT_FAILURE);
  }
  if (verbose) {
    printf("Setting data_buf_reset to 0x%" PRIx32 "\n", mask);
  }
  // Write the 17-bit mask to the data_buf_reset register
  *(sys_ctrl->data_buf_reset) = mask & 0x1FFFF; // Mask to 17 bits
  if (verbose) {
    printf("data_buf_reset set to 0x%" PRIx32 "\n", *(sys_ctrl->data_buf_reset));
  }
}
