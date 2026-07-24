#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "sys_ctrl.h"

int main(int argc, char **argv) {
  // Handle help flag
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printf("Usage: %s [-h|--help]\n", argv[0]);
      printf("\n");
      printf("Reset the Rev D shim amplifier hardware to a safe, powered-off state.\n");
      printf("\n");
      printf("This tool:\n");
      printf("  - Powers off the shim system.\n");
      printf("  - Pulses the 17-bit reset mask on the command and data buffers.\n");
      printf("\n");
      printf("Options:\n");
      printf("  -h, --help  Show this help message (without executing) and exit.\n");
      return EXIT_SUCCESS;
    }
  }

  bool verbose = true;
  
  // Load up the system config
  struct sys_ctrl_t sys_ctrl = create_sys_ctrl(verbose);
  
  // Immediately power it off
  sys_ctrl_turn_off(&sys_ctrl, verbose);
  
  // Pulse the 17-bit mask reset to buffers
  uint32_t reset_mask = 0x1FFFF; // Full 17-bit mask
  sys_ctrl_set_cmd_buf_reset(&sys_ctrl, reset_mask, verbose);
  sys_ctrl_set_data_buf_reset(&sys_ctrl, reset_mask, verbose);
  usleep(1000); // Small delay
  sys_ctrl_set_cmd_buf_reset(&sys_ctrl, 0, verbose); // Release reset
  sys_ctrl_set_data_buf_reset(&sys_ctrl, 0, verbose); // Release reset
  
  printf("Reset pulse complete.\n");
  return 0;
}
