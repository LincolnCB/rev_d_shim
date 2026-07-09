#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "sys_ctrl.h"

int main(int argc, char **argv) {
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
