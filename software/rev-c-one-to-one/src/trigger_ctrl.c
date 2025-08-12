#include <stdio.h>
#include <stdlib.h>
#include "trigger_ctrl.h"
#include "map_memory.h"

// Function to create trigger control structure
struct trigger_ctrl_t create_trigger_ctrl(bool verbose) {
  struct trigger_ctrl_t trigger_ctrl;
  
  // Map trigger command FIFO
  trigger_ctrl.buffer = map_32bit_memory(TRIG_CMD_FIFO, 1, "Trigger CMD FIFO", verbose);
  if (trigger_ctrl.buffer == NULL) {
    fprintf(stderr, "Failed to map trigger FIFO access memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  return trigger_ctrl;
}
