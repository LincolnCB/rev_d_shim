#include <stdio.h>
#include <stdlib.h>
#include "trigger_ctrl.h"
#include "map_memory.h"

// Function to create trigger control structure
struct trigger_ctrl_t create_trigger_ctrl(bool verbose) {
  struct trigger_ctrl_t trigger_ctrl;
  
  // Map trigger command FIFO
  trigger_ctrl.cmd_fifo = map_32bit_memory(TRIG_CMD_FIFO, TRIG_FIFO_WORDCOUNT, "Trigger CMD FIFO", verbose);
  if (trigger_ctrl.cmd_fifo == NULL) {
    fprintf(stderr, "Failed to map trigger command FIFO memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  // Map trigger data FIFO
  trigger_ctrl.data_fifo = map_32bit_memory(TRIG_DATA_FIFO, TRIG_FIFO_WORDCOUNT, "Trigger DATA FIFO", verbose);
  if (trigger_ctrl.data_fifo == NULL) {
    fprintf(stderr, "Failed to map trigger data FIFO memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  return trigger_ctrl;
}
