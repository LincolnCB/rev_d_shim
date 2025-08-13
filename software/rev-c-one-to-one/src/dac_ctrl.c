#include <stdio.h>
#include <stdlib.h>
#include "dac_ctrl.h"
#include "map_memory.h"

// Create DAC control structure for all boards
struct dac_ctrl_t create_dac_ctrl(bool verbose) {
  struct dac_ctrl_t dac_ctrl;

  // Map DAC FIFO for each board
  for (int board = 0; board < 8; board++) {
    dac_ctrl.buffer[board] = map_32bit_memory(DAC_FIFO(board), 1, "DAC FIFO", verbose);
    if (dac_ctrl.buffer[board] == NULL) {
      fprintf(stderr, "Failed to map DAC FIFO access for board %d\n", board);
      exit(EXIT_FAILURE);
    }
  }

  return dac_ctrl;
}

// Read DAC value from a specific board
uint32_t dac_read(struct dac_ctrl_t *dac_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  return *(dac_ctrl->buffer[board]);
}
