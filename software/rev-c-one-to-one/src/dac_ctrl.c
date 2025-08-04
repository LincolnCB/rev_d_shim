#include <stdio.h>
#include <stdlib.h>
#include "dac_ctrl.h"
#include "map_memory.h"

// Function to create DAC control structure for a single board
struct dac_ctrl_t create_dac_ctrl(uint8_t board_id, bool verbose) {
  struct dac_ctrl_t dac_ctrl;
  
  if (board_id > 7) {
    fprintf(stderr, "Invalid board ID: %d. Must be 0-7.\n", board_id);
    exit(EXIT_FAILURE);
  }
  
  // Map DAC command FIFO
  dac_ctrl.cmd_fifo = map_32bit_memory(DAC_CMD_FIFO(board_id), DAC_FIFO_WORDCOUNT, "DAC CMD FIFO", verbose);
  if (dac_ctrl.cmd_fifo == NULL) {
    fprintf(stderr, "Failed to map DAC command FIFO for board %d.\n", board_id);
    exit(EXIT_FAILURE);
  }
  
  dac_ctrl.board_id = board_id;
  return dac_ctrl;
}

// Function to create DAC control structures for all boards
struct dac_ctrl_array_t create_dac_ctrl_array(bool verbose) {
  struct dac_ctrl_array_t dac_array;
  
  for (int i = 0; i < 8; i++) {
    dac_array.boards[i] = create_dac_ctrl(i, verbose);
  }
  
  return dac_array;
}
