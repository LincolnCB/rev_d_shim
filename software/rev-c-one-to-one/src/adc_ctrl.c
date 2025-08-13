#include <stdio.h>
#include <stdlib.h>
#include "adc_ctrl.h"
#include "map_memory.h"

// Create ADC control structure for a single board
struct adc_ctrl_t create_adc_ctrl(bool verbose) {
  struct adc_ctrl_t adc_ctrl;

  // Map ADC FIFO for each board
  for (int board = 0; board < 8; board++) {
    adc_ctrl.buffer[board] = map_32bit_memory(ADC_FIFO(board), 1, "ADC FIFO", verbose);
    if (adc_ctrl.buffer[board] == NULL) {
      fprintf(stderr, "Failed to map ADC FIFO access for board %d\n", board);
      exit(EXIT_FAILURE);
    }
  }

  return adc_ctrl;
}

// Read ADC value from a specific board
uint32_t adc_read(struct adc_ctrl_t *adc_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  return *(adc_ctrl->buffer[board]);
}
