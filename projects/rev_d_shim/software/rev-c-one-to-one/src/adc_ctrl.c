#include <stdio.h>
#include <stdlib.h>
#include "adc_ctrl.h"
#include "map_memory.h"

// Function to create ADC control structure for a single board
struct adc_ctrl_t create_adc_ctrl(uint8_t board_id, bool verbose) {
  struct adc_ctrl_t adc_ctrl;
  
  if (board_id > 7) {
    fprintf(stderr, "Invalid board ID: %d. Must be 0-7.\n", board_id);
    exit(EXIT_FAILURE);
  }
  
  // Map ADC command FIFO
  adc_ctrl.cmd_fifo = map_32bit_memory(ADC_CMD_FIFO(board_id), ADC_FIFO_WORDCOUNT, "ADC CMD FIFO", verbose);
  if (adc_ctrl.cmd_fifo == NULL) {
    fprintf(stderr, "Failed to map ADC command FIFO for board %d.\n", board_id);
    exit(EXIT_FAILURE);
  }
  
  // Map ADC data FIFO
  adc_ctrl.data_fifo = map_32bit_memory(ADC_DATA_FIFO(board_id), ADC_FIFO_WORDCOUNT, "ADC DATA FIFO", verbose);
  if (adc_ctrl.data_fifo == NULL) {
    fprintf(stderr, "Failed to map ADC data FIFO for board %d.\n", board_id);
    exit(EXIT_FAILURE);
  }
  
  adc_ctrl.board_id = board_id;
  return adc_ctrl;
}

// Function to create ADC control structures for all boards
struct adc_ctrl_array_t create_adc_ctrl_array(bool verbose) {
  struct adc_ctrl_array_t adc_array;
  
  for (int i = 0; i < 8; i++) {
    adc_array.boards[i] = create_adc_ctrl(i, verbose);
  }
  
  return adc_array;
}
