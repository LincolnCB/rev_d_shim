#ifndef DAC_CTRL_H
#define DAC_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// DAC Control Definitions ////////////////////
// DAC FIFO address
#define DAC_FIFO(board)    (0x80000000 + (board) * 0x10000)

// DAC FIFO depths
#define DAC_CMD_FIFO_WORDCOUNT   (uint32_t) 1024 // Size in 32-bit words
#define DAC_DATA_FIFO_WORDCOUNT  (uint32_t) 1024 // Size in 32-bit words

//////////////////////////////////////////////////////////////////

// DAC control structure for a single board
struct dac_ctrl_t {
  volatile uint32_t *buffer; // DAC FIFO (command and data)
  uint8_t board_id;          // Board identifier (0-7)
};

// DAC control structure for all boards
struct dac_ctrl_array_t {
  struct dac_ctrl_t boards[8]; // Array of DAC control structures for 8 boards
};

// Function declarations
struct dac_ctrl_t create_dac_ctrl(uint8_t board_id, bool verbose);
struct dac_ctrl_array_t create_dac_ctrl_array(bool verbose);

#endif // DAC_CTRL_H
