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

// DAC control structure
struct dac_ctrl_t {
  volatile uint32_t *buffer[8];  // DAC FIFO (command and data)
};

// Function declarations
// Create DAC control structure
struct dac_ctrl_t create_dac_ctrl(bool verbose);
// Read DAC value from a specific board
uint32_t dac_read(struct dac_ctrl_t *dac_ctrl, uint8_t board);

#endif // DAC_CTRL_H
