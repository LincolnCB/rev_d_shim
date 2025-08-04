#ifndef DAC_CTRL_H
#define DAC_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// DAC Control Definitions ////////////////////
// DAC command FIFO
#define DAC_CMD_FIFO(board)   (0x80000000 + (board) * 0x10000)
#define DAC_FIFO_WORDCOUNT    (uint32_t) 1024 // Size in 32-bit words

// DAC status register offset (within sys_sts register)
#define DAC_CMD_FIFO_STS_OFFSET(board)  (1 + 3 * (board)) // DAC command FIFO status

//////////////////////////////////////////////////////////////////

// DAC control structure for a single board
struct dac_ctrl_t {
  volatile uint32_t *cmd_fifo; // DAC command FIFO
  uint8_t board_id;            // Board identifier (0-7)
};

// DAC control structure for all boards
struct dac_ctrl_array_t {
  struct dac_ctrl_t boards[8]; // Array of DAC control structures for 8 boards
};

// Function declarations
struct dac_ctrl_t create_dac_ctrl(uint8_t board_id, bool verbose);
struct dac_ctrl_array_t create_dac_ctrl_array(bool verbose);

#endif // DAC_CTRL_H
