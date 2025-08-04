#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// ADC Control Definitions ////////////////////
// ADC command and data FIFOs
#define ADC_CMD_FIFO(board)   (0x80001000 + (board) * 0x10000)
#define ADC_DATA_FIFO(board)  (0x80002000 + (board) * 0x10000)
#define ADC_FIFO_WORDCOUNT    (uint32_t) 1024 // Size in 32-bit words

// ADC status register offsets (within sys_sts register)
#define ADC_CMD_FIFO_STS_OFFSET(board)  (2 + 3 * (board)) // ADC command FIFO status
#define ADC_DATA_FIFO_STS_OFFSET(board) (3 + 3 * (board)) // ADC data FIFO status

//////////////////////////////////////////////////////////////////

// ADC control structure for a single board
struct adc_ctrl_t {
  volatile uint32_t *cmd_fifo;  // ADC command FIFO
  volatile uint32_t *data_fifo; // ADC data FIFO
  uint8_t board_id;             // Board identifier (0-7)
};

// ADC control structure for all boards
struct adc_ctrl_array_t {
  struct adc_ctrl_t boards[8];  // Array of ADC control structures for 8 boards
};

// Function declarations
struct adc_ctrl_t create_adc_ctrl(uint8_t board_id, bool verbose);
struct adc_ctrl_array_t create_adc_ctrl_array(bool verbose);

#endif // ADC_CTRL_H
