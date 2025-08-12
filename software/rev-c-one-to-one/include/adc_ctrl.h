#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// ADC Control Definitions ////////////////////
// ADC FIFO address
#define ADC_FIFO(board)    (0x80001000 + (board) * 0x10000)

// ADC FIFO depths
#define ADC_CMD_FIFO_WORDCOUNT  (uint32_t) 1024 // Size in 32-bit words
#define ADC_DATA_FIFO_WORDCOUNT  (uint32_t) 1024 // Size in 32-bit words

//////////////////////////////////////////////////////////////////

// ADC control structure for a single board
struct adc_ctrl_t {
  volatile uint32_t *buffer;  // ADC FIFO (command and data)
  uint8_t board_id;           // Board identifier (0-7)
};

// ADC control structure for all boards
struct adc_ctrl_array_t {
  struct adc_ctrl_t boards[8];  // Array of ADC control structures for 8 boards
};

// Function declarations
struct adc_ctrl_t create_adc_ctrl(uint8_t board_id, bool verbose);
struct adc_ctrl_array_t create_adc_ctrl_array(bool verbose);

#endif // ADC_CTRL_H
