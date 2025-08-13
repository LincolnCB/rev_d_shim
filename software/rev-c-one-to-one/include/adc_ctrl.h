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

// ADC control structure
struct adc_ctrl_t {
  volatile uint32_t *buffer[8];  // ADC FIFO (command and data)
};

// Function declarations
// Create ADC control structure
struct adc_ctrl_t create_adc_ctrl(bool verbose);
// Read ADC value from a specific board
uint32_t adc_read(struct adc_ctrl_t *adc_ctrl, uint8_t board);

#endif // ADC_CTRL_H
