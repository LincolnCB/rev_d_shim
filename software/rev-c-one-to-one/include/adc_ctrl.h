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

// ADC command codes (2 MSB of command word)
#define ADC_CMD_NO_OP    0x0
#define ADC_CMD_ADC_RD   0x1
#define ADC_CMD_SET_ORD  0x2
#define ADC_CMD_CANCEL   0x3

// ADC command bits
#define ADC_CMD_TRIG_BIT 29
#define ADC_CMD_CONT_BIT 28

// ADC state codes
#define ADC_STATE_RESET      0
#define ADC_STATE_INIT       1
#define ADC_STATE_TEST_WR    2
#define ADC_STATE_REQ_RD     3
#define ADC_STATE_TEST_RD    4
#define ADC_STATE_IDLE       5
#define ADC_STATE_DELAY      6
#define ADC_STATE_TRIG_WAIT  7
#define ADC_STATE_ADC_RD     8
#define ADC_STATE_ERROR      9

// ADC debug codes
#define ADC_DBG(word)             (((word) >> 28) & 0x0F) // Top 4 bits for debug code
#define ADC_DBG_MISO_DATA         1
#define ADC_DBG_STATE_TRANSITION  2
#define ADC_DBG_N_CS_TIMER        3
#define ADC_DBG_SPI_BIT           4


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
// Interpret and print ADC value as debug information
void adc_print_debug(uint32_t adc_value);
// Interpret and print the ADC state
void adc_print_state(uint8_t state_code);

#endif // ADC_CTRL_H
