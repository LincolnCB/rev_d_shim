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
#define ADC_STATE_ADC_RD_CH  9
#define ADC_STATE_LOOP_NEXT  10
#define ADC_STATE_ERROR      15

// ADC command codes (3 MSB of command word)
#define ADC_CMD_NO_OP     0
#define ADC_CMD_SET_ORD   1
#define ADC_CMD_ADC_RD    2
#define ADC_CMD_ADC_RD_CH 3
#define ADC_CMD_LOOP      4
#define ADC_CMD_CANCEL    7

// ADC command bits
#define ADC_CMD_CMD_LSB  29
#define ADC_CMD_TRIG_BIT 28
#define ADC_CMD_CONT_BIT 27

// ADC debug codes
#define ADC_DBG(word)             (((word) >> 28) & 0x0F) // Top 4 bits for debug code
#define ADC_DBG_MISO_DATA         1
#define ADC_DBG_STATE_TRANSITION  2
#define ADC_DBG_N_CS_TIMER        3
#define ADC_DBG_SPI_BIT           4

// ADC signed to offset conversion
#define ADC_OFFSET_TO_SIGNED(val) \
  (((val) == 0xFFFF) ? 0 : ((int16_t)((val) - 32767)))
#define ADC_SIGNED_TO_OFFSET(val) \
  (((val) < -32767 || (val) > 32767) ? 32767 : ((uint16_t)((val) + 32767)))


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
// Read a single ADC sample (one channel) from a specific board
int16_t adc_read_ch(struct adc_ctrl_t *adc_ctrl, uint8_t board);
// Interpret and print ADC value as debug information
void adc_print_debug(uint32_t adc_value);
// Interpret and print the ADC state
void adc_print_state(uint8_t state_code);
// Convert and print a pair of ADC samples from a 32-bit word
void adc_print_pair(uint32_t data_word);
// Convert and print a single ADC sample from a 32-bit word
void adc_print_single(uint32_t data_word);

// ADC command word functions
void adc_cmd_noop(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, bool verbose);
void adc_cmd_adc_rd(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, bool verbose);
void adc_cmd_adc_rd_ch(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t ch, bool verbose);
void adc_cmd_set_ord(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t channel_order[8], bool verbose);
void adc_cmd_cancel(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool verbose);
void adc_cmd_loop_next(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint32_t loop_count, bool verbose);

#endif // ADC_CTRL_H
