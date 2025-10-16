#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "map_memory.h"

//////////////////// ADC Control Definitions ////////////////////
// ADC FIFO address
#define ADC_FIFO(board)    (0x80001000 + (board) * 0x10000)

// ADC FIFO depths
#define ADC_CMD_FIFO_WORDCOUNT   (uint32_t)(1 << 10) // 1024 words (2^10)
#define ADC_DATA_FIFO_WORDCOUNT  (uint32_t)(1 << 13) // 8192 words (2^13)

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
#define ADC_STATE_ERROR      15

// ADC command codes (3 MSB of command word)
#define ADC_CMD_NO_OP     0
#define ADC_CMD_SET_ORD   1
#define ADC_CMD_ADC_RD    2
#define ADC_CMD_ADC_RD_CH 3
#define ADC_CMD_CANCEL    7

// ADC command bits
#define ADC_CMD_CMD_LSB  29
#define ADC_CMD_TRIG_BIT 28
#define ADC_CMD_CONT_BIT 27
#define ADC_CMD_REPEAT_BIT 26

// ADC debug codes
#define ADC_DBG(word)                (((word) >> 28) & 0x0F) // Top 4 bits for debug code
#define ADC_DBG_MISO_DATA            1
#define ADC_DBG_STATE_TRANSITION     2
#define ADC_DBG_REPEAT               3
#define ADC_DBG_COMMAND(word)        (((word) >> 19) & 0x7) // Bits [21:19] for command (used in repeat or cmd_done)
#define ADC_DBG_REPEAT_BIT(word)     (((word) >> 16) & 0x1) // Bit 16 for repeat bit
#define ADC_DBG_N_CS_TIMER           4
#define ADC_DBG_SPI_BIT              5
#define ADC_DBG_CMD_DONE             6
#define ADC_DBG_NEXT_CMD_READY(word) (((word) >> 22) & 0x1) // Bit 22 for next command ready

//////////////////////////////////////////////////////////////////

// ADC control structure
struct adc_ctrl_t {
  volatile uint32_t *buffer[8];  // ADC FIFO (command and data)
};

// Function declarations
// Create ADC control structure
struct adc_ctrl_t create_adc_ctrl(bool verbose);
// Read ADC data word from a specific board
uint32_t adc_read_word(struct adc_ctrl_t *adc_ctrl, uint8_t board);
// Interpret and print ADC value as debug information
void adc_print_debug(uint32_t adc_value, bool verbose);
// Interpret and print the ADC state
void adc_print_state(uint8_t state_code, bool verbose);
// Interpret and print uint8_t command code as string
void adc_print_command(uint8_t cmd_code, bool verbose);
// Convert and print a pair of ADC samples from a 32-bit word
void adc_print_pair(uint32_t data_word, bool verbose);
// Convert and print a single ADC sample from a 32-bit word
void adc_print_single(uint32_t data_word, bool verbose);

// ADC command word functions
void adc_cmd_noop(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, uint32_t loop_count, bool verbose);
void adc_cmd_adc_rd(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, uint32_t loop_count, bool verbose);
void adc_cmd_adc_rd_ch(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t ch, uint32_t loop_count, bool verbose);
void adc_cmd_set_ord(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t channel_order[8], bool verbose);
void adc_cmd_cancel(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool verbose);

#endif // ADC_CTRL_H
