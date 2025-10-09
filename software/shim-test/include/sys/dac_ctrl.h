#ifndef DAC_CTRL_H
#define DAC_CTRL_H

#include <stdint.h>
#include <stdbool.h>
#include "map_memory.h"

//////////////////// DAC Control Definitions ////////////////////
// DAC FIFO address
#define DAC_FIFO(board)    (0x80000000 + (board) * 0x10000)

// DAC FIFO depths
#define DAC_CMD_FIFO_WORDCOUNT   (uint32_t)(1 << 13) // 8192 words (2^13)
#define DAC_DATA_FIFO_WORDCOUNT  (uint32_t)(1 << 10) // 1024 words (2^10)

// DAC state codes
#define DAC_STATE_RESET      0
#define DAC_STATE_INIT       1
#define DAC_STATE_TEST_WR    2
#define DAC_STATE_REQ_RD     3
#define DAC_STATE_TEST_RD    4
#define DAC_STATE_SET_MID    5
#define DAC_STATE_IDLE       6
#define DAC_STATE_DELAY      7
#define DAC_STATE_TRIG_WAIT  8
#define DAC_STATE_DAC_WR     9
#define DAC_STATE_DAC_WR_CH  10
#define DAC_STATE_ERROR      15

// DAC command codes (3 MSB of command word)
#define DAC_CMD_NO_OP     0
#define DAC_CMD_SET_CAL   1
#define DAC_CMD_DAC_WR    2
#define DAC_CMD_DAC_WR_CH 3
#define DAC_CMD_GET_CAL   4
#define DAC_CMD_ZERO      5
#define DAC_CMD_CANCEL    7

// DAC command bits
#define DAC_CMD_CMD_LSB  29
#define DAC_CMD_TRIG_BIT 28
#define DAC_CMD_CONT_BIT 27
#define DAC_CMD_LDAC_BIT 26

// DAC data codes
#define DAC_DATA_CODE(word)       (((word) >> 28) & 0x0F) // Top 4 bits for debug code
#define DAC_DBG_MISO_DATA         1
#define DAC_DBG_STATE_TRANSITION  2
#define DAC_DBG_N_CS_TIMER        3
#define DAC_DBG_SPI_BIT           4
#define DAC_CAL_DATA              8
#define DAC_CAL_DATA_CH(word)     (((word) >> 16) & 0x07) // Bits [18:16] for channel in cal data
#define DAC_CAL_DATA_VAL(word)    ((int16_t)((word) & 0xFFFF)) // Bits [15:0] for cal value (signed)

//////////////////////////////////////////////////////////////////

// DAC control structure
struct dac_ctrl_t {
  volatile uint32_t *buffer[8];  // DAC FIFO (command and data)
};

// Function declarations
// Create DAC control structure
struct dac_ctrl_t create_dac_ctrl(bool verbose);
// Read DAC data from a specific board
uint32_t dac_read_data(struct dac_ctrl_t *dac_ctrl, uint8_t board);
// Interpret and print DAC data word as calibration or debug information
void dac_print_data(uint32_t dac_value);
// Interpret and print the DAC state
void dac_print_state(uint8_t state_code);

// DAC command word functions
void dac_cmd_noop(struct dac_ctrl_t *dac_ctrl, uint8_t board, bool trig, bool cont, bool ldac, uint32_t value, bool verbose);
void dac_cmd_dac_wr(struct dac_ctrl_t *dac_ctrl, uint8_t board, int16_t ch_vals[8], bool trig, bool cont, bool ldac, uint32_t value, bool verbose);
void dac_cmd_dac_wr_ch(struct dac_ctrl_t *dac_ctrl, uint8_t board, uint8_t ch, int16_t ch_val, bool verbose);
void dac_cmd_set_cal(struct dac_ctrl_t *dac_ctrl, uint8_t board, uint8_t channel, int16_t offset, bool verbose);
void dac_cmd_get_cal(struct dac_ctrl_t *dac_ctrl, uint8_t board, uint8_t channel, bool verbose);
void dac_cmd_zero(struct dac_ctrl_t *dac_ctrl, uint8_t board, bool verbose);
void dac_cmd_cancel(struct dac_ctrl_t *dac_ctrl, uint8_t board, bool verbose);

#endif // DAC_CTRL_H
