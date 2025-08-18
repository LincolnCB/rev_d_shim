#include <stdio.h>
#include <stdlib.h>
#include "dac_ctrl.h"
#include "map_memory.h"

// Create DAC control structure for all boards
struct dac_ctrl_t create_dac_ctrl(bool verbose) {
  struct dac_ctrl_t dac_ctrl;

  // Map DAC FIFO for each board
  for (int board = 0; board < 8; board++) {
    dac_ctrl.buffer[board] = map_32bit_memory(DAC_FIFO(board), 1, "DAC FIFO", verbose);
    if (dac_ctrl.buffer[board] == NULL) {
      fprintf(stderr, "Failed to map DAC FIFO access for board %d\n", board);
      exit(EXIT_FAILURE);
    }
  }

  return dac_ctrl;
}

// Read DAC value from a specific board
uint32_t dac_read(struct dac_ctrl_t *dac_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  return *(dac_ctrl->buffer[board]);
}

// Interpret and print DAC value as debug information
void dac_print_debug(uint32_t dac_value) {
  uint8_t debug_code = DAC_DBG(dac_value);
  switch (debug_code) {
    case DAC_DBG_MISO_DATA:
      printf("Debug: MISO Data = 0x%04X\n", dac_value & 0xFFFF);
      break;
    case DAC_DBG_STATE_TRANSITION: {
      uint8_t from_state = (dac_value >> 4) & 0x0F;
      uint8_t to_state = dac_value & 0x0F;
      printf("Debug: State Transition from ");
      dac_print_state(from_state);
      printf(" to ");
      dac_print_state(to_state);
      printf("\n");
      break;
    }
    case DAC_DBG_N_CS_TIMER:
      printf("Debug: n_cs Timer = %d\n", dac_value & 0x0FFF);
      break;
    case DAC_DBG_SPI_BIT:
      printf("Debug: SPI Bit Counter = %d\n", dac_value & 0x1F);
      break;
    default:
      printf("Debug: Unknown code %d with value 0x%X\n", debug_code, dac_value);
      break;
  }
}

// Interpret and print the DAC state
void dac_print_state(uint8_t state_code) {
  switch (state_code) {
    case DAC_STATE_RESET:
      printf("RESET");
      break;
    case DAC_STATE_INIT:
      printf("INIT");
      break;
    case DAC_STATE_TEST_WR:
      printf("TEST Write");
      break;
    case DAC_STATE_REQ_RD:
      printf("Request Read");
      break;
    case DAC_STATE_TEST_RD:
      printf("TEST Read");
      break;
    case DAC_STATE_IDLE:
      printf("IDLE");
      break;
    case DAC_STATE_DELAY:
      printf("DELAY");
      break;
    case DAC_STATE_TRIG_WAIT:
      printf("Trigger Wait");
      break;
    case DAC_STATE_DAC_WR:
      printf("DAC Write");
      break;
    case DAC_STATE_ERROR:
      printf("ERROR");
      break;
    default:
      printf("Unknown State: %d", state_code);
  }
}

// DAC command word functions
void dac_cmd_noop(struct dac_ctrl_t *dac_ctrl, uint8_t board, bool trig, bool cont, bool ldac, uint32_t delay) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  if (delay > 0x0FFFFFFF) {
    fprintf(stderr, "Invalid delay value: %u. Must be 0 to 268435455.\n", delay);
    exit(EXIT_FAILURE);
  }
  uint32_t cmd_word = (DAC_CMD_NO_OP << 30) |
                     ((trig ? 1 : 0) << DAC_CMD_TRIG_BIT) |
                     ((cont ? 1 : 0) << DAC_CMD_CONT_BIT) |
                     ((ldac ? 1 : 0) << DAC_CMD_LDAC_BIT) |
                     (delay & 0x0FFFFFFF);
  
  *(dac_ctrl->buffer[board]) = cmd_word;
}

void dac_cmd_dac_wr(struct dac_ctrl_t *dac_ctrl, uint8_t board, int16_t ch_vals[8], bool trig, bool cont, bool ldac, uint32_t delay) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  if (delay > 0x0FFFFFFF) {
    fprintf(stderr, "Invalid delay value: %u. Must be 0 to 268435455.\n", delay);
    exit(EXIT_FAILURE);
  }
  
  uint32_t cmd_word = (DAC_CMD_DAC_WR << 30) |
                     ((trig ? 1 : 0) << DAC_CMD_TRIG_BIT) |
                     ((cont ? 1 : 0) << DAC_CMD_CONT_BIT) |
                     ((ldac ? 1 : 0) << DAC_CMD_LDAC_BIT) |
                     (delay & 0x0FFFFFFF);
  
  *(dac_ctrl->buffer[board]) = cmd_word;

  // Write channel values
  for (int i = 0; i < 8; i += 2) {
    // Each word contains two channels: [31:16] = ch N+1, [15:0] = ch N
    uint16_t val0 = DAC_SIGNED_TO_OFFSET(ch_vals[i]);
    uint16_t val1 = DAC_SIGNED_TO_OFFSET(ch_vals[i + 1]);
    uint32_t word = ((uint32_t)val1 << 16) | val0;
    *(dac_ctrl->buffer[board]) = word;
  }
}

void dac_cmd_set_cal(struct dac_ctrl_t *dac_ctrl, uint8_t board, uint8_t channel, int16_t cal) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  if (channel > 7) {
    fprintf(stderr, "Invalid channel: %d. Must be 0-7.\n", channel);
    exit(EXIT_FAILURE);
  }

  uint32_t cmd_word = (DAC_CMD_SET_CAL << 30) |
                      (channel << 16) | // Channel index
                      ((uint16_t)cal & 0xFFFF);

  *(dac_ctrl->buffer[board]) = cmd_word;
}

void dac_cmd_cancel(struct dac_ctrl_t *dac_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid DAC board: %d. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  uint32_t cmd_word = (DAC_CMD_CANCEL << 30);
  *(dac_ctrl->buffer[board]) = cmd_word;
}
