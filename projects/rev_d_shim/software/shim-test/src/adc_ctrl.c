#include <stdio.h>
#include <stdlib.h>
#include "adc_ctrl.h"
#include "map_memory.h"

// Create ADC control structure for a single board
struct adc_ctrl_t create_adc_ctrl(bool verbose) {
  struct adc_ctrl_t adc_ctrl;

  // Map ADC FIFO for each board
  for (int board = 0; board < 8; board++) {
    adc_ctrl.buffer[board] = map_32bit_memory(ADC_FIFO(board), 1, "ADC FIFO", verbose);
    if (adc_ctrl.buffer[board] == NULL) {
      fprintf(stderr, "Failed to map ADC FIFO access for board %d\n", board);
      exit(EXIT_FAILURE);
    }
  }

  return adc_ctrl;
}

// Read ADC value from a specific board
uint32_t adc_read(struct adc_ctrl_t *adc_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return 0; // Return 0 for invalid board
  }

  return *(adc_ctrl->buffer[board]);
}

// Interpret and print ADC value as debug information
void adc_print_debug(uint32_t adc_value) {
  uint8_t debug_code = ADC_DBG(adc_value);
  switch (debug_code) {
    case ADC_DBG_MISO_DATA:
      printf("Debug: MISO Data = 0x%04X\n", adc_value & 0xFFFF);
      break;
    case ADC_DBG_STATE_TRANSITION: {
      uint8_t from_state = (adc_value >> 4) & 0x0F;
      uint8_t to_state = adc_value & 0x0F;
      printf("Debug: State Transition from ");
      adc_print_state(from_state);
      printf(" to ");
      adc_print_state(to_state);
      printf("\n");
      break;
    }
    case ADC_DBG_N_CS_TIMER:
      printf("Debug: n_cs Timer = %d\n", adc_value & 0x0FFF);
      break;
    case ADC_DBG_SPI_BIT:
      printf("Debug: SPI Bit Counter = %d\n", adc_value & 0x1F);
      break;
    default:
      printf("Debug: Unknown code %d with value 0x%X\n", debug_code, adc_value);
      break;
  }
}

// Interpret and print the ADC state
void adc_print_state(uint8_t state_code) {
  switch (state_code) {
    case ADC_STATE_RESET:
      printf("RESET");
      break;
    case ADC_STATE_INIT:
      printf("INIT");
      break;
    case ADC_STATE_TEST_WR:
      printf("TEST Write");
      break;
    case ADC_STATE_REQ_RD:
      printf("Request Read");
      break;
    case ADC_STATE_TEST_RD:
      printf("TEST Read");
      break;
    case ADC_STATE_IDLE:
      printf("IDLE");
      break;
    case ADC_STATE_DELAY:
      printf("DELAY");
      break;
    case ADC_STATE_TRIG_WAIT:
      printf("Trigger Wait");
      break;
    case ADC_STATE_ADC_RD:
      printf("ADC Read");
      break;
    case ADC_STATE_ERROR:
      printf("ERROR");
      break;
    default:
      printf("Unknown State: %d", state_code);
  }
}

// ADC command word functions
void adc_cmd_noop(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (value > 0x3FFFFFF) {
    fprintf(stderr, "Invalid command value: %u. Must be 0 to 67108863 (26-bit value).\n", value);
    return;
  }
  uint32_t cmd_word = (ADC_CMD_NO_OP  << ADC_CMD_CMD_LSB ) |
                      ((trig ? 1 : 0) << ADC_CMD_TRIG_BIT) |
                      ((cont ? 1 : 0) << ADC_CMD_CONT_BIT) |
                      (value & 0x3FFFFFF);
  
  if (verbose) {
    printf("ADC[%d] NO_OP command word: 0x%08X\n", board, cmd_word);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}

void adc_cmd_adc_rd(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (value > 0x3FFFFFF) {
    fprintf(stderr, "Invalid command value: %u. Must be 0 to 67108863 (26-bit value).\n", value);
    return;
  }
  uint32_t cmd_word = (ADC_CMD_ADC_RD << ADC_CMD_CMD_LSB ) |
                      ((trig ? 1 : 0) << ADC_CMD_TRIG_BIT) |
                      ((cont ? 1 : 0) << ADC_CMD_CONT_BIT) |
                      (value & 0x3FFFFFF);
  
  if (verbose) {
    printf("ADC[%d] ADC_RD command word: 0x%08X\n", board, cmd_word);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}void adc_cmd_set_ord(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t channel_order[8], bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  // Validate channel order values (duplicates are okay, just must be 0-7)
  for (int i = 0; i < 8; i++) {
    if (channel_order[i] > 7) {
      fprintf(stderr, "Invalid channel index in order: %d. Must be 0-7.\n", channel_order[i]);
      return;
    }
  }

  uint32_t cmd_word = (ADC_CMD_SET_ORD << ADC_CMD_CMD_LSB) |
                      ((channel_order[7] & 0x7) << 21    ) |
                      ((channel_order[6] & 0x7) << 18    ) |
                      ((channel_order[5] & 0x7) << 15    ) |
                      ((channel_order[4] & 0x7) << 12    ) |
                      ((channel_order[3] & 0x7) <<  9    ) |
                      ((channel_order[2] & 0x7) <<  6    ) |
                      ((channel_order[1] & 0x7) <<  3    ) |
                      ((channel_order[0] & 0x7) <<  0    );

  if (verbose) {
    printf("ADC[%d] SET_ORD command word: 0x%08X (order: [%d,%d,%d,%d,%d,%d,%d,%d])\n", 
           board, cmd_word, channel_order[0], channel_order[1], channel_order[2], channel_order[3],
           channel_order[4], channel_order[5], channel_order[6], channel_order[7]);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}

void adc_cmd_cancel(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  
  uint32_t cmd_word = (ADC_CMD_CANCEL << ADC_CMD_CMD_LSB);
  
  if (verbose) {
    printf("ADC[%d] CANCEL command word: 0x%08X\n", board, cmd_word);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}

void adc_cmd_loop_next(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint32_t loop_count, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (loop_count > 0x3FFFFFF) {
    fprintf(stderr, "Invalid loop count: %u. Must be 0 to 67108863 (26-bit value).\n", loop_count);
    return;
  }
  
  uint32_t cmd_word = (ADC_CMD_LOOP << ADC_CMD_CMD_LSB) |
                      (loop_count & 0x3FFFFFF);
  
  if (verbose) {
    printf("ADC[%d] LOOP command word: 0x%08X (loop count: %u)\n", board, cmd_word, loop_count);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}
