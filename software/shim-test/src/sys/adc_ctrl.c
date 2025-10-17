#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

// Read ADC sample pair word from a specific board
uint32_t adc_read_word(struct adc_ctrl_t *adc_ctrl, uint8_t board) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return 0; // Return 0 for invalid board
  }
  
  // Additional safety check for null pointer
  if (adc_ctrl->buffer[board] == NULL) {
    fprintf(stderr, "Error: ADC buffer[%d] is NULL. Cannot read data.\n", board);
    return 0;
  }
  
  // Use volatile access to prevent compiler optimization and ensure actual memory read
  volatile uint32_t *buffer_ptr = adc_ctrl->buffer[board];
  uint32_t value = *buffer_ptr;
  
  return value;
}

// Interpret and format ADC value as debug information
char* adc_format_debug(uint32_t adc_value, bool verbose) {
  static char buffer[512];  // Static buffer for return string
  char temp_buffer[256];
  buffer[0] = '\0';  // Initialize as empty string
  
  if (verbose) {
    snprintf(temp_buffer, sizeof(temp_buffer), "ADC Debug word: 0x%08X\n", adc_value);
    strcat(buffer, temp_buffer);
  }
  
  uint8_t debug_code = ADC_DBG(adc_value);
  switch (debug_code) {
    case ADC_DBG_MISO_DATA: {
      snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Startup test MISO Data = 0x%04X", adc_value & 0xFFFF);
      strcat(buffer, temp_buffer);
      break;
    }
    case ADC_DBG_STATE_TRANSITION: {
      uint8_t from_state = (adc_value >> 4) & 0x0F;
      uint8_t to_state = adc_value & 0x0F;
      char from_state_str[64];  // Local copy to avoid static buffer overwrite
      strcpy(from_state_str, adc_format_state(from_state, verbose));  // Copy the first result
      char to_state_str[64];  // Local copy to avoid static buffer overwrite
      strcpy(to_state_str, adc_format_state(to_state, verbose));  // Copy the second
      snprintf(temp_buffer, sizeof(temp_buffer), "Debug: State Transition from %s to %s", from_state_str, to_state_str);
      strcat(buffer, temp_buffer);
      break;
    }
    case ADC_DBG_REPEAT: {
      char *cmd_str = adc_format_command((uint8_t)(ADC_DBG_COMMAND(adc_value)), verbose);
      if (ADC_DBG_REPEAT_BIT(adc_value)) {
        snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Repeat (Count = %d) command %s with Repeat Bit set", 
                 adc_value & 0xFFFF, cmd_str);
      } else {
        snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Repeat (Count = %d) command %s with Repeat Bit clear", 
                 adc_value & 0xFFFF, cmd_str);
      }
      strcat(buffer, temp_buffer);
      break;
    }
    case ADC_DBG_N_CS_TIMER: {
      snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Starting ~CS timer with value = %d", adc_value & 0x0FFF);
      strcat(buffer, temp_buffer);
      break;
    }
    case ADC_DBG_SPI_BIT: {
      snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Starting SPI command at bit = %d", adc_value & 0x1F);
      strcat(buffer, temp_buffer);
      break;
    }
    case ADC_DBG_CMD_DONE: {
      if (ADC_DBG_NEXT_CMD_READY(adc_value)) {
        char *cmd_str = adc_format_command((uint8_t)(ADC_DBG_COMMAND(adc_value)), verbose);
        snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Command Done with next command ready: %s -- Remaining repeat count: %d", 
                 cmd_str, adc_value & 0xFFFF);
      } else {
        snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Command Done with no next command ready -- Remaining repeat count: %d", 
                 adc_value & 0xFFFF);
      }
      strcat(buffer, temp_buffer);
      break;
    }
    default: {
      snprintf(temp_buffer, sizeof(temp_buffer), "Debug: Unknown code %d with value 0x%X", debug_code, adc_value);
      strcat(buffer, temp_buffer);
      break;
    }
  }
  
  return buffer;
}

// Interpret and format the ADC state
char* adc_format_state(uint8_t state_code, bool verbose) {
  static char buffer[64];  // Static buffer for return string
  buffer[0] = '\0';  // Initialize as empty string
  
  if (verbose) {
    char temp[32];
    snprintf(temp, sizeof(temp), "ADC State code: %d\n", state_code);
    strcat(buffer, temp);
  }
  
  switch (state_code) {
    case ADC_STATE_RESET:
      strcat(buffer, "RESET");
      break;
    case ADC_STATE_INIT:
      strcat(buffer, "Init");
      break;
    case ADC_STATE_TEST_WR:
      strcat(buffer, "Test Write");
      break;
    case ADC_STATE_REQ_RD:
      strcat(buffer, "Request Read");
      break;
    case ADC_STATE_TEST_RD:
      strcat(buffer, "Test Read");
      break;
    case ADC_STATE_IDLE:
      strcat(buffer, "Idle");
      break;
    case ADC_STATE_DELAY:
      strcat(buffer, "Delay Wait");
      break;
    case ADC_STATE_TRIG_WAIT:
      strcat(buffer, "Trigger Wait");
      break;
    case ADC_STATE_ADC_RD:
      strcat(buffer, "ADC Read");
      break;
    case ADC_STATE_ADC_RD_CH:
      strcat(buffer, "ADC Read Channel");
      break;
    case ADC_STATE_ERROR:
      strcat(buffer, "ERROR");
      break;
    default: {
      char temp[32];
      snprintf(temp, sizeof(temp), "Unknown State: %d", state_code);
      strcat(buffer, temp);
      break;
    }
  }
  
  return buffer;
}

// Interpret and format uint8_t command code as string
char* adc_format_command(uint8_t cmd_code, bool verbose) {
  static char buffer[64];  // Static buffer for return string
  buffer[0] = '\0';  // Initialize as empty string
  
  if (verbose) {
    char temp[32];
    snprintf(temp, sizeof(temp), "ADC Command code: %d\n", cmd_code);
    strcat(buffer, temp);
  }
  
  switch (cmd_code) {
    case ADC_CMD_NO_OP: {
      strcat(buffer, "NO_OP");
      break;
    }
    case ADC_CMD_SET_ORD: {
      strcat(buffer, "SET_ORD");
      break;
    }
    case ADC_CMD_ADC_RD: {
      strcat(buffer, "ADC_RD");
      break;
    }
    case ADC_CMD_ADC_RD_CH: {
      strcat(buffer, "ADC_RD_CH");
      break;
    }
    case ADC_CMD_CANCEL: {
      strcat(buffer, "CANCEL");
      break;
    }
    default: {
      char temp[32];
      snprintf(temp, sizeof(temp), "Unknown Command: %d", cmd_code);
      strcat(buffer, temp);
      break;
    }
  }
  
  return buffer;
}

// ADC command word functions
void adc_cmd_noop(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (value > 0x1FFFFFF) {
    fprintf(stderr, "Invalid command value: %u. Must be 0 to 33554431 (25-bit value).\n", value);
    return;
  }
  uint32_t cmd_word = (ADC_CMD_NO_OP  << ADC_CMD_CMD_LSB ) |
                      ((trig ? 1 : 0) << ADC_CMD_TRIG_BIT) |
                      ((cont ? 1 : 0) << ADC_CMD_CONT_BIT) |
                      (value & 0x1FFFFFF);
  
  if (verbose) {
    printf("ADC[%d] NO_OP command word: 0x%08X\n", board, cmd_word);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;
}

void adc_cmd_adc_rd(struct adc_ctrl_t *adc_ctrl, uint8_t board, bool trig, bool cont, uint32_t value, uint32_t repeat_count, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (value > 0x1FFFFFF) {
    fprintf(stderr, "Invalid command value: %u. Must be 0 to 33554431 (25-bit value).\n", value);
    return;
  }
  uint32_t cmd_word = (ADC_CMD_ADC_RD << ADC_CMD_CMD_LSB ) |
                      ((trig ? 1 : 0) << ADC_CMD_TRIG_BIT) |
                      ((cont ? 1 : 0) << ADC_CMD_CONT_BIT) |
                      (((repeat_count > 0) ? 1 : 0) << ADC_CMD_REPEAT_BIT) |
                      (value & 0x1FFFFFF);
  
  if (verbose) {
    printf("ADC[%d] ADC_RD command word: 0x%08X\n", board, cmd_word);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;

  if (repeat_count > 0) {
    if (verbose) {
      printf("ADC[%d] REPEAT count: 0x%08X (repeat count: %u)\n", board, repeat_count, repeat_count);
    }
    *(adc_ctrl->buffer[board]) = repeat_count;
  }
}

void adc_cmd_adc_rd_ch(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t ch, uint32_t repeat_count, bool verbose) {
  if (board > 7) {
    fprintf(stderr, "Invalid ADC board: %d. Must be 0-7.\n", board);
    return;
  }
  if (ch > 7) {
    fprintf(stderr, "Invalid ADC channel: %d. Must be 0-7.\n", ch);
    return;
  }
  
  uint32_t cmd_word = (ADC_CMD_ADC_RD_CH << ADC_CMD_CMD_LSB ) |
                      (((repeat_count > 0) ? 1 : 0) << ADC_CMD_REPEAT_BIT) |
                      ((ch & 0x7) << 0);
  
  if (verbose) {
    printf("ADC[%d] ADC_RD_CH command word: 0x%08X (channel: %d)\n", board, cmd_word, ch);
  }
  *(adc_ctrl->buffer[board]) = cmd_word;

  if (repeat_count > 0) {
    if (verbose) {
      printf("ADC[%d] REPEAT count: 0x%08X (repeat count: %u)\n", board, repeat_count, repeat_count);
    }
    *(adc_ctrl->buffer[board]) = repeat_count;
  }
}

void adc_cmd_set_ord(struct adc_ctrl_t *adc_ctrl, uint8_t board, uint8_t channel_order[8], bool verbose) {
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

// Convert and format a single ADC sample from a 32-bit word (low 16 bits)
char* adc_format_single(uint32_t data_word, bool verbose) {
  static char buffer[64];  // Static buffer for return string
  char temp_buffer[32];
  buffer[0] = '\0';  // Initialize as empty string
  
  if (verbose) {
    snprintf(temp_buffer, sizeof(temp_buffer), "ADC Data word: 0x%08X\n", data_word);
    strcat(buffer, temp_buffer);
  }
  
  uint16_t lower_16 = data_word & 0xFFFF;
  int16_t signed_value = offset_to_signed(lower_16);
  snprintf(temp_buffer, sizeof(temp_buffer), "%d", signed_value);
  strcat(buffer, temp_buffer);
  
  return buffer;
}

// Convert and format a pair of ADC samples from a 32-bit word
char* adc_format_pair(uint32_t data_word, bool verbose) {
  static char buffer[64];  // Static buffer for return string
  char temp_buffer[32];
  buffer[0] = '\0';  // Initialize as empty string
  
  if (verbose) {
    snprintf(temp_buffer, sizeof(temp_buffer), "ADC Data word: 0x%08X\n", data_word);
    strcat(buffer, temp_buffer);
  }
  
  uint16_t lower_16 = data_word & 0xFFFF;
  uint16_t upper_16 = (data_word >> 16) & 0xFFFF;
  
  int16_t first_value = offset_to_signed(lower_16);
  int16_t second_value = offset_to_signed(upper_16);
  
  snprintf(temp_buffer, sizeof(temp_buffer), "%d, %d", first_value, second_value);
  strcat(buffer, temp_buffer);
  
  return buffer;
}
