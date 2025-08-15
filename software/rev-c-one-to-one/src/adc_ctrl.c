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
    exit(EXIT_FAILURE);
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
