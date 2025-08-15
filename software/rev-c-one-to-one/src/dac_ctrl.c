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
