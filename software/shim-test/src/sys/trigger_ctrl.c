#include <stdio.h>
#include <stdlib.h>
#include "trigger_ctrl.h"
#include "map_memory.h"

// Create trigger control structure
struct trigger_ctrl_t create_trigger_ctrl(bool verbose) {
  struct trigger_ctrl_t trigger_ctrl;
  
  // Map trigger command FIFO
  trigger_ctrl.buffer = map_32bit_memory(TRIG_FIFO, 1, "Trigger FIFO", verbose);
  if (trigger_ctrl.buffer == NULL) {
    fprintf(stderr, "Failed to map trigger FIFO access memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  return trigger_ctrl;
}

// Read 64-bit trigger data from FIFO as a pair of 32-bit words
uint64_t trigger_read(struct trigger_ctrl_t *trigger_ctrl) {
  uint32_t low_word = *(trigger_ctrl->buffer);
  uint32_t high_word = *(trigger_ctrl->buffer);
  return ((uint64_t)high_word << 32) | low_word; // Combine into 64-bit value
}

// Trigger command functions
void trigger_cmd_sync_ch(struct trigger_ctrl_t *trigger_ctrl, bool log, bool verbose) {
  uint32_t cmd_word = (TRIG_CMD_SYNC_CH << TRIG_CMD_CODE_SHIFT) |
                      ((log ? 1 : 0) << TRIG_CMD_LOG_BIT);
  
  if (verbose) {
    printf("  Writing trigger sync_ch command: 0x%08X (cmd=0x%X, log=%d)\n", 
           cmd_word, TRIG_CMD_SYNC_CH, log ? 1 : 0);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_set_lockout(struct trigger_ctrl_t *trigger_ctrl, uint32_t cycles, bool verbose) {
  if (cycles > 0x0FFFFFFF) {
    fprintf(stderr, "Lockout cycles out of range: %u (valid range: 0 - 268435455)\n", cycles);
    return;
  }
  
  uint32_t cmd_word = (TRIG_CMD_SET_LOCKOUT << TRIG_CMD_CODE_SHIFT) | (cycles & TRIG_CMD_VALUE_MASK);
  
  if (verbose) {
    printf("  Writing trigger set_lockout command: 0x%08X (cmd=0x%X, cycles=%u)\n", 
           cmd_word, TRIG_CMD_SET_LOCKOUT, cycles);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_expect_ext(struct trigger_ctrl_t *trigger_ctrl, uint32_t count, bool log, bool verbose) {
  if (count > 0x0FFFFFFF) {
    fprintf(stderr, "External trigger count out of range: %u (valid range: 0 - 268435455)\n", count);
    return;
  }
  
  uint32_t cmd_word = (TRIG_CMD_EXPECT_EXT << TRIG_CMD_CODE_SHIFT) |
                      ((log ? 1 : 0) << TRIG_CMD_LOG_BIT) |
                      (count & TRIG_CMD_VALUE_MASK);
  
  if (verbose) {
    printf("  Writing trigger expect_ext command: 0x%08X (cmd=0x%X, log=%d, count=%u)\n", 
           cmd_word, TRIG_CMD_EXPECT_EXT, log ? 1 : 0, count);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_delay(struct trigger_ctrl_t *trigger_ctrl, uint32_t cycles, bool verbose) {
  if (cycles > 0x0FFFFFFF) {
    fprintf(stderr, "Delay cycles out of range: %u (valid range: 0 - 268435455)\n", cycles);
    return;
  }
  
  uint32_t cmd_word = (TRIG_CMD_DELAY << TRIG_CMD_CODE_SHIFT) | (cycles & TRIG_CMD_VALUE_MASK);
  
  if (verbose) {
    printf("  Writing trigger delay command: 0x%08X (cmd=0x%X, cycles=%u)\n", 
           cmd_word, TRIG_CMD_DELAY, cycles);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_force_trig(struct trigger_ctrl_t *trigger_ctrl, bool log, bool verbose) {
  uint32_t cmd_word = (TRIG_CMD_FORCE_TRIG << TRIG_CMD_CODE_SHIFT) |
                      ((log ? 1 : 0) << TRIG_CMD_LOG_BIT);
  
  if (verbose) {
    printf("  Writing trigger force_trig command: 0x%08X (cmd=0x%X, log=%d)\n", 
           cmd_word, TRIG_CMD_FORCE_TRIG, log ? 1 : 0);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_cancel(struct trigger_ctrl_t *trigger_ctrl, bool verbose) {
  uint32_t cmd_word = (TRIG_CMD_CANCEL << TRIG_CMD_CODE_SHIFT);
  
  if (verbose) {
    printf("  Writing trigger cancel command: 0x%08X (cmd=0x%X)\n", 
           cmd_word, TRIG_CMD_CANCEL);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}

void trigger_cmd_reset_count(struct trigger_ctrl_t *trigger_ctrl, bool verbose) {
  uint32_t cmd_word = (TRIG_CMD_RESET_COUNT << TRIG_CMD_CODE_SHIFT);
  
  if (verbose) {
    printf("  Writing trigger reset_count command: 0x%08X (cmd=0x%X)\n", 
           cmd_word, TRIG_CMD_RESET_COUNT);
  }
  
  *(trigger_ctrl->buffer) = cmd_word;
}
  
