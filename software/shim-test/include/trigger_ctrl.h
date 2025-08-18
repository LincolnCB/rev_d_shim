#ifndef TRIGGER_CTRL_H
#define TRIGGER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Trigger Control Definitions ////////////////////
// Trigger FIFO address
#define TRIG_FIFO                 (uint32_t) 0x80100000

// Trigger FIFO depths
#define TRIG_CMD_FIFO_WORDCOUNT   (uint32_t) 1024 // Size in 32-bit words
#define TRIG_DATA_FIFO_WORDCOUNT  (uint32_t) 1024 // Size in 32-bit words

// Trigger command codes (top 3 bits of command word)
#define TRIG_CMD_SYNC_CH      0x1  // Synchronize channels
#define TRIG_CMD_SET_LOCKOUT  0x2  // Set trigger lockout time
#define TRIG_CMD_EXPECT_EXT   0x3  // Expect external triggers
#define TRIG_CMD_DELAY        0x4  // Delay for specified cycles
#define TRIG_CMD_FORCE_TRIG   0x5  // Force trigger immediately
#define TRIG_CMD_CANCEL       0x7  // Cancel current operation

// Trigger command bits
#define TRIG_CMD_CODE_SHIFT   29   // Command code position
#define TRIG_CMD_VALUE_MASK   0x1FFFFFFF // Command value mask (lower 29 bits)

//////////////////////////////////////////////////////////////////

// Trigger control structure
struct trigger_ctrl_t {
  volatile uint32_t *buffer; // Trigger FIFO (command and data)
};

// Create trigger control structure
struct trigger_ctrl_t create_trigger_ctrl(bool verbose);

// Read 64-bit trigger data from FIFO as a pair of 32-bit words
uint64_t trigger_read(struct trigger_ctrl_t *trigger_ctrl);

// Trigger command functions
void trigger_cmd_sync_ch(struct trigger_ctrl_t *trigger_ctrl);
void trigger_cmd_set_lockout(struct trigger_ctrl_t *trigger_ctrl, uint32_t cycles);
void trigger_cmd_expect_ext(struct trigger_ctrl_t *trigger_ctrl, uint32_t count);
void trigger_cmd_delay(struct trigger_ctrl_t *trigger_ctrl, uint32_t cycles);
void trigger_cmd_force_trig(struct trigger_ctrl_t *trigger_ctrl);
void trigger_cmd_cancel(struct trigger_ctrl_t *trigger_ctrl);

#endif // TRIGGER_CTRL_H
