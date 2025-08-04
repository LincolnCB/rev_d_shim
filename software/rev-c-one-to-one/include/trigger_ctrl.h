#ifndef TRIGGER_CTRL_H
#define TRIGGER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Trigger Control Definitions ////////////////////
// Trigger FIFOs
#define TRIG_CMD_FIFO  (uint32_t) 0x80100000
#define TRIG_DATA_FIFO (uint32_t) 0x80101000
#define TRIG_FIFO_WORDCOUNT (uint32_t) 1024 // Size in 32-bit words

// Trigger status register offsets (within sys_sts register)
#define TRIG_CMD_FIFO_STS_OFFSET    (uint32_t) (3 * 8) + 1 // Trigger command FIFO status
#define TRIG_DATA_FIFO_STS_OFFSET   (uint32_t) (3 * 8) + 2 // Trigger data FIFO status

//////////////////////////////////////////////////////////////////

// Trigger control structure
struct trigger_ctrl_t {
  volatile uint32_t *cmd_fifo;  // Trigger command FIFO
  volatile uint32_t *data_fifo; // Trigger data FIFO
};

// Function declaration
struct trigger_ctrl_t create_trigger_ctrl(bool verbose);

#endif // TRIGGER_CTRL_H
