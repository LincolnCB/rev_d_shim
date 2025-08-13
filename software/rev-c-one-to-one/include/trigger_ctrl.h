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

//////////////////////////////////////////////////////////////////

// Trigger control structure
struct trigger_ctrl_t {
  volatile uint32_t *buffer; // Trigger FIFO (command and data)
};

// Function declarations
struct trigger_ctrl_t create_trigger_ctrl(bool verbose);

#endif // TRIGGER_CTRL_H
