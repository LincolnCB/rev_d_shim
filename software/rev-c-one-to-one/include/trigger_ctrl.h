#ifndef TRIGGER_CTRL_H
#define TRIGGER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Trigger Control Definitions ////////////////////
// Trigger FIFO address
#define TRIG_FIFO(board)    (0x80100000 + (board) * 0x10000)

// Trigger FIFO depths
#define TRIG_CMD_FIFO_WORDCOUNT   (uint32_t) 1024 // Size in 32-bit words
#define TRIG_DATA_FIFO_WORDCOUNT  (uint32_t) 1024 // Size in 32-bit words

//////////////////////////////////////////////////////////////////

// Trigger control structure for a single board
struct trigger_ctrl_t {
  volatile uint32_t *buffer; // Trigger FIFO (command and data)
  uint8_t board_id;          // Board identifier (0-7)
};

// Trigger control structure for all boards
struct trigger_ctrl_array_t {
  struct trigger_ctrl_t boards[8]; // Array of trigger control structures for 8 boards
};

// Function declarations
struct trigger_ctrl_t create_trigger_ctrl(uint8_t board_id, bool verbose);
struct trigger_ctrl_array_t create_trigger_ctrl_array(bool verbose);

#endif // TRIGGER_CTRL_H
