#ifndef BUF_STS_H
#define BUF_STS_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// Buffer Status Definitions ////////////////////
// FIFO unavailable register
#define FIFO_UNAVAILABLE           (uint32_t) 0x40110000
#define FIFO_UNAVAILABLE_WORDCOUNT (uint32_t) 1 // Size in 32-bit words

//////////////////////////////////////////////////////////////////

// Buffer status structure
struct buf_sts_t {
  volatile uint32_t *fifo_unavailable; // FIFO unavailable status register
};

// Function declaration
struct buf_sts_t create_buf_sts(bool verbose);

#endif // BUF_STS_H
