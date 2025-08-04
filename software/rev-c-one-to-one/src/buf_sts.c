#include <stdio.h>
#include <stdlib.h>
#include "buf_sts.h"
#include "map_memory.h"

// Function to create buffer status structure
struct buf_sts_t create_buf_sts(bool verbose) {
  struct buf_sts_t buf_sts;
  
  // Map FIFO unavailable status register
  buf_sts.fifo_unavailable = map_32bit_memory(FIFO_UNAVAILABLE, FIFO_UNAVAILABLE_WORDCOUNT, "FIFO Unavailable Status", verbose);
  if (buf_sts.fifo_unavailable == NULL) {
    fprintf(stderr, "Failed to map FIFO unavailable status memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  return buf_sts;
}
