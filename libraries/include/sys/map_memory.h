#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Ceiling on the number of boards (64 channels at 8 per board). Per-board arrays
// are sized to this; the count actually present is discovered at runtime.
#define MAX_BOARDS 8

// Per-board FIFO register window (pl-reg-shim, one /dev node per board). The DAC half
// sits at offset 0 and the ADC half 0x1000 into the two-page window, so a board occupies
// a single node that dac_ctrl (write) and adc_ctrl (read) both map.
#define BOARD_FIFO_DEV_FMT         "/dev/board_fifo_%d"
#define BOARD_FIFO_ADC_WORD_OFFSET (0x1000 / 4)  // ADC half, in 32-bit words

// Map a pl-reg register window by its /dev node. Opens dev_path (published
// world-rw by pl-reg-shim, so no root) and mmaps its single page-sized window at
// offset 0. Returns the mapping, or NULL on failure. Replaces the old /dev/mem +
// physical-address path: userspace opens registers by name, not by address.
uint32_t *map_pl_reg(const char *dev_path, bool verbose);

// Map a per-board FIFO node's full two-page window and return a pointer offset word_offset
// 32-bit words into it: 0 for the DAC half, BOARD_FIFO_ADC_WORD_OFFSET for the ADC half.
// Returns NULL on failure.
uint32_t *map_board_fifo(const char *dev_path, size_t word_offset, bool verbose);

// Number of boards the running bitstream instantiated, found by probing which
// /dev/board_fifo_<n> nodes pl-reg-shim published. Boards are contiguous 0..N-1;
// computed once and cached.
int board_count(void);

#endif // MAP_MEMORY_H
