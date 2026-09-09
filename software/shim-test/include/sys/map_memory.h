#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdbool.h>
#include <stdint.h>

// Ceiling on the number of boards (64 channels at 8 per board). Per-board arrays
// are sized to this; the count actually present is discovered at runtime.
#define MAX_BOARDS 8

// Map a pl-reg register window by its /dev node. Opens dev_path (published
// world-rw by pl-reg-shim, so no root) and mmaps its single page-sized window at
// offset 0. Returns the mapping, or NULL on failure. Replaces the old /dev/mem +
// physical-address path: userspace opens registers by name, not by address.
uint32_t *map_pl_reg(const char *dev_path, bool verbose);

// Number of boards the running bitstream instantiated, found by probing which
// /dev/dac_fifo_<n> nodes pl-reg-shim published. Boards are contiguous 0..N-1;
// computed once and cached.
int board_count(void);

#endif // MAP_MEMORY_H
