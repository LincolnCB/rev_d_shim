#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdbool.h>
#include <stdint.h>

// Map a pl-reg register window by its /dev node. Opens dev_path (published
// world-rw by pl-reg-shim, so no root) and mmaps its single page-sized window at
// offset 0. Returns the mapping, or NULL on failure. Replaces the old /dev/mem +
// physical-address path: userspace opens registers by name, not by address.
uint32_t *map_pl_reg(const char *dev_path, bool verbose);

#endif // MAP_MEMORY_H
