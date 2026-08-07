#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Map a named shim-regs device into userspace as a volatile 32-bit word array.
// Opens `dev_path` (e.g. "/dev/axi_sys_ctrl"), mmaps `wordcount` words at
// offset 0, and returns a pointer to the mapped region. The caller never
// touches a physical address. Returns NULL on failure.
volatile uint32_t *map_32bit_memory(const char *dev_path, size_t wordcount, const char *name, bool verbose);

#endif // MAP_MEMORY_H
