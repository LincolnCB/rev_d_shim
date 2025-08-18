#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdint.h>

// Function declaration for mapping 32-bit memory regions
uint32_t *map_32bit_memory(uint32_t base_addr, uint32_t size, char *name, int verbose);

#endif // MAP_MEMORY_H
