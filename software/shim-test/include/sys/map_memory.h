#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdint.h>

// Function declaration for mapping 32-bit memory regions
uint32_t *map_32bit_memory(uint32_t base_addr, uint32_t size, char *name, int verbose);

// Function declarations for signed/offset conversion
int16_t offset_to_signed(uint16_t val);
uint16_t signed_to_offset(int16_t val);

#endif // MAP_MEMORY_H
