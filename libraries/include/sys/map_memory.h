#ifndef MAP_MEMORY_H
#define MAP_MEMORY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Ceiling on the number of boards (64 channels at 8 per board). Per-board arrays
// are sized to this; the count actually present is read from sys_sts at runtime
// (board_count() in sys_sts.h).
#define MAX_BOARDS 8

// Board FIFO array register window (pl-reg-shim, one /dev node for every board). Each
// board owns one 4 KB page, and each FIFO lane has its own word inside the page (see
// cores/base/axi_board_fifo_array): a write pushes a command FIFO, a read pops a debug or
// data FIFO. Lane bit 2 picks DAC/ADC, bits 1:0 pick command/debug/data. Word 7 is the ADC
// capture rate lane, a plain register path rather than a FIFO.
#define BOARD_FIFO_DEV            "/dev/board_fifo"
#define BOARD_FIFO_PAGE_WORDS     (0x1000 / 4)
#define BOARD_FIFO_DAC_CMD_WORD   0  // write: DAC command (DMA-owned in DMA mode)
#define BOARD_FIFO_DAC_DEBUG_WORD 1  // read:  DAC debug
#define BOARD_FIFO_DAC_DATA_WORD  2  // read:  DAC calibration readback
#define BOARD_FIFO_ADC_CMD_WORD   4  // write: ADC command
#define BOARD_FIFO_ADC_DEBUG_WORD 5  // read:  ADC debug
#define BOARD_FIFO_ADC_DATA_WORD  6  // read:  ADC data    (DMA-owned in DMA mode)
#define BOARD_FIFO_ADC_RATE_WORD  7  // write: capture read delta; read: capture backlog

// Map a pl-reg register window by its /dev node. Opens dev_path (published
// world-rw by pl-reg-shim, so no root) and mmaps its single page-sized window at
// offset 0. Returns the mapping, or NULL on failure. Replaces the old /dev/mem +
// physical-address path: userspace opens registers by name, not by address.
uint32_t *map_pl_reg(const char *dev_path, bool verbose);

// Map the board FIFO array window (MAX_BOARDS pages) and return its base. The window is
// mapped once and shared, so dac_ctrl and adc_ctrl index the same mapping. Board b's lane
// word is base + b * BOARD_FIFO_PAGE_WORDS + BOARD_FIFO_*_WORD. Returns NULL on failure.
uint32_t *map_board_fifo(bool verbose);

#endif // MAP_MEMORY_H
