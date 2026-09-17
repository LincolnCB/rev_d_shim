#ifndef DMA_COMMANDS_H
#define DMA_COMMANDS_H

#include "command_helper.h"

// Set a board's datapath_mode bit (0 = PIO, 1 = DMA). Locked while the system runs, so
// it must be set while the system is off (before ctrl_on).
int cmd_dma_mode(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
// Show datapath_mode plus the MCDMA and DMA-lane FIFO status for a board (or all boards).
int cmd_dma_status(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
// DMA round-trip on one channel: MM2S a DAC write, PIO-trigger an ADC read, S2MM the sample.
int cmd_dma_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // DMA_COMMANDS_H
