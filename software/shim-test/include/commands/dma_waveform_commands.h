#ifndef DMA_WAVEFORM_COMMANDS_H
#define DMA_WAVEFORM_COMMANDS_H

#include "command_helper.h"

// Multi-word DMA waveform bench test (single board, 8 channels). The load command arms the
// DMA datapath and starts a background collector thread; a manual trigger (force_trig, one
// or more) then starts the run. The collector drains the S2MM capture into <outfile>_adc.csv
// as it arrives -- flushing continuously so data survives a later crash -- until it has the
// expected number of samples or is stopped. There is no timeout; a run may wait arbitrarily
// long for its trigger(s).
//
//   dma_waveform_test [board] [buf_KB] [carrier_Hz] [env_Hz] [amp_A] [extra_ms] [outfile] [--def] [--no_reset]
//     Synthesize a per-channel triangle * per-channel-phase envelope at the matched DAC/ADC
//     max cadence, write the intended waveform to <outfile>_dac.csv in amps, prebuffer it and
//     arm capture, write the PIO ADC read command, and start the collector. Any omitted
//     argument is prompted (Enter accepts the printed default); --def takes every default.
//   dma_waveform_status   Show DAC/ADC command counts and capture progress (read / expected).
//   dma_waveform_stop     Stop the collector (writing whatever was captured) and disarm.
int cmd_dma_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dma_waveform_status(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dma_waveform_stop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // DMA_WAVEFORM_COMMANDS_H
