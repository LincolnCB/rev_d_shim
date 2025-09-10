#ifndef EXPERIMENT_COMMANDS_H
#define EXPERIMENT_COMMANDS_H

#include "command_helper.h"

// Channel test command - set and check current on individual channels
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Waveform test command - easily load, run, and log waveforms
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // EXPERIMENT_COMMANDS_H
