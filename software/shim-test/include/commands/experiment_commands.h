#ifndef EXPERIMENT_COMMANDS_H
#define EXPERIMENT_COMMANDS_H

#include "command_helper.h"

// Channel test command - set and check current on individual channels
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Channel calibration command - calibrate DAC/ADC channels
int cmd_channel_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Waveform test command - easily load, run, and log waveforms
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Rev C compatibility command - convert Rev C DAC files and stream to ADC output
int cmd_rev_c_compat(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Zero all DAC channels command - set all channels to 0 on all connected boards
int cmd_zero_all_dacs(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // EXPERIMENT_COMMANDS_H
