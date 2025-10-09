#ifndef EXPERIMENT_COMMANDS_H
#define EXPERIMENT_COMMANDS_H

#include "command_helper.h"

// Channel test command - set and check current on individual channels
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Channel calibration command - calibrate DAC/ADC channels
int cmd_channel_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC bias calibration command - find and store ADC bias values for all connected channels
int cmd_find_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC bias management commands
int cmd_print_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_save_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_load_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Waveform test command - easily load, run, and log waveforms
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Rev C compatibility command - convert Rev C DAC files and stream to ADC output
int cmd_rev_c_compat(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Zero all DAC channels command - set all channels to 0 on all connected boards


// Fieldmap data collection command - automated field mapping with current sweep
int cmd_fieldmap(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Stop fieldmap data collection command
int cmd_stop_fieldmap(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Stop trigger monitor command
int cmd_stop_trigger_monitor(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Stop waveform test command - stops all streaming and monitoring
int cmd_stop_waveform(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // EXPERIMENT_COMMANDS_H
