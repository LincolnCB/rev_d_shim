#ifndef TRIGGER_COMMANDS_H
#define TRIGGER_COMMANDS_H

#include "command_helper.h"

// Structure to pass data to the trigger data streaming thread (for reading trigger timestamps to file)
typedef struct {
  command_context_t* ctx;
  char file_path[1024];
  uint64_t sample_count;           // Number of trigger samples to read (64-bit each)
  volatile bool* should_stop;
  bool binary_mode;                // true for binary format, false for ASCII format
} trigger_data_stream_params_t;

// Trigger FIFO status commands
int cmd_trig_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Trigger data reading commands
int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Basic trigger command operations
int cmd_trig_sync_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_force_trig(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Trigger command operations with value parameters
int cmd_trig_set_lockout(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_delay(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_expect_ext(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Trigger data streaming commands
int cmd_stream_trig_data_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_trig_data_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // TRIGGER_COMMANDS_H
