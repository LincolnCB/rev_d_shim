#ifndef DAC_COMMANDS_H
#define DAC_COMMANDS_H

#include "command_helper.h"

// Structure to hold a parsed waveform command
typedef struct {
  bool is_trigger;     // true for T, false for D
  uint32_t value;      // command value
  bool has_ch_vals;    // whether channel values are present
  int16_t ch_vals[8];  // channel values (if present)
  bool cont;           // continue flag
} waveform_command_t;

// Structure to pass data to the DAC streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  waveform_command_t* commands;
  int command_count;
  int loop_count;       // Number of times to loop through the waveform (1 = play once)
} dac_command_stream_params_t;

// DAC FIFO status commands
int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC data reading commands
int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_dac_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC command operations
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_write_dac_update(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Single channel DAC operations
int cmd_do_dac_wr_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Streaming operations (placeholders - need full implementation)
int cmd_stream_dac_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_dac_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Other DAC operations
int cmd_set_and_check(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // DAC_COMMANDS_H
