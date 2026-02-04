#ifndef DAC_COMMANDS_H
#define DAC_COMMANDS_H

#include "command_helper.h"

// Enum for DAC command types
typedef enum {
  DAC_DELAY_CMD,         // Delay-based DAC write command
  DAC_TRIGGER_CMD,       // Trigger-based DAC write command
  DAC_NOOP_TRIGGER_CMD,  // No-op with trigger mode
  DAC_NOOP_DELAY_CMD     // No-op with delay mode
} dac_command_type_t;

// Structure to hold a parsed waveform command
typedef struct {
  dac_command_type_t type;  // Command type
  uint32_t value;           // Command value
  bool has_ch_vals;         // Whether channel values are present
  int16_t ch_vals[8];       // Channel values (if present)
  bool cont;                // Continue flag
} waveform_command_t;

// Structure to pass data to the DAC streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  waveform_command_t* commands;
  int command_count;
  int iterations;       // Number of times to iterate through the waveform
} dac_command_stream_params_t;

// Structure to pass data to the DAC debug streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
} dac_debug_stream_params_t;

// DAC FIFO status commands
int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC data reading commands
int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC command operations
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_do_dac_wr(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Single channel DAC operations
int cmd_do_dac_wr_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_get_dac_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_do_dac_get_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_dac_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_zero(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC command streaming operations (streaming commands from files)
int cmd_stream_dac_commands_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_dac_cmd_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// DAC debug streaming operations (streaming debug data to files)
int cmd_stream_dac_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_dac_debug_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Other DAC operations
int cmd_set_and_check(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // DAC_COMMANDS_H
