#ifndef ADC_COMMANDS_H
#define ADC_COMMANDS_H

#include "command_helper.h"

// Structure for ADC command data (used for streaming commands from file)
typedef struct {
  char type;              // Command type: 'L' (Loop), 'T' (Trigger), 'D' (Delay), 'O' (Order)
  uint32_t value;         // Command value (for L, T, D commands - loop count, trigger cycles, delay cycles)
  uint8_t order[8];       // Channel order array (for O commands - specifies sampling order 0-7)
} adc_command_t;

// Structure to pass data to the ADC data streaming thread (for reading ADC data to file)
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  uint64_t word_count;         // Number of words to read from ADC
  volatile bool* should_stop;
} adc_data_stream_params_t;

// Structure to pass data to the ADC command streaming thread (for streaming commands from file)
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  adc_command_t* commands;
  int command_count;
  int loop_count;       // Number of times to loop through the commands (1 = play once)
  bool simple_mode;     // Whether to unroll loops instead of using loop command
} adc_command_stream_params_t;

// ADC FIFO status commands
int cmd_adc_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_adc_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC data reading commands
int cmd_read_adc_pair(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_adc_single(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_adc_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC command operations
int cmd_adc_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_adc_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_adc_set_ord(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC reading operations
int cmd_do_adc_simple_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_do_adc_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_do_adc_rd_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC data streaming operations (reading ADC data to files)
int cmd_read_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stream_adc_data_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_adc_data_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// ADC command streaming operations (streaming commands from files)
int cmd_stream_adc_commands_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_adc_cmd_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // ADC_COMMANDS_H
