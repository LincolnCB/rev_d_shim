#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "spi_clk_ctrl.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

// Command processing structures and types
#define MAX_ARGS 16  // Increased to accommodate command + arguments + flags
#define MAX_FLAGS 5

typedef enum {
  FLAG_ALL,
  FLAG_VERBOSE,
  FLAG_CONTINUE,
  FLAG_SIMPLE
} command_flag_t;

typedef struct {
  int min_args;      // Minimum required arguments (not counting command name)
  int max_args;      // Maximum allowed arguments
  command_flag_t valid_flags[MAX_FLAGS];  // Valid flags for this command (-1 terminated)
  const char* description;
} command_info_t;

// Command context structure
typedef struct command_context {
  struct sys_ctrl_t* sys_ctrl;
  struct spi_clk_ctrl_t* spi_clk_ctrl;
  struct sys_sts_t* sys_sts;
  struct dac_ctrl_t* dac_ctrl;
  struct adc_ctrl_t* adc_ctrl;
  struct trigger_ctrl_t* trigger_ctrl;
  bool* verbose;
  bool* should_exit;
  pthread_t adc_stream_threads[8];      // Thread handles for ADC streaming
  bool adc_stream_running[8];           // Status of each ADC stream thread
  volatile bool adc_stream_stop[8];     // Stop signals for each ADC stream thread
  pthread_t dac_stream_threads[8];      // Thread handles for DAC streaming
  bool dac_stream_running[8];           // Status of each DAC stream thread
  volatile bool dac_stream_stop[8];     // Stop signals for each DAC stream thread
  FILE* log_file;                       // File handle for command logging
  bool logging_enabled;                 // Whether command logging is active
} command_context_t;

typedef int (*command_handler_t)(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Command table entry structure
typedef struct {
  const char* name;
  command_handler_t handler;
  command_info_t info;
} command_entry_t;

// Main command processing functions
int execute_command(const char* line, command_context_t* ctx);
void print_help(void);

// Utility functions
uint32_t parse_value(const char* str, char** endptr);
int parse_board_number(const char* str);
int has_flag(const command_flag_t* flags, int flag_count, command_flag_t target_flag);

// Command parsing functions
int parse_command_line(const char* line, const char** args, int* arg_count, command_flag_t* flags, int* flag_count);
command_entry_t* find_command(const char* name);

//// ---- Command handler function declarations
// Basic system commands (no arguments)
int cmd_help(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                // Show help message
int cmd_verbose(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Toggle verbose mode
int cmd_on(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                  // Turn the system on
int cmd_off(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                 // Turn the system off
int cmd_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                 // Show hardware manager status
int cmd_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                 // Show debug registers
int cmd_hard_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);          // Perform hard reset sequence
int cmd_exit(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                // Exit the program

// Configuration commands (require 1 value argument)
int cmd_set_boot_test_skip(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);  // Set boot test skip register
int cmd_set_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Set debug register
int cmd_set_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);   // Set command buffer reset register
int cmd_set_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);  // Set data buffer reset register

// SPI polarity commands (no arguments)
int cmd_invert_mosi_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Invert MOSI SCK polarity
int cmd_invert_miso_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Invert MISO SCK polarity

// FIFO status commands (require 1 board number argument)
int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);    // Show DAC command FIFO status
int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);   // Show DAC data FIFO status
int cmd_adc_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);    // Show ADC command FIFO status
int cmd_adc_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);   // Show ADC data FIFO status

// Trigger FIFO status commands (no arguments)
int cmd_trig_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);   // Show trigger command FIFO status
int cmd_trig_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);  // Show trigger data FIFO status

// Data reading commands (require board number for DAC/ADC, support --all flag)
int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);        // Read raw DAC data sample(s)
int cmd_read_adc_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);        // Read raw ADC data sample(s)

// Trigger data reading commands (no arguments, support --all flag)
int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);       // Read trigger data sample(s)

// Debug reading commands (require board number, support --all flag)
int cmd_read_dac_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);         // Read and print debug info for DAC data
int cmd_read_adc_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);         // Read and print debug info for ADC data

// Trigger command functions (no arguments)
int cmd_trig_sync_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);         // Send trigger synchronize channels command
int cmd_trig_force_trig(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);      // Send trigger force trigger command
int cmd_trig_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);          // Send trigger cancel command

// Trigger command functions (require 1 value argument with range validation)
int cmd_trig_set_lockout(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Send trigger set lockout command with cycles (1 - 0x1FFFFFFF)
int cmd_trig_delay(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Send trigger delay command with cycles (0 - 0x1FFFFFFF)
int cmd_trig_expect_ext(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);      // Send trigger expect external command with count (0 - 0x1FFFFFFF)

// DAC and ADC command functions (require board, trig_mode, value arguments)
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Send DAC no-op command with board, trig mode, and value
int cmd_adc_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Send ADC no-op command with board, trig mode, and value

// DAC and ADC cancel command functions (require board number)
int cmd_dac_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Send DAC cancel command to specified board
int cmd_adc_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Send ADC cancel command to specified board

// DAC write command functions (require board, 8 channel values, trigger mode, and value)
int cmd_write_dac_update(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Send DAC write update command with 8 channel values

// ADC channel order command functions (require board and 8 channel order values)
int cmd_adc_set_ord(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Set ADC channel order for specified board

// ADC simple read command functions (require board and loop count)
int cmd_adc_simple_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);        // Perform simple ADC reads with specified board and loop count
int cmd_adc_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);               // Perform ADC read using loop command

// ADC file output command functions (require board and file path, support --all flag)
int cmd_read_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);        // Read ADC data and write to file with conversion

// ADC streaming command functions (require board and file path)
int cmd_stream_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);       // Start ADC streaming to file
int cmd_stop_adc_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);          // Stop ADC streaming for specified board

// DAC streaming command functions (require board and file path)
int cmd_stream_dac_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Start DAC streaming from waveform file
int cmd_stop_dac_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);          // Stop DAC streaming for specified board

// Command logging and playback functions (require file path)
int cmd_log_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Start logging commands to file
int cmd_stop_log(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);                // Stop logging commands
int cmd_load_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);            // Load and execute commands from file

// Single channel commands (require channel 0-63)
int cmd_do_dac_wr_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Write DAC single channel
int cmd_do_adc_rd_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Read ADC single channel  
int cmd_read_adc_single(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);          // Read single ADC channel data
int cmd_set_and_check(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);           // Set DAC and check ADC with buffer validation

// New test commands
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);             // Set and check current on individual channels
int cmd_stream_adc_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);     // Start ADC streaming from command file  
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);            // Load, run, and log waveforms


#endif // COMMAND_HANDLER_H
