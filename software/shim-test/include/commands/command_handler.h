#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#include "sys_ctrl.h"
#include "sys_sts.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "trigger_ctrl.h"
#include "spi_clk_ctrl.h"

// Include command module headers in logical order
#include "command_helper.h"     // Shared utilities used by all modules
#include "system_commands.h"    // System control and configuration commands
#include "adc_commands.h"       // ADC data acquisition commands
#include "dac_commands.h"       // DAC waveform generation commands
#include "trigger_commands.h"   // Trigger and synchronization commands

// Command metadata for validation and help
typedef struct {
  int min_args;      // Minimum required arguments (not counting command name)
  int max_args;      // Maximum allowed arguments
  command_flag_t valid_flags[MAX_FLAGS];  // Valid flags for this command (-1 terminated)
  const char* description;
} command_info_t;

// Function pointer type for command handlers
typedef int (*command_handler_t)(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Command table entry structure
typedef struct {
  const char* name;
  command_handler_t handler;
  command_info_t info;
} command_entry_t;

// Main command execution and help functions
int execute_command(const char* line, command_context_t* ctx);
void print_help(void);
int cmd_help(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Command parsing and lookup functions
int parse_command_line(const char* line, const char** args, int* arg_count, command_flag_t* flags, int* flag_count);
command_entry_t* find_command(const char* name);

// Command logging utilities
void log_command_if_enabled(command_context_t* ctx, const char* command_line);

// Command logging and loading commands
int cmd_log_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_stop_log(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_load_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // COMMAND_HANDLER_H
