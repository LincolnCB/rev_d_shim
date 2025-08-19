#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "spi_clk_ctrl.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

// Command processing structures and types
#define MAX_ARGS 3
#define MAX_FLAGS 2

typedef enum {
  FLAG_ALL,
  FLAG_VERBOSE
} command_flag_t;

typedef struct {
  const char* name;
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

// Command handler function declarations
int cmd_help(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_verbose(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_on(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_off(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_exit(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_boot_test_skip(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_adc_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_adc_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_trig_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_adc_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_dac_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_read_adc_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // COMMAND_HANDLER_H
