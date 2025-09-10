#ifndef SYSTEM_COMMANDS_H
#define SYSTEM_COMMANDS_H

#include "command_helper.h"

// Basic system commands
int cmd_verbose(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_on(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_off(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_hard_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_exit(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Configuration commands
int cmd_set_boot_test_skip(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// Control register commands
int cmd_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_set_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

// SPI polarity commands
int cmd_invert_mosi_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);
int cmd_invert_miso_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // SYSTEM_COMMANDS_H
