#ifndef TRIGGER_COMMANDS_H
#define TRIGGER_COMMANDS_H

#include "command_helper.h"

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

#endif // TRIGGER_COMMANDS_H
