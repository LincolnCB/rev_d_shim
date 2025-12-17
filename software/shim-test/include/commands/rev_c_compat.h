#ifndef REV_C_COMPAT_H
#define REV_C_COMPAT_H

#include "command_helper.h"

// Rev C compatibility command - convert Rev C DAC files and stream to ADC output
int cmd_rev_c_compat(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx);

#endif // REV_C_COMPAT_H
