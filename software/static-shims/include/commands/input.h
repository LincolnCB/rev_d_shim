#ifndef COMMANDS_INPUT_H
#define COMMANDS_INPUT_H

#include <stddef.h>
#include "commands.h"

#define INPUT_LINE_MAX 512

typedef enum {
	INPUT_PARSE_OK = 0,
	INPUT_PARSE_EMPTY,
	INPUT_PARSE_ERROR
} input_parse_result_t;

input_parse_result_t input_parse_line(const char *line,
																			parsed_command_t *out,
																	    const shim_runtime_state_t *runtime_state,
																			char *error_buf,
																			size_t error_buf_size);

#endif // COMMANDS_INPUT_H
