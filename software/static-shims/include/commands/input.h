#ifndef COMMANDS_INPUT_H
#define COMMANDS_INPUT_H

#include <stddef.h>
#include "commands.h"

// Max length for a single input line (for typical commands like "L filename" or "U 1.0 2.0 ...")
#define INPUT_LINE_MAX 256

// Array command ('A') buffer sizing:
// - INPUT_ARRAY_MAX_LINES: maximum number of data lines in an inline array
// - INPUT_ARRAY_CHARS_PER_CHANNEL: character width per channel value (e.g., "-5.1234")
#define INPUT_ARRAY_MAX_LINES 512
#define INPUT_ARRAY_CHARS_PER_CHANNEL 8

// Static buffer for array data: HW_MAX_CHANNELS * 8 chars + separators, * 512 lines
// Calculated as: ~9 bytes per channel (8 chars + 1 separator) * channels * lines + margin
#define INPUT_ARRAY_DATA_MAX ((HW_MAX_CHANNELS * INPUT_ARRAY_CHARS_PER_CHANNEL + HW_MAX_CHANNELS) * INPUT_ARRAY_MAX_LINES + 1024)

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
