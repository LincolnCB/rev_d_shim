#include "input.h"

#include <ctype.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  char *token;
  char *cursor;
  const shim_runtime_state_t *runtime_state;
  char *error_buf;
  size_t error_buf_size;
} parse_state_t;

typedef bool (*parse_fn_t)(parsed_command_t *out, parse_state_t *state);

// Compare one command character, case-insensitive for alphabetic characters.
static bool cmd_match(char value, char expected) {
  if (expected >= 'a' && expected <= 'z') {
    char upper = (char)(expected - ('a' - 'A'));
    return (value == expected || value == upper);
  }
  if (expected >= 'A' && expected <= 'Z') {
    char lower = (char)(expected + ('a' - 'A'));
    return (value == expected || value == lower);
  }
  return (value == expected);
}

// Trim leading/trailing ASCII whitespace in place.
static char *trim_whitespace(char *s) {
  while (*s != '\0' && isspace((unsigned char)*s)) {
    ++s;
  }

  if (*s == '\0') {
    return s;
  }

  char *end = s + strlen(s) - 1;
  while (end > s && isspace((unsigned char)*end)) {
    *end = '\0';
    --end;
  }
  return s;
}

// Get the next whitespace-delimited token from a mutable string cursor.
static char *next_token(char **cursor) {
  if (cursor == NULL || *cursor == NULL) {
    return NULL;
  }

  char *p = *cursor;
  while (*p != '\0' && isspace((unsigned char)*p)) {
    ++p;
  }
  if (*p == '\0') {
    *cursor = p;
    return NULL;
  }

  char *start = p;
  while (*p != '\0' && !isspace((unsigned char)*p)) {
    ++p;
  }

  if (*p != '\0') {
    *p = '\0';
    ++p;
  }

  *cursor = p;
  return start;
}

// Return the non-whitespace remainder of the input cursor.
static char *remaining_text(char *cursor) {
  if (cursor == NULL) {
    return NULL;
  }
  while (*cursor != '\0' && isspace((unsigned char)*cursor)) {
    ++cursor;
  }
  return (*cursor == '\0') ? NULL : cursor;
}

// Parse a base-10 uint32 from a full token.
static bool parse_u32(const char *text, uint32_t *value) {
  if (text == NULL || *text == '\0') {
    return false;
  }

  char *end = NULL;
  unsigned long v = strtoul(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }

  uint32_t parsed = (uint32_t)v;
  if ((unsigned long)parsed != v) {
    return false;
  }

  *value = parsed;
  return true;
}

// Parse a base-10 int from a full token.
static bool parse_int(const char *text, int *value) {
  if (text == NULL || *text == '\0') {
    return false;
  }

  char *end = NULL;
  long v = strtol(text, &end, 10);
  if (end == text || *end != '\0') {
    return false;
  }

  int parsed = (int)v;
  if ((long)parsed != v) {
    return false;
  }

  *value = parsed;
  return true;
}

// Parse a double from a full token.
static bool parse_double(const char *text, double *value) {
  if (text == NULL || *text == '\0') {
    return false;
  }

  char *end = NULL;
  double v = strtod(text, &end);
  if (end == text || *end != '\0' || v != v || v > DBL_MAX || v < -DBL_MAX) {
    return false;
  }

  *value = v;
  return true;
}

// Write a parser error message into the shared error buffer.
static void command_error(char *error_buf, size_t error_buf_size, const char *msg) {
  if (error_buf == NULL || error_buf_size == 0U) {
    return;
  }
  (void)snprintf(error_buf, error_buf_size, "%s", msg);
}

// Validate that no additional arguments are present.
static bool parse_expect_no_args(parse_state_t *state, const char *message) {
  if (next_token(&state->cursor) != NULL) {
    command_error(state->error_buf, state->error_buf_size, message);
  }
  return true;
}

//// -- General commands --

// Parse H or ? (help).
static bool parse_help(parsed_command_t *out, parse_state_t *state) {
  if (!((state->token[1] == '\0' && cmd_match(state->token[0], 'H')) ||
        (state->token[0] == '?' && state->token[1] == '\0'))) {
    return false;
  }
  out->type = CMD_HELP;
  return true;
}

// Parse Q (quit).
static bool parse_quit(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'Q'))) {
    return false;
  }
  out->type = CMD_QUIT;
  return true;
}

// Parse S (status).
static bool parse_status(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'S'))) {
    return false;
  }
  out->type = CMD_STATUS;
  return parse_expect_no_args(state, "S does not take arguments");
}

// Parse P (power on).
static bool parse_power_on(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'P'))) {
    return false;
  }
  out->type = CMD_POWER_ON;
  return parse_expect_no_args(state, "P does not take arguments");
}

// Parse X (hard reset).
static bool parse_hard_reset(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'X'))) {
    return false;
  }
  out->type = CMD_HARD_RESET;
  return parse_expect_no_args(state, "X does not take arguments");
}

// Parse Z (zero outputs).
static bool parse_zero(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'Z'))) {
    return false;
  }
  out->type = CMD_ZERO;
  return parse_expect_no_args(state, "Z does not take arguments");
}

// Parse I [file] (read currents and optional dump path).
static bool parse_read(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'I'))) {
    return false;
  }
  out->type = CMD_READ;
  char *rest = remaining_text(state->cursor);
  if (rest != NULL) {
    (void)snprintf(out->file_path, sizeof(out->file_path), "%s", rest);
  }
  return true;
}

//// -- Manual mode (no loaded file) --

// Parse manual set: n x (channel index, current in amps).
static bool parse_manual_set(parsed_command_t *out, parse_state_t *state) {
  int channel = -1;
  if (!parse_int(state->token, &channel)) {
    return false;
  }
  if (channel < 0 || (uint32_t)channel >= state->runtime_state->hw->channel_count) {
    command_error(state->error_buf, state->error_buf_size, "channel index out of range (valid: 0..ch_count-1)");
    return true;
  }

  char *amps_token = next_token(&state->cursor);
  double amps = 0.0;
  if (amps_token == NULL || !parse_double(amps_token, &amps)) {
    command_error(state->error_buf, state->error_buf_size, "manual set must be: n x");
    return true;
  }
  if (next_token(&state->cursor) != NULL) {
    command_error(state->error_buf, state->error_buf_size, "manual set accepts exactly two values: n x");
    return true;
  }

  out->type = CMD_SET_CHANNEL;
  out->channel = channel;
  out->amps = amps;
  return true;
}

// Parse U x1 x2 x3 ... (immediately update all channels, no loaded file).
// Requires exactly hw->channel_count amp values.
static bool parse_update(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'U'))) {
    return false;
  }
  out->type = CMD_UPDATE;

  uint32_t expected = state->runtime_state->hw->channel_count;
  uint32_t count = 0;
  char *value_token = NULL;

  while ((value_token = next_token(&state->cursor)) != NULL) {
    if (count >= expected || count >= HW_MAX_CHANNELS) {
      command_error(state->error_buf, state->error_buf_size, "U: too many values for channel count");
      return true;
    }
    double amps = 0.0;
    if (!parse_double(value_token, &amps)) {
      command_error(state->error_buf, state->error_buf_size, "U: all values must be numeric amps");
      return true;
    }
    out->update_amps[count] = amps;
    ++count;
  }

  if (count != expected) {
    command_error(state->error_buf, state->error_buf_size, "U requires exactly one value per channel");
    return true;
  }

  return true;
}

// Parse B x1 x2 x3 ... (buffer DAC commands for all channels, no loaded file).
// Requires exactly hw->channel_count amp values.
static bool parse_buffer(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'B'))) {
    return false;
  }
  out->type = CMD_BUFFER;

  uint32_t expected = state->runtime_state->hw->channel_count;
  uint32_t count = 0;
  char *value_token = NULL;

  while ((value_token = next_token(&state->cursor)) != NULL) {
    if (count >= expected || count >= HW_MAX_CHANNELS) {
      command_error(state->error_buf, state->error_buf_size, "B: too many values for channel count");
      return true;
    }
    double amps = 0.0;
    if (!parse_double(value_token, &amps)) {
      command_error(state->error_buf, state->error_buf_size, "B: all values must be numeric amps");
      return true;
    }
    out->update_amps[count] = amps;
    ++count;
  }

  if (count != expected) {
    command_error(state->error_buf, state->error_buf_size, "B requires exactly one value per channel");
    return true;
  }

  return true;
}


// Parse C (calibrate).
static bool parse_calibrate(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'C'))) {
    return false;
  }
  out->type = CMD_CALIBRATE;
  return parse_expect_no_args(state, "C does not take arguments");
}

// Parse D t (set trigger lockout time in ms, no loaded file).
static bool parse_lockout(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'D'))) {
    return false;
  }
  out->type = CMD_LOCKOUT;

  char *t_token = next_token(&state->cursor);
  double trigger_lockout_ms = 0.0;
  if (t_token == NULL || !parse_double(t_token, &trigger_lockout_ms)) {
    command_error(state->error_buf, state->error_buf_size, "D t requires a numeric time in ms");
    return true;
  }
  if (trigger_lockout_ms < 0.0) {
    command_error(state->error_buf, state->error_buf_size, "D t requires a non-negative time in ms");
    return true;
  }
  if (next_token(&state->cursor) != NULL) {
    command_error(state->error_buf, state->error_buf_size, "D accepts exactly one argument: t");
    return true;
  }

  out->trigger_lockout_ms = trigger_lockout_ms;
  return true;
}

// -- Loaded file mode (after L command) --

// Parse L [file] (load block file).
static bool parse_load(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'L'))) {
    return false;
  }
  out->type = CMD_LOAD;
  char *rest = remaining_text(state->cursor);
  if (rest != NULL) {
    (void)snprintf(out->file_path, sizeof(out->file_path), "%s", rest);
  }
  return true;
}

// Parse E (exit loaded file mode).
static bool parse_exit_file(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'E'))) {
    return false;
  }
  out->type = CMD_EXIT_FILE;
  return parse_expect_no_args(state, "E does not take arguments");
}

// Parse R (reset trigger counter).
static bool parse_reset(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'R'))) {
    return false;
  }
  out->type = CMD_RESET;
  return parse_expect_no_args(state, "R does not take arguments");
}

// Parse T [n] (trigger count, default 1).
static bool parse_trigger(parsed_command_t *out, parse_state_t *state) {
  if (!(state->token[1] == '\0' && cmd_match(state->token[0], 'T'))) {
    return false;
  }
  out->type = CMD_TRIGGER;
  out->trigger_count = 1;
  char *count_token = next_token(&state->cursor);
  if (count_token != NULL) {
    if (!parse_u32(count_token, &out->trigger_count) || out->trigger_count == 0U) {
      command_error(state->error_buf, state->error_buf_size, "T n requires n >= 1");
      return true;
    }
    if (next_token(&state->cursor) != NULL) {
      command_error(state->error_buf, state->error_buf_size, "T accepts at most one argument");
    }
  }
  return true;
}

// Parse one input line into a command and argument payload.
input_parse_result_t input_parse_line(const char *line,
                                  parsed_command_t *out,
                                  const shim_runtime_state_t *runtime_state,
                                  char *error_buf,
                                  size_t error_buf_size) {
  if (out == NULL) {
    command_error(error_buf, error_buf_size, "internal parser error");
    return INPUT_PARSE_ERROR;
  }

  if (error_buf != NULL && error_buf_size > 0U) {
    error_buf[0] = '\0';
  }

  out->type = CMD_INVALID;
  out->channel = -1;
  out->amps = 0.0;
  out->trigger_count = 0;
  out->file_path[0] = '\0';
  out->trigger_lockout_ms = 0.0;

  if (line == NULL) {
    command_error(error_buf, error_buf_size, "empty command");
    return INPUT_PARSE_EMPTY;
  }

  char buffer[INPUT_LINE_MAX];
  size_t n = strlen(line);
  if (n >= sizeof(buffer)) {
    command_error(error_buf, error_buf_size, "command line too long");
    return INPUT_PARSE_ERROR;
  }

  memcpy(buffer, line, n + 1U);
  char *trimmed = trim_whitespace(buffer);
  if (*trimmed == '\0') {
    return INPUT_PARSE_EMPTY;
  }

  char *cursor = trimmed;
  char *token = next_token(&cursor);
  if (token == NULL) {
    return INPUT_PARSE_EMPTY;
  }

  parse_state_t state = {
    .token = token,
    .cursor = cursor,
    .runtime_state = runtime_state,
    .error_buf = error_buf,
    .error_buf_size = error_buf_size,
  };

  // Try parsers in priority order (matches command_type_t enum order);
  // first match wins.
  static const parse_fn_t parsers[] = {
    parse_help,
    parse_quit,
    parse_status,
    parse_power_on,
    parse_hard_reset,
    parse_zero,
    parse_read,
    parse_manual_set,
    parse_update,
    parse_calibrate,
    parse_lockout,
    parse_load,
    parse_exit_file,
    parse_reset,
    parse_trigger,
  };

  for (size_t i = 0; i < (sizeof(parsers) / sizeof(parsers[0])); ++i) {
    if (parsers[i](out, &state)) {
      if (error_buf != NULL && error_buf_size > 0U && error_buf[0] != '\0') {
        return INPUT_PARSE_ERROR;
      }
      return INPUT_PARSE_OK;
    }
  }

  command_error(error_buf, error_buf_size, "unknown command (try H for help)");
  return INPUT_PARSE_ERROR;
}
