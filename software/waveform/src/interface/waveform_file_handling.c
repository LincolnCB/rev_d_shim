#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>

#include "waveform_file_handling.h"

#define MAX_LINE_LEN 4096

static void strip_comments_and_trim(char *line) {
  if (line == NULL) {
    return;
  }

  char *comment = strchr(line, '#');
  if (comment != NULL) {
    *comment = '\0';
  }

  char *start = line;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
    start++;
  }

  char *end = start + strlen(start);
  while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
    *--end = '\0';
  }

  if (start != line) {
    memmove(line, start, (size_t)(end - start) + 1);
  }
}

// Count the whitespace-or-comma-separated fields in a line (ignores trailing newline).
// Returns 0 for an empty/blank line.
static int count_fields(const char *line) {
  if (line == NULL || line[0] == '\0' || line[0] == '\n' || line[0] == '\r') {
    return 0;
  }

  char buffer[MAX_LINE_LEN];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';

  int fields = 0;
  char *token = strtok(buffer, " ,\t\r\n");
  while (token != NULL) {
    fields++;
    token = strtok(NULL, " ,\t\r\n");
  }
  return fields;
}

// Parse the next whitespace-or-comma-separated double out of *cursor,
// advancing *cursor past it. Returns 0 on success, -1 if the field is
// missing or not a valid number.
static int next_double(char **cursor, double *out) {
  if (cursor == NULL || *cursor == NULL || **cursor == '\0') {
    return -1;
  }

  char *start = *cursor;
  while (*start == ' ' || *start == '\t' || *start == ',' || *start == '\r' || *start == '\n') {
    start++;
  }
  if (*start == '\0') {
    return -1;
  }

  char *end = start;
  while (*end != '\0' && *end != ' ' && *end != '\t' && *end != ',' && *end != '\r' && *end != '\n') {
    end++;
  }

  // Advance past the delimiter we're about to null out, so the next call
  // starts after it rather than sitting on the null byte forever.
  char *next = end;
  if (*end != '\0') {
    *end = '\0';
    next = end + 1;
  }

  char *parse_end;
  errno = 0;
  double val = strtod(start, &parse_end);
  if (errno != 0 || parse_end == start) {
    return -1;
  }

  *cursor = next;
  *out = val;
  return 0;
}

// --- Shared thread-control helpers --------------------------------------
//
// All three *_file_info_t structs embed a stream_ctrl_t. These helpers hold
// the one copy of the mutex-guarded init/stop/query/set-state logic; the
// public per-type wrappers further down just forward into them.

static void stream_ctrl_init(stream_ctrl_t *ctrl) {
  pthread_mutex_init(&ctrl->mutex, NULL);
  ctrl->stop_requested = false;
  ctrl->state = STREAM_THREAD_IDLE;
}

static void stream_ctrl_destroy(stream_ctrl_t *ctrl) {
  pthread_mutex_destroy(&ctrl->mutex);
}

static void stream_ctrl_request_stop(stream_ctrl_t *ctrl) {
  pthread_mutex_lock(&ctrl->mutex);
  ctrl->stop_requested = true;
  // Don't clobber a terminal state: a thread that already completed or stopped
  // must stay reported as finished even if a late stop request arrives.
  if (ctrl->state != STREAM_THREAD_COMPLETED && ctrl->state != STREAM_THREAD_STOPPED) {
    ctrl->state = STREAM_THREAD_STOP_REQUESTED;
  }
  pthread_mutex_unlock(&ctrl->mutex);
}

static bool stream_ctrl_should_stop(const stream_ctrl_t *ctrl) {
  pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
  bool stop = ctrl->stop_requested;
  pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
  return stop;
}

static bool stream_ctrl_is_finished(const stream_ctrl_t *ctrl) {
  pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
  bool finished = (ctrl->state == STREAM_THREAD_COMPLETED ||
                   ctrl->state == STREAM_THREAD_STOPPED);
  pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
  return finished;
}

static bool stream_ctrl_stopped_early(const stream_ctrl_t *ctrl) {
  pthread_mutex_lock((pthread_mutex_t *)&ctrl->mutex);
  bool stopped_early = (ctrl->state == STREAM_THREAD_STOPPED);
  pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
  return stopped_early;
}

static void stream_ctrl_set_state(stream_ctrl_t *ctrl, stream_thread_state_t state) {
  pthread_mutex_lock(&ctrl->mutex);
  ctrl->state = state;
  pthread_mutex_unlock(&ctrl->mutex);
}

// --- Validation -----------------------------------------------------------

// Validate the input file (consistent channel count, realistic values, etc.)
int validate_input_file(const char *path, waveform_file_info_t *info) {
  // Initialize the info structure to zero to ensure all fields are set to default values
  memset(info, 0, sizeof(*info));

  // Check file existence and readability
  if (access(path, R_OK) != 0) {
    fprintf(stderr, "Error: input file '%s' does not exist or is not readable\n", path);
    return -1;
  }

  // Open the file for reading
  FILE *fp = fopen(path, "r");
  if (fp == NULL) {
    fprintf(stderr, "Error: could not open input file '%s': %s\n", path, strerror(errno));
    return -1;
  }

  // Tracking variables
  char line[MAX_LINE_LEN];
  long line_num = 0;
  int fields = -1;
  int num_channels = 0;
  long num_rows = 0;
  long num_trigs = 0;
  double prev_timestamp = 0.0;
  bool has_prev = false;
  double min_current = 0.0;
  double max_current = 0.0;
  bool has_current = false;
  double min_dt = -1.0;
  bool has_min_dt = false;

  // Read through all lines in the file
  while (fgets(line, sizeof(line), fp) != NULL) {
    line_num++;
    strip_comments_and_trim(line);
    if (line[0] == '\0') {
      continue; // skip blank lines and comment-only lines
    }

    // Count the number of fields in the line to track the channel count
    // Validate against previous lines
    int line_fields = count_fields(line);
    if (fields < 0) {
      fields = line_fields;
      num_channels = fields - 1;
      if (num_channels < MIN_CHANNELS || num_channels > MAX_CHANNELS) {
        fprintf(stderr,
          "Error: input file '%s' line %ld has %d channel(s), must be between %d and %d\n",
          path, line_num, num_channels, MIN_CHANNELS, MAX_CHANNELS);
        fclose(fp);
        return -1;
      }
    } else if (line_fields != fields) {
      fprintf(stderr, "Error: input file '%s' line %ld has %d field(s), expected %d\n",
        path, line_num, line_fields, fields);
      fclose(fp);
      return -1;
    }

    char *cursor = line;

    // Parse the timestamp (first field) and validate it
    double timestamp;
    if (next_double(&cursor, &timestamp) != 0) {
      fprintf(stderr, "Error: input file '%s' line %ld has an invalid timestamp\n", path, line_num);
      fclose(fp);
      return -1;
    }
    if (timestamp < 0.0) {
      fprintf(stderr, "Error: input file '%s' line %ld has a negative timestamp (%g)\n",
        path, line_num, timestamp);
      fclose(fp);
      return -1;
    }

    // A row is a trigger point if it's the first row, or if time has reset
    // (this row's timestamp is lower than the previous row's)
    if (!has_prev) {
      num_trigs++;
    } else if (timestamp < prev_timestamp) {
      num_trigs++;
    } else {
      double dt = timestamp - prev_timestamp;
      if (!has_min_dt || dt < min_dt) {
        min_dt = dt;
        has_min_dt = true;
      }
    }
    prev_timestamp = timestamp;
    has_prev = true;

    // Check the values for each channel
    for (int c = 0; c < num_channels; c++) {
      double current;
      if (next_double(&cursor, &current) != 0) {
        fprintf(stderr, "Error: input file '%s' line %ld has an invalid value in channel %d\n",
          path, line_num, c + 1);
        fclose(fp);
        return -1;
      }
      if (current < CURRENT_MIN_AMPS || current > CURRENT_MAX_AMPS) {
        fprintf(stderr,
          "Error: input file '%s' line %ld channel %d value %g A is outside the allowed range [%g, %g] A\n",
          path, line_num, c + 1, current, CURRENT_MIN_AMPS, CURRENT_MAX_AMPS);
        fclose(fp);
        return -1;
      }
      if (!has_current) {
        min_current = current;
        max_current = current;
        has_current = true;
      } else {
        if (current < min_current) min_current = current;
        if (current > max_current) max_current = current;
      }
    }

    num_rows++;
  }

  fclose(fp);

  if (num_rows == 0) {
    fprintf(stderr, "Error: input file '%s' has no data rows\n", path);
    return -1;
  }

  // Fill in the info structure with the gathered data
  info->path = path;
  info->num_channels = num_channels;
  info->num_rows = num_rows;
  info->num_trigs = num_trigs;
  info->min_current = min_current;
  info->max_current = max_current;
  info->min_dt = min_dt; // -1.0 if never computed (e.g. every row was a trigger point)
  info->iters = 0;       // caller sets this before starting dac_stream_thread
  stream_ctrl_init(&info->ctrl);
  return 0;
}

// Validate the ADC file (just a list of sample timestamps)
int validate_adc_file(const char *path, long expected_trigs, adc_cmd_file_info_t *info) {
  // Initialize the info structure to zero to ensure all fields are set to default values
  memset(info, 0, sizeof(*info));

  // Check file existence and readability
  if (access(path, R_OK) != 0) {
    fprintf(stderr, "Error: ADC file '%s' does not exist or is not readable\n", path);
    return -1;
  }

  // Open the file for reading
  FILE *fp = fopen(path, "r");
  if (fp == NULL) {
    fprintf(stderr, "Error: could not open ADC file '%s': %s\n", path, strerror(errno));
    return -1;
  }

  // Tracking variables
  char line[MAX_LINE_LEN];
  long line_num = 0;
  long num_rows = 0;
  long num_trigs = 0;
  double prev_timestamp = 0.0;
  bool has_prev = false;
  double min_dt = -1.0;
  bool has_min_dt = false;

  // Read through all lines in the file
  while (fgets(line, sizeof(line), fp) != NULL) {
    line_num++;
    strip_comments_and_trim(line);
    if (line[0] == '\0') {
      continue; // skip blank lines and comment-only lines
    }

    // Count the number of fields in the line to ensure there is exactly one timestamp per line
    int line_fields = count_fields(line);
    if (line_fields != 1) {
      fprintf(stderr,
        "Error: ADC file '%s' line %ld has %d field(s), expected exactly 1 (timestamp only)\n",
        path, line_num, line_fields);
      fclose(fp);
      return -1;
    }

    // Parse the timestamp and validate it
    char *cursor = line;
    double timestamp;
    if (next_double(&cursor, &timestamp) != 0) {
      fprintf(stderr, "Error: ADC file '%s' line %ld has an invalid timestamp\n", path, line_num);
      fclose(fp);
      return -1;
    }
    if (timestamp < 0.0) {
      fprintf(stderr, "Error: ADC file '%s' line %ld has a negative timestamp (%g)\n",
        path, line_num, timestamp);
      fclose(fp);
      return -1;
    }

    // Same trigger-point rule as the input file: first line, or a time reset,
    // marks the start of a new sweep.
    if (!has_prev) {
      num_trigs++;
    } else if (timestamp < prev_timestamp) {
      num_trigs++;
    } else {
      double dt = timestamp - prev_timestamp;
      if (!has_min_dt || dt < min_dt) {
        min_dt = dt;
        has_min_dt = true;
      }
    }
    prev_timestamp = timestamp;
    has_prev = true;

    num_rows++;
  }

  fclose(fp);

  if (num_rows == 0) {
    fprintf(stderr, "Error: ADC file '%s' has no sample timestamps\n", path);
    return -1;
  }

  if (num_trigs != expected_trigs) {
    fprintf(stderr,
      "Error: ADC file '%s' has %ld trigger point(s), expected %ld to match the input file\n",
      path, num_trigs, expected_trigs);
    return -1;
  }

  // Fill in the info structure with the gathered data
  info->path = path;
  info->num_rows = num_rows;
  info->num_trigs = num_trigs;
  info->min_dt = min_dt; // -1.0 if never computed (e.g. every line was a trigger point)
  info->iters = 0;       // caller sets this before starting adc_cmd_stream_thread
  stream_ctrl_init(&info->ctrl);
  return 0;
}

void adc_data_file_info_init(adc_data_file_info_t *info, long num_samples, int iters) {
  if (info == NULL) {
    return;
  }
  info->path = NULL;
  info->num_samples = num_samples;
  info->iters = iters;
  info->hw = NULL;
  stream_ctrl_init(&info->ctrl);
}

void trigger_file_info_init(trigger_file_info_t *info, const char *label, long num_trigs, int iters) {
  if (info == NULL) {
    return;
  }
  info->label = label;
  info->path = NULL;
  info->num_trigs = num_trigs;
  info->iters = iters;
  info->hw = NULL;
  stream_ctrl_init(&info->ctrl);
}

// --- Public per-type control wrappers -------------------------------------
//
// Thin forwarders onto the shared stream_ctrl_t helpers above.

void waveform_file_info_destroy(waveform_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_destroy(&info->ctrl);
}
void waveform_file_info_request_stop(waveform_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_request_stop(&info->ctrl);
}
bool waveform_file_info_should_stop(const waveform_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_should_stop(&info->ctrl);
}
bool waveform_file_info_is_finished(const waveform_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_is_finished(&info->ctrl);
}
bool waveform_file_info_stopped_early(const waveform_file_info_t *info) {
  if (info == NULL) return false;
  return stream_ctrl_stopped_early(&info->ctrl);
}
static void waveform_file_info_set_state(waveform_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

void adc_cmd_file_info_destroy(adc_cmd_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_destroy(&info->ctrl);
}
void adc_cmd_file_info_request_stop(adc_cmd_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_request_stop(&info->ctrl);
}
bool adc_cmd_file_info_should_stop(const adc_cmd_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_should_stop(&info->ctrl);
}
bool adc_cmd_file_info_is_finished(const adc_cmd_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_is_finished(&info->ctrl);
}
bool adc_cmd_file_info_stopped_early(const adc_cmd_file_info_t *info) {
  if (info == NULL) return false;
  return stream_ctrl_stopped_early(&info->ctrl);
}
static void adc_cmd_file_info_set_state(adc_cmd_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

void adc_data_file_info_destroy(adc_data_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_destroy(&info->ctrl);
}
void adc_data_file_info_request_stop(adc_data_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_request_stop(&info->ctrl);
}
bool adc_data_file_info_should_stop(const adc_data_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_should_stop(&info->ctrl);
}
bool adc_data_file_info_is_finished(const adc_data_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_is_finished(&info->ctrl);
}
bool adc_data_file_info_stopped_early(const adc_data_file_info_t *info) {
  if (info == NULL) return false;
  return stream_ctrl_stopped_early(&info->ctrl);
}
static void adc_data_file_info_set_state(adc_data_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

void trigger_file_info_destroy(trigger_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_destroy(&info->ctrl);
}
void trigger_file_info_request_stop(trigger_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_request_stop(&info->ctrl);
}
bool trigger_file_info_should_stop(const trigger_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_should_stop(&info->ctrl);
}
bool trigger_file_info_is_finished(const trigger_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_is_finished(&info->ctrl);
}
bool trigger_file_info_stopped_early(const trigger_file_info_t *info) {
  if (info == NULL) return false;
  return stream_ctrl_stopped_early(&info->ctrl);
}
static void trigger_file_info_set_state(trigger_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

// --- Stream threads ---------------------------------------------------
//
// The DAC and ADC threads stream real hardware commands. The trigger thread
// still uses placeholder logic (to be replaced when its real streaming is
// implemented).

// --- Oversized-delay splitting ----------------------------------------
//
// A single DAC/ADC command can encode at most HW_MAX_DELAY_CLKS clock cycles of
// delay (a 25-bit field). A larger gap is broken into a run of no-op delay
// commands -- each occupies a command-buffer slot but is not a sample -- plus a
// residual that the real command (DAC write / ADC read) carries. When a real
// command carries the residual, it must not fall below the hardware minimum
// delay or the hardware flags it "too short"; the no-op commands themselves have
// no such minimum.

// Peel the next no-op delay chunk off `remaining` (which is > HW_MAX_DELAY_CLKS).
// If taking a full-size chunk would leave a residual below min_delay (making the
// real command "too short"), the chunk is trimmed to leave exactly min_delay
// behind; otherwise a full HW_MAX_DELAY_CLKS chunk is taken. min_delay is a
// 25-bit value, so HW_MAX_DELAY_CLKS + min_delay cannot overflow a uint32_t.
static uint32_t delay_next_chunk(uint32_t remaining, uint32_t min_delay) {
  if (remaining <= HW_MAX_DELAY_CLKS + min_delay) {
    return remaining - min_delay;
  }
  return HW_MAX_DELAY_CLKS;
}

// Count the no-op delay commands needed to shave `delay_clks` down to a residual
// that fits the command delay field (and stays >= min_delay), and report that
// residual via *residual. Uses the same chunking as the emission loops so the
// counts always match.
static uint32_t delay_noop_split(uint32_t delay_clks, uint32_t min_delay, uint32_t *residual) {
  uint32_t count = 0;
  uint32_t remaining = delay_clks;
  while (remaining > HW_MAX_DELAY_CLKS) {
    remaining -= delay_next_chunk(remaining, min_delay);
    count++;
  }
  *residual = remaining;
  return count;
}

// Send a DAC command to the hardware, blocking until there is command space.
// *space tracks the locally-known number of free DAC sample command slots; it
// is decremented per command and refreshed from hardware when it runs low.
// When there isn't enough space the thread re-checks and, if still short,
// sleeps DAC_STREAM_NO_SPACE_SLEEP_US before trying again. `last` marks the
// final command of the final iteration (clears the continue flag).
//
// When `noop_first` is set the command is preceded by a trigger-wait no-op
// (used for a trigger point at a non-zero time: wait for the trigger, then
// delay to the row's timestamp before applying it).
//
// The DAC applies a command's delay *before* the write (pre-delay), so when a
// delay exceeds HW_MAX_DELAY_CLKS the excess is emitted as no-op delay commands
// ahead of the write and the write itself carries the residual. Each of those
// no-ops, plus any trigger-wait no-op, decrements *space by one.
//
// Returns 0 on success. Returns non-zero if the command could not be sent:
// either a stop was requested while waiting for command space, or a hardware
// command reported an error (logged here before returning). In both cases the
// caller should stop streaming.
static int dac_stream_send(waveform_file_info_t *info, bool noop_first,
                           bool is_trig, uint32_t delay_clks,
                           const double *amps, bool last, int *space) {
  // A trigger wait carries no delay; a delay command may need leading no-ops to
  // consume the portion of the delay beyond the 25-bit command delay field.
  uint32_t residual = delay_clks;
  uint32_t split_noops = 0;
  if (!is_trig) {
    split_noops = delay_noop_split(delay_clks, info->hw->dac_min_delay, &residual);
  }

  int needed = (noop_first ? 1 : 0) + (int)split_noops + 1;
  while (*space < needed) {
    *space = hw_get_dac_sample_cmd_space(info->hw);
    if (*space >= needed) {
      break;
    }
    if (waveform_file_info_should_stop(info)) {
      return -1;
    }
    usleep(DAC_STREAM_NO_SPACE_SLEEP_US);
  }

  if (noop_first) {
    if (hw_dac_noop_trig(info->hw) != 0) {
      fprintf(stderr, "Error: [DAC] failed to send trigger-wait no-op\n");
      return -1;
    }
    (*space)--;
  }

  // Emit the leading no-op delay commands that consume the oversized portion of
  // the delay, leaving `residual` for the write itself (pre-delay semantics).
  uint32_t remaining = is_trig ? 0 : delay_clks;
  while (remaining > HW_MAX_DELAY_CLKS) {
    uint32_t chunk = delay_next_chunk(remaining, info->hw->dac_min_delay);
    if (hw_dac_noop_delay(info->hw, chunk) != 0) {
      fprintf(stderr, "Error: [DAC] failed to send oversized-delay no-op\n");
      return -1;
    }
    (*space)--;
    remaining -= chunk;
  }

  if (is_trig) {
    if (hw_set_dacs_trig(info->hw, amps, last) != 0) {
      fprintf(stderr, "Error: [DAC] failed to send trigger-wait command\n");
      return -1;
    }
  } else {
    if (hw_set_dacs_delay(info->hw, amps, residual, last) != 0) {
      fprintf(stderr, "Error: [DAC] failed to send delay command\n");
      return -1;
    }
  }
  (*space)--;
  return 0;
}

// DAC streaming thread entry point. Reads the waveform file, converts timestamps 
// from seconds to SPI clock cycles, and sends commands to the DAC
// hardware, handling trigger points and delays as specified.
void *dac_stream_thread(void *arg) {
  waveform_file_info_t *info = (waveform_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  if (info->hw == NULL) {
    fprintf(stderr, "Error: [DAC] no hardware handle provided\n");
    waveform_file_info_set_state(info, STREAM_THREAD_STOPPED);
    return NULL;
  }
  waveform_file_info_set_state(info, STREAM_THREAD_RUNNING);

  // A command is buffered ("pending") until we know whether another command
  // follows it. This lets us flag the final command of the final iteration --
  // which clears the hardware continue flag -- even when the file ends with
  // blank or comment-only lines. The pending buffer persists across iterations
  // so the last row of one pass is flagged non-final once the next pass starts.
  bool     have_pending = false;
  bool     pending_noop_first = false;
  bool     pending_is_trig = false;
  uint32_t pending_delay_clks = 0;
  double   pending_amps[HW_MAX_CHANNELS] = {0.0}; // inactive channels stay 0 for the whole run

  // Trigger points are detected exactly as in validation (first row of the
  // whole stream, or a timestamp below the previous one), so the number of
  // trigger-wait commands matches the expected trigger count. This state
  // persists across iterations, so an iteration boundary counts as a reset.
  bool     has_prev = false;
  double   prev_timestamp = 0.0;
  uint32_t prev_clks = 0;

  // Locally-tracked number of free DAC sample command slots.
  int space = hw_get_dac_sample_cmd_space(info->hw);
  bool stopped = false;

  for (int iter = 0; iter < info->iters && !stopped; iter++) {
    FILE *fp = fopen(info->path, "r");
    if (fp == NULL) {
      fprintf(stderr, "Error: [DAC] could not reopen '%s': %s\n", info->path, strerror(errno));
      break;
    }

    char line[MAX_LINE_LEN];

    while (fgets(line, sizeof(line), fp) != NULL) {
      if (waveform_file_info_should_stop(info)) {
        stopped = true;
        break;
      }
      strip_comments_and_trim(line);
      if (line[0] == '\0') {
        continue;
      }

      char *cursor = line;
      double timestamp;
      if (next_double(&cursor, &timestamp) != 0) {
        fprintf(stderr, "Broken DAC line that should have been caught earlier.");
        exit(1);
      }

      // Flush the previously buffered command: another line follows it, so it
      // is not the final command (last = false).
      if (have_pending) {
        if (dac_stream_send(info, pending_noop_first, pending_is_trig,
                            pending_delay_clks, pending_amps, false, &space) != 0) {
          stopped = true;
          break;
        }
        have_pending = false;
      }

      // Convert this timestamp (in seconds) to absolute SPI clock cycles up
      // front so dt is computed from absolute cycle counts, avoiding rounding
      // drift.
      uint32_t t_clks = (uint32_t)((timestamp * (double)info->hw->spi_clk_hz) + 0.5);

      // A trigger point is the first row of the whole stream or a time reset
      // (this timestamp is below the previous one). Within a sweep, the command
      // is just a delay of dt cycles from the previous row.
      if (!has_prev || timestamp < prev_timestamp) {
        if (t_clks == 0) {
          // Wait for a trigger, then apply this row immediately.
          pending_noop_first = false;
          pending_is_trig = true;
          pending_delay_clks = 0;
        } else {
          // Wait for a trigger (no-op), then delay to this row's time before
          // applying it (the delay is the row's t, i.e. dt measured from 0).
          pending_noop_first = true;
          pending_is_trig = false;
          pending_delay_clks = t_clks;
        }
      } else {
        pending_noop_first = false;
        pending_is_trig = false;
        pending_delay_clks = t_clks - prev_clks;
      }
      prev_timestamp = timestamp;
      prev_clks = t_clks;
      has_prev = true;

      // Fill the channel-indexed amps buffer. The channel count is constant,
      // so inactive channels stay at their preloaded 0 and only the active
      // channels are overwritten each line.
      for (int c = 0; c < info->num_channels; c++) {
        double value;
        if (next_double(&cursor, &value) != 0) {
          value = 0.0;
        }
        pending_amps[c] = value;
      }
      have_pending = true;
    }

    fclose(fp);
  }

  // Flush the final buffered command. If every iteration completed without a
  // stop, this is the last command of the whole sequence (last = true).
  if (have_pending && !stopped) {
    (void)dac_stream_send(info, pending_noop_first, pending_is_trig,
                          pending_delay_clks, pending_amps, true, &space);
  }

  waveform_file_info_set_state(info, stopped ? STREAM_THREAD_STOPPED
                                             : STREAM_THREAD_COMPLETED);
  return NULL;
}

// Send an ADC command sequence to the hardware, blocking until there is
// command space. *space tracks the locally-known number of free ADC command
// slots; it is decremented per command and refreshed from hardware when it
// runs low. When there isn't enough space the thread re-checks and, if still
// short, sleeps ADC_CMD_STREAM_NO_SPACE_SLEEP_US before trying again.
//
// A read command may be preceded by prefix no-ops that belong to a trigger
// point: `noop_trig_first` emits a trigger-wait no-op (only used before the
// very first read, which has nothing ahead of it to supply a trigger), and
// `noop_delay_first` emits a delay no-op of `noop_delay_clks` cycles (used for
// a trigger point at a non-zero time, to delay from the trigger to the sample).
//
// The read itself is either a trigger-wait read (`is_trig`) or a delay read of
// `delay_clks` cycles -- this is the delay that follows the read, since on the
// ADC side a command's wait happens after it. `last` marks the final read of
// the final iteration (a trigger-wait read that clears the continue flag).
//
// Both delays may exceed the 25-bit command delay field. An oversized pre-read
// delay no-op is simply split into several no-op delay commands. An oversized
// post-read delay is carried as a residual on the read plus trailing no-op delay
// commands (kept >= the hardware minimum so the read's own delay is not "too
// short").
//
// Each emitted command decrements *space by one. Returns 0 on success. Returns
// non-zero if the sequence could not be sent: either a stop was requested while
// waiting for command space, or a hardware command reported an error (logged
// here before returning). In both cases the caller should stop streaming.
static int adc_cmd_stream_send(adc_cmd_file_info_t *info, bool noop_trig_first,
                           bool noop_delay_first, uint32_t noop_delay_clks,
                           bool is_trig, uint32_t delay_clks, bool last, int *space) {
  // The pre-read delay no-op is itself a no-op (no real command carries part of
  // it), so it just splits into ceil(noop_delay_clks / HW_MAX_DELAY_CLKS) pieces.
  uint32_t pre_noops = 0;
  if (noop_delay_first) {
    pre_noops = (noop_delay_clks + HW_MAX_DELAY_CLKS - 1) / HW_MAX_DELAY_CLKS;
  }

  // The post-read delay is carried by the read plus trailing no-op delays; a
  // trigger wait carries no post-read delay.
  uint32_t residual = delay_clks;
  uint32_t post_noops = 0;
  if (!is_trig) {
    post_noops = delay_noop_split(delay_clks, info->hw->adc_min_delay, &residual);
  }

  int needed = (noop_trig_first ? 1 : 0) + (int)pre_noops + 1 + (int)post_noops;
  while (*space < needed) {
    *space = hw_get_adc_cmd_space(info->hw);
    if (*space >= needed) {
      break;
    }
    if (adc_cmd_file_info_should_stop(info)) {
      return -1;
    }
    usleep(ADC_CMD_STREAM_NO_SPACE_SLEEP_US);
  }

  if (noop_trig_first) {
    if (hw_adc_noop_trig(info->hw) != 0) {
      fprintf(stderr, "Error: [ADC] failed to send trigger-wait no-op\n");
      return -1;
    }
    (*space)--;
  }
  if (noop_delay_first) {
    // Split an oversized pre-read delay across several no-op delay commands.
    uint32_t rem = noop_delay_clks;
    while (rem > 0) {
      uint32_t chunk = (rem > HW_MAX_DELAY_CLKS) ? HW_MAX_DELAY_CLKS : rem;
      if (hw_adc_noop_delay(info->hw, chunk) != 0) {
        fprintf(stderr, "Error: [ADC] failed to send delay no-op\n");
        return -1;
      }
      (*space)--;
      rem -= chunk;
    }
  }

  if (is_trig) {
    if (hw_adc_read_trig(info->hw, last) != 0) {
      fprintf(stderr, "Error: [ADC] failed to send trigger-wait read\n");
      return -1;
    }
  } else {
    if (hw_adc_read_delay(info->hw, residual, last) != 0) {
      fprintf(stderr, "Error: [ADC] failed to send delay read\n");
      return -1;
    }
  }
  (*space)--;

  // Emit trailing no-op delay commands that carry the portion of the post-read
  // delay beyond what the read command itself can encode (delay-read case only).
  uint32_t remaining = is_trig ? 0 : delay_clks;
  while (remaining > HW_MAX_DELAY_CLKS) {
    uint32_t chunk = delay_next_chunk(remaining, info->hw->adc_min_delay);
    if (hw_adc_noop_delay(info->hw, chunk) != 0) {
      fprintf(stderr, "Error: [ADC] failed to send oversized-delay no-op\n");
      return -1;
    }
    (*space)--;
    remaining -= chunk;
  }
  return 0;
}

// ADC streaming thread entry point. Reads the ADC file (a list of sample
// timestamps), converts them from seconds to SPI clock cycles, and sends
// read commands to the ADC hardware.
//
// On the ADC side a command's wait happens *after* it, so the delay tied to a
// row is the gap to the *next* row. Each read is therefore buffered ("pending")
// until the following row is known: if the next row is a trigger point the read
// gets a trigger-wait afterwards, otherwise it gets a delay of dt cycles. The
// final pending read of the final iteration becomes a trigger-wait read with
// last = true, which clears the continue flag and handles the trailing wait.
//
// Trigger points (first row of the whole stream, or a timestamp below the
// previous one) start a new sweep. The very first read is prefixed with a
// trigger-wait no-op, since nothing precedes it to supply the first trigger;
// later trigger points get their trigger from the previous read's trigger-wait.
// A trigger point at a non-zero time additionally gets a delay no-op prefix, to
// wait from the trigger to that row's timestamp before reading.
void *adc_cmd_stream_thread(void *arg) {
  adc_cmd_file_info_t *info = (adc_cmd_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  if (info->hw == NULL) {
    fprintf(stderr, "Error: [ADC] no hardware handle provided\n");
    adc_cmd_file_info_set_state(info, STREAM_THREAD_STOPPED);
    return NULL;
  }
  adc_cmd_file_info_set_state(info, STREAM_THREAD_RUNNING);

  // A read command is buffered ("pending") until the next row is known, since
  // the delay that follows a read is the gap to the next row. The pending state
  // persists across iterations so the last read of one pass is flagged non-final
  // (and gets a trigger-wait, since the next pass restarts) once the next pass
  // begins.
  bool     have_pending = false;
  bool     pending_noop_trig = false;   // prefix: trigger-wait no-op (first read only)
  bool     pending_noop_delay = false;  // prefix: delay no-op (non-zero-time trigger point)
  uint32_t pending_noop_delay_clks = 0;
  uint32_t pending_t_clks = 0;          // this pending row's absolute time, in SPI clocks

  // Trigger points are detected exactly as in validation (first row of the whole
  // stream, or a timestamp below the previous one). This state persists across
  // iterations, so an iteration boundary counts as a reset (new trigger point).
  bool     has_prev = false;
  double   prev_timestamp = 0.0;

  // Locally-tracked number of free ADC command slots.
  int space = hw_get_adc_cmd_space(info->hw);
  bool stopped = false;

  for (int iter = 0; iter < info->iters && !stopped; iter++) {
    FILE *fp = fopen(info->path, "r");
    if (fp == NULL) {
      fprintf(stderr, "Error: [ADC] could not reopen '%s': %s\n", info->path, strerror(errno));
      break;
    }

    char line[MAX_LINE_LEN];

    while (fgets(line, sizeof(line), fp) != NULL) {
      if (adc_cmd_file_info_should_stop(info)) {
        stopped = true;
        break;
      }
      strip_comments_and_trim(line);
      if (line[0] == '\0') {
        continue;
      }

      char *cursor = line;
      double timestamp;
      if (next_double(&cursor, &timestamp) != 0) {
        fprintf(stderr, "Broken ADC line that should have been caught earlier.");
        exit(1);
      }

      // Convert this timestamp (in seconds) to absolute SPI clock cycles up
      // front so dt is computed from absolute cycle counts, avoiding rounding
      // drift.
      uint32_t t_clks = (uint32_t)((timestamp * (double)info->hw->spi_clk_hz) + 0.5);

      // A trigger point is the first row of the whole stream or a time reset
      // (this timestamp is below the previous one).
      bool is_trig_point = (!has_prev) || (timestamp < prev_timestamp);

      // Flush the buffered read: its trailing wait is determined by this row.
      // If this row starts a new sweep, the previous read waits for a trigger;
      // otherwise it delays by the gap between the two rows.
      if (have_pending) {
        bool     p_is_trig = is_trig_point;
        uint32_t p_delay_clks = is_trig_point ? 0 : (t_clks - pending_t_clks);
        if (adc_cmd_stream_send(info, pending_noop_trig, pending_noop_delay,
                            pending_noop_delay_clks, p_is_trig, p_delay_clks,
                            false, &space) != 0) {
          stopped = true;
          break;
        }
        have_pending = false;
      }

      // Determine the prefix no-ops for this row (emitted just before its read
      // when it is later flushed).
      bool     this_noop_trig = false;
      bool     this_noop_delay = false;
      uint32_t this_noop_delay_clks = 0;
      if (is_trig_point) {
        if (!has_prev) {
          // Very first read of the whole stream: nothing precedes it to supply
          // the first trigger, so prefix a trigger-wait no-op.
          this_noop_trig = true;
        }
        if (t_clks != 0) {
          // Trigger point at a non-zero time: wait from the trigger to this
          // row's timestamp before reading.
          this_noop_delay = true;
          this_noop_delay_clks = t_clks;
        }
      }

      // Buffer this row as the new pending read.
      pending_noop_trig = this_noop_trig;
      pending_noop_delay = this_noop_delay;
      pending_noop_delay_clks = this_noop_delay_clks;
      pending_t_clks = t_clks;
      have_pending = true;

      prev_timestamp = timestamp;
      has_prev = true;
    }

    fclose(fp);
  }

  // Flush the final buffered read. If every iteration completed without a stop,
  // this is the last read of the whole sequence: a trigger-wait read with
  // last = true, which clears the continue flag and handles the trailing wait.
  if (have_pending && !stopped) {
    (void)adc_cmd_stream_send(info, pending_noop_trig, pending_noop_delay,
                          pending_noop_delay_clks, true, 0, true, &space);
  }

  adc_cmd_file_info_set_state(info, stopped ? STREAM_THREAD_STOPPED
                                        : STREAM_THREAD_COMPLETED);
  return NULL;
}

// ADC data streaming thread entry point. Once the ADC command stream is
// running, this drains the ADC data buffer: it polls for available samples,
// reads each one (8 channels per active board, converted to amps), and writes
// it to the output file. It reads num_samples * iters samples in total --
// matching the ADC command stream -- so the buffer never overflows.
//
// Draining the buffer is required even without an output file, so if info->path
// is unset the samples are still read (and discarded) to keep the hardware from
// overflowing. Writing the samples out is wired up once a path is provided.
void *adc_data_stream_thread(void *arg) {
  adc_data_file_info_t *info = (adc_data_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  if (info->hw == NULL) {
    fprintf(stderr, "Error: [ADC data] no hardware handle provided\n");
    adc_data_file_info_set_state(info, STREAM_THREAD_STOPPED);
    return NULL;
  }
  adc_data_file_info_set_state(info, STREAM_THREAD_RUNNING);

  // Open the output file if a path was provided. When it's unset (the path is
  // wired up later) samples are still drained from the buffer but discarded.
  FILE *out = NULL;
  if (info->path != NULL) {
    out = fopen(info->path, "w");
    if (out == NULL) {
      fprintf(stderr, "Error: [ADC data] could not open '%s': %s\n", info->path, strerror(errno));
      adc_data_file_info_set_state(info, STREAM_THREAD_STOPPED);
      return NULL;
    }
  }

  long total = info->num_samples * (long)info->iters;
  long read_count = 0;
  double amps[HW_MAX_CHANNELS] = {0.0};
  bool stopped = false;

  while (read_count < total && !stopped) {
    if (adc_data_file_info_should_stop(info)) {
      stopped = true;
      break;
    }

    int available = hw_get_adc_sample_count(info->hw);
    if (available < 0) {
      fprintf(stderr, "Error: [ADC data] failed to read available sample count\n");
      stopped = true;
      break;
    }
    if (available == 0) {
      usleep(ADC_DATA_STREAM_NO_DATA_SLEEP_US);
      continue;
    }

    for (int i = 0; i < available && read_count < total; i++) {
      if (hw_read_adc_data(info->hw, amps) != 0) {
        fprintf(stderr, "Error: [ADC data] failed to read sample\n");
        stopped = true;
        break;
      }
      read_count++;
      // Write the active-channel amps as one comma-separated line per sample.
      // The amps buffer is indexed by channel number, so the active channels are
      // just indices 0 .. channel_count-1. When there's no output file the
      // sample is still read above, draining the buffer, and simply discarded.
      if (out != NULL) {
        for (uint32_t c = 0; c < info->hw->channel_count; c++) {
          if (fprintf(out, "%s%.6g", (c == 0) ? "" : ",", amps[c]) < 0) {
            fprintf(stderr, "Error: [ADC data] failed to write to '%s': %s\n",
                    info->path, strerror(errno));
            stopped = true;
            break;
          }
        }
        if (!stopped && fputc('\n', out) == EOF) {
          fprintf(stderr, "Error: [ADC data] failed to write to '%s': %s\n",
                  info->path, strerror(errno));
          stopped = true;
        }
        if (stopped) {
          break;
        }
      }
    }
  }

  if (out != NULL) {
    fclose(out);
  }

  adc_data_file_info_set_state(info, stopped ? STREAM_THREAD_STOPPED
                                             : STREAM_THREAD_COMPLETED);
  return NULL;
}

void *trigger_stream_thread(void *arg) {
  trigger_file_info_t *info = (trigger_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  const char *label = info->label ? info->label : "Trigger";
  if (info->hw == NULL) {
    fprintf(stderr, "Error: [%s] no hardware handle provided\n", label);
    trigger_file_info_set_state(info, STREAM_THREAD_STOPPED);
    return NULL;
  }
  trigger_file_info_set_state(info, STREAM_THREAD_RUNNING);

  // Open the output file if a path was provided. When it's unset (the path is
  // wired up later) timepoints are still drained from the buffer but discarded.
  FILE *out = NULL;
  if (info->path != NULL) {
    out = fopen(info->path, "w");
    if (out == NULL) {
      fprintf(stderr, "Error: [%s] could not open '%s': %s\n", label, info->path, strerror(errno));
      trigger_file_info_set_state(info, STREAM_THREAD_STOPPED);
      return NULL;
    }
  }

  long total = info->num_trigs * (long)info->iters;
  long read_count = 0;
  bool stopped = false;

  while (read_count < total && !stopped) {
    if (trigger_file_info_should_stop(info)) {
      stopped = true;
      break;
    }

    int available = hw_get_trigger_sample_count(info->hw);
    if (available < 0) {
      fprintf(stderr, "Error: [%s] failed to read available sample count\n", label);
      stopped = true;
      break;
    }
    if (available == 0) {
      usleep(TRIGGER_DATA_STREAM_NO_DATA_SLEEP_US);
      continue;
    }

    for (int i = 0; i < available && read_count < total; i++) {
      double trigger_time;
      if (hw_read_trigger_data(info->hw, &trigger_time) != 0) {
        fprintf(stderr, "Error: [%s] failed to read trigger sample\n", label);
        stopped = true;
        break;
      }
      read_count++;
      // Write one trigger time (in seconds) per line. When there's no output
      // file the timepoint is still read above, draining the buffer, and simply
      // discarded.
      if (out != NULL) {
        if (fprintf(out, "%.9g\n", trigger_time) < 0) {
          fprintf(stderr, "Error: [%s] failed to write to '%s': %s\n",
                  label, info->path, strerror(errno));
          stopped = true;
          break;
        }
      }
    }
  }

  if (out != NULL) {
    fclose(out);
  }

  trigger_file_info_set_state(info, stopped ? STREAM_THREAD_STOPPED
                                            : STREAM_THREAD_COMPLETED);
  return NULL;
}
