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
  ctrl->state = STREAM_THREAD_STOP_REQUESTED;
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
  bool finished = (ctrl->state == STREAM_THREAD_FINISHED);
  pthread_mutex_unlock((pthread_mutex_t *)&ctrl->mutex);
  return finished;
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
int validate_adc_file(const char *path, long expected_trigs, adc_file_info_t *info) {
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
  info->iters = 0;       // caller sets this before starting adc_stream_thread
  stream_ctrl_init(&info->ctrl);
  return 0;
}

void trigger_file_info_init(trigger_file_info_t *info, const char *label, long num_trigs, int iters) {
  if (info == NULL) {
    return;
  }
  info->label = label;
  info->num_trigs = num_trigs;
  info->iters = iters;
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
static void waveform_file_info_set_state(waveform_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

void adc_file_info_destroy(adc_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_destroy(&info->ctrl);
}
void adc_file_info_request_stop(adc_file_info_t *info) {
  if (info == NULL) return;
  stream_ctrl_request_stop(&info->ctrl);
}
bool adc_file_info_should_stop(const adc_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_should_stop(&info->ctrl);
}
bool adc_file_info_is_finished(const adc_file_info_t *info) {
  if (info == NULL) return true;
  return stream_ctrl_is_finished(&info->ctrl);
}
static void adc_file_info_set_state(adc_file_info_t *info, stream_thread_state_t state) {
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
static void trigger_file_info_set_state(trigger_file_info_t *info, stream_thread_state_t state) {
  stream_ctrl_set_state(&info->ctrl, state);
}

// --- Stream threads ---------------------------------------------------
//
// Placeholder streaming logic: each thread replays its source `iters`
// times. DAC/ADC re-open and re-read their file from disk on every pass;
// within a pass, dt is measured from the previous line (0.0 for the first
// line of a pass). Replace the bodies below with real streaming logic when
// it's ready -- the file-reopen/parse loop and dt bookkeeping can go.

void *dac_stream_thread(void *arg) {
  waveform_file_info_t *info = (waveform_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  waveform_file_info_set_state(info, STREAM_THREAD_RUNNING);

  for (int iter = 0; iter < info->iters && !waveform_file_info_should_stop(info); iter++) {
    FILE *fp = fopen(info->path, "r");
    if (fp == NULL) {
      fprintf(stderr, "Error: [DAC] could not reopen '%s': %s\n", info->path, strerror(errno));
      break;
    }

    char line[MAX_LINE_LEN];
    bool has_prev = false;
    double prev_timestamp = 0.0;

    while (fgets(line, sizeof(line), fp) != NULL) {
      if (waveform_file_info_should_stop(info)) {
        break;
      }
      strip_comments_and_trim(line);
      if (line[0] == '\0') {
        continue;
      }

      char *cursor = line;
      double timestamp;
      if (next_double(&cursor, &timestamp) != 0) {
        continue; // already validated; shouldn't happen
      }
      double dt = has_prev ? (timestamp - prev_timestamp) : 0.0;
      prev_timestamp = timestamp;
      has_prev = true;

      double values[MAX_CHANNELS];
      for (int c = 0; c < info->num_channels; c++) {
        if (next_double(&cursor, &values[c]) != 0) {
          values[c] = 0.0;
        }
      }

      printf("[DAC] iter %d dt=%g values=[", iter + 1, dt);
      for (int c = 0; c < info->num_channels; c++) {
        printf("%s%g", c == 0 ? "" : ", ", values[c]);
      }
      printf("]\n");
    }

    fclose(fp);
  }

  waveform_file_info_set_state(info, STREAM_THREAD_FINISHED);
  return NULL;
}

void *adc_stream_thread(void *arg) {
  adc_file_info_t *info = (adc_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  adc_file_info_set_state(info, STREAM_THREAD_RUNNING);

  for (int iter = 0; iter < info->iters && !adc_file_info_should_stop(info); iter++) {
    FILE *fp = fopen(info->path, "r");
    if (fp == NULL) {
      fprintf(stderr, "Error: [ADC] could not reopen '%s': %s\n", info->path, strerror(errno));
      break;
    }

    char line[MAX_LINE_LEN];
    bool has_prev = false;
    double prev_timestamp = 0.0;

    while (fgets(line, sizeof(line), fp) != NULL) {
      if (adc_file_info_should_stop(info)) {
        break;
      }
      strip_comments_and_trim(line);
      if (line[0] == '\0') {
        continue;
      }

      char *cursor = line;
      double timestamp;
      if (next_double(&cursor, &timestamp) != 0) {
        continue; // already validated; shouldn't happen
      }
      double dt = has_prev ? (timestamp - prev_timestamp) : 0.0;
      prev_timestamp = timestamp;
      has_prev = true;

      printf("[ADC] iter %d dt=%g\n", iter + 1, dt);
    }

    fclose(fp);
  }

  adc_file_info_set_state(info, STREAM_THREAD_FINISHED);
  return NULL;
}

void *trigger_stream_thread(void *arg) {
  trigger_file_info_t *info = (trigger_file_info_t *)arg;
  if (info == NULL) {
    return NULL;
  }
  trigger_file_info_set_state(info, STREAM_THREAD_RUNNING);

  long total = info->num_trigs * (long)info->iters;
  for (long i = 1; i <= total; i++) {
    if (trigger_file_info_should_stop(info)) {
      break;
    }
    printf("[Trigger] trigger %ld / %ld\n", i, total);
  }

  trigger_file_info_set_state(info, STREAM_THREAD_FINISHED);
  return NULL;
}
