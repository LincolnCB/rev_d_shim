#ifndef WAVEFORM_FILE_HANDLING_H
#define WAVEFORM_FILE_HANDLING_H

#include <stdbool.h>
#include <pthread.h>

#include "waveform_hw.h"

// Channel count limits: first column is timestamp, remaining columns are
// channels, so a file needs at least MIN_CHANNELS + 1 and at most
// MAX_CHANNELS + 1 fields in its header.
#define MIN_CHANNELS 1
#define MAX_CHANNELS 64

// Allowed current range, in amps. Any value outside this range is a hard validation error.
#define CURRENT_MIN_AMPS (-3.0)
#define CURRENT_MAX_AMPS   3.0

// When the DAC command FIFO has no free space, the DAC stream thread sleeps
// this long (in microseconds) before re-checking for available command space.
#define DAC_STREAM_NO_SPACE_SLEEP_US 1000

// Lifecycle state of a stream thread. Defined up front since the *_file_info_t
// structs below all embed one via stream_ctrl_t.
typedef enum {
  STREAM_THREAD_IDLE = 0,
  STREAM_THREAD_RUNNING,
  STREAM_THREAD_STOP_REQUESTED,
  STREAM_THREAD_FINISHED
} stream_thread_state_t;

// Thread-control block shared by all three *_file_info_t structs below.
// Bundles what's needed to request a cooperative stop and poll status from
// the main thread while a stream thread runs.
typedef struct {
  pthread_mutex_t mutex;
  bool stop_requested;
  stream_thread_state_t state;
} stream_ctrl_t;

// Summary info gathered while validating the input (DAC) waveform CSV file.
// Also used to drive the DAC stream thread once validation succeeds.
typedef struct {
  const char *path;    // source file path
  int num_channels;    // number of current-value columns (excludes timestamp)
  long num_rows;       // total number of data rows in the file
  long num_trigs;      // number of trigger points (see below)
  double min_current;  // smallest current value seen across all channels
  double max_current;  // largest current value seen across all channels
  double min_dt;       // smallest non-negative gap between consecutive timestamps within a trigger;
                       //  -1.0 if it could not be computed (fewer than 2 rows in a trigger)
  int iters;           // number of times to replay the file; set by caller before streaming
  hw_t *hw;            // hardware handle used to stream commands; set by caller before streaming
  stream_ctrl_t ctrl;
} waveform_file_info_t;

// Summary info gathered while validating the ADC file. The ADC file is just
// a list of sample timestamps (one per line, no header, no channel data).
// Also used to drive the ADC stream thread once validation succeeds.
typedef struct {
  const char *path;    // source file path
  long num_rows;       // total number of sample timestamps in the file
  long num_trigs;      // number of trigger points (see below)
  double min_dt;       // smallest non-negative gap between consecutive timestamps within a trigger;
                       //  -1.0 if it could not be computed (fewer than 2 rows in a trigger)
  int iters;           // number of times to replay the file; set by caller before streaming
  stream_ctrl_t ctrl;
} adc_file_info_t;

// Info driving the trigger stream. Unlike the DAC/ADC files there's no file
// to read here: the per-iteration trigger count comes from the DAC file's
// trigger count (num_trigs), and label is purely for log messages.
typedef struct {
  const char *label;   // display label, e.g. "Trigger"
  long num_trigs;      // trigger points per iteration (matches the DAC file)
  int iters;            // number of times to replay
  stream_ctrl_t ctrl;
} trigger_file_info_t;


// Validate the input (DAC) waveform CSV file.
//
// Checks that the file exists and is readable, has a header row followed by
// at least one data row, that every row has the same number of fields as the
// header, that the header describes between MIN_CHANNELS and MAX_CHANNELS
// channels, that all timestamps (assumed to be the first column) are
// non-negative, and that all current values (remaining columns) fall within
// [CURRENT_MIN_AMPS, CURRENT_MAX_AMPS].
//
// A row is a trigger point if it is the first row of the file, or if its
// timestamp is lower than the preceding row's timestamp (i.e. time has
// reset, marking the start of a new sweep). info->num_trigs counts these.
// info->min_dt is the smallest non-negative gap between a row's timestamp
// and the previous row's timestamp (gaps at trigger-point resets are
// negative and are not counted).
//
// On success, fills *info (with info->iters left at 0 -- set it before
// starting dac_stream_thread) and returns 0. On failure, prints an error
// (including the offending line number where applicable) to stderr and
// returns -1.

int validate_input_file(const char *path, waveform_file_info_t *info);


// Validate the ADC file. The ADC file is just a list of sample timestamps:
// one non-negative timestamp per line, no header, no channel columns.
//
// Uses the same trigger-point rule as the input file: a line is a trigger
// point if it's the first line, or its timestamp is lower than the
// preceding line's (time has reset, marking a new sweep). Also tracks the
// smallest non-negative gap between consecutive timestamps within a
// trigger (info->min_dt).
//
// Checks that the resulting trigger count matches expected_trigs (i.e. the
// input file's trigger count).
//
// On success, fills *info (with info->iters left at 0 -- set it before
// starting adc_stream_thread) and returns 0. On failure, prints an error
// (including the offending line number where applicable) to stderr and
// returns -1.

int validate_adc_file(const char *path, long expected_trigs, adc_file_info_t *info);

// Initialize a trigger_file_info_t. num_trigs should match the DAC file's
// trigger count; iters is the number of times the whole run will be
// replayed. Must be called before starting trigger_stream_thread.
void trigger_file_info_init(trigger_file_info_t *info, const char *label, long num_trigs, int iters);

void waveform_file_info_destroy(waveform_file_info_t *info);
void waveform_file_info_request_stop(waveform_file_info_t *info);
bool waveform_file_info_should_stop(const waveform_file_info_t *info);
bool waveform_file_info_is_finished(const waveform_file_info_t *info);

void adc_file_info_destroy(adc_file_info_t *info);
void adc_file_info_request_stop(adc_file_info_t *info);
bool adc_file_info_should_stop(const adc_file_info_t *info);
bool adc_file_info_is_finished(const adc_file_info_t *info);

void trigger_file_info_destroy(trigger_file_info_t *info);
void trigger_file_info_request_stop(trigger_file_info_t *info);
bool trigger_file_info_should_stop(const trigger_file_info_t *info);
bool trigger_file_info_is_finished(const trigger_file_info_t *info);

// Stream thread entry points, intended for use with pthread_create().
//
// Each one replays its source `iters` times (dac/adc re-read their file
// from disk on each pass; trigger just counts). For each DAC/ADC line, it
// computes dt from the previous line within that pass (0.0 for the first
// line of a pass) and, for DAC, also parses the channel values, then
// prints them. The trigger thread counts up to num_trigs * iters, printing
// each trigger number. This is a placeholder for real streaming logic.
//
// Each thread sets its own state to STREAM_THREAD_FINISHED and returns
// when its work is done, or earlier if a stop is requested via the
// corresponding *_file_info_request_stop().
void *dac_stream_thread(void *arg);
void *adc_stream_thread(void *arg);
void *trigger_stream_thread(void *arg);

#endif // WAVEFORM_FILE_HANDLING_H
