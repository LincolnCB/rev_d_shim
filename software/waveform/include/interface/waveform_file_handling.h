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

// When the ADC command FIFO has no free space, the ADC stream thread sleeps
// this long (in microseconds) before re-checking for available command space.
#define ADC_STREAM_NO_SPACE_SLEEP_US 1000

// When the ADC data buffer has no samples ready, the ADC data stream thread
// sleeps this long (in microseconds) before polling for available samples again.
#define ADC_DATA_STREAM_NO_DATA_SLEEP_US 1000

// When the trigger data buffer has no samples ready, the trigger data stream
// thread sleeps this long (in microseconds) before polling for samples again.
#define TRIGGER_DATA_STREAM_NO_DATA_SLEEP_US 1000

// Lifecycle state of a stream thread. Defined up front since the *_file_info_t
// structs below all embed one via stream_ctrl_t.
typedef enum {
  STREAM_THREAD_IDLE = 0,
  STREAM_THREAD_RUNNING,
  STREAM_THREAD_STOP_REQUESTED,
  STREAM_THREAD_COMPLETED,   // terminal: ran to completion (all iterations sent)
  STREAM_THREAD_STOPPED      // terminal: ended early (stop requested or hardware error)
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
  hw_t *hw;            // hardware handle used to stream commands; set by caller before streaming
  stream_ctrl_t ctrl;
} adc_file_info_t;

// Info driving the ADC data stream. This is an output stream: once the ADC
// command stream is running, it polls the ADC data buffer for available
// samples, reads them, and writes them to an output file. It runs only when
// the ADC command stream runs.
//
// There's no input file to validate here: num_samples (the samples expected
// per iteration) comes from the ADC file's row count, and path is the output
// file to write, set by the caller before streaming.
typedef struct {
  const char *path;    // output file path; set by caller before streaming
  long num_samples;    // samples expected per iteration (matches the ADC file rows)
  int iters;           // number of times the run is replayed
  hw_t *hw;            // hardware handle used to read samples; set by caller before streaming
  stream_ctrl_t ctrl;
} adc_data_file_info_t;

// Info driving the trigger data stream. This is an output stream: it polls the
// trigger data buffer for logged trigger timepoints, reads them, and writes
// them to an output file. Unlike the DAC/ADC command files there's no input
// file here: the per-iteration trigger count comes from the DAC file's trigger
// count (num_trigs), path is the output file to write (set by the caller before
// streaming), and label is purely for log messages.
typedef struct {
  const char *label;   // display label, e.g. "Trigger"
  const char *path;    // output file path; set by caller before streaming
  long num_trigs;      // trigger points per iteration (matches the DAC file)
  int iters;            // number of times to replay
  hw_t *hw;            // hardware handle used to read samples; set by caller before streaming
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

// Initialize an adc_data_file_info_t. num_samples should match the ADC file's
// row count; iters is the number of times the whole run will be replayed. The
// caller must set ->hw and ->path before starting adc_data_stream_thread.
void adc_data_file_info_init(adc_data_file_info_t *info, long num_samples, int iters);

// Initialize a trigger_file_info_t. num_trigs should match the DAC file's
// trigger count; iters is the number of times the whole run will be replayed.
// The caller must set ->hw and ->path before starting trigger_stream_thread.
void trigger_file_info_init(trigger_file_info_t *info, const char *label, long num_trigs, int iters);

void waveform_file_info_destroy(waveform_file_info_t *info);
void waveform_file_info_request_stop(waveform_file_info_t *info);
bool waveform_file_info_should_stop(const waveform_file_info_t *info);
bool waveform_file_info_is_finished(const waveform_file_info_t *info);
bool waveform_file_info_stopped_early(const waveform_file_info_t *info);

void adc_file_info_destroy(adc_file_info_t *info);
void adc_file_info_request_stop(adc_file_info_t *info);
bool adc_file_info_should_stop(const adc_file_info_t *info);
bool adc_file_info_is_finished(const adc_file_info_t *info);
bool adc_file_info_stopped_early(const adc_file_info_t *info);

void adc_data_file_info_destroy(adc_data_file_info_t *info);
void adc_data_file_info_request_stop(adc_data_file_info_t *info);
bool adc_data_file_info_should_stop(const adc_data_file_info_t *info);
bool adc_data_file_info_is_finished(const adc_data_file_info_t *info);
bool adc_data_file_info_stopped_early(const adc_data_file_info_t *info);

void trigger_file_info_destroy(trigger_file_info_t *info);
void trigger_file_info_request_stop(trigger_file_info_t *info);
bool trigger_file_info_should_stop(const trigger_file_info_t *info);
bool trigger_file_info_is_finished(const trigger_file_info_t *info);
bool trigger_file_info_stopped_early(const trigger_file_info_t *info);

// Stream thread entry points, intended for use with pthread_create().
//
// There are four streams, two input (command) and two output (data):
//   - dac_stream_thread     (input)  replays the DAC command file to hardware.
//   - adc_stream_thread     (input)  replays the ADC command file to hardware.
//   - adc_data_stream_thread (output) drains the ADC data buffer, reading each
//                                    sample and writing it to its output file.
//   - trigger_stream_thread (output) drains the trigger data buffer, reading
//                                    each logged timepoint and writing it to its
//                                    output file.
//
// The two input threads re-read their command file from disk on each of the
// `iters` passes. The two output threads poll the hardware buffer for available
// samples and read up to their expected count (num_samples * iters for ADC
// data, num_trigs * iters for trigger data), draining the buffer so it does
// not overflow; when their output path is set they also write each sample out.
//
// Each thread sets its own terminal state and returns when its work is done:
// STREAM_THREAD_COMPLETED if it processed every iteration, or STREAM_THREAD_STOPPED
// if it ended early because a stop was requested (via the corresponding
// *_file_info_request_stop()) or a hardware command failed. Callers can tell
// the two apart with *_file_info_stopped_early().
void *dac_stream_thread(void *arg);
void *adc_stream_thread(void *arg);
void *adc_data_stream_thread(void *arg);
void *trigger_stream_thread(void *arg);

#endif // WAVEFORM_FILE_HANDLING_H
