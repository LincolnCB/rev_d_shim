#ifndef WAVEFORM_FILE_HANDLING_H
#define WAVEFORM_FILE_HANDLING_H

/* Channel count limits: first column is timestamp, remaining columns are
 * channels, so a file needs at least MIN_CHANNELS + 1 and at most
 * MAX_CHANNELS + 1 fields in its header. */
#define MIN_CHANNELS 1
#define MAX_CHANNELS 64

/* Allowed current range, in amps. Any value outside this range is a hard
 * validation error. */
#define CURRENT_MIN_AMPS (-3.0)
#define CURRENT_MAX_AMPS   3.0

/* Summary info gathered while validating the input (DAC) waveform CSV
 * file. */
typedef struct {
    int num_channels;    /* number of current-value columns (excludes timestamp) */
    long num_rows;        /* total number of data rows in the file */
    long num_trigs;        /* number of trigger points (see below) */
    double min_current;   /* smallest current value seen across all channels */
    double max_current;   /* largest current value seen across all channels */
    double min_dt;         /* smallest non-negative gap between consecutive
                             * timestamps within a trigger; -1.0 if it could
                             * not be computed (fewer than 2 rows in a
                             * trigger) */
} waveform_file_info_t;

/* Summary info gathered while validating the ADC file. The ADC file is just
 * a list of sample timestamps (one per line, no header, no channel data). */
typedef struct {
    long num_rows;   /* total number of sample timestamps in the file */
    long num_trigs;   /* number of trigger points (see below) */
    double min_dt;     /* smallest non-negative gap between consecutive
                         * timestamps within a trigger; -1.0 if it could not
                         * be computed (fewer than 2 rows in a trigger) */
} adc_file_info_t;

/*
 * Validate the input (DAC) waveform CSV file.
 *
 * Checks that the file exists and is readable, has a header row followed by
 * at least one data row, that every row has the same number of fields as the
 * header, that the header describes between MIN_CHANNELS and MAX_CHANNELS
 * channels, that all timestamps (assumed to be the first column) are
 * non-negative, and that all current values (remaining columns) fall within
 * [CURRENT_MIN_AMPS, CURRENT_MAX_AMPS].
 *
 * A row is a trigger point if it is the first row of the file, or if its
 * timestamp is lower than the preceding row's timestamp (i.e. time has
 * reset, marking the start of a new sweep). info->num_trigs counts these.
 *
 * info->min_dt is the smallest non-negative gap between a row's timestamp
 * and the previous row's timestamp (gaps at trigger-point resets are
 * negative and are not counted).
 *
 * On success, fills *info and returns 0. On failure, prints an error
 * (including the offending line number where applicable) to stderr and
 * returns -1.
 */
int validate_input_file(const char *path, waveform_file_info_t *info);

/*
 * Validate the ADC file. The ADC file is just a list of sample timestamps:
 * one non-negative timestamp per line, no header, no channel columns.
 *
 * Uses the same trigger-point rule as the input file: a line is a trigger
 * point if it's the first line, or its timestamp is lower than the
 * preceding line's (time has reset, marking a new sweep). Also tracks the
 * smallest non-negative gap between consecutive timestamps within a
 * trigger (info->min_dt).
 *
 * Checks that the resulting trigger count matches expected_trigs (i.e. the
 * input file's trigger count).
 *
 * On success, fills *info and returns 0. On failure, prints an error
 * (including the offending line number where applicable) to stderr and
 * returns -1.
 */
int validate_adc_file(const char *path, long expected_trigs, adc_file_info_t *info);

/* Argument passed to each stream thread. path may be NULL if the stream has
 * no associated file (e.g. the trigger stream). */
typedef struct {
    const char *label;  /* e.g. "DAC", "ADC", "Trigger" -- used in log output */
    const char *path;
} stream_thread_arg_t;

/*
 * Stream thread entry points, intended for use with pthread_create().
 *
 * Each one currently "opens" the stream, sleeps for 1 second three times to
 * stand in for real streaming work, prints "Completed", and "closes" the
 * stream. Replace the body of run_stream() in the .c file with real
 * streaming logic when it's ready.
 */
void *dac_stream_thread(void *arg);
void *adc_stream_thread(void *arg);
void *trigger_stream_thread(void *arg);

#endif /* WAVEFORM_FILE_HANDLING_H */
