/*
 * waveform_file_handling.c
 *
 * File validation helpers and the stream worker threads used by waveform.c.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>

#include "waveform_file_handling.h"

#define MAX_LINE_LEN 4096

/* Count the comma-separated fields in a line (ignores trailing newline).
 * Returns 0 for an empty/blank line. */
static int count_fields(const char *line) {
    if (line == NULL || line[0] == '\0' || line[0] == '\n' || line[0] == '\r') {
        return 0;
    }
    int fields = 1;
    for (const char *p = line; *p != '\0' && *p != '\n' && *p != '\r'; p++) {
        if (*p == ',') {
            fields++;
        }
    }
    return fields;
}

/* Parse the next comma-separated double out of *cursor, advancing *cursor
 * past it. Returns 0 on success, -1 if the field is missing or not a valid
 * number. */
static int next_double(char **cursor, double *out) {
    if (*cursor == NULL || **cursor == '\0') {
        return -1;
    }
    char *comma = strchr(*cursor, ',');
    if (comma != NULL) {
        *comma = '\0';
    }
    char *end;
    errno = 0;
    double val = strtod(*cursor, &end);
    if (errno != 0 || end == *cursor) {
        return -1;
    }
    *cursor = (comma != NULL) ? comma + 1 : *cursor + strlen(*cursor);
    *out = val;
    return 0;
}

/* Core validator shared by validate_input_file() and validate_adc_file().
 * file_kind is used only for error messages, e.g. "input file" or
 * "ADC file". */
static int validate_waveform_csv(const char *path, const char *file_kind, waveform_file_info_t *info) {
    memset(info, 0, sizeof(*info));

    if (access(path, R_OK) != 0) {
        fprintf(stderr, "Error: %s '%s' does not exist or is not readable\n", file_kind, path);
        return -1;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error: could not open %s '%s': %s\n", file_kind, path, strerror(errno));
        return -1;
    }

    char line[MAX_LINE_LEN];
    if (fgets(line, sizeof(line), fp) == NULL) {
        fprintf(stderr, "Error: %s '%s' is empty\n", file_kind, path);
        fclose(fp);
        return -1;
    }

    /* Assume header format: timestamp,ch1,ch2,...,chN */
    int fields = count_fields(line);
    int num_channels = fields - 1;
    if (num_channels < MIN_CHANNELS || num_channels > MAX_CHANNELS) {
        fprintf(stderr,
                "Error: %s '%s' has %d channel(s) in its header, must be between %d and %d\n",
                file_kind, path, num_channels, MIN_CHANNELS, MAX_CHANNELS);
        fclose(fp);
        return -1;
    }

    long num_rows = 0;
    long num_trigs = 0;
    double prev_timestamp = 0.0;
    bool has_prev = false;
    double min_current = 0.0;
    double max_current = 0.0;
    bool has_current = false;
    double min_dt = -1.0;
    bool has_min_dt = false;

    while (fgets(line, sizeof(line), fp) != NULL) {
        if (line[0] == '\n' || line[0] == '\0' || line[0] == '\r') {
            continue; /* skip blank lines */
        }

        long line_num = num_rows + 2; /* header is line 1 */

        int line_fields = count_fields(line);
        if (line_fields != fields) {
            fprintf(stderr, "Error: %s '%s' line %ld has %d field(s), expected %d\n",
                    file_kind, path, line_num, line_fields, fields);
            fclose(fp);
            return -1;
        }

        char *cursor = line;

        double timestamp;
        if (next_double(&cursor, &timestamp) != 0) {
            fprintf(stderr, "Error: %s '%s' line %ld has an invalid timestamp\n", file_kind, path, line_num);
            fclose(fp);
            return -1;
        }
        if (timestamp < 0.0) {
            fprintf(stderr, "Error: %s '%s' line %ld has a negative timestamp (%g)\n",
                    file_kind, path, line_num, timestamp);
            fclose(fp);
            return -1;
        }

        /* A row is a trigger point if it's the first row, or if time has
         * reset (this row's timestamp is lower than the previous row's),
         * marking the start of a new sweep. */
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

        for (int c = 0; c < num_channels; c++) {
            double current;
            if (next_double(&cursor, &current) != 0) {
                fprintf(stderr, "Error: %s '%s' line %ld has an invalid value in channel %d\n",
                        file_kind, path, line_num, c + 1);
                fclose(fp);
                return -1;
            }
            if (current < CURRENT_MIN_AMPS || current > CURRENT_MAX_AMPS) {
                fprintf(stderr,
                        "Error: %s '%s' line %ld channel %d value %g A is outside the allowed range [%g, %g] A\n",
                        file_kind, path, line_num, c + 1, current, CURRENT_MIN_AMPS, CURRENT_MAX_AMPS);
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
        fprintf(stderr, "Error: %s '%s' has a header but no data rows\n", file_kind, path);
        return -1;
    }

    info->num_channels = num_channels;
    info->num_rows = num_rows;
    info->num_trigs = num_trigs;
    info->min_current = min_current;
    info->max_current = max_current;
    info->min_dt = min_dt; /* -1.0 if never computed (e.g. every row was a trigger point) */
    return 0;
}

int validate_input_file(const char *path, waveform_file_info_t *info) {
    return validate_waveform_csv(path, "input file", info);
}

int validate_adc_file(const char *path, long expected_trigs, adc_file_info_t *info) {
    memset(info, 0, sizeof(*info));

    if (access(path, R_OK) != 0) {
        fprintf(stderr, "Error: ADC file '%s' does not exist or is not readable\n", path);
        return -1;
    }

    FILE *fp = fopen(path, "r");
    if (fp == NULL) {
        fprintf(stderr, "Error: could not open ADC file '%s': %s\n", path, strerror(errno));
        return -1;
    }

    char line[MAX_LINE_LEN];
    long num_rows = 0;
    long num_trigs = 0;
    double prev_timestamp = 0.0;
    bool has_prev = false;
    double min_dt = -1.0;
    bool has_min_dt = false;

    while (fgets(line, sizeof(line), fp) != NULL) {
        if (line[0] == '\n' || line[0] == '\0' || line[0] == '\r') {
            continue; /* skip blank lines */
        }

        long line_num = num_rows + 1; /* no header row, so the first sample is line 1 */

        int line_fields = count_fields(line);
        if (line_fields != 1) {
            fprintf(stderr,
                    "Error: ADC file '%s' line %ld has %d field(s), expected exactly 1 (timestamp only)\n",
                    path, line_num, line_fields);
            fclose(fp);
            return -1;
        }

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

        /* Same trigger-point rule as the input file: first line, or a time
         * reset, marks the start of a new sweep. */
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

    info->num_rows = num_rows;
    info->num_trigs = num_trigs;
    info->min_dt = min_dt; /* -1.0 if never computed (e.g. every line was a trigger point) */

    if (info->num_trigs != expected_trigs) {
        fprintf(stderr,
                "Error: ADC file '%s' has %ld trigger point(s), expected %ld to match the input file\n",
                path, info->num_trigs, expected_trigs);
        return -1;
    }

    return 0;
}

/* Shared body for all three stream threads. Currently a stand-in for real
 * streaming logic: "opens" the stream, sleeps 1 second three times, prints
 * "Completed", then "closes" the stream. */
static void run_stream(const char *label, const char *path) {
    if (path != NULL) {
        printf("[%s] Opening stream: %s\n", label, path);
    } else {
        printf("[%s] Opening stream\n", label);
    }

    for (int i = 0; i < 3; i++) {
        sleep(1);
    }

    printf("[%s] Completed\n", label);
    printf("[%s] Closing stream\n", label);
}

void *dac_stream_thread(void *arg) {
    stream_thread_arg_t *targ = (stream_thread_arg_t *)arg;
    run_stream(targ->label, targ->path);
    return NULL;
}

void *adc_stream_thread(void *arg) {
    stream_thread_arg_t *targ = (stream_thread_arg_t *)arg;
    run_stream(targ->label, targ->path);
    return NULL;
}

void *trigger_stream_thread(void *arg) {
    stream_thread_arg_t *targ = (stream_thread_arg_t *)arg;
    run_stream(targ->label, targ->path);
    return NULL;
}
