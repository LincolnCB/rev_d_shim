#include "file_handling.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Read stop_requested under the mutex. Called from inside the loader thread.
static bool loader_should_stop(file_loader_t *loader) {
  bool stop;
  pthread_mutex_lock(&loader->mutex);
  stop = loader->stop_requested;
  pthread_mutex_unlock(&loader->mutex);
  return stop;
}

// Write status under the mutex. Called from inside the loader thread.
static void loader_set_status(file_loader_t *loader, file_loader_status_t status) {
  pthread_mutex_lock(&loader->mutex);
  loader->status = status;
  pthread_mutex_unlock(&loader->mutex);
}

// ---------------------------------------------------------------------------
// Loader thread
// ---------------------------------------------------------------------------

static void *loader_thread_fn(void *arg) {
  file_loader_t *loader = (file_loader_t *)arg;

  loader_set_status(loader, FILE_LOADER_LOADED);

  if (loader->verbose) {
    printf("File loader: starting '%s'\n", loader->path);
  }

  // Open the shim block file at loader->path and loop continuously
  // Block file format:
  // CSV (comma OR space separated) with one row per trigger, one column per DAC channel.
  // Each value is a floating-point number in amps, from -5.0 to 5.0.
  // By default, the entire file will loop until file_loader_request_stop() is called.
  // However, by adding lines that look like "x3" or "x5", you can delimit a block that will loop that many times.
  // A block will be defined as the rows between two "xN" lines (or file boundaries).
  // For instance a file like:
  //   0.0, 0.0
  //   1.0, 1.0
  //   x3
  //   0.0, 0.0
  // will loop through the first two rows three times, then play the last row once, 
  // then repeat the entire file from the top.
  // This CAN be done recursively, where two "xN" lines in a row will make the second one a second-level delimiter.
  // This will be bounded by the last second-level delimiter, so a file like:
  //   0.0, 0.0
  //   1.0, 1.0
  //   x3
  //   0.0, 0.0
  //   x5
  //   x2
  //   1.0, 1.0
  // will loop through the first two rows three times, then the next row five times, 
  // THEN repeat that twice before playing the last row once, and then repeat the entire file from the top.
  // You can use "x1" as a dummy delimiter (play once) to bound blocks or for adding levels to delimiters.
  // N needs to be a positive integer
  FILE *f = fopen(loader->path, "r");
  if (f == NULL) {
    fprintf(stderr, "File loader: failed to open '%s'\n", loader->path);
    loader_set_status(loader, FILE_LOADER_ERROR);
    return NULL;
  }

  // ---------------------------------------------------------------------------
  // Block/level tracking
  //
  // Levels are 0-indexed: level 0 is the innermost (a single "xN" line),
  // level 1 is formed by two consecutive delimiters, etc., up to FILE_LOOP_MAX_LEVEL-1.
  //
  // For each level we track:
  //   block_start[L]  -- file offset where the current level-L block began
  //   iters_left[L]   -- how many more times to loop the current level-L block
  //                      (0 means "not yet set / fall through")
  //
  // Rule: when we see a delimiter "xN" immediately after finishing a block
  // (i.e. the previous non-blank content was also a delimiter), we promote it
  // one level higher.  Concretely, we track `last_was_delimiter` and the level
  // it was at; a new delimiter goes to last_level + 1.
  //
  // When a delimiter at level L fires (iters_left[L] reaches 0):
  //   - seek back to block_start[L]
  //   - reset all levels below L (they will be re-discovered on the next pass)
  //
  // When all iterations of level L are exhausted, execution falls through to
  // the next line (no seek), and block_start[L] advances to just past this
  // delimiter so it is ready to catch the next same-level block.
  //
  // At EOF / outer loop wrap: reset everything and seek to file start.
  // ---------------------------------------------------------------------------


  long  block_start[FILE_LOOP_MAX_LEVEL];
  int   iters_left[FILE_LOOP_MAX_LEVEL];
  bool  level_active[FILE_LOOP_MAX_LEVEL]; // true once a delimiter has set this level

  // Initialize: all levels begin at file start, none active.
  for (int l = 0; l < FILE_LOOP_MAX_LEVEL; l++) {
    block_start[l]  = 0;
    iters_left[l]   = 0;
    level_active[l] = false;
  }

  int  last_delim_level = -1; // level of the most recent delimiter line (-1 = last line was data)
  char line[1024];

  // Outer loop: repeats the whole file indefinitely until stop is requested.
  while (!loader_should_stop(loader)) {

    if (fgets(line, sizeof(line), f) == NULL) {
      // EOF: treat like the outermost delimiter fired -- reset all levels,
      // seek back to file start, and begin again.
      fseek(f, 0, SEEK_SET);
      for (int l = 0; l < FILE_LOOP_MAX_LEVEL; l++) {
        block_start[l]  = 0;
        iters_left[l]   = 0;
        level_active[l] = false;
      }
      last_delim_level = -1;
      continue;
    }

    // Trim trailing newline/whitespace for easier parsing.
    line[strcspn(line, "\r\n")] = '\0';

    // Skip blank lines.
    if (line[0] == '\0') continue;

    // ------------------------------------------------------------------
    // Check if this line is a delimiter ("xN" where N is a positive int).
    // ------------------------------------------------------------------
    int repeat_count = 0;
    bool is_delimiter = (line[0] == 'x' || line[0] == 'X') &&
                        sscanf(line + 1, "%d", &repeat_count) == 1 &&
                        repeat_count > 0;

    if (is_delimiter) {
      // Determine which level this delimiter belongs to.
      // Two consecutive delimiters: the new one is one level higher than the last.
      // A data line resets the "consecutive" chain back to -1.
      int level = last_delim_level + 1; // promotes by 1 each time delimiters stack
      if (level >= FILE_LOOP_MAX_LEVEL) {
        // The pre-flight check in loader_start should have caught this; if we
        // still hit it (e.g. file changed after validation), error out rather
        // than silently miscounting levels.
        fprintf(stderr,
                "File loader: '%s' exceeded FILE_LOOP_MAX_LEVEL (%d) while running -- "
                "stopping.\n",
                loader->path, FILE_LOOP_MAX_LEVEL);
        fclose(f);
        loader_set_status(loader, FILE_LOADER_ERROR);
        return NULL;
      }

      if (!level_active[level]) {
        // First time we've seen a delimiter at this level: set up the block.
        // The block starts at the beginning of the file for level > 0 that
        // hasn't been set yet -- but block_start was already initialised to 0,
        // so we just activate it.
        iters_left[level]   = repeat_count - 1; // -1 because we already ran through once
        level_active[level] = true;
        // block_start[level] was set either at init (file start) or when this
        // level last fell through (see below), so it correctly points to the
        // beginning of this level's block.
      } else {
        // We've been here before: decrement the counter.
        iters_left[level]--;
      }

      if (iters_left[level] > 0) {
        // More iterations remain: seek back to the start of this block and
        // reset all lower levels so they are re-discovered cleanly.
        fseek(f, block_start[level], SEEK_SET);
        for (int l = 0; l < level; l++) {
          block_start[l]  = block_start[level]; // lower levels start fresh from here
          iters_left[l]   = 0;
          level_active[l] = false;
        }
        last_delim_level = -1; // after the seek we're back to "just saw data"
      } else {
        // All iterations exhausted: fall through.
        // Advance this level's block_start to just past this delimiter,
        // ready to catch the next block at the same level.
        // Also deactivate it so the next delimiter at this level starts fresh.
        block_start[level]  = ftell(f);
        iters_left[level]   = 0;
        level_active[level] = false;

        // Reset all lower levels to start from the new position too.
        for (int l = 0; l < level; l++) {
          block_start[l]  = block_start[level];
          iters_left[l]   = 0;
          level_active[l] = false;
        }
      }

      // Record this delimiter's level so the next consecutive delimiter can
      // promote itself one level higher.
      last_delim_level = level;

    } else {
      // ------------------------------------------------------------------
      // Data line: process it (placeholder -- replace with DAC output).
      // ------------------------------------------------------------------
      printf("Line: %s\n", line);
      usleep(500000); // 0.5 s placeholder; remove when wiring up real DAC writes

      last_delim_level = -1; // data line breaks any delimiter chain
    }

    if (loader_should_stop(loader)) break;
  }

  fclose(f);

  if (loader->verbose) {
    printf("File loader: exiting '%s'\n", loader->path);
  }

  loader_set_status(loader, FILE_LOADER_EMPTY);
  return NULL;
}

// ---------------------------------------------------------------------------
// Pre-flight validation
// ---------------------------------------------------------------------------

// Scan the file at `path` and report the maximum delimiter nesting depth found.
// Depth is measured by the longest consecutive run of delimiter lines ("xN"),
// since each additional consecutive delimiter promotes to the next level.
// Returns 0 on success (max_level_out is set), -1 if the file cannot be opened.
static int file_check_max_level(const char *path, int *max_level_out) {
  FILE *f = fopen(path, "r");
  if (f == NULL) {
    return -1;
  }

  char line[1024];
  int current_run = 0; // consecutive delimiter lines seen so far
  int max_run     = 0; // longest such run (= highest level index reached)

  while (fgets(line, sizeof(line), f) != NULL) {
    line[strcspn(line, "\r\n")] = '\0';
    if (line[0] == '\0') continue; // blank line -- doesn't break the run

    int repeat_count = 0;
    bool is_delimiter = (line[0] == 'x' || line[0] == 'X') &&
                        sscanf(line + 1, "%d", &repeat_count) == 1 &&
                        repeat_count > 0;

    if (is_delimiter) {
      current_run++;
      if (current_run > max_run) max_run = current_run;
    } else {
      current_run = 0; // data line resets the consecutive-delimiter chain
    }
  }

  fclose(f);
  *max_level_out = max_run;
  return 0;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize the loader struct and its mutex. Call once before any other
// file_loader_* function. Returns 0 on success, non-zero on failure.
file_loader_t file_loader_init(hw_t *hw, double trigger_lockout_ms, bool verbose) {
  file_loader_t loader;
  memset(&loader, 0, sizeof(loader));

  loader.hw      = hw;
  loader.trigger_lockout_ms = trigger_lockout_ms;
  loader.verbose = verbose;

  if (pthread_mutex_init(&(loader.mutex), NULL) != 0) {
    fprintf(stderr, "File loader: failed to initialize mutex.\n");
    exit(1);
  }
  loader.stop_requested = false;
  loader.status         = FILE_LOADER_EMPTY;
  loader.started        = false;

  return loader;
}

// Set the loader's trigger_lockout_ms value. Must be called before file_loader_start() and not while the loader is running.
int file_loader_set_trigger_lockout(file_loader_t *loader, double trigger_lockout_ms) {
  if (loader == NULL) {
    return -1;
  }
  pthread_mutex_lock(&loader->mutex);
  if (loader->started) {
    fprintf(stderr, "File loader: cannot set trigger_lockout_ms while loader is running.\n");
    pthread_mutex_unlock(&loader->mutex);
    return -1;
  }
  loader->trigger_lockout_ms = trigger_lockout_ms;
  pthread_mutex_unlock(&loader->mutex);
  return 0;
}

// Destroy the loader's mutex. Call after the thread has been joined and the
// loader is no longer needed.
void file_loader_destroy(file_loader_t *loader) {
  if (loader == NULL) {
    return;
  }
  pthread_mutex_destroy(&loader->mutex);
}

// Spawn the loader thread for the file at loader->path. The caller must have
// already set loader->path, loader->hw, and loader->verbose.
// Returns 0 on success, non-zero if the thread could not be created.
int file_loader_start(file_loader_t *loader) {
  if (loader == NULL || loader->hw == NULL || loader->path[0] == '\0') {
    return -1;
  }

  pthread_mutex_lock(&loader->mutex);
  if (loader->started) {
    // A thread handle is already outstanding -- caller must
    // request_stop() + join() before starting again, or we'd overwrite
    // 'thread' and leak the handle.
    pthread_mutex_unlock(&loader->mutex);
    return -1;
  }
  // Reset flags so this struct can be reused for a fresh load.
  loader->stop_requested = false;
  loader->status         = FILE_LOADER_EMPTY;
  pthread_mutex_unlock(&loader->mutex);

  // Validate the file's nesting depth before spawning the thread so we can
  // return -1 directly to the caller if it exceeds FILE_LOOP_MAX_LEVEL.
  int max_level = 0;
  if (file_check_max_level(loader->path, &max_level) != 0) {
    fprintf(stderr, "File loader: failed to open '%s' for validation.\n", loader->path);
    return -1;
  }
  if (max_level > FILE_LOOP_MAX_LEVEL) {
    fprintf(stderr,
            "File loader: '%s' uses %d nesting levels but FILE_LOOP_MAX_LEVEL is %d.\n",
            loader->path, max_level, FILE_LOOP_MAX_LEVEL);
    return -1;
  }

  if (pthread_create(&loader->thread, NULL, loader_thread_fn, loader) != 0) {
    fprintf(stderr, "File loader: failed to create loader thread.\n");
    return -1;
  }

  pthread_mutex_lock(&loader->mutex);
  loader->started = true;
  pthread_mutex_unlock(&loader->mutex);
  return 0;
}

// Signal the loader thread to stop at the next safe checkpoint. Returns
// immediately; the thread may still be running when this returns. Call
// file_loader_join() afterward to wait for it to finish.
// Thread-safe: may be called from any thread.
void file_loader_request_stop(file_loader_t *loader) {
  if (loader == NULL) {
    return;
  }
  pthread_mutex_lock(&loader->mutex);
  loader->stop_requested = true;
  pthread_mutex_unlock(&loader->mutex);
}

// Block until the loader thread has exited. Safe to call even if the thread
// finished on its own. Returns the thread's exit status via *status_out
// (pass NULL to ignore).
void file_loader_join(file_loader_t *loader, file_loader_status_t *status_out) {
  if (loader == NULL) {
    return;
  }

  pthread_mutex_lock(&loader->mutex);
  bool was_started = loader->started;
  pthread_mutex_unlock(&loader->mutex);

  if (was_started) {
    pthread_join(loader->thread, NULL);
    pthread_mutex_lock(&loader->mutex);
    loader->started = false;
    pthread_mutex_unlock(&loader->mutex);
  }
  if (status_out != NULL) {
    *status_out = file_loader_get_status(loader);
  }
}

// Read the current status without blocking. Thread-safe.
file_loader_status_t file_loader_get_status(file_loader_t *loader) {
  if (loader == NULL) {
    return FILE_LOADER_ERROR;
  }
  pthread_mutex_lock(&loader->mutex);
  file_loader_status_t status = loader->status;
  pthread_mutex_unlock(&loader->mutex);
  return status;
}

// Append CSV-formatted ADC amp readings to the file at path. Returns 0 on success, non-zero on failure.
int file_append_adc_dump(const char *path, const double *adc_values, uint32_t channel_count) {
  if (path == NULL || adc_values == NULL) {
    return -1;
  }

  FILE *f = fopen(path, "a");
  if (f == NULL) {
    fprintf(stderr, "Failed to open ADC dump file '%s' for appending.\n", path);
    return -1;
  }

  // Write ADC values as a single CSV line: ch0_amps, ch1_amps, ...
  for (uint32_t ch = 0; ch < channel_count; ch++) {
    if (ch > 0) {
      fprintf(f, ",%.4f", adc_values[ch]);
    } else {
      fprintf(f, "%.4f", adc_values[ch]);
    }
  }
  fprintf(f, "\n");
  fclose(f);
  return 0;
}
