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

  loader_set_status(loader, FILE_LOADER_RUNNING);

  if (loader->verbose) {
    printf("File loader: starting parse of '%s'\n", loader->path);
  }

  // TODO: open and parse the shim block file at loader->path.
  // The general structure should be:
  //
  //   for each chunk of work:
  //       if (loader_should_stop(loader)) {
  //           loader_set_status(loader, FILE_LOADER_STOPPED);
  //           return NULL;
  //       }
  //       // ... process chunk, write converted DAC data to hw ...
  //
  // Call loader_should_stop() at every natural checkpoint so that
  // file_loader_request_stop() takes effect promptly.

  if (loader_should_stop(loader)) {
    if (loader->verbose) {
      printf("File loader: stop requested before completion, aborting.\n");
    }
    loader_set_status(loader, FILE_LOADER_STOPPED);
    return NULL;
  }

  if (loader->verbose) {
    printf("File loader: finished parsing '%s'\n", loader->path);
  }

  loader_set_status(loader, FILE_LOADER_DONE);
  return NULL;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialize the loader struct and its mutex. Call once before any other
// file_loader_* function. Returns 0 on success, non-zero on failure.
file_loader_t file_loader_init(hw_t *hw, bool verbose) {
  file_loader_t loader;
  memset(&loader, 0, sizeof(loader));

  loader.hw      = hw;
  loader.verbose = verbose;

  if (pthread_mutex_init(&(loader.mutex), NULL) != 0) {
    fprintf(stderr, "File loader: failed to initialize mutex.\n");
    exit(1);
  }
  loader.stop_requested = false;
  loader.status         = FILE_LOADER_IDLE;
  loader.started        = false;

  return loader;
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
  loader->status         = FILE_LOADER_IDLE;
  pthread_mutex_unlock(&loader->mutex);

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
