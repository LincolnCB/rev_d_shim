#ifndef FILE_HANDLING_H
#define FILE_HANDLING_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>


#include "hardware.h"

#define FILE_HANDLING_PATH_MAX 256
#define FILE_LOOP_MAX_LEVEL 10  // maximum supported delimiter nesting depth


// Status of the file loader, readable from outside the thread.
typedef enum {
  FILE_LOADER_EMPTY,      // No file is currently loaded
  FILE_LOADER_LOADED,     // Thread is active and looping through the file
  FILE_LOADER_ERROR,      // Thread encountered a fatal error
} file_loader_status_t;

// All state for one file load operation. Embed one of these wherever a load
// can be in flight (e.g. as a static local in run_load, or in a larger
// context struct). Create with file_loader_init() and destroy with 
// file_loader_destroy() when done.
typedef struct {
  // Public, set before calling file_loader_start(); do not modify while running.
  char        path[FILE_HANDLING_PATH_MAX];
  hw_t       *hw;
  double      trigger_lockout_ms;
  bool        verbose;

  // Private -- access only through the file_loader_* functions below.
  pthread_t            thread;
  pthread_mutex_t      mutex;
  bool                 stop_requested; // main thread writes; loader thread reads
  file_loader_status_t status;         // loader thread writes; main thread reads
  bool                 started;        // true once 'thread' holds a real, joinable handle
} file_loader_t;

// Initialize the loader struct and its mutex.
file_loader_t file_loader_init(hw_t *hw, double trigger_lockout_ms, bool verbose);

// Set the loader's trigger_lockout_ms value. Must be called before file_loader_start() and not while the loader is running.
int file_loader_set_trigger_lockout(file_loader_t *loader, double trigger_lockout_ms);

// Destroy the loader's mutex. Call after the thread has been joined and the
// loader is no longer needed.
void file_loader_destroy(file_loader_t *loader);

// Spawn the loader thread for the file at loader->path. The caller must have
// already set loader->path, loader->hw, and loader->verbose.
// Returns 0 on success, non-zero if the thread could not be created.
int file_loader_start(file_loader_t *loader);

// Signal the loader thread to stop at the next safe checkpoint. Returns
// immediately; the thread may still be running when this returns. Call
// file_loader_join() afterward to wait for it to finish.
// Thread-safe: may be called from any thread.
void file_loader_request_stop(file_loader_t *loader);

// Block until the loader thread has exited. Safe to call even if the thread
// finished on its own. Returns the thread's exit status via *status_out
// (pass NULL to ignore).
void file_loader_join(file_loader_t *loader, file_loader_status_t *status_out);

// Read the current status without blocking. Thread-safe.
file_loader_status_t file_loader_get_status(file_loader_t *loader);

// Append CSV-formatted ADC amp readings to the file at path. Returns 0 on success, non-zero on failure.
int file_append_adc_dump(const char *path, const double *adc_values, uint32_t channel_count);

#endif // FILE_HANDLING_H
