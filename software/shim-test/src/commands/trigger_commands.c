#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
#include <glob.h>
#include "trigger_commands.h"
#include "command_helper.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

// Forward declaration for helper function
static void* trigger_data_stream_thread(void* arg);

// Trigger FIFO status commands
int cmd_trig_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t fifo_status = sys_sts_get_trig_cmd_fifo_status(ctx->sys_sts, *(ctx->verbose));
  print_fifo_status(fifo_status, "Trigger Command");
  return 0;
}

int cmd_trig_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t fifo_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose));
  print_fifo_status(fifo_status, "Trigger Data");
  return 0;
}

// Trigger data reading commands
int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (FIFO_PRESENT(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose))) == 0) {
    printf("Trigger data FIFO is not present. Cannot read data.\n");
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose)))) {
    printf("Trigger data FIFO is empty. Cannot read data.\n");
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from trigger FIFO...\n");
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose)))) {
      uint64_t data = trigger_read(ctx->trigger_ctrl);
      printf("Sample %d - Trigger data: 0x%" PRIx64 "\n", ++count, data);
      print_trigger_data(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint64_t data = trigger_read(ctx->trigger_ctrl);
    printf("Read trigger data: 0x%" PRIx64 "\n", data);
    print_trigger_data(data);
  }
  return 0;
}

// Basic trigger command operations
int cmd_trig_sync_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_sync_ch(ctx->trigger_ctrl);
  printf("Trigger synchronize channels command sent.\n");
  return 0;
}

int cmd_trig_force_trig(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_force_trig(ctx->trigger_ctrl);
  printf("Trigger force trigger command sent.\n");
  return 0;
}

int cmd_trig_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_cancel(ctx->trigger_ctrl);
  printf("Trigger cancel command sent.\n");
  return 0;
}

// Trigger command operations with value parameters
int cmd_trig_set_lockout(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid lockout cycles for trig_set_lockout: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (cycles == 0 || cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Lockout cycles out of range: %u (valid range: 1 - 536870911)\n", cycles);
    return -1;
  }
  
  trigger_cmd_set_lockout(ctx->trigger_ctrl, cycles);
  printf("Trigger set lockout command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_delay(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid delay cycles for trig_delay: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Delay cycles out of range: %u (valid range: 0 - 536870911)\n", cycles);
    return -1;
  }
  
  trigger_cmd_delay(ctx->trigger_ctrl, cycles);
  printf("Trigger delay command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_expect_ext(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t count = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid count for trig_expect_ext: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (count > 0x1FFFFFFF) {
    fprintf(stderr, "Count out of range: %u (valid range: 0 - 536870911)\n", count);
    return -1;
  }
  
  trigger_cmd_expect_ext(ctx->trigger_ctrl, count);
  printf("Trigger expect external command sent with count %u.\n", count);
  return 0;
}

// Thread function for trigger data streaming
static void* trigger_data_stream_thread(void* arg) {
  trigger_data_stream_params_t* stream_data = (trigger_data_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* file_path = stream_data->file_path;
  uint64_t sample_count = stream_data->sample_count;
  volatile bool* should_stop = stream_data->should_stop;
  bool binary_mode = stream_data->binary_mode;
  bool verbose = *(ctx->verbose);
  
  printf("Trigger Stream Thread: Starting to write %llu samples to file '%s' (%s format)\n", 
         sample_count, file_path, binary_mode ? "binary" : "ASCII");
  
  // Open file for writing (binary or text mode based on format)
  FILE* file = fopen(file_path, binary_mode ? "wb" : "w");
  if (file == NULL) {
    fprintf(stderr, "Trigger Stream Thread: Failed to open file '%s' for writing: %s\n", 
           file_path, strerror(errno));
    goto cleanup;
  }
  
  uint64_t samples_written = 0;
  
  while (samples_written < sample_count && !(*should_stop)) {
    // Check trigger data FIFO status
    uint32_t data_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, false);
    
    if (FIFO_PRESENT(data_status) == 0) {
      fprintf(stderr, "Trigger Stream Thread: Data FIFO not present, stopping stream\n");
      break;
    }
    
    if (!FIFO_STS_EMPTY(data_status)) {
      // Read 64-bit trigger data
      uint64_t trigger_data = trigger_read(ctx->trigger_ctrl);
      
      // Write data based on format mode
      if (binary_mode) {
        // Binary mode: write raw 64-bit value directly
        size_t written = fwrite(&trigger_data, sizeof(uint64_t), 1, file);
        if (written != 1) {
          fprintf(stderr, "Trigger Stream Thread: Failed to write to file: %s\n", strerror(errno));
          break;
        }
      } else {
        // ASCII mode: write one trigger sample per line
        fprintf(file, "0x%016" PRIx64 "\n", trigger_data);
      }
      
      // Flush the file to ensure data is written
      fflush(file);
      
      samples_written++;
      
      if (verbose && samples_written % 1000 == 0) {
        printf("Trigger Stream Thread: Written %llu/%llu samples (%.1f%%)\n",
               samples_written, sample_count,
               (double)samples_written / sample_count * 100.0);
      }
    } else {
      // No data available, sleep briefly
      usleep(100);
    }
  }
  
  if (file) {
    fclose(file);
  }
  
  if (*should_stop) {
    printf("Trigger Stream Thread: Stream stopped by user after writing %llu samples\n",
           samples_written);
  } else {
    printf("Trigger Stream Thread: Stream completed, wrote %llu samples to file '%s'\n",
           samples_written, file_path);
  }
  
cleanup:
  ctx->trig_data_stream_running = false;
  free(stream_data);
  return NULL;
}

int cmd_stream_trig_data_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse sample count
  char* endptr;
  uint64_t sample_count = parse_value(args[0], &endptr);
  if (*endptr != '\0' || sample_count == 0) {
    fprintf(stderr, "Invalid sample count for stream_trig_data_to_file: '%s'. Must be greater than 0.\n", args[0]);
    return -1;
  }
  
  // Check for binary mode flag
  bool binary_mode = has_flag(flags, flag_count, FLAG_BIN);
  
  // Check if stream is already running
  if (ctx->trig_data_stream_running) {
    printf("Trigger data streaming is already running. Stop it first.\n");
    return -1;
  }
  
  if (*(ctx->verbose)) {
    printf("Setting up trigger data streaming: %llu samples, %s mode\n", 
           sample_count, binary_mode ? "binary" : "ASCII");
  }
  
  // Check trigger data FIFO presence
  if (FIFO_PRESENT(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose))) == 0) {
    printf("Trigger data FIFO is not present. Cannot stream data.\n");
    return -1;
  }
  
  if (*(ctx->verbose)) {
    printf("Trigger data FIFO is present and ready for streaming\n");
  }
  
  // For output files, use the path directly (no glob pattern resolution needed)
  // Just clean and expand the path to handle ~ and relative paths
  char full_path[1024];
  clean_and_expand_path(args[1], full_path, sizeof(full_path));
  
  // Modify file extension based on format
  char final_path[1024];
  strcpy(final_path, full_path);
  
  // If no extension is provided, add default extension based on format
  char* dot = strrchr(final_path, '.');
  char* slash = strrchr(final_path, '/');
  
  // Check if there's a dot after the last slash (or no slash at all)
  if (dot == NULL || (slash != NULL && dot < slash)) {
    // No extension found, add default
    if (binary_mode) {
      strcat(final_path, ".dat");
    } else {
      strcat(final_path, ".csv");
    }
  }
  // If extension exists, user specified it explicitly, so keep it
  
  if (*(ctx->verbose)) {
    printf("Final output file path: %s\n", final_path);
  }
  
  // Allocate thread data structure
  trigger_data_stream_params_t* stream_data = malloc(sizeof(trigger_data_stream_params_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for trigger stream data\n");
    return -1;
  }
  
  if (*(ctx->verbose)) {
    printf("Allocated trigger stream data structure\n");
  }
  
  stream_data->ctx = ctx;
  strcpy(stream_data->file_path, final_path);
  stream_data->sample_count = sample_count;
  stream_data->should_stop = &(ctx->trig_data_stream_stop);
  stream_data->binary_mode = binary_mode;
  
  if (*(ctx->verbose)) {
    printf("Initialized trigger stream parameters\n");
  }
  
  // Set file permissions for group access
  set_file_permissions(final_path, *(ctx->verbose));
  
  // Initialize stop flag and mark stream as running
  ctx->trig_data_stream_stop = false;
  ctx->trig_data_stream_running = true;
  
  if (*(ctx->verbose)) {
    printf("Set trigger stream flags, creating thread\n");
  }
  
  // Create the streaming thread
  if (pthread_create(&(ctx->trig_data_stream_thread), NULL, trigger_data_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create trigger data streaming thread\n");
    ctx->trig_data_stream_running = false;
    free(stream_data);
    return -1;
  }
  
  if (*(ctx->verbose)) {
    printf("Created trigger data streaming thread successfully\n");
  }
  
  printf("Started trigger data streaming to file '%s' (%llu samples, %s format)\n", 
         final_path, sample_count, binary_mode ? "binary" : "ASCII");
  return 0;
}

int cmd_stop_trig_data_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Check if stream is running
  if (!ctx->trig_data_stream_running) {
    printf("Trigger data streaming is not currently running.\n");
    return 0;
  }
  
  printf("Stopping trigger data streaming...\n");
  
  // Signal the thread to stop
  ctx->trig_data_stream_stop = true;
  
  // Wait for the thread to finish
  if (pthread_join(ctx->trig_data_stream_thread, NULL) != 0) {
    fprintf(stderr, "Failed to join trigger data streaming thread\n");
    return -1;
  }
  
  printf("Trigger data streaming has been stopped.\n");
  return 0;
}
