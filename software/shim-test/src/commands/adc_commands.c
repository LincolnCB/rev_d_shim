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
#include "adc_commands.h"
#include "command_helper.h"
#include "sys_sts.h"
#include "adc_ctrl.h"

// Forward declarations for helper functions
static void* adc_data_stream_thread(void* arg);
static void* adc_stream_thread(void* arg);
static int parse_adc_command_file(const char* file_path, adc_command_t** commands, int* command_count);

// Local helper function to check if system is running
static int validate_system_running(command_context_t* ctx) {
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state != S_RUNNING) {
    printf("Error: Hardware manager is not running (state: %u). Use 'on' command first.\n", state);
    return -1;
  }
  return 0;
}

// ADC FIFO status commands
int cmd_adc_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_cmd_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "ADC Command");
  return 0;
}

int cmd_adc_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_data_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "ADC Data");
  return 0;
}

// ADC data reading commands
int cmd_read_adc_pair(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_data: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from ADC FIFO for board %d...\n", board);
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      if (*(ctx->verbose)) printf("Sample %d - ADC data from board %d: 0x%" PRIx32 "\n", ++count, board, data);
      adc_print_pair(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    if (*(ctx->verbose)) printf("Read ADC data from board %d: 0x%" PRIx32 "\n", board, data);
    adc_print_pair(data);
  }
  return 0;
}

int cmd_read_adc_single(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_single: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all available ADC data for board %d...\n", board);
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      if (*(ctx->verbose)) printf("Sample %d - ADC data from board %d: 0x%" PRIx32 "\n", ++count, board, data);
      adc_print_single(data);
      printf("\n");
    }
    printf("Read %d samples total for board %d.\n", count, board);
  } else {
    if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
      return -1;
    }
    
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    if (*(ctx->verbose)) printf("Read ADC data from board %d: 0x%" PRIx32 "\n", board, data);
    adc_print_single(data);
  }
  return 0;
}

int cmd_read_adc_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_dbg: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all debug information from ADC FIFO for board %d...\n", board);
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      adc_print_debug(data);
    }
  } else {
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    printf("Reading one debug sample from ADC FIFO for board %d...\n", board);
    adc_print_debug(data);
  }
  return 0;
}

// ADC command operations
int cmd_adc_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_noop: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  bool is_trigger;
  uint32_t value;
  if (parse_trigger_mode(args[1], args[2], &is_trigger, &value) < 0) {
    return -1;
  }
  
  if (value > 0x0FFFFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - %u)\n", value, 0x0FFFFFFF);
    return -1;
  }
  
  bool cont = has_flag(flags, flag_count, FLAG_CONTINUE);
  
  adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, is_trigger, cont, value, *(ctx->verbose));
  printf("ADC no-op command sent to board %d with %s mode, value %u%s.\n", 
         board, is_trigger ? "trigger" : "delay", value, cont ? ", continuous" : "");
  return 0;
}

int cmd_adc_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_cancel: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
  printf("ADC cancel command sent to board %d.\n", board);
  return 0;
}

int cmd_adc_set_ord(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_set_ord: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse 8 channel order values (must be 0-7)
  uint8_t channel_order[8];
  for (int i = 0; i < 8; i++) {
    char* endptr;
    long val = strtol(args[i + 1], &endptr, 0);
    if (*endptr != '\0') {
      fprintf(stderr, "Invalid channel order value for adc_set_ord at position %d: '%s'. Must be a number.\n", i, args[i + 1]);
      return -1;
    }
    if (val < 0 || val > 7) {
      fprintf(stderr, "Invalid channel order value for adc_set_ord at position %d: %ld. Must be 0-7.\n", i, val);
      return -1;
    }
    channel_order[i] = (uint8_t)val;
  }
  
  adc_cmd_set_ord(ctx->adc_ctrl, (uint8_t)board, channel_order, *(ctx->verbose));
  printf("ADC channel order set for board %d: [%d, %d, %d, %d, %d, %d, %d, %d]\n", 
         board, channel_order[0], channel_order[1], channel_order[2], channel_order[3],
         channel_order[4], channel_order[5], channel_order[6], channel_order[7]);
  return 0;
}

// ADC reading operations
int cmd_do_adc_simple_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_simple_read: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  char* endptr;
  long loop_count = strtol(args[1], &endptr, 0);
  if (*endptr != '\0' || loop_count < 1) {
    fprintf(stderr, "Invalid loop count for adc_simple_read: '%s'. Must be at least 1.\n", args[1]);
    return -1;
  }
  
  long delay_cycles = strtol(args[2], &endptr, 0);
  if (*endptr != '\0' || delay_cycles < 0 || delay_cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Invalid delay cycles for adc_simple_read: '%s'. Must be 0 to 536870911.\n", args[2]);
    return -1;
  }
  
  printf("Performing %ld simple ADC reads on board %d (delay mode, value %ld)...\n", loop_count, board, delay_cycles);
  
  for (long i = 0; i < loop_count; i++) {
    adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, (uint32_t)delay_cycles, *(ctx->verbose));
    if (*(ctx->verbose)) {
      printf("ADC read command %ld sent to board %d\n", i + 1, board);
    }
  }
  
  printf("Completed %ld ADC read commands on board %d.\n", loop_count, board);
  return 0;
}

int cmd_do_adc_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_read: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  char* endptr;
  long loop_count = strtol(args[1], &endptr, 0);
  if (*endptr != '\0' || loop_count < 1 || loop_count > 0x1FFFFFF) {
    fprintf(stderr, "Invalid loop count for adc_read: '%s'. Must be 1 to 33554431.\n", args[1]);
    return -1;
  }
  
  long delay_cycles = strtol(args[2], &endptr, 0);
  if (*endptr != '\0' || delay_cycles < 0 || delay_cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Invalid delay cycles for adc_read: '%s'. Must be 0 to 536870911.\n", args[2]);
    return -1;
  }
  
  printf("Performing ADC read on board %d using loop command (loop count: %ld, delay mode, value %ld)...\n", board, loop_count, delay_cycles);
  
  adc_cmd_loop_next(ctx->adc_ctrl, (uint8_t)board, (uint32_t)loop_count, *(ctx->verbose));
  adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, (uint32_t)delay_cycles, *(ctx->verbose));
  
  printf("ADC read commands sent to board %d: loop_next(%ld) + adc_rd(delay, %ld).\n", board, loop_count, delay_cycles);
  return 0;
}

int cmd_do_adc_rd_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    return -1;
  }
  
  if (validate_system_running(ctx) < 0) {
    return -1;
  }
  
  printf("Reading ADC channel %d (board %d, channel %d)...\n", atoi(args[0]), board, channel);
  
  // Read the specific ADC channel
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  
  printf("ADC read channel command sent for channel %d (board %d, channel %d).\n", atoi(args[0]), board, channel);
  return 0;
}

// Thread function for ADC data streaming
static void* adc_data_stream_thread(void* arg) {
  adc_data_stream_params_t* stream_data = (adc_data_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  const char* file_path = stream_data->file_path;
  uint64_t word_count = stream_data->word_count;
  volatile bool* should_stop = stream_data->should_stop;
  bool verbose = *(ctx->verbose);
  
  printf("ADC Stream Thread[%d]: Starting to write %llu words to file '%s'\n", board, word_count, file_path);
  
  // Open file for writing
  FILE* file = fopen(file_path, "wb");
  if (file == NULL) {
    fprintf(stderr, "ADC Stream Thread[%d]: Failed to open file '%s' for writing: %s\n", 
           board, file_path, strerror(errno));
    goto cleanup;
  }
  
  uint64_t words_written = 0;
  uint32_t write_buffer[256]; // Buffer for writing data
  
  while (words_written < word_count && !(*should_stop)) {
    // Check data FIFO status
    uint32_t data_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, false);
    
    if (FIFO_PRESENT(data_status) == 0) {
      fprintf(stderr, "ADC Stream Thread[%d]: Data FIFO not present, stopping stream\n", board);
      break;
    }
    
    uint32_t words_available = FIFO_STS_WORD_COUNT(data_status);
    
    if (words_available > 0) {
      // Determine how many words to read (up to buffer size and remaining count)
      uint32_t words_to_read = words_available;
      if (words_to_read > 256) {
        words_to_read = 256;
      }
      if (words_written + words_to_read > word_count) {
        words_to_read = (uint32_t)(word_count - words_written);
      }
      
      // Read data from FIFO
      for (uint32_t i = 0; i < words_to_read; i++) {
        write_buffer[i] = adc_read(ctx->adc_ctrl, board);
      }
      
      // Write to file
      size_t written = fwrite(write_buffer, sizeof(uint32_t), words_to_read, file);
      if (written != words_to_read) {
        fprintf(stderr, "ADC Stream Thread[%d]: Failed to write to file: %s\n", 
               board, strerror(errno));
        break;
      }
      
      words_written += written;
      
      if (verbose && words_written % 10000 == 0) {
        printf("ADC Stream Thread[%d]: Written %llu/%llu words (%.1f%%)\n",
               board, words_written, word_count,
               (double)words_written / word_count * 100.0);
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
    printf("ADC Stream Thread[%d]: Stream stopped by user after writing %llu words\n",
           board, words_written);
  } else {
    printf("ADC Stream Thread[%d]: Stream completed, wrote %llu words to file '%s'\n",
           board, words_written, file_path);
  }
  
cleanup:
  ctx->adc_stream_running[board] = false;
  free(stream_data);
  return NULL;
}

int cmd_read_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_to_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse word count
  char* endptr;
  uint64_t word_count = parse_value(args[1], &endptr);
  if (*endptr != '\0' || word_count == 0) {
    fprintf(stderr, "Invalid word count for read_adc_to_file: '%s'. Must be a positive integer.\n", args[1]);
    return -1;
  }
  
  // Check if stream is already running
  if (ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check data FIFO presence
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot start streaming.\n", board);
    return -1;
  }
  
  // Resolve glob pattern if present
  char resolved_path[1024];
  if (resolve_file_pattern(args[2], resolved_path, sizeof(resolved_path)) != 0) {
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(resolved_path, full_path, sizeof(full_path));
  
  // Allocate thread data structure
  adc_data_stream_params_t* stream_data = malloc(sizeof(adc_data_stream_params_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for stream data\n");
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->word_count = word_count;
  stream_data->should_stop = &(ctx->adc_stream_stop[board]);
  
  // Set file permissions for group access
  set_file_permissions(full_path, *(ctx->verbose));
  
  // Initialize stop flag and mark stream as running
  ctx->adc_stream_stop[board] = false;
  ctx->adc_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->adc_stream_threads[board]), NULL, adc_data_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(stream_data);
    return -1;
  }
  
  printf("Started ADC streaming for board %d to file '%s' (%llu words)\n", board, full_path, word_count);
  return 0;
}

int cmd_stream_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // This is an alias for read_adc_to_file
  return cmd_read_adc_to_file(args, arg_count, flags, flag_count, ctx);
}

int cmd_stop_adc_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stop_adc_stream: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if stream is running
  if (!ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is not running.\n", board);
    return -1;
  }
  
  printf("Stopping ADC streaming for board %d...\n", board);
  
  // Signal the thread to stop
  ctx->adc_stream_stop[board] = true;
  
  // Wait for the thread to finish
  if (pthread_join(ctx->adc_stream_threads[board], NULL) != 0) {
    fprintf(stderr, "Failed to join ADC streaming thread for board %d: %s\n", board, strerror(errno));
    return -1;
  }
  
  printf("ADC streaming for board %d has been stopped.\n", board);
  return 0;
}

// Function to validate and parse an ADC command file
static int parse_adc_command_file(const char* file_path, adc_command_t** commands, int* command_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open ADC command file '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  // First pass: count lines and validate format
  char line[512];
  int line_num = 0;
  int valid_lines = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Check if line starts with L, T, D, or O
    if (*trimmed != 'L' && *trimmed != 'T' && *trimmed != 'D' && *trimmed != 'O') {
      fprintf(stderr, "Invalid line %d: must start with 'L', 'T', 'D', or 'O'\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Parse the line to validate format
    char mode;
    uint32_t value;
    uint8_t s1, s2, s3, s4, s5, s6, s7, s8;
    
    if (*trimmed == 'O') {
      // Order command: O s1 s2 s3 s4 s5 s6 s7 s8
      int parsed = sscanf(trimmed, "%c %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu", 
                         &mode, &s1, &s2, &s3, &s4, &s5, &s6, &s7, &s8);
      if (parsed != 9) {
        fprintf(stderr, "Invalid line %d: 'O' command must have 8 order values\n", line_num);
        fclose(file);
        return -1;
      }
      // Validate order values (0-7)
      if (s1 > 7 || s2 > 7 || s3 > 7 || s4 > 7 || s5 > 7 || s6 > 7 || s7 > 7 || s8 > 7) {
        fprintf(stderr, "Invalid line %d: order values must be 0-7\n", line_num);
        fclose(file);
        return -1;
      }
    } else {
      // L, T, D commands: <cmd> <value>
      int parsed = sscanf(trimmed, "%c %u", &mode, &value);
      if (parsed != 2) {
        fprintf(stderr, "Invalid line %d: must have command and value\n", line_num);
        fclose(file);
        return -1;
      }
      
      // Validate value range
      if (value > 0x1FFFFFF) {
        fprintf(stderr, "Invalid line %d: value %u out of range (max 0x1FFFFFF or 33554431)\n", line_num, value);
        fclose(file);
        return -1;
      }
    }
    
    valid_lines++;
  }
  
  if (valid_lines == 0) {
    fprintf(stderr, "No valid commands found in ADC command file\n");
    fclose(file);
    return -1;
  }
  
  // Allocate memory for commands
  *commands = malloc(valid_lines * sizeof(adc_command_t));
  if (*commands == NULL) {
    fprintf(stderr, "Failed to allocate memory for ADC commands\n");
    fclose(file);
    return -1;
  }
  
  // Second pass: actually parse the commands
  rewind(file);
  line_num = 0;
  *command_count = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    adc_command_t* cmd = &(*commands)[*command_count];
    cmd->type = *trimmed;
    
    if (*trimmed == 'O') {
      uint8_t s1, s2, s3, s4, s5, s6, s7, s8;
      sscanf(trimmed, "%c %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu", 
             &cmd->type, &s1, &s2, &s3, &s4, &s5, &s6, &s7, &s8);
      cmd->order[0] = s1; cmd->order[1] = s2; cmd->order[2] = s3; cmd->order[3] = s4;
      cmd->order[4] = s5; cmd->order[5] = s6; cmd->order[6] = s7; cmd->order[7] = s8;
      cmd->value = 0; // Not used for order commands
    } else {
      sscanf(trimmed, "%c %u", &cmd->type, &cmd->value);
      // Initialize order array (not used for non-O commands)
      for (int i = 0; i < 8; i++) {
        cmd->order[i] = 0;
      }
    }
    
    (*command_count)++;
  }
  
  fclose(file);
  return 0;
}

// ADC streaming thread function (for streaming commands from file)
static void* adc_stream_thread(void* arg) {
  adc_command_stream_params_t* stream_data = (adc_command_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  bool verbose = *(ctx->verbose);
  
  printf("Starting ADC streaming thread for board %d\n", board);
  
  // Stream commands for each loop iteration
  for (int loop = 0; loop < stream_data->loop_count && !*(stream_data->should_stop); loop++) {
    if (verbose) {
      printf("ADC stream loop %d/%d for board %d\n", loop + 1, stream_data->loop_count, board);
    }
    
    // Stream each command
    for (int i = 0; i < stream_data->command_count && !*(stream_data->should_stop); i++) {
      adc_command_t* cmd = &stream_data->commands[i];
      
      // Check if command FIFO has space
      uint32_t cmd_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false);
      if (!FIFO_PRESENT(cmd_status)) {
        fprintf(stderr, "ADC command FIFO for board %d is not present\n", board);
        break;
      }
      
      // Wait if FIFO is full
      while (FIFO_STS_FULL(cmd_status) && !*(stream_data->should_stop)) {
        usleep(1000); // Wait 1ms
        cmd_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false);
      }
      
      if (*(stream_data->should_stop)) break;
      
      // Send command based on type
      switch (cmd->type) {
        case 'L':
          if (stream_data->simple_mode) {
            // Simple mode: unroll the loop by sending the next command multiple times
            if (i + 1 < stream_data->command_count) {
              adc_command_t* next_cmd = &stream_data->commands[i + 1];
              for (uint32_t j = 0; j < cmd->value && !*(stream_data->should_stop); j++) {
                // Wait for FIFO space
                while (FIFO_STS_FULL(sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false)) && !*(stream_data->should_stop)) {
                  usleep(1000);
                }
                if (*(stream_data->should_stop)) break;
                
                // Send the next command
                switch (next_cmd->type) {
                  case 'T':
                    adc_cmd_adc_rd(ctx->adc_ctrl, board, true, false, next_cmd->value, verbose);
                    break;
                  case 'D':
                    adc_cmd_adc_rd(ctx->adc_ctrl, board, false, false, next_cmd->value, verbose);
                    break;
                  case 'O':
                    adc_cmd_set_ord(ctx->adc_ctrl, board, next_cmd->order, verbose);
                    break;
                }
              }
              i++; // Skip the next command since we already executed it
            }
          } else {
            // Use hardware loop command
            adc_cmd_loop_next(ctx->adc_ctrl, board, cmd->value, verbose);
          }
          break;
        
        case 'T':
          adc_cmd_adc_rd(ctx->adc_ctrl, board, true, false, cmd->value, verbose);
          break;
        
        case 'D':
          adc_cmd_adc_rd(ctx->adc_ctrl, board, false, false, cmd->value, verbose);
          break;
        
        case 'O':
          adc_cmd_set_ord(ctx->adc_ctrl, board, cmd->order, verbose);
          break;
        default:
          fprintf(stderr, "Invalid ADC command type: %c\n", cmd->type);
          break;
      }
      
      if (verbose) {
        printf("ADC stream sent command %c for board %d\n", cmd->type, board);
      }
    }
  }
  
  printf("ADC streaming thread for board %d completed\n", board);
  
  // Mark stream as not running and clean up
  ctx->adc_stream_running[board] = false;
  free(stream_data->commands);
  free(stream_data);
  return NULL;
}

int cmd_stream_adc_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stream_adc_from_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse optional loop count (default is 1 - play once)
  int loop_count = 1;
  if (arg_count >= 3) {
    char* endptr;
    loop_count = (int)parse_value(args[2], &endptr);
    if (*endptr != '\0' || loop_count < 1) {
      fprintf(stderr, "Invalid loop count for stream_adc_from_file: '%s'. Must be a positive integer.\n", args[2]);
      return -1;
    }
  }
  
  // Check for simple flag
  bool simple_mode = has_flag(flags, flag_count, FLAG_SIMPLE);
  
  // Check if stream is already running
  if (ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check ADC command FIFO presence
  if (FIFO_PRESENT(sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC command FIFO for board %d is not present. Cannot start streaming.\n", board);
    return -1;
  }
  
  // Resolve glob pattern if present
  char resolved_path[1024];
  if (resolve_file_pattern(args[1], resolved_path, sizeof(resolved_path)) != 0) {
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(resolved_path, full_path, sizeof(full_path));
  
  // Parse and validate the ADC command file
  adc_command_t* commands = NULL;
  int command_count = 0;
  
  if (parse_adc_command_file(full_path, &commands, &command_count) != 0) {
    return -1; // Error already printed by parse_adc_command_file
  }
  
  printf("Parsed %d commands from ADC command file '%s'\n", command_count, full_path);
  if (simple_mode) {
    printf("Using simple mode (unrolling loops)\n");
  }
  
  // Allocate thread data structure
  adc_command_stream_params_t* stream_data = malloc(sizeof(adc_command_stream_params_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for ADC stream data\n");
    free(commands);
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->should_stop = &(ctx->adc_stream_stop[board]);
  stream_data->commands = commands;
  stream_data->command_count = command_count;
  stream_data->loop_count = loop_count;
  stream_data->simple_mode = simple_mode;
  
  // Initialize stop flag and mark stream as running
  ctx->adc_stream_stop[board] = false;
  ctx->adc_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->adc_stream_threads[board]), NULL, adc_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(commands);
    free(stream_data);
    return -1;
  }
  
  printf("Started ADC streaming for board %d from file '%s' (looping %d time%s)%s\n", 
         board, full_path, loop_count, loop_count == 1 ? "" : "s",
         simple_mode ? " in simple mode" : "");
  return 0;
}
