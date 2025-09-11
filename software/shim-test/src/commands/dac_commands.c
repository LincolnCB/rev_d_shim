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
#include "dac_commands.h"
#include "command_helper.h"
#include "sys_sts.h"
#include "dac_ctrl.h"

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

// DAC FIFO status commands
int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_cmd_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "DAC Command");
  return 0;
}

int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_data_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "DAC Data");
  return 0;
}

int cmd_read_dac_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_dac_dbg: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC data FIFO for board %d is not present. Cannot read debug data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("DAC data FIFO for board %d is empty. Cannot read debug data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all debug information from DAC FIFO for board %d...\n", board);
    while (!FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
      dac_print_debug(data);
    }
  } else {
    uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
    printf("Reading one debug sample from DAC FIFO for board %d...\n", board);
    dac_print_debug(data);
  }
  return 0;
}

// DAC command operations
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_noop: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC no-op command to board %d: DAC command stream is currently running. Stop the stream first.\n", board);
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
  
  dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, is_trigger, cont, false, value, *(ctx->verbose));
  printf("DAC no-op command sent to board %d with %s mode, value %u%s.\n", 
         board, is_trigger ? "trigger" : "delay", value, cont ? ", continuous" : "");
  return 0;
}

int cmd_dac_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_cancel: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC cancel command to board %d: DAC command stream is currently running. Stop the stream first.\n", board);
    return -1;
  }
  
  dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  printf("DAC cancel command sent to board %d.\n", board);
  return 0;
}

int cmd_write_dac_update(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for write_dac_update: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC write update command to board %d: DAC command stream is currently running. Stop the stream first.\n", board);
    return -1;
  }
  
  // Parse 8 channel values (16-bit signed integers)
  int16_t ch_vals[8];
  for (int i = 0; i < 8; i++) {
    char* endptr;
    long val = strtol(args[i + 1], &endptr, 0);
    if (*endptr != '\0') {
      fprintf(stderr, "Invalid channel %d value for write_dac_update: '%s'\n", i, args[i + 1]);
      return -1;
    }
    if (val < -32767 || val > 32767) {
      fprintf(stderr, "Channel %d value out of range: %ld (valid range: -32767 to 32767)\n", i, val);
      return -1;
    }
    ch_vals[i] = (int16_t)val;
  }
  
  bool is_trigger;
  uint32_t value;
  if (parse_trigger_mode(args[9], args[10], &is_trigger, &value) < 0) {
    return -1;
  }
  
  if (value > 0x0FFFFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - %u)\n", value, 0x0FFFFFFF);
    return -1;
  }
  
  bool cont = has_flag(flags, flag_count, FLAG_CONTINUE);
  
  // Execute DAC write update command with ldac = true
  dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, is_trigger, cont, true, value, *(ctx->verbose));
  printf("DAC write update command sent to board %d with %s mode, value %u%s.\n", 
         board, is_trigger ? "trigger" : "delay", value, cont ? ", continuous" : "");
  printf("Channel values: [%d, %d, %d, %d, %d, %d, %d, %d]\n",
         ch_vals[0], ch_vals[1], ch_vals[2], ch_vals[3], 
         ch_vals[4], ch_vals[5], ch_vals[6], ch_vals[7]);
  return 0;
}

// Single channel DAC operations
int cmd_do_dac_wr_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    return -1;
  }
  
  if (validate_system_running(ctx) < 0) {
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot write to DAC channel %d (board %d): DAC command stream is currently running. Stop the stream first.\n", atoi(args[0]), board);
    return -1;
  }
  
  char* endptr;
  long value = strtol(args[1], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for do_dac_wr_ch: '%s'. Must be a number.\n", args[1]);
    return -1;
  }
  if (value < -32767 || value > 32767) {
    fprintf(stderr, "Value out of range: %ld (valid range: -32767 to 32767)\n", value);
    return -1;
  }
  
  printf("Writing value %ld to DAC channel %d (board %d, channel %d)...\n", value, atoi(args[0]), board, channel);
  
  // Use the dedicated single channel write command
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)value, *(ctx->verbose));
  
  printf("Wrote value %ld to DAC channel %d (board %d, channel %d).\n", value, atoi(args[0]), board, channel);
  return 0;
}

// Function to validate and parse a waveform file
static int parse_waveform_file(const char* file_path, waveform_command_t** commands, int* command_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open waveform file '%s': %s\n", file_path, strerror(errno));
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
    
    // Check if line starts with D or T
    if (*trimmed != 'D' && *trimmed != 'T') {
      fprintf(stderr, "Invalid line %d: must start with 'D' or 'T'\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Parse the line to validate format
    char mode;
    uint32_t value;
    int16_t ch_vals[8];
    int parsed = sscanf(trimmed, "%c %u %hd %hd %hd %hd %hd %hd %hd %hd", 
                       &mode, &value, &ch_vals[0], &ch_vals[1], &ch_vals[2], &ch_vals[3],
                       &ch_vals[4], &ch_vals[5], &ch_vals[6], &ch_vals[7]);
    
    if (parsed < 2) {
      fprintf(stderr, "Invalid line %d: must have at least mode and value\n", line_num);
      fclose(file);
      return -1;
    }
    
    if (parsed != 2 && parsed != 10) {
      fprintf(stderr, "Invalid line %d: must have either 2 fields (mode, value) or 10 fields (mode, value, 8 channels)\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Validate value range
    if (value > 0x1FFFFFF) {
      fprintf(stderr, "Invalid line %d: value %u out of range (max 0x1FFFFFF or 33554431)\n", line_num, value);
      fclose(file);
      return -1;
    }
    
    // Validate channel values if present
    if (parsed == 10) {
      for (int i = 0; i < 8; i++) {
        if (ch_vals[i] < -32767 || ch_vals[i] > 32767) {
          fprintf(stderr, "Invalid line %d: channel %d value %d out of range (-32767 to 32767)\n", 
                 line_num, i, ch_vals[i]);
          fclose(file);
          return -1;
        }
      }
    }
    
    valid_lines++;
  }
  
  if (valid_lines == 0) {
    fprintf(stderr, "No valid commands found in waveform file\n");
    fclose(file);
    return -1;
  }
  
  // Allocate memory for commands
  *commands = malloc(valid_lines * sizeof(waveform_command_t));
  if (*commands == NULL) {
    fprintf(stderr, "Failed to allocate memory for waveform commands\n");
    fclose(file);
    return -1;
  }
  
  // Second pass: parse and store commands
  rewind(file);
  line_num = 0;
  int cmd_index = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    waveform_command_t* cmd = &(*commands)[cmd_index];
    
    char mode;
    uint32_t value;
    int16_t ch_vals[8];
    int parsed = sscanf(trimmed, "%c %u %hd %hd %hd %hd %hd %hd %hd %hd", 
                       &mode, &value, &ch_vals[0], &ch_vals[1], &ch_vals[2], &ch_vals[3],
                       &ch_vals[4], &ch_vals[5], &ch_vals[6], &ch_vals[7]);
    
    cmd->is_trigger = (mode == 'T');
    cmd->value = value;
    cmd->has_ch_vals = (parsed == 10);
    cmd->cont = (cmd_index < valid_lines - 1); // true for all except last command
    
    if (cmd->has_ch_vals) {
      for (int i = 0; i < 8; i++) {
        cmd->ch_vals[i] = ch_vals[i];
      }
    }
    
    cmd_index++;
  }
  
  fclose(file);
  *command_count = valid_lines;
  return 0;
}

// Thread function for DAC streaming
void* dac_cmd_stream_thread(void* arg) {
  dac_command_stream_params_t* stream_data = (dac_command_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  const char* file_path = stream_data->file_path;
  volatile bool* should_stop = stream_data->should_stop;
  waveform_command_t* commands = stream_data->commands;
  int command_count = stream_data->command_count;
  int loop_count = stream_data->loop_count;
  
  printf("DAC Stream Thread[%d]: Started streaming from file '%s' (%d commands, %d loop%s)\n", 
         board, file_path, command_count, loop_count, loop_count == 1 ? "" : "s");
  
  int total_commands_sent = 0;
  int current_loop = 0;
  
  while (!(*should_stop) && current_loop < loop_count) {
    int cmd_index = 0;
    int commands_sent_this_loop = 0;
    
    // Process all commands in the current loop
    while (!(*should_stop) && cmd_index < command_count) {
      // Check DAC command FIFO status
      uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, board, false);
      
      if (FIFO_PRESENT(fifo_status) == 0) {
        fprintf(stderr, "DAC Stream Thread[%d]: FIFO not present, stopping stream\n", board);
        goto cleanup;
      }
      
      uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status) + 1; // +1 for safety margin
      uint32_t words_available = DAC_CMD_FIFO_WORDCOUNT - words_used;
      
      // Check if we have space for the next command
      waveform_command_t* cmd = &commands[cmd_index];
      uint32_t words_needed = cmd->has_ch_vals ? 5 : 1; // dac_wr needs 5 words, noop needs 1
      
      if (words_available >= words_needed) {
        // For looping, we need to adjust the 'cont' flag:
        // - Set cont=true for all commands except the last command of the last loop
        bool is_last_command_of_last_loop = (current_loop == loop_count - 1) && (cmd_index == command_count - 1);
        bool cont_flag = !is_last_command_of_last_loop;
        
        // Send the command
        if (cmd->has_ch_vals) {
          dac_cmd_dac_wr(ctx->dac_ctrl, board, cmd->ch_vals, cmd->is_trigger, cont_flag, true, cmd->value, false);
        } else {
          dac_cmd_noop(ctx->dac_ctrl, board, cmd->is_trigger, cont_flag, false, cmd->value, false);
        }
        
        commands_sent_this_loop++;
        total_commands_sent++;
        cmd_index++;
        
        if (*(ctx->verbose)) {
          printf("DAC Stream Thread[%d]: Loop %d/%d, Sent command %d/%d (%s, value=%u, %s, cont=%s) [FIFO: %u/%u words]\n", 
                 board, current_loop + 1, loop_count, commands_sent_this_loop, command_count, 
                 cmd->is_trigger ? "trigger" : "delay", cmd->value,
                 cmd->has_ch_vals ? "with ch_vals" : "noop",
                 cont_flag ? "true" : "false", words_used + words_needed + 1, DAC_CMD_FIFO_WORDCOUNT);
        }
      } else {
        // Not enough space in FIFO, sleep and try again
        usleep(1000); // 1ms
      }
    }
    
    current_loop++;
    if (current_loop < loop_count && *(ctx->verbose)) {
      printf("DAC Stream Thread[%d]: Completed loop %d/%d, starting next loop\n", 
             board, current_loop, loop_count);
    }
  }

cleanup:
  if (*should_stop) {
    printf("DAC Stream Thread[%d]: Stopping stream (user requested), sent %d total commands (%d complete loops)\n", 
           board, total_commands_sent, current_loop);
  } else {
    printf("DAC Stream Thread[%d]: Stream completed, sent %d total commands from file '%s' (%d loops)\n", 
           board, total_commands_sent, file_path, loop_count);
  }
  
  ctx->dac_cmd_stream_running[board] = false;
  free(stream_data->commands);
  free(stream_data);
  return NULL;
}

int cmd_stream_dac_commands_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stream_dac_from_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse optional loop count (default is 1 - play once)
  int loop_count = 1;
  if (arg_count >= 3) {
    char* endptr;
    loop_count = (int)parse_value(args[2], &endptr);
    if (*endptr != '\0' || loop_count < 1) {
      fprintf(stderr, "Invalid loop count for stream_dac_from_file: '%s'. Must be a positive integer.\n", args[2]);
      return -1;
    }
  }
  
  // Check if stream is already running
  if (ctx->dac_cmd_stream_running[board]) {
    printf("DAC command stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check DAC command FIFO presence
  if (FIFO_PRESENT(sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC command FIFO for board %d is not present. Cannot start streaming.\n", board);
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
  
  // Parse and validate the waveform file
  waveform_command_t* commands = NULL;
  int command_count = 0;
  
  if (parse_waveform_file(full_path, &commands, &command_count) != 0) {
    return -1; // Error already printed by parse_waveform_file
  }
  
  printf("Parsed %d commands from waveform file '%s'\n", command_count, full_path);
  
  // Allocate thread data structure
  dac_command_stream_params_t* stream_data = malloc(sizeof(dac_command_stream_params_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for stream data\n");
    free(commands);
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->should_stop = &(ctx->dac_cmd_stream_stop[board]);
  stream_data->commands = commands;
  stream_data->command_count = command_count;
  stream_data->loop_count = loop_count;
  
  // Initialize stop flag and mark stream as running
  ctx->dac_cmd_stream_stop[board] = false;
  ctx->dac_cmd_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->dac_cmd_stream_threads[board]), NULL, dac_cmd_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create DAC command streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->dac_cmd_stream_running[board] = false;
    free(commands);
    free(stream_data);
    return -1;
  }
  
  printf("Started DAC command streaming for board %d from file '%s' (looping %d time%s)\n", 
         board, full_path, loop_count, loop_count == 1 ? "" : "s");
  return 0;
}

int cmd_stop_dac_cmd_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stop_dac_cmd_stream: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if stream is running
  if (!ctx->dac_cmd_stream_running[board]) {
    printf("DAC command stream for board %d is not running.\n", board);
    return -1;
  }
  
  printf("Stopping DAC command streaming for board %d...\n", board);
  
  // Signal the thread to stop
  ctx->dac_cmd_stream_stop[board] = true;
  
  // Wait for the thread to finish
  if (pthread_join(ctx->dac_cmd_stream_threads[board], NULL) != 0) {
    fprintf(stderr, "Failed to join DAC command streaming thread for board %d: %s\n", board, strerror(errno));
    return -1;
  }
  
  printf("DAC command streaming for board %d has been stopped.\n", board);
  return 0;
}


