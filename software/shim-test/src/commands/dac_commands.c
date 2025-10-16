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
#include "system_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
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

int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_dac_data: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("DAC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from DAC FIFO for board %d...\n", board);
    while (!FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = dac_read_data(ctx->dac_ctrl, (uint8_t)board);
      if (*(ctx->verbose)) {
        printf("  Raw DAC Data word: 0x%08X\n", data);
      }
      dac_print_data(data, *(ctx->verbose));
    }
  } else {
    uint32_t data = dac_read_data(ctx->dac_ctrl, (uint8_t)board);
    printf("Reading one data sample from DAC FIFO for board %d...\n", board);
    dac_print_data(data, *(ctx->verbose));
  }
  return 0;
}

// DAC command operations
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  bool use_all = (strcmp(args[0], "all") == 0);
  
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
  
  if (use_all) {
    // Check which boards are connected by checking FIFOs
    bool connected_boards[8] = {false};
    int connected_count = 0;
    
    for (int board = 0; board < 8; board++) {
      uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(dac_cmd_fifo_status) && FIFO_PRESENT(dac_data_fifo_status)) {
        connected_boards[board] = true;
        connected_count++;
      }
    }
    
    if (connected_count == 0) {
      printf("No boards are connected.\n");
      return 0;
    }
    
    printf("Sending DAC no-op command to %d connected board(s) with %s mode, value %u%s:\n", 
           connected_count, is_trigger ? "trigger" : "delay", value, cont ? ", continuous" : "");
    
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      // Check if DAC command stream is running for this board
      if (ctx->dac_cmd_stream_running[board]) {
        printf("  Board %d: Skipped (DAC command stream is running)\n", board);
        continue;
      }
      
      dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, is_trigger, cont, false, value, *(ctx->verbose));
      printf("  Board %d: DAC no-op command sent\n", board);
    }
  } else {
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
    
    dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, is_trigger, cont, false, value, *(ctx->verbose));
    printf("DAC no-op command sent to board %d with %s mode, value %u%s.\n", 
           board, is_trigger ? "trigger" : "delay", value, cont ? ", continuous" : "");
  }
  
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

int cmd_do_dac_wr(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for do_dac_wr: '%s'. Must be 0-7.\n", args[0]);
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
      fprintf(stderr, "Invalid channel %d value for do_dac_wr: '%s'\n", i, args[i + 1]);
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

// Get DAC calibration value for a single channel
int cmd_get_dac_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse arguments - either a channel number (0-63) or --all flag
  bool get_all = has_flag(flags, flag_count, FLAG_ALL);
  int start_ch = 0, end_ch = 0;
  bool connected_boards[8] = {false}; // Track which boards are connected
  
  if (get_all && arg_count > 0) {
    fprintf(stderr, "Error: Cannot specify both channel number and --all flag\n");
    return -1;
  }
  
  if (!get_all && arg_count != 1) {
    fprintf(stderr, "Usage: get_dac_cal <channel> [--no_reset] OR get_dac_cal --all [--no_reset]\n");
    return -1;
  }
  
  if (get_all) {
    start_ch = 0;
    end_ch = 63;
    
    // Check which boards are connected by checking FIFOs
    int connected_count = 0;
    printf("Checking connected boards...\n");
    
    for (int board = 0; board < 8; board++) {
      uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(dac_cmd_fifo_status) && FIFO_PRESENT(dac_data_fifo_status)) {
        connected_boards[board] = true;
        connected_count++;
        printf("  Board %d: Connected\n", board);
      } else {
        printf("  Board %d: Not connected\n", board);
      }
    }
    
    if (connected_count == 0) {
      printf("No boards are connected. Aborting.\n");
      return -1;
    }
    
    printf("Getting calibration values for all channels on %d connected board(s)\n", connected_count);
  } else {
    // Parse single channel number
    int board, channel;
    if (validate_channel_number(args[0], &board, &channel) < 0) {
      return -1;
    }
    start_ch = end_ch = atoi(args[0]);
    
    // Check if the board for this channel is connected
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    
    if (FIFO_PRESENT(dac_cmd_fifo_status) && FIFO_PRESENT(dac_data_fifo_status)) {
      connected_boards[board] = true;
      printf("Getting calibration value for channel %d (board %d connected)\n", start_ch, board);
    } else {
      printf("Error: Board %d for channel %d is not connected\n", board, start_ch);
      return -1;
    }
  }
  
  // Validate system is running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state != S_RUNNING) {
    fprintf(stderr, "System is not running. Current state: %d\n", state);
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Reset buffers once at the start (unless --no_reset flag is used)
  if (!skip_reset) {
    printf("Resetting all buffers...\n");
    safe_buffer_reset(ctx, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
  // Send cancel commands once per connected board
  if (get_all) {
    printf("Sending cancel commands to connected boards...\n");
    for (int board = 0; board < 8; board++) {
      if (connected_boards[board]) {
        dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
      }
    }
  } else {
    // For single channel, send cancel to its board
    int board = start_ch / 8;
    printf("Sending cancel commands to board %d...\n", board);
    dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  }
  usleep(10000); // 10ms to let cancel commands complete
  
  // Iterate through all channels to get calibration values
  for (int ch = start_ch; ch <= end_ch; ch++) {
    int board, channel;
    board = ch / 8;
    channel = ch % 8;
    
    // If getting all channels, skip boards that are not connected
    if (get_all && !connected_boards[board]) {
      continue;
    }
    
    if (get_all) {
      printf("Ch %02d : ", ch);
      fflush(stdout);
    }
    
    // Send get_cal command
    dac_cmd_get_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
    
    // Read calibration data once available
    int tries = 0;
    uint32_t dac_data_fifo_status;
    while (tries < 100) {
      dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
      if (FIFO_STS_WORD_COUNT(dac_data_fifo_status) > 0) break;
      usleep(100); // 0.1ms
      tries++;
    }
    
    if (FIFO_STS_WORD_COUNT(dac_data_fifo_status) == 0) {
      if (get_all) {
        printf("No data available\n");
      } else {
        fprintf(stderr, "No DAC calibration data available after timeout\n");
        return -1;
      }
      continue;
    }
    
    // Read and print calibration data
    uint32_t cal_data_word = dac_read_data(ctx->dac_ctrl, (uint8_t)board);
    if (!get_all && *(ctx->verbose)) {
      printf("  ");
    }
    dac_print_data(cal_data_word, *(ctx->verbose));
  }
  
  if (*(ctx->verbose)) {
    printf("Get calibration completed.\n");
  }
  return 0;
}

// Simple DAC get calibration command - just sends the command without resets or reads
int cmd_do_dac_get_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    return -1;
  }
  
  if (validate_system_running(ctx) < 0) {
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot get DAC calibration for channel %d (board %d): DAC command stream is currently running. Stop the stream first.\n", atoi(args[0]), board);
    return -1;
  }
  
  printf("Sending GET_CAL command for channel %d (board %d, channel %d)...\n", atoi(args[0]), board, channel);
  
  // Just send the get_cal command without any resets or reads
  dac_cmd_get_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  
  printf("GET_CAL command sent for channel %d (board %d, channel %d).\n", atoi(args[0]), board, channel);
  return 0;
}

// Set DAC calibration command - sets the calibration value for a channel
int cmd_set_dac_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    return -1;
  }
  
  if (validate_system_running(ctx) < 0) {
    return -1;
  }
  
  // Check if DAC command stream is running for this board
  if (ctx->dac_cmd_stream_running[board]) {
    fprintf(stderr, "Cannot set DAC calibration for channel %d (board %d): DAC command stream is currently running. Stop the stream first.\n", atoi(args[0]), board);
    return -1;
  }
  
  // Parse calibration value
  char* endptr;
  long cal_value = strtol(args[1], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid calibration value for set_dac_cal: '%s'. Must be a number.\n", args[1]);
    return -1;
  }
  if (cal_value < -32768 || cal_value > 32767) {
    fprintf(stderr, "Calibration value out of range: %ld (valid range: -32768 to 32767)\n", cal_value);
    return -1;
  }
  
  printf("Setting DAC calibration for channel %d (board %d, channel %d) to %ld...\n", 
         atoi(args[0]), board, channel, cal_value);
  
  // Send the set_cal command
  dac_cmd_set_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)cal_value, *(ctx->verbose));
  
  printf("DAC calibration set for channel %d (board %d, channel %d) to %ld.\n", 
         atoi(args[0]), board, channel, cal_value);
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
  
  if (*(ctx->verbose)) {
    printf("DAC Command Stream Thread[%d]: Started streaming from file '%s' (%d commands, %d loop%s)\n", 
           board, file_path, command_count, loop_count, loop_count == 1 ? "" : "s");
  }
  
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
        fprintf(stderr, "DAC Command Stream Thread[%d]: FIFO not present, stopping stream\n", board);
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
          printf("DAC Command Stream Thread[%d]: Loop %d/%d, Sent command %d/%d (%s, value=%u, %s, cont=%s) [FIFO: %u/%u words]\n", 
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
      printf("DAC Command Stream Thread[%d]: Completed loop %d/%d, starting next loop\n", 
             board, current_loop, loop_count);
    }
  }

cleanup:
  if (*should_stop) {
    printf("DAC Command Stream Thread[%d]: Stopping stream (user requested), sent %d total commands (%d complete loops)\n", 
           board, total_commands_sent, current_loop);
  } else {
    printf("DAC Command Stream Thread[%d]: Stream completed, sent %d total commands from file '%s' (%d loops)\n", 
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
  
  if (*(ctx->verbose)) {
    printf("Parsed %d commands from waveform file '%s'\n", command_count, full_path);
  }
  
  // Validate trigger gaps to prevent FIFO underflow
  int words_since_last_trigger = 0;
  int max_gap = 0;
  bool found_trigger = false;
  
  for (int i = 0; i < command_count; i++) {
    waveform_command_t* cmd = &commands[i];
    uint32_t words_needed = cmd->has_ch_vals ? 5 : 1; // dac_wr needs 5 words, noop needs 1
    
    if (cmd->is_trigger) {
      // Found a trigger command - check the gap since last trigger
      if (found_trigger && words_since_last_trigger > max_gap) {
        max_gap = words_since_last_trigger;
      }
      found_trigger = true;
      words_since_last_trigger = words_needed; // Reset counter with current command
    } else {
      words_since_last_trigger += words_needed;
    }
  }
  
  // Check the final gap (from last trigger to end)
  if (found_trigger && words_since_last_trigger > max_gap) {
    max_gap = words_since_last_trigger;
  }
  
  // Warn if any gap exceeds FIFO size
  if (!found_trigger) {
    // No trigger commands found - check if total command size exceeds FIFO
    int total_words = words_since_last_trigger;
    if (total_words > DAC_CMD_FIFO_WORDCOUNT) {
      printf("WARNING: Waveform contains no triggers and requires %d words, which exceeds DAC FIFO size (%u words).\n", 
             total_words, DAC_CMD_FIFO_WORDCOUNT);
      printf("         This may cause FIFO overflow during streaming. Consider adding trigger commands, keeping delays long, or reducing waveform size.\n");
    } else if (*(ctx->verbose)) {
      printf("No trigger validation: Waveform requires %d words (FIFO size: %u words) - OK\n", 
             total_words, DAC_CMD_FIFO_WORDCOUNT);
    }
  } else if (max_gap > DAC_CMD_FIFO_WORDCOUNT) {
    printf("WARNING: Maximum gap between triggers is %d words, which exceeds DAC FIFO size (%u words).\n", 
           max_gap, DAC_CMD_FIFO_WORDCOUNT);
    printf("         This may cause FIFO underflow during streaming. Consider reducing number of delay commands between triggers or keeping delays long.\n");
  } else if (*(ctx->verbose)) {
    printf("Trigger gap validation: Maximum gap is %d words (FIFO size: %u words) - OK\n", 
           max_gap, DAC_CMD_FIFO_WORDCOUNT);
  }
  
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
  
  if (*(ctx->verbose)) {
    printf("Started DAC command streaming for board %d from file '%s' (looping %d time%s)\n", 
           board, full_path, loop_count, loop_count == 1 ? "" : "s");
  }
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

// DAC zero command - set all DAC channels to calibrated zero
int cmd_dac_zero(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Validate arguments - must have exactly 1 argument (board number or "all")
  if (arg_count != 1) {
    printf("Error: dac_zero requires exactly one argument: board number (0-7) or 'all'\n");
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  bool target_all = false;
  bool target_boards[8] = {false};
  int target_board = -1;
  
  // Parse the argument
  if (strcasecmp(args[0], "all") == 0) {
    target_all = true;
    printf("Setting all DAC channels to calibrated zero on all connected boards...\n");
  } else {
    target_board = validate_board_number(args[0]);
    if (target_board < 0) {
      printf("Error: Invalid board number '%s'. Must be 0-7 or 'all'\n", args[0]);
      return -1;
    }
    target_boards[target_board] = true;
    printf("Setting all DAC channels to calibrated zero on board %d...\n", target_board);
  }
  
  // Check which boards are connected and validate targets
  bool connected_boards[8] = {false};
  int connected_count = 0;
  
  if (*(ctx->verbose)) {
    printf("Checking board connections...\n");
  }
  
  for (int board = 0; board < 8; board++) {
    // Check if DAC command stream is running for this board
    if (ctx->dac_cmd_stream_running[board]) {
      if (target_all || target_boards[board]) {
        fprintf(stderr, "Cannot zero DAC channels on board %d: DAC command stream is currently running. Stop the stream first.\n", board);
        return -1;
      }
    }
    
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    
    if (FIFO_PRESENT(dac_cmd_fifo_status) && FIFO_PRESENT(dac_data_fifo_status)) {
      connected_boards[board] = true;
      connected_count++;
      if (*(ctx->verbose)) {
        printf("  Board %d: Connected\n", board);
      }
    } else {
      if (*(ctx->verbose)) {
        printf("  Board %d: Not connected\n", board);
      }
      // If targeting a specific board that's not connected, error
      if (!target_all && target_boards[board]) {
        printf("Error: Board %d is not connected\n", board);
        return -1;
      }
    }
  }
  
  if (connected_count == 0) {
    printf("No DAC boards are connected. Nothing to zero.\n");
    return 0;
  }
  
  printf("Found %d connected DAC board(s)\n", connected_count);
  
  // Reset only DAC command buffers for connected target boards that have commands (unless --no_reset flag is used)
  if (!skip_reset) {
    if (*(ctx->verbose)) {
      printf("Resetting DAC command buffers for target boards with commands...\n");
    }
    
    uint32_t cmd_reset_mask = 0;
    
    // Check DAC command buffers for target boards and build reset mask
    for (int board = 0; board < 8; board++) {
      if (connected_boards[board] && (target_all || target_boards[board])) {
        uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        if (FIFO_PRESENT(dac_cmd_fifo_status) && FIFO_STS_WORD_COUNT(dac_cmd_fifo_status) > 0) {
          uint32_t dac_bit = board * 2;  // DAC command buffers: bits 0, 2, 4, 6, 8, 10, 12, 14
          cmd_reset_mask |= (1U << dac_bit);
          if (*(ctx->verbose)) {
            printf("  Board %d DAC command buffer: has %d entries - will reset\n", 
                   board, FIFO_STS_WORD_COUNT(dac_cmd_fifo_status));
          }
        } else if (*(ctx->verbose)) {
          printf("  Board %d DAC command buffer: empty - skipping reset\n", board);
        }
      }
    }
    
    // Apply DAC command buffer resets if any are needed
    if (cmd_reset_mask != 0) {
      if (*(ctx->verbose)) {
        printf("Setting DAC command buffer reset mask: 0x%05X\n", cmd_reset_mask);
      }
      sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, cmd_reset_mask, *(ctx->verbose));
      usleep(1000); // 1ms delay
      sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    } else if (*(ctx->verbose)) {
      printf("No DAC command buffers need resetting\n");
    }
    
    __sync_synchronize(); // Memory barrier
    usleep(10000); // 10ms delay
  } else {
    if (*(ctx->verbose)) {
      printf("Skipping buffer reset (--no_reset flag specified)\n");
    }
  }
  
  // Send cancel commands to target boards
  if (*(ctx->verbose)) {
    printf("Sending CANCEL commands to target boards...\n");
  }
  for (int board = 0; board < 8; board++) {
    if (connected_boards[board] && (target_all || target_boards[board])) {
      dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
    }
  }
  usleep(1000); // 1ms to let cancel commands complete
  
  // Zero all channels on target boards using the new ZERO command
  int channels_zeroed = 0;
  int boards_zeroed = 0;
  
  printf("Setting DAC channels to calibrated zero values...\n");
  for (int board = 0; board < 8; board++) {
    if (connected_boards[board] && (target_all || target_boards[board])) {
      // Send DAC ZERO command - sets all channels to their calibrated midrange values
      dac_cmd_zero(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
      channels_zeroed += 8; // 8 channels per board
      boards_zeroed++;
      
      if (*(ctx->verbose)) {
        printf("  Board %d: All 8 channels set to calibrated zero\n", board);
      } else {
        printf("  Board %d: Zeroed\n", board);
      }
    }
  }
  
  // Brief pause to let the DAC commands complete
  usleep(1000); // 1ms delay
  
  printf("Successfully zeroed %d DAC channels on %d board(s) using calibrated zero values\n", 
         channels_zeroed, boards_zeroed);
  
  return 0;
}


