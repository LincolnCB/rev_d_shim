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
#include <math.h>
#include "experiment_commands.h"
#include "command_helper.h"
#include "adc_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "trigger_ctrl.h"

// Forward declarations for helper functions
static int validate_system_running(command_context_t* ctx);
static int count_trigger_lines_in_file(const char* file_path);
static int parse_adc_experiment_file(const char* file_path, adc_command_t** commands, int* command_count, bool simple_mode);
static void* adc_experiment_stream_thread(void* arg);

// Structure to pass data to the ADC experiment streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  adc_command_t* commands;
  int command_count;
  int loop_count;
  bool simple_mode;
} adc_experiment_stream_params_t;

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

// Helper function to count trigger lines in a DAC file
static int count_trigger_lines_in_file(const char* file_path) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open DAC file '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  char line[512];
  int trigger_count = 0;
  
  while (fgets(line, sizeof(line), file)) {
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Check if line starts with T (trigger)
    if (*trimmed == 'T') {
      trigger_count++;
    }
  }
  
  fclose(file);
  return trigger_count;
}

// Enhanced ADC command parser that supports the new command format
static int parse_adc_experiment_file(const char* file_path, adc_command_t** commands, int* command_count, bool simple_mode) {
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
  
  // Second pass: parse commands
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
    
    if (cmd->type == 'L') {
      // Loop command: L <value>
      if (sscanf(trimmed, "L %u", &cmd->value) != 1) {
        fprintf(stderr, "Invalid loop command on line %d\n", line_num);
        free(*commands);
        fclose(file);
        return -1;
      }
    } else if (cmd->type == 'T') {
      // Trigger command: T <value>
      if (sscanf(trimmed, "T %u", &cmd->value) != 1) {
        fprintf(stderr, "Invalid trigger command on line %d\n", line_num);
        free(*commands);
        fclose(file);
        return -1;
      }
    } else if (cmd->type == 'D') {
      // Delay command: D <value>
      if (sscanf(trimmed, "D %u", &cmd->value) != 1) {
        fprintf(stderr, "Invalid delay command on line %d\n", line_num);
        free(*commands);
        fclose(file);
        return -1;
      }
    } else if (cmd->type == 'O') {
      // Order command: O <s1> <s2> ... <s8>
      int order_vals[8];
      if (sscanf(trimmed, "O %d %d %d %d %d %d %d %d", 
                 &order_vals[0], &order_vals[1], &order_vals[2], &order_vals[3],
                 &order_vals[4], &order_vals[5], &order_vals[6], &order_vals[7]) != 8) {
        fprintf(stderr, "Invalid order command on line %d: must have 8 values\n", line_num);
        free(*commands);
        fclose(file);
        return -1;
      }
      
      // Validate order values (0-7)
      for (int i = 0; i < 8; i++) {
        if (order_vals[i] < 0 || order_vals[i] > 7) {
          fprintf(stderr, "Invalid order value %d on line %d: must be 0-7\n", order_vals[i], line_num);
          free(*commands);
          fclose(file);
          return -1;
        }
        cmd->order[i] = (uint8_t)order_vals[i];
      }
    }
    
    (*command_count)++;
  }
  
  fclose(file);
  return 0;
}

// ADC experiment streaming thread
static void* adc_experiment_stream_thread(void* arg) {
  adc_experiment_stream_params_t* stream_data = (adc_experiment_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  
  printf("ADC experiment streaming thread started for board %d\n", board);
  
  for (int loop = 0; loop < stream_data->loop_count && !*(stream_data->should_stop); loop++) {
    if (*(ctx->verbose)) {
      printf("ADC experiment loop %d/%d for board %d\n", loop + 1, stream_data->loop_count, board);
    }
    
    for (int i = 0; i < stream_data->command_count && !*(stream_data->should_stop); i++) {
      adc_command_t* cmd = &stream_data->commands[i];
      
      switch (cmd->type) {
        case 'L':
          if (stream_data->simple_mode) {
            // In simple mode, unroll the loop by repeating the next command
            if (i + 1 < stream_data->command_count) {
              adc_command_t* next_cmd = &stream_data->commands[i + 1];
              for (uint32_t j = 0; j < cmd->value && !*(stream_data->should_stop); j++) {
                // Execute the next command
                switch (next_cmd->type) {
                  case 'T':
                    adc_cmd_adc_rd(ctx->adc_ctrl, board, true, false, next_cmd->value, *(ctx->verbose));
                    break;
                  case 'D':
                    adc_cmd_adc_rd(ctx->adc_ctrl, board, false, false, next_cmd->value, *(ctx->verbose));
                    break;
                  case 'O':
                    adc_cmd_set_ord(ctx->adc_ctrl, board, next_cmd->order, *(ctx->verbose));
                    break;
                }
              }
              i++; // Skip the next command since we already executed it
            }
          } else {
            // Use hardware loop command
            adc_cmd_loop_next(ctx->adc_ctrl, board, cmd->value, *(ctx->verbose));
          }
          break;
        
        case 'T':
          adc_cmd_adc_rd(ctx->adc_ctrl, board, true, false, cmd->value, *(ctx->verbose));
          break;
        
        case 'D':
          adc_cmd_adc_rd(ctx->adc_ctrl, board, false, false, cmd->value, *(ctx->verbose));
          break;
        
        case 'O':
          adc_cmd_set_ord(ctx->adc_ctrl, board, cmd->order, *(ctx->verbose));
          break;
      }
      
      // Brief delay between commands
      usleep(1000); // 1ms
    }
  }
  
  printf("ADC experiment streaming for board %d completed\n", board);
  
  // Clean up
  ctx->adc_stream_running[board] = false;
  free(stream_data->commands);
  free(stream_data);
  return NULL;
}

// Channel test command implementation
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (arg_count < 3) {
    fprintf(stderr, "Usage: channel_test <board> <channel> <value>\n");
    return -1;
  }
  
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Parse board number
  int board = validate_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse channel number
  char* endptr;
  int channel = (int)parse_value(args[1], &endptr);
  if (*endptr != '\0' || channel < 0 || channel > 7) {
    fprintf(stderr, "Invalid channel number: '%s'. Must be 0-7.\n", args[1]);
    return -1;
  }
  
  // Parse DAC value
  int dac_value = (int)parse_value(args[2], &endptr);
  if (*endptr != '\0' || dac_value < -32767 || dac_value > 32767) {
    fprintf(stderr, "Invalid DAC value: '%s'. Must be -32767 to 32767.\n", args[2]);
    return -1;
  }
  
  printf("Starting channel test for board %d, channel %d, value %d\n", board, channel, dac_value);
  
  // Step 1: Check that the system is on (already done above)
  printf("  Step 1: System is running ✓\n");
  
  // Step 2: Reset the ADC and DAC buffers for that board
  printf("  Step 2: Resetting ADC and DAC buffers for board %d\n", board);
  uint32_t board_mask = 1 << board;
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  usleep(10000); // 10ms
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(10000); // 10ms
  
  // Step 3: Send wr_ch command to DAC, 100000 cycle delay to ADC, and rd_ch command to ADC
  printf("  Step 3: Sending commands to DAC and ADC\n");
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)dac_value, *(ctx->verbose));
  adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, 100000, *(ctx->verbose)); // 100000 cycle delay
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  
  // Step 4: Sleep 10 ms
  printf("  Step 4: Waiting 10ms for ADC conversion\n");
  usleep(10000); // 10ms
  
  // Step 5: Read single from ADC
  printf("  Step 5: Reading ADC value\n");
  int16_t adc_reading = adc_read_ch(ctx->adc_ctrl, (uint8_t)board);
  
  // Step 6: Calculate and print error
  printf("  Step 6: Calculating error\n");
  printf("    DAC value set: %d\n", dac_value);
  printf("    ADC value read: %d\n", adc_reading);
  
  int absolute_error = abs(adc_reading - dac_value);
  double percent_error = 0.0;
  if (dac_value != 0) {
    percent_error = (double)absolute_error / abs(dac_value) * 100.0;
  } else if (adc_reading != 0) {
    percent_error = 100.0; // If we set 0 but read non-zero, it's 100% error
  }
  
  printf("    Absolute error: %d\n", absolute_error);
  printf("    Percent error: %.2f%%\n", percent_error);
  
  printf("Channel test completed.\n");
  return 0;
}

// Waveform test command implementation
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Starting interactive waveform test...\n");
  
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Step 1: Reset all buffers
  printf("Step 1: Resetting all buffers\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose)); // Reset all boards + trigger
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  usleep(10000); // 10ms
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(10000); // 10ms
  
  // Step 2: Prompt for board number
  int board;
  printf("Enter board number (0-7): ");
  if (scanf("%d", &board) != 1 || board < 0 || board > 7) {
    fprintf(stderr, "Invalid board number. Must be 0-7.\n");
    return -1;
  }
  
  // Step 3: Prompt for DAC command file
  char dac_file[1024];
  printf("Enter DAC command file path: ");
  if (scanf("%1023s", dac_file) != 1) {
    fprintf(stderr, "Failed to read DAC file path.\n");
    return -1;
  }
  
  // Step 4: Prompt for ADC command file
  char adc_file[1024];
  printf("Enter ADC command file path: ");
  if (scanf("%1023s", adc_file) != 1) {
    fprintf(stderr, "Failed to read ADC file path.\n");
    return -1;
  }
  
  // Step 5: Prompt for number of loops
  int loops;
  printf("Enter number of loops: ");
  if (scanf("%d", &loops) != 1 || loops < 1) {
    fprintf(stderr, "Invalid number of loops. Must be >= 1.\n");
    return -1;
  }
  
  // Step 6: Prompt for output file
  char output_file[1024];
  printf("Enter output file path: ");
  if (scanf("%1023s", output_file) != 1) {
    fprintf(stderr, "Failed to read output file path.\n");
    return -1;
  }
  
  // Step 7: Prompt for trigger lockout time
  uint32_t lockout_time;
  printf("Enter trigger lockout time (cycles): ");
  if (scanf("%u", &lockout_time) != 1) {
    fprintf(stderr, "Invalid trigger lockout time.\n");
    return -1;
  }
  
  // Step 8: Count trigger lines in DAC file
  int trigger_count = count_trigger_lines_in_file(dac_file);
  if (trigger_count < 0) {
    return -1;
  }
  
  uint32_t total_expected_triggers = trigger_count * loops;
  printf("Expecting %d total external triggers (%d triggers × %d loops)\n", 
         total_expected_triggers, trigger_count, loops);
  
  // Step 9: Set trigger lockout and expect external triggers
  printf("Setting trigger lockout time to %u cycles\n", lockout_time);
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_time);
  
  if (total_expected_triggers > 0) {
    printf("Setting expected external triggers to %u\n", total_expected_triggers);
    trigger_cmd_expect_ext(ctx->trigger_ctrl, total_expected_triggers);
  }
  
  // Step 10: Start DAC streaming
  printf("Starting DAC streaming from file '%s'\n", dac_file);
  // Note: This would need to call the DAC streaming function
  // For now, we'll print what we would do
  printf("  [Would start DAC streaming for board %d, file '%s', %d loops]\n", board, dac_file, loops);
  
  // Step 11: Start ADC streaming with simple flag
  printf("Starting ADC streaming from file '%s' (simple mode)\n", adc_file);
  
  // Parse ADC command file
  adc_command_t* adc_commands = NULL;
  int adc_command_count = 0;
  
  if (parse_adc_experiment_file(adc_file, &adc_commands, &adc_command_count, true) != 0) {
    return -1;
  }
  
  // Check if ADC stream is already running
  if (ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is already running. Stopping it first.\n", board);
    ctx->adc_stream_stop[board] = true;
    if (ctx->adc_stream_running[board]) {
      pthread_join(ctx->adc_stream_threads[board], NULL);
    }
  }
  
  // Allocate thread data structure
  adc_experiment_stream_params_t* stream_data = malloc(sizeof(adc_experiment_stream_params_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for ADC stream data\n");
    free(adc_commands);
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, adc_file);
  stream_data->should_stop = &(ctx->adc_stream_stop[board]);
  stream_data->commands = adc_commands;
  stream_data->command_count = adc_command_count;
  stream_data->loop_count = loops;
  stream_data->simple_mode = true;
  
  // Initialize stop flag and mark stream as running
  ctx->adc_stream_stop[board] = false;
  ctx->adc_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->adc_stream_threads[board]), NULL, adc_experiment_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC experiment streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(adc_commands);
    free(stream_data);
    return -1;
  }
  
  // Step 12: Start output file streaming
  printf("Starting output streaming to file '%s'\n", output_file);
  printf("  [Would stream ADC data to output file, checking buffer levels]\n");
  printf("  [Would sleep 1000us if buffer empties below 4 samples]\n");
  
  printf("Waveform test setup completed. Streaming in progress...\n");
  printf("Use 'stop_adc_stream %d' to stop the ADC streaming when test is complete.\n", board);
  
  return 0;
}
