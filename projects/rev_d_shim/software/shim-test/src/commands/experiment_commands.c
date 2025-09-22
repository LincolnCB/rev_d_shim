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
#include "dac_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "trigger_ctrl.h"

// Forward declarations for helper functions
static int validate_system_running(command_context_t* ctx);
static int count_trigger_lines_in_file(const char* file_path);
static uint64_t calculate_expected_samples(const char* file_path, int loop_count);

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

// Helper function to calculate expected number of samples from an ADC command file
static uint64_t calculate_expected_samples(const char* file_path, int loop_count) {
  // Use the existing ADC command parser to get the parsed commands
  adc_command_t* commands = NULL;
  int command_count = 0;
  
  // Parse the ADC command file using the existing function
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open ADC command file '%s': %s\n", file_path, strerror(errno));
    return 0;
  }
  
  // First pass: count valid lines (excluding comments and empty lines)
  char line[512];
  int valid_lines = 0;
  
  while (fgets(line, sizeof(line), file)) {
    // Skip empty lines and comments (same logic as parse_adc_command_file)
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Check if line starts with valid command
    if (*trimmed == 'L' || *trimmed == 'T' || *trimmed == 'D' || *trimmed == 'O') {
      valid_lines++;
    }
  }
  
  if (valid_lines == 0) {
    fclose(file);
    return 0;
  }
  
  // Allocate memory for commands
  commands = malloc(valid_lines * sizeof(adc_command_t));
  if (commands == NULL) {
    fclose(file);
    return 0;
  }
  
  // Second pass: parse commands (same logic as parse_adc_command_file)
  rewind(file);
  command_count = 0;
  
  while (fgets(line, sizeof(line), file)) {
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    adc_command_t* cmd = &commands[command_count];
    cmd->type = *trimmed;
    
    if (cmd->type == 'L') {
      if (sscanf(trimmed, "L %u", &cmd->value) != 1) {
        free(commands);
        fclose(file);
        return 0;
      }
    } else if (cmd->type == 'T') {
      if (sscanf(trimmed, "T %u", &cmd->value) != 1) {
        free(commands);
        fclose(file);
        return 0;
      }
    } else if (cmd->type == 'D') {
      if (sscanf(trimmed, "D %u", &cmd->value) != 1) {
        free(commands);
        fclose(file);
        return 0;
      }
    } else if (cmd->type == 'O') {
      int order_vals[8];
      if (sscanf(trimmed, "O %d %d %d %d %d %d %d %d", 
                 &order_vals[0], &order_vals[1], &order_vals[2], &order_vals[3],
                 &order_vals[4], &order_vals[5], &order_vals[6], &order_vals[7]) != 8) {
        free(commands);
        fclose(file);
        return 0;
      }
      for (int j = 0; j < 8; j++) {
        cmd->order[j] = (uint8_t)order_vals[j];
      }
    }
    
    command_count++;
  }
  
  fclose(file);
  
  // Now simulate the execution in simple mode to count samples
  uint64_t samples_per_loop = 0;
  
  for (int i = 0; i < command_count; i++) {
    adc_command_t* cmd = &commands[i];
    
    switch (cmd->type) {
      case 'L':
        // In simple mode, this repeats the next command cmd->value times
        if (i + 1 < command_count) {
          adc_command_t* next_cmd = &commands[i + 1];
          if (next_cmd->type == 'T' || next_cmd->type == 'D') {
            // T and D commands generate 4 words each
            samples_per_loop += cmd->value * 4;
          }
          // Skip the next command since loop consumes it
          i++;
        }
        break;
      
      case 'T':
      case 'D':
        // These generate 4 words each (if not consumed by a loop)
        samples_per_loop += 4;
        break;
      
      case 'O':
        // Order commands don't generate samples
        break;
    }
  }
  
  free(commands);
  
  uint64_t total_samples = samples_per_loop * loop_count;
  printf("Calculated %llu samples per loop, %llu total samples (%d loops)\n", 
         samples_per_loop, total_samples, loop_count);
  
  return total_samples;
}

// Channel test command implementation
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (arg_count != 2) {
    fprintf(stderr, "Usage: channel_test <channel> <value>\n");
    fflush(stderr);
    return -1;
  }
  
  // Parse channel number and calculate board/channel
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    fflush(stdout);
    return -1;
  }
  
  // Parse DAC value
  char* endptr;
  int dac_value = (int)parse_value(args[1], &endptr);
  if (*endptr != '\0' || dac_value < -32767 || dac_value > 32767) {
    fprintf(stderr, "Invalid DAC value: '%s'. Must be -32767 to 32767.\n", args[1]);
    fflush(stderr);
    return -1;
  }
  
  // Step 1: Check that the system is on
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    fflush(stdout);
    return -1;
  }
  // Check if ADC data FIFO is present before attempting to read
  uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (FIFO_PRESENT(adc_data_fifo_status) == 0) {
    fprintf(stderr, "ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    fflush(stderr);
    return -1;
  }
  printf("  Step 1: System is running\n");
  fflush(stdout);
  
  printf("Starting channel test for channel %d (board %d, channel %d), value %d\n", 
         atoi(args[0]), board, channel, dac_value);
  fflush(stdout);
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Step 2: Reset the ADC and DAC buffers for all boards (unless --no_reset flag is used)
  if (skip_reset) {
    printf("  Step 2: Skipping buffer reset (--no_reset flag specified)\n");
    fflush(stdout);
  } else {
    printf("  Step 2: Resetting ADC and DAC buffers for all boards\n");
    fflush(stdout);
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose)); // Reset all boards + trigger
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(100000); // 100ms delay
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(100000); // 100ms delay
    printf("  Buffer resets completed\n");
    fflush(stdout);
  }

  // Step 3: Send cancel command to DAC and ADC for that board
  printf("  Step 3: Sending CANCEL command to DAC and ADC for board %d\n", board);
  fflush(stdout);
  dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
  __sync_synchronize(); // Memory barrier
  usleep(10000); // 10ms delay
  printf("  Cancel commands completed\n");
  fflush(stdout);
  
  // Step 4: Send wr_ch command to DAC, 100ms delay, then and rd_ch command to ADC
  printf("  Step 4: Sending commands to DAC and ADC\n");
  fflush(stdout);
  printf("    Writing DAC value %d to board %d, channel %d\n", dac_value, board, channel);
  fflush(stdout);
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)dac_value, *(ctx->verbose));
  __sync_synchronize(); // Memory barrier
  usleep(100000); // 100ms delay to let DAC settle
  printf("    Reading ADC from board %d, channel %d\n", board, channel);
  fflush(stdout);
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  // Wait a short moment to ensure ADC data is ready
  __sync_synchronize(); // Memory barrier
  usleep(100000); // 100ms delay
  printf("  DAC/ADC commands completed\n");
  fflush(stdout);

  // Step 6: Send wr_ch command to DAC to set channel back to 0
  printf("  Step 5: Resetting DAC to 0\n");
  fflush(stdout);
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));

  // Brief pause to let the DAC settle
  __sync_synchronize(); // Memory barrier
  usleep(100000); // 100ms 
  printf("  DAC reset to 0 completed\n");
  fflush(stdout);
  
  // Step 7: Read single from ADC
  printf("  Step 6: Reading ADC value\n");
  fflush(stdout);
  
  // Add some debug info before attempting to read
  printf("    Checking ADC data FIFO status before reading...\n");
  fflush(stdout);
  
  // Check if the ADC data FIFO has data
  fflush(stdout);
  adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) == 0) {
    fprintf(stderr, "ADC data buffer is still empty after waiting 1 second.\n");
    fflush(stderr);
    return -1;
  } else {
    printf("    ADC data FIFO has %u words available.\n", FIFO_STS_WORD_COUNT(adc_data_fifo_status));
    fflush(stdout);
  }
  
  // Add a memory barrier and small delay to ensure hardware state is stable
  __sync_synchronize(); // Memory barrier
  usleep(50000); // Additional 50ms delay for hardware settling
  
  uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
  int16_t adc_reading = ADC_OFFSET_TO_SIGNED(adc_word & 0xFFFF);
  __sync_synchronize(); // Memory barrier after critical read to ensure completion
  
  // Step 8: Calculate and print error
  printf("  Step 7: Calculating error\n");
  fflush(stdout);
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
  fflush(stdout);
  
  printf("Channel test completed.\n");
  fflush(stdout);
  return 0;
}

// Waveform test command implementation
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Starting interactive waveform test...\n");

  // Make sure the system is NOT running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state == S_RUNNING) {
    printf("Error: Hardware manager is currently running (state: %u). Use 'off' command first.\n", state);
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Step 1: Reset all buffers (unless --no_reset flag is used)
  if (skip_reset) {
    printf("Step 1: Skipping buffer reset (--no_reset flag specified)\n");
  } else {
    printf("Step 1: Resetting all buffers\n");
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose)); // Reset all boards + trigger
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
    usleep(10000); // 10ms
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
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
  
  // Resolve DAC file glob pattern
  char resolved_dac_file[1024];
  if (resolve_file_pattern(dac_file, resolved_dac_file, sizeof(resolved_dac_file)) != 0) {
    fprintf(stderr, "Failed to resolve DAC file pattern: '%s'\n", dac_file);
    return -1;
  }
  
  // Step 4: Prompt for ADC command file
  char adc_file[1024];
  printf("Enter ADC command file path: ");
  if (scanf("%1023s", adc_file) != 1) {
    fprintf(stderr, "Failed to read ADC file path.\n");
    return -1;
  }
  
  // Resolve ADC file glob pattern
  char resolved_adc_file[1024];
  if (resolve_file_pattern(adc_file, resolved_adc_file, sizeof(resolved_adc_file)) != 0) {
    fprintf(stderr, "Failed to resolve ADC file pattern: '%s'\n", adc_file);
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
  
  // Step 8: Calculate expected number of samples from ADC command file
  uint64_t sample_count = calculate_expected_samples(resolved_adc_file, loops);
  if (sample_count == 0) {
    fprintf(stderr, "Failed to calculate expected sample count from ADC command file\n");
    return -1;
  }
  
  // Step 9: Count trigger lines in DAC file
  int trigger_count = count_trigger_lines_in_file(resolved_dac_file);
  if (trigger_count < 0) {
    return -1;
  }
  
  uint32_t total_expected_triggers = trigger_count * loops;
  printf("Expecting %d total external triggers (%d triggers x %d loops)\n", 
         total_expected_triggers, trigger_count, loops);
  
  // Step 10: Set trigger lockout and expect external triggers
  printf("Setting trigger lockout time to %u cycles\n", lockout_time);
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_time);
  
  if (total_expected_triggers > 0) {
    printf("Setting expected external triggers to %u\n", total_expected_triggers);
    trigger_cmd_expect_ext(ctx->trigger_ctrl, total_expected_triggers);
  }
  
  // Step 11: Start DAC command streaming using existing function
  printf("Starting DAC command streaming from file '%s' (%d loops)\n", resolved_dac_file, loops);
  char board_str[16], loops_str[16];
  snprintf(board_str, sizeof(board_str), "%d", board);
  snprintf(loops_str, sizeof(loops_str), "%d", loops);
  
  const char* dac_args[] = {board_str, resolved_dac_file, loops_str};
  if (cmd_stream_dac_commands_from_file(dac_args, 3, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start DAC command streaming\n");
    return -1;
  }
  
  // Step 12: Start ADC command streaming using existing function (with simple flag)
  printf("Starting ADC command streaming from file '%s' (%d loops, simple mode)\n", resolved_adc_file, loops);
  
  const char* adc_args[] = {board_str, resolved_adc_file, loops_str};
  command_flag_t simple_flag = {FLAG_SIMPLE, NULL};
  if (cmd_stream_adc_commands_from_file(adc_args, 3, &simple_flag, 1, ctx) != 0) {
    fprintf(stderr, "Failed to start ADC command streaming\n");
    return -1;
  }
  
  // Step 13: Start ADC data streaming to output file using existing function
  printf("Starting ADC data streaming to file '%s' (%llu samples)\n", output_file, sample_count);
  char sample_count_str[32];
  snprintf(sample_count_str, sizeof(sample_count_str), "%llu", sample_count);
  
  const char* adc_data_args[] = {board_str, sample_count_str, output_file};
  if (cmd_stream_adc_data_to_file(adc_data_args, 3, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start ADC data streaming\n");
    return -1;
  }
  
  printf("Waveform test setup completed. All streaming started successfully.\n");
  printf("Use the following commands to monitor and stop streams:\n");
  printf("  - 'stop_dac_cmd_stream %d' to stop DAC command streaming\n", board);
  printf("  - 'stop_adc_cmd_stream %d' to stop ADC command streaming\n", board);
  printf("  - 'stop_adc_data_stream %d' to stop ADC data streaming\n", board);
  
  return 0;
}
