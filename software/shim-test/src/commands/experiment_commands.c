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
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open ADC command file '%s': %s\n", file_path, strerror(errno));
    return 0;
  }
  
  char line[512];
  uint64_t samples_per_loop = 0;
  int i = 0;
  
  // Parse the file to count samples per loop iteration
  while (fgets(line, sizeof(line), file)) {
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    char cmd_type = *trimmed;
    uint32_t value = 0;
    
    if (cmd_type == 'L') {
      // Loop command: L <value>
      if (sscanf(trimmed, "L %u", &value) == 1) {
        // This will repeat the next command 'value' times
        // We need to look at the next command to see what gets repeated
        if (fgets(line, sizeof(line), file)) {
          char* next_trimmed = line;
          while (*next_trimmed == ' ' || *next_trimmed == '\t') next_trimmed++;
          if (*next_trimmed != '\n' && *next_trimmed != '\r' && *next_trimmed != '\0' && *next_trimmed != '#') {
            char next_cmd = *next_trimmed;
            if (next_cmd == 'T' || next_cmd == 'D') {
              // T and D commands generate samples
              samples_per_loop += value;
            }
            // O commands don't generate samples by themselves
          }
        }
      }
    } else if (cmd_type == 'T' || cmd_type == 'D') {
      // Trigger and Delay commands generate one sample each
      samples_per_loop++;
    }
    // O (Order) commands don't generate samples by themselves
  }
  
  fclose(file);
  
  uint64_t total_samples = samples_per_loop * loop_count;
  printf("Calculated %llu samples per loop, %llu total samples (%d loops)\n", 
         samples_per_loop, total_samples, loop_count);
  
  return total_samples;
}

// Channel test command implementation
int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (arg_count < 2) {
    fprintf(stderr, "Usage: channel_test <channel> <value>\n");
    return -1;
  }
  
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Parse channel number and calculate board/channel
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) {
    return -1;
  }
  
  // Parse DAC value
  char* endptr;
  int dac_value = (int)parse_value(args[1], &endptr);
  if (*endptr != '\0' || dac_value < -32767 || dac_value > 32767) {
    fprintf(stderr, "Invalid DAC value: '%s'. Must be -32767 to 32767.\n", args[1]);
    return -1;
  }
  
  printf("Starting channel test for channel %d (board %d, channel %d), value %d\n", 
         atoi(args[0]), board, channel, dac_value);
  
  // Step 1: Check that the system is on (already done above)
  printf("  Step 1: System is running\n");
  
  // Step 2: Reset the ADC and DAC buffers for that board
  printf("  Step 2: Resetting ADC and DAC buffers for board %d\n", board);
  uint32_t board_mask = 3 << (2 * board);
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  usleep(10000); // 10ms
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(10000); // 10ms

  // Step 3: Send cancel command to DAC and ADC for that board
  printf("  Step 3: Sending CANCEL command to DAC and ADC for board %d\n", board);
  dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
  usleep(10000); // 10ms
  
  // Step 4: Send wr_ch command to DAC, 100ms delay, then and rd_ch command to ADC
  printf("  Step 4: Sending commands to DAC and ADC\n");
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)dac_value, *(ctx->verbose));
  usleep(100000); // 100ms
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  // Wait a short moment to ensure ADC data is ready
  usleep(100000); // 100ms

  // Step 6: Send wr_ch command to DAC to set channel back to 0
  printf("  Step 5: Resetting DAC to 0\n");
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));
  
  // Step 7: Read single from ADC
  printf("  Step 6: Reading ADC value\n");
  int16_t adc_reading = ADC_OFFSET_TO_SIGNED(adc_read_word(ctx->adc_ctrl, (uint8_t)board) & 0xFFFF);
  
  // Step 8: Calculate and print error
  printf("  Step 7: Calculating error\n");
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

  // Make sure the system is NOT running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state == S_RUNNING) {
    printf("Error: Hardware manager is currently running (state: %u). Use 'off' command first.\n", state);
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
  
  // Step 8: Calculate expected number of samples from ADC command file
  uint64_t sample_count = calculate_expected_samples(adc_file, loops);
  if (sample_count == 0) {
    fprintf(stderr, "Failed to calculate expected sample count from ADC command file\n");
    return -1;
  }
  
  // Step 9: Count trigger lines in DAC file
  int trigger_count = count_trigger_lines_in_file(dac_file);
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
  printf("Starting DAC command streaming from file '%s' (%d loops)\n", dac_file, loops);
  char board_str[16], loops_str[16];
  snprintf(board_str, sizeof(board_str), "%d", board);
  snprintf(loops_str, sizeof(loops_str), "%d", loops);
  
  const char* dac_args[] = {board_str, dac_file, loops_str};
  if (cmd_stream_dac_commands_from_file(dac_args, 3, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start DAC command streaming\n");
    return -1;
  }
  
  // Step 12: Start ADC command streaming using existing function (with simple flag)
  printf("Starting ADC command streaming from file '%s' (%d loops, simple mode)\n", adc_file, loops);
  
  const char* adc_args[] = {board_str, adc_file, loops_str};
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
  if (cmd_read_adc_to_file(adc_data_args, 3, NULL, 0, ctx) != 0) {
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
