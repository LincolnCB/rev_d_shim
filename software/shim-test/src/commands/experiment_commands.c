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
#include "experiment_commands.h"
#include "command_helper.h"
#include "adc_commands.h"
#include "dac_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "map_memory.h"
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
  // Check if ADC, DAC, and command FIFOs are present before attempting to read
  uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);

  if (FIFO_PRESENT(adc_data_fifo_status) == 0) {
    fprintf(stderr, "ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    fflush(stderr);
    return -1;
  }
  if (FIFO_PRESENT(dac_cmd_fifo_status) == 0) {
    fprintf(stderr, "DAC command FIFO for board %d is not present. Cannot send DAC commands.\n", board);
    fflush(stderr);
    return -1;
  }
  if (FIFO_PRESENT(adc_cmd_fifo_status) == 0) {
    fprintf(stderr, "ADC command FIFO for board %d is not present. Cannot send ADC commands.\n", board);
    fflush(stderr);
    return -1;
  }
  if (*(ctx->verbose)) {
    printf("  Step 1: System is running\n");
    fflush(stdout);
  }
  
  printf("Starting channel test for channel %d (board %d, channel %d), value %d\n", 
         atoi(args[0]), board, channel, dac_value);
  fflush(stdout);
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Step 2: Reset the ADC and DAC buffers for all boards (unless --no_reset flag is used)
  if (skip_reset) {
    if (*(ctx->verbose)) {
      printf("  Step 2: Skipping buffer reset (--no_reset flag specified)\n");
      fflush(stdout);
    }
  } else {
    if (*(ctx->verbose)) {
      printf("  Step 2: Resetting ADC and DAC buffers for all boards\n");
      fflush(stdout);
    }
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose)); // Reset all boards + trigger
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(1000); // 1ms delay
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(1000); // 1ms delay
    if (*(ctx->verbose)) {
      printf("  Buffer resets completed\n");
      fflush(stdout);
    }
  }

  // Step 3: Send cancel command to DAC and ADC for that board
  if (*(ctx->verbose)) {
    printf("  Step 3: Sending CANCEL command to DAC and ADC for board %d\n", board);
    fflush(stdout);
  }
  dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
  __sync_synchronize(); // Memory barrier
  usleep(1000); // 1ms delay
  if (*(ctx->verbose)) {
    printf("  Cancel commands completed\n");
    fflush(stdout);
  }
  
  // Step 4: Send wr_ch command to DAC, 100ms delay, then and rd_ch command to ADC
  if (*(ctx->verbose)) {
    printf("  Step 4: Sending commands to DAC and ADC\n");
    fflush(stdout);
    printf("    Writing DAC value %d to board %d, channel %d\n", dac_value, board, channel);
    fflush(stdout);
  }
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, (int16_t)dac_value, *(ctx->verbose));
  __sync_synchronize(); // Memory barrier
  usleep(1000); // 1ms delay to let DAC settle
  if (*(ctx->verbose)) {
    printf("    Reading ADC from board %d, channel %d\n", board, channel);
    fflush(stdout);
  }
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
  // Wait a short moment to ensure ADC data is ready
  __sync_synchronize(); // Memory barrier
  usleep(1000); // 1ms delay
  if (*(ctx->verbose)) {
    printf("  DAC/ADC commands completed\n");
    fflush(stdout);
  }

  // Step 6: Send wr_ch command to DAC to set channel back to 0
  if (*(ctx->verbose)) {
    printf("  Step 5: Resetting DAC to 0\n");
    fflush(stdout);
  }
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));

  // Brief pause to let the DAC settle
  __sync_synchronize(); // Memory barrier
  usleep(1000); // 1ms 
  if (*(ctx->verbose)) {
    printf("  DAC reset to 0 completed\n");
    fflush(stdout);
  }
  
  // Step 7: Read single from ADC
  if (*(ctx->verbose)) {
    printf("  Step 6: Reading ADC value\n");
    fflush(stdout);
  
    // Add some debug info before attempting to read
    printf("    Checking ADC data FIFO status before reading...\n");
    fflush(stdout);
  }
  
  // Check if the ADC data FIFO has data
  fflush(stdout);
  adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) == 0) {
    fprintf(stderr, "ADC data buffer is empty.\n");
    fflush(stderr);
    return -1;
  } else {
    if (*(ctx->verbose)) {
      printf("    ADC data FIFO has %u words available.\n", FIFO_STS_WORD_COUNT(adc_data_fifo_status));
      fflush(stdout);
    }
  }
  __sync_synchronize(); // Memory barrier

  uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
  int16_t adc_reading = offset_to_signed(adc_word & 0xFFFF);
  
  // Step 8: Calculate and print error
  if (*(ctx->verbose)) {
    printf("  Step 7: Calculating error\n");
    fflush(stdout);
  }
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
  
  if (*(ctx->verbose)) {
    printf("Channel test completed.\n");
    fflush(stdout);
  }
  return 0;
}

// Channel calibration command implementation
int cmd_channel_cal(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse arguments - either a channel number (0-63) or --all flag
  bool calibrate_all = has_flag(flags, flag_count, FLAG_ALL);
  int start_ch = 0, end_ch = 0;
  bool connected_boards[8] = {false}; // Track which boards are connected
  
  if (calibrate_all && arg_count > 0) {
    fprintf(stderr, "Error: Cannot specify both channel number and --all flag\n");
    return -1;
  }
  
  if (!calibrate_all && arg_count != 1) {
    fprintf(stderr, "Usage: channel_cal <channel> [--no_reset] OR channel_cal --all [--no_reset]\n");
    return -1;
  }
  
  if (calibrate_all) {
    start_ch = 0;
    end_ch = 63;
    
    // Check which boards are connected by checking FIFOs
    int connected_count = 0;
    printf("Checking connected boards...\n");
    
    for (int board = 0; board < 8; board++) {
      uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(adc_data_fifo_status) && 
          FIFO_PRESENT(dac_cmd_fifo_status) && 
          FIFO_PRESENT(adc_cmd_fifo_status) &&
          FIFO_PRESENT(dac_data_fifo_status)) {
        connected_boards[board] = true;
        connected_count++;
        printf("  Board %d: Connected\n", board);
      } else {
        printf("  Board %d: Not connected\n", board);
      }
    }
    
    if (connected_count == 0) {
      printf("No boards are connected. Aborting calibration.\n");
      return -1;
    }
    
    printf("Starting calibration for all channels on %d connected board(s)\n", connected_count);
  } else {
    // Parse single channel number
    int board, channel;
    if (validate_channel_number(args[0], &board, &channel) < 0) {
      return -1;
    }
    start_ch = end_ch = atoi(args[0]);
    
    // Check if the board for this channel is connected
    uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    
    if (FIFO_PRESENT(adc_data_fifo_status) && 
        FIFO_PRESENT(dac_cmd_fifo_status) && 
        FIFO_PRESENT(adc_cmd_fifo_status) &&
        FIFO_PRESENT(dac_data_fifo_status)) {
      connected_boards[board] = true;
      printf("Starting calibration for channel %d (board %d connected)\n", start_ch, board);
    } else {
      printf("Error: Board %d for channel %d is not connected\n", board, start_ch);
      return -1;
    }
  }
  
  // Check that system is on
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Reset buffers once at the start (unless --no_reset flag is used)
  if (!skip_reset) {
    printf("Resetting all buffers...\n");
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, false); // Reset all boards + trigger
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, false);
    usleep(1000); // 1ms
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, false);
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, false);
    usleep(1000); // 1ms
  }
  
  // Send cancel commands once per connected board
  if (calibrate_all) {
    printf("Sending cancel commands to connected boards...\n");
    for (int board = 0; board < 8; board++) {
      if (connected_boards[board]) {
        dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, false);
        adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, false);
      }
    }
  } else {
    // For single channel, send cancel to its board
    int board = start_ch / 8;
    printf("Sending cancel commands to board %d...\n", board);
    dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, false);
    adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, false);
  }
  usleep(1000); // 1ms to let cancel commands complete
  
  // Calibration constants
  const int dac_values[] = {-3276, -1638, 0, 1638, 3276};
  const int num_dac_values = 5;
  const int average_count = 5;
  const int calibration_iterations = 3;
  const int delay_ms = 1; // 1ms delay
  
  // Iterate through all channels to calibrate
  for (int ch = start_ch; ch <= end_ch; ch++) {
    int board, channel;
    board = ch / 8;
    channel = ch % 8;
    
    // If calibrating all channels, skip boards that are not connected
    if (calibrate_all && !connected_boards[board]) {
      continue;
    }
    
    printf("Ch %02d : ", ch);
    fflush(stdout);
    
    // Get current DAC calibration value and store it
    dac_cmd_get_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, false);
    
    // Wait for calibration data to be available
    int tries = 0;
    uint32_t dac_data_fifo_status;
    while (tries < 100) {
      dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      if (FIFO_STS_WORD_COUNT(dac_data_fifo_status) > 0) break;
      usleep(100); // 0.1ms
      tries++;
    }
    
    if (FIFO_STS_WORD_COUNT(dac_data_fifo_status) == 0) {
      printf("-F- |\n");
      continue;
    }
    
    uint32_t original_cal_data_word = dac_read_data(ctx->dac_ctrl, (uint8_t)board);
    uint16_t original_cal_value = DAC_CAL_DATA_VAL(original_cal_data_word);
    int16_t current_cal_value = original_cal_value;
    
    // Perform calibration iterations
    bool calibration_failed = false;
    bool poor_linearity = false;
    int completed_iterations = 0;
    
    for (int iter = 0; iter < calibration_iterations && !calibration_failed && !poor_linearity; iter++) {
      // Arrays to store DAC values and corresponding averaged ADC readings
      double dac_vals[num_dac_values];
      double avg_adc_vals[num_dac_values];
      
      // Test each DAC value
      for (int i = 0; i < num_dac_values; i++) {
        int16_t dac_val = dac_values[i];
        dac_vals[i] = (double)dac_val;
        
        // Perform multiple reads and average them
        double sum_adc = 0.0;
        
        for (int avg = 0; avg < average_count; avg++) {
          // Write DAC value
          dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, dac_val, false);
          usleep(delay_ms * 1000); // Wait fixed delay
          
          // Read ADC value
          adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, false);
          
          // Wait for ADC data to be available (similar to DAC data loop)
          int adc_tries = 0;
          uint32_t adc_data_fifo_status;
          while (adc_tries < 100) {
            adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
            if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) > 0) break;
            usleep(100); // 0.1ms
            adc_tries++;
          }
          
          if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) == 0) {
            calibration_failed = true;
            break;
          }
          
          uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
          int16_t adc_reading = offset_to_signed(adc_word & 0xFFFF);
          sum_adc += (double)adc_reading;
        }
        
        if (calibration_failed) break;
        
        avg_adc_vals[i] = sum_adc / average_count;
      }
      
      if (calibration_failed) break;
      
      // Perform linear regression: y = mx + b
      // Calculate slope (m) and intercept (b)
      double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
      for (int i = 0; i < num_dac_values; i++) {
        sum_x += dac_vals[i];
        sum_y += avg_adc_vals[i];
        sum_xy += dac_vals[i] * avg_adc_vals[i];
        sum_x2 += dac_vals[i] * dac_vals[i];
      }
      
      // Check for division by zero before calculating slope
      double denominator = num_dac_values * sum_x2 - sum_x * sum_x;
      double slope, intercept;
      bool division_by_zero = false;
      
      if (denominator == 0) {
        division_by_zero = true;
        slope = 0; // Set to 0 to avoid using uninitialized value
        intercept = sum_y / num_dac_values; // Simple average for intercept
      } else {
        slope = (num_dac_values * sum_xy - sum_x * sum_y) / denominator;
        intercept = (sum_y - slope * sum_x) / num_dac_values;
      }
      
      // Check for poor linearity conditions
      bool this_iter_poor_linearity = false;
      
      // Check for division by zero (infinite slope)
      if (division_by_zero) {
        this_iter_poor_linearity = true;
      }
      // Check for slope outside acceptable range (< 0.99 or > 1.01)
      else if (slope < 0.95 || slope > 1.05) {
        this_iter_poor_linearity = true;
      }
      // Check for negative slope
      else if (slope < 0) {
        this_iter_poor_linearity = true;
      }

      // If verbose, print the updates to the calibration value
      if (*(ctx->verbose)) {
        printf("  Iteration %d: Current cal=%d, Slope=%.4f, Intercept=%.2f\n", 
               iter + 1, current_cal_value, slope, intercept);
        fflush(stdout);
      }
      
      // Update calibration value: subtract intercept from current cal value
      current_cal_value = current_cal_value - (int16_t)(intercept >= 0 ? intercept + 0.5 : intercept - 0.5);

      // Print update info if verbose
      if (*(ctx->verbose)) {
        printf("    Updated cal value to %d\n", current_cal_value);
        fflush(stdout);
      }

      // Clamp calibration value to valid range
      if (current_cal_value < -4095) {
        if (*(ctx->verbose)) {
          printf("    Calibration value clamped to -4095\n");
          fflush(stdout);
        }
        current_cal_value = -4095;
      }
      if (current_cal_value > 4095) {
        if (*(ctx->verbose)) {
          printf("    Calibration value clamped to 4095\n");
          fflush(stdout);
        }
        current_cal_value = 4095;
      }
      
      // Set new calibration value
      dac_cmd_set_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, current_cal_value, *(ctx->verbose));
      
      // Convert offset and slope to amps (range -4.0 to 4.0 for ±32767)
      double offset_amps = intercept * 4.0 / 32767.0;
      
      // If NOT verbose, print this iteration's results with special slope formatting
      if (!*(ctx->verbose)) {
        if (division_by_zero) {
          printf("%+.4f A ( inf.) | ", offset_amps);
        } else if (slope < 0) {
          printf("%+.4f A ( neg.) | ", offset_amps);
        } else if (slope > 9.99) {
          printf("%+.4f A (9.999) | ", offset_amps);
        } else {
          printf("%+.4f A (%.3f) | ", offset_amps, slope);
        }
      }
      
      completed_iterations++;
      
      // Set poor linearity flag if this iteration has issues
      if (this_iter_poor_linearity) {
        poor_linearity = true;
      }
      
      fflush(stdout);
    }

    // Zero the channel to finalize
    dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, false);
    usleep(1000); // 1ms to let DAC settle
    
    // Print spaces for skipped iterations to maintain column alignment
    for (int i = completed_iterations; i < calibration_iterations; i++) {
      if (*(ctx->verbose)) printf("  -- Skipped iteration number %d", i + 1);
      else printf("----------------- | "); // 18 spaces to match the format above
    }
    
    // Print final status
    if (*(ctx->verbose)) {
      if (calibration_failed) {
        printf(" Calibration FAILED (code bug)");
      } else if (poor_linearity) {
        printf(" Poor linearity (check connections)");
      } else {
        printf(" Calibration OK");
      }
    }
    else {
      if (calibration_failed) {
        printf("-F- |");
      } else if (poor_linearity) {
        printf("-X- |");
      } else {
        printf("--- |");
      }
    }
    
    printf("\n");
  }
  
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
    usleep(1000); // 1ms
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    usleep(1000); // 1ms
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
  command_flag_t simple_flag = FLAG_SIMPLE;
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

// Helper function to validate Rev C DAC file format
static int validate_rev_c_file_format(const char* file_path, int* line_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open Rev C DAC file '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  char line[2048]; // Buffer for line (32 numbers * ~5 chars + spaces + newline)
  int valid_lines = 0;
  int line_num = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Parse exactly 32 space-separated integers
    uint16_t values[32];
    int parsed = 0;
    char* token_start = trimmed;
    char* endptr;
    
    for (int i = 0; i < 32; i++) {
      // Skip leading whitespace
      while (*token_start == ' ' || *token_start == '\t') token_start++;
      
      if (*token_start == '\n' || *token_start == '\r' || *token_start == '\0') {
        break; // End of line
      }
      
      // Parse integer
      unsigned long val = strtoul(token_start, &endptr, 10);
      if (endptr == token_start) {
        break; // No valid number found
      }
      
      // Check range (0 to 65535)
      if (val > 65535) {
        fprintf(stderr, "Rev C DAC file line %d, value %d: %lu out of range (0-65535)\n", 
                line_num, i+1, val);
        fclose(file);
        return -1;
      }
      
      values[i] = (uint16_t)val;
      parsed++;
      token_start = endptr;
      
      // Skip whitespace after number
      while (*token_start == ' ' || *token_start == '\t') token_start++;
    }
    
    if (parsed != 32) {
      fprintf(stderr, "Rev C DAC file line %d: Expected 32 values, got %d\n", line_num, parsed);
      fclose(file);
      return -1;
    }
    
    // Check that we're at end of line
    while (*token_start == ' ' || *token_start == '\t') token_start++;
    if (*token_start != '\n' && *token_start != '\r' && *token_start != '\0') {
      fprintf(stderr, "Rev C DAC file line %d: Extra data after 32 values\n", line_num);
      fclose(file);
      return -1;
    }
    
    valid_lines++;
  }
  
  fclose(file);
  
  if (valid_lines == 0) {
    fprintf(stderr, "Rev C DAC file '%s' contains no valid data lines\n", file_path);
    return -1;
  }
  
  *line_count = valid_lines;
  return 0;
}

// Helper function to check that boards 0-3 are connected
static int check_boards_connected(command_context_t* ctx) {
  for (int board = 0; board < 4; board++) {
    uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);

    if (FIFO_PRESENT(adc_data_fifo_status) == 0) {
      fprintf(stderr, "Board %d: ADC data FIFO not present - board not connected\n", board);
      return -1;
    }
    if (FIFO_PRESENT(dac_cmd_fifo_status) == 0) {
      fprintf(stderr, "Board %d: DAC command FIFO not present - board not connected\n", board);
      return -1;
    }
    if (FIFO_PRESENT(adc_cmd_fifo_status) == 0) {
      fprintf(stderr, "Board %d: ADC command FIFO not present - board not connected\n", board);
      return -1;
    }
    
    if (*(ctx->verbose)) {
      printf("Board %d: All FIFOs present and connected\n", board);
    }
  }
  
  return 0;
}

// Structures and thread functions for Rev C compatibility streaming
typedef struct {
  command_context_t* ctx;
  const char* dac_file;
  int loops;
  int line_count;
  uint32_t delay_cycles;
  volatile bool* should_stop;
} rev_c_dac_stream_params_t;

typedef struct {
  command_context_t* ctx;
  const char* adc_output_file;
  uint64_t expected_samples;
  bool binary_mode;
  volatile bool* should_stop;
} rev_c_adc_stream_params_t;

// Thread function for Rev C DAC command streaming to all 4 boards
static void* rev_c_dac_stream_thread(void* arg) {
  rev_c_dac_stream_params_t* stream_data = (rev_c_dac_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* dac_file = stream_data->dac_file;
  int loops = stream_data->loops;
  int line_count = stream_data->line_count;
  uint32_t delay_cycles = stream_data->delay_cycles;
  volatile bool* should_stop = stream_data->should_stop;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C DAC Stream Thread: Starting streaming from file '%s' (%d lines, %d loops)\n", 
         dac_file, line_count, loops);
  
  FILE* file = fopen(dac_file, "r");
  if (file == NULL) {
    fprintf(stderr, "Rev C DAC Stream Thread: Failed to open file '%s': %s\n", dac_file, strerror(errno));
    return NULL;
  }
  
  int total_commands_sent = 0;
  
  for (int loop = 0; loop < loops && !(*should_stop); loop++) {
    rewind(file);
    char line[2048];
    int line_num = 0;
    
    while (fgets(line, sizeof(line), file) && !(*should_stop)) {
      // Skip empty lines and comments
      char* trimmed = line;
      while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
      if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
        continue;
      }
      
      line_num++;
      
      // Parse 32 values from the line (already validated in main function)
      uint16_t values[32];
      char* token_start = trimmed;
      char* endptr;
      
      for (int i = 0; i < 32; i++) {
        while (*token_start == ' ' || *token_start == '\t') token_start++;
        values[i] = (uint16_t)strtoul(token_start, &endptr, 10);
        token_start = endptr;
      }
      
      // Convert values to signed format and send DAC write commands to each board
      for (int board = 0; board < 4; board++) {
        // Wait for FIFO space
        bool fifo_space_available = false;
        int retries = 0;
        const int max_retries = 1000;
        
        while (!fifo_space_available && retries < max_retries && !(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C DAC Stream Thread: Board %d FIFO not present, stopping\n", board);
            goto cleanup;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = DAC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 5) { // Need space for 1 command + 4 data words
            fifo_space_available = true;
          } else {
            usleep(1000); // 1ms delay
            retries++;
          }
        }
        
        if (!fifo_space_available) {
          fprintf(stderr, "Rev C DAC Stream Thread: Board %d FIFO timeout, stopping\n", board);
          goto cleanup;
        }
        
        // Convert offset values to signed format for this board's 8 channels
        int16_t ch_vals[8];
        for (int ch = 0; ch < 8; ch++) {
          uint16_t offset_val = values[board * 8 + ch];
          ch_vals[ch] = offset_to_signed(offset_val);
        }
        
        // Determine cont flag: true for all commands except the final one at the last loop for each board
        bool is_final_command = (loop == loops - 1) && (line_num == line_count);
        bool cont_flag = !is_final_command;
        
        // Send DAC write command with trigger wait (trig=true, cont=cont_flag, ldac=false, value=0)
        dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, true, cont_flag, false, 0, verbose);
        total_commands_sent++;
        
        if (verbose && line_num <= 3) { // Only show first few lines to avoid spam
          printf("Rev C DAC Stream Thread: Board %d, Line %d, Loop %d, sent DAC write (channels %d-%d)\n", 
                 board, line_num, loop + 1, board * 8, board * 8 + 7);
        }
      }
      
      // Small delay between lines to prevent overwhelming the system
      usleep(100); // 100μs delay
    }
    
    if (verbose) {
      printf("Rev C DAC Stream Thread: Completed loop %d/%d\n", loop + 1, loops);
    }
  }
  
cleanup:
  fclose(file);
  
  if (*should_stop) {
    printf("Rev C DAC Stream Thread: Stopping stream (user requested), sent %d total commands\n", total_commands_sent);
  } else {
    printf("Rev C DAC Stream Thread: Stream completed, sent %d total commands (%d loops)\n", total_commands_sent, loops);
  }
  
  return NULL;
}

// Thread function for Rev C ADC command streaming to all 4 boards
static void* rev_c_adc_cmd_stream_thread(void* arg) {
  rev_c_dac_stream_params_t* stream_data = (rev_c_dac_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  int loops = stream_data->loops;
  int line_count = stream_data->line_count;
  uint32_t delay_cycles = stream_data->delay_cycles;
  volatile bool* should_stop = stream_data->should_stop;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C ADC Command Stream Thread: Starting (%d lines, %d loops)\n", line_count, loops);
  
  int total_commands_sent = 0;
  
  for (int loop = 0; loop < loops && !(*should_stop); loop++) {
    for (int line = 0; line < line_count && !(*should_stop); line++) {
      // For each line, send 3 ADC commands to each board
      for (int board = 0; board < 4; board++) {
        // Command 1: No-op with trigger wait (value = 1)
        bool fifo_space_available = false;
        int retries = 0;
        
        while (!fifo_space_available && retries < 1000 && !(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C ADC Command Stream Thread: Board %d FIFO not present, stopping\n", board);
            return NULL;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 3) { // Need space for 3 commands
            fifo_space_available = true;
          } else {
            usleep(1000);
            retries++;
          }
        }
        
        if (!fifo_space_available) {
          fprintf(stderr, "Rev C ADC Command Stream Thread: Board %d FIFO timeout, stopping\n", board);
          return NULL;
        }
        
        // Send 3 ADC commands per board per line
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, verbose); // Trigger wait (value=1)
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, verbose); // Delay
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, verbose); // ADC read with trigger wait (value=0)
        
        total_commands_sent += 3;
        
        if (verbose && line <= 2 && loop == 0) { // Show first few lines of first loop
          printf("Rev C ADC Command Stream Thread: Board %d, Line %d, sent 3 commands\n", board, line + 1);
        }
      }
      
      // Small delay between lines
      usleep(100); // 100μs delay
    }
    
    if (verbose) {
      printf("Rev C ADC Command Stream Thread: Completed loop %d/%d\n", loop + 1, loops);
    }
  }
  
  if (*should_stop) {
    printf("Rev C ADC Command Stream Thread: Stopping (user requested), sent %d total commands\n", total_commands_sent);
  } else {
    printf("Rev C ADC Command Stream Thread: Completed, sent %d total commands\n", total_commands_sent);
  }
  
  return NULL;
}

// Thread function for Rev C ADC data streaming from all 4 boards
static void* rev_c_adc_data_stream_thread(void* arg) {
  rev_c_adc_stream_params_t* stream_data = (rev_c_adc_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* adc_output_file = stream_data->adc_output_file;
  uint64_t expected_samples = stream_data->expected_samples;
  bool binary_mode = stream_data->binary_mode;
  volatile bool* should_stop = stream_data->should_stop;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C ADC Data Stream Thread: Starting to write %llu samples to file '%s' (%s format)\n", 
         expected_samples, adc_output_file, binary_mode ? "binary" : "ASCII");
  
  // Open output file
  FILE* file = fopen(adc_output_file, binary_mode ? "wb" : "w");
  if (file == NULL) {
    fprintf(stderr, "Rev C ADC Data Stream Thread: Failed to open output file '%s': %s\n", 
           adc_output_file, strerror(errno));
    return NULL;
  }
  
  uint64_t samples_written = 0;
  uint32_t read_buffer[4]; // Buffer for reading one sample from each board
  
  while (samples_written < expected_samples && !(*should_stop)) {
    // Read one sample from each board (4 boards = 32 channels total)
    bool all_boards_have_data = true;
    
    // Check that all boards have data available
    for (int board = 0; board < 4; board++) {
      uint32_t fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(fifo_status) == 0) {
        fprintf(stderr, "Rev C ADC Data Stream Thread: Board %d FIFO not present, stopping\n", board);
        all_boards_have_data = false;
        break;
      }
      
      if (FIFO_STS_WORD_COUNT(fifo_status) == 0) {
        all_boards_have_data = false;
        break;
      }
    }
    
    if (!all_boards_have_data) {
      usleep(1000); // Wait 1ms and try again
      continue;
    }
    
    // Read one sample from each board
    for (int board = 0; board < 4; board++) {
      read_buffer[board] = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
    }
    
    if (binary_mode) {
      // Binary mode: write raw 32-bit words directly
      size_t written = fwrite(read_buffer, sizeof(uint32_t), 4, file);
      if (written != 4) {
        fprintf(stderr, "Rev C ADC Data Stream Thread: Failed to write binary data: %s\n", strerror(errno));
        break;
      }
    } else {
      // ASCII mode: convert to signed values and write as space-separated text
      bool first_sample = true;
      for (int board = 0; board < 4; board++) {
        uint32_t word = read_buffer[board];
        
        // Extract two 16-bit samples from the 32-bit word
        uint16_t sample1_offset = (uint16_t)(word & 0xFFFF);        // Bits 15:0
        uint16_t sample2_offset = (uint16_t)((word >> 16) & 0xFFFF); // Bits 31:16
        
        // Convert to signed format
        int16_t sample1_signed = offset_to_signed(sample1_offset);
        int16_t sample2_signed = offset_to_signed(sample2_offset);
        
        // Write samples with spaces between them
        if (!first_sample) {
          fprintf(file, " ");
        }
        fprintf(file, "%d %d", sample1_signed, sample2_signed);
        first_sample = false;
      }
      fprintf(file, "\n"); // End the line with 32 samples
    }
    
    samples_written++;
    
    // Flush periodically
    if (samples_written % 100 == 0) {
      fflush(file);
      
      if (verbose && samples_written % 1000 == 0) {
        printf("Rev C ADC Data Stream Thread: Written %llu/%llu samples\n", samples_written, expected_samples);
      }
    }
  }
  
  fclose(file);
  
  if (*should_stop) {
    printf("Rev C ADC Data Stream Thread: Stopping (user requested), wrote %llu samples\n", samples_written);
  } else {
    printf("Rev C ADC Data Stream Thread: Completed, wrote %llu samples to file '%s'\n", 
           samples_written, adc_output_file);
  }
  
  return NULL;
}

// Rev C compatibility command implementation
int cmd_rev_c_compat(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse arguments
  const char* dac_file = args[0];
  
  char* endptr;
  int loops = (int)parse_value(args[1], &endptr);
  if (*endptr != '\0' || loops < 1) {
    fprintf(stderr, "Invalid loop count: '%s'. Must be a positive integer.\n", args[1]);
    return -1;
  }
  
  const char* adc_output_file = args[2];
  
  uint32_t delay_cycles = (uint32_t)parse_value(args[3], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid delay cycles: '%s'. Must be a non-negative integer.\n", args[3]);
    return -1;
  }
  
  // Check for binary mode flag
  bool binary_mode = has_flag(flags, flag_count, FLAG_BIN);
  
  printf("Starting Rev C compatibility mode:\n");
  printf("  Input DAC file: %s\n", dac_file);
  printf("  Loops: %d\n", loops);
  printf("  ADC output file: %s\n", adc_output_file);
  printf("  Delay cycles: %u\n", delay_cycles);
  printf("  Output format: %s\n", binary_mode ? "binary" : "ASCII");
  
  // Step 1: Validate file format
  printf("Step 1: Validating Rev C DAC file format...\n");
  int line_count;
  if (validate_rev_c_file_format(dac_file, &line_count) != 0) {
    return -1;
  }
  printf("  File validation passed: %d valid data lines\n", line_count);
  
  // Step 2: Check system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Step 3: Check boards 0-3 are connected
  printf("Step 2: Checking board connections (boards 0-3)...\n");
  if (check_boards_connected(ctx) != 0) {
    return -1;
  }
  printf("  Board connection check passed\n");
  
  // Calculate expected sample count: 3 ADC reads per board per line * loops
  uint64_t expected_samples = (uint64_t)line_count * loops * 3; // 3 ADC reads per board, reading from 4 boards
  printf("Step 3: Calculated %llu expected ADC samples (%d lines * %d loops * 3 reads/line)\n", 
         expected_samples, line_count, loops);
  
  // Clean and expand file paths
  char full_dac_path[1024], full_adc_path[1024];
  clean_and_expand_path(dac_file, full_dac_path, sizeof(full_dac_path));
  clean_and_expand_path(adc_output_file, full_adc_path, sizeof(full_adc_path));
  
  // Set file permissions for the output file
  set_file_permissions(full_adc_path, *(ctx->verbose));
  
  // Prepare streaming thread data structures
  static volatile bool dac_stream_stop = false;
  static volatile bool adc_cmd_stream_stop = false;
  static volatile bool adc_data_stream_stop = false;
  
  // Reset stop flags
  dac_stream_stop = false;
  adc_cmd_stream_stop = false;
  adc_data_stream_stop = false;
  
  // Prepare DAC streaming thread data
  rev_c_dac_stream_params_t dac_stream_data = {
    .ctx = ctx,
    .dac_file = full_dac_path,
    .loops = loops,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &dac_stream_stop
  };
  
  // Prepare ADC command streaming thread data (reuse dac stream structure)
  rev_c_dac_stream_params_t adc_cmd_stream_data = {
    .ctx = ctx,
    .dac_file = NULL, // Not used for ADC commands
    .loops = loops,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &adc_cmd_stream_stop
  };
  
  // Prepare ADC data streaming thread data
  rev_c_adc_stream_params_t adc_data_stream_data = {
    .ctx = ctx,
    .adc_output_file = full_adc_path,
    .expected_samples = expected_samples,
    .binary_mode = binary_mode,
    .should_stop = &adc_data_stream_stop
  };
  
  // Start ADC data streaming thread first
  printf("Step 4: Starting ADC data streaming thread...\n");
  pthread_t adc_data_thread;
  if (pthread_create(&adc_data_thread, NULL, rev_c_adc_data_stream_thread, &adc_data_stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC data streaming thread: %s\n", strerror(errno));
    return -1;
  }
  
  // Start ADC command streaming thread
  printf("Step 5: Starting ADC command streaming thread...\n");
  pthread_t adc_cmd_thread;
  if (pthread_create(&adc_cmd_thread, NULL, rev_c_adc_cmd_stream_thread, &adc_cmd_stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC command streaming thread: %s\n", strerror(errno));
    adc_data_stream_stop = true;
    pthread_join(adc_data_thread, NULL);
    return -1;
  }
  
  // Start DAC command streaming thread
  printf("Step 6: Starting DAC command streaming thread...\n");
  pthread_t dac_thread;
  if (pthread_create(&dac_thread, NULL, rev_c_dac_stream_thread, &dac_stream_data) != 0) {
    fprintf(stderr, "Failed to create DAC command streaming thread: %s\n", strerror(errno));
    adc_cmd_stream_stop = true;
    adc_data_stream_stop = true;
    pthread_join(adc_cmd_thread, NULL);
    pthread_join(adc_data_thread, NULL);
    return -1;
  }
  
  printf("Rev C compatibility mode: All streaming threads started successfully\n");
  printf("Streaming in progress... Use Ctrl+C to stop or wait for completion\n");
  
  // Wait for all threads to complete
  pthread_join(dac_thread, NULL);
  pthread_join(adc_cmd_thread, NULL);
  pthread_join(adc_data_thread, NULL);
  
  printf("Rev C compatibility mode completed successfully\n");
  return 0;
}

int cmd_zero_all_dacs(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Validate system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  printf("Zeroing all DAC channels on connected boards...\n");
  
  // Check which boards are connected
  bool connected_boards[8] = {false};
  int connected_count = 0;
  
  if (*(ctx->verbose)) {
    printf("Checking board connections...\n");
  }
  
  for (int board = 0; board < 8; board++) {
    // Check if DAC command stream is running for this board
    if (ctx->dac_cmd_stream_running[board]) {
      fprintf(stderr, "Cannot zero DAC channels on board %d: DAC command stream is currently running. Stop the stream first.\n", board);
      return -1;
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
    }
  }
  
  if (connected_count == 0) {
    printf("No DAC boards are connected. Nothing to zero.\n");
    return 0;
  }
  
  printf("Found %d connected DAC board(s)\n", connected_count);
  
  // Reset buffers (unless --no_reset flag is used)
  if (!skip_reset) {
    if (*(ctx->verbose)) {
      printf("Resetting DAC command and data buffers for all boards...\n");
    }
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose)); // Reset all boards + trigger
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(1000); // 1ms delay
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(1000); // 1ms delay
  } else {
    if (*(ctx->verbose)) {
      printf("Skipping buffer reset (--no_reset flag specified)\n");
    }
  }
  
  // Send cancel commands to all connected boards
  if (*(ctx->verbose)) {
    printf("Sending CANCEL commands to connected boards...\n");
  }
  for (int board = 0; board < 8; board++) {
    if (connected_boards[board]) {
      dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
    }
  }
  usleep(1000); // 1ms to let cancel commands complete
  
  // Zero all channels on all connected boards
  int16_t zero_values[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int channels_zeroed = 0;
  
  printf("Setting all DAC channels to 0...\n");
  for (int board = 0; board < 8; board++) {
    if (connected_boards[board]) {
      // Send DAC write command with all channels set to 0
      // Use trigger=false, cont=false, ldac=true, value=0 (immediate update)
      dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, zero_values, false, false, true, 0, *(ctx->verbose));
      channels_zeroed += 8; // 8 channels per board
      
      if (*(ctx->verbose)) {
        printf("  Board %d: All 8 channels set to 0\n", board);
      } else {
        printf("  Board %d: Zeroed\n", board);
      }
    }
  }
  
  // Brief pause to let the DAC commands complete
  usleep(1000); // 1ms delay
  
  printf("Successfully zeroed %d DAC channels on %d connected board(s)\n", 
         channels_zeroed, connected_count);
  
  return 0;
}
