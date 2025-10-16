#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
#include <glob.h>
#include <time.h>
#include "experiment_commands.h"
#include "command_helper.h"
#include "adc_commands.h"
#include "dac_commands.h"
#include "trigger_commands.h"
#include "system_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
#include "dac_ctrl.h"
#include "map_memory.h"
#include "adc_ctrl.h"
#include "map_memory.h"
#include "trigger_ctrl.h"

// Forward declarations for helper functions
static int validate_system_running(command_context_t* ctx);
static int count_trigger_lines_in_file(const char* file_path);
static uint64_t calculate_expected_samples(const char* file_path, int repeat_count, bool verbose);

// Structure for trigger monitoring thread
typedef struct {
  struct sys_sts_t* sys_sts;
  uint32_t expected_total_triggers;
  volatile bool* should_stop;
  bool verbose;
} trigger_monitor_params_t;

// Thread function for trigger monitoring
static void* trigger_monitor_thread(void* arg);

// Global trigger monitor control
static volatile bool g_trigger_monitor_should_stop = false;
static pthread_t g_trigger_monitor_tid;
static bool g_trigger_monitor_active = false;

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
static uint64_t calculate_expected_samples(const char* file_path, int repeat_count, bool verbose) {
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
  
  uint64_t total_samples = samples_per_loop * (repeat_count + 1);
  if (verbose) {
    printf("Calculated %llu samples per loop, %llu total samples (%d repeats + 1)\n", 
           samples_per_loop, total_samples, repeat_count);
  }
  
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
    safe_buffer_reset(ctx, *(ctx->verbose));
    __sync_synchronize(); // Memory barrier
    usleep(10000); // 10ms delay
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
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));
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
  int16_t adc_reading_raw = offset_to_signed(adc_word & 0xFFFF);
  
  // Apply ADC bias correction if available
  int ch = atoi(args[0]); // Get the global channel number (0-63)
  double adc_reading_corrected = (double)adc_reading_raw;
  if (ctx->adc_bias_valid[ch]) {
    adc_reading_corrected -= ctx->adc_bias[ch];
  }
  // Simple rounding without math library: add 0.5 and truncate
  int16_t adc_reading = (int16_t)(adc_reading_corrected + (adc_reading_corrected >= 0 ? 0.5 : -0.5));
  
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
  // Parse arguments - either a channel number (0-63) or "all"
  bool calibrate_all = (strcmp(args[0], "all") == 0);
  int start_ch = 0, end_ch = 0;
  bool connected_boards[8] = {false}; // Track which boards are connected
  
  if (arg_count != 1) {
    fprintf(stderr, "Usage: channel_cal <channel|all> [--no_reset] (channel 0-63, board=ch/8, ch=ch%%8)\n");
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
    safe_buffer_reset(ctx, false);
    usleep(10000); // 10ms
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
  const int average_count = 10;
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
    
    if (*(ctx->verbose)) {
      printf("\n  Starting calibration for channel %d (board %d, channel %d)\n", ch, board, channel);
    }
    
    // Get current DAC calibration value and store it
    dac_cmd_get_cal(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, *(ctx->verbose));
    
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
      if (*(ctx->verbose)) {
        printf("FAILED - DAC calibration data timeout (no data in FIFO after %d tries)\n", tries);
      } else {
        printf("-F- |\n");
      }
      
      // Check hardware status when calibration fails - if system is halted, abort calibration
      printf("Reading hardware status register...\n");
      uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
      if (HW_STS_STATE(hw_status) == S_HALTED) {
        printf("Hardware status shows system is HALTED. Aborting channel calibration.\n");
        print_hw_status(hw_status, *(ctx->verbose));
        return -1;
      }
      
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
        
        if (*(ctx->verbose)) {
          printf("    Testing DAC value %d (%d/%d), averaging %d samples...\n", dac_val, i+1, num_dac_values, average_count);
        }
        
        // Perform multiple reads and average them
        double sum_adc = 0.0;
        
        for (int avg = 0; avg < average_count; avg++) {
          // Write DAC value
          dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, dac_val, *(ctx->verbose));
          usleep(delay_ms * 1000); // Wait fixed delay
          
          // Read ADC value
          adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));
          
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
            if (*(ctx->verbose)) {
              printf("      ADC data timeout (no data in FIFO after %d tries)\n", adc_tries);
            }
            calibration_failed = true;
            break;
          }
          
          uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
          int16_t adc_reading = offset_to_signed(adc_word & 0xFFFF);
          double adc_value = (double)adc_reading;
          
          // Subtract ADC bias if available
          if (ctx->adc_bias_valid[ch]) {
            adc_value -= ctx->adc_bias[ch];
          }
          
          if (*(ctx->verbose) && avg < 3) {  // Only show first few readings to avoid spam
            printf("      Sample %d: ADC raw=0x%08X, signed=%d, bias_corrected=%.1f\n", 
                   avg+1, adc_word, adc_reading, adc_value);
          }
          
          sum_adc += adc_value;
        }
        
        if (calibration_failed) break;
        
        avg_adc_vals[i] = sum_adc / average_count;
        
        if (*(ctx->verbose)) {
          printf("    DAC=%d -> ADC_avg=%.2f\n", dac_val, avg_adc_vals[i]);
        }
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
      
      // Convert offset and slope to amps (range -5.1 to 5.1 for ±32767)
      double offset_amps = intercept * 5.1 / 32767.0;
      
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
    dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, *(ctx->verbose));
    usleep(1000); // 1ms to let DAC settle
    
    // Print spaces for skipped iterations to maintain column alignment
    for (int i = completed_iterations; i < calibration_iterations; i++) {
      if (*(ctx->verbose)) printf("  -- Skipped iteration number %d", i + 1);
      else printf("----------------- | "); // 18 spaces to match the format above
    }
    
    // Check hardware status when calibration fails - if system is halted, abort calibration
    if (calibration_failed) {
      printf("Reading hardware status register...\n");
      uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
      if (HW_STS_STATE(hw_status) == S_HALTED) {
        printf("Hardware status shows system is HALTED. Aborting channel calibration.\n");
        print_hw_status(hw_status, *(ctx->verbose));
        return -1;
      }
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

// Thread function for trigger monitoring
static void* trigger_monitor_thread(void* arg) {
  trigger_monitor_params_t* params = (trigger_monitor_params_t*)arg;
  time_t last_display = time(NULL);
  uint32_t last_trigger_count = 0;  // Since we reset count after sync, start from 0
  bool completed_message_shown = false;
  
  if (params->verbose) {
    printf("Trigger monitor thread started. Expected: %u triggers\n",
           params->expected_total_triggers);
  }
  
  while (!*(params->should_stop)) {
    usleep(500000); // 500ms polling interval
    
    uint32_t current_trigger_count = sys_sts_get_trig_counter(params->sys_sts, false);
    // Since we reset the count after sync_ch, current_trigger_count is the actual triggers received
    
    // Check if 3 seconds have passed since last display
    time_t current_time = time(NULL);
    if (current_time - last_display >= 3) {
      printf("Trigger count: %u/%u\n", 
             current_trigger_count, params->expected_total_triggers);
      fflush(stdout);
      last_display = current_time;
    }
    
    // Check if we've reached the expected trigger count
    if (current_trigger_count >= params->expected_total_triggers) {
      if (!completed_message_shown) {
        printf("\nExpected trigger count reached: %u/%u\n", 
               current_trigger_count, params->expected_total_triggers);
        fflush(stdout);
        completed_message_shown = true;
        
        // Auto-stop monitoring after reaching expected count
        printf("Trigger monitoring auto-stopping after reaching expected count.\n");
        break;
      }
    }
    
    last_trigger_count = current_trigger_count;
  }
  
  if (params->verbose) {
    printf("Trigger monitor thread stopping\n");
  }
  
  pthread_exit(NULL);
}

// Waveform test command implementation
int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Starting interactive waveform test...\n");

  // Make sure the system IS running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state != S_RUNNING) {
    printf("Error: Hardware manager is not running (state: %u). Use 'on' command first.\n", state);
    return -1;
  }
  
  // Check if --no_reset and --no_cal flags are present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  bool skip_cal = has_flag(flags, flag_count, FLAG_NO_CAL);
  
  if (*(ctx->verbose)) {
    printf("Waveform test flags: skip_reset=%s, skip_cal=%s (flag_count=%d)\n", 
           skip_reset ? "true" : "false", skip_cal ? "true" : "false", flag_count);
  }
  
  // Step 1: Reset all buffers (unless --no_reset flag is used)
  if (skip_reset) {
    printf("Step 1: Skipping buffer reset (--no_reset flag specified)\n");
  } else {
    printf("Step 1: Resetting all buffers\n");
    safe_buffer_reset(ctx, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
  // Step 2: Check which boards are connected
  bool connected_boards[8] = {false}; // Track which boards are connected
  int connected_count = 0;
  printf("Step 2: Checking connected boards...\n");
  
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
    fprintf(stderr, "Error: No boards are connected. Cannot run waveform test.\n");
    return -1;
  }
  
  printf("Found %d connected board(s)\n", connected_count);
  
  // Step 3: Prompt for DAC and ADC command files for each connected board
  char resolved_dac_files[8][1024] = {0};  // Resolved DAC file paths
  char resolved_adc_files[8][1024] = {0};  // Resolved ADC file paths
  
  char previous_dac_file[1024] = "";
  char previous_adc_file[1024] = "";
  
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    printf("\nBoard %d configuration:\n", board);
    
    // Prompt for DAC command file using helper function
    char dac_prompt[128];
    snprintf(dac_prompt, sizeof(dac_prompt), "Enter DAC command file for board %d", board);
    
    if (prompt_file_selection(dac_prompt, 
                             strlen(previous_dac_file) > 0 ? previous_dac_file : NULL,
                             resolved_dac_files[board], 
                             sizeof(resolved_dac_files[board])) != 0) {
      fprintf(stderr, "Failed to get DAC file for board %d\n", board);
      return -1;
    }
    strcpy(previous_dac_file, resolved_dac_files[board]);
    
    // Prompt for ADC command file using helper function
    char adc_prompt[128];
    snprintf(adc_prompt, sizeof(adc_prompt), "Enter ADC command file for board %d", board);
    
    if (prompt_file_selection(adc_prompt, 
                             strlen(previous_adc_file) > 0 ? previous_adc_file : NULL,
                             resolved_adc_files[board], 
                             sizeof(resolved_adc_files[board])) != 0) {
      fprintf(stderr, "Failed to get ADC file for board %d\n", board);
      return -1;
    }
    strcpy(previous_adc_file, resolved_adc_files[board]);
  }
  
  // Step 4: Prompt for DAC and ADC repeat counts for each connected board
  int dac_loops[8] = {0};  // DAC loop counts for each board  
  int adc_repeats[8] = {0};  // ADC repeat counts for each board
  
  int previous_dac_loops = 0;
  int previous_adc_repeats = 0;
  
  printf("\nLoop count configuration:\n");
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    // Prompt for DAC loops with default handling
    char input_buffer[64];
    printf("Enter DAC loop count for board %d", board);
    if (previous_dac_loops > 0) {
      printf(" (default: %d)", previous_dac_loops);
    }
    printf(": ");
    fflush(stdout);
    
    if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
      fprintf(stderr, "Failed to read DAC loop count input.\n");
      return -1;
    }
    
    // Remove newline
    size_t len = strlen(input_buffer);
    if (len > 0 && input_buffer[len - 1] == '\n') {
      input_buffer[len - 1] = '\0';
    }
    
    // Check for default (empty input or ".")
    if ((strlen(input_buffer) == 0 || strcmp(input_buffer, ".") == 0) && previous_dac_loops > 0) {
      dac_loops[board] = previous_dac_loops;
    } else {
      dac_loops[board] = atoi(input_buffer);
      if (dac_loops[board] < 1) {
        fprintf(stderr, "Invalid DAC loop count for board %d. Must be >= 1.\n", board);
        return -1;
      }
    }
    previous_dac_loops = dac_loops[board];
    
    // Prompt for ADC repeats with default handling
    printf("Enter ADC repeat count for board %d", board);
    int default_adc_repeats = (previous_adc_repeats >= 0) ? previous_adc_repeats : 0;
    printf(" (default: %d)", default_adc_repeats);
    printf(": ");
    fflush(stdout);
    
    if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
      fprintf(stderr, "Failed to read ADC repeat count input.\n");
      return -1;
    }
    
    // Remove newline
    len = strlen(input_buffer);
    if (len > 0 && input_buffer[len - 1] == '\n') {
      input_buffer[len - 1] = '\0';
    }
    
    // Check for default (empty input or ".")
    if (strlen(input_buffer) == 0 || strcmp(input_buffer, ".") == 0) {
      adc_repeats[board] = default_adc_repeats;
    } else {
      adc_repeats[board] = atoi(input_buffer);
      if (adc_repeats[board] < 0) {
        fprintf(stderr, "Invalid ADC repeat count for board %d. Must be >= 0.\n", board);
        return -1;
      }
    }
    previous_adc_repeats = adc_repeats[board];
  }
  
  // Step 5: Prompt for base output file name
  char base_output_file[1024];
  char input_buffer[1024];
  printf("Enter base output file path: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read output file path.\n");
    return -1;
  }
  
  // Remove newline and copy to base_output_file
  size_t len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  strncpy(base_output_file, input_buffer, sizeof(base_output_file) - 1);
  base_output_file[sizeof(base_output_file) - 1] = '\0';
  
  if (strlen(base_output_file) == 0) {
    fprintf(stderr, "Output file path cannot be empty.\n");
    return -1;
  }
  
  printf("\nOutput files will be created with the following naming:\n");
  printf("  ADC data: <base>_bd_<N>.<ext> (one per connected board)\n");
  printf("  Trigger data: <base>_trig.<ext>\n");
  printf("  Extensions: .csv (ASCII) or .dat (binary)\n");
  
  // Step 6: Prompt for SPI frequency and trigger lockout time
  double spi_freq_mhz;
  printf("Enter SPI clock frequency in MHz: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read SPI frequency.\n");
    return -1;
  }
  
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  spi_freq_mhz = atof(input_buffer);
  if (spi_freq_mhz <= 0.0) {
    fprintf(stderr, "Invalid SPI frequency. Must be > 0 MHz.\n");
    return -1;
  }
  
  double lockout_ms;
  printf("Enter trigger lockout time (milliseconds): ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read trigger lockout time.\n");
    return -1;
  }
  
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  lockout_ms = atof(input_buffer);
  if (lockout_ms <= 0) {
    fprintf(stderr, "Invalid trigger lockout time. Must be > 0 milliseconds.\n");
    return -1;
  }
  
  // Calculate lockout cycles from milliseconds and SPI frequency
  uint32_t lockout_time = (uint32_t)(lockout_ms * spi_freq_mhz * 1000.0);
  printf("Calculated lockout: %u cycles (%.3f ms at %.3f MHz)\n", 
         lockout_time, lockout_ms, spi_freq_mhz);
  
  // Step 7: Calculate expected samples and triggers for each connected board
  uint64_t sample_counts[8] = {0};  // Expected samples per board
  uint32_t board_triggers[8] = {0};  // Triggers per board
  uint32_t total_expected_triggers = 0;
  
  if (*(ctx->verbose)) {
    printf("\nCalculating expected data counts:\n");
  }
  
  // First pass: calculate triggers for each board and validate consistency
  int reference_trigger_count = -1;
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    // Count trigger lines in DAC file
    int dac_trigger_count = count_trigger_lines_in_file(resolved_dac_files[board]);
    if (dac_trigger_count < 0) {
      return -1;
    }
    
    // Count trigger lines in ADC file  
    int adc_trigger_count = count_trigger_lines_in_file(resolved_adc_files[board]);
    if (adc_trigger_count < 0) {
      return -1;
    }
    
    // Calculate total triggers for this board
    uint32_t dac_total_triggers = dac_trigger_count * dac_loops[board];
    uint32_t adc_total_triggers = adc_trigger_count * (adc_repeats[board] + 1);
    
    // Validate that DAC and ADC have same trigger count for this board
    if (dac_total_triggers != adc_total_triggers) {
      fprintf(stderr, "Error: Board %d DAC triggers (%u) != ADC triggers (%u)\n", 
              board, dac_total_triggers, adc_total_triggers);
      fprintf(stderr, "  DAC: %d triggers/file × %d loops = %u\n", 
              dac_trigger_count, dac_loops[board], dac_total_triggers);
      fprintf(stderr, "  ADC: %d triggers/file × %d loops = %u\n", 
              adc_trigger_count, adc_repeats[board] + 1, adc_total_triggers);
      return -1;
    }
    
    board_triggers[board] = dac_total_triggers;
    
    // Validate that all boards have the same trigger count
    if (reference_trigger_count == -1) {
      reference_trigger_count = dac_total_triggers;
    } else if (reference_trigger_count != dac_total_triggers) {
      fprintf(stderr, "Error: All boards must have the same total trigger count\n");
      fprintf(stderr, "  Reference (previous boards): %d triggers\n", reference_trigger_count);
      fprintf(stderr, "  Board %d: %u triggers\n", board, dac_total_triggers);
      return -1;
    }
  }
  
  // Second pass: calculate sample counts now that triggers are validated
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    // Calculate expected number of samples from ADC command file using board-specific ADC loops
    sample_counts[board] = calculate_expected_samples(resolved_adc_files[board], adc_repeats[board], *(ctx->verbose));
    if (sample_counts[board] == 0) {
      fprintf(stderr, "Failed to calculate expected sample count from ADC command file for board %d\n", board);
      return -1;
    }
    
    total_expected_triggers = board_triggers[board]; // All boards have same count
    
    if (*(ctx->verbose)) {
      printf("Board %d: DAC loops=%d, ADC loops=%d, triggers=%u, ADC samples=%llu\n", 
             board, dac_loops[board], adc_repeats[board], board_triggers[board], sample_counts[board]);
    }
  }
  
  if (*(ctx->verbose)) {
    printf("Total expected external triggers (consistent across all boards): %u\n", total_expected_triggers);
  }
  
  // Step 8: Run calibration for all boards unless --no_cal flag is set
  if (!skip_cal) {
    printf("\nRunning channel calibration for all connected boards...\n");
    
    // Use --all flag to calibrate all channels on all connected boards
    command_flag_t all_flag = FLAG_ALL;
    int cal_result = cmd_channel_cal(NULL, 0, &all_flag, 1, ctx);
    
    if (cal_result != 0) {
      printf("\nSome channels may have calibration issues (see output above).\n");
      printf("Do you want to continue with the waveform test anyway? (y/n): ");
      fflush(stdout);
      
      char response[16];
      if (fgets(response, sizeof(response), stdin) == NULL) {
        fprintf(stderr, "Failed to read user response\n");
        return -1;
      }
      
      if (response[0] != 'y' && response[0] != 'Y') {
        printf("Waveform test cancelled by user.\n");
        return 0;
      }
      printf("Continuing with waveform test...\n");
    } else {
      printf("All board calibrations completed successfully\n");
    }
  } else {
    printf("\nSkipping calibration (--no_cal flag set)\n");
  }
  
  // Step 9: Add buffer stoppers (NOOP commands waiting for 1 trigger) to all buffers BEFORE starting streams
  printf("Adding buffer stoppers before starting streams...\n");
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    if (*(ctx->verbose)) {
      printf("  Board %d: Adding DAC and ADC buffer stoppers\n", board);
    }
    
    // Add DAC NOOP stopper
    dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, true, false, false, 1, *(ctx->verbose)); // Wait for 1 trigger
    
    // Add ADC NOOP stopper  
    adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, *(ctx->verbose)); // Wait for 1 trigger
  }

  // Step 10a: Start command streaming for each connected board
  if (*(ctx->verbose)) {
    printf("\nStarting command streaming for %d connected boards...\n", connected_count);
  }
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    char board_str[16], dac_loops_str[16], adc_repeats_str[16];
    snprintf(board_str, sizeof(board_str), "%d", board);
    snprintf(dac_loops_str, sizeof(dac_loops_str), "%d", dac_loops[board]);
    snprintf(adc_repeats_str, sizeof(adc_repeats_str), "%d", adc_repeats[board]);
    
    // Start DAC command streaming with board-specific DAC loop count
    if (*(ctx->verbose)) {
      printf("  Board %d: Starting DAC command streaming from '%s' (%d loops)\n", 
             board, resolved_dac_files[board], dac_loops[board]);
    }
    const char* dac_args[] = {board_str, resolved_dac_files[board], dac_loops_str};
    if (cmd_stream_dac_commands_from_file(dac_args, 3, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start DAC command streaming for board %d\n", board);
      return -1;
    }
    
    // Start ADC command streaming with board-specific ADC repeat count (using hardware repeats)
    if (*(ctx->verbose)) {
      printf("  Board %d: Starting ADC command streaming from '%s' (%d repeats)\n", 
             board, resolved_adc_files[board], adc_repeats[board]);
    }
    const char* adc_args[] = {board_str, resolved_adc_files[board], adc_repeats_str};
    if (cmd_stream_adc_commands_from_file(adc_args, 3, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start ADC command streaming for board %d\n", board);
      return -1;
    }
  }
  
  // Step 10b: Start ADC data streaming for each connected board
  if (*(ctx->verbose)) {
    printf("Starting ADC data streaming for %d connected boards...\n", connected_count);
  }
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    // Create board-specific output file name
    char board_output_file[1024];
    char* ext_pos = strrchr(base_output_file, '.');
    if (ext_pos != NULL) {
      // Insert _bd_N before extension
      size_t base_len = ext_pos - base_output_file;
      snprintf(board_output_file, sizeof(board_output_file), "%.*s_bd_%d%s", 
               (int)base_len, base_output_file, board, ext_pos);
    } else {
      // No extension, just append _bd_N
      snprintf(board_output_file, sizeof(board_output_file), "%s_bd_%d", base_output_file, board);
    }
    
    char board_str[16], sample_count_str[32];
    snprintf(board_str, sizeof(board_str), "%d", board);
    snprintf(sample_count_str, sizeof(sample_count_str), "%llu", sample_counts[board]);
    
    if (*(ctx->verbose)) {
      printf("  Board %d: Starting ADC data streaming to '%s' (%llu samples)\n", 
             board, board_output_file, sample_counts[board]);
    }
    const char* adc_data_args[] = {board_str, sample_count_str, board_output_file};
    if (cmd_stream_adc_data_to_file(adc_data_args, 3, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start ADC data streaming for board %d\n", board);
      return -1;
    }
  }
  
  // Step 11: Start trigger data streaming if we expect triggers
  if (total_expected_triggers > 0) {
    // Create trigger output file name
    char trigger_output_file[1024];
    char* ext_pos = strrchr(base_output_file, '.');
    if (ext_pos != NULL) {
      // Insert _trig before extension
      size_t base_len = ext_pos - base_output_file;
      snprintf(trigger_output_file, sizeof(trigger_output_file), "%.*s_trig%s", 
               (int)base_len, base_output_file, ext_pos);
    } else {
      // No extension, just append _trig
      snprintf(trigger_output_file, sizeof(trigger_output_file), "%s_trig", base_output_file);
    }
    
    char trigger_count_str[32];
    snprintf(trigger_count_str, sizeof(trigger_count_str), "%u", total_expected_triggers);
    
    if (*(ctx->verbose)) {
      printf("Starting trigger data streaming to '%s' (%u samples)\n", 
             trigger_output_file, total_expected_triggers);
    }
    const char* trig_args[] = {trigger_count_str, trigger_output_file};
    if (cmd_stream_trig_data_to_file(trig_args, 2, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start trigger data streaming\n");
      return -1;
    }
  }
  
  // Wait for command buffers to preload before sending sync trigger
  printf("Waiting for command buffers to preload (at least 10 words or stream completion)...\n");
  bool buffers_ready = false;
  int check_count = 0;
  const int max_checks = 500; // Max 5 seconds at 10ms per check
  
  while (!buffers_ready && check_count < max_checks) {
    buffers_ready = true;
    
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      // Check DAC command buffer
      if (ctx->dac_cmd_stream_running[board]) {
        uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        uint32_t dac_words = FIFO_STS_WORD_COUNT(dac_cmd_fifo_status);
        if (dac_words < 10) {
          buffers_ready = false;
          if (*(ctx->verbose)) {
            printf("  Board %d DAC buffer: %u words (waiting for 10+)\n", board, dac_words);
          }
        }
      }
      
      // Check ADC command buffer  
      if (ctx->adc_cmd_stream_running[board]) {
        uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        uint32_t adc_words = FIFO_STS_WORD_COUNT(adc_cmd_fifo_status);
        if (adc_words < 10) {
          buffers_ready = false;
          if (*(ctx->verbose)) {
            printf("  Board %d ADC buffer: %u words (waiting for 10+)\n", board, adc_words);
          }
        }
      }
    }
    
    if (!buffers_ready) {
      usleep(10000); // Wait 10ms
      check_count++;
    }
  }
  
  if (check_count >= max_checks) {
    printf("Warning: Timeout waiting for buffer preload!\n");
    printf("Current buffer status:\n");
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      if (ctx->dac_cmd_stream_running[board]) {
        uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        uint32_t dac_words = FIFO_STS_WORD_COUNT(dac_cmd_fifo_status);
        printf("  Board %d DAC command buffer: %u words\n", board, dac_words);
      }
      
      if (ctx->adc_cmd_stream_running[board]) {
        uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        uint32_t adc_words = FIFO_STS_WORD_COUNT(adc_cmd_fifo_status);
        printf("  Board %d ADC command buffer: %u words\n", board, adc_words);
      }
    }
    
    char input_buffer[1024];
    printf("Do you want to proceed anyway? (y/N): ");
    fflush(stdout);
    
    if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
      printf("Failed to read user input, aborting\n");
      return -1;
    }
    
    // Remove newline
    size_t len = strlen(input_buffer);
    if (len > 0 && input_buffer[len - 1] == '\n') {
      input_buffer[len - 1] = '\0';
    }
    
    if (strcasecmp(input_buffer, "y") != 0 && strcasecmp(input_buffer, "yes") != 0) {
      printf("Waveform test aborted by user\n");
      return -1;
    }
    
    printf("Proceeding with waveform test...\n");
  } else if (*(ctx->verbose)) {
    printf("Command buffers preloaded successfully\n");
  }
  
  // Send sync_ch trigger to start all streams (after all streams including trigger are set up)
  printf("Sending sync trigger to start waveform test...\n");
  trigger_cmd_sync_ch(ctx->trigger_ctrl, false, *(ctx->verbose));
  
  // Reset trigger count after sync_ch to start counting from 0
  if (*(ctx->verbose)) {
    printf("Resetting trigger count after sync\n");
  }
  trigger_cmd_reset_count(ctx->trigger_ctrl, *(ctx->verbose));
  
  // Set trigger lockout
  if (*(ctx->verbose)) {
    printf("Setting trigger lockout time to %u cycles\n", lockout_time);
  }
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_time, *(ctx->verbose));
  
  // Set up trigger expectations after sync trigger is sent and trigger streaming is ready
  if (total_expected_triggers > 0) {
    if (*(ctx->verbose)) {
      printf("Setting expected external triggers to %u with logging enabled\n", total_expected_triggers);
    }
    trigger_cmd_expect_ext(ctx->trigger_ctrl, total_expected_triggers, true, *(ctx->verbose));
  }
  
  if (*(ctx->verbose)) {
    printf("Waveform test setup completed. All streaming started successfully.\n");
  }
  
  // Stop any existing trigger monitor
  if (g_trigger_monitor_active) {
    g_trigger_monitor_should_stop = true;
    usleep(100000); // Give it 100ms to stop
    g_trigger_monitor_active = false;
  }
  
  // Start new trigger monitoring thread
  g_trigger_monitor_should_stop = false;
  
  static trigger_monitor_params_t monitor_params;
  monitor_params.sys_sts = ctx->sys_sts;
  monitor_params.expected_total_triggers = total_expected_triggers; // Just the external triggers
  monitor_params.should_stop = &g_trigger_monitor_should_stop;
  monitor_params.verbose = *(ctx->verbose);
  
  int thread_result = pthread_create(&g_trigger_monitor_tid, NULL, trigger_monitor_thread, &monitor_params);
  if (thread_result != 0) {
    fprintf(stderr, "Failed to create trigger monitoring thread: %d\n", thread_result);
    return -1;
  }
  
  // Detach the thread so it can clean up automatically
  pthread_detach(g_trigger_monitor_tid);
  g_trigger_monitor_active = true;
  
  printf("\nWaveform test started - streams running in background, trigger monitoring active.\n");
  
  if (*(ctx->verbose)) {
    printf("Trigger progress will be displayed every 3 seconds during execution.\n");
    
    printf("The following commands can manually control the streams:\n");
    
    // Print connected boards list once
    printf("Connected boards: ");
    bool first = true;
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      if (!first) printf(", ");
      printf("%d", board);
      first = false;
    }
    printf("\n");
    
    printf("  - 'stop_waveform' to stop all waveform test streaming and monitoring\n");
    printf("  - 'stop_dac_cmd_stream <board>' to stop DAC command streaming\n");
    printf("  - 'stop_adc_cmd_stream <board>' to stop ADC command streaming\n");
    printf("  - 'stop_adc_data_stream <board>' to stop ADC data streaming\n");
    if (total_expected_triggers > 0) {
      printf("  - 'stop_trig_data_stream' to stop trigger data streaming\n");
    }
    printf("  - 'stop_trigger_monitor' to stop trigger monitoring thread\n");
    printf("  - 'sts' to check current hardware status\n");
    printf("  - 'trig_count' to manually check trigger count\n");
    
    printf("Expected data collection:\n");
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      printf("  - Board %d: %llu ADC samples\n", board, sample_counts[board]);
    }
    if (total_expected_triggers > 0) {
      printf("  - Trigger data: %u samples\n", total_expected_triggers);
    }
    
    printf("Waveform test started successfully. Streams running in background.\n");
  }
  
  return 0;
}

// Stop trigger monitoring command
int cmd_stop_trigger_monitor(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)args; (void)arg_count; (void)flags; (void)flag_count; (void)ctx; // Suppress unused parameter warnings
  
  if (g_trigger_monitor_active) {
    printf("Stopping trigger monitor...\n");
    g_trigger_monitor_should_stop = true;
    usleep(100000); // Give it 100ms to stop
    g_trigger_monitor_active = false;
    printf("Trigger monitor stopped.\n");
  } else {
    printf("No trigger monitor is currently running.\n");
  }
  
  return 0;
}

// Stop waveform test command - stops all streaming and monitoring
int cmd_stop_waveform(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)args; (void)arg_count; (void)flags; (void)flag_count; // Suppress unused parameter warnings
  
  printf("Stopping waveform test - shutting down all streams and monitoring...\n");
  
  bool anything_stopped = false;
  
  // Stop trigger monitor if running
  if (g_trigger_monitor_active) {
    printf("  Stopping trigger monitor\n");
    g_trigger_monitor_should_stop = true;
    usleep(100000); // Give it 100ms to stop
    g_trigger_monitor_active = false;
    anything_stopped = true;
  }
  
  // Stop trigger data streaming if running
  if (ctx->trig_data_stream_running) {
    printf("  Stopping trigger data stream\n");
    ctx->trig_data_stream_stop = true;
    if (pthread_join(ctx->trig_data_stream_thread, NULL) != 0) {
      fprintf(stderr, "Warning: Failed to join trigger data streaming thread\n");
    }
    ctx->trig_data_stream_running = false;
    anything_stopped = true;
  }
  
  // Stop all board streaming (DAC command, ADC command, ADC data)
  for (int board = 0; board < 8; board++) {
    // Stop DAC command streaming
    if (ctx->dac_cmd_stream_running[board]) {
      printf("  Stopping DAC command stream for board %d\n", board);
      ctx->dac_cmd_stream_stop[board] = true;
      if (pthread_join(ctx->dac_cmd_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join DAC command streaming thread for board %d\n", board);
      }
      ctx->dac_cmd_stream_running[board] = false;
      anything_stopped = true;
    }
    
    // Stop ADC command streaming
    if (ctx->adc_cmd_stream_running[board]) {
      printf("  Stopping ADC command stream for board %d\n", board);
      ctx->adc_cmd_stream_stop[board] = true;
      if (pthread_join(ctx->adc_cmd_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join ADC command streaming thread for board %d\n", board);
      }
      ctx->adc_cmd_stream_running[board] = false;
      anything_stopped = true;
    }
    
    // Stop ADC data streaming
    if (ctx->adc_data_stream_running[board]) {
      printf("  Stopping ADC data stream for board %d\n", board);
      ctx->adc_data_stream_stop[board] = true;
      if (pthread_join(ctx->adc_data_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join ADC data streaming thread for board %d\n", board);
      }
      ctx->adc_data_stream_running[board] = false;
      anything_stopped = true;
    }
  }
  
  if (anything_stopped) {
    printf("Waveform test stopped - all streams and monitoring shut down.\n");
  } else {
    printf("No waveform test streams or monitoring were running.\n");
  }
  
  return 0;
}

// Fieldmap data collection thread structure
typedef struct {
  command_context_t* ctx;
  int start_channel;
  int end_channel;
  double amplitude;
  double delay_ms;
  uint32_t delay_cycles;
  double spi_freq_mhz;
  const char* log_file;
  bool connected_boards[8];
  bool verbose;
  volatile bool* should_stop;
} fieldmap_params_t;

// Thread function for fieldmap data collection
static void* fieldmap_thread(void* arg) {
  fieldmap_params_t* params = (fieldmap_params_t*)arg;
  command_context_t* ctx = params->ctx;
  int start_ch = params->start_channel;
  int end_ch = params->end_channel;
  double amplitude = params->amplitude;
  const char* log_file = params->log_file;
  bool* connected_boards = params->connected_boards;
  volatile bool* should_stop = params->should_stop;
  double spi_freq_mhz = params->spi_freq_mhz;
  bool verbose = params->verbose;
  
  // Open log file
  FILE* file = fopen(log_file, "w");
  if (file == NULL) {
    fprintf(stderr, "Fieldmap Thread: Failed to open log file '%s': %s\n", log_file, strerror(errno));
    return NULL;
  }
  
  // Write delay information and CSV header
  fprintf(file, "# ADC Delay: %.3f ms (%" PRIu32 " clock cycles at %.3f MHz SPI frequency)\n",
          params->delay_ms, params->delay_cycles, spi_freq_mhz);
  fprintf(file, "time_sec,channel,polarity");
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    for (int ch_offset = 0; ch_offset < 8; ch_offset++) {
      int ch = board * 8 + ch_offset;
      fprintf(file, ",ch%02d", ch);
    }
  }
  fprintf(file, "\n");
  fflush(file);
  
  int current_channel = start_ch;
  bool positive_polarity = true; // Start with positive polarity
  int total_samples_expected = (end_ch - start_ch + 1) * 2; // 2 polarities per channel
  int samples_collected = 0;
  
  printf("Fieldmap Thread: Starting data collection for %d samples\n", total_samples_expected);
  if (verbose) {
    printf("Fieldmap Thread [VERBOSE]: Channels %d-%d, verbose mode enabled\n", start_ch, end_ch);
    printf("Fieldmap Thread [VERBOSE]: Connected boards: ");
    for (int i = 0; i < 8; i++) {
      if (connected_boards[i]) printf("%d ", i);
    }
    printf("\n");
  }
  
  // Time-based verbose logging variables
  time_t last_verbose_time = time(NULL);
  time_t last_status_check_time = time(NULL);
  
  while (samples_collected < total_samples_expected && !(*should_stop)) {
    int current_board = current_channel / 8;
    time_t current_time = time(NULL);
    
    // Check if all connected boards have data available (4 words each) and trigger has 2 words
    bool all_data_ready = true;
    uint32_t trig_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, false);
    
    // Periodic system status check and verbose logging (once every 5 seconds)
    if ((current_time - last_status_check_time) >= 5) {
      // Check system status for halt conditions (always run)
      uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, false);
      uint32_t state = HW_STS_STATE(hw_status);
      uint32_t status_code = HW_STS_CODE(hw_status);
      
      if (state != S_RUNNING || status_code != STS_OK) {
        printf("Fieldmap Thread [ERROR]: System not running properly!\n");
        print_hw_status(hw_status, true);
        
        if (state == S_HALTED) {
          printf("Fieldmap Thread [ERROR]: System is HALTED - stopping fieldmap!\n");
          break; // Exit the measurement loop
        }
      }
      
      // Verbose logging (only when verbose enabled)
      if (verbose) {
        printf("Fieldmap Thread [VERBOSE]: Checking for sample %d/%d (ch%02d%c)\n", 
               samples_collected + 1, total_samples_expected, current_channel,
               positive_polarity ? '+' : '-');
        
        // Show status for all connected boards
        for (int board = 0; board < 8; board++) {
          if (!connected_boards[board]) continue;
          uint32_t adc_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          printf("Fieldmap Thread [VERBOSE]: Board %d ADC FIFO status=0x%08X (count=%u)\n",
                 board, adc_status, FIFO_STS_WORD_COUNT(adc_status));
        }
        printf("Fieldmap Thread [VERBOSE]: Trigger FIFO status=0x%08X (count=%u)\n",
               trig_status, FIFO_STS_WORD_COUNT(trig_status));
      }
      
      last_status_check_time = current_time;
    }
    
    // Check that all connected boards have 4 words available and trigger has 2 words
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      uint32_t adc_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      if (FIFO_STS_WORD_COUNT(adc_status) < 4) {
        all_data_ready = false;
        break;
      }
    }
    
    if (all_data_ready && FIFO_STS_WORD_COUNT(trig_status) >= 2) {
      if (verbose) {
        printf("Fieldmap Thread [VERBOSE]: Data available! Reading sample %d/%d (ch%02d%c)\n",
               samples_collected + 1, total_samples_expected, current_channel,
               positive_polarity ? '+' : '-');
      }
      
      // Read ADC data from all connected boards (4 words each)
      int16_t channel_data[64]; // Store data for all 64 channels (8 boards x 8 channels)
      bool channel_valid[64] = {false}; // Track which channels have valid data
      
      for (int board = 0; board < 8; board++) {
        if (!connected_boards[board]) continue;
        
        // Read 4 words from this board (ADC pair data)
        for (int word = 0; word < 4; word++) {
          uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
          
          // Each word contains 2 channels (lower 16 bits = even channel, upper 16 bits = odd channel)
          int ch_base = board * 8 + word * 2;
          int16_t ch_even_raw = offset_to_signed(adc_word & 0xFFFF);
          int16_t ch_odd_raw = offset_to_signed((adc_word >> 16) & 0xFFFF);
          
          // Apply bias correction for even channel
          int16_t ch_even = ch_even_raw;
          if (ch_base < 64 && ctx->adc_bias_valid[ch_base]) {
            double corrected = (double)ch_even_raw - ctx->adc_bias[ch_base];
            ch_even = (int16_t)(corrected + (corrected >= 0 ? 0.5 : -0.5));
          }
          
          // Apply bias correction for odd channel  
          int16_t ch_odd = ch_odd_raw;
          if (ch_base + 1 < 64 && ctx->adc_bias_valid[ch_base + 1]) {
            double corrected = (double)ch_odd_raw - ctx->adc_bias[ch_base + 1];
            ch_odd = (int16_t)(corrected + (corrected >= 0 ? 0.5 : -0.5));
          }
          
          if (ch_base < 64) {
            channel_data[ch_base] = ch_even;
            channel_valid[ch_base] = true;
          }
          if (ch_base + 1 < 64) {
            channel_data[ch_base + 1] = ch_odd;
            channel_valid[ch_base + 1] = true;
          }
        }
      }
      
      // Read trigger data (64-bit)
      uint64_t trigger_data = trigger_read(ctx->trigger_ctrl);
      double time_seconds = (double)trigger_data / (spi_freq_mhz * 1e6);
      
      if (verbose) {
        printf("Fieldmap Thread [VERBOSE]: Read data from all boards, trigger_data=0x%016llX (%.6f sec)\n",
               (unsigned long long)trigger_data, time_seconds);
      }
      
      // Write to CSV file - connected channels only
      fprintf(file, "%.4f,ch%02d,%c", time_seconds, current_channel, positive_polarity ? '+' : '-');
      for (int board = 0; board < 8; board++) {
        if (!connected_boards[board]) continue;
        for (int ch_offset = 0; ch_offset < 8; ch_offset++) {
          int ch = board * 8 + ch_offset;
          if (channel_valid[ch]) {
            double current_amps = (double)channel_data[ch] / 32767.0 * 5.1;
            fprintf(file, ",%.3f", current_amps);
          } else {
            fprintf(file, ",0.000");  // Default value for channels on connected boards
          }
        }
      }
      fprintf(file, "\n");
      fflush(file);
      
      // Find target channel data and max current from other channels
      double target_current = 0.0;
      if (channel_valid[current_channel]) {
        target_current = (double)channel_data[current_channel] / 32767.0 * 5.1;
      }
      
      double max_other_current = 0.0;
      int max_other_channel = -1;
      for (int ch = 0; ch < 64; ch++) {
        if (ch == current_channel || !channel_valid[ch]) continue;
        double current_amps = (double)channel_data[ch] / 32767.0 * 5.1;
        double abs_current = (current_amps < 0.0) ? -current_amps : current_amps;
        double abs_max = (max_other_current < 0.0) ? -max_other_current : max_other_current;
        if (abs_current > abs_max) {
          max_other_current = current_amps;
          max_other_channel = ch;
        }
      }
      
      // Print ADC values in real-time
      if (max_other_channel != -1) {
        printf("Fieldmap: ch%02d%c = %7.3f A | max_other: ch%02d = %7.3f A [%d/%d]\n", 
               current_channel, positive_polarity ? '+' : '-', target_current,
               max_other_channel, max_other_current,
               samples_collected + 1, total_samples_expected);
      } else {
        printf("Fieldmap: ch%02d%c = %7.3f A | [%d/%d]\n", 
               current_channel, positive_polarity ? '+' : '-', target_current,
               samples_collected + 1, total_samples_expected);
      }
      fflush(stdout);
      
      samples_collected++;
      
      // Advance to next measurement
      if (positive_polarity) {
        positive_polarity = false; // Switch to negative
      } else {
        positive_polarity = true;  // Switch back to positive
        current_channel++;         // Move to next channel
      }
    } else {
      // No data available, sleep briefly
      if (verbose && (current_time - last_verbose_time) >= 5) {
        printf("Fieldmap Thread [VERBOSE]: No data available (Trigger count=%u), waiting...\n",
               FIFO_STS_WORD_COUNT(trig_status));
        last_verbose_time = current_time;
      }
      usleep(1000); // 1ms
    }
  }
  
  if (verbose) {
    printf("Fieldmap Thread [VERBOSE]: Loop ended - samples_collected=%d, total_expected=%d, should_stop=%s\n",
           samples_collected, total_samples_expected, *should_stop ? "true" : "false");
  }
  
  fclose(file);
  
  if (*should_stop) {
    printf("Fieldmap Thread: Stopped by user after collecting %d samples\n", samples_collected);
  } else {
    printf("Fieldmap Thread: Collection completed, %d samples written to '%s'\n", 
           samples_collected, log_file);
  }
  
  return NULL;
}

// Fieldmap command implementation
int cmd_fieldmap(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Starting fieldmap data collection...\n");
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Verbose mode enabled\n");
  }

  // Check flags
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  bool skip_cal = has_flag(flags, flag_count, FLAG_NO_CAL);
  
  // Check that system is running
  if (validate_system_running(ctx) != 0) {
    return -1;
  }
  
  // Step 1: Prompt for start and end channels
  char input_buffer[256];
  int start_channel, end_channel;
  
  printf("Enter start channel (0-63): ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read start channel.\n");
    return -1;
  }
  start_channel = atoi(input_buffer);
  if (start_channel < 0 || start_channel > 63) {
    fprintf(stderr, "Invalid start channel. Must be 0-63.\n");
    return -1;
  }
  
  printf("Enter end channel (0-63): ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read end channel.\n");
    return -1;
  }
  end_channel = atoi(input_buffer);
  if (end_channel < 0 || end_channel > 63) {
    fprintf(stderr, "Invalid end channel. Must be 0-63.\n");
    return -1;
  }
  
  if (start_channel > end_channel) {
    fprintf(stderr, "Start channel must be <= end channel.\n");
    return -1;
  }
  
  // Step 2: Check which boards are connected and validate channels
  bool connected_boards[8] = {false};
  int connected_count = 0;
  printf("Checking connected boards...\n");
  
  for (int board = 0; board < 8; board++) {
    uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
    
    if (FIFO_PRESENT(adc_data_fifo_status) && 
        FIFO_PRESENT(dac_cmd_fifo_status) && 
        FIFO_PRESENT(adc_cmd_fifo_status)) {
      connected_boards[board] = true;
      connected_count++;
      printf("  Board %d: Connected\n", board);
    } else {
      printf("  Board %d: Not connected\n", board);
    }
  }
  
  if (connected_count == 0) {
    fprintf(stderr, "Error: No boards are connected.\n");
    return -1;
  }
  
  // Validate that all required boards for the channel range are connected
  for (int ch = start_channel; ch <= end_channel; ch++) {
    int board = ch / 8;
    if (!connected_boards[board]) {
      fprintf(stderr, "Error: Channel %d requires board %d, but board is not connected.\n", ch, board);
      return -1;
    }
  }
  
  // Step 3: Prompt for remaining parameters
  double amplitude;
  printf("Enter amplitude in amps (0.0 to 5.1): ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read amplitude.\n");
    return -1;
  }
  amplitude = atof(input_buffer);
  if (amplitude < 0.0 || amplitude > 5.1) {
    fprintf(stderr, "Invalid amplitude. Must be 0.0 to 5.1 amps.\n");
    return -1;
  }
  
  double delay_ms;
  printf("Enter ADC read delay in milliseconds: ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read delay.\n");
    return -1;
  }
  delay_ms = atof(input_buffer);
  if (delay_ms < 0.0) {
    fprintf(stderr, "Invalid delay. Must be >= 0.0 ms.\n");
    return -1;
  }
  
  double spi_freq_mhz;
  printf("Enter SPI clock frequency in MHz: ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read SPI frequency.\n");
    return -1;
  }
  spi_freq_mhz = atof(input_buffer);
  if (spi_freq_mhz <= 0.0) {
    fprintf(stderr, "Invalid SPI frequency. Must be > 0 MHz.\n");
    return -1;
  }
  
  double lockout_ms;
  printf("Enter trigger lockout time in milliseconds: ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read lockout time.\n");
    return -1;
  }
  lockout_ms = atof(input_buffer);
  if (lockout_ms <= 0.0) {
    fprintf(stderr, "Invalid lockout time. Must be > 0 milliseconds.\n");
    return -1;
  }
  
  // Calculate delay cycles from milliseconds and SPI frequency
  uint32_t delay_cycles = (uint32_t)(delay_ms * spi_freq_mhz * 1000.0);
  
  // Calculate lockout cycles from milliseconds and SPI frequency  
  uint32_t lockout_cycles = (uint32_t)(lockout_ms * spi_freq_mhz * 1000.0);
  
  char log_filename[1024];
  printf("Enter log file name: ");
  fflush(stdout);
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read log file name.\n");
    return -1;
  }
  
  // Remove newline and copy to log_filename
  size_t len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  strncpy(log_filename, input_buffer, sizeof(log_filename) - 1);
  log_filename[sizeof(log_filename) - 1] = '\0';
  
  if (strlen(log_filename) == 0) {
    fprintf(stderr, "Log file name cannot be empty.\n");
    return -1;
  }
  
  // Add .csv extension if not present
  char final_log_path[1024];
  strcpy(final_log_path, log_filename);
  char* dot = strrchr(final_log_path, '.');
  char* slash = strrchr(final_log_path, '/');
  
  if (dot == NULL || (slash != NULL && dot < slash)) {
    strcat(final_log_path, ".csv");
  }
  
  printf("\nFieldmap configuration:\n");
  printf("  Channels: %d to %d (%d channels)\n", start_channel, end_channel, end_channel - start_channel + 1);
  printf("  Amplitude: %.3f amps\n", amplitude);
  printf("  Delay: %.3f ms (%u clock cycles)\n", delay_ms, delay_cycles);
  printf("  Lockout: %.3f ms (%u clock cycles)\n", lockout_ms, lockout_cycles);
  printf("  Log file: %s\n", final_log_path);
  printf("  SPI frequency: %.3f MHz\n", spi_freq_mhz);
  
  // Step 4: Run calibration for all connected boards unless --no_cal flag is set
  if (!skip_cal) {
    printf("\nRunning channel calibration for all connected boards...\n");
    
    // Use --all flag to calibrate all channels on all connected boards
    command_flag_t all_flag = FLAG_ALL;
    int cal_result = cmd_channel_cal(NULL, 0, &all_flag, 1, ctx);
    if (cal_result != 0) {
      fprintf(stderr, "Calibration failed\n");
      return -1;
    }
  } else {
    printf("\nSkipping calibration (--no_cal flag set)\n");
  }
  
  // Step 5: Reset buffers unless --no_reset flag is set
  if (!skip_reset) {
    printf("Resetting buffers...\n");
    safe_buffer_reset(ctx, false);
    usleep(10000); // 10ms delay
  } else {
    printf("Skipping buffer reset (--no_reset flag set)\n");
  }
  
  // Step 6: Add buffer stoppers
  printf("Adding buffer stoppers...\n");
  for (int board = 0; board < 8; board++) {
    if (!connected_boards[board]) continue;
    
    dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, true, false, false, 1, *(ctx->verbose));
    adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, *(ctx->verbose));
  }
  
  // Calculate DAC values
  int16_t dac_positive = (int16_t)(32767.0 * (amplitude / 5.1));
  int16_t dac_negative = -dac_positive;
  
  printf("DAC values: +%d, %d (for %.3f amps)\n", dac_positive, dac_negative, amplitude);
  
  // Step 7: Queue DAC commands
  printf("Queueing DAC commands...\n");
  int total_dac_commands = 0;
  for (int ch = start_channel; ch <= end_channel; ch++) {
    int target_board = ch / 8;
    int target_channel = ch % 8;
    
    if (*(ctx->verbose)) {
      printf("Fieldmap [VERBOSE]: Queueing DAC commands for ch%02d (board %d, channel %d)\n", 
             ch, target_board, target_channel);
    }
    
    // Positive polarity commands for all boards
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      int16_t ch_vals[8] = {0};
      if (board == target_board) {
        ch_vals[target_channel] = dac_positive;
      }
      
      dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, true, false, true, 1, *(ctx->verbose));
      total_dac_commands++;
    }
    
    // Negative polarity commands for all boards
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      int16_t ch_vals[8] = {0};
      if (board == target_board) {
        ch_vals[target_channel] = dac_negative;
      }
      
      dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, true, false, true, 1, *(ctx->verbose));
      total_dac_commands++;
    }
  }
  
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Queued %d total DAC commands\n", total_dac_commands);
  }
  
  // Step 8: Queue ADC commands
  printf("Queueing ADC commands...\n");
  int total_adc_commands = 0;
  for (int ch = start_channel; ch <= end_channel; ch++) {
    int target_board = ch / 8;
    int target_channel = ch % 8;
    
    if (*(ctx->verbose)) {
      printf("Fieldmap [VERBOSE]: Queueing ADC commands for ch%02d (board %d, channel %d)\n", 
             ch, target_board, target_channel);
    }
    
    // Positive polarity measurement
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      // NOOP wait for trigger
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, true, 1, 0, *(ctx->verbose));
      // Delay then read for all connected boards
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, true, delay_cycles, 0, *(ctx->verbose));
      adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, 0, *(ctx->verbose));
      total_adc_commands += 3;
    }
    
    // Negative polarity measurement
    for (int board = 0; board < 8; board++) {
      if (!connected_boards[board]) continue;
      
      // NOOP wait for trigger
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, true, 1, 0, *(ctx->verbose));
      // Delay then read for all connected boards
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, true, delay_cycles, 0, *(ctx->verbose));
      adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, 0, *(ctx->verbose));
      total_adc_commands += 3;
    }
  }
  
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Queued %d total ADC commands\n", total_adc_commands);
  }
  
  // Step 9: Start data collection thread
  printf("Starting data collection thread...\n");
  
  fieldmap_params_t thread_params = {
    .ctx = ctx,
    .start_channel = start_channel,
    .end_channel = end_channel,
    .amplitude = amplitude,
    .delay_ms = delay_ms,
    .delay_cycles = delay_cycles,
    .spi_freq_mhz = spi_freq_mhz,
    .log_file = final_log_path,
    .verbose = *(ctx->verbose),
    .should_stop = &ctx->fieldmap_stop
  };
  
  // Copy connected boards array
  for (int i = 0; i < 8; i++) {
    thread_params.connected_boards[i] = connected_boards[i];
  }
  
  ctx->fieldmap_stop = false;
  ctx->fieldmap_running = true;
  
  if (pthread_create(&(ctx->fieldmap_thread), NULL, fieldmap_thread, &thread_params) != 0) {
    fprintf(stderr, "Failed to create fieldmap data collection thread\n");
    ctx->fieldmap_running = false;
    return -1;
  }
  
  // Step 10: Send sync trigger to start
  printf("Sending sync trigger to start fieldmap...\n");
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Sending sync trigger\n");
  }
  trigger_cmd_sync_ch(ctx->trigger_ctrl, false, *(ctx->verbose)); // Use verbose mode for logging
  
  // Reset trigger count after sync_ch to start counting from 0
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Resetting trigger count after sync\n");
  }
  trigger_cmd_reset_count(ctx->trigger_ctrl, *(ctx->verbose));
  
  // Step 11: Set up trigger system after sync
  int total_triggers = (end_channel - start_channel + 1) * 2; // 2 per channel
  printf("Setting up trigger system for %d triggers...\n", total_triggers);
  
  // Set lockout cycles (calculated earlier from user input)
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Setting lockout to %u cycles (%.3f ms at %.3f MHz)\n", lockout_cycles, lockout_ms, spi_freq_mhz);
  }
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_cycles, *(ctx->verbose));
  
  if (*(ctx->verbose)) {
    printf("Fieldmap [VERBOSE]: Expecting %d external triggers\n", total_triggers);
  }
  trigger_cmd_expect_ext(ctx->trigger_ctrl, total_triggers, true, *(ctx->verbose)); // Enable logging
  
  printf("\nFieldmap data collection started successfully!\n");
  printf("Data collection is running in the background.\n");
  printf("ADC values will be printed as they are read.\n");
  printf("Use 'stop_fieldmap' command to stop data collection.\n");
  return 0;
}

// Stop fieldmap command implementation
int cmd_stop_fieldmap(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (!ctx->fieldmap_running) {
    printf("No fieldmap data collection is currently running.\n");
    return 0;
  }
  
  printf("Stopping fieldmap data collection...\n");
  ctx->fieldmap_stop = true;
  
  // Wait for thread to finish
  if (pthread_join(ctx->fieldmap_thread, NULL) != 0) {
    fprintf(stderr, "Failed to join fieldmap thread.\n");
    return -1;
  }
  
  ctx->fieldmap_running = false;
  printf("Fieldmap data collection stopped.\n");
  return 0;
}

// Helper function to convert Amps to DAC units
// Maps -4.0A -> 0, 0A -> 32767, 4.0A -> 65535
static uint16_t amps_to_dac(float amps) {
  // Clamp to valid range
  if (amps < -4.0f) amps = -4.0f;
  if (amps > 4.0f) amps = 4.0f;
  
  // Convert: -4.0 to 4.0 maps to 0 to 65535
  // Formula: dac = ((amps + 4.0) / 8.0) * 65535
  float normalized = (amps + 4.0f) / 8.0f;  // 0.0 to 1.0
  uint16_t dac_value = (uint16_t)(normalized * 65535.0f + 0.5f);  // Round to nearest
  return dac_value;
}

// Helper function to validate Rev C DAC file format (Amps)
static int validate_rev_c_file_format_amps(const char* file_path, int* line_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open Rev C DAC file (Amps) '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  char line[2048]; // Buffer for line (32 numbers * ~10 chars + spaces + newline)
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
    
    // Parse exactly 32 space-separated floats
    float values[32];
    int parsed = 0;
    char* token_start = trimmed;
    char* endptr;
    
    for (int i = 0; i < 32; i++) {
      // Skip leading whitespace
      while (*token_start == ' ' || *token_start == '\t') token_start++;
      
      if (*token_start == '\n' || *token_start == '\r' || *token_start == '\0') {
        break; // End of line
      }
      
      // Parse float
      float val = strtof(token_start, &endptr);
      if (endptr == token_start) {
        break; // No valid number found
      }
      
      // Check range (-4.0 to 4.0)
      if (val < -4.0f || val > 4.0f) {
        fprintf(stderr, "Rev C DAC file (Amps) line %d, value %d: %.3f out of range (-4.0 to 4.0)\n", 
                line_num, i+1, val);
        fclose(file);
        return -1;
      }
      
      values[i] = val;
      parsed++;
      token_start = endptr;
      
      // Skip whitespace after number
      while (*token_start == ' ' || *token_start == '\t') token_start++;
    }
    
    if (parsed != 32) {
      fprintf(stderr, "Rev C DAC file (Amps) line %d: Expected 32 values, got %d\n", line_num, parsed);
      fclose(file);
      return -1;
    }
    
    // Check that we're at end of line
    while (*token_start == ' ' || *token_start == '\t') token_start++;
    if (*token_start != '\n' && *token_start != '\r' && *token_start != '\0') {
      fprintf(stderr, "Rev C DAC file (Amps) line %d: Extra data after 32 values\n", line_num);
      fclose(file);
      return -1;
    }
    
    valid_lines++;
  }
  
  fclose(file);
  
  if (valid_lines == 0) {
    fprintf(stderr, "Rev C DAC file (Amps) '%s' contains no valid data lines\n", file_path);
    return -1;
  }
  
  *line_count = valid_lines;
  return 0;
}

// Helper function to validate Rev C DAC file format (DAC units)
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
      
      // Check range (0 to 65535, which is 2^16 - 1)
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
  bool input_is_amps;  // true for Amps input, false for DAC units
} rev_c_dac_stream_params_t;

typedef struct {
  command_context_t* ctx;
  const char* adc_output_file;
  uint64_t expected_samples;
  bool binary_mode;
  volatile bool* should_stop;
} rev_c_adc_stream_params_t;

// New data structure for updated rev_c streaming with final zero trigger support
typedef struct {
  command_context_t* ctx;
  char* dac_file;
  int loops;
  int line_count;
  uint32_t delay_cycles;
  volatile bool* should_stop;
  bool input_is_amps;
  bool final_zero_trigger;
} rev_c_new_params_t;

// Thread function for Rev C DAC command streaming to all 4 boards (new implementation)
static void* rev_c_new_dac_stream_thread(void* arg) {
  rev_c_new_params_t* stream_data = (rev_c_new_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* dac_file = stream_data->dac_file;
  int loops = stream_data->loops;
  int line_count = stream_data->line_count;
  volatile bool* should_stop = stream_data->should_stop;
  bool input_is_amps = stream_data->input_is_amps;
  bool final_zero_trigger = stream_data->final_zero_trigger;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C DAC Stream Thread: Starting streaming from file '%s' (%d lines, %d loops, final_zero=%s)\n", 
         dac_file, line_count, loops, final_zero_trigger ? "yes" : "no");
  
  FILE* file = fopen(dac_file, "r");
  if (file == NULL) {
    fprintf(stderr, "Rev C DAC Stream Thread: Failed to open file '%s': %s\n", dac_file, strerror(errno));
    return NULL;
  }
  
  int total_commands_sent = 0;
  
  // Process all loops
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
      
      if (input_is_amps) {
        // Parse as floats and convert to DAC units
        for (int i = 0; i < 32; i++) {
          while (*token_start == ' ' || *token_start == '\t') token_start++;
          float amp_value = strtof(token_start, &endptr);
          values[i] = amps_to_dac(amp_value);
          token_start = endptr;
        }
      } else {
        // Parse as DAC units (integers)
        for (int i = 0; i < 32; i++) {
          while (*token_start == ' ' || *token_start == '\t') token_start++;
          values[i] = (uint16_t)strtoul(token_start, &endptr, 10);
          token_start = endptr;
        }
      }
      
      // Send DAC write commands to each of the 4 boards
      for (int board = 0; board < 4; board++) {
        // Wait for FIFO space
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C New DAC Stream Thread: Board %d FIFO not present, stopping\n", board);
            goto cleanup;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = DAC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 5) { // Need space for 1 command + 4 data words
            break; // Space available, proceed with command
          }
          
          usleep(1000); // 1ms delay before checking again
        }
        
        // Convert values to signed format for this board's 8 channels (ch 0-7, 8-15, 16-23, 24-31)
        int16_t ch_vals[8];
        for (int ch = 0; ch < 8; ch++) {
          uint16_t offset_val = values[board * 8 + ch];
          ch_vals[ch] = offset_to_signed(offset_val);
        }
        
        // Send DAC write command with trigger wait (trig=true, cont=true, ldac=true, 1 trigger)
        dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, true, true, true, 1, verbose);
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
  
  // Send final zero trigger if requested
  if (final_zero_trigger && !(*should_stop)) {
    printf("Rev C DAC Stream Thread: Sending final zero trigger...\n");
    
    // Create array of zeros in signed format (0.0 amps = 32767 offset = 0 signed)
    int16_t zero_vals[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    
    for (int board = 0; board < 4; board++) {
      // Wait for FIFO space for final zero trigger
      while (!(*should_stop)) {
        uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        
        if (FIFO_PRESENT(fifo_status) == 0) {
          fprintf(stderr, "Rev C New DAC Stream Thread: Board %d FIFO not present for final zero, stopping\n", board);
          goto cleanup;
        }
        
        uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
        uint32_t available_words = DAC_CMD_FIFO_WORDCOUNT - words_used;
        if (available_words >= 5) { // Need space for 1 command + 4 data words
          break; // Space available, proceed with command
        }
        
        usleep(1000); // 1ms delay before checking again
      }
      
      // Send DAC write command with trigger wait (trig=true, cont=true, ldac=true, 1 trigger)
      dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, zero_vals, true, true, true, 1, verbose);
      total_commands_sent++;
      
      if (verbose) {
        printf("Rev C DAC Stream Thread: Board %d, sent final zero DAC write\n", board);
      }
    }
  }
  
cleanup:
  fclose(file);
  
  if (*should_stop) {
    printf("Rev C DAC Stream Thread: Stopping stream (user requested), sent %d total commands\n", total_commands_sent);
  } else {
    printf("Rev C DAC Stream Thread: Stream completed, sent %d total commands (%d loops%s)\n", 
           total_commands_sent, loops, final_zero_trigger ? " + final zero" : "");
  }
  
  return NULL;
}

// Thread function for Rev C ADC command streaming to all 4 boards (new implementation)
static void* rev_c_new_adc_cmd_stream_thread(void* arg) {
  rev_c_new_params_t* stream_data = (rev_c_new_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  int loops = stream_data->loops;
  int line_count = stream_data->line_count;
  uint32_t delay_cycles = stream_data->delay_cycles;
  volatile bool* should_stop = stream_data->should_stop;
  bool final_zero_trigger = stream_data->final_zero_trigger;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C ADC Command Stream Thread: Starting (%d lines, %d loops, delay=%u cycles, final_zero=%s)\n", 
         line_count, loops, delay_cycles, final_zero_trigger ? "yes" : "no");
  
  int total_commands_sent = 0;
  
  // First, send set_ord commands to all boards (order: 01234567)
  printf("Rev C ADC Command Stream Thread: Sending set_ord commands to all boards...\n");
  uint8_t channel_order[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  for (int board = 0; board < 4; board++) {
    // Wait for FIFO space for set_ord command
    while (!(*should_stop)) {
      uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(fifo_status) == 0) {
        fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present for set_ord, stopping\n", board);
        return NULL;
      }
      
      uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
      uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
      if (available_words >= 1) { // Need space for 1 command
        break; // Space available, proceed with command
      }
      
      usleep(1000); // 1ms delay before checking again
    }
    
    adc_cmd_set_ord(ctx->adc_ctrl, (uint8_t)board, channel_order, verbose);
    total_commands_sent++;
    
    if (verbose) {
      printf("Rev C ADC Command Stream Thread: Board %d, sent set_ord command\n", board);
    }
  }
  
  // Process all loops
  for (int loop = 0; loop < loops && !(*should_stop); loop++) {
    for (int line_num = 1; line_num <= line_count && !(*should_stop); line_num++) {
      
      // For each line, send 3 ADC commands per board:
      // 1. noop with trigger wait (1 trigger)  
      // 2. noop with delay wait (delay_cycles)
      // 3. adc_read with no trigger wait (0 triggers)
      for (int board = 0; board < 4; board++) {
        
        // Wait for FIFO space for all 3 commands
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present, stopping\n", board);
            return NULL;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 3) { // Need space for 3 commands
            break; // Space available, proceed with commands
          }
          
          usleep(1000); // 1ms delay before checking again
        }
        
        // Command 1: NOOP with trigger wait for 1 trigger
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, verbose);
        total_commands_sent++;
        
        // Command 2: NOOP with delay wait for delay_cycles
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, 0, verbose);
        total_commands_sent++;
        
        // Command 3: ADC read with no trigger wait (0 triggers)
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, 0, 0, verbose);
        total_commands_sent++;
        
        if (verbose && line_num <= 3) { // Only show first few lines to avoid spam
          printf("Rev C ADC Command Stream Thread: Board %d, Line %d, Loop %d, sent 3 ADC commands\n", 
                 board, line_num, loop + 1);
        }
      }
      
      // Small delay between lines to prevent overwhelming the system
      usleep(100); // 100μs delay
    }
    
    if (verbose) {
      printf("Rev C ADC Command Stream Thread: Completed loop %d/%d\n", loop + 1, loops);
    }
  }
  
  // Send final zero ADC commands if requested
  if (final_zero_trigger && !(*should_stop)) {
    printf("Rev C ADC Command Stream Thread: Sending final zero ADC commands...\n");
    
    for (int board = 0; board < 4; board++) {
      // Wait for FIFO space for final zero trigger commands
      while (!(*should_stop)) {
        uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        
        if (FIFO_PRESENT(fifo_status) == 0) {
          fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present for final zero, stopping\n", board);
          return NULL;
        }
        
        uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
        uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
        if (available_words >= 3) { // Need space for 3 commands
          break; // Space available, proceed with commands
        }
        
        usleep(1000); // 1ms delay before checking again
      }
      
      // Command 1: NOOP with trigger wait for 1 trigger
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, verbose);
      total_commands_sent++;
      
      // Command 2: NOOP with delay wait for delay_cycles
      adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, 0, verbose);
      total_commands_sent++;
      
      // Command 3: ADC read with no trigger wait (0 triggers)
      adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, 0, 0, verbose);
      total_commands_sent++;
      
      if (verbose) {
        printf("Rev C ADC Command Stream Thread: Board %d, sent final zero ADC commands\n", board);
      }
    }
  }
  
  if (*should_stop) {
    printf("Rev C ADC Command Stream Thread: Stopping stream (user requested), sent %d total commands\n", total_commands_sent);
  } else {
    printf("Rev C ADC Command Stream Thread: Stream completed, sent %d total commands (%d loops%s)\n", 
           total_commands_sent, loops, final_zero_trigger ? " + final zero" : "");
  }
  
  return NULL;
}

// Thread function for Rev C DAC command streaming to all 4 boards
static void* rev_c_dac_stream_thread(void* arg) {
  rev_c_dac_stream_params_t* stream_data = (rev_c_dac_stream_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* dac_file = stream_data->dac_file;
  int loops = stream_data->loops;
  int line_count = stream_data->line_count;
  uint32_t delay_cycles = stream_data->delay_cycles;
  volatile bool* should_stop = stream_data->should_stop;
  bool input_is_amps = stream_data->input_is_amps;
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
      
      if (input_is_amps) {
        // Parse as floats and convert to DAC units
        for (int i = 0; i < 32; i++) {
          while (*token_start == ' ' || *token_start == '\t') token_start++;
          float amp_value = strtof(token_start, &endptr);
          values[i] = amps_to_dac(amp_value);
          token_start = endptr;
        }
      } else {
        // Parse as DAC units (integers)
        for (int i = 0; i < 32; i++) {
          while (*token_start == ' ' || *token_start == '\t') token_start++;
          values[i] = (uint16_t)strtoul(token_start, &endptr, 10);
          token_start = endptr;
        }
      }
      
      // Convert values to signed format and send DAC write commands to each board
      for (int board = 0; board < 4; board++) {
        // Wait for FIFO space
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C DAC Stream Thread: Board %d FIFO not present, stopping\n", board);
            goto cleanup;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = DAC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 5) { // Need space for 1 command + 4 data words
            break; // Space available, proceed with command
          }
          
          usleep(1000); // 1ms delay before checking again
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
        
        // Send DAC write command with trigger wait (trig=true, cont=cont_flag, ldac=true, value=0)
        dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, true, cont_flag, true, 0, verbose);
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
        // Wait for FIFO space for all 3 commands
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C ADC Command Stream Thread: Board %d FIFO not present, stopping\n", board);
            return NULL;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 3) { // Need space for 3 commands
            break; // Space available, proceed with commands
          }
          
          usleep(1000); // 1ms delay before checking again
        }
        
        // Send 3 ADC commands per board per line
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, verbose); // Trigger wait (value=1)
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, 0, verbose); // Delay
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, 0, verbose); // ADC read with trigger wait (value=0)
        
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
        int16_t sample1_raw = offset_to_signed(sample1_offset);
        int16_t sample2_raw = offset_to_signed(sample2_offset);
        
        // Apply bias correction if available (Rev C mode reads one word per board)
        // Each word contains 2 samples, so we get channels in pairs
        // Board 0: channels 0-1, Board 1: channels 8-9, etc.
        int ch1 = board * 8;     // First channel for this board 
        int ch2 = ch1 + 1;       // Second channel for this board
        
        int16_t sample1_signed = sample1_raw;
        if (ch1 < 64 && ctx->adc_bias_valid[ch1]) {
          double corrected = (double)sample1_raw - ctx->adc_bias[ch1];
          sample1_signed = (int16_t)(corrected + (corrected >= 0 ? 0.5 : -0.5));
        }
        
        int16_t sample2_signed = sample2_raw;
        if (ch2 < 64 && ctx->adc_bias_valid[ch2]) {
          double corrected = (double)sample2_raw - ctx->adc_bias[ch2];
          sample2_signed = (int16_t)(corrected + (corrected >= 0 ? 0.5 : -0.5));
        }
        
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
  printf("Starting Rev C compatibility mode...\n");

  // Step 1: Make sure the system IS running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state != S_RUNNING) {
    printf("Error: Hardware manager is not running (state: %u). Use 'on' command first.\n", state);
    return -1;
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  bool binary_mode = has_flag(flags, flag_count, FLAG_BIN);
  
  if (*(ctx->verbose)) {
    printf("Rev C compat flags: skip_reset=%s, binary=%s (flag_count=%d)\n", 
           skip_reset ? "true" : "false", binary_mode ? "true" : "false", flag_count);
  }
  
  // Step 2: Reset all buffers (unless --no_reset flag is used)
  if (skip_reset) {
    printf("Step 1: Skipping buffer reset (--no_reset flag specified)\n");
  } else {
    printf("Step 1: Resetting all buffers\n");
    safe_buffer_reset(ctx, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
  // Step 3: Check that boards 0-3 are connected
  printf("Step 2: Checking board connections (boards 0-3)...\n");
  bool connected_boards[4] = {false}; // Only check first 4 boards
  int connected_count = 0;
  
  for (int board = 0; board < 4; board++) {
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
  
  if (connected_count < 4) {
    fprintf(stderr, "Error: Rev C compatibility mode requires all 4 boards (0-3) to be connected. Found %d.\n", connected_count);
    return -1;
  }
  
  printf("All 4 boards (0-3) are connected\n");
  
  // Step 4: Prompt for DAC command file
  char resolved_dac_file[1024];
  if (prompt_file_selection("Enter DAC command file (32 space-separated values per line)", 
                           NULL, resolved_dac_file, sizeof(resolved_dac_file)) != 0) {
    fprintf(stderr, "Failed to get DAC file\n");
    return -1;
  }
  
  // Step 5: Prompt for input units type
  char input_buffer[64];
  bool input_is_amps = false;
  
  printf("\nInput units selection:\n");
  printf("  1) Unsigned DAC units (integers 0-65535)\n");
  printf("  2) Amps (floats -4.0 to 4.0)\n");
  printf("Enter choice (1 or 2): ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read input choice.\n");
    return -1;
  }
  
  // Remove newline
  size_t len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  int choice = atoi(input_buffer);
  if (choice == 1) {
    input_is_amps = false;
    printf("Selected: Unsigned DAC units (0-65535)\n");
  } else if (choice == 2) {
    input_is_amps = true;
    printf("Selected: Amps (-4.0 to 4.0)\n");
  } else {
    fprintf(stderr, "Invalid choice '%s'. Please enter 1 or 2.\n", input_buffer);
    return -1;
  }
  
  // Step 6: Validate file format
  printf("Step 3: Validating DAC file format...\n");
  int line_count;
  if (input_is_amps) {
    if (validate_rev_c_file_format_amps(resolved_dac_file, &line_count) != 0) {
      return -1;
    }
    printf("  Amps file validation passed: %d valid data lines\n", line_count);
  } else {
    if (validate_rev_c_file_format(resolved_dac_file, &line_count) != 0) {
      return -1;
    }
    printf("  DAC units file validation passed: %d valid data lines\n", line_count);
  }
  
  // Step 7: Prompt for number of loops
  printf("Enter number of loops: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read loop count.\n");
    return -1;
  }
  
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  int loops = atoi(input_buffer);
  if (loops < 1) {
    fprintf(stderr, "Invalid loop count. Must be >= 1.\n");
    return -1;
  }
  
  // Step 8: Prompt for SPI frequency and ADC sample delay
  double spi_freq_mhz;
  printf("Enter SPI clock frequency in MHz: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read SPI frequency.\n");
    return -1;
  }
  
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  spi_freq_mhz = atof(input_buffer);
  if (spi_freq_mhz <= 0.0) {
    fprintf(stderr, "Invalid SPI frequency. Must be > 0 MHz.\n");
    return -1;
  }
  
  double adc_delay_ms;
  printf("Enter ADC sample delay (milliseconds): ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read ADC delay.\n");
    return -1;
  }
  
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  adc_delay_ms = atof(input_buffer);
  if (adc_delay_ms < 0.0) {
    fprintf(stderr, "Invalid ADC delay. Must be >= 0 milliseconds.\n");
    return -1;
  }
  
  // Calculate delay cycles from milliseconds and SPI frequency
  uint32_t delay_cycles = (uint32_t)(adc_delay_ms * spi_freq_mhz * 1000.0);
  printf("Calculated ADC delay: %u cycles (%.3f ms at %.3f MHz)\n", 
         delay_cycles, adc_delay_ms, spi_freq_mhz);
  
  // Step 9: Prompt for final zero trigger
  bool final_zero_trigger = false;
  printf("Add final zero trigger? (y/n): ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read final zero trigger choice.\n");
    return -1;
  }
  
  if (input_buffer[0] == 'y' || input_buffer[0] == 'Y') {
    final_zero_trigger = true;
    printf("Final zero trigger enabled\n");
  } else {
    final_zero_trigger = false;
    printf("Final zero trigger disabled\n");
  }
  
  // Step 10: Prompt for base output file name
  char base_output_file[1024];
  printf("Enter base output file path: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read output file path.\n");
    return -1;
  }
  
  // Remove newline and copy to base_output_file
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  strncpy(base_output_file, input_buffer, sizeof(base_output_file) - 1);
  base_output_file[sizeof(base_output_file) - 1] = '\0';
  
  if (strlen(base_output_file) == 0) {
    fprintf(stderr, "Output file path cannot be empty.\n");
    return -1;
  }
  
  printf("\nOutput files will be created with the following naming:\n");
  printf("  ADC data: <base>_bd_<N>.<ext> (one per connected board)\n");
  printf("  Trigger data: <base>_trig.<ext>\n");
  printf("  Extensions: .csv (ASCII) or .dat (binary)\n");
  
  // Calculate expected sample count: 3 ADC reads per board per line * loops (plus final zero if enabled)
  uint64_t total_lines = (uint64_t)line_count * loops;
  if (final_zero_trigger) {
    total_lines++; // Add one more line for final zero
  }
  uint64_t expected_samples_per_board = total_lines * 3; // 3 ADC reads per line per board
  uint32_t expected_triggers = (uint32_t)total_lines; // 1 trigger per line
  
  printf("\nCalculated expected data counts:\n");
  printf("  Lines per loop: %d\n", line_count);
  printf("  Total loops: %d\n", loops);
  printf("  Final zero trigger: %s\n", final_zero_trigger ? "Yes" : "No");
  printf("  Total lines to process: %llu\n", total_lines);
  printf("  Expected triggers: %u\n", expected_triggers);
  printf("  Expected ADC samples per board: %llu\n", expected_samples_per_board);
  
  printf("\nStarting Rev C compatibility mode with:\n");
  printf("  Input DAC file: %s\n", resolved_dac_file);
  printf("  Input format: %s\n", input_is_amps ? "Amps (-4.0 to 4.0)" : "DAC units (0-65535)");
  printf("  Loops: %d\n", loops);
  printf("  ADC delay: %.3f ms (%u cycles)\n", adc_delay_ms, delay_cycles);
  printf("  Output format: %s\n", binary_mode ? "binary" : "ASCII");
  printf("  Final zero trigger: %s\n", final_zero_trigger ? "enabled" : "disabled");
  
  // Step 11: Add buffer stoppers before starting streams
  printf("Step 4: Adding buffer stoppers before starting streams...\n");
  for (int board = 0; board < 4; board++) {
    if (*(ctx->verbose)) {
      printf("  Board %d: Adding DAC and ADC buffer stoppers\n", board);
    }
    
    // Add DAC NOOP stopper
    dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, true, false, false, 1, *(ctx->verbose)); // Wait for 1 trigger
    
    // Add ADC NOOP stopper  
    adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, *(ctx->verbose)); // Wait for 1 trigger
  }

  // Step 12: Start ADC data streaming for each board
  printf("Step 5: Starting ADC data streaming for all 4 boards...\n");
  for (int board = 0; board < 4; board++) {
    // Create board-specific output file name
    char board_output_file[1024];
    char* ext_pos = strrchr(base_output_file, '.');
    if (ext_pos != NULL) {
      // Insert _bd_N before extension
      size_t base_len = ext_pos - base_output_file;
      snprintf(board_output_file, sizeof(board_output_file), "%.*s_bd_%d%s", 
               (int)base_len, base_output_file, board, ext_pos);
    } else {
      // No extension, just append _bd_N
      snprintf(board_output_file, sizeof(board_output_file), "%s_bd_%d", base_output_file, board);
    }
    
    char board_str[16], sample_count_str[32];
    snprintf(board_str, sizeof(board_str), "%d", board);
    snprintf(sample_count_str, sizeof(sample_count_str), "%llu", expected_samples_per_board);
    
    if (*(ctx->verbose)) {
      printf("  Board %d: Starting ADC data streaming to '%s' (%llu samples)\n", 
             board, board_output_file, expected_samples_per_board);
    }
    const char* adc_data_args[] = {board_str, sample_count_str, board_output_file};
    if (cmd_stream_adc_data_to_file(adc_data_args, 3, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start ADC data streaming for board %d\n", board);
      return -1;
    }
  }
  
  // Step 13: Start trigger data streaming
  if (expected_triggers > 0) {
    // Create trigger output file name
    char trigger_output_file[1024];
    char* ext_pos = strrchr(base_output_file, '.');
    if (ext_pos != NULL) {
      // Insert _trig before extension
      size_t base_len = ext_pos - base_output_file;
      snprintf(trigger_output_file, sizeof(trigger_output_file), "%.*s_trig%s", 
               (int)base_len, base_output_file, ext_pos);
    } else {
      // No extension, just append _trig
      snprintf(trigger_output_file, sizeof(trigger_output_file), "%s_trig", base_output_file);
    }
    
    char trigger_count_str[32];
    snprintf(trigger_count_str, sizeof(trigger_count_str), "%u", expected_triggers);
    
    if (*(ctx->verbose)) {
      printf("Step 6: Starting trigger data streaming to '%s' (%u samples)\n", 
             trigger_output_file, expected_triggers);
    }
    const char* trig_args[] = {trigger_count_str, trigger_output_file};
    if (cmd_stream_trig_data_to_file(trig_args, 2, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start trigger data streaming\n");
      return -1;
    }
  }
  
  // Step 14: Start command streaming threads
  printf("Step 7: Starting command streaming...\n");
  
  // Prepare streaming thread data structures
  static volatile bool dac_stream_stop = false;
  static volatile bool adc_cmd_stream_stop = false;
  
  // Reset stop flags
  dac_stream_stop = false;
  adc_cmd_stream_stop = false;
  
  // Prepare DAC streaming thread data
  rev_c_new_params_t dac_stream_data = {
    .ctx = ctx,
    .dac_file = resolved_dac_file,
    .loops = loops,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &dac_stream_stop,
    .input_is_amps = input_is_amps,
    .final_zero_trigger = final_zero_trigger
  };
  
  // Prepare ADC command streaming thread data
  rev_c_new_params_t adc_cmd_stream_data = {
    .ctx = ctx,
    .dac_file = NULL, // Not used for ADC commands
    .loops = loops,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &adc_cmd_stream_stop,
    .input_is_amps = false,  // Not used for ADC commands
    .final_zero_trigger = final_zero_trigger
  };
  
  // Start trigger monitoring similar to waveform_test
  printf("Step 8: Starting trigger monitoring...\n");
  trigger_monitor_params_t trigger_monitor_data = {
    .sys_sts = ctx->sys_sts,
    .expected_total_triggers = expected_triggers,
    .should_stop = &g_trigger_monitor_should_stop,
    .verbose = *(ctx->verbose)
  };
  
  g_trigger_monitor_should_stop = false;
  g_trigger_monitor_active = true;
  
  if (pthread_create(&g_trigger_monitor_tid, NULL, trigger_monitor_thread, &trigger_monitor_data) != 0) {
    fprintf(stderr, "Failed to create trigger monitor thread\n");
    g_trigger_monitor_active = false;
    return -1;
  }
  
  // Step 9: Start DAC and ADC command streaming threads
  printf("Starting DAC command streaming thread...\n");
  pthread_t dac_thread;
  if (pthread_create(&dac_thread, NULL, rev_c_new_dac_stream_thread, &dac_stream_data) != 0) {
    fprintf(stderr, "Failed to create DAC command streaming thread: %s\n", strerror(errno));
    g_trigger_monitor_should_stop = true;
    if (g_trigger_monitor_active) {
      pthread_join(g_trigger_monitor_tid, NULL);
      g_trigger_monitor_active = false;
    }
    return -1;
  }
  
  printf("Starting ADC command streaming thread...\n");
  pthread_t adc_cmd_thread;
  if (pthread_create(&adc_cmd_thread, NULL, rev_c_new_adc_cmd_stream_thread, &adc_cmd_stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC command streaming thread: %s\n", strerror(errno));
    dac_stream_stop = true;
    pthread_join(dac_thread, NULL);
    g_trigger_monitor_should_stop = true;
    if (g_trigger_monitor_active) {
      pthread_join(g_trigger_monitor_tid, NULL);
      g_trigger_monitor_active = false;
    }
    return -1;
  }
  
  // Wait for command buffers to preload before sending sync trigger
  printf("Step 10: Waiting for command buffers to preload (at least 10 words)...\n");
  bool buffers_ready = false;
  int check_count = 0;
  const int max_checks = 500; // Max 5 seconds at 10ms per check
  
  while (!buffers_ready && check_count < max_checks) {
    buffers_ready = true;
    
    for (int board = 0; board < 4; board++) {
      // Check DAC command buffer
      uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_words = FIFO_STS_WORD_COUNT(dac_cmd_fifo_status);
      if (dac_words < 10) {
        buffers_ready = false;
        if (*(ctx->verbose)) {
          printf("  Board %d DAC buffer: %u words (waiting for 10+)\n", board, dac_words);
        }
      }
      
      // Check ADC command buffer  
      uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t adc_words = FIFO_STS_WORD_COUNT(adc_cmd_fifo_status);
      if (adc_words < 10) {
        buffers_ready = false;
        if (*(ctx->verbose)) {
          printf("  Board %d ADC buffer: %u words (waiting for 10+)\n", board, adc_words);
        }
      }
    }
    
    if (!buffers_ready) {
      usleep(10000); // Wait 10ms
      check_count++;
    }
  }
  
  if (check_count >= max_checks) {
    printf("Warning: Timeout waiting for buffer preload!\n");
    printf("Current buffer status:\n");
    for (int board = 0; board < 4; board++) {
      uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t dac_words = FIFO_STS_WORD_COUNT(dac_cmd_fifo_status);
      printf("  Board %d DAC command buffer: %u words\n", board, dac_words);
      
      uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      uint32_t adc_words = FIFO_STS_WORD_COUNT(adc_cmd_fifo_status);
      printf("  Board %d ADC command buffer: %u words\n", board, adc_words);
    }
    
    printf("Do you want to continue anyway? (y/n): ");
    fflush(stdout);
    
    char response[16];
    if (fgets(response, sizeof(response), stdin) == NULL || (response[0] != 'y' && response[0] != 'Y')) {
      printf("Aborting Rev C compatibility mode.\n");
      dac_stream_stop = true;
      adc_cmd_stream_stop = true;
      pthread_join(dac_thread, NULL);
      pthread_join(adc_cmd_thread, NULL);
      g_trigger_monitor_should_stop = true;
      if (g_trigger_monitor_active) {
        pthread_join(g_trigger_monitor_tid, NULL);
        g_trigger_monitor_active = false;
      }
      return -1;
    }
  }
  
  // Step 11: Send sync trigger to start the process
  printf("Step 11: Sending sync trigger to start Rev C compatibility mode...\n");
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Sending sync trigger\n");
  }
  trigger_cmd_sync_ch(ctx->trigger_ctrl, false, *(ctx->verbose));
  
  // Reset trigger count after sync_ch to start counting from 0
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Resetting trigger count after sync\n");
  }
  trigger_cmd_reset_count(ctx->trigger_ctrl, *(ctx->verbose));
  
  // Set up trigger system after sync
  printf("Setting up trigger system for %u triggers...\n", expected_triggers);
  
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Expecting %u external triggers\n", expected_triggers);
  }
  trigger_cmd_expect_ext(ctx->trigger_ctrl, expected_triggers, true, *(ctx->verbose));
  
  printf("\nRev C compatibility mode: All streaming started successfully!\n");
  printf("Data collection is running. Commands are being sent to all 4 boards.\n");
  printf("ADC data will be saved to separate files for each board.\n");
  printf("Trigger data will be saved to the trigger file.\n");
  printf("Use 'stop_waveform' command to stop data collection.\n");
  
  // Wait for all threads to complete
  printf("Waiting for streaming threads to complete...\n");
  pthread_join(dac_thread, NULL);
  pthread_join(adc_cmd_thread, NULL);
  
  // Wait for trigger monitor to complete
  if (g_trigger_monitor_active) {
    printf("Waiting for trigger monitor to complete...\n");
    pthread_join(g_trigger_monitor_tid, NULL);
    g_trigger_monitor_active = false;
  }
  
  printf("Rev C compatibility mode completed successfully\n");
  return 0;
}



// ADC bias calibration command - find and store ADC bias values for all connected channels
int cmd_find_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Starting ADC bias calibration for all connected boards...\n");
  
  // Validate system is running
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  uint32_t state = HW_STS_STATE(hw_status);
  if (state != S_RUNNING) {
    fprintf(stderr, "System is not running. Current state: %d. Use 'on' command first.\n", state);
    return -1;
  }
  
  // Check which boards are connected by checking FIFOs
  bool connected_boards[8] = {false};
  int connected_count = 0;
  
  if (*(ctx->verbose)) {
    printf("Checking connected boards...\n");
  }
  
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
    printf("No boards are connected. Aborting ADC bias calibration.\n");
    return -1;
  }
  
  printf("Starting ADC bias calibration for all channels on %d connected board(s)\n", connected_count);
  
  // Store previous bias values before starting new measurement
  for (int ch = 0; ch < 64; ch++) {
    ctx->adc_bias_previous[ch] = ctx->adc_bias[ch];
    ctx->adc_bias_previous_valid[ch] = ctx->adc_bias_valid[ch];
  }
  
  // Check if --no_reset flag is present
  bool skip_reset = has_flag(flags, flag_count, FLAG_NO_RESET);
  
  // Reset buffers once at the start (unless --no_reset flag is used)
  if (!skip_reset) {
    if (*(ctx->verbose)) {
      printf("Resetting all buffers...\n");
    }
    safe_buffer_reset(ctx, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
  // Send cancel commands to all connected boards
  if (*(ctx->verbose)) {
    printf("Sending cancel commands to connected boards...\n");
  }
  for (int board = 0; board < 8; board++) {
    if (connected_boards[board]) {
      dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
      adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
    }
  }
  usleep(1000); // 1ms to let cancel commands complete
  
  // Calibration constants
  const int dac_values[] = {-3276, -1638, 0, 1638, 3276};
  const int num_dac_values = 5;
  const int slope_samples = 5; // Samples per DAC value for slope check
  const int bias_sample_count = 50; // Take 50 samples per channel for bias
  const int delay_ms = 1; // 1ms delay
  const double slope_tolerance = 0.1; // +/-0.1 slope tolerance
  
  int channels_calibrated = 0;
  int channels_failed = 0;
  bool channel_slope_valid[64] = {false}; // Track which channels pass slope test
  
  // Track failed channels for reporting
  int failed_channels_phase1[64];
  char failed_reasons_phase1[64][64];
  int phase1_failed_count = 0;
  
  int failed_channels_phase2[64];
  char failed_reasons_phase2[64][64];
  int phase2_failed_count = 0;
  
  // Initialize all ADC bias values as invalid
  for (int ch = 0; ch < 64; ch++) {
    ctx->adc_bias_valid[ch] = false;
    ctx->adc_bias[ch] = 0.0;
  }
  
  // Phase 1: Slope validation for all channels
  printf("Phase 1: Validating channels are unplugged (slope near zero)...\n");
  
  for (int ch = 0; ch < 64; ch++) {
    int board = ch / 8;
    int channel = ch % 8;
    
    // Skip boards that are not connected
    if (!connected_boards[board]) {
      continue;
    }
    
    if (*(ctx->verbose)) {
      printf("Checking slope for Ch %02d...\n", ch);
    }
    
    // Arrays to store DAC values and corresponding averaged ADC readings for slope test
    double dac_vals[num_dac_values];
    double avg_adc_vals[num_dac_values];
    bool slope_test_failed = false;
    
    // Test each DAC value for slope calculation
    for (int i = 0; i < num_dac_values && !slope_test_failed; i++) {
      int16_t dac_val = dac_values[i];
      dac_vals[i] = (double)dac_val;
      
      // Perform multiple reads and average them
      double sum_adc = 0.0;
      
      for (int avg = 0; avg < slope_samples && !slope_test_failed; avg++) {
        // Write DAC value
        dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, dac_val, false);
        usleep(delay_ms * 1000); // Wait fixed delay
        
        // Read ADC value
        adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, false);
        
        // Wait for ADC data to be available
        int adc_tries = 0;
        uint32_t adc_data_fifo_status;
        while (adc_tries < 100) {
          adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) > 0) break;
          usleep(100); // 0.1ms
          adc_tries++;
        }
        
        if (FIFO_STS_WORD_COUNT(adc_data_fifo_status) == 0) {
          slope_test_failed = true;
          break;
        }
        
        uint32_t adc_word = adc_read_word(ctx->adc_ctrl, (uint8_t)board);
        int16_t adc_reading = offset_to_signed(adc_word & 0xFFFF);
        double adc_val = (double)adc_reading;
        
        sum_adc += adc_val;
      }
      
      if (slope_test_failed) break;
      
      avg_adc_vals[i] = sum_adc / slope_samples;
    }
    
    if (slope_test_failed) {
      if (*(ctx->verbose)) {
        printf("  Ch %02d: FAIL (no ADC data)\n", ch);
      }
      failed_channels_phase1[phase1_failed_count] = ch;
      snprintf(failed_reasons_phase1[phase1_failed_count], sizeof(failed_reasons_phase1[phase1_failed_count]), "no ADC data");
      phase1_failed_count++;
      channels_failed++;
      continue;
    }
    
    // Perform linear regression: y = mx + b
    double sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
    for (int i = 0; i < num_dac_values; i++) {
      sum_x += dac_vals[i];
      sum_y += avg_adc_vals[i];
      sum_xy += dac_vals[i] * avg_adc_vals[i];
      sum_x2 += dac_vals[i] * dac_vals[i];
    }
    
    // Calculate slope and check for division by zero
    double denominator = num_dac_values * sum_x2 - sum_x * sum_x;
    double slope;
    
    if (denominator == 0) {
      if (*(ctx->verbose)) {
        printf("  Ch %02d: FAIL (div by zero)\n", ch);
      }
      failed_channels_phase1[phase1_failed_count] = ch;
      snprintf(failed_reasons_phase1[phase1_failed_count], sizeof(failed_reasons_phase1[phase1_failed_count]), "div by zero");
      phase1_failed_count++;
      channels_failed++;
      continue;
    }
    
    slope = (num_dac_values * sum_xy - sum_x * sum_y) / denominator;
    
    // Check if slope is within tolerance (close to 0)
    if (slope < -slope_tolerance || slope > slope_tolerance) {
      if (*(ctx->verbose)) {
        printf("  Ch %02d: FAIL (slope=%.4f, not near 0)\n", ch, slope);
      }
      failed_channels_phase1[phase1_failed_count] = ch;
      snprintf(failed_reasons_phase1[phase1_failed_count], sizeof(failed_reasons_phase1[phase1_failed_count]), "slope=%.4f (not near 0)", slope);
      phase1_failed_count++;
      channels_failed++;
      continue;
    }
    
    // Slope is acceptable
    channel_slope_valid[ch] = true;
    if (*(ctx->verbose)) {
      printf("  Ch %02d: slope check passed (slope=%.4f)\n", ch, slope);
    }
  }
  
  // Count channels that passed slope test
  int slope_passed_count = 0;
  for (int ch = 0; ch < 64; ch++) {
    if (channel_slope_valid[ch]) {
      slope_passed_count++;
    }
  }
  
  printf("Phase 1 complete: %d channels passed slope validation, %d failed\n", slope_passed_count, channels_failed);
  
  if (channels_failed > 0) {
    printf("Some channels failed slope validation. Aborting bias calibration to avoid partial results.\n");
    printf("Failed channels in Phase 1 (slope validation):\n");
    for (int i = 0; i < phase1_failed_count; i++) {
      int ch = failed_channels_phase1[i];
      int board = ch / 8;
      int channel = ch % 8;
      printf("  Ch %02d (Board %d, Channel %d): %s\n", ch, board, channel, failed_reasons_phase1[i]);
    }
    return -1;
  }
  
  if (slope_passed_count == 0) {
    printf("No channels passed slope validation. Aborting bias calibration.\n");
    return -1;
  }
  
  // Phase 2: Bias measurement for channels that passed slope test
  printf("Phase 2: Measuring ADC bias (sampling at DAC=0, 50 samples per channel)...\n");
  
  channels_failed = 0; // Reset for bias measurement phase
  
  // Iterate through channels that passed slope test
  for (int ch = 0; ch < 64; ch++) {
    int board = ch / 8;
    int channel = ch % 8;
    
    // Skip channels that didn't pass slope test or boards not connected
    if (!connected_boards[board] || !channel_slope_valid[ch]) {
      continue;
    }
    
    printf("Ch %02d : ", ch);
    fflush(stdout);
    
    // Array to store all samples for this channel
    double samples[bias_sample_count];
    bool calibration_failed = false;
    
    // Set DAC to zero value
    dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, false);
    usleep(delay_ms * 1000); // Wait for DAC to settle
    
    // Collect samples
    for (int i = 0; i < bias_sample_count && !calibration_failed; i++) {
      // Read ADC value
      adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, false);
      
      // Wait for ADC data to be available
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
      samples[i] = (double)adc_reading;
      
      usleep(delay_ms * 1000); // Small delay between samples
    }
    
    if (calibration_failed) {
      printf("FAIL (no ADC data)\n");
      failed_channels_phase2[phase2_failed_count] = ch;
      snprintf(failed_reasons_phase2[phase2_failed_count], sizeof(failed_reasons_phase2[phase2_failed_count]), "no ADC data");
      phase2_failed_count++;
      channels_failed++;
      continue;
    }
    
    // Calculate mean (bias)
    double sum = 0.0;
    for (int i = 0; i < bias_sample_count; i++) {
      sum += samples[i];
    }
    double bias_average = sum / bias_sample_count;
    
    // Calculate standard deviation
    double variance_sum = 0.0;
    for (int i = 0; i < bias_sample_count; i++) {
      double diff = samples[i] - bias_average;
      variance_sum += diff * diff;
    }
    double std_dev = variance_sum > 0 ? (variance_sum / (bias_sample_count - 1)) : 0.0;
    // Calculate square root manually without math library
    double std_dev_sqrt = 0.0;
    if (std_dev > 0) {
      // Newton's method for square root
      std_dev_sqrt = std_dev / 2.0;
      for (int iter = 0; iter < 10; iter++) {
        std_dev_sqrt = (std_dev_sqrt + std_dev / std_dev_sqrt) / 2.0;
      }
    }
    
    // Store the bias value
    ctx->adc_bias[ch] = bias_average;
    ctx->adc_bias_valid[ch] = true;
    
    // Format bias result with proper alignment and difference from previous
    char diff_str[32] = "";
    if (ctx->adc_bias_previous_valid[ch]) {
      double diff = bias_average - ctx->adc_bias_previous[ch];
      snprintf(diff_str, sizeof(diff_str), " (%+.2f)", diff);
    } else {
      snprintf(diff_str, sizeof(diff_str), " (new)");
    }
    
    printf("bias=%+7.2f, std=%5.2f%s\n", bias_average, std_dev_sqrt, diff_str);
    channels_calibrated++;
  }
  
  printf("\nADC bias calibration complete:\n");
  printf("  Channels calibrated: %d\n", channels_calibrated);
  printf("  Channels failed: %d\n", channels_failed);
  
  if (channels_failed > 0) {
    printf("Failed channels in Phase 2 (bias measurement):\n");
    for (int i = 0; i < phase2_failed_count; i++) {
      int ch = failed_channels_phase2[i];
      int board = ch / 8;
      int channel = ch % 8;
      printf("  Ch %02d (Board %d, Channel %d): %s\n", ch, board, channel, failed_reasons_phase2[i]);
    }
  }
  
  if (channels_calibrated == 0) {
    printf("No channels were successfully calibrated.\n");
    return -1;
  }
  
  printf("\nADC bias values for successfully calibrated channels:\n");
  for (int ch = 0; ch < 64; ch++) {
    if (ctx->adc_bias_valid[ch]) {
      int board = ch / 8;
      int channel = ch % 8;
      
      // Format difference from previous measurement
      char diff_str[32] = "";
      if (ctx->adc_bias_previous_valid[ch]) {
        double diff = ctx->adc_bias[ch] - ctx->adc_bias_previous[ch];
        snprintf(diff_str, sizeof(diff_str), " (%+.2f)", diff);
      } else {
        snprintf(diff_str, sizeof(diff_str), " (new)");
      }
      
      printf("  Ch %02d (Board %d, Channel %d): %+7.2f%s\n", 
             ch, board, channel, ctx->adc_bias[ch], diff_str);
    }
  }
  
  printf("ADC bias calibration completed successfully.\n");
  return 0;
}

// Print ADC bias values command
int cmd_print_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Current ADC bias values:\n");
  
  int valid_count = 0;
  int invalid_count = 0;
  
  printf("%-6s %-8s %-8s %-10s %s\n", "Ch", "Board", "Channel", "Bias", "Status");
  printf("%-6s %-8s %-8s %-10s %s\n", "------", "--------", "--------", "----------", "--------");
  
  for (int ch = 0; ch < 64; ch++) {
    int board = ch / 8;
    int channel = ch % 8;
    
    if (ctx->adc_bias_valid[ch]) {
      printf("%-6d %-8d %-8d %-10.2f %s\n", ch, board, channel, ctx->adc_bias[ch], "Valid");
      valid_count++;
    } else {
      printf("%-6d %-8d %-8d %-10s %s\n", ch, board, channel, "N/A", "Invalid");
      invalid_count++;
    }
  }
  
  printf("\nSummary: %d valid bias values, %d invalid\n", valid_count, invalid_count);
  return 0;
}

// Save ADC bias values to CSV file command
int cmd_save_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  const char* filename = args[0];
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(filename, full_path, sizeof(full_path));
  
  FILE* file = fopen(full_path, "w");
  if (file == NULL) {
    fprintf(stderr, "Failed to open file '%s' for writing: %s\n", full_path, strerror(errno));
    return -1;
  }
  
  // Write CSV header
  fprintf(file, "Channel,Board,Channel_Index,Bias_Value,Valid\n");
  
  int saved_count = 0;
  
  // Write all channels
  for (int ch = 0; ch < 64; ch++) {
    int board = ch / 8;
    int channel = ch % 8;
    
    fprintf(file, "%d,%d,%d,%.6f,%s\n", 
            ch, board, channel, 
            ctx->adc_bias_valid[ch] ? ctx->adc_bias[ch] : 0.0,
            ctx->adc_bias_valid[ch] ? "true" : "false");
    
    if (ctx->adc_bias_valid[ch]) {
      saved_count++;
    }
  }
  
  fclose(file);
  
  printf("Successfully saved ADC bias values to '%s'\n", full_path);
  printf("Saved %d valid bias values (out of 64 channels)\n", saved_count);
  
  return 0;
}

// Load ADC bias values from CSV file command
int cmd_load_adc_bias(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  const char* filename = args[0];
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(filename, full_path, sizeof(full_path));
  
  FILE* file = fopen(full_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open file '%s' for reading: %s\n", full_path, strerror(errno));
    return -1;
  }
  
  // Read and skip header line
  char line[256];
  if (fgets(line, sizeof(line), file) == NULL) {
    fprintf(stderr, "Failed to read header line from file '%s'\n", full_path);
    fclose(file);
    return -1;
  }
  
  // Initialize all bias values as invalid before loading
  for (int ch = 0; ch < 64; ch++) {
    ctx->adc_bias_valid[ch] = false;
    ctx->adc_bias[ch] = 0.0;
  }
  
  int loaded_count = 0;
  int line_num = 1;
  
  // Read data lines
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Parse CSV line: Channel,Board,Channel_Index,Bias_Value,Valid
    int channel, board, channel_index;
    double bias_value;
    char valid_str[16];
    
    int parsed = sscanf(line, "%d,%d,%d,%lf,%15s", 
                       &channel, &board, &channel_index, &bias_value, valid_str);
    
    if (parsed != 5) {
      fprintf(stderr, "Warning: Invalid format on line %d, skipping\n", line_num);
      continue;
    }
    
    // Validate channel number
    if (channel < 0 || channel >= 64) {
      fprintf(stderr, "Warning: Invalid channel %d on line %d, skipping\n", channel, line_num);
      continue;
    }
    
    // Check if this bias value should be marked as valid
    bool is_valid = (strcmp(valid_str, "true") == 0);
    
    if (is_valid) {
      ctx->adc_bias[channel] = bias_value;
      ctx->adc_bias_valid[channel] = true;
      loaded_count++;
    }
  }
  
  fclose(file);
  
  printf("Successfully loaded ADC bias values from '%s'\n", full_path);
  printf("Loaded %d valid bias values from file\n", loaded_count);
  
  if (loaded_count > 0) {
    printf("ADC bias correction is now active for %d channels\n", loaded_count);
  } else {
    printf("No valid bias values were loaded\n");
  }
  
  return 0;
}
