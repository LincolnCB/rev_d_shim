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
#include "rev_c_compat.h"
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

// Data structure for rev_c streaming
typedef struct {
  command_context_t* ctx;
  char* dac_file;
  int iterations;
  int ramp_samples;
  int ramp_delay_cycles;
  int line_count;
  uint32_t delay_cycles;
  volatile bool* should_stop;
  bool final_zero_trigger;
} rev_c_params_t;

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
    float amp_vals[32];
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
      
      // Check range (-5.0 to 5.0)
      if (val < -5.0f || val > 5.0f) {
        fprintf(stderr, "Rev C DAC file (Amps) line %d, value %d: %.3f out of range (-5.0 to 5.0)\n", 
                line_num, i+1, val);
        fclose(file);
        return -1;
      }
      
      amp_vals[i] = val;
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

// Thread function for Rev C DAC command streaming to all 4 boards
static void* rev_c_dac_cmd_stream_thread(void* arg) {
  rev_c_params_t* stream_data = (rev_c_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  const char* dac_file = stream_data->dac_file;
  int iterations = stream_data->iterations;
  int ramp_samples = stream_data->ramp_samples;
  int ramp_delay_cycles = stream_data->ramp_delay_cycles;
  int line_count = stream_data->line_count;
  volatile bool* should_stop = stream_data->should_stop;
  bool final_zero_trigger = stream_data->final_zero_trigger;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C DAC Stream Thread: Starting streaming from file '%s' (%d lines, %d iterations, final_zero=%s)\n", 
         dac_file, line_count, iterations, final_zero_trigger ? "yes" : "no");
  
  FILE* file = fopen(dac_file, "r");
  if (file == NULL) {
    fprintf(stderr, "Rev C DAC Stream Thread: Failed to open file '%s': %s\n", dac_file, strerror(errno));
    return NULL;
  }
  
  int total_commands_sent = 0;
  int total_words_sent = 0;
  
  // Process all iterations
  for (int iteration = 0; iteration < iterations && !(*should_stop); iteration++) {
    rewind(file);
    char line[2048];
    int line_num = 0;

    int16_t prev_line_dac_vals[32] = {0}; // To store previous values for ramping
    
    while (fgets(line, sizeof(line), file) && !(*should_stop)) {
      // Skip empty lines and comments
      char* trimmed = line;
      while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
      if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
        continue;
      }
      
      line_num++;
      
      // Parse 32 values from the line (already validated in main function)
      int16_t line_dac_vals[32];
      char* token_start = trimmed;
      char* endptr;
      
      // Parse as floats and convert to DAC units
      for (int i = 0; i < 32; i++) {
        while (*token_start == ' ' || *token_start == '\t') token_start++;
        float amp_val = strtof(token_start, &endptr);
        line_dac_vals[i] = amps_to_dac(amp_val);
        token_start = endptr;
      }
      
      // Send DAC write commands to each of the 4 boards
      for (int board = 0; board < 4; board++) {
        // Wait for FIFO space (for whole ramp including final sample)
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C New DAC Stream Thread: Board %d FIFO not present, stopping\n", board);
            goto cleanup;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = DAC_CMD_FIFO_WORDCOUNT - words_used;
          if (available_words >= 5 * (ramp_samples + 1)) { // Need space for 1 command + 4 data words
            break; // Space available, proceed with command
          }
          
          usleep(1000); // 1ms delay before checking again
        }
        
        // Handle ramping if requested
        for (int ramp_step = 0; ramp_step <= ramp_samples; ramp_step++) {
          float ramp_fraction = (float)(ramp_step + 1) / (float)(ramp_samples + 1);
          int16_t cmd_ch_vals[8];
          for (int ch = 0; ch < 8; ch++) {
            int16_t target_val = line_dac_vals[board * 8 + ch];
            int16_t start_val = prev_line_dac_vals[board * 8 + ch];
            int16_t ramped_val = start_val + (int16_t)((target_val - start_val) * ramp_fraction);
            cmd_ch_vals[ch] = ramped_val;
          }
        
          // For the last dac_wr command at the end of the ramp of the last line, cont should be false
          bool is_last_iteration = (iteration == iterations - 1);
          bool is_last_line = (line_num == line_count);
          bool is_last_ramp_step = (ramp_step == ramp_samples);
          bool cont = !(is_last_iteration && is_last_line && is_last_ramp_step);

          bool trig = (ramp_step == 0); // Only wait for trigger on first ramp step
          int count = trig ? 1 : ramp_delay_cycles; // 1 trigger or delay cycles

          // Send DAC write command with trigger wait (trig=trig, cont=cont, ldac=true, 1 trigger OR delay)
          dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, cmd_ch_vals, trig, cont, true, count, verbose);
          total_commands_sent++;
          total_words_sent += 5; // 1 command + 4 data words
        
          if (verbose && line_num <= 10) { // Only show first few lines to avoid spam
            printf("Rev C DAC Stream Thread: Board %d, Line %d, Iteration %d, sent DAC write (5 words, channels %d-%d)\n", 
                  board, line_num, iteration + 1, board * 8, board * 8 + 7);
          }
        }
      }
      
      // Small delay between lines to prevent overwhelming the system
      usleep(100); // 100μs delay
    }
    
    if (verbose) {
      printf("Rev C DAC Stream Thread: Completed iteration %d/%d\n", iteration + 1, iterations);
    }
  }
  
  // Send final zero trigger if requested
  if (final_zero_trigger && !(*should_stop)) {
    printf("Rev C DAC Stream Thread: Sending final zero trigger...\n");
    
    // Create array of zeros in signed format (0.0 amps = 0 signed)
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
      
      // Send DAC write command with trigger wait (trig=true, cont=false, ldac=true, 1 trigger)
      dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, zero_vals, true, false, true, 1, verbose);
      total_commands_sent++;
      total_words_sent += 5; // 1 command + 4 data words
      
      if (verbose) {
        printf("Rev C DAC Stream Thread: Board %d, sent final zero DAC write\n", board);
      }
    }
  }
  
cleanup:
  fclose(file);
  
  if (*should_stop) {
    printf("Rev C DAC Stream Thread: Stopping stream (user requested), sent %d total commands (%d total words)\n",
            total_commands_sent, total_words_sent);
  } else {
    printf("Rev C DAC Stream Thread: Stream completed, sent %d total commands (%d total words, %d iteration%s%s)\n", 
           total_commands_sent, total_words_sent, iterations, iterations == 1 ? "" : "s", final_zero_trigger ? " + final zero" : "");
  }
  
  return NULL;
}

// Thread function for Rev C ADC command streaming to all 4 boards
static void* rev_c_adc_cmd_stream_thread(void* arg) {
  rev_c_params_t* stream_data = (rev_c_params_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  int iterations = stream_data->iterations;
  int ramp_samples = stream_data->ramp_samples;
  int ramp_delay_cycles = stream_data->ramp_delay_cycles;
  int line_count = stream_data->line_count;
  uint32_t delay_cycles = stream_data->delay_cycles;
  volatile bool* should_stop = stream_data->should_stop;
  bool final_zero_trigger = stream_data->final_zero_trigger;
  bool verbose = *(ctx->verbose);
  
  printf("Rev C ADC Command Stream Thread: Starting (%d lines, %d iterations, delay=%u cycles, final_zero=%s)\n", 
         line_count, iterations, delay_cycles, final_zero_trigger ? "yes" : "no");
  
  int total_commands_sent = 0;
  int total_words_sent = 0;
  
  // First, send set_ord commands to all boards (order: 01234567)
  printf("Rev C ADC Command Stream Thread: Sending set_ord commands to all boards...\n");
  uint8_t channel_order[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  for (int board = 0; board < 4; board++) {
    // Wait for FIFO space for set_ord command
    while (!(*should_stop)) {
      uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
      
      if (FIFO_PRESENT(fifo_status) == 0) {
        fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present for set_ord, stopping\n", board);
        goto cleanup;
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
    total_words_sent ++;
    
    if (verbose) {
      printf("Rev C ADC Command Stream Thread: Board %d, sent set_ord command\n", board);
    }
  }
  
  // Process all iterations
  for (int iteration = 0; iteration < iterations && !(*should_stop); iteration++) {
    for (int line_num = 1; line_num <= line_count && !(*should_stop); line_num++) {
      
      // For each line, send:
      // No ramp: 3 ADC commands per board:
      // 1. noop with trigger wait (1 trigger)  
      // 2. noop with delay wait (delay_cycles)
      // 3. adc_read with no trigger wait (0 triggers)
      // Ramping: Repeated ADC read commands in sync with ramp samples until delay_cycles have passed:
      for (int board = 0; board < 4; board++) {
        
        // Wait for FIFO space for all 3 commands
        while (!(*should_stop)) {
          uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
          
          if (FIFO_PRESENT(fifo_status) == 0) {
            fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present, stopping\n", board);
            goto cleanup;
          }
          
          uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
          uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
          if (ramp_samples > 0) {
            if (available_words >= ramp_samples) { // Need space for ramp_samples commands
              break; // Space available, proceed with commands
            }
          } else {
            if (available_words >= 3) { // Need space for 3 commands
              break; // Space available, proceed with commands
            }
          }
          
          usleep(1000); // 1ms delay before checking again
        }
        
        if (ramp_samples > 0) {
          // If ramping, repeatedly sample until the delay_cycles have passed
          // Command 1: ADC read with trigger wait for 1 triggers
          adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, verbose);
          total_commands_sent++;
          total_words_sent++;
          
          // Repeat ramp samples
          for (int ramp_step = 1; ramp_step < ramp_samples - 1; ramp_step++) {
            adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, ramp_delay_cycles, 0, verbose);
            total_commands_sent++;
            total_words_sent++;
          }
          // Final ADC read with remaining delay cycles
          adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, ramp_delay_cycles + delay_cycles, 0, verbose);
          total_commands_sent++;
          total_words_sent++;

          if (verbose && line_num <= 3) { // Only show first few lines to avoid spam
            printf("Rev C ADC Command Stream Thread: Board %d, Line %d, Iteration %d, sent %d ADC commands\n", 
                    board, line_num, iteration + 1, ramp_samples + 1);
          }
        } else {
          // Command 1: NOOP with trigger wait for 1 trigger
          adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, verbose);
          total_commands_sent++;
          total_words_sent++;
          
          // Command 2: NOOP with delay wait for delay_cycles
          adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, verbose);
          total_commands_sent++;
          total_words_sent++;
          
          // Command 3: ADC read with trigger wait for no triggers (0 triggers)
          adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, 0, verbose);
          total_commands_sent++;
          total_words_sent++;
          
          if (verbose && line_num <= 3) { // Only show first few lines to avoid spam
            printf("Rev C ADC Command Stream Thread: Board %d, Line %d, Iteration %d, sent 3 ADC commands\n", 
                  board, line_num, iteration + 1);
          }
        }
      }
      
      // Small delay between lines to prevent overwhelming the system
      usleep(100); // 100μs delay
    }
    
    if (verbose) {
      printf("Rev C ADC Command Stream Thread: Completed iteration %d/%d\n", iteration + 1, iterations);
    }
  }
  
  // Send final ADC commands if final zero line is requested
  if (final_zero_trigger && !(*should_stop)) {
    printf("Rev C ADC Command Stream Thread: Sending final zero ADC commands...\n");
    
    for (int board = 0; board < 4; board++) {
      // Wait for FIFO space for final zero trigger commands
      while (!(*should_stop)) {
        uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
        
        if (FIFO_PRESENT(fifo_status) == 0) {
          fprintf(stderr, "Rev C New ADC Command Stream Thread: Board %d FIFO not present for final zero, stopping\n", board);
          goto cleanup;
        }
        
        uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
        uint32_t available_words = ADC_CMD_FIFO_WORDCOUNT - words_used;
        if (available_words >= 3) { // Need space for 3 commands
          break; // Space available, proceed with commands
        }
        
        usleep(1000); // 1ms delay before checking again
      }
      
      if (ramp_samples > 0) {
        // If ramping, repeatedly sample until the delay_cycles have passed
        // Command 1: ADC read with trigger wait for 1 triggers
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 1, 0, verbose);
        total_commands_sent++;
        total_words_sent++;

        // Repeat ramp samples
        for (int ramp_step = 1; ramp_step < ramp_samples - 1; ramp_step++) {
          adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, ramp_delay_cycles, 0, verbose);
          total_commands_sent++;
          total_words_sent++;
        }
        // Final ADC read with remaining delay cycles
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, ramp_delay_cycles + delay_cycles, 0, verbose);
        total_commands_sent++;
        total_words_sent++;
      } else {
        // Command 1: NOOP with trigger wait for 1 trigger
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, verbose);
        total_commands_sent++;
        total_words_sent++;
        
        // Command 2: NOOP with delay wait for delay_cycles
        adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, false, false, delay_cycles, verbose);
        total_commands_sent++;
        total_words_sent++;
        
        // Command 3: ADC read with trigger wait for no triggers (0 triggers)
        adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, true, false, 0, 0, verbose);
        total_commands_sent++;
        total_words_sent++;
      }
      
      if (verbose) {
        printf("Rev C ADC Command Stream Thread: Board %d, sent final zero ADC commands\n", board);
      }
    }
  }

cleanup:
  if (*should_stop) {
    printf("Rev C ADC Command Stream Thread: Stopping stream (user requested), sent %d total commands (%d total words)\n",
           total_commands_sent, total_words_sent);
  } else {
    printf("Rev C ADC Command Stream Thread: Stream completed, sent %d total commands (%d total words, %d iteration%s%s)\n", 
           total_commands_sent, total_words_sent, iterations, iterations == 1 ? "" : "s", final_zero_trigger ? " + final zero" : "");
  }
  
  return NULL;
}

// Rev C compatibility command implementation
int cmd_rev_c_compat(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)args; (void)arg_count; // Suppress unused parameter warnings
  
  printf("Starting Rev C compatibility mode...\n");

  // Make sure the system IS running
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
  
  // Reset all buffers (unless --no_reset flag is used)
  if (skip_reset) {
    printf("Skipping buffer reset (--no_reset flag specified)\n");
  } else {
    printf("Resetting all buffers\n");
    safe_buffer_reset(ctx, *(ctx->verbose));
    usleep(10000); // 10ms
  }
  
  // Check that boards 0-3 are connected
  printf("Checking board connections (boards 0-3)...\n");
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
  
  // Prompt for DAC command file
  char resolved_dac_file[1024];
  if (prompt_file_selection("Enter DAC command file (32 space-separated values per line)", 
                           NULL, resolved_dac_file, sizeof(resolved_dac_file)) != 0) {
    fprintf(stderr, "Failed to get DAC file\n");
    return -1;
  }
  
  // Validate file format
  printf("Validating DAC file format...\n");
  int line_count;
  if (validate_rev_c_file_format_amps(resolved_dac_file, &line_count) != 0) {
    return -1;
  }
  printf("  Amps file validation passed: %d valid data lines\n", line_count);
  
  char input_buffer[64];
  size_t len;

  // Prompt for number of iterations
  printf("Enter number of iterations: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read iteration count.\n");
    return -1;
  }

  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }
  
  int iterations = atoi(input_buffer);
  if (iterations < 1) {
    fprintf(stderr, "Invalid iteration count. Must be >= 1.\n");
    return -1;
  }
  
  // Prompt for SPI frequency
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
  
  // Prompt for the number of ramp samples
  printf("Enter number of ramp samples: ");
  fflush(stdout);
  
  if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
    fprintf(stderr, "Failed to read ramp samples.\n");
    return -1;
  }
  // Remove newline
  len = strlen(input_buffer);
  if (len > 0 && input_buffer[len - 1] == '\n') {
    input_buffer[len - 1] = '\0';
  }

  int ramp_samples = atoi(input_buffer);
  if (ramp_samples < 0) {
    fprintf(stderr, "Invalid ramp samples. Must be >= 0.\n");
    return -1;
  }

  // Prompt for the ramp time in milliseconds if ramp samples > 0
  int ramp_delay_cycles = 0;
  if (ramp_samples > 0) {
    printf("Enter ramp time in milliseconds: ");
    fflush(stdout);

    if (fgets(input_buffer, sizeof(input_buffer), stdin) == NULL) {
      fprintf(stderr, "Failed to read ramp time.\n");
      return -1;
    }
    
    // Remove newline
    len = strlen(input_buffer);
    if (len > 0 && input_buffer[len - 1] == '\n') {
      input_buffer[len - 1] = '\0';
    }

    double ramp_time_ms = atof(input_buffer);
    if (ramp_time_ms < 0.02 * ramp_samples) {
      fprintf(stderr, "Invalid ramp time. Must be >= %.2f ms for %d samples (20μs per sample).\n", 
              0.02 * ramp_samples, ramp_samples);
      return -1;
    }

    // Calculate ramp delay cycles from milliseconds and SPI frequency
    ramp_delay_cycles = (int)(ramp_time_ms * spi_freq_mhz * 1000.0 / ramp_samples);
    printf("Calculated ramp delay: %d cycles per sample (%.3f ms total for %d samples at %.3f MHz)\n", 
           ramp_delay_cycles, ramp_time_ms, ramp_samples, spi_freq_mhz);
  }
    
  // Prompt for ADC sample delay in milliseconds
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
  
  // Prompt for trigger lockout time
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
  
  // Prompt for final zero trigger
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
  
  // Prompt for base output file name
  char base_output_file[512];
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
  
  // Calculate expected sample count: 3 ADC reads per board per line * iterations (plus final zero if enabled)
  uint64_t total_lines = (uint64_t)line_count * iterations;
  if (final_zero_trigger) {
    total_lines++; // Add one more line for final zero
  }
  // Calculate total ADC ramp samples
  uint32_t adc_ramp_samples = (ramp_samples > 0) ? ramp_samples + (delay_cycles / ramp_delay_cycles) : 0;
  int delay_remainder = (ramp_samples > 0) ? (delay_cycles % ramp_delay_cycles) : delay_cycles;
  uint64_t expected_samples_per_board = total_lines * ((adc_ramp_samples > 0) ? adc_ramp_samples : 1) * 4; // 4 ADC samples per line per board
  uint32_t expected_triggers = (uint32_t)total_lines; // 1 trigger per line
  
  printf("\nCalculated expected data counts:\n");
  printf("  Lines per iteration: %d\n", line_count);
  printf("  Total iterations: %d\n", iterations);
  printf("  Final zero trigger: %s\n", final_zero_trigger ? "Yes" : "No");
  printf("  Total lines to process: %llu\n", total_lines);
  printf("  Expected triggers: %u\n", expected_triggers);
  printf("  Expected ADC samples per board: %llu\n", expected_samples_per_board);
  
  printf("\nStarting Rev C compatibility mode with:\n");
  printf("  Input DAC file: %s\n", resolved_dac_file);
  printf("  Input format: Amps (-5.0 to 5.0)");
  printf("  Iterations: %d\n", iterations);
  printf("  Ramp samples: %d\n", ramp_samples);
  if (ramp_samples > 0) {
    printf("  Ramp delay: %d cycles\n", ramp_delay_cycles);
  }
  printf("  ADC delay: %.3f ms (%u cycles)\n", adc_delay_ms, delay_cycles);
  printf("  Output format: %s\n", binary_mode ? "binary" : "ASCII");
  printf("  Final zero trigger: %s\n", final_zero_trigger ? "enabled" : "disabled");
  
  // Add buffer stoppers before starting streams
  printf("Adding buffer stoppers before starting streams...\n");
  for (int board = 0; board < 4; board++) {
    if (*(ctx->verbose)) {
      printf("  Board %d: Adding DAC and ADC buffer stoppers\n", board);
    }
    
    // Add DAC NOOP stopper
    dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, true, false, false, 1, *(ctx->verbose)); // Wait for 1 trigger
    
    // Add ADC NOOP stopper  
    adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, true, false, 1, *(ctx->verbose)); // Wait for 1 trigger
  }

  // Start ADC data streaming for each board
  printf("Starting ADC data streaming for all 4 boards...\n");
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
  
  // Start trigger data streaming
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
      printf("Starting trigger data streaming to '%s' (%u samples)\n", 
             trigger_output_file, expected_triggers);
    }
    const char* trig_args[] = {trigger_count_str, trigger_output_file};
    if (cmd_stream_trig_data_to_file(trig_args, 2, NULL, 0, ctx) != 0) {
      fprintf(stderr, "Failed to start trigger data streaming\n");
      return -1;
    }
  }
  
  // Start command streaming threads
  printf("Starting command streaming...\n");
  
  // Prepare streaming thread data structures
  static volatile bool dac_cmd_stream_stop = false;
  static volatile bool adc_cmd_stream_stop = false;
  
  // Reset stop flags
  dac_cmd_stream_stop = false;
  adc_cmd_stream_stop = false;
  
  // Prepare DAC streaming thread data
  rev_c_params_t dac_cmd_stream_data = {
    .ctx = ctx,
    .dac_file = resolved_dac_file,
    .iterations = iterations,
    .ramp_samples = ramp_samples,
    .ramp_delay_cycles = ramp_delay_cycles,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &dac_cmd_stream_stop,
    .final_zero_trigger = final_zero_trigger
  };
  
  // Prepare ADC command streaming thread data
  rev_c_params_t adc_cmd_stream_data = {
    .ctx = ctx,
    .dac_file = NULL, // Not used for ADC commands
    .iterations = iterations,
    .ramp_samples = adc_ramp_samples,
    .ramp_delay_cycles = ramp_delay_cycles,
    .line_count = line_count,
    .delay_cycles = delay_cycles,
    .should_stop = &adc_cmd_stream_stop,
    .final_zero_trigger = final_zero_trigger
  };
  
  // Start trigger monitoring similar to waveform_test
  printf("Starting trigger monitoring...\n");
  
  if (start_trigger_monitor(ctx->sys_sts, expected_triggers, *(ctx->verbose)) != 0) {
    fprintf(stderr, "Failed to start trigger monitor\n");
    return -1;
  }
  
  // Start DAC and ADC command streaming threads
  printf("Starting DAC command streaming thread...\n");
  pthread_t dac_thread;
  if (pthread_create(&dac_thread, NULL, rev_c_dac_cmd_stream_thread, &dac_cmd_stream_data) != 0) {
    fprintf(stderr, "Failed to create DAC command streaming thread: %s\n", strerror(errno));
    if (is_trigger_monitor_active()) {
      stop_trigger_monitor();
    }
    return -1;
  }
  
  // Detach the DAC thread so it can clean up automatically
  pthread_detach(dac_thread);
  
  printf("Starting ADC command streaming thread...\n");
  pthread_t adc_cmd_thread;
  if (pthread_create(&adc_cmd_thread, NULL, rev_c_adc_cmd_stream_thread, &adc_cmd_stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC command streaming thread: %s\n", strerror(errno));
    dac_cmd_stream_stop = true;
    if (is_trigger_monitor_active()) {
      stop_trigger_monitor();
    }
    return -1;
  }
  
  // Detach the ADC command thread so it can clean up automatically
  pthread_detach(adc_cmd_thread);
  
  // Wait for command buffers to preload before sending sync trigger
  printf("Waiting for command buffers to preload (at least 10 words)...\n");
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
      dac_cmd_stream_stop = true;
      adc_cmd_stream_stop = true;
      if (is_trigger_monitor_active()) {
        stop_trigger_monitor();
      }
      return -1;
    }
  }
  
  // Send sync trigger to start the process
  printf("  Sending sync trigger to start Rev C compatibility mode...\n");
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Sending sync trigger\n");
  }
  trigger_cmd_sync_ch(ctx->trigger_ctrl, false, *(ctx->verbose));
  
  // Reset trigger count after sync_ch to start counting from 0
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Resetting trigger count after sync\n");
  }
  trigger_cmd_reset_count(ctx->trigger_ctrl, *(ctx->verbose));
  
  // Set trigger lockout
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Setting trigger lockout time to %u cycles\n", lockout_time);
  }
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_time, *(ctx->verbose));
  
  // Set up trigger system after sync
  printf("Setting up trigger system for %u triggers...\n", expected_triggers);
  
  if (*(ctx->verbose)) {
    printf("Rev C [VERBOSE]: Expecting %u external triggers\n", expected_triggers);
  }
  trigger_cmd_expect_ext(ctx->trigger_ctrl, expected_triggers, true, *(ctx->verbose));
  
  printf("\nRev C compatibility mode started - streams running in background, trigger monitoring active.\n");
  printf("Data collection is running. Commands are being sent to all 4 boards.\n");
  printf("ADC data will be saved to separate files for each board.\n");
  printf("Trigger data will be saved to the trigger file.\n");
  printf("Use 'stop_waveform' command to stop data collection.\n");
  
  if (*(ctx->verbose)) {
    printf("Expected data collection:\n");
    printf("  Total triggers: %u\n", expected_triggers);
    printf("  ADC samples per board: %llu\n", expected_samples_per_board);
    printf("Rev C compatibility mode started successfully. Streams running in background.\n");
  }
  
  return 0;
}
