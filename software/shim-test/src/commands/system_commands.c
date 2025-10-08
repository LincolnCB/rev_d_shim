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
#include "system_commands.h"
#include "command_helper.h"
#include "experiment_commands.h"
#include "sys_sts.h"
#include "sys_ctrl.h"
#include "spi_clk_ctrl.h"

// Basic system commands
int cmd_verbose(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  *(ctx->verbose) = !*(ctx->verbose);
  printf("Verbose mode %s.\n", *(ctx->verbose) ? "enabled" : "disabled");
  return 0;
}

int cmd_on(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Turning the system on...\n");
  sys_ctrl_turn_on(ctx->sys_ctrl, *(ctx->verbose));
  
  // Wait a bit and check status
  usleep(100000); // 100ms
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  print_hw_status(hw_status, *(ctx->verbose));
  return 0;
}

int cmd_off(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Turning the system off...\n");
  sys_ctrl_turn_off(ctx->sys_ctrl, *(ctx->verbose));
  return 0;
}

int cmd_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  print_hw_status(hw_status, *(ctx->verbose));
  return 0;
}

int cmd_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  print_debug_register(ctx->sys_sts);
  return 0;
}

int cmd_hard_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Performing hard reset...\n");
  
  // Step 1: Cancel all DAC and ADC file streams
  printf("  Step 1: Stopping all active streaming threads\n");
  
  // Stop trigger monitor thread
  cmd_stop_trigger_monitor(NULL, 0, NULL, 0, ctx);
  
  // Stop trigger data streaming if running
  if (ctx->trig_data_stream_running) {
    printf("    Stopping trigger data stream\n");
    ctx->trig_data_stream_stop = true;
    if (pthread_join(ctx->trig_data_stream_thread, NULL) != 0) {
      fprintf(stderr, "Warning: Failed to join trigger data streaming thread\n");
    }
    ctx->trig_data_stream_running = false;
  }
  
  for (int board = 0; board < 8; board++) {
    // Stop DAC streams
    if (ctx->dac_cmd_stream_running[board]) {
      printf("    Stopping DAC command stream for board %d\n", board);
      ctx->dac_cmd_stream_stop[board] = true;
      if (pthread_join(ctx->dac_cmd_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join DAC command streaming thread for board %d\n", board);
      }
      ctx->dac_cmd_stream_running[board] = false;
    }
    
    // Stop ADC streams
    if (ctx->adc_data_stream_running[board]) {
      printf("    Stopping ADC data stream for board %d\n", board);
      ctx->adc_data_stream_stop[board] = true;
      if (pthread_join(ctx->adc_data_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join ADC data streaming thread for board %d\n", board);
      }
      ctx->adc_data_stream_running[board] = false;
    }
    if (ctx->adc_cmd_stream_running[board]) {
      printf("    Stopping ADC command stream for board %d\n", board);
      ctx->adc_cmd_stream_stop[board] = true;
      if (pthread_join(ctx->adc_cmd_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join ADC command streaming thread for board %d\n", board);
      }
      ctx->adc_cmd_stream_running[board] = false;
    }
  }
  
  // Step 2: Reset the set_debug and set_boot_test_skip registers
  printf("  Step 2: Resetting debug and boot_test_skip registers\n");
  sys_ctrl_set_debug(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_boot_test_skip(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(1000); // 1ms
  
  // Step 3: Turn system off
  printf("  Step 3: Turning system off\n");
  sys_ctrl_turn_off(ctx->sys_ctrl, *(ctx->verbose));
  usleep(1000); // 1ms
  
  // Step 4: Set buffer resets to 0x1FFFF
  printf("  Step 4: Setting buffer resets to 0x1FFFF\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  usleep(10000); // 10ms
  
  // Step 5: Set buffer resets to 0
  printf("  Step 5: Setting buffer resets to 0\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(10000); // 10ms
  
  printf("Hard reset completed.\n");
  
  // Show final status
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  print_hw_status(hw_status, *(ctx->verbose));
  return 0;
}

int cmd_exit(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Exiting...\n");
  *(ctx->should_exit) = true;
  return 0;
}

// Configuration commands
int cmd_set_boot_test_skip(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_boot_test_skip: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (value > 0xFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - 65535)\n", value);
    return -1;
  }
  
  sys_ctrl_set_boot_test_skip(ctx->sys_ctrl, (uint16_t)value, *(ctx->verbose));
  printf("Boot test skip register set to 0x%04X (%u).\n", (uint16_t)value, (uint16_t)value);
  return 0;
}

int cmd_set_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_debug: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (value > 0xFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - 65535)\n", value);
    return -1;
  }
  
  sys_ctrl_set_debug(ctx->sys_ctrl, (uint16_t)value, *(ctx->verbose));
  printf("Debug register set to 0x%04X (%u).\n", (uint16_t)value, (uint16_t)value);
  return 0;
}

int cmd_set_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_cmd_buf_reset: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (value > 0x1FFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - 131071)\n", value);
    return -1;
  }
  
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Command buffer reset register set to 0x%05X (%u).\n", value, value);
  return 0;
}

int cmd_set_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_data_buf_reset: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (value > 0x1FFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - 131071)\n", value);
    return -1;
  }
  
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Data buffer reset register set to 0x%05X (%u).\n", value, value);
  return 0;
}

// SPI polarity commands
int cmd_invert_mosi_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_invert_mosi_sck(ctx->sys_ctrl, *(ctx->verbose));
  printf("MOSI SCK polarity inverted.\n");
  return 0;
}

int cmd_invert_miso_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_invert_miso_sck(ctx->sys_ctrl, *(ctx->verbose));
  printf("MISO SCK polarity inverted.\n");
  return 0;
}

// SPI clock frequency commands
int cmd_spi_clk_freq(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t freq_hz = sys_sts_get_spi_clk_freq_hz(ctx->sys_sts, *(ctx->verbose));
  print_spi_clk_freq(freq_hz, *(ctx->verbose));
  return 0;
}

// Integrator configuration commands
int cmd_set_integ_window(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_integ_window: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  sys_ctrl_set_integ_window(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Integrator window register set to 0x%08X (%u).\n", value, value);
  return 0;
}

int cmd_set_integ_average(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_integ_average: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  sys_ctrl_set_integ_threshold_average(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Integrator threshold average register set to 0x%08X (%u).\n", value, value);
  return 0;
}

int cmd_set_integ_enable(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_integ_enable: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  sys_ctrl_set_integ_enable(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Integrator enable register set to 0x%08X (%u).\n", value, value);
  return 0;
}

// Safe buffer reset function that only resets buffers with entries (when system is on)
void safe_buffer_reset(command_context_t* ctx, bool verbose) {
  if (verbose) {
    printf("Performing safe buffer reset...\n");
  }
  
  // Check system status
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, verbose);
  uint32_t system_state = HW_STS_STATE(hw_status);
  bool system_is_off = (system_state != S_RUNNING);
  
  if (verbose) {
    printf("System state: %u %s\n", system_state, system_is_off ? "(OFF)" : "(ON)");
  }
  
  uint32_t cmd_reset_mask = 0;
  uint32_t data_reset_mask = 0;
  
  // Check DAC command and data buffers for boards 0-7
  // DAC command buffers: bits 0, 2, 4, 6, 8, 10, 12, 14
  // DAC data buffers: same bits
  for (int board = 0; board < 8; board++) {
    uint32_t dac_bit = board * 2;  // 0, 2, 4, 6, 8, 10, 12, 14
    
    // Check DAC command buffer
    uint32_t dac_cmd_fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, board, false);
    if (FIFO_PRESENT(dac_cmd_fifo_status)) {
      if (system_is_off || FIFO_STS_WORD_COUNT(dac_cmd_fifo_status) > 0) {
        cmd_reset_mask |= (1U << dac_bit);
        if (verbose) {
          printf("  DAC board %d command buffer: %s - will reset\n", board,
                 (FIFO_STS_WORD_COUNT(dac_cmd_fifo_status) > 0 ? "has entries" : "empty"));
        }
      } else if (verbose) {
        printf("  DAC board %d command buffer: empty - skipping reset\n", board);
      }
    }
    
    // Check DAC data buffer
    uint32_t dac_data_fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, board, false);
    if (FIFO_PRESENT(dac_data_fifo_status)) {
      if (system_is_off || FIFO_STS_WORD_COUNT(dac_data_fifo_status) > 0) {
        data_reset_mask |= (1U << dac_bit);
        if (verbose) {
          printf("  DAC board %d data buffer: %s - will reset\n", board,
                 (FIFO_STS_WORD_COUNT(dac_data_fifo_status) > 0 ? "has entries" : "empty"));
        }
      } else if (verbose) {
        printf("  DAC board %d data buffer: empty - skipping reset\n", board);
      }
    }
  }
  
  // Check ADC command and data buffers for boards 0-7
  // ADC command buffers: bits 1, 3, 5, 7, 9, 11, 13, 15
  // ADC data buffers: same bits
  for (int board = 0; board < 8; board++) {
    uint32_t adc_bit = board * 2 + 1;  // 1, 3, 5, 7, 9, 11, 13, 15
    
    // Check ADC command buffer
    uint32_t adc_cmd_fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false);
    if (FIFO_PRESENT(adc_cmd_fifo_status)) {
      if (system_is_off || FIFO_STS_WORD_COUNT(adc_cmd_fifo_status) > 0) {
        cmd_reset_mask |= (1U << adc_bit);
        if (verbose) {
          printf("  ADC board %d command buffer: %s - will reset\n", board,
                 (FIFO_STS_WORD_COUNT(adc_cmd_fifo_status) > 0 ? "has entries" : "empty"));
        }
      } else if (verbose) {
        printf("  ADC board %d command buffer: empty - skipping reset\n", board);
      }
    }
    
    // Check ADC data buffer
    uint32_t adc_data_fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, false);
    if (FIFO_PRESENT(adc_data_fifo_status)) {
      if (system_is_off || FIFO_STS_WORD_COUNT(adc_data_fifo_status) > 0) {
        data_reset_mask |= (1U << adc_bit);
        if (verbose) {
          printf("  ADC board %d data buffer: %s - will reset\n", board,
                 (FIFO_STS_WORD_COUNT(adc_data_fifo_status) > 0 ? "has entries" : "empty"));
        }
      } else if (verbose) {
        printf("  ADC board %d data buffer: empty - skipping reset\n", board);
      }
    }
  }
  
  // Check trigger command and data buffers - bit 16
  uint32_t trig_cmd_fifo_status = sys_sts_get_trig_cmd_fifo_status(ctx->sys_sts, false);
  if (FIFO_PRESENT(trig_cmd_fifo_status)) {
    if (system_is_off || FIFO_STS_WORD_COUNT(trig_cmd_fifo_status) > 0) {
      cmd_reset_mask |= (1U << 16);
      if (verbose) {
        printf("  Trigger command buffer: %s - will reset\n",
               (FIFO_STS_WORD_COUNT(trig_cmd_fifo_status) > 0 ? "has entries" : "empty"));
      }
    } else if (verbose) {
      printf("  Trigger command buffer: empty - skipping reset\n");
    }
  }
  
  uint32_t trig_data_fifo_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, false);
  if (FIFO_PRESENT(trig_data_fifo_status)) {
    if (system_is_off || FIFO_STS_WORD_COUNT(trig_data_fifo_status) > 0) {
      data_reset_mask |= (1U << 16);
      if (verbose) {
        printf("  Trigger data buffer: %s - will reset\n",
               (FIFO_STS_WORD_COUNT(trig_data_fifo_status) > 0 ? "has entries" : "empty"));
      }
    } else if (verbose) {
      printf("  Trigger data buffer: empty - skipping reset\n");
    }
  }
  
  // Apply the resets
  if (cmd_reset_mask != 0) {
    if (verbose) {
      printf("Setting command buffer reset mask: 0x%05X\n", cmd_reset_mask);
    }
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, cmd_reset_mask, verbose);
    usleep(1000); // 1ms delay
    sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, verbose);
  } else if (verbose) {
    printf("No command buffers need resetting\n");
  }
  
  if (data_reset_mask != 0) {
    if (verbose) {
      printf("Setting data buffer reset mask: 0x%05X\n", data_reset_mask);
    }
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, data_reset_mask, verbose);
    usleep(1000); // 1ms delay
    sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, verbose);
  } else if (verbose) {
    printf("No data buffers need resetting\n");
  }
  
  if (verbose) {
    printf("Safe buffer reset complete.\n");
  }
}
