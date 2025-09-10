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
  print_debug_registers(ctx->sys_sts);
  return 0;
}

int cmd_hard_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Performing hard reset...\n");
  
  // Step 1: Cancel all DAC and ADC file streams
  printf("  Step 1: Stopping all active streaming threads\n");
  for (int board = 0; board < 8; board++) {
    // Stop DAC streams
    if (ctx->dac_stream_running[board]) {
      printf("    Stopping DAC stream for board %d\n", board);
      ctx->dac_stream_stop[board] = true;
      if (pthread_join(ctx->dac_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join DAC streaming thread for board %d\n", board);
      }
      ctx->dac_stream_running[board] = false;
    }
    
    // Stop ADC streams
    if (ctx->adc_stream_running[board]) {
      printf("    Stopping ADC stream for board %d\n", board);
      ctx->adc_stream_stop[board] = true;
      if (pthread_join(ctx->adc_stream_threads[board], NULL) != 0) {
        fprintf(stderr, "Warning: Failed to join ADC streaming thread for board %d\n", board);
      }
      ctx->adc_stream_running[board] = false;
    }
  }
  
  // Step 2: Reset the set_debug and set_boot_test_skip registers
  printf("  Step 2: Resetting debug and boot_test_skip registers\n");
  sys_ctrl_set_debug(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_boot_test_skip(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(10000); // 10ms
  
  // Step 3: Turn system off
  printf("  Step 3: Turning system off\n");
  sys_ctrl_turn_off(ctx->sys_ctrl, *(ctx->verbose));
  usleep(100000); // 100ms
  
  // Step 4: Set buffer resets to 0x1FFFF
  printf("  Step 4: Setting buffer resets to 0x1FFFF\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  usleep(100000); // 100ms
  
  // Step 5: Set buffer resets to 0
  printf("  Step 5: Setting buffer resets to 0\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  usleep(100000); // 100ms
  
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
