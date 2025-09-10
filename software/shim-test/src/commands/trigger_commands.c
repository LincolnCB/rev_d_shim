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
#include "trigger_commands.h"
#include "command_helper.h"
#include "sys_sts.h"
#include "trigger_ctrl.h"

// Trigger FIFO status commands
int cmd_trig_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t fifo_status = sys_sts_get_trig_cmd_fifo_status(ctx->sys_sts, *(ctx->verbose));
  print_fifo_status(fifo_status, "Trigger Command");
  return 0;
}

int cmd_trig_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t fifo_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose));
  print_fifo_status(fifo_status, "Trigger Data");
  return 0;
}

// Trigger data reading commands
int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (FIFO_PRESENT(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose))) == 0) {
    printf("Trigger data FIFO is not present. Cannot read data.\n");
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose)))) {
    printf("Trigger data FIFO is empty. Cannot read data.\n");
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from trigger FIFO...\n");
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose)))) {
      uint64_t data = trigger_read(ctx->trigger_ctrl);
      printf("Sample %d - Trigger data: 0x%" PRIx64 "\n", ++count, data);
      print_trigger_data(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint64_t data = trigger_read(ctx->trigger_ctrl);
    printf("Read trigger data: 0x%" PRIx64 "\n", data);
    print_trigger_data(data);
  }
  return 0;
}

// Basic trigger command operations
int cmd_trig_sync_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_sync_ch(ctx->trigger_ctrl);
  printf("Trigger synchronize channels command sent.\n");
  return 0;
}

int cmd_trig_force_trig(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_force_trig(ctx->trigger_ctrl);
  printf("Trigger force trigger command sent.\n");
  return 0;
}

int cmd_trig_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_cancel(ctx->trigger_ctrl);
  printf("Trigger cancel command sent.\n");
  return 0;
}

// Trigger command operations with value parameters
int cmd_trig_set_lockout(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid lockout cycles for trig_set_lockout: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (cycles == 0 || cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Lockout cycles out of range: %u (valid range: 1 - 536870911)\n", cycles);
    return -1;
  }
  
  trigger_cmd_set_lockout(ctx->trigger_ctrl, cycles);
  printf("Trigger set lockout command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_delay(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid delay cycles for trig_delay: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Delay cycles out of range: %u (valid range: 0 - 536870911)\n", cycles);
    return -1;
  }
  
  trigger_cmd_delay(ctx->trigger_ctrl, cycles);
  printf("Trigger delay command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_expect_ext(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t count = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid count for trig_expect_ext: '%s'. Must be a number.\n", args[0]);
    return -1;
  }
  
  if (count > 0x1FFFFFFF) {
    fprintf(stderr, "Count out of range: %u (valid range: 0 - 536870911)\n", count);
    return -1;
  }
  
  trigger_cmd_expect_ext(ctx->trigger_ctrl, count);
  printf("Trigger expect external command sent with count %u.\n", count);
  return 0;
}
