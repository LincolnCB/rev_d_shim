#ifndef COMMANDS_COMMANDS_H
#define COMMANDS_COMMANDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware.h"
#include "file_handling.h"

#define COMMAND_FILE_PATH_MAX 256

typedef enum {
  CMD_NONE = 0,
  CMD_HELP,
  CMD_QUIT,
  CMD_STATUS,
  CMD_POWER_ON,
  CMD_HARD_RESET,
  CMD_ZERO,
  CMD_READ,
  CMD_SET_CHANNEL,
  CMD_UPDATE,
  CMD_BUFFER,
  CMD_CALIBRATE,
  CMD_LOCKOUT,
  CMD_LOAD,
  CMD_EXIT_FILE,
  CMD_RESET,
  CMD_TRIGGER,
  CMD_INVALID
} command_type_t;

typedef struct {
  command_type_t type;
  int channel;
  double amps;
  uint32_t trigger_count;
  char file_path[COMMAND_FILE_PATH_MAX];
  double update_amps[HW_MAX_CHANNELS];
  uint32_t update_count;
  double trigger_lockout_ms;
} parsed_command_t;

typedef struct {
  hw_t *hw;
  bool verbose;
  bool running;
  char last_file[COMMAND_FILE_PATH_MAX];
  double trigger_lockout_ms;
  file_loader_t loader;
} shim_runtime_state_t;

shim_runtime_state_t commands_init_state(hw_t *hw, bool verbose);
void commands_cleanup_state(shim_runtime_state_t *state);
void commands_print_help(void);
void commands_print_status(shim_runtime_state_t *state);
bool commands_execute(const parsed_command_t *cmd, shim_runtime_state_t *state);

#endif // COMMANDS_COMMANDS_H
