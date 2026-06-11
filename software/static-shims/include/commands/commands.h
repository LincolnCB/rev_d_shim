#ifndef COMMANDS_COMMANDS_H
#define COMMANDS_COMMANDS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define COMMAND_FILE_PATH_MAX 256

typedef enum {
	CMD_NONE = 0,
	CMD_LOAD,
	CMD_CALIBRATE,
	CMD_TRIGGER,
	CMD_RESET,
	CMD_ZERO,
	CMD_READ,
	CMD_SET_CHANNEL,
	CMD_POWER_ON,
	CMD_HARD_RESET,
	CMD_EXIT_FILE,
	CMD_STATUS,
	CMD_HELP,
	CMD_QUIT,
	CMD_INVALID
} command_type_t;

typedef struct {
	command_type_t type;
	int channel;
	double amps;
	uint32_t trigger_count;
	char file_path[COMMAND_FILE_PATH_MAX];
} parsed_command_t;

typedef struct {
	bool running;
	uint32_t channel_count;
	bool file_loaded;
	bool power_enabled;
	bool outputs_zeroed;
	uint32_t trigger_counter;
	char loaded_file[COMMAND_FILE_PATH_MAX];
	char last_file[COMMAND_FILE_PATH_MAX];
} shim_runtime_state_t;

shim_runtime_state_t commands_init_state(uint32_t channel_count);
void commands_print_help(void);
void commands_print_status(const shim_runtime_state_t *state);
bool commands_execute(const parsed_command_t *cmd, shim_runtime_state_t *state);

#endif // COMMANDS_COMMANDS_H
