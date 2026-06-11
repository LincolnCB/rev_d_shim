#include "commands.h"

#include <stdio.h>
#include <string.h>

// Build a default runtime state for a fixed channel count.
shim_runtime_state_t commands_init_state(uint32_t channel_count) {
	shim_runtime_state_t state;
	state.running = true;
	state.channel_count = channel_count;
	state.file_loaded = false;
	state.power_enabled = false;
	state.outputs_zeroed = true;
	state.trigger_counter = 0U;
	state.loaded_file[0] = '\0';
	state.last_file[0] = '\0';
	return state;
}

// Print all supported interactive commands.
void commands_print_help(void) {
	printf("Commands:\n");
	printf("  L [file] : Load shim block file (or previous file if omitted).\n");
	printf("  C        : Run calibration (only when no file is loaded).\n");
	printf("  T [n]    : Trigger next shim row n times (default n=1).\n");
	printf("  R        : Reset trigger counter to start of block.\n");
	printf("  Z        : Zero all shim currents and reset trigger counter.\n");
	printf("  I [file] : Read currents from ADC and optionally dump to file.\n");
	printf("  n x      : Set channel n to x amperes (manual mode only).\n");
	printf("  P        : Power amplifier system on.\n");
	printf("  F        : Hard reset, power off, unload file.\n");
	printf("  E        : Exit loaded file and reset trigger counter.\n");
	printf("  S        : Print status.\n");
	printf("  H or ?   : Show this help text.\n");
	printf("  Q        : Quit program.\n");
}

// Print current runtime state for quick diagnostics.
void commands_print_status(const shim_runtime_state_t *state) {
	if (state == NULL) {
		return;
	}

	printf("Status:\n");
	printf("  power_enabled  : %s\n", state->power_enabled ? "yes" : "no");
	printf("  channel_count  : %u\n", state->channel_count);
	printf("  file_loaded    : %s\n", state->file_loaded ? "yes" : "no");
	printf("  outputs_zeroed : %s\n", state->outputs_zeroed ? "yes" : "no");
	printf("  trigger_count  : %u\n", state->trigger_counter);
	printf("  loaded_file    : %s\n", state->loaded_file[0] != '\0' ? state->loaded_file : "(none)");
	printf("  last_file      : %s\n", state->last_file[0] != '\0' ? state->last_file : "(none)");
}

// Load a shim block file into the active playback buffer.
static bool run_load(const parsed_command_t *cmd, shim_runtime_state_t *state) {
	const char *requested = cmd->file_path;
	char selected_file[COMMAND_FILE_PATH_MAX] = {0};

	if (requested[0] != '\0') {
		(void)snprintf(selected_file, sizeof(selected_file), "%s", requested);
	} else if (state->last_file[0] != '\0') {
		(void)snprintf(selected_file, sizeof(selected_file), "%s", state->last_file);
	} else {
		fprintf(stderr, "L requires a file path the first time it is used.\n");
		return false;
	}

	// Hook point: clear command/data buffers and load converted DAC data from selected_file.
	printf("Loading shim block file: %s\n", selected_file);
	printf("Old shim buffer contents replaced.\n");

	state->file_loaded = true;
	state->outputs_zeroed = false;
	state->trigger_counter = 0U;
	(void)snprintf(state->loaded_file, sizeof(state->loaded_file), "%s", selected_file);
	(void)snprintf(state->last_file, sizeof(state->last_file), "%s", selected_file);
	return true;
}

// Run calibration when not in loaded-file playback mode.
static bool run_calibrate(const shim_runtime_state_t *state) {
	if (state->file_loaded) {
		fprintf(stderr, "C is disabled while a file is loaded. Use E first.\n");
		return false;
	}
	// Hook point: run board/channel calibration routine and report failures.
	printf("Running shim channel calibration...\n");
	printf("Calibration complete (placeholder).\n");
	return true;
}

// Advance through loaded shim rows by issuing trigger events.
static bool run_trigger(const parsed_command_t *cmd, shim_runtime_state_t *state) {
	if (!state->file_loaded) {
		fprintf(stderr, "T requires a loaded file.\n");
		return false;
	}
	// Hook point: issue trigger(s) into hardware command pipeline.
	printf("Issuing %u trigger(s).\n", cmd->trigger_count);
	state->trigger_counter += cmd->trigger_count;
	return true;
}

// Reset the trigger counter back to the block start.
static bool run_reset(shim_runtime_state_t *state) {
	// Hook point: reset master trigger counter in hardware.
	state->trigger_counter = 0U;
	printf("Trigger counter reset to start of block.\n");
	return true;
}

// Zero all outputs and reset trigger position.
static bool run_zero(shim_runtime_state_t *state) {
	// Hook point: immediately zero all channels in DAC pipeline.
	printf("Zeroing all shim currents immediately.\n");
	state->outputs_zeroed = true;
	return run_reset(state);
}

// Read back current values, optionally to a dump file.
static bool run_read(const parsed_command_t *cmd) {
	// Hook point: read channel currents from ADCs.
	printf("Reading shim currents from ADC (placeholder values).\n");
	if (cmd->file_path[0] != '\0') {
		// Hook point: write sampled values to the requested file.
		printf("Writing ADC current dump to: %s\n", cmd->file_path);
	}
	return true;
}

// Apply one manual channel update in amps.
static bool run_set_channel(const parsed_command_t *cmd, shim_runtime_state_t *state) {
	if (state->file_loaded) {
		fprintf(stderr, "Manual channel set (n x) is disabled while a file is loaded. Use E first.\n");
		return false;
	}
	if (cmd->channel < 0) {
		fprintf(stderr, "Channel index must be >= 0.\n");
		return false;
	}

	// Hook point: convert amps to DAC units then apply only to one channel.
	printf("Setting channel %d to %.6f A.\n", cmd->channel, cmd->amps);
	state->outputs_zeroed = false;
	return true;
}

// Enable amplifier system power.
static bool run_power_on(shim_runtime_state_t *state) {
	if (state->power_enabled) {
		printf("Amplifier system is already on.\n");
		return true;
	}
	// Hook point: assert board/system power enable sequence.
	printf("Powering amplifier system on.\n");
	state->power_enabled = true;
	return true;
}

// Hard reset system state and unload active file mode.
static bool run_hard_reset(shim_runtime_state_t *state) {
	// Hook point: perform hard system reset and clear FPGA/software buffers.
	printf("Performing hard reset: power off and unload file.\n");
	state->power_enabled = false;
	state->file_loaded = false;
	state->outputs_zeroed = true;
	state->trigger_counter = 0U;
	state->loaded_file[0] = '\0';
	return true;
}

// Exit loaded-file mode while preserving last-file history.
static bool run_exit_file(shim_runtime_state_t *state) {
	if (!state->file_loaded) {
		printf("No file is currently loaded.\n");
		state->trigger_counter = 0U;
		return true;
	}

	// Hook point: cancel active block playback while preserving last_file.
	printf("Exiting loaded file and resetting trigger counter.\n");
	state->file_loaded = false;
	state->trigger_counter = 0U;
	state->loaded_file[0] = '\0';
	return true;
}

// Dispatch a parsed command into runtime behavior.
bool commands_execute(const parsed_command_t *cmd, shim_runtime_state_t *state) {
	if (cmd == NULL || state == NULL) {
		return false;
	}

	switch (cmd->type) {
		case CMD_LOAD:
			return run_load(cmd, state);
		case CMD_CALIBRATE:
			return run_calibrate(state);
		case CMD_TRIGGER:
			return run_trigger(cmd, state);
		case CMD_RESET:
			return run_reset(state);
		case CMD_ZERO:
			return run_zero(state);
		case CMD_READ:
			return run_read(cmd);
		case CMD_SET_CHANNEL:
			return run_set_channel(cmd, state);
		case CMD_POWER_ON:
			return run_power_on(state);
		case CMD_HARD_RESET:
			return run_hard_reset(state);
		case CMD_EXIT_FILE:
			return run_exit_file(state);
		case CMD_STATUS:
			commands_print_status(state);
			return true;
		case CMD_HELP:
			commands_print_help();
			return true;
		case CMD_QUIT:
			state->running = false;
			return true;
		case CMD_NONE:
			return true;
		case CMD_INVALID:
		default:
			fprintf(stderr, "Invalid command.\n");
			return false;
	}
}
