#include "commands.h"

#include <stdio.h>
#include <string.h>

// Build a default runtime state for a fixed channel count
shim_runtime_state_t commands_init_state(hw_t *hw, bool verbose) {
  shim_runtime_state_t state;
  state.hw = hw;
  state.verbose = verbose;
  state.running = true;
  state.last_file[0] = '\0';
  state.trigger_lockout_ms = 10.0;
  state.loader = file_loader_init(hw, 10.0, verbose);
  return state;
}

// Stop any in-flight file loader and release associated resources
void commands_cleanup_state(shim_runtime_state_t *state) {
  if (state == NULL) {
    return;
  }
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }
  file_loader_destroy(&state->loader);
}

//// -- General commands --

// Print all supported interactive commands
void commands_print_help(void) {
  printf("Commands:\n");
  printf("  -- General commands --\n");
  printf("  H or ?     : Show this help text.\n");
  printf("  Q          : Quit program.\n");
  printf("  S          : Print status.\n");
  printf("  P          : Power amplifier system on.\n");
  printf("  F          : Hard reset, power off, unload file.\n");
  printf("  Z          : Zero all shim currents, reset file.\n");
  printf("  I [file]   : Read currents from ADC (optionally dump to file).\n");
  printf(" -- Manual mode (no loaded file) --\n");
  printf("  n x        : Set channel n to x amperes.\n");
  printf("  U x1 x2... : Update all channels to given amp values.\n");
  printf("  C          : Run calibration (only when no file is loaded).\n");
  printf("  D t        : Set trigger lockout time in ms (default 10.0).\n");
  printf(" -- Loaded file mode (after L command) --\n");
  printf("  L [file]   : Load shim block file (or previous file if omitted).\n");
  printf("  E          : Exit loaded file and reset trigger counter.\n");
  printf("  R          : Reset trigger counter to start of block.\n");
  printf("  T [n]      : Trigger next shim row n times (default n=1).\n");
}

// Print current runtime state for quick diagnostics
void commands_print_status(shim_runtime_state_t *state) {
  if (state == NULL) {
    return;
  }
  
  printf("Status:\n");
  printf("  Channel count          : %u\n", state->hw->channel_count);
  // File status
  file_loader_status_t loader_status = file_loader_get_status(&state->loader);
  if (loader_status == FILE_LOADER_EMPTY) {
    printf("  No file currently loaded.\n");
  } else if (loader_status == FILE_LOADER_LOADED) {
    printf("  Loaded file            : %s\n", state->loader.path);
    printf("  Trigger count          : %u\n", hw_get_trigger_count(state->hw));
  } else if (loader_status == FILE_LOADER_ERROR) {
    printf("  ERROR File error       : %s\n", state->loader.path);
  }
  printf("  Last file              : %s\n", state->last_file[0] != '\0' ? state->last_file : "(none)");
  printf("  Trigger lockout        : %.4f ms\n", state->trigger_lockout_ms);
  // Hardware status
  hw_status_summary(state->hw);
}

// Enable amplifier system power
// Reset trigger commands just in case
// If a file is loaded, set lockout and start triggers to begin playback
static bool run_power_on(shim_runtime_state_t *state) {
  if (state == NULL || state->hw == NULL) {
    return false;
  }
  if (hw_running(state->hw)) {
    printf("Hardware is already powered on.\n");
    return true;
  }
  // Reset trigger buffer
  if (hw_clear_trigger_buffers(state->hw) != 0) {
    fprintf(stderr, "P: failed to clear trigger hardware buffers.\n");
    return false;
  }
  // Power on hardware
  printf("Powering amplifier system on.\n");
  if (hw_power_on(state->hw) != 0) {
    fprintf(stderr, "P: failed to power on hardware.\n");
    return false;
  }
  // If a file is loaded, set trigger lockout and start triggers to begin playback
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("File is loaded, starting triggers with lockout %.4f ms.\n", state->trigger_lockout_ms);
    if (hw_set_trigger_lockout(state->hw, state->trigger_lockout_ms) != 0) {
      fprintf(stderr, "P: failed to set trigger lockout.\n");
      return false;
    }
    if (hw_start_triggers(state->hw) != 0) {
      fprintf(stderr, "P: failed to start triggers.\n");
      return false;
    }
  }
  return true;
}

// Hard reset system state and unload active file
static bool run_hard_reset(shim_runtime_state_t *state) {
  if (state == NULL || state->hw == NULL) {
    return false;
  }
  printf("Performing hard reset: power off and unload file.\n");
  // Stop any in-flight file load before resetting hardware state
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }
  // Power off hardware
  hw_power_off(state->hw);
  // Reset all buffers
  hw_clear_dac_buffers(state->hw);
  hw_clear_adc_buffers(state->hw);
  hw_clear_trigger_buffers(state->hw);
  return true;
}

// Zero all outputs and reset trigger position
// If a file is loaded, exit it as well
static bool run_zero(shim_runtime_state_t *state) {
  if (state == NULL) {
    return false;
  }
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("Exiting loaded file.\n");
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }
  printf("Zeroing all shim currents.\n");
  if (hw_zero_dacs(state->hw) != 0) {
    fprintf(stderr, "Z: failed to zero all DAC channels, powering off\n");
    hw_power_off(state->hw);
    return false;
  }
  return true;
}

// Read back current values, optionally to a dump file
static bool run_read(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  if (cmd == NULL) {
    return false;
  }
  if (!hw_running(state->hw)) {
    fprintf(stderr, "Cannot read ADC values because hardware is not powered on.\n");
    return false;
  }
  double adc_values_amps[HW_MAX_CHANNELS] = {0.0};
  if (hw_read_adcs(state->hw, adc_values_amps) != 0) {
    fprintf(stderr, "I: failed to read ADC values from hardware.\n");
    return false;
  }

  if (cmd->file_path[0] != '\0') {
    printf("Writing ADC current dump to: %s\n", cmd->file_path);
    if (file_append_adc_dump(cmd->file_path, adc_values_amps, state->hw->channel_count) != 0) {
      fprintf(stderr, "I: failed to write ADC dump to file %s\n", cmd->file_path);
      return false;
    }
  } else {
    printf("ADC current values:\n");
  }
  for (uint32_t ch = 0; ch < state->hw->channel_count; ch++) {
    printf("  Channel %2u: %+.6f A\n", ch, adc_values_amps[ch]);
  }
  return true;
}

// -- Manual mode (no loaded file) --

// Apply one manual channel update in amps
static bool run_set_channel(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  if (cmd->channel < 0) {
    fprintf(stderr, "Channel index must be >= 0.\n");
    return false;
  }
  if ((uint32_t)cmd->channel >= state->hw->channel_count) {
    fprintf(stderr, "Channel index out of range for this hardware configuration (max channel index is %u).\n", state->hw->channel_count - 1);
    return false;
  }
  if (!hw_running(state->hw)) {
    fprintf(stderr, "Cannot set channel because hardware is not powered on.\n");
    return false;
  }
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("Cancelling in-progress file to apply manual channel update (you can use 'L' to reload it from the beginning).\n");
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }

  // Set the channel to the new value in amps
  printf("Setting channel %d to %.6f A.\n", cmd->channel, cmd->amps);
  if (hw_set_dac_channel(state->hw, cmd->channel, cmd->amps) != 0) {
    fprintf(stderr, "Failed to set channel %d to %.6f A.\n", cmd->channel, cmd->amps);
    return false;
  }

  // Read the currents back to validate
  double readback_amps = 0.0;
  if (hw_read_adc_channel(state->hw, cmd->channel, &readback_amps) != 0) {
    fprintf(stderr, "Failed to read back channel %d after setting it.\n", cmd->channel);
    return false;
  }
  printf("  Channel %2u: %.6f A\n", cmd->channel, readback_amps);

  return true;
}

// Update all active shim channels with a list of new values in amps
// Command format: U v1 v2 v3 ... vn (up to channel count)
// Requires no file loaded and hardware powered on
static bool run_update(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  if (cmd == NULL) {
    return false;
  }
  if (!hw_running(state->hw)) {
    fprintf(stderr, "Cannot update channels because hardware is not powered on.\n");
    return false;
  }
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("Cancelling in-progress file to apply manual update to all channels (you can use 'L' to reload from the beginning).\n");
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }

  // Set each channel to the new value in amps
  printf("Updating all active channels with new values.\n");
  if (hw_set_dacs(state->hw, cmd->update_amps) != 0) {
    fprintf(stderr, "Failed to update all channels with new values.\n");
    return false;
  }

  // Read the currents back to validate
  double adc_values_amps[HW_MAX_CHANNELS] = {0.0};
  if (hw_read_adcs(state->hw, adc_values_amps) != 0) {
    fprintf(stderr, "I: failed to read ADC values from hardware.\n");
    return false;
  }
  printf("ADC current values after update:\n");
  for (uint32_t ch = 0; ch < state->hw->channel_count; ch++) {
    printf("  Channel %2u: %.6f A\n", ch, adc_values_amps[ch]);
  }
    
  return true;
}

// Run calibration when no file is loaded and hardware is powered on
static bool run_calibrate(shim_runtime_state_t *state) {
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("Cannot run calibration while a file is loaded. Please exit the file first using the 'E' command.\n");
    return false;
  }
  if (!hw_running(state->hw)) {
    printf("Cannot run calibration because hardware is not powered on. Please power on the hardware first using the 'P' command.\n");
    return false;
  }
  printf("Running shim channel calibration...\n");
  if (hw_calibrate(state->hw) != 0) {
    fprintf(stderr, "Calibration failed.\n");
    return false;
  }
  printf("Calibration complete.\n");
  return true;
}

// Set the trigger lockout time in milliseconds
static bool run_set_trigger_lockout(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  if (cmd == NULL) {
    return false;
  }
   if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    printf("Cannot set trigger lockout while a file is loaded. Please exit the file first using the 'E' command.\n");
    return false;
  }
  if (cmd->trigger_lockout_ms < 0.0) {
    fprintf(stderr, "Trigger lockout time must be non-negative.\n");
    return false;
  }
  printf("Setting trigger lockout to %.4f ms.\n", cmd->trigger_lockout_ms);
  if (hw_set_trigger_lockout(state->hw, cmd->trigger_lockout_ms) != 0) {
    fprintf(stderr, "Failed to set trigger lockout to %.4f ms.\n", cmd->trigger_lockout_ms);
    return false;
  }
  file_loader_set_trigger_lockout(&state->loader, cmd->trigger_lockout_ms);
  state->trigger_lockout_ms = cmd->trigger_lockout_ms;
  return true;
}

// -- Loaded file mode (after L command) --

// Load a shim block file into the active playback buffer
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

  // If a previous load is still in flight, stop it and wait for it to exit
  // before touching the hardware buffers
  file_loader_status_t prev_status = file_loader_get_status(&state->loader);
  if (prev_status == FILE_LOADER_LOADED) {
    printf("Cancelling in-progress file load...\n");
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }

  // Clear hardware buffers now that no loader thread is active
  if (hw_clear_dac_buffers(state->hw) != 0) {
    fprintf(stderr, "L: failed to clear DAC hardware buffers.\n");
    return false;
  }
  if (hw_clear_adc_buffers(state->hw) != 0) {
    fprintf(stderr, "L: failed to clear ADC hardware buffers.\n");
    return false;
  }
  if (hw_clear_trigger_buffers(state->hw) != 0) {
    fprintf(stderr, "L: failed to clear trigger hardware buffers.\n");
    return false;
  }

  printf("Loading shim block file: %s\n", selected_file);

  // Configure and start the loader thread
  (void)snprintf(state->loader.path, sizeof(state->loader.path), "%s", selected_file);

  if (file_loader_start(&state->loader) != 0) {
    fprintf(stderr, "L: failed to start file loader thread.\n");
    return false;
  }

  // If the hw is powered on, start trigger tracking immediately
  // Otherwise, power_on command will handle trigger tracking
  if (hw_running(state->hw)) {
    printf("Power is already on, starting triggers with lockout %.4f ms.\n", state->trigger_lockout_ms);
    if (hw_reset_triggers(state->hw) != 0) {
      fprintf(stderr, "L: failed to reset triggers after loading file.\n");
      return false;
    }
    if (hw_set_trigger_lockout(state->hw, state->trigger_lockout_ms) != 0) {
      fprintf(stderr, "L: failed to set trigger lockout after loading file.\n");
      return false;
    }
    if (hw_start_triggers(state->hw) != 0) {
      fprintf(stderr, "L: failed to start triggers after loading file.\n");
      return false;
    }
  }
    
  (void)snprintf(state->last_file, sizeof(state->last_file), "%s", selected_file);
  return true;
}

// Exit the current loaded file and reset trigger counter / buffers
static bool run_exit_file(shim_runtime_state_t *state) {
  if (state == NULL) {
    return false;
  }
  if (file_loader_get_status(&state->loader) != FILE_LOADER_LOADED) {
    printf("No file is currently loaded.\n");
    return true;
  }

  // Stop any in-flight file load before exiting file mode
  printf("Exiting loaded file and resetting trigger counter.\n");
  if (file_loader_get_status(&state->loader) == FILE_LOADER_LOADED) {
    file_loader_request_stop(&state->loader);
    file_loader_join(&state->loader, NULL);
  }

  // Clear out buffers and reset trigger counter
  printf("Clearing buffers and resetting trigger counter.\n");
  // If the hardware is running, reset trigger counter
  if (hw_running(state->hw)) {
    if (hw_reset_triggers(state->hw) != 0) {
      fprintf(stderr, "E: failed to reset triggers after exiting file.\n");
      return false;
    }
  // Otherwise, just clear trigger buffer
  } else {
    if (hw_clear_trigger_buffers(state->hw) != 0) {
      fprintf(stderr, "E: failed to clear trigger hardware buffers after exiting file.\n");
      return false;
    }
  }
  // Clear DAC and ADC buffers
  if (hw_clear_dac_buffers(state->hw) != 0) {
    fprintf(stderr, "E: failed to clear DAC hardware buffers after exiting file.\n");
  }
  if (hw_clear_adc_buffers(state->hw) != 0) {
    fprintf(stderr, "E: failed to clear ADC hardware buffers after exiting file.\n");
  }
  return true;
}

// Reset the trigger counter back to the block start
static bool run_reset(shim_runtime_state_t *state) {
  if (state == NULL) {
    return false;
  }
  if (file_loader_get_status(&state->loader) != FILE_LOADER_LOADED) {
    printf("No file is currently loaded.\n");
    return true;
  }
  if(!hw_running(state->hw)) {
    printf("Hardware isn't running, reset is unnecessary.\n");
    return true;
  }

  // Unload the file and reset the buffers and trigger counter
  printf("Resetting trigger counter and buffers to start of file.\n");
  // Unload the file
  file_loader_request_stop(&state->loader);
  file_loader_join(&state->loader, NULL);
  // Clear out buffers and reset trigger counter
  if (hw_reset_triggers(state->hw) != 0) {
    fprintf(stderr, "R: failed to reset triggers after resetting file.\n");
    return false;
  }
  if (hw_clear_dac_buffers(state->hw) != 0) {
    fprintf(stderr, "R: failed to clear DAC hardware buffers after resetting file.\n");
    return false;
  }
  if (hw_clear_adc_buffers(state->hw) != 0) {
    fprintf(stderr, "R: failed to clear ADC hardware buffers after resetting file.\n");
    return false;
  }

  // Reload the file
  (void)snprintf(state->loader.path, sizeof(state->loader.path), "%s", state->last_file);
  if (file_loader_start(&state->loader) != 0) {
    fprintf(stderr, "R: failed to start file loader thread after resetting file.\n");
    return false;
  }
  // Restart trigger tracking
  if (hw_start_triggers(state->hw) != 0) {
    fprintf(stderr, "R: failed to start triggers after resetting file.\n");
    return false;
  }
  printf("File reset successful, triggers restarted.\n");

  return true;
}

// Advance through loaded shim rows by issuing trigger events
static bool run_trigger(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  // Check that the hardware is powered on and a file is loaded before allowing triggers
  if (!hw_running(state->hw)) {
    printf("Cannot issue triggers because hardware is not powered on. Please power on the hardware first using the 'P' command.\n");
    return false;
  }
  if (file_loader_get_status(&state->loader) != FILE_LOADER_LOADED) {
    printf("Cannot issue triggers because no file is loaded. Please load a file first using the 'L' command.\n");
    return false;
  }

  // Clear trigger buffer to stop trigger checking
  if (hw_clear_trigger_buffers(state->hw) != 0) {
    fprintf(stderr, "T: failed to clear trigger hardware buffers.\n");
    return false;
  }

  // Send the requested number of triggers to hardware
  printf("Issuing %u trigger(s).\n", cmd->trigger_count);
  if (hw_force_trigger(state->hw, cmd->trigger_count) != 0) {
    fprintf(stderr, "T: failed to issue triggers to hardware.\n");
    return false;
  }

  return true;
}

// Dispatch a parsed command into runtime behavior
bool commands_execute(const parsed_command_t *cmd, shim_runtime_state_t *state) {
  if (cmd == NULL || state == NULL) {
    return false;
  }

  switch (cmd->type) {
    case CMD_NONE:
      return true;
    case CMD_HELP:
      commands_print_help();
      return true;
    case CMD_QUIT:
      state->running = false;
      return true;
    case CMD_STATUS:
      commands_print_status(state);
      return true;
    case CMD_POWER_ON:
      return run_power_on(state);
    case CMD_HARD_RESET:
      return run_hard_reset(state);
    case CMD_ZERO:
      return run_zero(state);
    case CMD_READ:
      return run_read(cmd, state);
    case CMD_SET_CHANNEL:
      return run_set_channel(cmd, state);
    case CMD_UPDATE:
      return run_update(cmd, state);
    case CMD_CALIBRATE:
      return run_calibrate(state);
    case CMD_LOCKOUT:
      return run_set_trigger_lockout(cmd, state);
    case CMD_LOAD:
      return run_load(cmd, state);
    case CMD_EXIT_FILE:
      return run_exit_file(state);
    case CMD_RESET:
      return run_reset(state);
    case CMD_TRIGGER:
      return run_trigger(cmd, state);
    case CMD_INVALID:
    default:
      fprintf(stderr, "Invalid command.\n");
      return false;
  }
}
