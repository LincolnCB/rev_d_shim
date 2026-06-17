#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware.h"
#include "commands.h"
#include "input.h"

// Global pointer to hardware state so the signal handler can reach it
static hw_t *g_hw = NULL;

// Handle Ctrl+C by powering off the hardware and exiting cleanly
static void handle_sigint(int sig) {
  (void)sig;
  printf("\nCaught SIGINT — powering off...\n");
  if (g_hw != NULL) {
    hw_power_off(g_hw);
  }
  exit(1);
}

// Entry point for interactive static-shims command interface
int main(int argc, char **argv) {
  bool verbose = false;
  const char *channel_count_arg = NULL;

  // Capture verbose flag and channel count argument
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (channel_count_arg == NULL) {
      channel_count_arg = argv[i];
    } else {
      fprintf(stderr, "Usage: %s [--verbose] <channel_count>\n", argv[0]);
      return 1;
    }
  }

  // Validate channel count argument
  if (channel_count_arg == NULL) {
    fprintf(stderr, "Usage: %s [--verbose] <channel_count>\n", argv[0]);
    return 1;
  }

  // Parse channel count
  char *end = NULL;
  uint32_t parsed_count = (uint32_t)strtoul(channel_count_arg, &end, 10);
  if (end == channel_count_arg || *end != '\0') {
    fprintf(stderr, "Invalid channel_count '%s'. Expected a positive integer.\n", channel_count_arg);
    return 1;
  }

  // Validate channel count range
  if (parsed_count < 1 || parsed_count > HW_MAX_CHANNELS) {
    fprintf(stderr, "Channel count '%u' is out of supported range (1-%u).\n", parsed_count, HW_MAX_CHANNELS);
    return 1;
  }

  // Register Ctrl+C handler before touching hardware
  signal(SIGINT, handle_sigint);

  // Boot sequence: init -> set_clk -> power_on
  hw_t hw = hw_init(parsed_count, verbose);
  g_hw = &hw;

  if (hw_set_clk(&hw) != 0) {
    fprintf(stderr, "Error: clock configuration failed.\n");
    hw_power_off(&hw);
    return 1;
  }

  shim_runtime_state_t state = commands_init_state(&hw, verbose);

  printf("Static shims command interface\n");
  printf("NOTE: THIS IS NOT ACTUALLY WORKING YET, DO NOT USE THIS FOR ANYTHING OTHER THAN DEVELOPMENT TESTING.\n");
  printf("Configured channels: %u\n", parsed_count);
  printf("Type H for help. Type Q to quit.\n\n");

  char line[INPUT_LINE_MAX];
  char error_buf[128];

  state.running = true;

  while (state.running) {
    printf("static-shims> ");
    fflush(stdout);

    if (fgets(line, sizeof(line), stdin) == NULL) {
      printf("\nExiting (EOF).\n");
      break;
    }

    size_t len = strlen(line);
    if (len > 0U && line[len - 1U] == '\n') {
      line[len - 1U] = '\0';
    }

    parsed_command_t cmd;
    input_parse_result_t parse_result = input_parse_line(line, &cmd, &state, error_buf, sizeof(error_buf));
    if (parse_result == INPUT_PARSE_EMPTY) {
      continue;
    }
    if (parse_result == INPUT_PARSE_ERROR) {
      fprintf(stderr, "Parse error: %s\n", error_buf);
      continue;
    }

    (void)commands_execute(&cmd, &state);
  }

  commands_cleanup_state(&state);
  hw_power_off(&hw);

  return 0;
}
