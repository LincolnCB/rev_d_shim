// channel-check.c
//
// One-shot channel health check for the Rev D shim amplifier. Boots the hardware in PIO mode and
// runs a single calibration pass over every configured channel, printing one result line per
// channel with the same markers the interactive calibration uses:
//   ---  connected / linear          -X-  flat near zero (likely disconnected)
//   ~E~  nonlinear (oscillations)     -F-  read failure during calibration
// Calibration drives the DACs and reads the ADCs over the PIO FIFO bridge, so the board is held
// in PIO (datapath_mode cleared); a hardware halt prints the decoded hw_manager status and exits
// non-zero. This reuses static-shims' hardware layer; only the single-pass entry point differs.
//
// Usage: channel-check [--verbose] <channel_count>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include "static_hardware.h"
#include "sys_ctrl.h"

static hw_t *g_hw = NULL;

static void print_usage(const char *prog) {
  printf("Usage: %s [--verbose] <channel_count>\n", prog);
  printf("\n");
  printf("One-shot channel health check. Boots the hardware in PIO mode and runs a single\n");
  printf("calibration pass over every channel, printing one line per channel:\n");
  printf("  ---  connected / linear          -X-  flat near zero (likely disconnected)\n");
  printf("  ~E~  nonlinear (oscillations)     -F-  read failure during calibration\n");
  printf("\n");
  printf("Arguments:\n");
  printf("  channel_count    Number of active shim channels (1-%u).\n", HW_MAX_CHANNELS);
  printf("\n");
  printf("Options:\n");
  printf("  --verbose        Enable detailed logging.\n");
  printf("  -h, --help       Show this help and exit.\n");
}

// Power the hardware off on Ctrl+C so a channel is never left driven.
static void handle_sigint(int sig) {
  (void)sig;
  printf("\nCaught SIGINT -- powering off...\n");
  if (g_hw != NULL) {
    hw_power_off(g_hw);
  }
  exit(1);
}

int main(int argc, char **argv) {
  bool verbose = false;
  const char *channel_count_arg = NULL;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      print_usage(argv[0]);
      return 0;
    } else if (strcmp(argv[i], "--verbose") == 0) {
      verbose = true;
    } else if (channel_count_arg == NULL) {
      channel_count_arg = argv[i];
    } else {
      print_usage(argv[0]);
      return 1;
    }
  }

  if (channel_count_arg == NULL) {
    print_usage(argv[0]);
    return 1;
  }

  char *end = NULL;
  uint32_t parsed_count = (uint32_t)strtoul(channel_count_arg, &end, 10);
  if (end == channel_count_arg || *end != '\0') {
    fprintf(stderr, "Invalid channel_count '%s'. Expected a positive integer.\n", channel_count_arg);
    return 1;
  }
  if (parsed_count < 1 || parsed_count > HW_MAX_CHANNELS) {
    fprintf(stderr, "Channel count '%u' is out of supported range (1-%u).\n", parsed_count, HW_MAX_CHANNELS);
    return 1;
  }

  // Power off on Ctrl+C before touching hardware.
  signal(SIGINT, handle_sigint);

  hw_t hw = hw_init(parsed_count, verbose);
  g_hw = &hw;

  if (hw_set_clk(&hw) != 0) {
    fprintf(stderr, "Error: clock configuration failed.\n");
    hw_power_off(&hw);
    return 1;
  }

  // Calibration is a PIO operation, so clear datapath_mode before power-on (it is locked once the
  // system is running). A stale DMA bit from a prior run would fault the first PIO FIFO access.
  sys_ctrl_set_datapath_mode(&hw.sys_ctrl, 0, verbose);

  if (hw_power_on(&hw) != 0) {
    fprintf(stderr, "Error: failed to power on hardware.\n");
    hw_power_off(&hw);
    return 1;
  }
  if (!hw_running(&hw)) {
    fprintf(stderr, "Error: hardware is not running after power-on.\n");
    hw_power_off(&hw);
    return 1;
  }

  printf("Channel check: one calibration pass over %u channel(s).\n\n", parsed_count);
  int rc = hw_calibrate_iters(&hw, 1);

  hw_power_off(&hw);

  if (rc != 0) {
    fprintf(stderr, "\nChannel check reported a failure (see status above).\n");
    return 1;
  }
  printf("\nChannel check complete.\n");
  return 0;
}
