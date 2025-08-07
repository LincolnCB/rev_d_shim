#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

// Include hardware control modules
#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "spi_clk_ctrl.h"
#include "sys_sts.h"
#include "buf_sts.h"
#include "trigger_ctrl.h"

//////////////////// Main ////////////////////
int main(int argc, char *argv[])
{
  //////////////////// 1. Setup ////////////////////
  printf("Rev. C to D One-to-One Test Program\n");
  printf("Setup:\n");

  //// Hardware control structures
  struct sys_ctrl_t sys_ctrl;              // System control and configuration
  struct spi_clk_ctrl_t spi_clk_ctrl;      // SPI clock control interface
  struct sys_sts_t sys_sts;                // System status
  struct buf_sts_t buf_sts;                // FIFO unavailable status
  struct dac_ctrl_array_t dac_ctrl;        // DAC command FIFOs (all boards)
  struct adc_ctrl_array_t adc_ctrl;        // ADC command and data FIFOs (all boards)
  struct trigger_ctrl_t trigger_ctrl;      // Trigger command and data FIFOs

  // Parse optional verbose argument
  bool verbose = false;
  if (argc == 2 && strcmp(argv[1], "--verbose") == 0) {
    verbose = true;
  }

  // Initialize hardware control structures
  printf("Initializing hardware control modules...\n");

  // Initialize control and status modules
  sys_ctrl = create_sys_ctrl(verbose);
  printf("System control module initialized\n");

  spi_clk_ctrl = create_spi_clk_ctrl(verbose);
  printf("SPI clock control module initialized\n");

  sys_sts = create_sys_sts(verbose);
  printf("System status module initialized\n");

  buf_sts = create_buf_sts(verbose);
  printf("Buffer status module initialized\n");

  // Initialize FIFO modules
  dac_ctrl = create_dac_ctrl_array(verbose);
  printf("DAC control modules initialized (8 boards)\n");

  adc_ctrl = create_adc_ctrl_array(verbose);
  printf("ADC control modules initialized (8 boards)\n");

  trigger_ctrl = create_trigger_ctrl(verbose);
  printf("Trigger control module initialized\n");

  printf("Hardware initialization complete.\n");

  //////////////////// 2. Command Loop ////////////////////
  printf("Entering command loop. Type 'help' for available commands.\n");

  char command[256];
  while (true) {
    printf("\n");
    printf("Command> ");
    if (fgets(command, sizeof(command), stdin) == NULL) {
      perror("Error reading command");
      continue;
    }
    printf("\n");

    // Remove trailing newline character
    size_t len = strlen(command);
    if (len > 0 && command[len - 1] == '\n') {
      command[len - 1] = '\0';
    }

    // Process commands
    if (strcmp(command, "help") == 0) {
      printf("Available commands:\n");
      printf("  help - Show this help message\n");
      printf("  verbose - Toggle verbose mode\n");
      printf("  on - Turn the system on\n");
      printf("  off - Turn the system off\n");
      printf("  sts - Show hardware manager status\n");
      printf("  exit - Exit the program\n");
    } else if (strcmp(command, "verbose") == 0) {
      verbose = !verbose;
      printf("Verbose mode is now %s.\n", verbose ? "enabled" : "disabled");
    } else if (strcmp(command, "on") == 0) {
      sys_ctrl_turn_on(&sys_ctrl, verbose);
      printf("System turned on.\n");
    } else if (strcmp(command, "off") == 0) {
      sys_ctrl_turn_off(&sys_ctrl, verbose);
      printf("System turned off.\n");
    } else if (strcmp(command, "sts") == 0) {
      printf("Hardware status:\n");
      print_hw_status(sys_sts_get_hw_status(&sys_sts, verbose), verbose);
    } else if (strcmp(command, "exit") == 0) {
      printf("Exiting program.\n");
      break;
    } else {
      printf("Unknown command: '%s'. Type 'help' for available commands.\n", command);
    }
  }

  //////////////////// Cleanup ////////////////////
  printf("Cleaning up and exiting...\n");
  // Cleanup code can be added here if necessary

  return 0; // Exit the program
}
