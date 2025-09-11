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
#include "trigger_ctrl.h"
#include "command_handler.h"

//////////////////// Main ////////////////////
int main(int argc, char *argv[])
{
  //////////////////// 1. Setup ////////////////////
  printf("Rev. C to D One-to-One Test Program\n");
  printf("Setup:\n");

  //// Hardware control structures
  struct sys_ctrl_t sys_ctrl;         // System control and configuration
  struct spi_clk_ctrl_t spi_clk_ctrl; // SPI clock control interface
  struct sys_sts_t sys_sts;           // System status
  struct dac_ctrl_t dac_ctrl;         // DAC command FIFOs (all boards)
  struct adc_ctrl_t adc_ctrl;         // ADC command and data FIFOs (all boards)
  struct trigger_ctrl_t trigger_ctrl; // Trigger command and data FIFOs

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

  // Initialize FIFO modules
  dac_ctrl = create_dac_ctrl(verbose);
  printf("DAC control modules initialized (8 boards)\n");

  adc_ctrl = create_adc_ctrl(verbose);
  printf("ADC control modules initialized (8 boards)\n");

  trigger_ctrl = create_trigger_ctrl(verbose);
  printf("Trigger control module initialized\n");

  printf("Hardware initialization complete.\n");

  // Print help
  print_help();

  //////////////////// 2. Command Loop ////////////////////
  printf("Entering command loop. Type 'help' for available commands.\n");

  // Set up command context
  bool should_exit = false;
  command_context_t cmd_ctx = {
    .sys_ctrl = &sys_ctrl,
    .spi_clk_ctrl = &spi_clk_ctrl,
    .sys_sts = &sys_sts,
    .dac_ctrl = &dac_ctrl,
    .adc_ctrl = &adc_ctrl,
    .trigger_ctrl = &trigger_ctrl,
    .verbose = &verbose,
    .should_exit = &should_exit,
    .adc_data_stream_threads = {0},    // Initialize thread handles to 0
    .adc_data_stream_running = {false}, // Initialize all data streams as not running
    .adc_data_stream_stop = {false},    // Initialize all data stream stop flags as false
    .adc_cmd_stream_running = {false},  // Initialize all command streams as not running
    .adc_cmd_stream_stop = {false},     // Initialize all command stream stop flags as false
    .dac_cmd_stream_running = {false},  // Initialize all DAC command streams as not running
    .dac_cmd_stream_stop = {false},     // Initialize all DAC command stream stop flags as false
    .log_file = NULL,              // Initialize log file as NULL
    .logging_enabled = false       // Initialize logging as disabled
  };

  char command[256];
  while (!should_exit) {
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

    // Skip empty commands
    if (strlen(command) == 0) {
      continue;
    }

    // Execute command
    execute_command(command, &cmd_ctx);
  }

  //////////////////// Cleanup ////////////////////
  printf("Cleaning up and exiting...\n");
  
  // Stop all running ADC streaming threads
  printf("Stopping all ADC streams...\n");
  for (int i = 0; i < 8; i++) {
    if (cmd_ctx.adc_data_stream_running[i]) {
      printf("Stopping ADC data stream for board %d...\n", i);
      cmd_ctx.adc_data_stream_stop[i] = true;
      if (pthread_join(cmd_ctx.adc_data_stream_threads[i], NULL) != 0) {
        fprintf(stderr, "Failed to join ADC data streaming thread for board %d\n", i);
      } else {
        printf("ADC data stream for board %d stopped.\n", i);
      }
    }
    if (cmd_ctx.adc_cmd_stream_running[i]) {
      printf("Stopping ADC command stream for board %d...\n", i);
      cmd_ctx.adc_cmd_stream_stop[i] = true;
      if (pthread_join(cmd_ctx.adc_cmd_stream_threads[i], NULL) != 0) {
        fprintf(stderr, "Failed to join ADC command streaming thread for board %d\n", i);
      } else {
        printf("ADC command stream for board %d stopped.\n", i);
      }
    }
    if (cmd_ctx.dac_cmd_stream_running[i]) {
      printf("Stopping DAC command stream for board %d...\n", i);
      cmd_ctx.dac_cmd_stream_stop[i] = true;
      if (pthread_join(cmd_ctx.dac_cmd_stream_threads[i], NULL) != 0) {
        fprintf(stderr, "Failed to join DAC command streaming thread for board %d\n", i);
      } else {
        printf("DAC command stream for board %d stopped.\n", i);
      }
    }
  }
  
  // Close log file if logging is active
  if (cmd_ctx.logging_enabled && cmd_ctx.log_file != NULL) {
    printf("Closing command log file...\n");
    fclose(cmd_ctx.log_file);
    cmd_ctx.log_file = NULL;
    cmd_ctx.logging_enabled = false;
  }
  
  sys_ctrl_turn_off(&sys_ctrl, verbose);
  printf("System turned off.\n");

  return 0; // Exit the program
}
