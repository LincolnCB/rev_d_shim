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

void print_help();

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
      print_help();
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
    } else if (strcmp(command, "dbg") == 0) {
      printf("Debug registers:\n");
      print_debug_registers(&sys_sts);
    } else if (strcmp(command, "exit") == 0) {
      printf("Exiting program.\n");
      break;
    } else if (strncmp(command, "set_boot_test_skip ", 19) == 0) {
      char *endptr;
      uint16_t value;
      // Support "0b" prefix for binary
      char *arg = command + 19;
      while (*arg == ' ' || *arg == '\t') arg++; // Skip leading whitespace
      if (strncmp(arg, "0b", 2) == 0) {
        value = (uint16_t)strtol(arg + 2, &endptr, 2);
      } else {
        value = (uint16_t)strtol(arg, &endptr, 0); // Handles 0x, decimal, octal
      }
      if (*endptr != '\0') {
        fprintf(stderr, "Invalid value for set_boot_test_skip: '%s'\n", command + 19);
      } else {
        sys_ctrl_set_boot_test_skip(&sys_ctrl, value, verbose);
        printf("Boot test skip register set to 0x%" PRIx32 "\n", value);
      }
    } else if (strncmp(command, "set_boot_test_debug ", 20) == 0) {
      char *endptr;
      uint16_t value;
      // Support "0b" prefix for binary
      char *arg = command + 20;
      while (*arg == ' ' || *arg == '\t') arg++; // Skip leading whitespace
      if (strncmp(arg, "0b", 2) == 0) {
        value = (uint16_t)strtol(arg + 2, &endptr, 2);
      } else {
        value = (uint16_t)strtol(arg, &endptr, 0); // Handles 0x, decimal, octal
      }
      if (*endptr != '\0') {
        fprintf(stderr, "Invalid value for set_boot_test_debug: '%s'\n", command + 20);
      } else {
        sys_ctrl_set_boot_test_debug(&sys_ctrl, value, verbose);
        printf("Boot test debug register set to 0x%" PRIx32 "\n", value);
      }
    } else if (strncmp(command, "set_cmd_buf_reset ", 18) == 0) {
      char *endptr;
      uint32_t value;
      // Support "0b" prefix for binary
      char *arg = command + 18;
      while (*arg == ' ' || *arg == '\t') arg++; // Skip leading whitespace
      if (strncmp(arg, "0b", 2) == 0) {
        value = (uint32_t)strtol(arg + 2, &endptr, 2);
      } else {
        value = (uint32_t)strtol(arg, &endptr, 0); // Handles 0x, decimal, octal
      }
      if (*endptr != '\0') {
        fprintf(stderr, "Invalid value for set_cmd_buf_reset: '%s'\n", command + 18);
      } else {
        sys_ctrl_set_cmd_buf_reset(&sys_ctrl, value, verbose);
        printf("Command buffer reset register set to 0x%" PRIx32 "\n", value);
      }
    } else if (strncmp(command, "set_data_buf_reset ", 19) == 0) {
      char *endptr;
      uint32_t value;
      // Support "0b" prefix for binary
      char *arg = command + 19;
      while (*arg == ' ' || *arg == '\t') arg++; // Skip leading whitespace
      if (strncmp(arg, "0b", 2) == 0) {
        value = (uint32_t)strtol(arg + 2, &endptr, 2);
      } else {
        value = (uint32_t)strtol(arg, &endptr, 0); // Handles 0x, decimal, octal
      }
      if (*endptr != '\0') {
        fprintf(stderr, "Invalid value for set_data_buf_reset: '%s'\n", command + 19);
      } else {
        sys_ctrl_set_data_buf_reset(&sys_ctrl, value, verbose);
        printf("Data buffer reset register set to 0x%" PRIx32 "\n", value);
      }
    } else if (strncmp(command, "dac_cmd_fifo_sts ", 17) == 0) {
      int board = atoi(command + 17);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for dac_cmd_fifo_sts: '%s'. Must be 0-7.\n", command + 17);
      } else {
        uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(&sys_sts, (uint8_t)board, verbose);
        print_fifo_status(fifo_status, "DAC Command");
      }
    } else if (strncmp(command, "dac_data_fifo_sts ", 18) == 0) {
      int board = atoi(command + 18);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for dac_data_fifo_sts: '%s'. Must be 0-7.\n", command + 18);
      } else {
        uint32_t fifo_status = sys_sts_get_dac_data_fifo_status(&sys_sts, (uint8_t)board, verbose);
        print_fifo_status(fifo_status, "DAC Data");
      }
    } else if (strncmp(command, "adc_cmd_fifo_sts ", 17) == 0) {
      int board = atoi(command + 17);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for adc_cmd_fifo_sts: '%s'. Must be 0-7.\n", command + 17);
      } else {
        uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(&sys_sts, (uint8_t)board, verbose);
        print_fifo_status(fifo_status, "ADC Command");
      }
    } else if (strncmp(command, "adc_data_fifo_sts ", 18) == 0) {
      int board = atoi(command + 18);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for adc_data_fifo_sts: '%s'. Must be 0-7.\n", command + 18);
      } else {
        uint32_t fifo_status = sys_sts_get_adc_data_fifo_status(&sys_sts, (uint8_t)board, verbose);
        print_fifo_status(fifo_status, "ADC Data");
      }
    } else if (strcmp(command, "trig_cmd_fifo_sts") == 0) {
      uint32_t fifo_status = sys_sts_get_trig_cmd_fifo_status(&sys_sts, verbose);
      print_fifo_status(fifo_status, "Trigger Command");
    } else if (strcmp(command, "trig_data_fifo_sts") == 0) {
      uint32_t fifo_status = sys_sts_get_trig_data_fifo_status(&sys_sts, verbose);
      print_fifo_status(fifo_status, "Trigger Data");
    } else if (strncmp(command, "read_dac_data ", 14) == 0) {
      int board = atoi(command + 14);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for read_dac_data: '%s'. Must be 0-7.\n", command + 14);
      } else {
        if (FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(&sys_sts, (uint8_t)board, verbose))) {
          printf("DAC data FIFO for board %d is empty. Cannot read data.\n", board);
        } else {
          uint32_t data = dac_read(&dac_ctrl, (uint8_t)board);
          printf("Read DAC data from board %d: 0x%" PRIx32 "\n", board, data);
          // Display decimal and binary representations of first and second 16-bit words
          uint16_t word1 = data & 0xFFFF;
          uint16_t word2 = (data >> 16) & 0xFFFF;
          printf("  Word 1: Decimal: %u, Binary: ", word1);
          for (int bit = 15; bit >= 0; bit--) {
            printf("%u", (word1 >> bit) & 1);
          }
          printf("\n");
          printf("  Word 2: Decimal: %u, Binary: ", word2);
          for (int bit = 15; bit >= 0; bit--) {
            printf("%u", (word2 >> bit) & 1);
          }
          printf("\n");
        }
      }
    } else if (strncmp(command, "read_adc_data ", 14) == 0) {
      int board = atoi(command + 14);
      if (board < 0 || board > 7) {
        fprintf(stderr, "Invalid board number for read_adc_data: '%s'. Must be 0-7.\n", command + 14);
      } else {
        if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(&sys_sts, (uint8_t)board, verbose))) {
          printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
        } else {
          uint32_t data = adc_read(&adc_ctrl, (uint8_t)board);
          printf("Read ADC data from board %d: 0x%" PRIx32 "\n", board, data);
          // Display decimal and binary representations of first and second 16-bit words
          uint16_t word1 = data & 0xFFFF;
          uint16_t word2 = (data >> 16) & 0xFFFF;
          printf("  Word 1: Decimal: %u, Binary: ", word1);
          for (int bit = 15; bit >= 0; bit--) {
            printf("%u", (word1 >> bit) & 1);
          }
          printf("\n");
          printf("  Word 2: Decimal: %u, Binary: ", word2);
          for (int bit = 15; bit >= 0; bit--) {
            printf("%u", (word2 >> bit) & 1);
          }
          printf("\n");
        }
      }
    } else {
      printf("Unknown command: '%s'. Type 'help' for available commands.\n", command);
    }
  }

  //////////////////// Cleanup ////////////////////
  printf("Cleaning up and exiting...\n");
  // Cleanup code can be added here if necessary

  return 0; // Exit the program
}

void print_help() {
  printf("Available commands:\n");
  printf("\n");
  printf(" -- No arguments --\n");
  printf("  help - Show this help message\n");
  printf("  verbose - Toggle verbose mode\n");
  printf("  on - Turn the system on\n");
  printf("  off - Turn the system off\n");
  printf("  sts - Show hardware manager status\n");
  printf("  dbg - Show debug registers\n");
  printf("  exit - Exit the program\n");
  printf("\n");
  printf(" -- With arguments --\n");
  printf("  set_boot_test_skip <value>  - Set boot test skip register to a 16-bit value\n");
  printf("                                (prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")\n");
  printf("  set_boot_test_debug <value> - Set boot test debug register to a 16-bit value\n");
  printf("                                (prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")\n");
  printf("  set_cmd_buf_reset <value>   - Set command buffer reset register to a 17-bit value\n");
  printf("                                (prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")\n");
  printf("  set_data_buf_reset <value>  - Set data buffer reset register to a 17-bit value\n");
  printf("                                (prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")\n");
  printf("  dac_cmd_fifo_sts <board>    - Show DAC command FIFO status for specified board (0-7)\n");
  printf("  dac_data_fifo_sts <board>   - Show DAC data FIFO status for specified board (0-7)\n");
  printf("  adc_cmd_fifo_sts <board>    - Show ADC command FIFO status for specified board (0-7)\n");
  printf("  adc_data_fifo_sts <board>   - Show ADC data FIFO status for specified board (0-7)\n");
  printf("  trig_cmd_fifo_sts           - Show trigger command FIFO status\n");
  printf("  trig_data_fifo_sts          - Show trigger data FIFO status\n");
  printf("  read_dac_data <board>       - Read one DAC data sample from specified board (0-7)\n");
  printf("  read_adc_data <board>       - Read one ADC data sample from specified board (0-7)\n");
  printf("\n");
}
