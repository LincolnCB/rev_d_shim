#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "command_handler.h"

/**
 * Command Table
 * 
 * This table defines all available commands in the system. Each entry contains:
 * - Command name string (what the user types)
 * - Function pointer to the command handler
 * - Command metadata including:
 *   - min_args: minimum number of required arguments (excluding command name)
 *   - max_args: maximum number of allowed arguments
 *   - valid_flags: array of flags this command accepts (terminated by -1)
 *   - description: help text shown to the user
 * 
 * The table is processed sequentially, so command lookup is O(n). For better
 * performance with many commands, consider using a hash table or binary search.
 * 
 * To add a new command:
 * 1. Add the command handler function declaration to command_handler.h
 * 2. Implement the command handler function in this file
 * 3. Add an entry to this table before the sentinel (NULL entry)
 * 4. The help system will automatically generate help text from the metadata
 */
static command_entry_t command_table[] = {
  // Basic system commands (no arguments)
  {"help", cmd_help, {"help", 0, 0, {-1}, "Show this help message"}},
  {"verbose", cmd_verbose, {"verbose", 0, 0, {-1}, "Toggle verbose mode"}},
  {"on", cmd_on, {"on", 0, 0, {-1}, "Turn the system on"}},
  {"off", cmd_off, {"off", 0, 0, {-1}, "Turn the system off"}},
  {"sts", cmd_sts, {"sts", 0, 0, {-1}, "Show hardware manager status"}},
  {"dbg", cmd_dbg, {"dbg", 0, 0, {-1}, "Show debug registers"}},
  {"exit", cmd_exit, {"exit", 0, 0, {-1}, "Exit the program"}},
  
  // Configuration commands (require 1 value argument)
  {"set_boot_test_skip", cmd_set_boot_test_skip, {"set_boot_test_skip", 1, 1, {-1}, "Set boot test skip register to a 16-bit value"}},
  {"set_debug", cmd_set_debug, {"set_debug", 1, 1, {-1}, "Set debug register to a 16-bit value"}},
  {"set_cmd_buf_reset", cmd_set_cmd_buf_reset, {"set_cmd_buf_reset", 1, 1, {-1}, "Set command buffer reset register to a 17-bit value"}},
  {"set_data_buf_reset", cmd_set_data_buf_reset, {"set_data_buf_reset", 1, 1, {-1}, "Set data buffer reset register to a 17-bit value"}},
  
  // FIFO status commands (require 1 board number argument)
  {"dac_cmd_fifo_sts", cmd_dac_cmd_fifo_sts, {"dac_cmd_fifo_sts", 1, 1, {-1}, "Show DAC command FIFO status for specified board (0-7)"}},
  {"dac_data_fifo_sts", cmd_dac_data_fifo_sts, {"dac_data_fifo_sts", 1, 1, {-1}, "Show DAC data FIFO status for specified board (0-7)"}},
  {"adc_cmd_fifo_sts", cmd_adc_cmd_fifo_sts, {"adc_cmd_fifo_sts", 1, 1, {-1}, "Show ADC command FIFO status for specified board (0-7)"}},
  {"adc_data_fifo_sts", cmd_adc_data_fifo_sts, {"adc_data_fifo_sts", 1, 1, {-1}, "Show ADC data FIFO status for specified board (0-7)"}},
  
  // Trigger FIFO status commands (no arguments - triggers are global)
  {"trig_cmd_fifo_sts", cmd_trig_cmd_fifo_sts, {"trig_cmd_fifo_sts", 0, 0, {-1}, "Show trigger command FIFO status"}},
  {"trig_data_fifo_sts", cmd_trig_data_fifo_sts, {"trig_data_fifo_sts", 0, 0, {-1}, "Show trigger data FIFO status"}},
  
  // Data reading commands (require board number for DAC/ADC, support --all flag)
  {"read_dac_data", cmd_read_dac_data, {"read_dac_data", 1, 1, {FLAG_ALL, -1}, "Read raw DAC data sample(s) from specified board (0-7)"}},
  {"read_adc_data", cmd_read_adc_data, {"read_adc_data", 1, 1, {FLAG_ALL, -1}, "Read raw ADC data sample(s) from specified board (0-7)"}},
  
  // Trigger data reading commands (no arguments - triggers are global, support --all flag)
  {"read_trig_data", cmd_read_trig_data, {"read_trig_data", 0, 0, {FLAG_ALL, -1}, "Read trigger data sample(s)"}},
  
  // Debug reading commands (require board number, support --all flag)
  {"read_dac_dbg", cmd_read_dac_dbg, {"read_dac_dbg", 1, 1, {FLAG_ALL, -1}, "Read and print debug information for DAC data from specified board (0-7)"}},
  {"read_adc_dbg", cmd_read_adc_dbg, {"read_adc_dbg", 1, 1, {FLAG_ALL, -1}, "Read and print debug information for ADC data from specified board (0-7)"}},
  
  // Sentinel entry - marks end of table (must be last)
  {NULL, NULL, {NULL, 0, 0, {-1}, NULL}}
};

// Utility function implementations
uint32_t parse_value(const char* str, char** endptr) {
  const char* arg = str;
  while (*arg == ' ' || *arg == '\t') arg++; // Skip leading whitespace
  
  if (strncmp(arg, "0b", 2) == 0) {
    return (uint32_t)strtol(arg + 2, endptr, 2);
  } else {
    return (uint32_t)strtol(arg, endptr, 0); // Handles 0x, decimal, octal
  }
}

int parse_board_number(const char* str) {
  int board = atoi(str);
  if (board < 0 || board > 7) {
    return -1;
  }
  return board;
}

int has_flag(const command_flag_t* flags, int flag_count, command_flag_t target_flag) {
  for (int i = 0; i < flag_count; i++) {
    if (flags[i] == target_flag) {
      return 1;
    }
  }
  return 0;
}

// Command parsing function
int parse_command_line(const char* line, const char** args, int* arg_count, command_flag_t* flags, int* flag_count) {
  static char buffer[256];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  
  *arg_count = 0;
  *flag_count = 0;
  
  char* token = strtok(buffer, " \t");
  while (token != NULL && *arg_count < MAX_ARGS) {
    if (strcmp(token, "--all") == 0) {
      if (*flag_count < MAX_FLAGS) {
        flags[(*flag_count)++] = FLAG_ALL;
      }
    } else {
      args[(*arg_count)++] = token;
    }
    token = strtok(NULL, " \t");
  }
  
  return *arg_count > 0 ? 0 : -1;
}

// Function to find command in table
command_entry_t* find_command(const char* name) {
  for (int i = 0; command_table[i].name != NULL; i++) {
    if (strcmp(command_table[i].name, name) == 0) {
      return &command_table[i];
    }
  }
  return NULL;
}

// Command execution function
int execute_command(const char* line, command_context_t* ctx) {
  const char* args[MAX_ARGS];
  int arg_count;
  command_flag_t flags[MAX_FLAGS];
  int flag_count;
  
  if (parse_command_line(line, args, &arg_count, flags, &flag_count) != 0) {
    return -1;
  }
  
  command_entry_t* cmd = find_command(args[0]);
  if (cmd == NULL) {
    printf("Unknown command: '%s'. Type 'help' for available commands.\n", args[0]);
    return -1;
  }
  
  // Check argument count
  int actual_args = arg_count - 1; // Subtract command name
  if (actual_args < cmd->info.min_args || actual_args > cmd->info.max_args) {
    printf("Command '%s' expects %d-%d arguments, but %d were provided.\n", 
           cmd->name, cmd->info.min_args, cmd->info.max_args, actual_args);
    return -1;
  }
  
  // Validate flags
  for (int i = 0; i < flag_count; i++) {
    int flag_valid = 0;
    for (int j = 0; j < MAX_FLAGS && cmd->info.valid_flags[j] != -1; j++) {
      if (cmd->info.valid_flags[j] == flags[i]) {
        flag_valid = 1;
        break;
      }
    }
    if (!flag_valid) {
      printf("Invalid flag for command '%s'.\n", cmd->name);
      return -1;
    }
  }
  
  // Execute command
  return cmd->handler(&args[1], actual_args, flags, flag_count, ctx);
}

void print_help(void) {
  printf("Available commands:\n");
  printf("\n");
  
  // First pass: commands with no arguments
  printf(" -- No arguments --\n");
  for (int i = 0; command_table[i].name != NULL; i++) {
    if (command_table[i].info.min_args == 0 && command_table[i].info.max_args == 0) {
      printf("  %-20s - %s\n", command_table[i].name, command_table[i].info.description);
    }
  }
  printf("\n");
  
  // Second pass: commands with arguments
  printf(" -- With arguments --\n");
  for (int i = 0; command_table[i].name != NULL; i++) {
    if (command_table[i].info.min_args > 0 || command_table[i].info.max_args > 0) {
      // Build argument string
      char arg_str[256] = "";
      
      // Add required arguments
      for (int j = 0; j < command_table[i].info.min_args; j++) {
        if (strstr(command_table[i].name, "set_") == command_table[i].name) {
          strcat(arg_str, " <value>");
        } else if (strstr(command_table[i].name, "_fifo_sts") != NULL || 
                   (strstr(command_table[i].name, "read_") == command_table[i].name && 
                    strstr(command_table[i].name, "trig") == NULL)) {
          strcat(arg_str, " <board>");
        } else {
          strcat(arg_str, " <arg>");
        }
      }
      
      // Add optional flags
      bool has_all_flag = false;
      for (int j = 0; j < MAX_FLAGS && command_table[i].info.valid_flags[j] != -1; j++) {
        if (command_table[i].info.valid_flags[j] == FLAG_ALL) {
          has_all_flag = true;
          break;
        }
      }
      if (has_all_flag) {
        strcat(arg_str, " [--all]");
      }
      
      printf("  %s%-*s - %s\n", 
             command_table[i].name, 
             (int)(20 - strlen(command_table[i].name)), 
             arg_str, 
             command_table[i].info.description);
      
      // Add special notes for certain commands
      if (strstr(command_table[i].name, "set_") == command_table[i].name) {
        printf("%-24s   (prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")\n", "");
      } else if (strstr(command_table[i].name, "_fifo_sts") != NULL || 
                 (strstr(command_table[i].name, "read_") == command_table[i].name && 
                  strstr(command_table[i].name, "trig") == NULL)) {
        if (strstr(command_table[i].name, "board") || strstr(command_table[i].name, "dac") || 
            strstr(command_table[i].name, "adc")) {
          printf("%-24s   Board number must be 0-7\n", "");
        }
      }
      if (has_all_flag) {
        printf("%-24s   Use --all to read all data currently in the FIFO\n", "");
      }
    }
  }
  printf("\n");
}

// Helper function for printing data words
static void print_data_words(uint32_t data) {
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

// Helper function for printing 64-bit trigger data
static void print_trigger_data(uint64_t data) {
  uint32_t low_word = data & 0xFFFFFFFF;
  uint32_t high_word = (data >> 32) & 0xFFFFFFFF;
  printf("  Low 32 bits:  0x%08" PRIx32 " (%" PRIu32 ")\n", low_word, low_word);
  printf("  High 32 bits: 0x%08" PRIx32 " (%" PRIu32 ")\n", high_word, high_word);
}

// Command handler implementations
int cmd_help(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  print_help();
  return 0;
}

int cmd_verbose(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  *(ctx->verbose) = !*(ctx->verbose);
  printf("Verbose mode is now %s.\n", *(ctx->verbose) ? "enabled" : "disabled");
  return 0;
}

int cmd_on(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_turn_on(ctx->sys_ctrl, *(ctx->verbose));
  printf("System turned on.\n");
  return 0;
}

int cmd_off(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_turn_off(ctx->sys_ctrl, *(ctx->verbose));
  printf("System turned off.\n");
  return 0;
}

int cmd_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Hardware status:\n");
  print_hw_status(sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose)), *(ctx->verbose));
  return 0;
}

int cmd_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Debug registers:\n");
  print_debug_registers(ctx->sys_sts);
  return 0;
}

int cmd_exit(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Exiting program.\n");
  *(ctx->should_exit) = true;
  return 0;
}

int cmd_set_boot_test_skip(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint16_t value = (uint16_t)parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_boot_test_skip: '%s'\n", args[0]);
    return -1;
  }
  sys_ctrl_set_boot_test_skip(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Boot test skip register set to 0x%" PRIx16 "\n", value);
  return 0;
}

int cmd_set_debug(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint16_t value = (uint16_t)parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_debug: '%s'\n", args[0]);
    return -1;
  }
  sys_ctrl_set_debug(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Debug register set to 0x%" PRIx16 "\n", value);
  return 0;
}

int cmd_set_cmd_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_cmd_buf_reset: '%s'\n", args[0]);
    return -1;
  }
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Command buffer reset register set to 0x%" PRIx32 "\n", value);
  return 0;
}

int cmd_set_data_buf_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t value = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for set_data_buf_reset: '%s'\n", args[0]);
    return -1;
  }
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, value, *(ctx->verbose));
  printf("Data buffer reset register set to 0x%" PRIx32 "\n", value);
  return 0;
}

int cmd_dac_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_cmd_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "DAC Command");
  return 0;
}

int cmd_dac_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_data_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "DAC Data");
  return 0;
}

int cmd_adc_cmd_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_cmd_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "ADC Command");
  return 0;
}

int cmd_adc_data_fifo_sts(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_data_fifo_sts: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  uint32_t fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose));
  print_fifo_status(fifo_status, "ADC Data");
  return 0;
}

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

int cmd_read_dac_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_dac_data: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("DAC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from DAC FIFO for board %d...\n", board);
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
      printf("Sample %d - DAC data from board %d: 0x%" PRIx32 "\n", ++count, board, data);
      print_data_words(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
    printf("Read DAC data from board %d: 0x%" PRIx32 "\n", board, data);
    print_data_words(data);
  }
  return 0;
}

int cmd_read_adc_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_data: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from ADC FIFO for board %d...\n", board);
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      printf("Sample %d - ADC data from board %d: 0x%" PRIx32 "\n", ++count, board, data);
      print_data_words(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    printf("Read ADC data from board %d: 0x%" PRIx32 "\n", board, data);
    print_data_words(data);
  }
  return 0;
}

int cmd_read_trig_data(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  uint32_t fifo_status = sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose));
  if (FIFO_PRESENT(fifo_status) == 0) {
    printf("Trigger data FIFO is not present. Cannot read data.\n");
    return -1;
  }

  if (FIFO_STS_WORD_COUNT(fifo_status) < 2) {
    printf("Trigger data FIFO does not have enough words (need at least 2). Cannot read data.\n");
    return -1;
  }

  bool read_all = has_flag(flags, flag_count, FLAG_ALL);

  if (read_all) {
    printf("Reading all data from trigger FIFO...\n");
    int count = 0;
    while (FIFO_STS_WORD_COUNT(sys_sts_get_trig_data_fifo_status(ctx->sys_sts, *(ctx->verbose))) >= 2) {
      uint64_t data = trigger_read(ctx->trigger_ctrl);
      printf("Sample %d - Trigger data: 0x%016" PRIx64 "\n", ++count, data);
      print_trigger_data(data);
      printf("\n");
    }
    printf("Read %d samples total.\n", count);
  } else {
    uint64_t data = trigger_read(ctx->trigger_ctrl);
    printf("Read trigger data: 0x%016" PRIx64 "\n", data);
    print_trigger_data(data);
  }
  return 0;
}

int cmd_read_dac_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_dac_dbg: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC data FIFO for board %d is not present. Cannot read debug data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("DAC data FIFO for board %d is empty. Cannot read debug data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all debug information from DAC FIFO for board %d...\n", board);
    while (!FIFO_STS_EMPTY(sys_sts_get_dac_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
      dac_print_debug(data);
    }
  } else {
    uint32_t data = dac_read(ctx->dac_ctrl, (uint8_t)board);
    printf("Reading one debug sample from DAC FIFO for board %d...\n", board);
    dac_print_debug(data);
  }
  return 0;
}

int cmd_read_adc_dbg(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_dbg: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all debug information from ADC FIFO for board %d...\n", board);
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      adc_print_debug(data);
    }
  } else {
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    printf("Reading one debug sample from ADC FIFO for board %d...\n", board);
    adc_print_debug(data);
  }
  return 0;
}
