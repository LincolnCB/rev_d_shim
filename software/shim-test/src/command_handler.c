#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <pwd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <pthread.h>
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
  {"help", cmd_help, {0, 0, {-1}, "Show this help message"}},
  {"verbose", cmd_verbose, {0, 0, {-1}, "Toggle verbose mode"}},
  {"on", cmd_on, {0, 0, {-1}, "Turn the system on"}},
  {"off", cmd_off, {0, 0, {-1}, "Turn the system off"}},
  {"sts", cmd_sts, {0, 0, {-1}, "Show hardware manager status"}},
  {"dbg", cmd_dbg, {0, 0, {-1}, "Show debug registers"}},
  {"hard_reset", cmd_hard_reset, {0, 0, {-1}, "Perform hard reset: turn the system off, set cmd/data buffer resets to 0x1FFFF, then to 0"}},
  {"exit", cmd_exit, {0, 0, {-1}, "Exit the program"}},
  
  // Configuration commands (require 1 value argument)
  {"set_boot_test_skip", cmd_set_boot_test_skip, {1, 1, {-1}, "Set boot test skip register to a 16-bit value"}},
  {"set_debug", cmd_set_debug, {1, 1, {-1}, "Set debug register to a 16-bit value"}},
  {"set_cmd_buf_reset", cmd_set_cmd_buf_reset, {1, 1, {-1}, "Set command buffer reset register to a 17-bit value"}},
  {"set_data_buf_reset", cmd_set_data_buf_reset, {1, 1, {-1}, "Set data buffer reset register to a 17-bit value"}},
  
  // SPI polarity commands (no arguments)
  {"invert_mosi_clk", cmd_invert_mosi_clk, {0, 0, {-1}, "Invert MOSI SCK polarity register"}},
  {"invert_miso_clk", cmd_invert_miso_clk, {0, 0, {-1}, "Invert MISO SCK polarity register"}},
  
  // FIFO status commands (require 1 board number argument)
  {"dac_cmd_fifo_sts", cmd_dac_cmd_fifo_sts, {1, 1, {-1}, "Show DAC command FIFO status for specified board (0-7)"}},
  {"dac_data_fifo_sts", cmd_dac_data_fifo_sts, {1, 1, {-1}, "Show DAC data FIFO status for specified board (0-7)"}},
  {"adc_cmd_fifo_sts", cmd_adc_cmd_fifo_sts, {1, 1, {-1}, "Show ADC command FIFO status for specified board (0-7)"}},
  {"adc_data_fifo_sts", cmd_adc_data_fifo_sts, {1, 1, {-1}, "Show ADC data FIFO status for specified board (0-7)"}},
  
  // Trigger FIFO status commands (no arguments - triggers are global)
  {"trig_cmd_fifo_sts", cmd_trig_cmd_fifo_sts, {0, 0, {-1}, "Show trigger command FIFO status"}},
  {"trig_data_fifo_sts", cmd_trig_data_fifo_sts, {0, 0, {-1}, "Show trigger data FIFO status"}},
  
  // Data reading commands (require board number for DAC/ADC, support --all flag)
  {"read_dac_data", cmd_read_dac_data, {1, 1, {FLAG_ALL, -1}, "Read raw DAC data sample(s) from specified board (0-7)"}},
  {"read_adc_data", cmd_read_adc_data, {1, 1, {FLAG_ALL, -1}, "Read raw ADC data sample(s) from specified board (0-7)"}},
  
  // Trigger data reading commands (no arguments - triggers are global, support --all flag)
  {"read_trig_data", cmd_read_trig_data, {0, 0, {FLAG_ALL, -1}, "Read trigger data sample(s)"}},
  
  // Debug reading commands (require board number, support --all flag)
  {"read_dac_dbg", cmd_read_dac_dbg, {1, 1, {FLAG_ALL, -1}, "Read and print debug information for DAC data from specified board (0-7)"}},
  {"read_adc_dbg", cmd_read_adc_dbg, {1, 1, {FLAG_ALL, -1}, "Read and print debug information for ADC data from specified board (0-7)"}},
  
  // Trigger command functions (no arguments)
  {"sync_ch", cmd_trig_sync_ch, {0, 0, {-1}, "Send trigger synchronize channels command"}},
  {"force_trig", cmd_trig_force_trig, {0, 0, {-1}, "Send trigger force trigger command"}},
  {"trig_cancel", cmd_trig_cancel, {0, 0, {-1}, "Send trigger cancel command"}},
  
  // Trigger command functions (require 1 value argument with range validation)
  {"trig_set_lockout", cmd_trig_set_lockout, {1, 1, {-1}, "Send trigger set lockout command with cycles (1 - 0x1FFFFFFF)"}},
  {"trig_delay", cmd_trig_delay, {1, 1, {-1}, "Send trigger delay command with cycles (0 - 0x1FFFFFFF)"}},
  {"trig_expect_ext", cmd_trig_expect_ext, {1, 1, {-1}, "Send trigger expect external command with count (0 - 0x1FFFFFFF)"}},
  
  // DAC and ADC command functions (require board, trig_mode, value arguments)
  {"dac_noop", cmd_dac_noop, {3, 3, {FLAG_CONTINUE, -1}, "Send DAC no-op command: <board> <\"trig\"|\"delay\"> <value> [--continue]"}},
  {"adc_noop", cmd_adc_noop, {3, 3, {FLAG_CONTINUE, -1}, "Send ADC no-op command: <board> <\"trig\"|\"delay\"> <value> [--continue]"}},
  
  // DAC and ADC cancel command functions (require board number)
  {"dac_cancel", cmd_dac_cancel, {1, 1, {-1}, "Send DAC cancel command to specified board (0-7)"}},
  {"adc_cancel", cmd_adc_cancel, {1, 1, {-1}, "Send ADC cancel command to specified board (0-7)"}},
  
  // DAC write command functions (require board, 8 channel values, trigger mode, and value)
  {"write_dac_update", cmd_write_dac_update, {11, 11, {FLAG_CONTINUE, -1}, "Send DAC write update command: <board> <ch0> <ch1> <ch2> <ch3> <ch4> <ch5> <ch6> <ch7> <\"trig\"|\"delay\"> <value> [--continue]"}},
  
  // ADC channel order command functions (require board and 8 channel order values)
  {"adc_set_ord", cmd_adc_set_ord, {9, 9, {-1}, "Set ADC channel order: <board> <ord0> <ord1> <ord2> <ord3> <ord4> <ord5> <ord6> <ord7> (each order value must be 0-7)"}},
  
  // ADC simple read command functions (require board, loop count, and delay cycles)
  {"adc_simple_read", cmd_adc_simple_read, {3, 3, {-1}, "Perform simple ADC reads: <board> <loop_count> <delay_cycles> (reads ADC with delay mode)"}},
  {"adc_read", cmd_adc_read, {3, 3, {-1}, "Perform ADC read using loop command: <board> <loop_count> <delay_cycles> (sends loop_next command then single read command)"}},
  
  // ADC file output command functions (require board and file path, support --all flag)
  {"read_adc_to_file", cmd_read_adc_to_file, {2, 2, {FLAG_ALL, -1}, "Read ADC data to file: <board> <file_path> [--all] (converts to signed values, writes one per line)"}},
  
  // ADC streaming command functions (require board and file path)
  {"stream_adc_to_file", cmd_stream_adc_to_file, {2, 2, {-1}, "Start ADC streaming to file: <board> <file_path> (reads 4 words at a time, 8 samples)"}},
  {"stop_adc_stream", cmd_stop_adc_stream, {1, 1, {-1}, "Stop ADC streaming for specified board (0-7)"}},
  
  // DAC streaming command functions (require board and file path, optional loop count)
  {"stream_dac_from_file", cmd_stream_dac_from_file, {2, 3, {-1}, "Start DAC streaming from waveform file: <board> <file_path> [loop_count] (D/T prefix with optional 8 ch values)"}},
  {"stop_dac_stream", cmd_stop_dac_stream, {1, 1, {-1}, "Stop DAC streaming for specified board (0-7)"}},
  
  // Command logging and playback functions (require file path)
  {"log_commands", cmd_log_commands, {1, 1, {-1}, "Start logging commands to file: <file_path>"}},
  {"stop_log", cmd_stop_log, {0, 0, {-1}, "Stop logging commands"}},
  {"load_commands", cmd_load_commands, {1, 1, {-1}, "Load and execute commands from file: <file_path> (0.25s delay between commands)"}},
  
  // Single channel commands (require channel 0-63)
  {"do_dac_wr_ch", cmd_do_dac_wr_ch, {2, 2, {-1}, "Write DAC single channel: <channel> <value> (channel 0-63, board=ch/8, ch=ch%8)"}},
  {"do_adc_rd_ch", cmd_do_adc_rd_ch, {1, 1, {-1}, "Read ADC single channel: <channel> (channel 0-63, board=ch/8, ch=ch%8)"}},
  {"read_adc_single", cmd_read_adc_single, {1, 1, {FLAG_ALL, -1}, "Read single ADC channel data: <channel> (channel 0-63) [--all]"}},
  {"set_and_check", cmd_set_and_check, {2, 2, {-1}, "Set DAC and check ADC: <channel> <value> (channel 0-63, checks buffers, writes DAC, waits 500ms, reads ADC)"}},
  
  // New test commands
  {"channel_test", cmd_channel_test, {2, 2, {-1}, "Set and check current on individual channels: <channel> <value> (channel 0-63)"}},
  {"stream_adc_from_file", cmd_stream_adc_from_file, {2, 3, {FLAG_SIMPLE, -1}, "Start ADC streaming from command file: <board> <file_path> [loop_count] [--simple]"}},
  {"waveform_test", cmd_waveform_test, {0, 0, {-1}, "Interactive waveform test: prompts for DAC/ADC files, loops, output file, and trigger lockout"}},
  
  // Sentinel entry - marks end of table (must be last)
  {NULL, NULL, {0, 0, {-1}, NULL}}
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

// Helper function to clean and expand file paths
void clean_and_expand_path(const char* input_path, char* full_path, size_t full_path_size) {
  const char* rel_path = input_path;
  const char* shim_home_dir = "/home/shim";
  
  // Remove leading and trailing quotation marks
  char cleaned_path[1024];
  strncpy(cleaned_path, rel_path, sizeof(cleaned_path) - 1);
  cleaned_path[sizeof(cleaned_path) - 1] = '\0';
  
  // Remove leading quotes
  if (cleaned_path[0] == '"' || cleaned_path[0] == '\'') {
    memmove(cleaned_path, cleaned_path + 1, strlen(cleaned_path));
  }
  
  // Remove trailing quotes
  size_t len = strlen(cleaned_path);
  if (len > 0 && (cleaned_path[len - 1] == '"' || cleaned_path[len - 1] == '\'')) {
    cleaned_path[len - 1] = '\0';
  }
  
  // Expand path
  if (cleaned_path[0] == '~' && cleaned_path[1] == '/') {
    snprintf(full_path, full_path_size, "%s/%s", shim_home_dir, cleaned_path + 2);
  } else if (cleaned_path[0] == '~' && cleaned_path[1] == '\0') {
    strncpy(full_path, shim_home_dir, full_path_size - 1);
    full_path[full_path_size - 1] = '\0';
  } else if (cleaned_path[0] == '/') {
    strncpy(full_path, cleaned_path, full_path_size - 1);
    full_path[full_path_size - 1] = '\0';
  } else {
    snprintf(full_path, full_path_size, "%s/%s", shim_home_dir, cleaned_path);
  }
}

// Helper function to set file permissions for group read/write access
void set_file_permissions(const char* file_path, bool verbose) {
  // Set permissions to 666 (owner: rw, group: rw, others: rw)
  if (chmod(file_path, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH) != 0) {
    if (verbose) {
      fprintf(stderr, "Warning: Could not set permissions for file '%s': %s\n", 
              file_path, strerror(errno));
    }
  } else if (verbose) {
    printf("Set file permissions to 666 for '%s'\n", file_path);
  }
}

// Command parsing function
int parse_command_line(const char* line, const char** args, int* arg_count, command_flag_t* flags, int* flag_count) {
  static char buffer[256];
  strncpy(buffer, line, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  
  *arg_count = 0;
  *flag_count = 0;
  
  char* token = strtok(buffer, " \t");
  while (token != NULL) {  // Removed the MAX_ARGS constraint to process all tokens
    if (strcmp(token, "--all") == 0) {
      if (*flag_count < MAX_FLAGS) {
        flags[(*flag_count)++] = FLAG_ALL;
      }
    } else if (strcmp(token, "--continue") == 0) {
      if (*flag_count < MAX_FLAGS) {
        flags[(*flag_count)++] = FLAG_CONTINUE;
      }
    } else if (strcmp(token, "--simple") == 0) {
      if (*flag_count < MAX_FLAGS) {
        flags[(*flag_count)++] = FLAG_SIMPLE;
      }
    } else {
      if (*arg_count < MAX_ARGS) {  // Only limit argument count, not total token processing
        args[(*arg_count)++] = token;
      }
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
  
  // Log command if logging is enabled (but don't log the logging commands themselves)
  if (ctx->logging_enabled && ctx->log_file != NULL && 
      strcmp(args[0], "log_commands") != 0 && 
      strcmp(args[0], "stop_log") != 0 && 
      strcmp(args[0], "load_commands") != 0) {
    fprintf(ctx->log_file, "%s\n", line);
    fflush(ctx->log_file);
  }
  
  // Execute command
  return cmd->handler(&args[1], actual_args, flags, flag_count, ctx);
}

// Helper function to print text with line wrapping at 100 characters
static void print_wrapped_line(const char* prefix, const char* text, const char* continuation_indent) {
  char line[512];
  snprintf(line, sizeof(line), "%s%s", prefix, text);
  
  if (strlen(line) <= 100) {
    printf("%s\n", line);
    return;
  }
  
  // Find the best place to break the line (before character 100)
  int break_pos = 100;
  for (int i = 99; i >= 50; i--) {
    if (line[i] == ' ') {
      break_pos = i;
      break;
    }
  }
  
  // Print first part
  char first_part[512];
  strncpy(first_part, line, break_pos);
  first_part[break_pos] = '\0';
  printf("%s\n", first_part);
  
  // Print remaining part with continuation indent
  const char* remaining = line + break_pos + 1; // Skip the space
  while (strlen(remaining) > 0) {
    char continuation_line[512];
    snprintf(continuation_line, sizeof(continuation_line), "%s%s", continuation_indent, remaining);
    
    if (strlen(continuation_line) <= 100) {
      printf("%s\n", continuation_line);
      break;
    } else {
      // Find next break point
      int next_break = 100 - strlen(continuation_indent);
      for (int i = next_break - 1; i >= 20; i--) {
        if (remaining[i] == ' ') {
          next_break = i;
          break;
        }
      }
      
      char next_part[512];
      strncpy(next_part, remaining, next_break);
      next_part[next_break] = '\0';
      printf("%s%s\n", continuation_indent, next_part);
      remaining = remaining + next_break + 1; // Skip the space
    }
  }
}

void print_help(void) {
  printf("Available commands:\n");
  printf("\n");
  
  // First pass: commands with no arguments
  printf(" -- No arguments --\n");
  for (int i = 0; command_table[i].name != NULL; i++) {
    if (command_table[i].info.min_args == 0 && command_table[i].info.max_args == 0) {
      char prefix[64];
      snprintf(prefix, sizeof(prefix), "  %-20s - ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
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
      bool has_continue_flag = false;
      bool has_simple_flag = false;
      for (int j = 0; j < MAX_FLAGS && command_table[i].info.valid_flags[j] != -1; j++) {
        if (command_table[i].info.valid_flags[j] == FLAG_ALL) {
          has_all_flag = true;
        } else if (command_table[i].info.valid_flags[j] == FLAG_CONTINUE) {
          has_continue_flag = true;
        } else if (command_table[i].info.valid_flags[j] == FLAG_SIMPLE) {
          has_simple_flag = true;
        }
      }
      if (has_all_flag) {
        strcat(arg_str, " [--all]");
      }
      if (has_continue_flag) {
        strcat(arg_str, " [--continue]");
      }
      if (has_simple_flag) {
        strcat(arg_str, " [--simple]");
      }
      
      // Create the full command line prefix
      char prefix[512];
      snprintf(prefix, sizeof(prefix), "  %s%-*s - ", 
               command_table[i].name, 
               (int)(20 - strlen(command_table[i].name)), 
               arg_str);
      
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      
      // Add special notes for certain commands
      if (strstr(command_table[i].name, "set_") == command_table[i].name) {
        print_wrapped_line("                         ", 
                          "(prefix binary with \"0b\", octal with \"0\", and hex with \"0x\")",
                          "                         ");
      } else if (strstr(command_table[i].name, "_fifo_sts") != NULL || 
                 (strstr(command_table[i].name, "read_") == command_table[i].name && 
                  strstr(command_table[i].name, "trig") == NULL)) {
        if (strstr(command_table[i].name, "board") || strstr(command_table[i].name, "dac") || 
            strstr(command_table[i].name, "adc")) {
          print_wrapped_line("                         ", "Board number must be 0-7", "                         ");
        }
      }
      if (has_all_flag) {
        print_wrapped_line("                         ", 
                          "Use --all to read all data currently in the FIFO", 
                          "                         ");
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
  // Start hardware manager interrupt monitoring before turning on the system
  if (sys_sts_start_hw_manager_irq_monitor(ctx->sys_sts, *(ctx->verbose)) != 0) {
    fprintf(stderr, "Warning: Failed to start hardware manager interrupt monitoring\n");
  }
  
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

int cmd_hard_reset(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("Performing hard reset sequence...\n");
  
  // Step 1: Cancel all streaming operations
  printf("Cancelling all DAC and ADC streams...\n");
  for (int i = 0; i < 8; i++) {
    if (ctx->dac_stream_running[i]) {
      printf("Stopping DAC stream for board %d...\n", i);
      ctx->dac_stream_stop[i] = true;
      if (pthread_join(ctx->dac_stream_threads[i], NULL) != 0) {
        fprintf(stderr, "Failed to join DAC streaming thread for board %d\n", i);
      }
      ctx->dac_stream_running[i] = false;
    }
    if (ctx->adc_stream_running[i]) {
      printf("Stopping ADC stream for board %d...\n", i);
      ctx->adc_stream_stop[i] = true;
      if (pthread_join(ctx->adc_stream_threads[i], NULL) != 0) {
        fprintf(stderr, "Failed to join ADC streaming thread for board %d\n", i);
      }
      ctx->adc_stream_running[i] = false;
    }
  }
  
  // Step 2: Turn the system off
  printf("Turning system off...\n");
  sys_ctrl_turn_off(ctx->sys_ctrl, *(ctx->verbose));
  
  // Step 3: Reset debug and boot_test_skip registers  
  printf("Resetting debug register to 0...\n");
  sys_ctrl_set_debug(ctx->sys_ctrl, 0, *(ctx->verbose));
  
  printf("Resetting boot_test_skip register to 0...\n");
  sys_ctrl_set_boot_test_skip(ctx->sys_ctrl, 0, *(ctx->verbose));
  
  // Step 4: Set command buffer reset to 0x1FFFF
  printf("Setting command buffer reset to 0x1FFFF...\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  
  // Step 5: Set data buffer reset to 0x1FFFF
  printf("Setting data buffer reset to 0x1FFFF...\n");
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  
  // Step 6: Set command buffer reset back to 0
  printf("Setting command buffer reset to 0...\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  
  // Step 7: Set data buffer reset back to 0
  printf("Setting data buffer reset to 0...\n");
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  
  printf("Hard reset sequence completed.\n");
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

int cmd_invert_mosi_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_invert_mosi_sck(ctx->sys_ctrl, *(ctx->verbose));
  printf("MOSI SCK polarity inverted.\n");
  return 0;
}

int cmd_invert_miso_clk(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  sys_ctrl_invert_miso_sck(ctx->sys_ctrl, *(ctx->verbose));
  printf("MISO SCK polarity inverted.\n");
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

// Trigger command implementations
int cmd_trig_sync_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_sync_ch(ctx->trigger_ctrl);
  printf("Trigger synchronize channels command sent.\n");
  return 0;
}

int cmd_trig_force_trig(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_force_trig(ctx->trigger_ctrl);
  printf("Trigger force trigger command sent.\n");
  return 0;
}

int cmd_trig_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  trigger_cmd_cancel(ctx->trigger_ctrl);
  printf("Trigger cancel command sent.\n");
  return 0;
}

int cmd_trig_set_lockout(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for trig_set_lockout: '%s'\n", args[0]);
    return -1;
  }
  
  if (cycles < 1 || cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Lockout cycles out of range: %u (valid range: 1 - %u)\n", cycles, 0x1FFFFFFF);
    return -1;
  }
  
  trigger_cmd_set_lockout(ctx->trigger_ctrl, cycles);
  printf("Trigger set lockout command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_delay(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t cycles = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for trig_delay: '%s'\n", args[0]);
    return -1;
  }
  
  if (cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Delay cycles out of range: %u (valid range: 0 - %u)\n", cycles, 0x1FFFFFFF);
    return -1;
  }
  
  trigger_cmd_delay(ctx->trigger_ctrl, cycles);
  printf("Trigger delay command sent with %u cycles.\n", cycles);
  return 0;
}

int cmd_trig_expect_ext(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  char* endptr;
  uint32_t count = parse_value(args[0], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for trig_expect_ext: '%s'\n", args[0]);
    return -1;
  }
  
  if (count > 0x1FFFFFFF) {
    fprintf(stderr, "External trigger count out of range: %u (valid range: 0 - %u)\n", count, 0x1FFFFFFF);
    return -1;
  }
  
  trigger_cmd_expect_ext(ctx->trigger_ctrl, count);
  printf("Trigger expect external command sent with count %u.\n", count);
  return 0;
}

// DAC and ADC no-op command implementations
int cmd_dac_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_noop: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC stream is running for this board
  if (ctx->dac_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC no-op command to board %d: DAC stream is currently running. Stop the stream first.\n", board);
    return -1;
  }
  
  // Parse trigger mode
  bool trig;
  if (strcmp(args[1], "trig") == 0) {
    trig = true;
  } else if (strcmp(args[1], "delay") == 0) {
    trig = false;
  } else {
    fprintf(stderr, "Invalid trigger mode for dac_noop: '%s'. Must be 'trig' or 'delay'.\n", args[1]);
    return -1;
  }
  
  // Parse value
  char* endptr;
  uint32_t value = parse_value(args[2], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for dac_noop: '%s'\n", args[2]);
    return -1;
  }
  
  if (value > 0x0FFFFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - %u)\n", value, 0x0FFFFFFF);
    return -1;
  }
  
  // Check for continue flag
  bool cont = has_flag(flags, flag_count, FLAG_CONTINUE);
  
  // Execute DAC no-op command
  dac_cmd_noop(ctx->dac_ctrl, (uint8_t)board, trig, cont, false, value, *(ctx->verbose));
  printf("DAC no-op command sent to board %d with %s mode, value %u%s.\n", 
         board, trig ? "trigger" : "delay", value, cont ? ", continuous" : "");
  return 0;
}

int cmd_adc_noop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_noop: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse trigger mode
  bool trig;
  if (strcmp(args[1], "trig") == 0) {
    trig = true;
  } else if (strcmp(args[1], "delay") == 0) {
    trig = false;
  } else {
    fprintf(stderr, "Invalid trigger mode for adc_noop: '%s'. Must be 'trig' or 'delay'.\n", args[1]);
    return -1;
  }
  
  // Parse value
  char* endptr;
  uint32_t value = parse_value(args[2], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for adc_noop: '%s'\n", args[2]);
    return -1;
  }
  
  if (value > 0x0FFFFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - %u)\n", value, 0x0FFFFFFF);
    return -1;
  }
  
  // Check for continue flag
  bool cont = has_flag(flags, flag_count, FLAG_CONTINUE);
  
  // Execute ADC no-op command
  adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, trig, cont, value, *(ctx->verbose));
  printf("ADC no-op command sent to board %d with %s mode, value %u%s.\n", 
         board, trig ? "trigger" : "delay", value, cont ? ", continuous" : "");
  return 0;
}

// DAC and ADC cancel command implementations
int cmd_dac_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for dac_cancel: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC stream is running for this board
  if (ctx->dac_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC cancel command to board %d: DAC stream is currently running. Stop the stream first.\n", board);
    return -1;
  }
  
  // Execute DAC cancel command
  dac_cmd_cancel(ctx->dac_ctrl, (uint8_t)board, *(ctx->verbose));
  printf("DAC cancel command sent to board %d.\n", board);
  return 0;
}

int cmd_adc_cancel(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_cancel: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Execute ADC cancel command
  adc_cmd_cancel(ctx->adc_ctrl, (uint8_t)board, *(ctx->verbose));
  printf("ADC cancel command sent to board %d.\n", board);
  return 0;
}

int cmd_write_dac_update(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for write_dac_update: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if DAC stream is running for this board
  if (ctx->dac_stream_running[board]) {
    fprintf(stderr, "Cannot send DAC write update command to board %d: DAC stream is currently running. Stop the stream first.\n", board);
    return -1;
  }
  
  // Parse 8 channel values (16-bit signed integers)
  int16_t ch_vals[8];
  for (int i = 0; i < 8; i++) {
    char* endptr;
    long val = strtol(args[i + 1], &endptr, 0);
    if (*endptr != '\0') {
      fprintf(stderr, "Invalid channel %d value for write_dac_update: '%s'\n", i, args[i + 1]);
      return -1;
    }
    if (val < -32767 || val > 32767) {
      fprintf(stderr, "Channel %d value out of range: %ld (valid range: -32767 to 32767)\n", i, val);
      return -1;
    }
    ch_vals[i] = (int16_t)val;
  }
  
  // Parse trigger mode (argument 9: args[9])
  bool trig;
  if (strcmp(args[9], "trig") == 0) {
    trig = true;
  } else if (strcmp(args[9], "delay") == 0) {
    trig = false;
  } else {
    fprintf(stderr, "Invalid trigger mode for write_dac_update: '%s'. Must be 'trig' or 'delay'.\n", args[9]);
    return -1;
  }
  
  // Parse value (argument 10: args[10])
  char* endptr;
  uint32_t value = parse_value(args[10], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid value for write_dac_update: '%s'\n", args[10]);
    return -1;
  }
  
  if (value > 0x0FFFFFFF) {
    fprintf(stderr, "Value out of range: %u (valid range: 0 - %u)\n", value, 0x0FFFFFFF);
    return -1;
  }
  
  // Check for continue flag
  bool cont = has_flag(flags, flag_count, FLAG_CONTINUE);
  
  // Execute DAC write update command with ldac = true
  dac_cmd_dac_wr(ctx->dac_ctrl, (uint8_t)board, ch_vals, trig, cont, true, value, *(ctx->verbose));
  printf("DAC write update command sent to board %d with %s mode, value %u%s.\n", 
         board, trig ? "trigger" : "delay", value, cont ? ", continuous" : "");
  printf("Channel values: [%d, %d, %d, %d, %d, %d, %d, %d]\n",
         ch_vals[0], ch_vals[1], ch_vals[2], ch_vals[3], 
         ch_vals[4], ch_vals[5], ch_vals[6], ch_vals[7]);
  return 0;
}

int cmd_adc_set_ord(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_set_ord: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse 8 channel order values (must be 0-7)
  uint8_t channel_order[8];
  for (int i = 0; i < 8; i++) {
    char* endptr;
    long val = strtol(args[i + 1], &endptr, 0);
    if (*endptr != '\0') {
      fprintf(stderr, "Invalid channel order value for adc_set_ord at position %d: '%s'. Must be a number.\n", i, args[i + 1]);
      return -1;
    }
    if (val < 0 || val > 7) {
      fprintf(stderr, "Invalid channel order value for adc_set_ord at position %d: %ld. Must be 0-7.\n", i, val);
      return -1;
    }
    channel_order[i] = (uint8_t)val;
  }
  
  // Execute ADC set order command
  adc_cmd_set_ord(ctx->adc_ctrl, (uint8_t)board, channel_order, *(ctx->verbose));
  printf("ADC channel order set for board %d: [%d, %d, %d, %d, %d, %d, %d, %d]\n", 
         board, channel_order[0], channel_order[1], channel_order[2], channel_order[3],
         channel_order[4], channel_order[5], channel_order[6], channel_order[7]);
  return 0;
}

int cmd_adc_simple_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_simple_read: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse loop count
  char* endptr;
  long loop_count = strtol(args[1], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid loop count for adc_simple_read: '%s'. Must be a number.\n", args[1]);
    return -1;
  }
  if (loop_count < 1) {
    fprintf(stderr, "Invalid loop count for adc_simple_read: %ld. Must be at least 1.\n", loop_count);
    return -1;
  }
  
  // Parse delay cycles
  long delay_cycles = strtol(args[2], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid delay cycles for adc_simple_read: '%s'. Must be a number.\n", args[2]);
    return -1;
  }
  if (delay_cycles < 0) {
    fprintf(stderr, "Invalid delay cycles for adc_simple_read: %ld. Must be non-negative.\n", delay_cycles);
    return -1;
  }
  if (delay_cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Delay cycles too large for adc_simple_read: %ld. Must be 0 to 536870911 (29-bit value).\n", delay_cycles);
    return -1;
  }
  
  printf("Performing %ld simple ADC reads on board %d (delay mode, value %ld)...\n", loop_count, board, delay_cycles);
  
  // Execute ADC read commands in loop
  for (long i = 0; i < loop_count; i++) {
    adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, (uint32_t)delay_cycles, *(ctx->verbose));
    if (*(ctx->verbose)) {
      printf("ADC read command %ld sent to board %d\n", i + 1, board);
    }
  }
  
  printf("Completed %ld ADC read commands on board %d.\n", loop_count, board);
  return 0;
}

int cmd_adc_read(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for adc_read: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse loop count
  char* endptr;
  long loop_count = strtol(args[1], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid loop count for adc_read: '%s'. Must be a number.\n", args[1]);
    return -1;
  }
  if (loop_count < 1) {
    fprintf(stderr, "Invalid loop count for adc_read: %ld. Must be at least 1.\n", loop_count);
    return -1;
  }
  if (loop_count > 0x1FFFFFF) {
    fprintf(stderr, "Loop count too large for adc_read: %ld. Must be 0 to 33554431 (25-bit value).\n", loop_count);
    return -1;
  }
  
  // Parse delay cycles
  long delay_cycles = strtol(args[2], &endptr, 0);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid delay cycles for adc_read: '%s'. Must be a number.\n", args[2]);
    return -1;
  }
  if (delay_cycles < 0) {
    fprintf(stderr, "Invalid delay cycles for adc_read: %ld. Must be non-negative.\n", delay_cycles);
    return -1;
  }
  if (delay_cycles > 0x1FFFFFFF) {
    fprintf(stderr, "Delay cycles too large for adc_read: %ld. Must be 0 to 536870911 (29-bit value).\n", delay_cycles);
    return -1;
  }
  
  printf("Performing ADC read on board %d using loop command (loop count: %ld, delay mode, value %ld)...\n", board, loop_count, delay_cycles);
  
  // Send loop_next command with the loop count
  adc_cmd_loop_next(ctx->adc_ctrl, (uint8_t)board, (uint32_t)loop_count, *(ctx->verbose));
  
  // Send single ADC read command
  adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, false, false, (uint32_t)delay_cycles, *(ctx->verbose));
  
  printf("ADC read commands sent to board %d: loop_next(%ld) + adc_rd(delay, %ld).\n", board, loop_count, delay_cycles);
  return 0;
}

int cmd_read_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for read_adc_to_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check FIFO presence and status
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[1], full_path, sizeof(full_path));
  
  // Open file for append (create if doesn't exist)
  FILE* file = fopen(full_path, "a");
  if (file == NULL) {
    fprintf(stderr, "Failed to open file '%s' for writing: %s\n", full_path, strerror(errno));
    return -1;
  }
  
  // Set file permissions for group access
  set_file_permissions(full_path, *(ctx->verbose));
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  int samples_written = 0;
  
  if (read_all) {
    printf("Reading all ADC data from board %d and writing to file '%s'...\n", board, full_path);
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose)))) {
      uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
      
      // Split 32-bit data into two 16-bit values
      uint16_t lower_16 = data & 0xFFFF;
      uint16_t upper_16 = (data >> 16) & 0xFFFF;
      
      // Convert from offset to signed
      int16_t signed_lower = ADC_OFFSET_TO_SIGNED(lower_16);
      int16_t signed_upper = ADC_OFFSET_TO_SIGNED(upper_16);
      
      // Write to file (one value per line)
      fprintf(file, "%d\n", signed_lower);
      fprintf(file, "%d\n", signed_upper);
      
      samples_written++;
      if (*(ctx->verbose)) {
        printf("Sample %d written: %d, %d\n", samples_written, signed_lower, signed_upper);
      }
    }
    printf("Wrote %d samples (%d values) to file '%s'.\n", samples_written, samples_written * 2, full_path);
  } else {
    printf("Reading one ADC sample from board %d and writing to file '%s'...\n", board, full_path);
    uint32_t data = adc_read(ctx->adc_ctrl, (uint8_t)board);
    
    // Split 32-bit data into two 16-bit values
    uint16_t lower_16 = data & 0xFFFF;
    uint16_t upper_16 = (data >> 16) & 0xFFFF;
    
    // Convert from offset to signed
    int16_t signed_lower = ADC_OFFSET_TO_SIGNED(lower_16);
    int16_t signed_upper = ADC_OFFSET_TO_SIGNED(upper_16);
    
    // Write to file (one value per line)
    fprintf(file, "%d\n", signed_lower);
    fprintf(file, "%d\n", signed_upper);
    
    printf("Wrote 1 sample (2 values: %d, %d) to file '%s'.\n", signed_lower, signed_upper, full_path);
  }
  
  fclose(file);
  return 0;
}

// Structure to pass data to the streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
} adc_stream_data_t;

// Thread function for ADC streaming
void* adc_stream_thread(void* arg) {
  adc_stream_data_t* stream_data = (adc_stream_data_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  const char* file_path = stream_data->file_path;
  volatile bool* should_stop = stream_data->should_stop;
  
  // Open file for append (create if doesn't exist)
  FILE* file = fopen(file_path, "a");
  if (file == NULL) {
    fprintf(stderr, "ADC Stream Thread[%d]: Failed to open file '%s' for writing: %s\n", 
            board, file_path, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(stream_data);
    return NULL;
  }
  
  // Set file permissions for group access
  set_file_permissions(file_path, false); // Don't show verbose messages in thread
  
  printf("ADC Stream Thread[%d]: Started streaming to file '%s'\n", board, file_path);
  
  int total_samples = 0;
  
  while (!(*should_stop)) {
    // Check FIFO status
    uint32_t fifo_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, false);
    
    if (FIFO_PRESENT(fifo_status) == 0) {
      fprintf(stderr, "ADC Stream Thread[%d]: FIFO not present, stopping stream\n", board);
      break;
    }
    
    uint32_t word_count = FIFO_STS_WORD_COUNT(fifo_status);
    
    // Read in multiples of 4 words (8 samples)
    if (word_count >= 4) {
      uint32_t words_to_read = (word_count / 4) * 4;  // Highest multiple of 4
      
      for (uint32_t i = 0; i < words_to_read; i++) {
        uint32_t data = adc_read(ctx->adc_ctrl, board);
        
        // Split 32-bit data into two 16-bit values
        uint16_t lower_16 = data & 0xFFFF;
        uint16_t upper_16 = (data >> 16) & 0xFFFF;
        
        // Convert from offset to signed
        int16_t signed_lower = ADC_OFFSET_TO_SIGNED(lower_16);
        int16_t signed_upper = ADC_OFFSET_TO_SIGNED(upper_16);
        
        // Write to file (one value per line)
        fprintf(file, "%d\n", signed_lower);
        fprintf(file, "%d\n", signed_upper);
        
        total_samples++;
      }
      
      // Flush the file to ensure data is written
      fflush(file);
      
      if (*(ctx->verbose)) {
        printf("ADC Stream Thread[%d]: Read %u words (%u samples), total: %d\n", 
               board, words_to_read, words_to_read * 2, total_samples * 2);
      }
    } else {
      // Sleep for 100us if not enough data
      usleep(100);
    }
  }
  
  printf("ADC Stream Thread[%d]: Stopping stream, wrote %d samples (%d values) to file '%s'\n", 
         board, total_samples, total_samples * 2, file_path);
  
  fclose(file);
  ctx->adc_stream_running[board] = false;
  free(stream_data);
  return NULL;
}

int cmd_stream_adc_to_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stream_adc_to_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if stream is already running
  if (ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check FIFO presence
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot start streaming.\n", board);
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[1], full_path, sizeof(full_path));
  
  // Allocate thread data structure
  adc_stream_data_t* stream_data = malloc(sizeof(adc_stream_data_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for stream data\n");
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->should_stop = &(ctx->adc_stream_stop[board]);
  
  // Initialize stop flag and mark stream as running
  ctx->adc_stream_stop[board] = false;
  ctx->adc_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->adc_stream_threads[board]), NULL, adc_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(stream_data);
    return -1;
  }
  
  printf("Started ADC streaming for board %d to file '%s'\n", board, full_path);
  return 0;
}

int cmd_stop_adc_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stop_adc_stream: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if stream is running
  if (!ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is not running.\n", board);
    return -1;
  }
  
  printf("Stopping ADC streaming for board %d...\n", board);
  
  // Signal the thread to stop
  ctx->adc_stream_stop[board] = true;
  
  // Wait for the thread to finish
  if (pthread_join(ctx->adc_stream_threads[board], NULL) != 0) {
    fprintf(stderr, "Failed to join ADC streaming thread for board %d: %s\n", board, strerror(errno));
    return -1;
  }
  
  printf("ADC streaming for board %d has been stopped.\n", board);
  return 0;
}

// Command logging and playback functions
int cmd_log_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Stop current logging if active
  if (ctx->logging_enabled && ctx->log_file != NULL) {
    fclose(ctx->log_file);
    ctx->log_file = NULL;
    ctx->logging_enabled = false;
    printf("Previous log file closed.\n");
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[0], full_path, sizeof(full_path));
  
  // Open file for writing (create if doesn't exist, truncate if exists)
  ctx->log_file = fopen(full_path, "w");
  if (ctx->log_file == NULL) {
    fprintf(stderr, "Failed to open log file '%s' for writing: %s\n", full_path, strerror(errno));
    return -1;
  }
  
  // Set file permissions for group access
  set_file_permissions(full_path, *(ctx->verbose));
  
  ctx->logging_enabled = true;
  printf("Started logging commands to file '%s'\n", full_path);
  return 0;
}

int cmd_stop_log(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (!ctx->logging_enabled || ctx->log_file == NULL) {
    printf("Command logging is not currently active.\n");
    return 0;
  }
  
  fclose(ctx->log_file);
  ctx->log_file = NULL;
  ctx->logging_enabled = false;
  printf("Command logging stopped.\n");
  return 0;
}

int cmd_load_commands(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[0], full_path, sizeof(full_path));
  
  // Open file for reading
  FILE* file = fopen(full_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open command file '%s' for reading: %s\n", full_path, strerror(errno));
    return -1;
  }
  
  printf("Loading and executing commands from file '%s'...\n", full_path);
  
  char line[256];
  int line_number = 0;
  int commands_executed = 0;
  
  while (fgets(line, sizeof(line), file) != NULL) {
    line_number++;
    
    // Remove newline character
    size_t len = strlen(line);
    if (len > 0 && line[len - 1] == '\n') {
      line[len - 1] = '\0';
    }
    
    // Skip empty lines and lines starting with # (comments)
    if (strlen(line) == 0 || line[0] == '#') {
      continue;
    }
    
    printf("Executing line %d: %s\n", line_number, line);
    
    // Execute the command
    int result = execute_command(line, ctx);
    if (result != 0) {
      printf("Invalid command at line %d: '%s'\n", line_number, line);
      printf("Performing hard reset and exiting...\n");
      fclose(file);
      
      // Perform hard reset
      cmd_hard_reset(NULL, 0, NULL, 0, ctx);
      
      // Exit
      *(ctx->should_exit) = true;
      return -1;
    }
    
    commands_executed++;
    
    // 0.25 second delay between commands
    usleep(250000);
  }
  
  fclose(file);
  printf("Successfully executed %d commands from file '%s'.\n", commands_executed, full_path);
  return 0;
}

// Structure to hold a parsed waveform command
typedef struct {
  bool is_trigger;     // true for T, false for D
  uint32_t value;      // command value
  bool has_ch_vals;    // whether channel values are present
  int16_t ch_vals[8];  // channel values (if present)
  bool cont;           // continue flag
} waveform_command_t;

// ADC command structure
typedef struct {
  char type;           // 'L', 'T', 'D', 'O'
  uint32_t value;      // Value for the command 
  uint8_t order[8];    // For 'O' command, sample order
} adc_command_t;

// Structure to pass data to the DAC streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  waveform_command_t* commands;
  int command_count;
  int loop_count;       // Number of times to loop through the waveform (1 = play once)
} dac_stream_data_t;

// Structure to pass data to the ADC streaming thread
typedef struct {
  command_context_t* ctx;
  uint8_t board;
  char file_path[1024];
  volatile bool* should_stop;
  adc_command_t* commands;
  int command_count;
  int loop_count;       // Number of times to loop through the commands (1 = play once)
  bool simple_mode;     // Whether to unroll loops instead of using loop command
} adc_stream_data_t;

// Function to validate and parse a waveform file
static int parse_waveform_file(const char* file_path, waveform_command_t** commands, int* command_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open waveform file '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  // First pass: count lines and validate format
  char line[512];
  int line_num = 0;
  int valid_lines = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Check if line starts with D or T
    if (*trimmed != 'D' && *trimmed != 'T') {
      fprintf(stderr, "Invalid line %d: must start with 'D' or 'T'\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Parse the line to validate format
    char mode;
    uint32_t value;
    int16_t ch_vals[8];
    int parsed = sscanf(trimmed, "%c %u %hd %hd %hd %hd %hd %hd %hd %hd", 
                       &mode, &value, &ch_vals[0], &ch_vals[1], &ch_vals[2], &ch_vals[3],
                       &ch_vals[4], &ch_vals[5], &ch_vals[6], &ch_vals[7]);
    
    if (parsed < 2) {
      fprintf(stderr, "Invalid line %d: must have at least mode and value\n", line_num);
      fclose(file);
      return -1;
    }
    
    if (parsed != 2 && parsed != 10) {
      fprintf(stderr, "Invalid line %d: must have either 2 fields (mode, value) or 10 fields (mode, value, 8 channels)\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Validate value range
    if (value > 0x1FFFFFF) {
      fprintf(stderr, "Invalid line %d: value %u out of range (max 0x1FFFFFF or 33554431)\n", line_num, value);
      fclose(file);
      return -1;
    }
    
    // Validate channel values if present
    if (parsed == 10) {
      for (int i = 0; i < 8; i++) {
        if (ch_vals[i] < -32767 || ch_vals[i] > 32767) {
          fprintf(stderr, "Invalid line %d: channel %d value %d out of range (-32767 to 32767)\n", 
                 line_num, i, ch_vals[i]);
          fclose(file);
          return -1;
        }
      }
    }
    
    valid_lines++;
  }
  
  if (valid_lines == 0) {
    fprintf(stderr, "No valid commands found in waveform file\n");
    fclose(file);
    return -1;
  }
  
  // Allocate memory for commands
  *commands = malloc(valid_lines * sizeof(waveform_command_t));
  if (*commands == NULL) {
    fprintf(stderr, "Failed to allocate memory for waveform commands\n");
    fclose(file);
    return -1;
  }
  
  // Second pass: parse and store commands
  rewind(file);
  line_num = 0;
  int cmd_index = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    waveform_command_t* cmd = &(*commands)[cmd_index];
    
    char mode;
    uint32_t value;
    int16_t ch_vals[8];
    int parsed = sscanf(trimmed, "%c %u %hd %hd %hd %hd %hd %hd %hd %hd", 
                       &mode, &value, &ch_vals[0], &ch_vals[1], &ch_vals[2], &ch_vals[3],
                       &ch_vals[4], &ch_vals[5], &ch_vals[6], &ch_vals[7]);
    
    cmd->is_trigger = (mode == 'T');
    cmd->value = value;
    cmd->has_ch_vals = (parsed == 10);
    cmd->cont = (cmd_index < valid_lines - 1); // true for all except last command
    
    if (cmd->has_ch_vals) {
      for (int i = 0; i < 8; i++) {
        cmd->ch_vals[i] = ch_vals[i];
      }
    }
    
    cmd_index++;
  }
  
  fclose(file);
  *command_count = valid_lines;
  return 0;
}

// Thread function for DAC streaming
void* dac_stream_thread(void* arg) {
  dac_stream_data_t* stream_data = (dac_stream_data_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  const char* file_path = stream_data->file_path;
  volatile bool* should_stop = stream_data->should_stop;
  waveform_command_t* commands = stream_data->commands;
  int command_count = stream_data->command_count;
  int loop_count = stream_data->loop_count;
  
  printf("DAC Stream Thread[%d]: Started streaming from file '%s' (%d commands, %d loop%s)\n", 
         board, file_path, command_count, loop_count, loop_count == 1 ? "" : "s");
  
  int total_commands_sent = 0;
  int current_loop = 0;
  
  while (!(*should_stop) && current_loop < loop_count) {
    int cmd_index = 0;
    int commands_sent_this_loop = 0;
    
    // Process all commands in the current loop
    while (!(*should_stop) && cmd_index < command_count) {
      // Check DAC command FIFO status
      uint32_t fifo_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, board, false);
      
      if (FIFO_PRESENT(fifo_status) == 0) {
        fprintf(stderr, "DAC Stream Thread[%d]: FIFO not present, stopping stream\n", board);
        goto cleanup;
      }
      
      uint32_t words_used = FIFO_STS_WORD_COUNT(fifo_status);
      uint32_t words_available = DAC_CMD_FIFO_WORDCOUNT - (words_used + 1);
      
      // Check if we have space for the next command
      waveform_command_t* cmd = &commands[cmd_index];
      uint32_t words_needed = cmd->has_ch_vals ? 5 : 1; // dac_wr needs 5 words, noop needs 1
      
      if (words_available >= words_needed) {
        // For looping, we need to adjust the 'cont' flag:
        // - Set cont=true for all commands except the last command of the last loop
        bool is_last_command_of_last_loop = (current_loop == loop_count - 1) && (cmd_index == command_count - 1);
        bool cont_flag = !is_last_command_of_last_loop;
        
        // Send the command
        if (cmd->has_ch_vals) {
          dac_cmd_dac_wr(ctx->dac_ctrl, board, cmd->ch_vals, cmd->is_trigger, cont_flag, true, cmd->value, false);
        } else {
          dac_cmd_noop(ctx->dac_ctrl, board, cmd->is_trigger, cont_flag, false, cmd->value, false);
        }
        
        commands_sent_this_loop++;
        total_commands_sent++;
        cmd_index++;
        
        if (*(ctx->verbose)) {
          printf("DAC Stream Thread[%d]: Loop %d/%d, Sent command %d/%d (%s, value=%u, %s, cont=%s)\n", 
                 board, current_loop + 1, loop_count, commands_sent_this_loop, command_count, 
                 cmd->is_trigger ? "trigger" : "delay", cmd->value,
                 cmd->has_ch_vals ? "with ch_vals" : "noop",
                 cont_flag ? "true" : "false");
        }
      } else {
        // Not enough space, sleep and try again
        usleep(100);
      }
    }
    
    current_loop++;
    if (current_loop < loop_count && *(ctx->verbose)) {
      printf("DAC Stream Thread[%d]: Completed loop %d/%d, starting next loop\n", 
             board, current_loop, loop_count);
    }
  }

cleanup:
  if (*should_stop) {
    printf("DAC Stream Thread[%d]: Stopping stream (user requested), sent %d total commands (%d complete loops)\n", 
           board, total_commands_sent, current_loop);
  } else {
    printf("DAC Stream Thread[%d]: Stream completed, sent %d total commands from file '%s' (%d loops)\n", 
           board, total_commands_sent, file_path, loop_count);
  }
  
  ctx->dac_stream_running[board] = false;
  free(stream_data->commands);
  free(stream_data);
  return NULL;
}

// Function to validate and parse an ADC command file
static int parse_adc_command_file(const char* file_path, adc_command_t** commands, int* command_count) {
  FILE* file = fopen(file_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open ADC command file '%s': %s\n", file_path, strerror(errno));
    return -1;
  }
  
  // First pass: count lines and validate format
  char line[512];
  int line_num = 0;
  int valid_lines = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    // Check if line starts with L, T, D, or O
    if (*trimmed != 'L' && *trimmed != 'T' && *trimmed != 'D' && *trimmed != 'O') {
      fprintf(stderr, "Invalid line %d: must start with 'L', 'T', 'D', or 'O'\n", line_num);
      fclose(file);
      return -1;
    }
    
    // Parse the line to validate format
    char mode;
    uint32_t value;
    uint8_t s1, s2, s3, s4, s5, s6, s7, s8;
    
    if (*trimmed == 'O') {
      // Order command: O s1 s2 s3 s4 s5 s6 s7 s8
      int parsed = sscanf(trimmed, "%c %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu", 
                         &mode, &s1, &s2, &s3, &s4, &s5, &s6, &s7, &s8);
      if (parsed != 9) {
        fprintf(stderr, "Invalid line %d: 'O' command must have 8 order values\n", line_num);
        fclose(file);
        return -1;
      }
      // Validate order values (0-7)
      if (s1 > 7 || s2 > 7 || s3 > 7 || s4 > 7 || s5 > 7 || s6 > 7 || s7 > 7 || s8 > 7) {
        fprintf(stderr, "Invalid line %d: order values must be 0-7\n", line_num);
        fclose(file);
        return -1;
      }
    } else {
      // L, T, D commands: <cmd> <value>
      int parsed = sscanf(trimmed, "%c %u", &mode, &value);
      if (parsed != 2) {
        fprintf(stderr, "Invalid line %d: must have command and value\n", line_num);
        fclose(file);
        return -1;
      }
      
      // Validate value range
      if (value > 0x1FFFFFF) {
        fprintf(stderr, "Invalid line %d: value %u out of range (max 0x1FFFFFF or 33554431)\n", line_num, value);
        fclose(file);
        return -1;
      }
    }
    
    valid_lines++;
  }
  
  if (valid_lines == 0) {
    fprintf(stderr, "No valid commands found in ADC command file\n");
    fclose(file);
    return -1;
  }
  
  // Allocate memory for commands
  *commands = malloc(valid_lines * sizeof(adc_command_t));
  if (*commands == NULL) {
    fprintf(stderr, "Failed to allocate memory for ADC commands\n");
    fclose(file);
    return -1;
  }
  
  // Second pass: actually parse the commands
  rewind(file);
  line_num = 0;
  *command_count = 0;
  
  while (fgets(line, sizeof(line), file)) {
    line_num++;
    
    // Skip empty lines and comments
    char* trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t') trimmed++;
    if (*trimmed == '\n' || *trimmed == '\r' || *trimmed == '\0' || *trimmed == '#') {
      continue;
    }
    
    adc_command_t* cmd = &(*commands)[*command_count];
    cmd->type = *trimmed;
    
    if (*trimmed == 'O') {
      uint8_t s1, s2, s3, s4, s5, s6, s7, s8;
      sscanf(trimmed, "%c %hhu %hhu %hhu %hhu %hhu %hhu %hhu %hhu", 
             &cmd->type, &s1, &s2, &s3, &s4, &s5, &s6, &s7, &s8);
      cmd->order[0] = s1; cmd->order[1] = s2; cmd->order[2] = s3; cmd->order[3] = s4;
      cmd->order[4] = s5; cmd->order[5] = s6; cmd->order[6] = s7; cmd->order[7] = s8;
      cmd->value = 0; // Not used for order commands
    } else {
      sscanf(trimmed, "%c %u", &cmd->type, &cmd->value);
      // Initialize order array (not used for non-O commands)
      for (int i = 0; i < 8; i++) {
        cmd->order[i] = 0;
      }
    }
    
    (*command_count)++;
  }
  
  fclose(file);
  return 0;
}

int cmd_stream_dac_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stream_dac_from_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse optional loop count (default is 1 - play once)
  int loop_count = 1;
  if (arg_count >= 3) {
    char* endptr;
    loop_count = (int)parse_value(args[2], &endptr);
    if (*endptr != '\0' || loop_count < 1) {
      fprintf(stderr, "Invalid loop count for stream_dac_from_file: '%s'. Must be a positive integer.\n", args[2]);
      return -1;
    }
  }
  
  // Check if stream is already running
  if (ctx->dac_stream_running[board]) {
    printf("DAC stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check DAC command FIFO presence
  if (FIFO_PRESENT(sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("DAC command FIFO for board %d is not present. Cannot start streaming.\n", board);
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[1], full_path, sizeof(full_path));
  
  // Parse and validate the waveform file
  waveform_command_t* commands = NULL;
  int command_count = 0;
  
  if (parse_waveform_file(full_path, &commands, &command_count) != 0) {
    return -1; // Error already printed by parse_waveform_file
  }
  
  printf("Parsed %d commands from waveform file '%s'\n", command_count, full_path);
  
  // Allocate thread data structure
  dac_stream_data_t* stream_data = malloc(sizeof(dac_stream_data_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for stream data\n");
    free(commands);
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->should_stop = &(ctx->dac_stream_stop[board]);
  stream_data->commands = commands;
  stream_data->command_count = command_count;
  stream_data->loop_count = loop_count;
  
  // Initialize stop flag and mark stream as running
  ctx->dac_stream_stop[board] = false;
  ctx->dac_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->dac_stream_threads[board]), NULL, dac_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create DAC streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->dac_stream_running[board] = false;
    free(commands);
    free(stream_data);
    return -1;
  }
  
  printf("Started DAC streaming for board %d from file '%s' (looping %d time%s)\n", 
         board, full_path, loop_count, loop_count == 1 ? "" : "s");
  return 0;
}

int cmd_stop_dac_stream(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stop_dac_stream: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Check if stream is running
  if (!ctx->dac_stream_running[board]) {
    printf("DAC stream for board %d is not running.\n", board);
    return -1;
  }
  
  printf("Stopping DAC streaming for board %d...\n", board);
  
  // Signal the thread to stop
  ctx->dac_stream_stop[board] = true;
  
  // Wait for the thread to finish
  if (pthread_join(ctx->dac_stream_threads[board], NULL) != 0) {
    fprintf(stderr, "Failed to join DAC streaming thread for board %d: %s\n", board, strerror(errno));
    return -1;
  }
  
  printf("DAC streaming for board %d has been stopped.\n", board);
  return 0;
}

// ADC streaming thread function
static void* adc_stream_thread(void* arg) {
  adc_stream_data_t* stream_data = (adc_stream_data_t*)arg;
  command_context_t* ctx = stream_data->ctx;
  uint8_t board = stream_data->board;
  bool verbose = *(ctx->verbose);
  
  printf("Starting ADC streaming thread for board %d\n", board);
  
  // Stream commands for each loop iteration
  for (int loop = 0; loop < stream_data->loop_count && !*(stream_data->should_stop); loop++) {
    if (verbose) {
      printf("ADC stream loop %d/%d for board %d\n", loop + 1, stream_data->loop_count, board);
    }
    
    // Stream each command
    for (int i = 0; i < stream_data->command_count && !*(stream_data->should_stop); i++) {
      adc_command_t* cmd = &stream_data->commands[i];
      
      // Check if command FIFO has space
      uint32_t cmd_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false);
      if (!FIFO_PRESENT(cmd_status)) {
        fprintf(stderr, "ADC command FIFO for board %d is not present\n", board);
        break;
      }
      
      // Wait if FIFO is full
      while (FIFO_STS_FULL(cmd_status) && !*(stream_data->should_stop)) {
        usleep(1000); // Wait 1ms
        cmd_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false);
      }
      
      if (*(stream_data->should_stop)) break;
      
      // Send command based on type
      switch (cmd->type) {
        case 'L':
          if (stream_data->simple_mode) {
            // Simple mode: unroll the loop by sending the next command multiple times
            if (i + 1 < stream_data->command_count) {
              adc_command_t* next_cmd = &stream_data->commands[i + 1];
              for (uint32_t j = 0; j < cmd->value && !*(stream_data->should_stop); j++) {
                // Wait for FIFO space
                while (FIFO_STS_FULL(sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, false)) && !*(stream_data->should_stop)) {
                  usleep(1000);
                }
                if (*(stream_data->should_stop)) break;
                
                // Send the next command
                switch (next_cmd->type) {
                  case 'T':
                    adc_cmd_noop(ctx->adc_ctrl, board, true, false, next_cmd->value, verbose);
                    break;
                  case 'D':
                    adc_cmd_noop(ctx->adc_ctrl, board, false, false, next_cmd->value, verbose);
                    break;
                  case 'O':
                    adc_cmd_set_ord(ctx->adc_ctrl, board, next_cmd->order, verbose);
                    break;
                  default:
                    fprintf(stderr, "Invalid ADC command type after loop: %c\n", next_cmd->type);
                    break;
                }
              }
              // Skip the next command since we already processed it
              i++;
            }
          } else {
            // Use loop next command
            adc_cmd_loop_next(ctx->adc_ctrl, board, cmd->value, verbose);
          }
          break;
          
        case 'T':
          adc_cmd_noop(ctx->adc_ctrl, board, true, false, cmd->value, verbose);
          break;
          
        case 'D':
          adc_cmd_noop(ctx->adc_ctrl, board, false, false, cmd->value, verbose);
          break;
          
        case 'O':
          adc_cmd_set_ord(ctx->adc_ctrl, board, cmd->order, verbose);
          break;
          
        default:
          fprintf(stderr, "Invalid ADC command type: %c\n", cmd->type);
          break;
      }
      
      if (verbose) {
        printf("ADC stream sent command %c for board %d\n", cmd->type, board);
      }
    }
  }
  
  printf("ADC streaming thread for board %d completed\n", board);
  
  // Mark stream as not running and clean up
  ctx->adc_stream_running[board] = false;
  free(stream_data->commands);
  free(stream_data);
  return NULL;
}

int cmd_stream_adc_from_file(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse board number
  int board = parse_board_number(args[0]);
  if (board < 0) {
    fprintf(stderr, "Invalid board number for stream_adc_from_file: '%s'. Must be 0-7.\n", args[0]);
    return -1;
  }
  
  // Parse optional loop count (default is 1 - play once)
  int loop_count = 1;
  if (arg_count >= 3) {
    char* endptr;
    loop_count = (int)parse_value(args[2], &endptr);
    if (*endptr != '\0' || loop_count < 1) {
      fprintf(stderr, "Invalid loop count for stream_adc_from_file: '%s'. Must be a positive integer.\n", args[2]);
      return -1;
    }
  }
  
  // Check for simple flag
  bool simple_mode = has_flag(flags, flag_count, FLAG_SIMPLE);
  
  // Check if stream is already running
  if (ctx->adc_stream_running[board]) {
    printf("ADC stream for board %d is already running.\n", board);
    return -1;
  }
  
  // Check ADC command FIFO presence
  if (FIFO_PRESENT(sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, *(ctx->verbose))) == 0) {
    printf("ADC command FIFO for board %d is not present. Cannot start streaming.\n", board);
    return -1;
  }
  
  // Clean and expand file path
  char full_path[1024];
  clean_and_expand_path(args[1], full_path, sizeof(full_path));
  
  // Parse and validate the ADC command file
  adc_command_t* commands = NULL;
  int command_count = 0;
  
  if (parse_adc_command_file(full_path, &commands, &command_count) != 0) {
    return -1; // Error already printed by parse_adc_command_file
  }
  
  printf("Parsed %d commands from ADC command file '%s'\n", command_count, full_path);
  if (simple_mode) {
    printf("Using simple mode (unrolling loops)\n");
  }
  
  // Allocate thread data structure
  adc_stream_data_t* stream_data = malloc(sizeof(adc_stream_data_t));
  if (stream_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for ADC stream data\n");
    free(commands);
    return -1;
  }
  
  stream_data->ctx = ctx;
  stream_data->board = (uint8_t)board;
  strcpy(stream_data->file_path, full_path);
  stream_data->should_stop = &(ctx->adc_stream_stop[board]);
  stream_data->commands = commands;
  stream_data->command_count = command_count;
  stream_data->loop_count = loop_count;
  stream_data->simple_mode = simple_mode;
  
  // Initialize stop flag and mark stream as running
  ctx->adc_stream_stop[board] = false;
  ctx->adc_stream_running[board] = true;
  
  // Create the streaming thread
  if (pthread_create(&(ctx->adc_stream_threads[board]), NULL, adc_stream_thread, stream_data) != 0) {
    fprintf(stderr, "Failed to create ADC streaming thread for board %d: %s\n", board, strerror(errno));
    ctx->adc_stream_running[board] = false;
    free(commands);
    free(stream_data);
    return -1;
  }
  
  printf("Started ADC streaming for board %d from file '%s' (looping %d time%s)%s\n", 
         board, full_path, loop_count, loop_count == 1 ? "" : "s",
         simple_mode ? " in simple mode" : "");
  return 0;
}

// Single channel commands implementation
int cmd_do_dac_wr_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse channel (0-63)
  int channel = atoi(args[0]);
  if (channel < 0 || channel > 63) {
    fprintf(stderr, "Invalid channel number for do_dac_wr_ch: '%s'. Must be 0-63.\n", args[0]);
    return -1;
  }
  
  // Parse channel value
  char *endptr;
  int16_t ch_val = (int16_t)parse_value(args[1], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid channel value for do_dac_wr_ch: '%s'. Must be a valid integer.\n", args[1]);
    return -1;
  }
  
  // Calculate board and local channel
  uint8_t board = (uint8_t)(channel / 8);
  uint8_t ch = (uint8_t)(channel % 8);
  
  if (*(ctx->verbose)) {
    printf("Writing DAC channel %d (board %d, ch %d) with value %d\n", channel, board, ch, ch_val);
  }
  
  // Call dac_cmd_dac_wr_ch
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, board, ch, ch_val, *(ctx->verbose));
  
  return 0;
}

int cmd_do_adc_rd_ch(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse channel (0-63)
  int channel = atoi(args[0]);
  if (channel < 0 || channel > 63) {
    fprintf(stderr, "Invalid channel number for do_adc_rd_ch: '%s'. Must be 0-63.\n", args[0]);
    return -1;
  }
  
  // Calculate board and local channel
  uint8_t board = (uint8_t)(channel / 8);
  uint8_t ch = (uint8_t)(channel % 8);
  
  if (*(ctx->verbose)) {
    printf("Reading ADC channel %d (board %d, ch %d)\n", channel, board, ch);
  }
  
  // Call adc_cmd_adc_rd_ch
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, board, ch, *(ctx->verbose));
  
  return 0;
}

int cmd_read_adc_single(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse channel (0-63)
  int channel = atoi(args[0]);
  if (channel < 0 || channel > 63) {
    fprintf(stderr, "Invalid channel number for read_adc_single: '%s'. Must be 0-63.\n", args[0]);
    return -1;
  }
  
  // Calculate board and local channel
  uint8_t board = (uint8_t)(channel / 8);
  uint8_t ch = (uint8_t)(channel % 8);
  
  // Check if ADC data FIFO is present
  if (FIFO_PRESENT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose))) == 0) {
    printf("ADC data FIFO for board %d is not present. Cannot read data.\n", board);
    return -1;
  }
  
  // Check if ADC data FIFO is empty
  if (FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose)))) {
    printf("ADC data FIFO for board %d is empty. Cannot read data.\n", board);
    return -1;
  }
  
  bool read_all = has_flag(flags, flag_count, FLAG_ALL);
  
  if (read_all) {
    printf("Reading all data from ADC FIFO for channel %d (board %d)...\n", channel, board);
    int count = 0;
    while (!FIFO_STS_EMPTY(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose)))) {
      int16_t data = adc_read_ch(ctx->adc_ctrl, board);
      printf("Sample %d - ADC channel %d data: %d (0x%04X)\n", ++count, channel, data, (uint16_t)data);
    }
    printf("Read %d samples total.\n", count);
  } else {
    int16_t data = adc_read_ch(ctx->adc_ctrl, board);
    printf("Read ADC channel %d data: %d (0x%04X)\n", channel, data, (uint16_t)data);
  }
  
  return 0;
}

int cmd_set_and_check(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse channel (0-63)
  int channel = atoi(args[0]);
  if (channel < 0 || channel > 63) {
    fprintf(stderr, "Invalid channel number for set_and_check: '%s'. Must be 0-63.\n", args[0]);
    return -1;
  }
  
  // Parse channel value
  char *endptr;
  int16_t ch_val = (int16_t)parse_value(args[1], &endptr);
  if (*endptr != '\0') {
    fprintf(stderr, "Invalid channel value for set_and_check: '%s'. Must be a valid integer.\n", args[1]);
    return -1;
  }
  
  // Calculate board and local channel
  uint8_t board = (uint8_t)(channel / 8);
  uint8_t ch = (uint8_t)(channel % 8);
  
  printf("Set and check: channel %d (board %d, ch %d) with value %d\n", channel, board, ch, ch_val);
  
  // Step 1: Check that DAC command buffer is empty
  uint32_t dac_cmd_status = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, board, *(ctx->verbose));
  if (FIFO_PRESENT(dac_cmd_status) == 0) {
    fprintf(stderr, "DAC command FIFO for board %d is not present. Cannot proceed.\n", board);
    return -1;
  }
  if (!FIFO_STS_EMPTY(dac_cmd_status)) {
    fprintf(stderr, "DAC command FIFO for board %d is not empty. Cannot proceed.\n", board);
    return -1;
  }
  printf("✓ DAC command buffer for board %d is empty\n", board);
  
  // Step 2: Check that ADC command buffer is empty
  uint32_t adc_cmd_status = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, board, *(ctx->verbose));
  if (FIFO_PRESENT(adc_cmd_status) == 0) {
    fprintf(stderr, "ADC command FIFO for board %d is not present. Cannot proceed.\n", board);
    return -1;
  }
  if (!FIFO_STS_EMPTY(adc_cmd_status)) {
    fprintf(stderr, "ADC command FIFO for board %d is not empty. Cannot proceed.\n", board);
    return -1;
  }
  printf("✓ ADC command buffer for board %d is empty\n", board);
  
  // Step 3: Check that ADC data buffer is empty
  uint32_t adc_data_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose));
  if (FIFO_PRESENT(adc_data_status) == 0) {
    fprintf(stderr, "ADC data FIFO for board %d is not present. Cannot proceed.\n", board);
    return -1;
  }
  if (!FIFO_STS_EMPTY(adc_data_status)) {
    fprintf(stderr, "ADC data FIFO for board %d is not empty. Cannot proceed.\n", board);
    return -1;
  }
  printf("✓ ADC data buffer for board %d is empty\n", board);
  
  // Step 4: Execute DAC write (do_dac_wr_ch equivalent)
  printf("Writing DAC channel %d with value %d...\n", channel, ch_val);
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, board, ch, ch_val, *(ctx->verbose));
  
  // Step 5: Wait 500ms
  printf("Waiting 500ms...\n");
  usleep(500000); // 500ms = 500,000 microseconds
  
  // Step 6: Execute ADC read (do_adc_rd_ch equivalent)
  printf("Reading ADC channel %d...\n", channel);
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, board, ch, *(ctx->verbose));
  
  // Step 7: Read single ADC data (read_adc_single equivalent)
  // Wait a bit for the ADC read command to complete and data to be available
  printf("Waiting for ADC data...\n");
  usleep(10000); // Wait 10ms for data to be ready
  
  // Check if ADC data is now available
  adc_data_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose));
  if (FIFO_STS_EMPTY(adc_data_status)) {
    fprintf(stderr, "ADC data FIFO for board %d is still empty after read command. No data available.\n", board);
    return -1;
  }
  
  // Read the ADC data
  int16_t data = adc_read_ch(ctx->adc_ctrl, board);
  printf("✓ Read ADC channel %d data: %d (0x%04X)\n", channel, data, (uint16_t)data);
  
  printf("Set and check completed successfully.\n");
  return 0;
}

// New test commands implementations

int cmd_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  // Parse channel (0-63)
  char* endptr;
  uint32_t channel = parse_value(args[0], &endptr);
  if (*endptr != '\0' || channel > 63) {
    fprintf(stderr, "Invalid channel for channel_test: '%s'. Must be 0-63.\n", args[0]);
    return -1;
  }
  
  // Parse DAC value
  int32_t value = (int32_t)parse_value(args[1], &endptr);
  if (*endptr != '\0' || value < -32767 || value > 32767) {
    fprintf(stderr, "Invalid value for channel_test: '%s'. Must be -32767 to 32767.\n", args[1]);
    return -1;
  }

  uint8_t board = channel / 8;
  uint8_t ch = channel % 8;
  int16_t ch_val = (int16_t)value;

  printf("=== Channel Test ===\n");
  printf("Channel: %u (Board %u, Channel %u)\n", channel, board, ch);
  printf("Target Value: %d\n", ch_val);
  printf("\n");

  // Step 1: Check that system is on
  printf("Step 1: Checking system status...\n");
  uint32_t hw_status = sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose));
  if (HW_STS_STATE(hw_status) != HW_STATE_RUNNING) {
    fprintf(stderr, "System is not running. Current state: 0x%X. Please turn system on first.\n", HW_STS_STATE(hw_status));
    return -1;
  }
  printf("✓ System is running\n");

  // Step 2: Reset ADC and DAC buffers for this board
  printf("\nStep 2: Resetting buffers for board %u...\n", board);
  uint32_t board_mask = 1 << board;
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, board_mask, *(ctx->verbose));
  usleep(1000); // Wait 1ms
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  printf("✓ Buffers reset for board %u\n", board);

  // Step 3: Send wr_ch command to DAC, 100000 cycle delay command to ADC, and rd_ch command to ADC
  printf("\nStep 3: Sending commands...\n");
  
  // Send DAC write channel command
  printf("Sending DAC write channel command...\n");
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, board, ch, ch_val, *(ctx->verbose));
  
  // Send ADC delay command (100000 cycles)
  printf("Sending ADC delay command (100000 cycles)...\n");
  adc_cmd_noop(ctx->adc_ctrl, board, false, false, 100000, *(ctx->verbose));
  
  // Send ADC read channel command
  printf("Sending ADC read channel command...\n");
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, board, ch, *(ctx->verbose));
  
  // Step 4: Sleep 10 ms
  printf("\nStep 4: Waiting 10ms for commands to execute...\n");
  usleep(10000); // 10ms
  
  // Step 5: Read single from ADC
  printf("\nStep 5: Reading ADC data...\n");
  uint32_t adc_data_status = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, board, *(ctx->verbose));
  if (FIFO_STS_EMPTY(adc_data_status)) {
    fprintf(stderr, "ADC data FIFO for board %u is empty. No data available.\n", board);
    return -1;
  }
  
  int16_t adc_value = adc_read_ch(ctx->adc_ctrl, board);
  printf("✓ Read ADC value: %d (0x%04X)\n", adc_value, (uint16_t)adc_value);
  
  // Step 6: Calculate and print error
  printf("\nStep 6: Error Analysis\n");
  int32_t error = adc_value - ch_val;
  double percent_error = 0.0;
  if (ch_val != 0) {
    percent_error = (double)error / (double)ch_val * 100.0;
  }
  
  printf("Target Value: %d\n", ch_val);
  printf("Measured Value: %d\n", adc_value);
  printf("Absolute Error: %d\n", error);
  printf("Percent Error: %.2f%%\n", percent_error);
  
  printf("\n=== Channel Test Complete ===\n");
  return 0;
}

int cmd_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  printf("=== Waveform Test ===\n");
  printf("This interactive test will run DAC and ADC waveforms together.\n\n");
  
  char dac_file[1024];
  char adc_file[1024];
  char output_file[1024];
  int num_loops;
  uint32_t lockout_time;
  int board_number;
  
  // Step 1: Reset all buffers
  printf("Step 1: Resetting all buffers...\n");
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0x1FFFF, *(ctx->verbose));
  usleep(1000); // Wait 1ms
  sys_ctrl_set_cmd_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  sys_ctrl_set_data_buf_reset(ctx->sys_ctrl, 0, *(ctx->verbose));
  printf("✓ All buffers reset\n\n");
  
  // Step 2: Prompt for board number
  printf("Step 2: Board Selection\n");
  printf("Enter board number (0-7): ");
  fflush(stdout);
  if (scanf("%d", &board_number) != 1 || board_number < 0 || board_number > 7) {
    fprintf(stderr, "Invalid board number. Must be 0-7.\n");
    return -1;
  }
  // Clear input buffer
  int c;
  while ((c = getchar()) != '\n' && c != EOF);
  printf("✓ Using board %d\n\n");
  
  // Step 3: Prompt for DAC command file
  printf("Step 3: DAC Configuration\n");
  printf("Enter DAC command file path: ");
  fflush(stdout);
  if (fgets(dac_file, sizeof(dac_file), stdin) == NULL) {
    fprintf(stderr, "Failed to read DAC file path\n");
    return -1;
  }
  // Remove newline
  size_t len = strlen(dac_file);
  if (len > 0 && dac_file[len-1] == '\n') {
    dac_file[len-1] = '\0';
  }
  
  // Clean and expand DAC file path
  char full_dac_path[1024];
  clean_and_expand_path(dac_file, full_dac_path, sizeof(full_dac_path));
  
  // Parse DAC file to count trigger lines
  waveform_command_t* dac_commands = NULL;
  int dac_command_count = 0;
  if (parse_waveform_file(full_dac_path, &dac_commands, &dac_command_count) != 0) {
    return -1;
  }
  
  // Count trigger lines in DAC file
  int trigger_lines = 0;
  for (int i = 0; i < dac_command_count; i++) {
    if (dac_commands[i].trigger) {
      trigger_lines++;
    }
  }
  printf("✓ DAC file parsed: %d commands, %d trigger lines\n", dac_command_count, trigger_lines);
  
  // Step 4: Prompt for ADC command file
  printf("\nStep 4: ADC Configuration\n");
  printf("Enter ADC command file path: ");
  fflush(stdout);
  if (fgets(adc_file, sizeof(adc_file), stdin) == NULL) {
    fprintf(stderr, "Failed to read ADC file path\n");
    free(dac_commands);
    return -1;
  }
  // Remove newline
  len = strlen(adc_file);
  if (len > 0 && adc_file[len-1] == '\n') {
    adc_file[len-1] = '\0';
  }
  
  // Clean and expand ADC file path
  char full_adc_path[1024];
  clean_and_expand_path(adc_file, full_adc_path, sizeof(full_adc_path));
  
  // Parse ADC file
  adc_command_t* adc_commands = NULL;
  int adc_command_count = 0;
  if (parse_adc_command_file(full_adc_path, &adc_commands, &adc_command_count) != 0) {
    free(dac_commands);
    return -1;
  }
  printf("✓ ADC file parsed: %d commands\n", adc_command_count);
  
  // Step 5: Prompt for number of loops
  printf("\nStep 5: Loop Configuration\n");
  printf("Enter number of loops: ");
  fflush(stdout);
  if (scanf("%d", &num_loops) != 1 || num_loops < 1) {
    fprintf(stderr, "Invalid number of loops. Must be a positive integer.\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  // Clear input buffer
  int c;
  while ((c = getchar()) != '\n' && c != EOF);
  
  // Step 6: Prompt for output file
  printf("\nStep 6: Output Configuration\n");
  printf("Enter output file path: ");
  fflush(stdout);
  if (fgets(output_file, sizeof(output_file), stdin) == NULL) {
    fprintf(stderr, "Failed to read output file path\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  // Remove newline
  len = strlen(output_file);
  if (len > 0 && output_file[len-1] == '\n') {
    output_file[len-1] = '\0';
  }
  
  // Clean and expand output file path
  char full_output_path[1024];
  clean_and_expand_path(output_file, full_output_path, sizeof(full_output_path));
  
  // Step 7: Prompt for trigger lockout time
  printf("\nStep 7: Trigger Configuration\n");
  printf("Enter trigger lockout time (cycles): ");
  fflush(stdout);
  if (scanf("%u", &lockout_time) != 1 || lockout_time > 0x1FFFFFFF) {
    fprintf(stderr, "Invalid lockout time. Must be 0 to 536870911.\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  // Clear input buffer
  while ((c = getchar()) != '\n' && c != EOF);
  
  printf("\n=== Configuration Summary ===\n");
  printf("Board: %d\n", board_number);
  printf("DAC file: %s (%d commands, %d triggers)\n", full_dac_path, dac_command_count, trigger_lines);
  printf("ADC file: %s (%d commands)\n", full_adc_path, adc_command_count);
  printf("Output file: %s\n", full_output_path);
  printf("Loops: %d\n", num_loops);
  printf("Lockout time: %u cycles\n", lockout_time);
  printf("Total expected triggers: %d\n", trigger_lines * num_loops);
  printf("\n");
  
  // Step 8: Set trigger lockout
  printf("Step 8: Setting trigger lockout...\n");
  trigger_cmd_set_lockout(ctx->trigger_ctrl, lockout_time);
  printf("✓ Trigger lockout set to %u cycles\n", lockout_time);
  
  // Step 9: Send expect external triggers command
  uint32_t total_triggers = trigger_lines * num_loops;
  printf("\nStep 9: Setting expected external triggers...\n");
  trigger_cmd_expect_ext(ctx->trigger_ctrl, total_triggers);
  printf("✓ Expecting %u external triggers\n", total_triggers);
  
  // Step 10: Start streaming (DAC and ADC on selected board)
  printf("\nStep 10: Starting waveform streaming...\n");
  
  // Start DAC streaming using existing function (simulate command)
  char board_str[16];
  sprintf(board_str, "%d", board_number);
  const char* dac_args[] = {board_str, full_dac_path, ""};
  sprintf((char*)dac_args[2], "%d", num_loops);
  if (cmd_stream_dac_from_file(dac_args, 3, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start DAC streaming\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  
  // Start ADC streaming using existing function (simulate command)
  const char* adc_args[] = {board_str, full_adc_path, ""};
  sprintf((char*)adc_args[2], "%d", num_loops);
  if (cmd_stream_adc_from_file(adc_args, 3, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start ADC streaming\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  
  printf("✓ DAC and ADC streaming started\n");
  
  // Step 11: Start output file streaming
  printf("\nStep 11: Starting output file streaming...\n");
  
  // Start ADC to file streaming (selected board)
  const char* output_args[] = {board_str, full_output_path};
  if (cmd_stream_adc_to_file(output_args, 2, NULL, 0, ctx) != 0) {
    fprintf(stderr, "Failed to start output file streaming\n");
    free(dac_commands);
    free(adc_commands);
    return -1;
  }
  printf("✓ Output file streaming started\n");
  
  printf("\n=== Waveform Test Running ===\n");
  printf("DAC and ADC are now streaming on board %d. Monitor the output file: %s\n", board_number, full_output_path);
  printf("Use 'stop_dac_stream %d', 'stop_adc_stream %d' to stop streaming manually.\n", board_number, board_number);
  printf("Test will run for %d loops with %d triggers per loop.\n", num_loops, trigger_lines);
  
  // Cleanup parsed command arrays
  free(dac_commands);
  free(adc_commands);
  
  printf("\n=== Waveform Test Setup Complete ===\n");
  return 0;
}
