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
#include <glob.h>
#include "command_handler.h"
#include "command_helper.h"
#include "system_commands.h"
#include "adc_commands.h"
#include "dac_commands.h"
#include "trigger_commands.h"
#include "experiment_commands.h"

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
 * 1. Add the command handler function declaration to the appropriate header file
 * 2. Implement the command handler function in the appropriate module
 * 3. Add an entry to this table before the sentinel (NULL entry)
 * 4. The help system will automatically generate help text from the metadata
 */
static command_entry_t command_table[] = {
  // ===== SYSTEM COMMANDS (from system_commands.h) =====
  {"help", cmd_help, {0, 0, {-1}, "Show this help message"}},
  {"verbose", cmd_verbose, {0, 0, {-1}, "Toggle verbose mode"}},
  {"on", cmd_on, {0, 0, {-1}, "Turn the system on"}},
  {"off", cmd_off, {0, 0, {-1}, "Turn the system off"}},
  {"sts", cmd_sts, {0, 0, {-1}, "Show hardware manager status"}},
  {"dbg", cmd_dbg, {0, 0, {-1}, "Show debug registers"}},
  {"hard_reset", cmd_hard_reset, {0, 0, {-1}, "Perform hard reset: turn the system off, set cmd/data buffer resets to 0x1FFFF, then to 0"}},
  {"exit", cmd_exit, {0, 0, {-1}, "Exit the program"}},
  {"set_boot_test_skip", cmd_set_boot_test_skip, {1, 1, {-1}, "Set boot test skip register to a 16-bit value"}},
  {"set_debug", cmd_set_debug, {1, 1, {-1}, "Set debug register to a 16-bit value"}},
  {"set_cmd_buf_reset", cmd_set_cmd_buf_reset, {1, 1, {-1}, "Set command buffer reset register to a 17-bit value"}},
  {"set_data_buf_reset", cmd_set_data_buf_reset, {1, 1, {-1}, "Set data buffer reset register to a 17-bit value"}},
  {"invert_mosi_clk", cmd_invert_mosi_clk, {0, 0, {-1}, "Invert MOSI SCK polarity register"}},
  {"invert_miso_clk", cmd_invert_miso_clk, {0, 0, {-1}, "Invert MISO SCK polarity register"}},
  
  // ===== DAC COMMANDS (from dac_commands.h) =====
  {"dac_cmd_fifo_sts", cmd_dac_cmd_fifo_sts, {1, 1, {-1}, "Show DAC command FIFO status for specified board (0-7)"}},
  {"dac_data_fifo_sts", cmd_dac_data_fifo_sts, {1, 1, {-1}, "Show DAC data FIFO status for specified board (0-7)"}},
  {"read_dac_dbg", cmd_read_dac_dbg, {1, 1, {FLAG_ALL, -1}, "Read and print debug information for DAC data from specified board (0-7)"}},
  {"dac_noop", cmd_dac_noop, {3, 3, {FLAG_CONTINUE, -1}, "Send DAC no-op command: <board> <\"trig\"|\"delay\"> <value> [--continue]"}},
  {"dac_cancel", cmd_dac_cancel, {1, 1, {-1}, "Send DAC cancel command to specified board (0-7)"}},
  {"write_dac_update", cmd_write_dac_update, {11, 11, {FLAG_CONTINUE, -1}, "Send DAC write update command: <board> <ch0> <ch1> <ch2> <ch3> <ch4> <ch5> <ch6> <ch7> <\"trig\"|\"delay\"> <value> [--continue]"}},
  {"do_dac_wr_ch", cmd_do_dac_wr_ch, {2, 2, {-1}, "Write DAC single channel: <channel> <value> (channel 0-63, board=ch/8, ch=ch%8)"}},
  {"stream_dac_commands_from_file", cmd_stream_dac_commands_from_file, {2, 3, {-1}, "Start DAC command streaming from waveform file: <board> <file_path> [loop_count] (supports * wildcards)"}},
  {"stop_dac_cmd_stream", cmd_stop_dac_cmd_stream, {1, 1, {-1}, "Stop DAC command streaming for specified board (0-7)"}},
  
  // ===== ADC COMMANDS (from adc_commands.h) =====
  {"adc_cmd_fifo_sts", cmd_adc_cmd_fifo_sts, {1, 1, {-1}, "Show ADC command FIFO status for specified board (0-7)"}},
  {"adc_data_fifo_sts", cmd_adc_data_fifo_sts, {1, 1, {-1}, "Show ADC data FIFO status for specified board (0-7)"}},
  {"read_adc_pair", cmd_read_adc_pair, {1, 1, {FLAG_ALL, -1}, "Read paired ADC channel sample(s) from specified board (0-7) [--all]"}},
  {"read_adc_single", cmd_read_adc_single, {1, 1, {FLAG_ALL, -1}, "Read single ADC channel data sample(s) from specified board (0-7) [--all]"}},
  {"read_adc_dbg", cmd_read_adc_dbg, {1, 1, {FLAG_ALL, -1}, "Read and print debug information for ADC data from specified board (0-7)"}},
  {"adc_noop", cmd_adc_noop, {3, 3, {FLAG_CONTINUE, -1}, "Send ADC no-op command: <board> <\"trig\"|\"delay\"> <value> [--continue]"}},
  {"adc_cancel", cmd_adc_cancel, {1, 1, {-1}, "Send ADC cancel command to specified board (0-7)"}},
  {"adc_set_ord", cmd_adc_set_ord, {9, 9, {-1}, "Set ADC channel order: <board> <ord0> <ord1> <ord2> <ord3> <ord4> <ord5> <ord6> <ord7> (each order value must be 0-7)"}},
  {"do_adc_simple_read", cmd_do_adc_simple_read, {3, 3, {-1}, "Perform simple ADC reads: <board> <loop_count> <delay_cycles> (reads ADC with delay mode)"}},
  {"do_adc_read", cmd_do_adc_read, {3, 3, {-1}, "Perform ADC read using loop command: <board> <loop_count> <delay_cycles> (sends loop_next command then single read command)"}},
  {"do_adc_rd_ch", cmd_do_adc_rd_ch, {1, 1, {-1}, "Read ADC single channel: <channel> (channel 0-63, board=ch/8, ch=ch%8)"}},
  {"read_adc_to_file", cmd_read_adc_to_file, {3, 3, {-1}, "Read ADC data to file: <board> <word_count> <file_path> (streams ADC data from FIFO to file)"}},
  {"stream_adc_data_to_file", cmd_stream_adc_data_to_file, {3, 3, {-1}, "Start ADC data streaming to file: <board> <word_count> <file_path> (alias for read_adc_to_file)"}},
  {"stream_adc_commands_from_file", cmd_stream_adc_commands_from_file, {2, 3, {FLAG_SIMPLE, -1}, "Start ADC command streaming from file: <board> <file_path> [loop_count] [--simple] (supports * wildcards)"}},
  {"stop_adc_data_stream", cmd_stop_adc_data_stream, {1, 1, {-1}, "Stop ADC data streaming for specified board (0-7)"}},
  {"stop_adc_cmd_stream", cmd_stop_adc_cmd_stream, {1, 1, {-1}, "Stop ADC command streaming for specified board (0-7)"}},
  
  // ===== TRIGGER COMMANDS (from trigger_commands.h) =====
  {"trig_cmd_fifo_sts", cmd_trig_cmd_fifo_sts, {0, 0, {-1}, "Show trigger command FIFO status"}},
  {"trig_data_fifo_sts", cmd_trig_data_fifo_sts, {0, 0, {-1}, "Show trigger data FIFO status"}},
  {"read_trig_data", cmd_read_trig_data, {0, 0, {FLAG_ALL, -1}, "Read trigger data sample(s)"}},
  {"sync_ch", cmd_trig_sync_ch, {0, 0, {-1}, "Send trigger synchronize channels command"}},
  {"force_trig", cmd_trig_force_trig, {0, 0, {-1}, "Send trigger force trigger command"}},
  {"trig_cancel", cmd_trig_cancel, {0, 0, {-1}, "Send trigger cancel command"}},
  {"trig_set_lockout", cmd_trig_set_lockout, {1, 1, {-1}, "Send trigger set lockout command with cycles (1 - 0x1FFFFFFF)"}},
  {"trig_delay", cmd_trig_delay, {1, 1, {-1}, "Send trigger delay command with cycles (0 - 0x1FFFFFFF)"}},
  {"trig_expect_ext", cmd_trig_expect_ext, {1, 1, {-1}, "Send trigger expect external command with count (0 - 0x1FFFFFFF)"}},
  
  // ===== EXPERIMENT COMMANDS (from experiment_commands.h) =====
  {"channel_test", cmd_channel_test, {3, 3, {-1}, "Set DAC and check ADC on individual channels: <board> <channel> <value> (board 0-7, channel 0-7, value -32767 to 32767)"}},
  {"waveform_test", cmd_waveform_test, {0, 0, {-1}, "Interactive waveform test: prompts for DAC/ADC files, loops, output file, and trigger lockout"}},
  
  // ===== COMMAND LOGGING/PLAYBACK (from command_handler.c) =====
  {"log_commands", cmd_log_commands, {1, 1, {-1}, "Start logging commands to file: <file_path>"}},
  {"stop_log", cmd_stop_log, {0, 0, {-1}, "Stop logging commands"}},
  {"load_commands", cmd_load_commands, {1, 1, {-1}, "Load and execute commands from file: <file_path> (0.25s delay between commands, supports * wildcards)"}},
  
  // Sentinel entry - marks end of table (must be last)
  {NULL, NULL, {0, 0, {-1}, NULL}}
};

// Forward declarations for helper functions
static void print_wrapped_line(const char* prefix, const char* text, const char* continuation_indent);

// Log command if logging is enabled
void log_command_if_enabled(command_context_t* ctx, const char* command_line) {
  if (ctx->logging_enabled && ctx->log_file != NULL) {
    fprintf(ctx->log_file, "%s\n", command_line);
    fflush(ctx->log_file);
  }
}

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
  // Resolve file pattern (handles glob patterns)
  char resolved_path[1024];
  if (resolve_file_pattern(args[0], resolved_path, sizeof(resolved_path)) != 0) {
    return -1;
  }
  
  // Open file for reading
  FILE* file = fopen(resolved_path, "r");
  if (file == NULL) {
    fprintf(stderr, "Failed to open command file '%s' for reading: %s\n", resolved_path, strerror(errno));
    return -1;
  }
  
  printf("Loading and executing commands from file '%s'...\n", resolved_path);
  
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
  printf("Successfully executed %d commands from file '%s'.\n", commands_executed, resolved_path);
  return 0;
}

// Helper function for printing wrapped lines
static void print_wrapped_line(const char* prefix, const char* text, const char* continuation_indent) {
  const int max_line_length = 80;
  int prefix_len = strlen(prefix);
  int available_width = max_line_length - prefix_len;
  
  if (available_width <= 0) {
    printf("%s%s\n", prefix, text);
    return;
  }
  
  printf("%s", prefix);
  
  const char* current = text;
  int current_line_length = prefix_len;
  bool first_line = true;
  
  while (*current) {
    // Find the next word boundary
    const char* word_start = current;
    while (*current && *current != ' ') current++;
    int word_length = current - word_start;
    
    // Skip spaces
    while (*current == ' ') current++;
    
    // Check if we need to wrap
    if (!first_line && current_line_length + word_length + 1 > max_line_length) {
      printf("\n%s", continuation_indent);
      current_line_length = strlen(continuation_indent);
      first_line = false;
    } else if (!first_line) {
      printf(" ");
      current_line_length++;
    }
    
    // Print the word
    for (int i = 0; i < word_length; i++) {
      printf("%c", word_start[i]);
    }
    current_line_length += word_length;
    first_line = false;
  }
  printf("\n");
}

// Print help for a specific command
void print_command_help(const char* command_name) {
  command_entry_t* cmd = find_command(command_name);
  if (cmd == NULL) {
    printf("Unknown command: %s\n", command_name);
    return;
  }
  
  printf("Usage: %s", command_name);
  
  // Show argument requirements
  if (cmd->info.min_args > 0) {
    for (int i = 0; i < cmd->info.min_args; i++) {
      printf(" <arg%d>", i + 1);
    }
    
    // Show optional arguments
    if (cmd->info.max_args > cmd->info.min_args) {
      for (int i = cmd->info.min_args; i < cmd->info.max_args; i++) {
        printf(" [arg%d]", i + 1);
      }
    }
  }
  
  // Show valid flags
  bool has_flags = false;
  for (int i = 0; i < MAX_FLAGS && cmd->info.valid_flags[i] != -1; i++) {
    if (!has_flags) {
      printf(" [flags]");
      has_flags = true;
      break;
    }
  }
  
  printf("\n");
  
  // Use print_wrapped_line for the description
  print_wrapped_line("Description: ", cmd->info.description, "             ");
  
  // Show valid flags if any
  if (has_flags) {
    printf("Valid flags:");
    for (int i = 0; i < MAX_FLAGS && cmd->info.valid_flags[i] != -1; i++) {
      switch (cmd->info.valid_flags[i]) {
        case FLAG_ALL:
          printf(" --all");
          break;
        case FLAG_CONTINUE:
          printf(" --continue");
          break;
        case FLAG_SIMPLE:
          printf(" --simple");
          break;
      }
    }
    printf("\n");
  }
}

// Main help printing function
void print_help(void) {
  printf("\nAvailable commands:\n");
  printf("==================\n\n");
  
  // Count total commands first
  int total_commands = 0;
  while (command_table[total_commands].name != NULL) {
    total_commands++;
  }
  
  // Track which commands have been printed
  bool printed[total_commands];
  for (int i = 0; i < total_commands; i++) {
    printed[i] = false;
  }
  
  // Group commands by category for better organization
  printf("System Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "help") || strstr(command_table[i].name, "verbose") ||
        strstr(command_table[i].name, "on") || strstr(command_table[i].name, "off") ||
        strstr(command_table[i].name, "sts") || strstr(command_table[i].name, "dbg") ||
        strstr(command_table[i].name, "hard_reset") || strstr(command_table[i].name, "exit")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  printf("\nConfiguration Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "set_") || strstr(command_table[i].name, "invert_")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  printf("\nDAC Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "dac")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  printf("\nADC Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "adc")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  printf("\nTrigger Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "trig") || strstr(command_table[i].name, "sync_ch") ||
        strstr(command_table[i].name, "force_trig")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }

  printf("\nExperiment Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "channel_test") || strstr(command_table[i].name, "waveform_test")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  printf("\nLogging and Loading Commands:\n");
  for (int i = 0; i < total_commands; i++) {
    if (strstr(command_table[i].name, "log_commands") || strstr(command_table[i].name, "stop_log") ||
        strstr(command_table[i].name, "load_commands")) {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
      printed[i] = true;
    }
  }
  
  // Print any remaining commands that weren't categorized
  bool has_other_commands = false;
  for (int i = 0; i < total_commands; i++) {
    if (!printed[i]) {
      if (!has_other_commands) {
        printf("\nOther Commands:\n");
        has_other_commands = true;
      }
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "  %-20s ", command_table[i].name);
      print_wrapped_line(prefix, command_table[i].info.description, "                         ");
    }
  }
  
  printf("\nFlags:\n");
  printf("  --all        Read all available data from FIFO\n");
  printf("  --continue   Continue flag for certain commands\n");
  printf("  --simple     Simple mode for certain commands\n");
  printf("\n");
}

// Help command implementation
int cmd_help(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  print_help();
  return 0;
}

// Command parsing functions
int parse_command_line(const char* line, const char** args, int* arg_count, command_flag_t* flags, int* flag_count) {
  // Basic implementation - would need full parsing logic
  *arg_count = 0;
  *flag_count = 0;
  
  char* line_copy = strdup(line);
  char* token = strtok(line_copy, " \t\n");
  
  while (token != NULL && *arg_count < MAX_ARGS) {
    if (token[0] == '-' && token[1] == '-') {
      // Handle flags
      if (strcmp(token, "--all") == 0) {
        flags[(*flag_count)++] = FLAG_ALL;
      } else if (strcmp(token, "--continue") == 0) {
        flags[(*flag_count)++] = FLAG_CONTINUE;
      } else if (strcmp(token, "--simple") == 0) {
        flags[(*flag_count)++] = FLAG_SIMPLE;
      }
    } else {
      args[(*arg_count)++] = strdup(token);
    }
    token = strtok(NULL, " \t\n");
  }
  
  free(line_copy);
  return 0;
}

command_entry_t* find_command(const char* name) {
  for (int i = 0; command_table[i].name != NULL; i++) {
    if (strcmp(command_table[i].name, name) == 0) {
      return &command_table[i];
    }
  }
  return NULL;
}

int execute_command(const char* line, command_context_t* ctx) {
  const char* args[MAX_ARGS];
  command_flag_t flags[MAX_FLAGS];
  int arg_count = 0;
  int flag_count = 0;
  
  if (parse_command_line(line, args, &arg_count, flags, &flag_count) < 0) {
    return -1;
  }
  
  if (arg_count == 0) {
    return 0; // Empty command
  }
  
  command_entry_t* cmd = find_command(args[0]);
  if (cmd == NULL) {
    printf("Unknown command: %s\n", args[0]);
    printf("Type 'help' to see all available commands.\n");
    return -1;
  }
  
  // Check argument count
  int cmd_args = arg_count - 1; // Exclude command name
  if (cmd_args < cmd->info.min_args || cmd_args > cmd->info.max_args) {
    printf("Error: Command '%s' requires %d", args[0], cmd->info.min_args);
    if (cmd->info.min_args != cmd->info.max_args) {
      printf("-%d", cmd->info.max_args);
    }
    printf(" arguments, got %d\n", cmd_args);
    printf("\n");
    print_command_help(args[0]);
    return -1;
  }
  
  // Log command if enabled
  log_command_if_enabled(ctx, line);
  
  // Execute command
  return cmd->handler(&args[1], cmd_args, flags, flag_count, ctx);
}
