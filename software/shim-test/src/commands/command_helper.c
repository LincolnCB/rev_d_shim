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
#include "command_helper.h"

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

// Resolve file patterns with glob wildcards
int resolve_file_pattern(const char* pattern, char* resolved_path, size_t resolved_path_size) {
  glob_t glob_result;
  
  // Try to expand the pattern
  int glob_ret = glob(pattern, GLOB_ERR, NULL, &glob_result);
  
  if (glob_ret == 0 && glob_result.gl_pathc > 0) {
    if (glob_result.gl_pathc == 1) {
      // Single match - use it directly
      strncpy(resolved_path, glob_result.gl_pathv[0], resolved_path_size - 1);
      resolved_path[resolved_path_size - 1] = '\0';
      globfree(&glob_result);
      return 0;
    } else {
      // Multiple matches - prompt user to choose
      printf("Multiple files match pattern '%s':\n", pattern);
      for (size_t i = 0; i < glob_result.gl_pathc; i++) {
        printf("  %zu: %s\n", i + 1, glob_result.gl_pathv[i]);
      }
      
      printf("Enter your choice (1-%zu): ", glob_result.gl_pathc);
      fflush(stdout);
      
      int choice;
      if (scanf("%d", &choice) != 1 || choice < 1 || choice > (int)glob_result.gl_pathc) {
        printf("Invalid choice. Using first match: %s\n", glob_result.gl_pathv[0]);
        choice = 1;
      }
      
      // Use the selected match (convert to 0-based index)
      strncpy(resolved_path, glob_result.gl_pathv[choice - 1], resolved_path_size - 1);
      resolved_path[resolved_path_size - 1] = '\0';
      printf("Selected: %s\n", resolved_path);
      
      globfree(&glob_result);
      return 0;
    }
  } else {
    // No matches or error, use the pattern as-is
    strncpy(resolved_path, pattern, resolved_path_size - 1);
    resolved_path[resolved_path_size - 1] = '\0';
    if (glob_ret == 0) globfree(&glob_result);
    return -1;
  }
}

int resolve_file_path(const char* pattern, char* resolved_path, size_t resolved_path_size) {
  return resolve_file_pattern(pattern, resolved_path, resolved_path_size);
}

// Parse trigger mode string ("trig" or "delay") into boolean and validate value
int parse_trigger_mode(const char* mode_str, const char* value_str, bool* is_trigger, uint32_t* value) {
  if (strcmp(mode_str, "trig") == 0) {
    *is_trigger = true;
    char* endptr;
    *value = parse_value(value_str, &endptr);
    if (*endptr != '\0' || *value == 0 || *value > 0x1FFFFFFF) {
      printf("Error: Trigger value must be between 1 and 0x1FFFFFFF\n");
      return -1;
    }
  } else if (strcmp(mode_str, "delay") == 0) {
    *is_trigger = false;
    char* endptr;
    *value = parse_value(value_str, &endptr);
    if (*endptr != '\0' || *value > 0x1FFFFFFF) {
      printf("Error: Delay value must be between 0 and 0x1FFFFFFF\n");
      return -1;
    }
  } else {
    printf("Error: Trigger mode must be \"trig\" or \"delay\"\n");
    return -1;
  }
  return 0;
}

// Validate board number and return it, or -1 on error
int validate_board_number(const char* board_str) {
  int board = parse_board_number(board_str);
  if (board == -1) {
    printf("Error: Invalid board number '%s'. Must be 0-7.\n", board_str);
    return -1;
  }
  return board;
}

// Validate channel number (0-63) and return board and channel, or -1 on error
int validate_channel_number(const char* channel_str, int* board, int* channel) {
  char* endptr;
  int ch = strtol(channel_str, &endptr, 10);
  if (*endptr != '\0' || ch < 0 || ch > 63) {
    printf("Error: Invalid channel number '%s'. Must be 0-63.\n", channel_str);
    return -1;
  }
  *board = ch / 8;
  *channel = ch % 8;
  return 0;
}

// Print 64-bit trigger data breakdown
void print_trigger_data(uint64_t data) {
  uint32_t low_word = data & 0xFFFFFFFF;
  uint32_t high_word = (data >> 32) & 0xFFFFFFFF;
  printf("  Low 32 bits:  0x%08X\n", low_word);
  printf("  High 32 bits: 0x%08X\n", high_word);
  printf("  64-bit value: 0x%016" PRIX64 " (%" PRIu64 ")\n", data, data);
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
