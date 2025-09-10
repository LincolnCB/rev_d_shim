#ifndef COMMAND_HELPER_H
#define COMMAND_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>

#include "sys_ctrl.h"
#include "sys_sts.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "trigger_ctrl.h"
#include "spi_clk_ctrl.h"

#define MAX_ARGS 16     // Maximum command arguments (including command name)
#define MAX_FLAGS 5     // Maximum command flags

// Supported command flags
typedef enum {
  FLAG_ALL,
  FLAG_VERBOSE,
  FLAG_CONTINUE,
  FLAG_SIMPLE
} command_flag_t;

// Global context passed to all command handlers
typedef struct command_context {
  // Hardware control interfaces
  struct sys_ctrl_t* sys_ctrl;
  struct spi_clk_ctrl_t* spi_clk_ctrl;
  struct sys_sts_t* sys_sts;
  struct dac_ctrl_t* dac_ctrl;
  struct adc_ctrl_t* adc_ctrl;
  struct trigger_ctrl_t* trigger_ctrl;
  
  // System state
  bool* verbose;
  bool* should_exit;
  
  // ADC streaming management
  pthread_t adc_stream_threads[8];      // Thread handles for ADC streaming
  bool adc_stream_running[8];           // Status of each ADC stream thread
  volatile bool adc_stream_stop[8];     // Stop signals for each ADC stream thread
  
  // DAC streaming management
  pthread_t dac_stream_threads[8];      // Thread handles for DAC streaming
  bool dac_stream_running[8];           // Status of each DAC stream thread
  volatile bool dac_stream_stop[8];     // Stop signals for each DAC stream thread
  
  // Command logging
  FILE* log_file;                       // File handle for command logging
  bool logging_enabled;                 // Whether command logging is active
} command_context_t;

// Basic parsing and validation utilities
uint32_t parse_value(const char* str, char** endptr);
int parse_board_number(const char* str);
int has_flag(const command_flag_t* flags, int flag_count, command_flag_t target_flag);

// Common validation functions
int parse_trigger_mode(const char* mode_str, const char* value_str, bool* is_trigger, uint32_t* value);
int validate_board_number(const char* board_str);
int validate_channel_number(const char* channel_str, int* board, int* channel);

// File path resolution with glob support
int resolve_file_path(const char* pattern, char* resolved_path, size_t resolved_path_size);
int resolve_file_pattern(const char* pattern, char* resolved_path, size_t resolved_path_size);

// File path utilities
void clean_and_expand_path(const char* input_path, char* full_path, size_t full_path_size);
void set_file_permissions(const char* file_path, bool verbose);

// Data conversion utilities
int16_t convert_to_signed_16bit(uint32_t raw_value);

// Display/output helper functions
void print_data_words(uint32_t data);
void print_trigger_data(uint64_t data);

#endif // COMMAND_HELPER_H
