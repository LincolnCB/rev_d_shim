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
  FLAG_SIMPLE,
  FLAG_BIN,
  FLAG_NO_RESET,
  FLAG_NO_CAL
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
  pthread_t adc_data_stream_threads[8];      // Thread handles for ADC data streaming (reading to file)
  bool adc_data_stream_running[8];           // Status of each ADC data stream thread
  volatile bool adc_data_stream_stop[8];     // Stop signals for each ADC data stream thread
  pthread_t adc_cmd_stream_threads[8];       // Thread handles for ADC command streaming (from file)
  bool adc_cmd_stream_running[8];            // Status of each ADC command stream thread
  volatile bool adc_cmd_stream_stop[8];      // Stop signals for each ADC command stream thread
  
  // DAC streaming management
  pthread_t dac_cmd_stream_threads[8];      // Thread handles for DAC command streaming
  bool dac_cmd_stream_running[8];           // Status of each DAC command stream thread
  volatile bool dac_cmd_stream_stop[8];     // Stop signals for each DAC command stream thread
  pthread_t dac_debug_stream_threads[8];    // Thread handles for DAC debug data streaming (reading to file)
  bool dac_debug_stream_running[8];         // Status of each DAC debug data stream thread
  volatile bool dac_debug_stream_stop[8];   // Stop signals for each DAC debug data stream thread
  
  // Trigger streaming management
  pthread_t trig_data_stream_thread;        // Thread handle for trigger data streaming
  bool trig_data_stream_running;            // Status of trigger data stream thread
  volatile bool trig_data_stream_stop;      // Stop signal for trigger data stream thread
  
  // Fieldmap data collection management
  pthread_t fieldmap_thread;                // Thread handle for fieldmap data collection
  bool fieldmap_running;                    // Status of fieldmap thread
  volatile bool fieldmap_stop;              // Stop signal for fieldmap thread
  
  // Command logging
  FILE* log_file;                       // File handle for command logging
  bool logging_enabled;                 // Whether command logging is active
  
  // ADC bias calibration storage (64 channels, 8 boards * 8 channels each)
  double adc_bias[64];                  // ADC bias values for each channel (0-63)
  bool adc_bias_valid[64];              // Whether each ADC bias value is valid
  double adc_bias_previous[64];         // Previous ADC bias values for comparison
  bool adc_bias_previous_valid[64];     // Whether each previous ADC bias value is valid
} command_context_t;

// Helper function to convert Amps to signed DAC units
// Maps -5.0A -> -32767, 0A -> 0, 5.0A -> 32767
int16_t amps_to_dac(double amps);
// Helper function to convert signed DAC units to Amps
double dac_to_amps(int16_t dac_value);
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

// File selection utilities
int prompt_file_selection(const char* prompt_text, const char* default_file,
                         char* resolved_path, size_t resolved_path_size);

// Display/output helper functions
void print_trigger_data(uint64_t data);

#endif // COMMAND_HELPER_H
