#include <stdio.h> // For printf and perror functions
#include <stdlib.h> // For exit function and NULL definition etc.
#include <inttypes.h> // For PRIx32 format specifier
#include <unistd.h> // For read, write, close functions
#include <fcntl.h> // For open function
#include <pthread.h> // For pthread functions
#include "sys_sts.h"
#include "map_memory.h"

// Function to create system status structure
struct sys_sts_t create_sys_sts(bool verbose) {
  struct sys_sts_t sys_sts;

  // Map system status register
  volatile uint32_t *sys_sts_ptr = map_32bit_memory(SYS_STS, SYS_STS_WORDCOUNT, "System Status", verbose);
  if (sys_sts_ptr == NULL) {
    fprintf(stderr, "Failed to map system status memory region.\n");
    exit(EXIT_FAILURE);
  }

  // Initialize the system status structure with the mapped memory addresses
  sys_sts.hw_status_reg = sys_sts_ptr + HW_STS_REG_OFFSET;

  // Initialize FIFO status pointers for each board
  for (int i = 0; i < 8; i++) {
    sys_sts.dac_cmd_fifo_sts[i] = sys_sts_ptr + DAC_CMD_FIFO_STS_OFFSET(i);
    sys_sts.dac_data_fifo_sts[i] = sys_sts_ptr + DAC_DATA_FIFO_STS_OFFSET(i);
    sys_sts.adc_cmd_fifo_sts[i] = sys_sts_ptr + ADC_CMD_FIFO_STS_OFFSET(i);
    sys_sts.adc_data_fifo_sts[i] = sys_sts_ptr + ADC_DATA_FIFO_STS_OFFSET(i);
  }

  // Initialize trigger FIFO status pointers
  sys_sts.trig_cmd_fifo_sts = sys_sts_ptr + TRIG_CMD_FIFO_STS_OFFSET;
  sys_sts.trig_data_fifo_sts = sys_sts_ptr + TRIG_DATA_FIFO_STS_OFFSET;

  // Initialize SPI clock frequency pointer
  sys_sts.clk_freq_hz = sys_sts_ptr + CLK_FREQ_OFFSET;

  // Initialize SPI source clock frequency pointer
  sys_sts.source_clk_freq_hz = sys_sts_ptr + SOURCE_CLK_FREQ_OFFSET;

  // Initialize trigger counter pointer
  sys_sts.trig_counter = sys_sts_ptr + TRIG_COUNTER_OFFSET;

  // Initialize debug register
  sys_sts.debug = sys_sts_ptr + DEBUG_REG_OFFSET;

  // Initialize DAC and ADC minimum delay time registers
  sys_sts.dac_min_delay_time = sys_sts_ptr + DEBUG_DAC_MIN_DELAY_TIME_OFFSET;
  sys_sts.adc_min_delay_time = sys_sts_ptr + DEBUG_ADC_MIN_DELAY_TIME_OFFSET;

  // Initialize last command and command counter registers for each board
  for (int i = 0; i < 8; i++) {
    sys_sts.last_received_dac_cmd[i] = sys_sts_ptr + DAC_LAST_RECEIVED_CMD_OFFSET(i);
    sys_sts.last_received_adc_cmd[i] = sys_sts_ptr + ADC_LAST_RECEIVED_CMD_OFFSET(i);
    sys_sts.dac_cmds_since_reset[i] = sys_sts_ptr + DAC_CMDS_SINCE_RESET_OFFSET(i);
    sys_sts.adc_cmds_since_reset[i] = sys_sts_ptr + ADC_CMDS_SINCE_RESET_OFFSET(i);
  }

  return sys_sts;
}

// Get hardware status register value
uint32_t sys_sts_get_hw_status(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading hardware status register...\n");
    printf("Hardware status raw: 0x%" PRIx32 "\n", *(sys_sts->hw_status_reg));
  }
  return *(sys_sts->hw_status_reg);
}

// Get SPI clock frequency in Hz
uint32_t sys_sts_get_clk_freq_hz(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading SPI clock frequency register...\n");
    printf("SPI clock frequency raw: 0x%"PRIx32 " (%" PRIu32 ")\n", *(sys_sts->clk_freq_hz), *(sys_sts->clk_freq_hz));
  }
  return *(sys_sts->clk_freq_hz);
}

// Get SPI source clock frequency in Hz
uint32_t sys_sts_get_source_clk_freq_hz(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading SPI source clock frequency register...\n");
    printf("SPI source clock frequency raw: 0x%" PRIx32 " (%" PRIu32 ")\n", *(sys_sts->source_clk_freq_hz), *(sys_sts->source_clk_freq_hz));
  }
  return *(sys_sts->source_clk_freq_hz);
}

// Get FIFO status from a status pointer
uint32_t get_fifo_status(volatile uint32_t *fifo_sts_ptr, const char *fifo_name, bool verbose) {
  if (verbose) {
    printf("Reading %s FIFO status register...\n", fifo_name);
    printf("%s FIFO status raw: 0x%08" PRIx32 "\n", fifo_name, *fifo_sts_ptr);
  }
  return *fifo_sts_ptr;
}

// Interpret and print hardware status
void print_hw_status(uint32_t hw_status, bool verbose) {
  bool print_status = verbose;
  bool print_board_number = verbose;
  if (verbose) printf("Raw hardware state code: 0x%" PRIx32 "\n", HW_STS_STATE(hw_status));
  switch (HW_STS_STATE(hw_status)) {
    case S_IDLE:
      printf("State: Idle (Waiting For Control Board Enable)\n");
      break;
    case S_CONFIRM_SPI_RST:
      printf("State: Confirm SPI Reset\n");
      break;
    case S_POWER_ON_CRTL_BRD:
      printf("State: Power On Control Board\n");
      break;
    case S_CONFIRM_SPI_START:
      printf("State: Confirm SPI Start\n");
      break;
    case S_WAIT_FOR_POW_EN:
      printf("State: Waiting For Power Board Enable\n");
      break;
    case S_POWER_ON_AMP_BRD:
      printf("State: Power On Amplifier Board\n");
      break;
    case S_AMP_POWER_WAIT:
      printf("State: Amplifier Power Wait\n");
      break;
    case S_RUNNING:
      printf("State: Running\n");
      break;
    case S_HALTING:
      printf("State: Halting\n");
      print_status = true;
      break;
    case S_HALTED:
      printf("State: Halted\n");
      print_status = true;
      break;
    default:
      printf("State: Unknown (0x%" PRIx32 ")\n", HW_STS_STATE(hw_status));
      break;
  }
  if (verbose) printf("Raw hardware status code: 0x%" PRIx32 "\n", HW_STS_CODE(hw_status));
  if (print_status) {
    switch (HW_STS_CODE(hw_status)) {
      case STS_EMPTY:
        printf("Status: Empty\n");
        break;
      case STS_OK:
        printf("Status: OK\n");
        break;
      case STS_PS_SHUTDOWN:
        printf("Status: Processing system shutdown\n");
        break;
      case STS_SPI_RESET_TIMEOUT:
        printf("Status: SPI initialization timeout\n");
        break;
      case STS_SPI_START_TIMEOUT:
        printf("Status: SPI start timeout\n");
        break;
      case STS_LOCK_VIOL:
        printf("Status: Configuration lock violation\n");
        break;
      case STS_CTRL_EN_OOB:
        printf("Status: Control board enable register out of bounds\n");
        break;
      case STS_POW_EN_OOB:
        printf("Status: Power board enable register out of bounds\n");
        break;
      case STS_CMD_BUF_RESET_OOB:
        printf("Status: Command buffer reset out of bounds\n");
        break;
      case STS_DATA_BUF_RESET_OOB:
        printf("Status: Data buffer reset out of bounds\n");
        break;
      case STS_THRESH_VAL_OOB:
        printf("Status: Threshold average out of bounds\n");
        break;
      case STS_THRESH_WINDOW_OOB:
        printf("Status: Threshold window out of bounds\n");
        break;
      case STS_THRESH_EN_OOB:
        printf("Status: Threshold enable register out of bounds\n");
        break;
      case STS_BOOT_TEST_SKIP_OOB:
        printf("Status: Boot test skip out of bounds\n");
        break;
      case STS_DEBUG_OOB:
        printf("Status: Debug out of bounds\n");
        break;
      case STS_DAC_CAL_INIT_OOB:
        printf("Status: DAC calibration initial value out of bounds\n");
        break;
      case STS_CLK_LOCKED_FAIL:
        printf("Status: SPI clock manager PLL not locked\n");
        break;
      case STS_CLK_RECONF_IN_PROG:
        printf("Status: SPI clock reconfiguration in progress\n");
        break;
      case STS_CLK_DIV_0:
        printf("Status: SPI clock divided by zero\n");
        break;
      case STS_CLK_OOB:
        printf("Status: SPI clock frequency out of bounds\n");
        break;
      case STS_SHUTDOWN_SENSE:
        printf("Status: Shutdown sense detected\n");
        print_board_number = true;
        break;
      case STS_EXT_SHUTDOWN:
        printf("Status: External shutdown triggered\n");
        break;
      case STS_OVER_THRESH:
        printf("Status: DAC over threshold\n");
        print_board_number = true;
        break;
      case STS_THRESH_UNDERFLOW:
        printf("Status: DAC threshold FIFO underflow\n");
        print_board_number = true;
        break;
      case STS_THRESH_OVERFLOW:
        printf("Status: DAC threshold FIFO overflow\n");
        print_board_number = true;
        break;
      case STS_BAD_TRIG_CMD:
        printf("Status: Bad trigger command\n");
        break;
      case STS_TRIG_CMD_BUF_OVERFLOW:
        printf("Status: Trigger command buffer overflow\n");
        break;
      case STS_TRIG_DATA_BUF_UNDERFLOW:
        printf("Status: Trigger data buffer underflow\n");
        break;
      case STS_TRIG_DATA_BUF_OVERFLOW:
        printf("Status: Trigger data buffer overflow\n");
        break;
      case STS_DAC_BOOT_FAIL:
        printf("Status: DAC boot failure\n");
        print_board_number = true;
        break;
      case STS_BAD_DAC_CMD:
        printf("Status: Bad DAC command\n");
        print_board_number = true;
        break;
      case STS_DAC_CAL_OOB:
        printf("Status: DAC calibration out of bounds\n");
        print_board_number = true;
        break;
      case STS_DAC_VAL_OOB:
        printf("Status: DAC value out of bounds\n");
        print_board_number = true;
        break;
      case STS_DAC_CMD_BUF_UNDERFLOW:
        printf("Status: DAC command buffer underflow\n");
        print_board_number = true;
        break;
      case STS_DAC_CMD_BUF_OVERFLOW:
        printf("Status: DAC command buffer overflow\n");
        print_board_number = true;
        break;
      case STS_DAC_DATA_BUF_UNDERFLOW:
        printf("Status: DAC data buffer underflow\n");
        print_board_number = true;
        break;
      case STS_DAC_DATA_BUF_OVERFLOW:
        printf("Status: DAC data buffer overflow\n");
        print_board_number = true;
        break;
      case STS_UNEXP_DAC_TRIG:
        printf("Status: Unexpected DAC trigger\n");
        print_board_number = true;
        break;
      case STS_LDAC_MISALIGN:
        printf("Status: LDAC misalignment error\n");
        print_board_number = true;
        break;
      case STS_DAC_DELAY_TOO_SHORT:
        printf("Status: DAC delay too short\n");
        print_board_number = true;
        break;
      case STS_ADC_BOOT_FAIL:
        printf("Status: ADC boot failure\n");
        print_board_number = true;
        break;
      case STS_BAD_ADC_CMD:
        printf("Status: Bad ADC command\n");
        print_board_number = true;
        break;
      case STS_ADC_CMD_BUF_UNDERFLOW:
        printf("Status: ADC command buffer underflow\n");
        print_board_number = true;
        break;
      case STS_ADC_CMD_BUF_OVERFLOW:
        printf("Status: ADC command buffer overflow\n");
        print_board_number = true;
        break;
      case STS_ADC_DATA_BUF_UNDERFLOW:
        printf("Status: ADC data buffer underflow\n");
        print_board_number = true;
        break;
      case STS_ADC_DATA_BUF_OVERFLOW:
        printf("Status: ADC data buffer overflow\n");
        print_board_number = true;
        break;
      case STS_UNEXP_ADC_TRIG:
        printf("Status: Unexpected ADC trigger\n");
        print_board_number = true;
        break;
      case STS_ADC_DELAY_TOO_SHORT:
        printf("Status: ADC delay too short\n");
        print_board_number = true;
        break;
      default:
        printf("Status: Unknown (0x%" PRIx32 ")\n", HW_STS_CODE(hw_status));
        break;
    }
  }
  if (print_board_number) {
    printf("Board Number: %u\n", HW_STS_BOARD(hw_status));
  }
}

// Print SPI clock frequency in Hz and MHz
void print_clk_freq(uint32_t freq_hz, bool verbose) {
  if (verbose) {
    printf("SPI clock frequency: %" PRIu32 " Hz (%.3f MHz)\n", freq_hz, freq_hz / 1000000.0);
  } else {
    printf("SPI Clock: %.3f MHz\n", freq_hz / 1000000.0);
  }
}

// Print debug register
void print_debug_register(struct sys_sts_t *sys_sts) {
  uint32_t value = *(sys_sts->debug);
  printf("Debug Register: 0x%08" PRIx32 " (0b", value);
  for (int bit = 31; bit >= 0; bit--) {
    printf("%u", (value >> bit) & 1);
  }
  printf(")\n");

  // Print specific bit interpretations
  printf("  SPI Clock Locked: %s\n", (value & (1 << DEBUG_CLK_LOCKED_BIT)) ? "Yes" : "No");
  printf("  SPI Off: %s\n", (value & (1 << DEBUG_SPI_OFF_BIT)) ? "Yes" : "No");
  printf("  DAC ~CS High Time: %u cycles\n", DEBUG_DAC_CS_HIGH_TIME(value));
  printf("  ADC ~CS High Time: %u cycles\n", DEBUG_ADC_CS_HIGH_TIME(value));
  printf("  SPI Snoop Reconfiguration State: %u\n", DEBUG_SPI_SNOOP_RECONF_STATE(value));
  switch(DEBUG_SPI_SNOOP_RECONF_STATE(value)) {
    case SNOOP_STATE_IDLE:
      printf("    (Idle)\n");
      break;
    case SNOOP_STATE_STARTING:
      printf("    (Starting Reconfiguration)\n");
      break;
    case SNOOP_STATE_GET_FULL_DIV_1:
      printf("    (Getting Full Divider Part 1)\n");
      break;
    case SNOOP_STATE_GET_FULL_DIV_2:
      printf("    (Getting Full Divider Part 2)\n");
      break;
    case SNOOP_STATE_GET_FULL_MULT:
      printf("    (Getting Full Multiplier)\n");
      break;
    case SNOOP_STATE_CALC_MULT:
      printf("    (Calculating Multiplication)\n");
      break;
    case SNOOP_STATE_CALC_DIV:
      printf("    (Calculating Division)\n");
      break;
    default:
      printf("    (Unknown State)\n");
      break;
  }
}

// Get DAC command FIFO status for a specific board
uint32_t sys_sts_get_dac_cmd_fifo_status(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for DAC command FIFO status. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  return get_fifo_status(sys_sts->dac_cmd_fifo_sts[board], "DAC Command", verbose);
}

// Get DAC data FIFO status for a specific board
uint32_t sys_sts_get_dac_data_fifo_status(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for DAC data FIFO status. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  return get_fifo_status(sys_sts->dac_data_fifo_sts[board], "DAC Data", verbose);
}

// Get ADC command FIFO status for a specific board
uint32_t sys_sts_get_adc_cmd_fifo_status(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for ADC command FIFO status. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  return get_fifo_status(sys_sts->adc_cmd_fifo_sts[board], "ADC Command", verbose);
}

// Get ADC data FIFO status for a specific board
uint32_t sys_sts_get_adc_data_fifo_status(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for ADC data FIFO status. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }
  return get_fifo_status(sys_sts->adc_data_fifo_sts[board], "ADC Data", verbose);
}

// Get trigger command FIFO status
uint32_t sys_sts_get_trig_cmd_fifo_status(struct sys_sts_t *sys_sts, bool verbose) {
  return get_fifo_status(sys_sts->trig_cmd_fifo_sts, "Trigger Command", verbose);
}

// Get trigger data FIFO status
uint32_t sys_sts_get_trig_data_fifo_status(struct sys_sts_t *sys_sts, bool verbose) {
  return get_fifo_status(sys_sts->trig_data_fifo_sts, "Trigger Data", verbose);
}

// Get trigger counter value
uint32_t sys_sts_get_trig_count(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading trigger counter register...\n");
    printf("Trigger counter raw: 0x%" PRIx32 "\n", *(sys_sts->trig_counter));
  }
  return *(sys_sts->trig_counter);
}

// Get debug register value
uint32_t sys_sts_get_debug(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading debug register...\n");
    printf("Debug register raw: 0x%" PRIx32 "\n", *(sys_sts->debug));
  }
  return *(sys_sts->debug);
}

// Get DAC minimum delay time in SPI clock cycles
uint32_t sys_sts_get_dac_min_delay_time(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading DAC 'delay too short' time register...\n");
    printf("DAC 'delay too short' time raw: 0x%" PRIx32 "\n", *(sys_sts->dac_min_delay_time));
  }
  return *(sys_sts->dac_min_delay_time);
}

// Get ADC minimum delay time in SPI clock cycles
uint32_t sys_sts_get_adc_min_delay_time(struct sys_sts_t *sys_sts, bool verbose) {
  if (verbose) {
    printf("Reading ADC 'delay too short' time register...\n");
    printf("ADC 'delay too short' time raw: 0x%" PRIx32 "\n", *(sys_sts->adc_min_delay_time));
  }
  return *(sys_sts->adc_min_delay_time);
}

// Get last received DAC command for a specific board
uint32_t sys_sts_get_last_received_dac_cmd(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for last received DAC command. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  uint32_t value = *(sys_sts->last_received_dac_cmd[board]);
  if (verbose) {
    printf("Reading last received DAC command for board %u...\n", board);
    printf("Last received DAC command raw: 0x%08" PRIx32 "\n", value);
  }
  return value;
}

// Get last received ADC command for a specific board
uint32_t sys_sts_get_last_received_adc_cmd(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for last received ADC command. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  uint32_t value = *(sys_sts->last_received_adc_cmd[board]);
  if (verbose) {
    printf("Reading last received ADC command for board %u...\n", board);
    printf("Last received ADC command raw: 0x%08" PRIx32 "\n", value);
  }
  return value;
}

// Get DAC command count since reset for a specific board
uint32_t sys_sts_get_dac_cmds_since_reset(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for DAC command count since reset. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  uint32_t value = *(sys_sts->dac_cmds_since_reset[board]);
  if (verbose) {
    printf("Reading DAC command count since reset for board %u...\n", board);
    printf("DAC command count since reset: %" PRIu32 " (0x%08" PRIx32 ")\n", value, value);
  }
  return value;
}

// Get ADC command count since reset for a specific board
uint32_t sys_sts_get_adc_cmds_since_reset(struct sys_sts_t *sys_sts, uint8_t board, bool verbose) {
  if (board >= 8) {
    fprintf(stderr, "Invalid board number %u for ADC command count since reset. Must be 0-7.\n", board);
    exit(EXIT_FAILURE);
  }

  uint32_t value = *(sys_sts->adc_cmds_since_reset[board]);
  if (verbose) {
    printf("Reading ADC command count since reset for board %u...\n", board);
    printf("ADC command count since reset: %" PRIu32 " (0x%08" PRIx32 ")\n", value, value);
  }
  return value;
}



// Print FIFO status details
void print_fifo_status(uint32_t fifo_status, const char *fifo_name) {
  printf("%s FIFO Status:\n", fifo_name);
  printf("  Present: %s\n", FIFO_PRESENT(fifo_status) ? "Yes" : "No");

  // Only print detailed status if FIFO is present
  if (FIFO_PRESENT(fifo_status)) {
    printf("  Word Count: %u\n", FIFO_STS_WORD_COUNT(fifo_status));
    printf("  Full: %s\n", FIFO_STS_FULL(fifo_status) ? "Yes" : "No");
    printf("  Almost Full: %s\n", FIFO_STS_ALMOST_FULL(fifo_status) ? "Yes" : "No");
    printf("  Empty: %s\n", FIFO_STS_EMPTY(fifo_status) ? "Yes" : "No");
    printf("  Almost Empty: %s\n", FIFO_STS_ALMOST_EMPTY(fifo_status) ? "Yes" : "No");
  }
}

// Structure to pass data to the interrupt monitoring thread
struct hw_manager_irq_data {
  struct sys_sts_t *sys_sts;
  bool verbose;
};

// Thread function to monitor hardware manager interrupt
static void *hw_manager_irq_thread_func(void *arg) {
  struct hw_manager_irq_data *irq_data = (struct hw_manager_irq_data *)arg;
  const char *uio_path = "/dev/uio0"; // Hardware manager interrupt is uio0
  int fd;
  uint32_t irq_count;
  uint32_t clear_value = 1;

  if (irq_data->verbose) {
    printf("Hardware manager interrupt monitor thread started\n");
  }

  // Open the UIO device for hardware manager interrupt
  fd = open(uio_path, O_RDWR);
  if (fd < 0) {
    perror("Failed to open hardware manager UIO device (/dev/uio0)");
    pthread_exit(NULL);
  }

  if (irq_data->verbose) {
    printf("Opened UIO device: %s\n", uio_path);
  }

  // Clear any pending interrupt at startup
  if (write(fd, &clear_value, sizeof(clear_value)) < 0) {
    perror("Failed to clear initial interrupt");
  } else if (irq_data->verbose) {
    printf("Cleared initial interrupt state\n");
  }

  // Continuous monitoring loop
  while (1) {
    // Wait for interrupt
    if (irq_data->verbose) {
      printf("Waiting for hardware manager interrupt...\n");
    }

    if (read(fd, &irq_count, sizeof(irq_count)) == sizeof(irq_count)) {
      printf("\nHardware manager interrupt received! (count: %u)\n", irq_count);

      // Get and print the hardware status
      uint32_t hw_status = sys_sts_get_hw_status(irq_data->sys_sts, irq_data->verbose);
      print_hw_status(hw_status, irq_data->verbose);

      // Clear the interrupt
      if (write(fd, &clear_value, sizeof(clear_value)) < 0) {
        perror("Failed to clear interrupt after handling");
      } else if (irq_data->verbose) {
        printf("Interrupt cleared successfully\n");
      }

      // Check if hardware status is "running" - if not, exit the monitoring loop
      if (HW_STS_STATE(hw_status) != S_RUNNING) {
        if (irq_data->verbose) {
          printf("Hardware is no longer running - exiting interrupt monitor\n");
        }
        break;
      }

      // If still running, continue monitoring for next interrupt
      if (irq_data->verbose) {
        printf("Hardware still running - continuing interrupt monitoring\n");
      }
    } else {
      perror("Failed to read from UIO device");
      break; // Exit on read error
    }
  }

  close(fd);

  if (irq_data->verbose) {
    printf("Hardware manager interrupt monitor thread exiting\n");
  }

  pthread_exit(NULL);
}

// Start hardware manager interrupt monitoring thread
int sys_sts_start_hw_manager_irq_monitor(struct sys_sts_t *sys_sts, bool verbose) {
  static struct hw_manager_irq_data irq_data;
  static pthread_t irq_thread;
  int result;

  // Setup thread data
  irq_data.sys_sts = sys_sts;
  irq_data.verbose = verbose;

  if (verbose) {
    printf("Starting hardware manager interrupt monitoring thread...\n");
  }

  // Create the interrupt monitoring thread
  result = pthread_create(&irq_thread, NULL, hw_manager_irq_thread_func, &irq_data);
  if (result != 0) {
    fprintf(stderr, "Failed to create hardware manager interrupt thread: %d\n", result);
    return -1;
  }

  // Detach the thread so it can clean up automatically when it exits
  result = pthread_detach(irq_thread);
  if (result != 0) {
    fprintf(stderr, "Failed to detach hardware manager interrupt thread: %d\n", result);
    return -1;
  }

  if (verbose) {
    printf("Hardware manager interrupt monitoring thread started successfully\n");
  }

  return 0;
}

