#include <stdio.h> // For printf and perror functions
#include <stdlib.h> // For exit function and NULL definition etc.
#include <inttypes.h> // For PRIx32 format specifier
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
    sys_sts.adc_cmd_fifo_sts[i] = sys_sts_ptr + ADC_CMD_FIFO_STS_OFFSET(i);
    sys_sts.adc_data_fifo_sts[i] = sys_sts_ptr + ADC_DATA_FIFO_STS_OFFSET(i);
  }
  
  // Initialize trigger FIFO status pointers
  sys_sts.trig_cmd_fifo_sts = sys_sts_ptr + TRIG_CMD_FIFO_STS_OFFSET;
  sys_sts.trig_data_fifo_sts = sys_sts_ptr + TRIG_DATA_FIFO_STS_OFFSET;

  // Initialize debug registers
  sys_sts.debug[0] = sys_sts_ptr + DEBUG_REG_OFFSET(0); // Assuming only one debug register for now
  
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

// Interpret and print hardware status
void print_hw_status(uint32_t hw_status, bool verbose) {
  bool print_status = verbose;
  bool print_board_number = verbose;
  if (verbose) printf("Raw hardware state code: 0x%" PRIx32 "\n", HW_STS_STATE(hw_status));
  switch (HW_STS_STATE(hw_status)) {
    case S_IDLE:
      printf("State: Idle\n");
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
    case S_POWER_ON_AMP_BRD:
      printf("State: Power On Amplifier Board\n");
      break;
    case S_AMP_POWER_WAIT:
      printf("State: Waiting for Amplifier Power\n");
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
      case STS_OK:
        printf("Status: OK\n");
        break;
      case STS_PS_SHUTDOWN:
        printf("Status: Processing system shutdown\n");
        break;
      case STS_SPI_START_TIMEOUT:
        printf("Status: SPI start timeout\n");
        break;
      case STS_SPI_RESET_TIMEOUT:
        printf("Status: SPI initialization timeout\n");
        break;
      case STS_INTEG_THRESH_AVG_OOB:
        printf("Status: Integrator threshold average out of bounds\n");
        break;
      case STS_INTEG_WINDOW_OOB:
        printf("Status: Integrator window out of bounds\n");
        break;
      case STS_INTEG_EN_OOB:
        printf("Status: Integrator enable register out of bounds\n");
        break;
      case STS_SYS_EN_OOB:
        printf("Status: System enable register out of bounds\n");
        break;
      case STS_LOCK_VIOL:
        printf("Status: Configuration lock violation\n");
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
      case STS_DAC_BUF_UNDERFLOW:
        printf("Status: DAC command buffer underflow\n");
        print_board_number = true;
        break;
      case STS_DAC_BUF_OVERFLOW:
        printf("Status: DAC command buffer overflow\n");
        print_board_number = true;
        break;
      case STS_UNEXP_DAC_TRIG:
        printf("Status: Unexpected DAC trigger\n");
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
      default:
        printf("Status: Unknown (0x%" PRIx32 ")\n", HW_STS_CODE(hw_status));
        break;
    }
  }
  if (print_board_number) {
    printf("Board Number: %u\n", HW_STS_BOARD(hw_status));
  }
}

// Print debug registers
void print_debug_registers(struct sys_sts_t *sys_sts) {
  for (int i = 0; i < 1; i++) { // Assuming only one debug register for now
    uint32_t value = *(sys_sts->debug[i]);
    printf("Debug register %d: 0b", i);
    for (int bit = 31; bit >= 0; bit--) {
      printf("%u", (value >> bit) & 1);
    }
    printf("\n");
  }
}

