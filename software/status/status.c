#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include <string.h>

#include "sys_sts.h"

static void print_usage(const char *prog) {
  printf(
      "Usage: %s [-h|--help]\n"
      "\n"
      "Read-only diagnostic tool for the Rev D shim amplifier. Reads the FPGA\n"
      "system status register (mapped at 0x40100000) and prints a human-readable\n"
      "snapshot of the hardware state. It only observes and reports, and never\n"
      "drives or reconfigures the hardware.\n"
      "\n"
      "Takes no arguments. Run it on the target to dump the current status.\n"
      "\n"
      "Reports:\n"
      "  - Hardware status: decoded state, status code, and associated board.\n"
      "  - Trigger count: number of hardware triggers seen.\n"
      "  - Clock frequencies: SPI clock and SPI source clock, in Hz.\n"
      "  - Timing debug register: clock-locked/SPI-off flags, DAC/ADC ~CS high\n"
      "    times, and SPI clock snoop reconfiguration state machine state.\n"
      "  - Minimum delay times: DAC and ADC \"delay too short\" thresholds.\n"
      "  - Per-board FIFO status (boards 0-7): DAC/ADC command/data FIFO status,\n"
      "    last received command word, and command count since reset.\n"
      "  - Trigger FIFOs: command and data FIFO status.\n"
      "\n"
      "Options:\n"
      "  -h, --help    Show this help message and exit.\n",
      prog);
}

int main(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      print_usage(argv[0]);
      return EXIT_SUCCESS;
    }
    fprintf(stderr, "Unknown argument: %s\n", argv[i]);
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  struct sys_sts_t sys_sts = create_sys_sts(false);

  uint32_t hw_status = sys_sts_get_hw_status(&sys_sts, false);

  printf("Hardware status:\n");
  print_hw_status(hw_status, true);
  printf("\n");

  printf("Trigger count          : %u\n", sys_sts_get_trig_count(&sys_sts, false));
  print_clk_freq(sys_sts_get_clk_freq_hz(&sys_sts, false), false);
  printf("SPI source ");
  print_clk_freq(sys_sts_get_source_clk_freq_hz(&sys_sts, false), false);
  printf("\n");

  print_debug_register(&sys_sts);
  printf("\n");

  printf("DAC min delay time     : %u cycles\n", sys_sts_get_dac_min_delay_time(&sys_sts, false));
  printf("ADC min delay time     : %u cycles\n", sys_sts_get_adc_min_delay_time(&sys_sts, false));
  printf("\n");

  for (uint8_t board = 0; board < 8; board++) {
    char name[64];
    snprintf(name, sizeof(name), "DAC %u cmd FIFO", board);
    uint32_t dac_cmd_status = sys_sts_get_dac_cmd_fifo_status(&sys_sts, board, false);
    print_fifo_status(dac_cmd_status, name);
    snprintf(name, sizeof(name), "DAC %u data FIFO", board);
    uint32_t dac_data_status = sys_sts_get_dac_data_fifo_status(&sys_sts, board, false);
    print_fifo_status(dac_data_status, name);

    if (FIFO_PRESENT(dac_cmd_status)) {
      printf("  Last received DAC command for board %u: 0x%08" PRIx32 "\n",
             board,
             sys_sts_get_last_received_dac_cmd(&sys_sts, board, false));
    }
    printf("  DAC commands since reset for board %u: %u\n",
           board,
           sys_sts_get_dac_cmds_since_reset(&sys_sts, board, false));
    printf("\n");
  }

  for (uint8_t board = 0; board < 8; board++) {
    char name[64];
    snprintf(name, sizeof(name), "ADC %u cmd FIFO", board);
    uint32_t adc_cmd_status = sys_sts_get_adc_cmd_fifo_status(&sys_sts, board, false);
    print_fifo_status(adc_cmd_status, name);
    snprintf(name, sizeof(name), "ADC %u data FIFO", board);
    uint32_t adc_data_status = sys_sts_get_adc_data_fifo_status(&sys_sts, board, false);
    print_fifo_status(adc_data_status, name);

    if (FIFO_PRESENT(adc_cmd_status)) {
      printf("  Last received ADC command for board %u: 0x%08" PRIx32 "\n",
             board,
             sys_sts_get_last_received_adc_cmd(&sys_sts, board, false));
    }
    printf("  ADC commands since reset for board %u: %u\n",
           board,
           sys_sts_get_adc_cmds_since_reset(&sys_sts, board, false));
    printf("\n");
  }

  print_fifo_status(sys_sts_get_trig_cmd_fifo_status(&sys_sts, false), "Trigger cmd FIFO");
  print_fifo_status(sys_sts_get_trig_data_fifo_status(&sys_sts, false), "Trigger data FIFO");
  printf("\n");

  return EXIT_SUCCESS;
}
