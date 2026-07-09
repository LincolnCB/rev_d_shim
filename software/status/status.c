#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "sys_sts.h"

int main(void) {
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
