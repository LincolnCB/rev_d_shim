#include <stdio.h>
#include <stdlib.h>
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
  sys_sts.hw_status_code = sys_sts_ptr + HW_STS_CODE_OFFSET;
  
  // Initialize FIFO status pointers for each board
  for (int i = 0; i < 8; i++) {
    sys_sts.dac_cmd_fifo_sts[i] = sys_sts_ptr + DAC_CMD_FIFO_STS_OFFSET(i);
    sys_sts.adc_cmd_fifo_sts[i] = sys_sts_ptr + ADC_CMD_FIFO_STS_OFFSET(i);
    sys_sts.adc_data_fifo_sts[i] = sys_sts_ptr + ADC_DATA_FIFO_STS_OFFSET(i);
  }
  
  // Initialize trigger FIFO status pointers
  sys_sts.trig_cmd_fifo_sts = sys_sts_ptr + TRIG_CMD_FIFO_STS_OFFSET;
  sys_sts.trig_data_fifo_sts = sys_sts_ptr + TRIG_DATA_FIFO_STS_OFFSET;
  
  return sys_sts;
}
