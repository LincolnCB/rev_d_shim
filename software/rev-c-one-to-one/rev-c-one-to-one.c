#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

// Include hardware control modules
#include "sys_ctrl.h"
#include "adc_ctrl.h"
#include "dac_ctrl.h"
#include "spi_clk_ctrl.h"
#include "sys_sts.h"
#include "buf_sts.h"
#include "trigger_ctrl.h"

//////////////////// Main ////////////////////
int main()
{
  //////////////////// 1. Setup ////////////////////
  printf("Rev. C to D One-to-One Test Program\n");
  printf("Setup:\n");

  //// Hardware control structures
  struct sys_ctrl_t sys_ctrl;              // System control and configuration
  struct spi_clk_ctrl_t spi_clk_ctrl;      // SPI clock control interface
  struct sys_sts_t sys_sts;                // System status
  struct buf_sts_t buf_sts;                // FIFO unavailable status
  struct dac_ctrl_array_t dac_ctrl;        // DAC command FIFOs (all boards)
  struct adc_ctrl_array_t adc_ctrl;        // ADC command and data FIFOs (all boards)
  struct trigger_ctrl_t trigger_ctrl;      // Trigger command and data FIFOs

  // Initialize hardware control structures
  printf("Initializing hardware control modules...\n");
  bool verbose = true; // Enable verbose logging

  // Initialize control and status modules
  sys_ctrl = create_sys_ctrl(verbose);
  printf("System control module initialized\n");

  spi_clk_ctrl = create_spi_clk_ctrl(verbose);
  printf("SPI clock control module initialized\n");

  sys_sts = create_sys_sts(verbose);
  printf("System status module initialized\n");

  buf_sts = create_buf_sts(verbose);
  printf("Buffer status module initialized\n");

  // Initialize FIFO modules
  dac_ctrl = create_dac_ctrl_array(verbose);
  printf("DAC control modules initialized (8 boards)\n");

  adc_ctrl = create_adc_ctrl_array(verbose);
  printf("ADC control modules initialized (8 boards)\n");

  trigger_ctrl = create_trigger_ctrl(verbose);
  printf("Trigger control module initialized\n");

  printf("Hardware initialization complete.\n");
}
