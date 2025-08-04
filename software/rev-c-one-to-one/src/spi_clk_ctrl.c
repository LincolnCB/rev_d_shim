#include <stdio.h>
#include <stdlib.h>
#include "spi_clk_ctrl.h"
#include "map_memory.h"

// Function to create SPI clock control structure
struct spi_clk_ctrl_t create_spi_clk_ctrl(bool verbose) {
  struct spi_clk_ctrl_t spi_clk_ctrl;
  
  // Map SPI clock control base address
  volatile uint32_t *spi_clk_ptr = map_32bit_memory(SPI_CLK_BASE, SPI_CLK_WORDCOUNT, "SPI Clock Ctrl", verbose);
  if (spi_clk_ptr == NULL) {
    fprintf(stderr, "Failed to map SPI clock control memory region.\n");
    exit(EXIT_FAILURE);
  }
  
  // Initialize the SPI clock control structure with the mapped memory addresses
  spi_clk_ctrl.reset = spi_clk_ptr + (SPI_CLK_RESET_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.status = spi_clk_ptr + (SPI_CLK_STATUS_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.cfg_0 = spi_clk_ptr + (SPI_CLK_CFG_0_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.cfg_1 = spi_clk_ptr + (SPI_CLK_CFG_1_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.phase = spi_clk_ptr + (SPI_CLK_PHASE_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.duty = spi_clk_ptr + (SPI_CLK_DUTY_OFFSET / sizeof(uint32_t));
  spi_clk_ctrl.enable = spi_clk_ptr + (SPI_CLK_ENABLE_OFFSET / sizeof(uint32_t));
  
  return spi_clk_ctrl;
}
