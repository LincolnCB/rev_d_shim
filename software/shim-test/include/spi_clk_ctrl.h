#ifndef SPI_CLK_CTRL_H
#define SPI_CLK_CTRL_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// SPI Clock Control Definitions ////////////////////
// SPI clock control
#define SPI_CLK_BASE          (uint32_t) 0x40200000
#define SPI_CLK_WORDCOUNT     (uint32_t) 2048 * 4 // Size in 32-bit words. 2048 bytes * 4 bytes per word
// 32-bit offsets within the SPI_CLK interface
#define SPI_CLK_RESET_OFFSET  (uint32_t) 0x0 // Reset register
#define SPI_CLK_STATUS_OFFSET (uint32_t) 0x4 // Status register
#define SPI_CLK_CFG_0_OFFSET  (uint32_t) 0x200 // Clock configuration register 0
#define SPI_CLK_CFG_1_OFFSET  (uint32_t) 0x208 // Clock configuration register 1
#define SPI_CLK_PHASE_OFFSET  (uint32_t) 0x20C // Clock phase register
#define SPI_CLK_DUTY_OFFSET   (uint32_t) 0x210 // Clock duty cycle register
#define SPI_CLK_ENABLE_OFFSET (uint32_t) 0x25C // Clock enable register

//////////////////////////////////////////////////////////////////

// SPI clock control structure
struct spi_clk_ctrl_t {
  volatile uint32_t *reset;  // Reset register
  volatile uint32_t *status; // Status register
  volatile uint32_t *cfg_0;  // Clock configuration register 0
  volatile uint32_t *cfg_1;  // Clock configuration register 1
  volatile uint32_t *phase;  // Clock phase register
  volatile uint32_t *duty;   // Clock duty cycle register
  volatile uint32_t *enable; // Clock enable register
};

// Function declaration
struct spi_clk_ctrl_t create_spi_clk_ctrl(bool verbose);

#endif // SPI_CLK_CTRL_H
