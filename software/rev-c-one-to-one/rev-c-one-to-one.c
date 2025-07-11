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

//////////////////// Mapped Memory Definitions ////////////////////
// AXI interface addresses
// Addresses are defined in the hardware design Tcl file

// Prestart configuration register
#define SYS_CTRL_BASE (uint32_t) 0x40000000
#define SYS_CTRL_SIZE (uint32_t) 5 * 4 // 5 32-bit words, 4 bytes each
// Offsets within the prestart configuration register 
//   (in bytes, each is 32 bits wide with possible reserved bits)
#define INTEGRATOR_THRESHOLD_AVERAGE_OFFSET (uint32_t) 0 * 4
#define INTEGRATOR_WINDOW_OFFSET            (uint32_t) 1 * 4
#define INTEGRATOR_ENABLE_OFFSET            (uint32_t) 2 * 4
#define BUFFER_RESET_OFFSET                 (uint32_t) 3 * 4
#define SYSTEM_ENABLE_OFFSET                (uint32_t) 4 * 4

// SPI clock control
#define SPI_CLK_BASE    (uint32_t) 0x40200000
#define SPI_CLK_SIZE    (uint32_t) 2048 // Size of the SPI_CLK interface in bytes
// Offsets within the SPI_CLK interface
#define SPI_CLK_RESET_OFFSET   (uint32_t) 0x0 // Reset register
#define SPI_CLK_STATUS_OFFSET  (uint32_t) 0x4 // Status register
#define SPI_CLK_CFG_0_OFFSET   (uint32_t) 0x200 // Clock configuration register 0
#define SPI_CLK_CFG_1_OFFSET   (uint32_t) 0x208 // Clock configuration register 1
#define SPI_CLK_PHASE_OFFSET   (uint32_t) 0x20C // Clock phase register
#define SPI_CLK_DUTY_OFFSET    (uint32_t) 0x210 // Clock duty cycle register
#define SPI_CLK_ENABLE_OFFSET  (uint32_t) 0x25C // Clock enable register

// DAC and ADC FIFOs
#define DAC_CMD_FIFO(board)   (0x80000000 + (board) * 0x10000)
#define ADC_CMD_FIFO(board)   (0x80001000 + (board) * 0x10000)
#define ADC_DATA_FIFO(board)  (0x80002000 + (board) * 0x10000)
// Trigger FIFOs
#define TRIG_CMD_FIFO  (uint32_t) 0x80100000
#define TRIG_DATA_FIFO (uint32_t) 0x80101000

// Status register
#define SYS_STS (uint32_t) 0x40100000
#define SYS_STS_SIZE (uint32_t) 864 / 8 // 864 bits, one address is one byte
// Offsets within the status register
#define HW_STS_CODE_OFFSET          (uint32_t) 0 // Hardware status code
#define DAC_CMD_FIFO_STS_OFFSET(board)  ( 4 + 12 * (board)) // DAC command FIFO status
#define ADC_CMD_FIFO_STS_OFFSET(board)  ( 8 + 12 * (board)) // ADC command FIFO status
#define ADC_DATA_FIFO_STS_OFFSET(board) (12 + 12 * (board)) // ADC data FIFO status
#define TRIG_CMD_FIFO_STS_OFFSET    (uint32_t) 800 / 8 // Trigger command FIFO status
#define TRIG_DATA_FIFO_STS_OFFSET   (uint32_t) 832 / 8 // Trigger data FIFO status
