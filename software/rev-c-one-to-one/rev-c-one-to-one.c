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

// System control and configuration register
#define SYS_CTRL_BASE                       (uint32_t) 0x40000000
#define SYS_CTRL_WORDCOUNT                  (uint32_t) 5 // Size in 32-bit words
// 32-bit offsets within the system control and configuration register 
#define INTEGRATOR_THRESHOLD_AVERAGE_OFFSET (uint32_t) 0
#define INTEGRATOR_WINDOW_OFFSET            (uint32_t) 1
#define INTEGRATOR_ENABLE_OFFSET            (uint32_t) 2
#define BUFFER_RESET_OFFSET                 (uint32_t) 3
#define SYSTEM_ENABLE_OFFSET                (uint32_t) 4

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

// DAC and ADC FIFOs
#define DAC_CMD_FIFO(board)   (0x80000000 + (board) * 0x10000)
#define ADC_CMD_FIFO(board)   (0x80001000 + (board) * 0x10000)
#define ADC_DATA_FIFO(board)  (0x80002000 + (board) * 0x10000)
// Trigger FIFOs
#define TRIG_CMD_FIFO  (uint32_t) 0x80100000
#define TRIG_DATA_FIFO (uint32_t) 0x80101000

// Status register
#define SYS_STS           (uint32_t) 0x40100000
#define SYS_STS_WORDCOUNT (uint32_t) 1 + (3 * 8) + 2 // Size in 32-bit words
// 32-bit offsets within the status register
#define HW_STS_CODE_OFFSET          (uint32_t) 0 // Hardware status code
// 8 boards, 0-7
#define DAC_CMD_FIFO_STS_OFFSET(board)  (1 + 3 * (board)) // DAC command FIFO status
#define ADC_CMD_FIFO_STS_OFFSET(board)  (2 + 3 * (board)) // ADC command FIFO status
#define ADC_DATA_FIFO_STS_OFFSET(board) (3 + 3 * (board)) // ADC data FIFO status
// Trigger FIFOs
#define TRIG_CMD_FIFO_STS_OFFSET    (uint32_t) (3 * 8) + 1 // Trigger command FIFO status
#define TRIG_DATA_FIFO_STS_OFFSET   (uint32_t) (3 * 8) + 2 // Trigger data FIFO status

// FIFO unavialable register
#define FIFO_UNAVAILABLE           (uint32_t) 0x40110000
#define FIFO_UNAVAILABLE_WORDCOUNT (uint32_t) 1 // Size in 32-bit words

//////////////////// Function Prototypes ////////////////////

uint32_t *map_32bit_memory(uint32_t base_addr, uint32_t size, int dev_mem_fd);

//////////////////// Main ////////////////////
int main()
{
  //////////////////// 1. Setup ////////////////////
  printf("Rev. C to D One-to-One Test Program\n");
  printf("Setup:\n");

  int fd; // File descriptor to open /dev/mem

  //// Pointers to top level memory-mapped registers

  // Control, configuration, and status registers
  volatile uint32_t *sys_ctrl; // System control and configuration register
  volatile uint32_t *spi_clk;  // SPI clock control interface
  volatile uint32_t *sys_sts;  // System status register
  volatile uint32_t *fifo_unavailable_sts; // FIFO unavailable status register

  // Arrays of pointers to DAC and ADC FIFOs
  volatile uint32_t *dac_cmd_fifo[8];  // DAC command FIFOs
  volatile uint32_t *adc_cmd_fifo[8];  // ADC command FIFOs
  volatile uint32_t *adc_data_fifo[8]; // ADC data FIFOs

  // Trigger FIFOs
  volatile uint32_t *trig_cmd_fifo;  // Trigger command FIFO
  volatile uint32_t *trig_data_fifo; // Trigger data FIFO

  printf("System page size: %d\n", sysconf(_SC_PAGESIZE));

  // Open /dev/mem to access physical memory
  printf("Opening /dev/mem...\n");
  if((fd = open("/dev/mem", O_RDWR)) < 0)
  {
    perror("open");
    return EXIT_FAILURE;
  }

  // Memory map register and FIFO addresses
  printf("Mapping registers and FIFOs...\n");

  // Control, configuration, and status registers
  sys_ctrl = map_32bit_memory(SYS_CTRL_BASE, SYS_CTRL_WORDCOUNT, fd);  
  printf("System control register mapped to 0x%" PRIx32 "\n", SYS_CTRL_BASE);

  spi_clk = map_32bit_memory(SPI_CLK_BASE, SPI_CLK_WORDCOUNT, fd);
  printf("SPI clock control interface mapped to 0x%" PRIx32 "\n", SPI_CLK_BASE);

  sys_sts = map_32bit_memory(SYS_STS, SYS_STS_WORDCOUNT, fd);
  printf("System status register mapped to 0x%" PRIx32 "\n", SYS_STS);

  fifo_unavailable_sts = map_32bit_memory(FIFO_UNAVAILABLE, FIFO_UNAVAILABLE_WORDCOUNT, fd);
  printf("FIFO unavailable status register mapped to 0x%" PRIx32 "\n", FIFO_UNAVAILABLE);

  // DAC and ADC FIFOs
  for (int i = 0; i < 8; i++) {
    dac_cmd_fifo[i] = map_32bit_memory(DAC_CMD_FIFO(i), 1024, fd);
    printf("DAC command FIFO %d mapped to 0x%" PRIx32 "\n", i, DAC_CMD_FIFO(i));

    adc_cmd_fifo[i] = map_32bit_memory(ADC_CMD_FIFO(i), 1024, fd);
    printf("ADC command FIFO %d mapped to 0x%" PRIx32 "\n", i, ADC_CMD_FIFO(i));

    adc_data_fifo[i] = map_32bit_memory(ADC_DATA_FIFO(i), 1024, fd);
    printf("ADC data FIFO %d mapped to 0x%" PRIx32 "\n", i, ADC_DATA_FIFO(i));
  }

  // Trigger FIFOs
  trig_cmd_fifo = map_32bit_memory(TRIG_CMD_FIFO, 1024, fd);
  printf("Trigger command FIFO mapped to 0x%" PRIx32 "\n", TRIG_CMD_FIFO);

  trig_data_fifo = map_32bit_memory(TRIG_DATA_FIFO, 1024, fd);
  printf("Trigger data FIFO mapped to 0x%" PRIx32 "\n", TRIG_DATA_FIFO);

  close(fd); // Close /dev/mem after mapping
  printf("Mapping complete.\n");
}

//////////////////// Helper Functions ////////////////////

uint32_t *map_32bit_memory(uint32_t base_addr, uint32_t wordcount, int dev_mem_fd)
{
  // Memory is mapped one page at a time. Page size is measured in bytes.
  // The wordcount is in 32-bit words, which each have 4 bytes.
  uint32_t map_size = ((wordcount * 4 - 1) / sysconf(_SC_PAGESIZE) + 1) * sysconf(_SC_PAGESIZE);
  return (uint32_t *)mmap(NULL, map_size, PROT_READ|PROT_WRITE, MAP_SHARED, dev_mem_fd, base_addr);
}
