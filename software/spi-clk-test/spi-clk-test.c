#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

//////////////////// Mapped Memory Definitions ////////////////////

#define CMA_ALLOC _IOWR('Z', 0, uint32_t)

#define AXI_BASE        (uint32_t)           0x40000000
#define AXI_CFG         (uint32_t) AXI_BASE + 0x0000000
#define AXI_STS         (uint32_t) AXI_BASE + 0x0100000
#define AXI_SPI_CLK     (uint32_t) AXI_BASE + 0x0200000

#define CFG_SIZE        (uint32_t) 1024 / 8 // Size of the configuration register in bytes
#define STS_SIZE        (uint32_t) 2048 / 8 // Size of the status register in bytes
#define SPI_CLK_SIZE    (uint32_t) 2048 // Size of the SPI_CLK interface in bytes



//////////////////// Function Prototypes ////////////////////

// Print out the available commands
void print_help();
// Map memory and report
uint32_t map_memory(int fd, volatile void **ptr, uint32_t addr, uint32_t size, char *name);



//////////////////// Main ////////////////////
int main()
{

  //////////////////// 1. Setup ////////////////////
  printf("Test program for Pavel Demin's AXI hub\n");
  printf("Setup:\n");

  int fd, i; // File descriptor, loop counter
  volatile void *cfg; // CFG register in AXI hub (set to 32 bits wide)
  volatile void *sts; // STS register in AXI hub (set to 32 bits wide)
  volatile void *spi_clk; // SPI_CLK interface in AXI hub (set to 32 bits wide)

  printf("System page size: %d\n", sysconf(_SC_PAGESIZE));

  // Open /dev/mem to access physical memory
  printf("Opening /dev/mem...\n");
  if((fd = open("/dev/mem", O_RDWR)) < 0)
  {
    perror("open");
    return EXIT_FAILURE;
  }

  // Map CFG and STS registers
  // The base address of the AXI hub is 0x40000000
  // Bits 24-26 are used to indicate the target in the hub
  // 0 is the CFG register and 1 is the STS register
  // 2-7 are ports 0-5 (n-2)
  printf("Mapping registers and ports...\n");

  // CFG register
  uint32_t cfg_page_count = map_memory(fd, &cfg, AXI_CFG, CFG_SIZE, "CFG register");
  // STS register
  uint32_t sts_page_count = map_memory(fd, &sts, AXI_STS, STS_SIZE, "STS register");
  // SPI_CLK interface
  uint32_t spi_clk_page_count = map_memory(fd, &spi_clk, AXI_SPI_CLK, SPI_CLK_SIZE, "SPI_CLK interface");
  
  // File can be closed after mapping without affecting the mapped memory
  close(fd);
  printf("Mapping complete. Page counts: CFG = %u, STS = %u, SPI_CLK = %u\n", cfg_page_count, sts_page_count, spi_clk_page_count);



  //////////////////// 2. Register mapping /////////////////////

  //// CFG
  //   31:0   -- 32b Trigger lockout
  volatile uint32_t *trigger_lockout      = (volatile uint32_t *)(cfg + 0);
  //   47:32  -- 16b Calibration offset
  //   63:48  --     Reserved
  volatile int16_t  *cal_offset           = (volatile int16_t  *)(cfg + 4);
  //   95:64  -- 32b Integrator threshold
  volatile uint32_t *integrator_threshold = (volatile uint32_t *)(cfg + 8);
  //  127:96  -- 32b Integrator span
  volatile uint32_t *integrator_span      = (volatile uint32_t *)(cfg + 12);
  //  128     --  1b Integrator enable
  //  159:129 --     Reserved
  volatile uint8_t  *integrator_enable    = (volatile uint8_t  *)(cfg + 16);
  //  160     --  1b Hardware enable
  //  191:161 --     Reserved
  volatile uint8_t  *hardware_enable      = (volatile uint8_t  *)(cfg + 20);

  //// STS
  //   31:0    -- 32b Hardware status code (31 stopped; 30:0 code)
  volatile uint32_t *hardware_status = (volatile uint32_t *)(sts + 0);
  //   63:32   --     Reserved
  // DAC and ADC FIFO status arrays
  volatile uint64_t *dac_fifo_status[8];
  volatile uint64_t *adc_fifo_status[8];
  for (i = 0; i < 8; i++) {
    dac_fifo_status[i] = (volatile uint64_t *)(sts + (8 + i * 16));
    adc_fifo_status[i] = (volatile uint64_t *)(sts + (16 + i * 16));
  }

  //// SPI_CLK
  volatile uint32_t *spi_clk_reset   = (volatile uint32_t *)(spi_clk + 0x0);
  volatile uint32_t *spi_clk_status  = (volatile uint32_t *)(spi_clk + 0x4);
  volatile uint32_t *spi_clk_cfg_0   = (volatile uint32_t *)(spi_clk + 0x200);
  volatile uint32_t *spi_clk_cfg_1   = (volatile uint32_t *)(spi_clk + 0x208);
  volatile uint32_t *spi_clk_phase   = (volatile uint32_t *)(spi_clk + 0x20C);
  volatile uint32_t *spi_clk_duty    = (volatile uint32_t *)(spi_clk + 0x210);
  volatile uint32_t *spi_clk_enable  = (volatile uint32_t *)(spi_clk + 0x25C);


  //////////////////// 3. Main command loop ////////////////////
  print_help();
  uint32_t verbose = 0;
  while(1){ // Command loop
    printf("Enter command: ");

    // Read command from user input
    char command[256];
    char *num_endptr; // Pointer for strtoul, will mark the end of numerals
    fgets(command, sizeof(command), stdin); // Read user input string
    command[strcspn(command, "\n")] = 0; // Remove newline character
    char *token = strtok(command, " "); // Get the first token

    // No command entered
    if(token == NULL) continue;

    if(strcmp(token, "help") == 0) { // help command
      print_help();
    } else if(strcmp(token, "verbose") == 0) { // verbose command
      verbose = !verbose;
      printf("Verbose mode %s\n", verbose ? "enabled" : "disabled");

    } else if(strcmp(token, "clk_mult") == 0) { // clk_mult command
      if (verbose) {
        printf("Accessing the clock multiplier (in Clock Configuration Register 0 -- %08x)\n", (uint32_t) spi_clk_cfg_0);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        uint32_t int_mult = (*spi_clk_cfg_0 >> 8) & 0xFF;
        uint32_t frac_mult = (*spi_clk_cfg_0 >> 16) & 0x3FF;
        printf("Current clk_mult values: int_val = %u, frac_val = %u\n", int_mult, frac_mult);
        printf("Equivalent multiplier: %f\n", (float)int_mult + (float)frac_mult / 1000.0);
        printf("To change the multiplier, use the same command but specify int_val and optionally frac_val.\n");
        continue;
      }
      uint8_t int_val = strtoul(token, &num_endptr, 10);
      if(num_endptr == token || int_val > 255) {
        printf("Invalid int_val specified: %s\n", token);
        printf("Range: 0-255\n");
        continue;
      }

      token = strtok(NULL, " ");
      uint16_t frac_val = 0;
      if(token != NULL) {
        frac_val = strtoul(token, &num_endptr, 10);
        if(num_endptr == token || frac_val > 875) {
          printf("Invalid frac_val specified: %s\n", token);
          printf("Range: 0-875\n");
          continue;
        }
      }
      printf("Setting clk_mult values: int_val = %u, frac_val = %u\n", int_val, frac_val);
      *spi_clk_cfg_0 = *spi_clk_cfg_0 & ~0x03FFFF00 | (int_val << 8) | (frac_val << 16);

    } else if(strcmp(token, "clk_div0") == 0) { // clk_div0 command
      if (verbose) {
        printf("Accessing the clock divider 0 (in Clock Configuration Register 0 -- %08x)\n", (uint32_t) spi_clk_cfg_0);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        uint32_t div = *spi_clk_cfg_0 & 0xFF;
        printf("Current clk_div0 value: %u\n", div);
        printf("To change the divider, use the same command but specify val.\n");
        continue;
      }
      uint8_t val = strtoul(token, &num_endptr, 10);
      if(num_endptr == token || val > 255) {
        printf("Invalid val specified: %s\n", token);
        printf("Range: 0-255\n");
        continue;
      }

      printf("Setting clk_div0 value: val = %u\n", val);
      *spi_clk_cfg_0 = *spi_clk_cfg_0 & ~0xFF | val;

    } else if(strcmp(token, "clk_div1") == 0) { // clk_div1 command
      if (verbose) {
        printf("Accessing the clock divider 1 (in Clock Configuration Register 2 -- %08x)\n", (uint32_t) spi_clk_cfg_1);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        uint32_t int_div = *spi_clk_cfg_1 & 0xFF;
        uint32_t frac_div = (*spi_clk_cfg_1 >> 8) & 0x3FF;
        printf("Current clk_div1 values: int_val = %u, frac_val = %u\n", int_div, frac_div);
        printf("Equivalent divider: %f\n", (float)int_div + (float)frac_div / 1000.0);
        printf("To change the divider, use the same command but specify int_val and optionally frac_val.\n");
        continue;
      }
      uint8_t int_val = strtoul(token, &num_endptr, 10);
      if(num_endptr == token || int_val > 255) {
        printf("Invalid int_val specified: %s\n", token);
        printf("Range: 0-255\n");
        continue;
      }

      token = strtok(NULL, " ");
      uint16_t frac_val = 0;
      if(token != NULL) {
        frac_val = strtoul(token, &num_endptr, 10);
        if(num_endptr == token || frac_val > 875) {
          printf("Invalid frac_val specified: %s\n", token);
          printf("Range: 0-875\n");
          continue;
        }
      }

      printf("Setting clk_div1 values: int_val = %u, frac_val = %u\n", int_val, frac_val);
      *spi_clk_cfg_1 = *spi_clk_cfg_1 & ~0x0003FFFF | (int_val) | (frac_val << 8);

    } else if(strcmp(token, "clk_phase") == 0) { // clk_phase command
      if (verbose) {
        printf("Accessing the clock phase (in Clock Configuration Register 3 -- %08x)\n", (uint32_t) spi_clk_phase);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        int32_t val = *spi_clk_phase;
        printf("Current clk_phase value: %d\n", val);
        printf("Equivalent phase: %f degrees\n", (float)val / 1000.0);
        printf("To change the phase, use the same command but specify val.\n");
        continue;
      }
      int32_t val = strtol(token, &num_endptr, 10);
      if(num_endptr == token || val < -360000 || val > 360000) {
        printf("Invalid val specified: %s\n", token);
        printf("Range: -360000 to 360000\n");
        continue;
      }

      printf("Setting clk_phase value: val = %d\n", val);
      *spi_clk_phase = val;

    } else if(strcmp(token, "clk_duty") == 0) { // clk_duty command
      if (verbose) {
        printf("Accessing the clock duty cycle (in Clock Configuration Register 4 -- %08x)\n", (uint32_t) spi_clk_duty);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        uint32_t val = *spi_clk_duty;
        printf("Current clk_duty value: %u\n", val);
        printf("Equivalent duty cycle: %f%%\n", (float)val / 1000.0);
        continue;
      }
      uint32_t val = strtoul(token, &num_endptr, 10);
      if(num_endptr == token || val > 100000) {
        printf("Invalid val specified: %s\n", token);
        printf("Range: 0-100000\n");
        continue;
      }

      printf("Setting clk_duty value: val = %u\n", val);
      *spi_clk_duty = val;

    } else if(strcmp(token, "clk_load") == 0) { // clk_load command
      if (verbose) {
        printf("Accessing the clock load/enable (in Clock Configuration Register 23 -- %08x)\n", (uint32_t) spi_clk_enable);
      }
      printf("Loading the custom clock configuration\n");
      *spi_clk_enable = 0x3;

        } else if(strcmp(token, "clk_default") == 0) { // clk_default command
      if (verbose) {
        printf("Accessing the clock load/enable (in Clock Configuration Register 23 -- %08x)\n", (uint32_t) spi_clk_enable);
      }
      printf("Loading the default clock configuration\n");
      *spi_clk_enable = 0x1;

    } else if(strcmp(token, "clk_info") == 0) { // clk_info command
      if (verbose) {
        printf("Accessing the clock settings\n");
      }
      printf("Clock Configuration Registers:\n");
      printf("  0: %08x\n", (uint32_t) spi_clk_cfg_0);
      printf("  2: %08x\n", (uint32_t) spi_clk_cfg_1);
      printf("  3: %08x\n", (uint32_t) spi_clk_phase);
      printf("  4: %08x\n", (uint32_t) spi_clk_duty);
      printf("  23: %08x\n", (uint32_t) spi_clk_enable);
      printf("Status Register:\n");
      printf("  0: %08x\n", (uint32_t) spi_clk_status);
      printf("Current clock settings:\n");
      uint32_t int_mult = (*spi_clk_cfg_0 >> 8) & 0xFF;
      uint32_t frac_mult = (*spi_clk_cfg_0 >> 16) & 0x3FF;
      uint32_t int_div_0 = *spi_clk_cfg_0 & 0xFF;
      uint32_t int_div_1 = *spi_clk_cfg_1 & 0xFF;
      uint32_t frac_div_1 = (*spi_clk_cfg_1 >> 8) & 0x3FF;
      int32_t phase = *spi_clk_phase;
      uint32_t duty = *spi_clk_duty;
      uint32_t locked = *spi_clk_status & 0x1;
      printf("  clk_mult: int_val = %u, frac_val = %u\n", int_mult, frac_mult);
      printf("  clk_div0: val = %u\n", int_div_0);
      printf("  clk_div1: int_val = %u, frac_val = %u\n", int_div_1, frac_div_1);
      printf("  clk_phase: val = %d\n", phase);
      printf("  clk_duty: val = %u\n", duty);
      printf("  clk_locked: %s\n", locked ? "true" : "false");

    } else if(strcmp(token, "clk_reset") == 0) { // clk_reset command
      if (verbose) {
        printf("Accessing the clock reset (in Software Reset Register -- %08x)\n", (uint32_t) spi_clk_reset);
      }
      printf("Pulsing a software reset to the clock\n");
      *spi_clk_reset = 0xA;

    } else if(strcmp(token, "trigger_lockout") == 0) { // trigger_lockout command
      if (verbose) {
        printf("Accessing the trigger lockout (in CFG register -- %08x)\n", (uint32_t) trigger_lockout);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
        uint32_t val = *trigger_lockout;
        printf("Current trigger lockout value: %u\n", val);
        printf("To change the lockout, use the same command but specify val.\n");
        continue;
      }
      uint32_t val = strtoul(token, &num_endptr, 10);
      if(num_endptr == token) {
        printf("Invalid val specified: %s\n", token);
        continue;
      }
      printf("Setting trigger lockout value: val = %u\n", val);
      *trigger_lockout = val;

    } else if(strcmp(token, "cal_offset") == 0) { // cal_offset command
      if (verbose) {
      printf("Accessing the calibration offset (in CFG register -- %08x)\n", (uint32_t) cal_offset);
      }
      token = strtok(NULL, " ");
      if(token == NULL) {
      int16_t val = *cal_offset;
      printf("Current calibration offset value: %d\n", val);
      printf("To change the offset, use the same command but specify val.\n");
      continue;
      }
      int16_t val = strtol(token, &num_endptr, 10);
      if(num_endptr == token) {
      printf("Invalid val specified: %s\n", token);
      continue;
      }
      printf("Setting calibration offset value: val = %d\n", val);
      *cal_offset = val;

    } else if(strcmp(token, "hw_status") == 0) { // hw_status command
      if (verbose) {
        printf("Accessing the hardware status code (in STS register -- %08x)\n", (uint32_t) hardware_status);
      }
      uint32_t val = *hardware_status;
      printf("Hardware status code: %u\n", val);

    } else if(strcmp(token, "fifo_status") == 0) { // fifo_status command
      token = strtok(NULL, " ");
      if(token == NULL) {
      printf("Please specify the board number (0-7).\n");
      continue;
      }
      uint32_t board = strtoul(token, &num_endptr, 10);
      if(num_endptr == token || board > 7) {
      printf("Invalid board number specified: %s\n", token);
      printf("Range: 0-7\n");
      continue;
      }
      printf("DAC FIFO status for board %u: %llu\n", board, *dac_fifo_status[board]);
      printf("ADC FIFO status for board %u: %llu\n", board, *adc_fifo_status[board]);
    } else if(strcmp(token, "exit") == 0) { // exit command
      break;

    } else { // Unknown command
      printf("Unknown command: %s\n", token);
      print_help();
    }
  } // End of command loop

  //////////////////// 4. Cleanup ////////////////////

  // Unmap memory
  printf("Unmapping memory...\n");
  munmap((void *)cfg, cfg_page_count * sysconf(_SC_PAGESIZE));
  munmap((void *)sts, sts_page_count * sysconf(_SC_PAGESIZE));
  munmap((void *)spi_clk, spi_clk_page_count * sysconf(_SC_PAGESIZE));

  printf("Exiting program.\n");
}


//////////////////// Functions ////////////////////

// Print out the available commands
void print_help()
{
  printf("Operations: <required> [optional]\n");
  printf("\n  help\n");
  printf("    - Print this help message\n");
  printf("\n  verbose\n");
  printf("    - Toggle verbose mode\n");
  printf("\n  clk_mult <int_val (uint8)> [frac_val (uint10)]\n");
  printf("    - Write the clock multiplier.\n");
  printf("      The multiplier is equal to int_val + frac_val/1000.\n");
  printf("      frac_val can be 0-875.\n");
  printf("      If no frac_val is specified, it will be set to 0.\n");
  printf("      If no int_val is specified, prints the current value.\n");
  printf("\n  clk_div0 <val (uint8)>\n");
  printf("    - Write the first clock divider\n");
  printf("\n  clk_div1 <int_val (uint8)> [frac_val (uint10)]\n");
  printf("    - Write the second clock divider.\n");
  printf("      The divider is equal to int_val + frac_val/1000.\n");
  printf("      frac_val can be 0-875.\n");
  printf("      If no frac_val is specified, it will be set to 0.\n");
  printf("      If no int_val is specified, prints the current value.\n");
  printf("\n  clk_phase <val (int32)>\n");
  printf("    - Write the clock phase in units of mdeg (1000 = 1deg)\n");
  printf("      val can be -360000 to 360000\n");
  printf("      If no val is specified, prints the current value.\n");
  printf("\n  clk_duty <val (uint32)>\n");
  printf("    - Write the clock duty cycle in units of m%% (1000 = 1%%)\n");
  printf("      val can be 0-100000\n");
  printf("      If no val is specified, prints the current value.\n");
  printf("\n  clk_load\n");
  printf("    - Load the written clock settings\n");
  printf("\n  clk_default\n");
  printf("    - Load the default clock settings\n");
  printf("\n  clk_info\n");
  printf("    - Print all the current clock settings and the locked bit\n");
  printf("\n  clk_reset\n");
  printf("    - Pulse a software reset to the clock\n");
  printf("\n  trigger_lockout <val (uint32)>\n");
  printf("    - Set the trigger lockout in SPI clock cycles\n");
  printf("      If no val is specified, prints the current value.\n");
  printf("\n  cal_offset <val (int16)>\n");
  printf("    - Set the calibration offset\n");
  printf("      If no val is specified, prints the current value.\n");
  printf("\n  hw_status\n");
  printf("    - Print the hardware status code\n");
  printf("\n  fifo_status <val (uint3)>\n");
  printf("    - Print the DAC and ADC FIFO status for board 0-7.\n");
  printf("\n  exit\n");
  printf("    - Exit the program\n");
}

// Map memory and report
uint32_t map_memory(int fd, volatile void **ptr, uint32_t addr, uint32_t size, char *name)
{
  uint32_t page_size = sysconf(_SC_PAGESIZE); // Get system page size
  uint32_t page_count = (size - 1) / page_size + 1; // Calculate number of pages needed
  *ptr = mmap(NULL, page_count * page_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, addr);
  if (*ptr == MAP_FAILED) {
    perror("mmap failed");
    return 0;
  }
  printf("%s mapped to 0x%x:0x%x (%d page[s])\n", name, addr, addr + size - 1, page_count);
  return page_count;
}
