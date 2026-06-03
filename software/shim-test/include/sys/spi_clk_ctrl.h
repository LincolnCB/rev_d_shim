#ifndef SPI_CLK_CTRL_H
#define SPI_CLK_CTRL_H

#include <stdint.h>
#include <stdbool.h>

// Forward declaration
struct sys_sts_t;

//////////////////// SPI Clock Control Definitions ////////////////////
//// SPI clock control
#define SPI_CLK_BASE            (uint32_t) 0x40200000
#define SPI_CLK_WORDCOUNT       (uint32_t) 2048 * 4 // Size in 32-bit words. 2048 bytes * 4 bytes per word
//// 32-bit offsets within the SPI_CLK interface
// Reset register
// To activate software reset, the value 0x0000_000A must be written to the register. 
// Any other access, read or write, has undefined results.
#define SPI_CLK_RESET_OFFSET    (uint32_t) 0x0 
// Status register
// Bit[0] = locked
// When 1 MMCM/PLL is locked and ready for reconfiguration. 
// The status of this bit is 0 during reconfiguration. 
#define SPI_CLK_STATUS_OFFSET   (uint32_t) 0x4
// Clock configuration register 0
// clk_fb_div [7:0]
// clk_fb_mult ([15:8] whole part and [25:16] fractional part)
#define SPI_CLK_FB_CFG_OFFSET   (uint32_t) 0x200
// Clock configuration register 1
// clk_fb_phase in signed degrees (-360000 to 360000, where 1000 = 1 degree)
#define SPI_CLK_FB_PHASE_OFFSET (uint32_t) 0x204
// Clock configuration register 2
// clk_div ([7:0] whole part and [17:8] fractional part)
#define SPI_CLK_DIV_OFFSET      (uint32_t) 0x208
// Clock configuration register 3
// clk_phase in signed degrees (-360000 to 360000, where 1000 = 1 degree)
#define SPI_CLK_PHASE_OFFSET    (uint32_t) 0x20C
// Clock configuration register 4
// clk_duty in unsigned m% (0 to 100000, where 1000 = 1%)
#define SPI_CLK_CFG_4_OFFSET    (uint32_t) 0x210
// Clock enable register
// Bit[0] = load
//   Loads Clock Configuration Register values to the internal register used for
//   dynamic reconfiguration and initiates reconfiguration state machine.
//   This bit should be asserted when the required settings are already written into Clock Configuration Registers.
//   This bit retains to 0, when the dynamic reconfiguration is done and the clock is locked.
// Bit[1] = use_user_params
//   When written 0, default configuration done in the Clocking Wizard GUI is loaded for dynamic reconfiguration.
//   When written 1, setting provided in the Clock Configuration Registers are used for dynamic reconfiguration. 
#define SPI_CLK_ENABLE_OFFSET   (uint32_t) 0x25C

//////////////////////////////////////////////////////////////////

// SPI clock control structure
struct spi_clk_ctrl_t {
  volatile uint32_t *reset;  // Reset register
  volatile uint32_t *status; // Status register
  volatile uint32_t *fb_cfg; // Feedback clock configuration register
  volatile uint32_t *fb_phase; // Feedback clock phase register
  volatile uint32_t *div; // Clock divider register
  volatile uint32_t *phase; // Clock phase register
  volatile uint32_t *duty; // Clock duty cycle register
  volatile uint32_t *enable; // Clock enable register
};

// Function declaration
struct spi_clk_ctrl_t create_spi_clk_ctrl(bool verbose);

// Software reset
void spi_clk_ctrl_reset(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
// Locked
bool spi_clk_ctrl_locked(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);

// Feedback clock configuration
uint8_t spi_clk_ctrl_get_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
int spi_clk_ctrl_set_clk_fb_div(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, uint8_t clk_fb_div, bool verbose);
double spi_clk_ctrl_get_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
int spi_clk_ctrl_set_clk_fb_mult(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, bool verbose);
int spi_clk_ctrl_set_clk_fb(struct spi_clk_ctrl_t *spi_clk_ctrl, struct sys_sts_t *sys_sts, double clk_fb_mult, uint8_t clk_fb_div, bool verbose);

// Feedback clock phase
int32_t spi_clk_ctrl_get_clk_fb_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
void spi_clk_ctrl_set_clk_fb_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, int32_t clk_fb_phase, bool verbose);

// Clock divider
double spi_clk_ctrl_get_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
void spi_clk_ctrl_set_clk_div(struct spi_clk_ctrl_t *spi_clk_ctrl, double clk_div, bool verbose);

// Clock phase
int32_t spi_clk_ctrl_get_clk_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
void spi_clk_ctrl_set_clk_phase(struct spi_clk_ctrl_t *spi_clk_ctrl, int32_t clk_phase, bool verbose);

// Clock duty cycle
uint32_t spi_clk_ctrl_get_clk_duty(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
void spi_clk_ctrl_set_clk_duty(struct spi_clk_ctrl_t *spi_clk_ctrl, uint32_t clk_duty, bool verbose);

// Load values
void spi_clk_ctrl_load_default(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);
void spi_clk_ctrl_load_user(struct spi_clk_ctrl_t *spi_clk_ctrl, bool verbose);

#endif // SPI_CLK_CTRL_H
