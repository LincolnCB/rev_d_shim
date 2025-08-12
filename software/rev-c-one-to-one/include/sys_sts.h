#ifndef SYS_STS_H
#define SYS_STS_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// System Status Definitions ////////////////////
// Status register
#define SYS_STS           (uint32_t) 0x40100000
#define SYS_STS_WORDCOUNT (uint32_t) 1 + (3 * 8) + 2 // Size in 32-bit words
// 32-bit offsets within the status register
#define HW_STS_REG_OFFSET (uint32_t) 0 // Hardware status register
// Command FIFO status offset for DAC board (in 32-bit words)
#define DAC_CMD_FIFO_STS_OFFSET_WORD(board)   (1 + (2 * (board)))
// Data FIFO status offset for DAC board (in 32-bit words)
#define DAC_DATA_FIFO_STS_OFFSET_WORD(board)  (18 + (2 * (board)))
// Command FIFO status offset for ADC board (in 32-bit words)
#define ADC_CMD_FIFO_STS_OFFSET_WORD(board)   (1 + (2 * (board) + 1))
// Data FIFO status offset for ADC board (in 32-bit words)
#define ADC_DATA_FIFO_STS_OFFSET_WORD(board)  (18 + (2 * (board) + 1))
#define TRIG_CMD_FIFO_STS_OFFSET    (uint32_t) 17 // Trigger command FIFO status
#define TRIG_DATA_FIFO_STS_OFFSET   (uint32_t) 34 // Trigger data FIFO status
// Debug registers
#define DEBUG_REG_COUNT  (uint32_t) 1 // Number of debug registers
#define DEBUG_REG_OFFSET(index) (uint32_t) (35 + (index)) // Debug register offset

// Macro for extracting the 4-bit state
#define HW_STS_STATE(hw_status) ((hw_status) & 0xF)
// Macro for extracting the 25-bit status code
#define HW_STS_CODE(hw_status) (((hw_status) >> 4) & 0x1FFFFFFF)
// Macro for extracting the 3-bit board number
#define HW_STS_BOARD(hw_status) (((hw_status) >> 29) & 0x7)

// State codes
#define S_IDLE                (uint32_t) 1
#define S_CONFIRM_SPI_RST     (uint32_t) 2
#define S_POWER_ON_CRTL_BRD   (uint32_t) 3
#define S_CONFIRM_SPI_START   (uint32_t) 4
#define S_POWER_ON_AMP_BRD    (uint32_t) 5
#define S_AMP_POWER_WAIT      (uint32_t) 6
#define S_RUNNING             (uint32_t) 7
#define S_HALTING             (uint32_t) 8
#define S_HALTED              (uint32_t) 9

// Status codes (matches hardware manager core status codes)
// 25 bits wide, see hardware shim_hw_manager.v for details
#define STS_EMPTY                    (uint32_t) 0x0000 // Empty status.
#define STS_OK                       (uint32_t) 0x0001 // System is operating normally.
#define STS_PS_SHUTDOWN              (uint32_t) 0x0002 // Processing system shutdown.
// SPI subsystem
#define STS_SPI_RESET_TIMEOUT        (uint32_t) 0x0100 // SPI initialization timeout.
#define STS_SPI_START_TIMEOUT        (uint32_t) 0x0101 // SPI start timeout.
// Pre-start configuration values
#define STS_LOCK_VIOL                (uint32_t) 0x0200 // Configuration lock violation.
#define STS_SYS_EN_OOB               (uint32_t) 0x0201 // System enable register out of bounds.
#define STS_CMD_BUFFER_RESET_OOB     (uint32_t) 0x0202 // Command buffer reset out of bounds.
#define STS_DATA_BUFFER_RESET_OOB    (uint32_t) 0x0203 // Data buffer reset out of bounds.
#define STS_INTEG_THRESH_AVG_OOB     (uint32_t) 0x0204 // Integrator threshold average out of bounds.
#define STS_INTEG_WINDOW_OOB         (uint32_t) 0x0205 // Integrator window out of bounds.
#define STS_INTEG_EN_OOB             (uint32_t) 0x0206 // Integrator enable register out of bounds.
#define STS_BOOT_TEST_SKIP_OOB       (uint32_t) 0x0207 // Boot test skip out of bounds.
#define STS_BOOT_TEST_DEBUG_OOB      (uint32_t) 0x0208 // Boot test debug out of bounds.
// Shutdown sense
#define STS_SHUTDOWN_SENSE           (uint32_t) 0x0300 // Shutdown sense detected.
#define STS_EXT_SHUTDOWN             (uint32_t) 0x0301 // External shutdown triggered.
// Integrator threshold core
#define STS_OVER_THRESH              (uint32_t) 0x0400 // DAC over threshold.
#define STS_THRESH_UNDERFLOW         (uint32_t) 0x0401 // DAC threshold FIFO underflow.
#define STS_THRESH_OVERFLOW          (uint32_t) 0x0402 // DAC threshold FIFO overflow.
// Trigger buffer and commands
#define STS_BAD_TRIG_CMD             (uint32_t) 0x0500 // Bad trigger command.
#define STS_TRIG_CMD_BUF_OVERFLOW    (uint32_t) 0x0501 // Trigger command buffer overflow.
#define STS_TRIG_DATA_BUF_UNDERFLOW  (uint32_t) 0x0502 // Trigger data buffer underflow.
#define STS_TRIG_DATA_BUF_OVERFLOW   (uint32_t) 0x0503 // Trigger data buffer overflow.
// DAC buffers and commands
#define STS_DAC_BOOT_FAIL            (uint32_t) 0x0600 // DAC boot failure.
#define STS_BAD_DAC_CMD              (uint32_t) 0x0601 // Bad DAC command.
#define STS_DAC_CAL_OOB              (uint32_t) 0x0602 // DAC calibration out of bounds.
#define STS_DAC_VAL_OOB              (uint32_t) 0x0603 // DAC value out of bounds.
#define STS_DAC_CMD_BUF_UNDERFLOW    (uint32_t) 0x0604 // DAC command buffer underflow.
#define STS_DAC_CMD_BUF_OVERFLOW     (uint32_t) 0x0605 // DAC command buffer overflow.
#define STS_DAC_DATA_BUF_UNDERFLOW   (uint32_t) 0x0606 // DAC data buffer underflow.
#define STS_DAC_DATA_BUF_OVERFLOW    (uint32_t) 0x0607 // DAC data buffer overflow.
#define STS_UNEXP_DAC_TRIG           (uint32_t) 0x0608 // Unexpected DAC trigger.
// ADC buffers and commands
#define STS_ADC_BOOT_FAIL            (uint32_t) 0x0700 // ADC boot failure.
#define STS_BAD_ADC_CMD              (uint32_t) 0x0701 // Bad ADC command.
#define STS_ADC_CMD_BUF_UNDERFLOW    (uint32_t) 0x0702 // ADC command buffer underflow.
#define STS_ADC_CMD_BUF_OVERFLOW     (uint32_t) 0x0703 // ADC command buffer overflow.
#define STS_ADC_DATA_BUF_UNDERFLOW   (uint32_t) 0x0704 // ADC data buffer underflow.
#define STS_ADC_DATA_BUF_OVERFLOW    (uint32_t) 0x0705 // ADC data buffer overflow.
#define STS_UNEXP_ADC_TRIG           (uint32_t) 0x0706 // Unexpected ADC trigger.

//////////////////////////////////////////////////////////////////

// System status structure
struct sys_sts_t {
  volatile uint32_t *hw_status_reg;          // Hardware status
  volatile uint32_t *dac_cmd_fifo_sts[8];    // DAC command FIFO status for 8 boards
  volatile uint32_t *dac_data_fifo_sts[8];   // DAC data FIFO status for 8 boards
  volatile uint32_t *adc_cmd_fifo_sts[8];    // ADC command FIFO status for 8 boards
  volatile uint32_t *adc_data_fifo_sts[8];   // ADC data FIFO status for 8 boards
  volatile uint32_t *trig_cmd_fifo_sts;      // Trigger command FIFO status
  volatile uint32_t *trig_data_fifo_sts;     // Trigger data FIFO status
  volatile uint32_t *debug[DEBUG_REG_COUNT]; // Debug registers
};

// Structure initialization function
struct sys_sts_t create_sys_sts(bool verbose);
// Get hardware status register value
uint32_t sys_sts_get_hw_status(struct sys_sts_t *sys_sts, bool verbose);
// Interpret and print hardware status
void print_hw_status(uint32_t hw_status, bool verbose);
// Print debug registers
void print_debug_registers(struct sys_sts_t *sys_sts);

#endif // SYS_STS_H
