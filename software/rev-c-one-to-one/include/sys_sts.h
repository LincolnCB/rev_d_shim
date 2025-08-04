#ifndef SYS_STS_H
#define SYS_STS_H

#include <stdint.h>
#include <stdbool.h>

//////////////////// System Status Definitions ////////////////////
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

//////////////////////////////////////////////////////////////////

// System status structure
struct sys_sts_t {
  volatile uint32_t *hw_status_code;         // Hardware status code
  volatile uint32_t *dac_cmd_fifo_sts[8];    // DAC command FIFO status for 8 boards
  volatile uint32_t *adc_cmd_fifo_sts[8];    // ADC command FIFO status for 8 boards
  volatile uint32_t *adc_data_fifo_sts[8];   // ADC data FIFO status for 8 boards
  volatile uint32_t *trig_cmd_fifo_sts;      // Trigger command FIFO status
  volatile uint32_t *trig_data_fifo_sts;     // Trigger data FIFO status
};

// Function declaration
struct sys_sts_t create_sys_sts(bool verbose);

#endif // SYS_STS_H
