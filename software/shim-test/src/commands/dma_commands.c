#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>

#include "dma_commands.h"
#include "command_helper.h"
#include "system_commands.h"   // safe_buffer_reset
#include "dma_ctrl.h"
#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "sys_ctrl.h"
#include "sys_sts.h"
#include "map_memory.h"        // board_count

// Settle time between the DMA'd DAC write, the ADC read trigger, and the capture. The DAC
// drives current through a coil that the ADC reads back, so this is a physical settle, not
// a bus delay -- expect to tune it on hardware.
#define DMA_TEST_SETTLE_US 100000

// Running-state check (the shared version is file-static in the other command modules).
static int dma_system_running(command_context_t* ctx) {
  uint32_t state = HW_STS_STATE(sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose)));
  if (state == S_HALTED) {
    printf("Error: Hardware manager is halted. Restart with: off / ctrl_on / pow_on\n");
    return -1;
  }
  if (state != S_RUNNING) {
    printf("Error: Hardware manager is not running (state %u). Use 'ctrl_on' and 'pow_on' first.\n", state);
    return -1;
  }
  return 0;
}

int cmd_dma_mode(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)flags; (void)flag_count;
  if (arg_count != 2) {
    fprintf(stderr, "Usage: dma_mode <board> <0|1>  (0 = PIO, 1 = DMA)\n");
    return -1;
  }
  int board = validate_board_number(args[0]);
  if (board < 0) return -1;

  char* endptr;
  long mode = strtol(args[1], &endptr, 0);
  if (*endptr != '\0' || (mode != 0 && mode != 1)) {
    fprintf(stderr, "Invalid mode '%s'. Must be 0 (PIO) or 1 (DMA).\n", args[1]);
    return -1;
  }

  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  uint8_t new_mask = mode ? (uint8_t)(mask | (1u << board))
                          : (uint8_t)(mask & ~(1u << board));
  sys_ctrl_set_datapath_mode(ctx->sys_ctrl, new_mask, *(ctx->verbose));

  uint8_t readback = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  if (readback != new_mask) {
    fprintf(stderr, "datapath_mode did not update (0x%02X, wanted 0x%02X). It is locked while the "
                    "system runs -- turn the system off first ('off').\n", readback, new_mask);
    return -1;
  }
  printf("datapath_mode = 0x%02X (board %d -> %s)\n", readback, board, mode ? "DMA" : "PIO");
  return 0;
}

static void dma_print_board_status(command_context_t* ctx, int board) {
  uint32_t dac_cmd  = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_data = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  printf("Board %d: dac_cmd fill=%u present=%u   adc_data fill=%u present=%u\n",
         board, FIFO_STS_WORD_COUNT(dac_cmd), FIFO_PRESENT(dac_cmd),
         FIFO_STS_WORD_COUNT(adc_data), FIFO_PRESENT(adc_data));
  dma_dump_status(ctx->dma_ctrl, board);
}

int cmd_dma_status(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)flags; (void)flag_count;
  if (!ctx->dma_ctrl->ok) {
    fprintf(stderr, "DMA is not available (/dev/mcdma or u-dma-buf missing).\n");
    return -1;
  }
  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  printf("datapath_mode = 0x%02X (a set bit selects DMA for that board)\n", mask);

  if (arg_count == 1) {
    int board = validate_board_number(args[0]);
    if (board < 0) return -1;
    dma_print_board_status(ctx, board);
  } else {
    for (int b = 0; b < board_count(); b++) dma_print_board_status(ctx, b);
  }
  return 0;
}

int cmd_dma_channel_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (arg_count != 2) {
    fprintf(stderr, "Usage: dma_channel_test <channel> <value>  (value -32767..32767)\n");
    return -1;
  }
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) return -1;

  char* endptr;
  int value = (int)parse_value(args[1], &endptr);
  if (*endptr != '\0' || value < -32767 || value > 32767) {
    fprintf(stderr, "Invalid DAC value '%s'. Must be -32767..32767.\n", args[1]);
    return -1;
  }
  if (!ctx->dma_ctrl->ok) {
    fprintf(stderr, "DMA is not available (/dev/mcdma or u-dma-buf missing).\n");
    return -1;
  }
  if (dma_system_running(ctx) != 0) return -1;

  // The board must be in DMA mode (set with 'dma_mode <board> 1' while the system is off).
  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  if (!((mask >> board) & 1u)) {
    fprintf(stderr, "Board %d is in PIO mode. Set 'dma_mode %d 1' while the system is off, then ctrl_on/pow_on.\n",
            board, board);
    return -1;
  }

  // FIFOs must be present. The ADC command lane stays PIO even in DMA mode.
  uint32_t dac_cmd_sts  = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_cmd_sts  = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_data_sts = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (!FIFO_PRESENT(dac_cmd_sts) || !FIFO_PRESENT(adc_cmd_sts) || !FIFO_PRESENT(adc_data_sts)) {
    fprintf(stderr, "Board %d FIFOs are not all present.\n", board);
    return -1;
  }

  bool verbose = *(ctx->verbose);
  printf("DMA channel test: channel %s (board %d, ch %d), value %d\n", args[0], board, channel, value);

  if (!has_flag(flags, flag_count, FLAG_NO_RESET)) {
    safe_buffer_reset(ctx, verbose);
    usleep(10000);
  }

  // MM2S: DMA a single DAC write command into the board's dac_cmd FIFO -> the DAC drives
  // current through the coil.
  uint32_t dac_words[8];
  int n = dac_encode_dac_wr_ch((uint8_t)channel, (int16_t)value, dac_words);
  if (dma_mm2s_send(ctx->dma_ctrl, board, dac_words, (uint32_t)n, verbose) != 0) return -1;
  usleep(DMA_TEST_SETTLE_US);   // let the DAC drive the coil

  // Arm S2MM BEFORE the ADC produces data: in DMA mode the packetizer drains adc_data
  // continuously, so the capture channel must already be running to receive the packet,
  // or it is dropped toward an idle S2MM.
  uint32_t cap[8] = {0};
  if (dma_s2mm_arm(ctx->dma_ctrl, board, 8, verbose) != 0) return -1;

  // PIO: trigger a single ADC read on the same channel (the adc_cmd lane is not gated).
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, verbose);

  // S2MM: wait for the captured sample(s) in DDR.
  int rx = dma_s2mm_wait(ctx->dma_ctrl, board, cap, 8, verbose);
  if (rx <= 0) {
    fprintf(stderr, "DMA channel test: no ADC data captured.\n");
    return -1;
  }

  printf("  Wrote DAC value %d to board %d channel %d via MM2S DMA.\n", value, board, channel);
  printf("  Captured %d ADC word(s) via S2MM DMA. First sample: %s\n", rx, adc_format_single(cap[0], verbose));
  if (verbose)
    for (int i = 0; i < rx; i++) printf("    adc_data[%d] = 0x%08X\n", i, cap[i]);
  return 0;
}
