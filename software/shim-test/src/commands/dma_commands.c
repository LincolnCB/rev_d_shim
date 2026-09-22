#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

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

// pl-irq-shim node carrying the single hw_manager interrupt (matches sys_sts.c).
#define HW_MANAGER_IRQ_DEV "/dev/hw_manager_irq"

int cmd_dma_irq_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  if (arg_count != 2) {
    fprintf(stderr, "Usage: dma_irq_test <channel> <value>  (value -32767..32767) [--no_reset]\n");
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

  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  if (!((mask >> board) & 1u)) {
    fprintf(stderr, "Board %d is in PIO mode. Set 'dma_mode %d 1' while the system is off, then ctrl_on/pow_on.\n",
            board, board);
    return -1;
  }
  uint32_t dac_cmd_sts  = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_cmd_sts  = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_data_sts = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (!FIFO_PRESENT(dac_cmd_sts) || !FIFO_PRESENT(adc_cmd_sts) || !FIFO_PRESENT(adc_data_sts)) {
    fprintf(stderr, "Board %d FIFOs are not all present.\n", board);
    return -1;
  }

  bool verbose = *(ctx->verbose);
  printf("DMA interrupt test: channel %s (board %d, ch %d), value %d\n", args[0], board, channel, value);

  int irq_fd = open(HW_MANAGER_IRQ_DEV, O_RDWR);
  if (irq_fd < 0) {
    perror("open " HW_MANAGER_IRQ_DEV);
    return -1;
  }
  // Arm the pl-irq (enable_irq is balanced; a no-op if already enabled) before waiting.
  uint32_t arm = 1;
  if (write(irq_fd, &arm, sizeof(arm)) != (ssize_t)sizeof(arm))
    fprintf(stderr, "warning: failed to arm the interrupt\n");

  if (!has_flag(flags, flag_count, FLAG_NO_RESET)) {
    safe_buffer_reset(ctx, verbose);
    usleep(10000);
  }

  // MM2S the DAC write (polled; only the S2MM interrupt is enabled below, so the doorbell
  // comes from the capture completion).
  uint32_t dac_words[8];
  int n = dac_encode_dac_wr_ch((uint8_t)channel, (int16_t)value, dac_words);
  if (dma_mm2s_send(ctx->dma_ctrl, board, dac_words, (uint32_t)n, verbose) != 0) { close(irq_fd); return -1; }
  usleep(DMA_TEST_SETTLE_US);

  // Arm S2MM before the ADC produces data, then enable its completion interrupt so the
  // MCDMA drives introut -> hw_manager pulses its single ps_interrupt.
  uint32_t cap[8] = {0};
  if (dma_s2mm_arm(ctx->dma_ctrl, board, 8, verbose) != 0) { close(irq_fd); return -1; }
  dma_s2mm_irq_enable(ctx->dma_ctrl, board);

  // PIO trigger (the adc_cmd lane is not gated).
  adc_cmd_adc_rd_ch(ctx->adc_ctrl, (uint8_t)board, (uint8_t)channel, 0, verbose);

  // Block on the hw_manager interrupt until the S2MM channel reports completion. The
  // introut is only a doorbell; the MCDMA channel status register is the source of truth.
  int interrupts = 0;
  int st = 0;
  for (int elapsed_ms = 0; elapsed_ms < 5000 && st == 0; elapsed_ms += 500) {
    struct pollfd pfd = { .fd = irq_fd, .events = POLLIN, .revents = 0 };
    int pr = poll(&pfd, 1, 500);
    if (pr < 0) { perror("poll"); break; }
    if (pr > 0 && (pfd.revents & POLLIN)) {
      uint32_t cnt = 0;
      if (read(irq_fd, &cnt, sizeof(cnt)) == (ssize_t)sizeof(cnt)) interrupts++;
      uint32_t rearm = 1;
      if (write(irq_fd, &rearm, sizeof(rearm)) != (ssize_t)sizeof(rearm))
        fprintf(stderr, "warning: failed to re-arm the interrupt\n");
    }
    st = dma_s2mm_status(ctx->dma_ctrl, board);
    if (st == 0 && HW_STS_STATE(sys_sts_get_hw_status(ctx->sys_sts, false)) != S_RUNNING)
      st = -1;   // a fault halted the run
  }

  dma_s2mm_irq_ack(ctx->dma_ctrl, board);
  close(irq_fd);

  if (st <= 0) {
    fprintf(stderr, "DMA interrupt test: no clean S2MM completion (interrupts=%d, status=%d).\n", interrupts, st);
    dma_dump_status(ctx->dma_ctrl, board);
    return -1;
  }

  int rx = dma_s2mm_collect(ctx->dma_ctrl, cap, 8);
  if (rx <= 0) {
    fprintf(stderr, "DMA interrupt test: S2MM completed but no data was captured.\n");
    return -1;
  }
  if (interrupts > 0)
    printf("  Completion delivered by the hw_manager interrupt (%d doorbell%s).\n",
           interrupts, interrupts == 1 ? "" : "s");
  else
    printf("  S2MM completed but no interrupt was observed -- the doorbell did not fire (check the introut fold).\n");
  printf("  Captured %d ADC word(s) via S2MM DMA. First sample: %s\n", rx, adc_format_single(cap[0], verbose));
  return interrupts > 0 ? 0 : -1;
}

int cmd_dma_mode_viol(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)flags; (void)flag_count;
  if (arg_count != 1) {
    fprintf(stderr, "Usage: dma_mode_viol <channel>  (wrong-mode PIO poke at a DMA-mode board)\n");
    return -1;
  }
  int board, channel;
  if (validate_channel_number(args[0], &board, &channel) < 0) return -1;
  if (dma_system_running(ctx) != 0) return -1;

  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  if (!((mask >> board) & 1u)) {
    fprintf(stderr, "Board %d is in PIO mode, so a PIO access is not a violation. "
                    "Set 'dma_mode %d 1' while the system is off, then ctrl_on/pow_on.\n", board, board);
    return -1;
  }

  bool verbose = *(ctx->verbose);
  uint32_t before = sys_sts_get_hw_status(ctx->sys_sts, false);
  printf("Mode-violation test: board %d is in DMA mode; issuing a PIO DAC write on channel %d.\n", board, channel);
  printf("  hw_status before: ");
  print_hw_status(before, verbose);

  // Deliberate wrong-mode access: the dac_cmd write port belongs to the DMA owner in DMA
  // mode, so the bridge accepts-and-discards this and raises mode_viol. If the bridge still
  // returned SLVERR this write would SIGBUS; reaching the next line proves the OKAY rework.
  dac_cmd_dac_wr_ch(ctx->dac_ctrl, (uint8_t)board, (uint8_t)channel, 0, verbose);
  printf("  PIO write returned without a bus error (accept-and-discard confirmed).\n");

  // hw_manager should fold mode_viol into a coordinated halt with STS_MODE_VIOL.
  uint32_t after = before;
  bool left_running = false;
  for (int i = 0; i < 250; i++) {
    after = sys_sts_get_hw_status(ctx->sys_sts, false);
    if (HW_STS_STATE(after) != S_RUNNING) { left_running = true; break; }
    usleep(1000);
  }
  printf("  hw_status after:  ");
  print_hw_status(after, verbose);

  if (left_running && HW_STS_CODE(after) == STS_MODE_VIOL && (int)HW_STS_BOARD(after) == board) {
    printf("PASS: wrong-mode access on board %d reported STS_MODE_VIOL and halted gracefully.\n", board);
    printf("  Restart with: off / ctrl_on / pow_on\n");
    return 0;
  }
  fprintf(stderr, "FAIL: expected a graceful STS_MODE_VIOL halt on board %d; got code 0x%04" PRIx32
                  " (board %u, state %u).\n",
          board, HW_STS_CODE(after), HW_STS_BOARD(after), HW_STS_STATE(after));
  return -1;
}
