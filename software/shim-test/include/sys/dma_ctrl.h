#ifndef DMA_CTRL_H
#define DMA_CTRL_H

#include <stdint.h>
#include <stdbool.h>

// MCDMA mover for the DMA datapath. Maps the MCDMA control window (via the pl-reg-shim
// /dev/mcdma node, /dev/mem fallback) and the two u-dma-buf regions (SG descriptors in
// udmabuf1, payloads in udmabuf0), and runs single-channel MM2S/S2MM transfers, polled
// to completion in the prebuffered model. Board b uses MCDMA channel b, matching the
// tdest = board wiring in the block design.
struct dma_ctrl_t {
  volatile uint8_t *reg;   // MCDMA control window
  int reg_fd;
  uint8_t  *desc;          // SG descriptor region (udmabuf1), mapped cached
  uint64_t  desc_phys;
  uint64_t  desc_size;
  int desc_fd;
  uint8_t  *data;          // payload/capture region (udmabuf0), mapped cached
  uint64_t  data_phys;
  uint64_t  data_size;
  int data_fd;
  bool ok;                 // true only if every mapping succeeded
};

// Map the MCDMA control window and both u-dma-buf regions. On any failure it prints a
// warning and returns a struct with ok == false, so a build/boot without the DMA nodes
// still lets the rest of the tool run.
struct dma_ctrl_t create_dma_ctrl(bool verbose);
void destroy_dma_ctrl(struct dma_ctrl_t *dma);

// Prebuffer n_words into DDR and push them through MM2S channel `board`, polled to
// completion. Returns 0 on success, negative on error. Injects a DAC command stream.
int dma_mm2s_send(struct dma_ctrl_t *dma, int board, const uint32_t *words, uint32_t n_words, bool verbose);

// Capture up to max_words from S2MM channel `board` into out, polled to completion.
// Returns the number of words received, or negative on error. Drains the ADC data lane.
// This is arm-then-wait in one call; use it only when the data is already queued.
int dma_s2mm_recv(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words, bool verbose);

// Two-phase S2MM for a live pipeline: arm the capture channel (it must be running before
// the PL produces the packet, or the packetizer drains it toward an idle S2MM and it is
// dropped), then wait for and copy out the captured words.
int dma_s2mm_arm(struct dma_ctrl_t *dma, int board, uint32_t max_words, bool verbose);
int dma_s2mm_wait(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words, bool verbose);

// Print the MCDMA common and per-channel status registers for `board` (debug).
void dma_dump_status(struct dma_ctrl_t *dma, int board);

// Interrupt-driven S2MM completion, for exercising the MCDMA -> hw_manager doorbell.
// Enable the S2MM channel's completion/error interrupt after arming so the MCDMA drives
// introut; classify the channel after a doorbell (1 = complete, -1 = error, 0 = neither);
// acknowledge (write-1-to-clear) so introut deasserts; and copy out the captured words
// without polling once completion is seen.
void dma_s2mm_irq_enable(struct dma_ctrl_t *dma, int board);
int  dma_s2mm_status(struct dma_ctrl_t *dma, int board);
void dma_s2mm_irq_ack(struct dma_ctrl_t *dma, int board);
int  dma_s2mm_collect(struct dma_ctrl_t *dma, uint32_t *out, uint32_t max_words);

#endif // DMA_CTRL_H
