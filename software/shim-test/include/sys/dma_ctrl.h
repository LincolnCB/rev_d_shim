#ifndef DMA_CTRL_H
#define DMA_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#include "map_memory.h"   // MAX_BOARDS

// A live, multi-word waveform run for one board, recorded between arming and teardown. The
// layout offsets index into the udmabuf regions: the DAC payload and the ADC capture share
// the data region (udmabuf0), the MM2S descriptor and the S2MM ring share the descriptor
// region (udmabuf1). cap_words is both the capture length and the number of one-word
// descriptors in the S2MM ring; cap_read is how many words have been read out so far (the
// read cursor for incremental draining). A concurrent run holds one of these per active
// board (see dma_wave_begin / dma_wave_add), each with its own region slice.
struct dma_wave_run {
  bool armed;
  int board;
  uint32_t mm2s_off, mm2s_len;   // DAC payload in the data region (bytes)
  uint32_t cap_off, cap_words;   // ADC capture in the data region; cap_words = ring length
  uint32_t cap_read;             // words already read out (read cursor)
  uint32_t mm2s_desc_off;        // MM2S descriptor in the descriptor region
  uint32_t ring_desc_off;        // S2MM descriptor ring base in the descriptor region
};

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
  struct dma_wave_run wave[MAX_BOARDS]; // per-board run state (see dma_wave_begin / _add)
  uint32_t wave_data_cursor; // bump allocator over the data region, reset by dma_wave_begin
  uint32_t wave_desc_cursor; // bump allocator over the descriptor region, reset likewise
  int wave_single_board;     // board targeted by the no-board single-run wrappers (-1 = none)
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

// --- Multi-word waveform run (prebuffered DAC stream + streamed ADC capture) ---
//
// Low-level DMA primitives only; the polling loop, file output, and waveform synthesis live
// in the command layer (see dma_waveform_commands.c), mirroring how the DAC/ADC split their
// low-level ctrl from the experiment orchestration.
//
// dma_wave_arm lays out the DAC payload and ADC capture in the data region and the MM2S
// descriptor and S2MM ring in the descriptor region, checking both fit; copies the DAC
// words into DDR; arms and starts the S2MM capture ring (one one-word descriptor per ADC
// word, so a k-word packet spans k descriptors and the capture is contiguous); and starts
// MM2S, which fills the DAC FIFO and then stalls behind the DAC's leading trigger-wait
// command. It fires no trigger and waits on nothing -- the PL fills the ring whenever it
// has data, across any number of triggers. cap_words is the exact expected capture length
// (4 words per 8-channel read). Returns 0 on success, negative on error (with a clear
// message if the run does not fit the udmabuf regions).
//
// This is the single-board wrapper (one run at a time): it runs dma_wave_begin then one
// dma_wave_add for `board`, and the no-board accessors below (dma_wave_avail / _read /
// _read_total / _expected) operate on that board. For a concurrent multi-board run use
// dma_wave_begin / dma_wave_add plus the per-board accessors directly.
int dma_wave_arm(struct dma_ctrl_t *dma, int board,
                 const uint32_t *dac_words, uint32_t dac_n_words,
                 uint32_t cap_words, bool verbose);

// --- Concurrent multi-board waveform run ---
//
// A real run drives every active board at once, so the run programs partition the udmabuf
// regions per board: dma_wave_begin resets both MCDMA directions once (the soft-reset is
// global) and clears the region allocator; dma_wave_add then carves a region slice for each
// board, builds its MM2S descriptor and S2MM ring, and arms+runs+triggers both of that
// board's channels (MM2S prebuffers the DAC FIFO behind the leading trigger-wait; S2MM
// starts capturing). No trigger is fired and nothing is waited on -- the caller releases the
// trigger system and drains each board's capture with the per-board accessors.
//
// dma_wave_begin returns 0 on success, negative on error. dma_wave_add returns 0 on success,
// negative on error (with a clear message if the board's slice does not fit). cap_words == 0
// means DAC-only: the MM2S stream is prebuffered but no S2MM capture ring is built or armed.
int dma_wave_begin(struct dma_ctrl_t *dma, bool verbose);
int dma_wave_add(struct dma_ctrl_t *dma, int board,
                 const uint32_t *dac_words, uint32_t dac_n_words,
                 uint32_t cap_words, bool verbose);

// Per-board capture accessors (mirror the single-run ones below, indexed by board).
uint32_t dma_wave_avail_board(struct dma_ctrl_t *dma, int board);
int      dma_wave_read_board(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words);
uint32_t dma_wave_read_total_board(const struct dma_ctrl_t *dma, int board);
uint32_t dma_wave_expected_board(const struct dma_ctrl_t *dma, int board);
bool     dma_wave_board_armed(const struct dma_ctrl_t *dma, int board);

// Disarm every board (software teardown; the hardware rings are left for the next begin or a
// buffer reset).
void dma_wave_disarm_all(struct dma_ctrl_t *dma);

// Halt the MCDMA: soft-reset both directions (stopping all channels and draining the S2MM
// downstream path), then clear all per-board run state and the region allocator. The
// coordinated-shutdown counterpart to dma_wave_begin -- call it before asserting FIFO
// buf_reset so the engine is not mid-transfer when the FIFOs reset.
void dma_wave_halt(struct dma_ctrl_t *dma, bool verbose);

// Words captured into DDR but not yet read out (contiguous completed descriptors past the
// read cursor). Cheap to poll.
uint32_t dma_wave_avail(struct dma_ctrl_t *dma);

// Copy up to max_words newly-available capture words into out, advancing the read cursor.
// Returns the number of words copied (0 if none are ready), negative on error.
int dma_wave_read(struct dma_ctrl_t *dma, uint32_t *out, uint32_t max_words);

// The read cursor and the total expected capture, both in words.
uint32_t dma_wave_read_total(const struct dma_ctrl_t *dma);
uint32_t dma_wave_expected(const struct dma_ctrl_t *dma);

// Clear the armed run (software teardown; the hardware ring is left for the next arm or a
// buffer reset). Safe to call whether or not a run is armed.
void dma_wave_disarm(struct dma_ctrl_t *dma);

// True if a waveform run is armed.
bool dma_wave_is_armed(const struct dma_ctrl_t *dma);

#endif // DMA_CTRL_H
