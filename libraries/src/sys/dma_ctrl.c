// MCDMA mover -- single-channel MM2S/S2MM transfers for the DMA datapath.
//
// The register model, SG descriptor layout (control at 0x14, SOF/EOF at bits 31/30),
// and the arm/run/trigger sequence match mainline drivers/dma/xilinx/xilinx_dma.c and
// the mcdma-loopback bring-up tool. Channels are addressed 0-based by board index (board
// b == MCDMA channel b == tdest b in the block design).

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "dma_ctrl.h"

// ------------------------------------------------------------------- config --

#define MCDMA_DEV       "/dev/mcdma"    // pl-reg-shim node (non-root)
#define MCDMA_MEM_BASE  0x40400000UL    // /dev/mem fallback (block_design.tcl)
#define MCDMA_WIN_BYTES 0x10000UL       // control-window size (64K)

#define DATA_UDMABUF    "udmabuf0"      // payloads + capture
#define DATA_DEV        "/dev/" DATA_UDMABUF
#define DATA_SYS        "/sys/class/u-dma-buf/" DATA_UDMABUF
#define DESC_UDMABUF    "udmabuf1"      // SG descriptors
#define DESC_DEV        "/dev/" DESC_UDMABUF
#define DESC_SYS        "/sys/class/u-dma-buf/" DESC_UDMABUF

#define RATE_CTRL_DEV       "/dev/adc_rate_ctrl" // adc_rate_ctrl register window (ships in the bitstream)
#define RATE_CTRL_WIN_BYTES 0x1000UL             // one page
// adc_rate_ctrl register map (32-bit word indices), mirroring adc_rate_ctrl.v. The packing
// threshold is a fixed RTL parameter (the packet size), not a register -- software only reports
// read deltas and reads back the per-board backlog.
#define RATE_READ_DELTA_BASE 0u
#define RATE_BACKLOG_BASE    8u

#define DESC_ALIGN      64u             // MCDMA SG descriptor size/alignment
// DAC payload split into this many words per MM2S packet descriptor, so the MCDMA completes one
// board's packet then arbitrates to the next board's channel -- the per-board FIFOs fill in
// lockstep instead of board 0 monopolizing the shared MM2S output. 256 words (= one 1024-byte
// descriptor buffer, matching the S2MM capture packet) keeps the DAC descriptor-efficient: 4 MB
// of descriptors backs the full 64 MB DAC budget. Still far under the FIFO depth, so the
// arbitration stays fair across boards.
#define MM2S_PACKET_WORDS 256u
// ADC capture descriptor buffer capacity (words). The S2MM engine fills 1..this per packet
// (closed at the packetizer's tlast or buffer-full) and records the real length; readback
// compacts the partial buffers. Fixed at the packetizer's MAX_PACKET_WORDS (DMA_CAPTURE_PACKET_WORDS).
#define S2MM_PACKET_WORDS DMA_CAPTURE_PACKET_WORDS

// Fixed slots in the payload region: one MM2S payload and one S2MM capture buffer. Only
// one transfer runs at a time in the low-level test path, so fixed offsets suffice.
#define MM2S_PAYLOAD_OFF  0u
#define MM2S_PAYLOAD_MAX  (8u * 1024u)
#define S2MM_CAPTURE_OFF  (8u * 1024u)
#define S2MM_CAPTURE_MAX  (8u * 1024u)
#define DATA_NEED         (S2MM_CAPTURE_OFF + S2MM_CAPTURE_MAX)

// Fixed descriptor slots (one per direction).
#define MM2S_DESC_OFF     0u
#define S2MM_DESC_OFF     DESC_ALIGN
#define DESC_NEED         (S2MM_DESC_OFF + DESC_ALIGN)

#define POLL_TIMEOUT_US   1000000       // 1 s

// ------------------------------------------------ MCDMA register offsets --
// Confirmed against mainline xilinx_dma.c (MCDMA path).

#define MM2S_CTRL       0x000           // common control (bit0 RS, bit2 reset)
#define MM2S_SR         0x004           // common status (bit0 HALTED)
#define MM2S_CHEN       0x008           // channel enable bitmask (bit b = channel b)
#define MM2S_CH_ERR     0x010
#define S2MM_CTRL       0x500
#define S2MM_SR         0x504
#define S2MM_CHEN       0x508
#define S2MM_CH_ERR     0x510

// Per-channel register blocks, board index (0-based): 0x40 + b*0x40 from the direction
// base (0x000 MM2S, 0x500 S2MM).
#define MM2S_CH_BASE(b) (0x040 + (b) * 0x40)
#define S2MM_CH_BASE(b) (0x540 + (b) * 0x40)
#define CH_CR           0x00
#define CH_SR           0x04
#define CH_CURDESC      0x08
#define CH_CURDESC_MSB  0x0C
#define CH_TAILDESC     0x10
#define CH_TAILDESC_MSB 0x14

#define CR_RS           0x00000001
#define CR_RESET        0x00000004
#define SR_HALTED       0x00000001

// MCDMA per-channel interrupt enable (CH_CR) and status (CH_SR) bits. The MCDMA layout
// puts IOC/DLY/ERR at bits 5/6/7 (not the AXI-DMA 12/13/14); the coalesce threshold in
// [23:16] must be at least 1 or IOC never fires. Confirmed against xilinx_dma.c (MCDMA).
#define CH_IRQ_IOC      0x00000020   // bit 5: completion
#define CH_IRQ_DLY      0x00000040   // bit 6: delay
#define CH_IRQ_ERR      0x00000080   // bit 7: error
#define CH_IRQ_THRESH1  0x00010000   // [23:16] = 1 coalesce threshold

#define DESC_CTRL_SOF      0x80000000   // start-of-packet (BIT 31)
#define DESC_CTRL_EOF      0x40000000   // end-of-packet   (BIT 30)
#define DESC_CTRL_LEN_MASK 0x03FFFFFF
#define DESC_STAT_CMPLT    0x80000000   // descriptor completed (BIT 31)
#define DESC_STAT_LEN_MASK 0x03FFFFFF

// MCDMA hardware descriptor (struct xilinx_aximcdma_desc_hw): control at 0x14, status at
// 0x18. 64 bytes, 64-byte aligned.
struct mcdma_desc {
  uint32_t nxtdesc;
  uint32_t nxtdesc_msb;
  uint32_t buffer_addr;
  uint32_t buffer_addr_msb;
  uint32_t rsvd;
  uint32_t control;      // SOF | EOF | length
  uint32_t status;       // CMPLT | transferred length
  uint32_t sideband_status;
  uint32_t app[8];
};

// ------------------------------------------------------------- small helpers --

static void reg_w(volatile uint8_t *base, uint32_t off, uint32_t val) {
  *(volatile uint32_t *)(base + off) = val;
}
static uint32_t reg_r(volatile uint8_t *base, uint32_t off) {
  return *(volatile uint32_t *)(base + off);
}
static int64_t now_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

// Read a u-dma-buf sysfs integer. phys_addr is 0x-hex, size is decimal; strtoull(base 0)
// handles both.
static int sysfs_read_num(const char *path, uint64_t *out) {
  FILE *f = fopen(path, "r");
  if (!f) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); return -1; }
  char buf[64] = {0};
  char *p = fgets(buf, sizeof(buf), f);
  fclose(f);
  if (!p) return -1;
  *out = strtoull(buf, NULL, 0);
  return 0;
}

// Trigger a u-dma-buf cache sync over [offset, offset+size). Unused now that the regions
// are mapped non-cached (open with O_SYNC), kept out of the build.

// ------------------------------------------------------- control-window map --

static volatile uint8_t *map_control_window(int *fd_out, bool verbose) {
  int fd = open(MCDMA_DEV, O_RDWR);
  if (fd >= 0) {
    void *p = mmap(NULL, MCDMA_WIN_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p != MAP_FAILED) {
      if (verbose) printf("MCDMA control: %s (pl-reg-shim, no root)\n", MCDMA_DEV);
      *fd_out = fd;
      return p;
    }
    close(fd);
  }
  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
    fprintf(stderr, "DMA: open /dev/mem: %s (and %s not present)\n", strerror(errno), MCDMA_DEV);
    return NULL;
  }
  void *p = mmap(NULL, MCDMA_WIN_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, MCDMA_MEM_BASE);
  if (p == MAP_FAILED) {
    fprintf(stderr, "DMA: mmap /dev/mem @ 0x%lx: %s\n", MCDMA_MEM_BASE, strerror(errno));
    close(fd);
    return NULL;
  }
  if (verbose) printf("MCDMA control: /dev/mem @ 0x%lx (root)\n", MCDMA_MEM_BASE);
  *fd_out = fd;
  return p;
}

// Map a u-dma-buf region non-cached; read phys/size from sysfs; mmap `need` bytes. O_SYNC
// tells u-dma-buf to map the region non-cached, so the descriptor write-back and captured
// data are directly coherent with no explicit cache sync.
static uint8_t *map_udmabuf(const char *dev, const char *sysdir, const char *name,
                            uint64_t need, uint64_t *phys_out, uint64_t *size_out,
                            int *fd_out, bool verbose) {
  int fd = open(dev, O_RDWR | O_SYNC);
  if (fd < 0) {
    fprintf(stderr, "DMA: open %s: %s\n", dev, strerror(errno));
    if (errno == ENOENT)
      fprintf(stderr, "  Is u-dma-buf loaded with a '%s' region? (dmesg | grep u-dma-buf)\n", name);
    return NULL;
  }
  uint64_t phys = 0, size = 0;
  char path[160];
  snprintf(path, sizeof(path), "%s/phys_addr", sysdir);
  if (sysfs_read_num(path, &phys)) { close(fd); return NULL; }
  snprintf(path, sizeof(path), "%s/size", sysdir);
  if (sysfs_read_num(path, &size)) { close(fd); return NULL; }
  if (size < need) {
    fprintf(stderr, "DMA: %s is %" PRIu64 " bytes, need %" PRIu64 "\n", name, size, need);
    close(fd);
    return NULL;
  }
  uint8_t *p = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    fprintf(stderr, "DMA: mmap %s: %s\n", dev, strerror(errno));
    close(fd);
    return NULL;
  }
  if (verbose)
    printf("u-dma-buf %s: phys 0x%" PRIx64 ", %" PRIu64 " bytes\n", name, phys, size);
  *phys_out = phys;
  *size_out = size;
  *fd_out = fd;
  return p;
}

// Map the adc_rate_ctrl register window (one page, non-cached). The core ships in the bitstream,
// so a missing node is a build/deploy error -- fail loudly rather than tolerating its absence.
static volatile uint32_t *map_rate_ctrl(int *fd_out, bool verbose) {
  int fd = open(RATE_CTRL_DEV, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "DMA: open %s: %s\n", RATE_CTRL_DEV, strerror(errno));
    return NULL;
  }
  void *p = mmap(NULL, RATE_CTRL_WIN_BYTES, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (p == MAP_FAILED) {
    fprintf(stderr, "DMA: mmap %s: %s\n", RATE_CTRL_DEV, strerror(errno));
    close(fd);
    return NULL;
  }
  if (verbose) printf("adc_rate_ctrl: %s (pl-reg-shim, no root)\n", RATE_CTRL_DEV);
  *fd_out = fd;
  return (volatile uint32_t *)p;
}

// ------------------------------------------------------------- descriptors --

// One single-buffer, single-packet descriptor covering [buf_phys, buf_phys+len), marked start-
// and end-of-packet, linking to next_phys (pass the descriptor's own phys to self-loop a lone
// descriptor). MM2S drives tdest from the channel, so no tdest field here.
static void build_desc(struct mcdma_desc *d, uint64_t next_phys, uint64_t buf_phys, uint32_t len) {
  memset(d, 0, sizeof(*d));
  d->nxtdesc         = (uint32_t)next_phys;
  d->nxtdesc_msb     = (uint32_t)(next_phys >> 32);
  d->buffer_addr     = (uint32_t)buf_phys;
  d->buffer_addr_msb = (uint32_t)(buf_phys >> 32);
  d->control         = DESC_CTRL_SOF | DESC_CTRL_EOF | (len & DESC_CTRL_LEN_MASK);
}

// One S2MM receive descriptor of `len` bytes at buf_phys, linked to next_phys. For S2MM
// the SOF/EOF bits are written by the engine on receive, so the control word carries only
// the buffer length.
static void build_rx_desc(struct mcdma_desc *d, uint64_t next_phys, uint64_t buf_phys, uint32_t len) {
  memset(d, 0, sizeof(*d));
  d->nxtdesc         = (uint32_t)next_phys;
  d->nxtdesc_msb     = (uint32_t)(next_phys >> 32);
  d->buffer_addr     = (uint32_t)buf_phys;
  d->buffer_addr_msb = (uint32_t)(buf_phys >> 32);
  d->control         = len & DESC_CTRL_LEN_MASK;
}

// Soft-reset a whole direction via its common control register (bit 2).
static void reset_direction(volatile uint8_t *r, uint32_t ctrl) {
  reg_w(r, ctrl, CR_RESET);
  int64_t deadline = now_us() + POLL_TIMEOUT_US;
  while (reg_r(r, ctrl) & CR_RESET)
    if (now_us() > deadline) { fprintf(stderr, "DMA: reset (ctrl 0x%x) did not clear\n", ctrl); break; }
}

// Arm channel `board`: program its current descriptor, enable it in CHEN, set per-channel RS.
static void arm_channel(volatile uint8_t *r, uint32_t ch_base, uint32_t chen, int board, uint64_t desc_phys) {
  reg_w(r, ch_base + CH_CURDESC,     (uint32_t)desc_phys);
  reg_w(r, ch_base + CH_CURDESC_MSB, (uint32_t)(desc_phys >> 32));
  reg_w(r, chen, reg_r(r, chen) | (1u << board));
  reg_w(r, ch_base + CH_CR, reg_r(r, ch_base + CH_CR) | CR_RS);
}

// Set the direction's common Run/Stop and wait for it to leave halted.
static void run_direction(volatile uint8_t *r, uint32_t ctrl) {
  reg_w(r, ctrl, reg_r(r, ctrl) | CR_RS);
  int64_t deadline = now_us() + POLL_TIMEOUT_US;
  while (reg_r(r, ctrl + 0x04) & SR_HALTED)
    if (now_us() > deadline) { fprintf(stderr, "DMA: direction (ctrl 0x%x) stayed halted\n", ctrl); break; }
}

// Write the tail descriptor -- this starts the BD fetch.
static void trigger_channel(volatile uint8_t *r, uint32_t ch_base, uint64_t desc_phys) {
  reg_w(r, ch_base + CH_TAILDESC,     (uint32_t)desc_phys);
  reg_w(r, ch_base + CH_TAILDESC_MSB, (uint32_t)(desc_phys >> 32));
}

// Wait for a transfer to complete. The descriptor completion bit (directly visible with
// the non-cached mapping) is primary; the engine IDLE status (SR bit 1) is a fallback
// after a short margin in case the descriptor write-back is absent. `sr_off` = MM2S_SR
// or S2MM_SR.
static int poll_complete(struct dma_ctrl_t *dma, uint32_t desc_off, uint32_t sr_off) {
  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + desc_off);
  int64_t start = now_us();
  for (;;) {
    if (d->status & DESC_STAT_CMPLT) return 0;
    if ((now_us() - start) > 10000 && (reg_r(dma->reg, sr_off) & 0x2)) return 0;
    if ((now_us() - start) > POLL_TIMEOUT_US) return -1;
  }
}

static uint32_t align_up_u32(uint32_t v, uint32_t a) {
  return (v + a - 1u) & ~(a - 1u);
}

// ------------------------------------------------------------- public API --

struct dma_ctrl_t create_dma_ctrl(bool verbose) {
  struct dma_ctrl_t dma;
  memset(&dma, 0, sizeof(dma));
  dma.reg_fd = dma.desc_fd = dma.data_fd = dma.rate_ctrl_fd = -1;
  dma.wave_single_board = -1;
  dma.ok = false;

  dma.reg = map_control_window(&dma.reg_fd, verbose);
  if (!dma.reg) {
    fprintf(stderr, "DMA control window unavailable; DMA commands disabled.\n");
    return dma;
  }
  dma.desc = map_udmabuf(DESC_DEV, DESC_SYS, DESC_UDMABUF, DESC_NEED,
                         &dma.desc_phys, &dma.desc_size, &dma.desc_fd, verbose);
  dma.data = map_udmabuf(DATA_DEV, DATA_SYS, DATA_UDMABUF, DATA_NEED,
                         &dma.data_phys, &dma.data_size, &dma.data_fd, verbose);
  if (!dma.desc || !dma.data) {
    fprintf(stderr, "DMA u-dma-buf regions unavailable; DMA commands disabled.\n");
    return dma;
  }
  dma.rate_ctrl = map_rate_ctrl(&dma.rate_ctrl_fd, verbose);
  if (!dma.rate_ctrl) {
    fprintf(stderr, "DMA adc_rate_ctrl window unavailable; DMA commands disabled.\n");
    return dma;
  }
  dma.ok = true;
  return dma;
}

void destroy_dma_ctrl(struct dma_ctrl_t *dma) {
  if (!dma) return;
  if (dma->reg)  munmap((void *)dma->reg, MCDMA_WIN_BYTES);
  if (dma->desc) munmap(dma->desc, dma->desc_size);
  if (dma->data) munmap(dma->data, dma->data_size);
  if (dma->rate_ctrl) munmap((void *)dma->rate_ctrl, RATE_CTRL_WIN_BYTES);
  if (dma->reg_fd  >= 0) close(dma->reg_fd);
  if (dma->desc_fd >= 0) close(dma->desc_fd);
  if (dma->data_fd >= 0) close(dma->data_fd);
  if (dma->rate_ctrl_fd >= 0) close(dma->rate_ctrl_fd);
  memset(dma, 0, sizeof(*dma));
  dma->reg_fd = dma->desc_fd = dma->data_fd = dma->rate_ctrl_fd = -1;
}

// --- ADC capture rate control (adc_rate_ctrl). The core is mapped at create time (required), so
// these assume the register window is present. The packing threshold is fixed in the bitstream
// (the packet size), so software only reports read deltas and reads the per-board backlog. ---

void dma_rate_report_read(struct dma_ctrl_t *dma, int board, uint32_t words) {
  if (!dma || board < 0 || board >= MAX_BOARDS || words == 0) return;
  dma->rate_ctrl[RATE_READ_DELTA_BASE + (uint32_t)board] = words;
}

uint32_t dma_rate_backlog(const struct dma_ctrl_t *dma, int board) {
  if (!dma || board < 0 || board >= MAX_BOARDS) return 0;
  return dma->rate_ctrl[RATE_BACKLOG_BASE + (uint32_t)board];
}

int dma_mm2s_send(struct dma_ctrl_t *dma, int board, const uint32_t *words, uint32_t n_words, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  if (n_words == 0 || n_words * 4u > MM2S_PAYLOAD_MAX) {
    fprintf(stderr, "DMA MM2S: bad word count %u (max %u).\n", n_words, MM2S_PAYLOAD_MAX / 4u);
    return -1;
  }
  uint32_t len = n_words * 4u;
  volatile uint8_t *r = dma->reg;

  memcpy(dma->data + MM2S_PAYLOAD_OFF, words, len);

  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + MM2S_DESC_OFF);
  uint64_t desc_phys = dma->desc_phys + MM2S_DESC_OFF;
  build_desc(d, desc_phys, dma->data_phys + MM2S_PAYLOAD_OFF, len);
  __sync_synchronize(); // ensure payload + descriptor are in DDR before the engine reads them

  reset_direction(r, MM2S_CTRL);
  arm_channel(r, MM2S_CH_BASE(board), MM2S_CHEN, board, desc_phys);
  run_direction(r, MM2S_CTRL);
  trigger_channel(r, MM2S_CH_BASE(board), desc_phys);

  if (poll_complete(dma, MM2S_DESC_OFF, MM2S_SR) < 0) {
    fprintf(stderr, "DMA MM2S ch%d: descriptor did not complete.\n", board);
    dma_dump_status(dma, board);
    return -1;
  }
  if (verbose) printf("DMA MM2S ch%d: sent %u words.\n", board, n_words);
  return 0;
}

int dma_s2mm_arm(struct dma_ctrl_t *dma, int board, uint32_t max_words, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  if (max_words == 0 || max_words * 4u > S2MM_CAPTURE_MAX) {
    fprintf(stderr, "DMA S2MM: bad word count %u (max %u).\n", max_words, S2MM_CAPTURE_MAX / 4u);
    return -1;
  }
  uint32_t len = max_words * 4u;
  volatile uint8_t *r = dma->reg;

  memset(dma->data + S2MM_CAPTURE_OFF, 0, len);
  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + S2MM_DESC_OFF);
  uint64_t desc_phys = dma->desc_phys + S2MM_DESC_OFF;
  build_desc(d, desc_phys, dma->data_phys + S2MM_CAPTURE_OFF, len);
  __sync_synchronize();

  reset_direction(r, S2MM_CTRL);
  arm_channel(r, S2MM_CH_BASE(board), S2MM_CHEN, board, desc_phys);
  run_direction(r, S2MM_CTRL);
  trigger_channel(r, S2MM_CH_BASE(board), desc_phys);
  if (verbose) printf("DMA S2MM ch%d: armed for up to %u words.\n", board, max_words);
  return 0;
}

int dma_s2mm_wait(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + S2MM_DESC_OFF);
  if (poll_complete(dma, S2MM_DESC_OFF, S2MM_SR) < 0) {
    fprintf(stderr, "DMA S2MM ch%d: capture did not complete (no packet arrived?).\n", board);
    dma_dump_status(dma, board);
    return -1;
  }
  uint32_t rx_words = (d->status & DESC_STAT_LEN_MASK) / 4u;
  if (rx_words > max_words) rx_words = max_words;
  memcpy(out, dma->data + S2MM_CAPTURE_OFF, (size_t)rx_words * 4u);
  if (verbose) printf("DMA S2MM ch%d: received %u words.\n", board, rx_words);
  return (int)rx_words;
}

// Convenience: arm then immediately wait. Only useful when data is already queued in the
// FIFO; for a live pipeline, arm before the source produces (see dma_channel_test).
int dma_s2mm_recv(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words, bool verbose) {
  int rc = dma_s2mm_arm(dma, board, max_words, verbose);
  if (rc < 0) return rc;
  return dma_s2mm_wait(dma, board, out, max_words, verbose);
}

void dma_dump_status(struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return; }
  volatile uint8_t *r = dma->reg;
  printf("  MM2S SR=0x%08x CH_ERR=0x%08x   S2MM SR=0x%08x CH_ERR=0x%08x\n",
         reg_r(r, MM2S_SR), reg_r(r, MM2S_CH_ERR), reg_r(r, S2MM_SR), reg_r(r, S2MM_CH_ERR));
  printf("  ch%d  MM2S CR=0x%08x SR=0x%08x   S2MM CR=0x%08x SR=0x%08x\n", board,
         reg_r(r, MM2S_CH_BASE(board) + CH_CR), reg_r(r, MM2S_CH_BASE(board) + CH_SR),
         reg_r(r, S2MM_CH_BASE(board) + CH_CR), reg_r(r, S2MM_CH_BASE(board) + CH_SR));
}

// Enable the S2MM channel's completion/error interrupt so the MCDMA drives its introut
// line, which hw_manager folds into its single ps_interrupt doorbell. Call after arming.
void dma_s2mm_irq_enable(struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok) return;
  volatile uint8_t *r = dma->reg;
  uint32_t cb = S2MM_CH_BASE(board);
  reg_w(r, cb + CH_CR, reg_r(r, cb + CH_CR) | CH_IRQ_IOC | CH_IRQ_ERR | CH_IRQ_THRESH1);
}

// Classify the S2MM channel after a doorbell: 1 = completed, -1 = error, 0 = neither yet.
// The channel status register is the source of truth (the introut line cannot tell
// completion from error); the descriptor completion bit is a secondary confirmation.
int dma_s2mm_status(struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok) return -1;
  volatile uint8_t *r = dma->reg;
  uint32_t sr = reg_r(r, S2MM_CH_BASE(board) + CH_SR);
  if ((sr & CH_IRQ_ERR) || (reg_r(r, S2MM_CH_ERR) & (1u << board))) return -1;
  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + S2MM_DESC_OFF);
  if ((sr & CH_IRQ_IOC) || (d->status & DESC_STAT_CMPLT)) return 1;
  return 0;
}

// Clear the S2MM channel's latched interrupt status (write-1-to-clear) so its introut
// line deasserts and the next completion can doorbell again.
void dma_s2mm_irq_ack(struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok) return;
  volatile uint8_t *r = dma->reg;
  reg_w(r, S2MM_CH_BASE(board) + CH_SR, CH_IRQ_IOC | CH_IRQ_DLY | CH_IRQ_ERR);
}

// Copy out the words the last S2MM capture wrote (the descriptor length), without polling.
// Use after dma_s2mm_status reports completion. Returns the word count copied.
int dma_s2mm_collect(struct dma_ctrl_t *dma, uint32_t *out, uint32_t max_words) {
  if (!dma || !dma->ok) return -1;
  struct mcdma_desc *d = (struct mcdma_desc *)(dma->desc + S2MM_DESC_OFF);
  uint32_t rx = (d->status & DESC_STAT_LEN_MASK) / 4u;
  if (rx > max_words) rx = max_words;
  memcpy(out, dma->data + S2MM_CAPTURE_OFF, (size_t)rx * 4u);
  return (int)rx;
}

// ------------------------------------------------- multi-word waveform run --

// --- Capacity math (shared by dma_wave_add and the waveform capacity precheck) ---------------
//
// With the backlog-at-threshold drain discipline a capture packs into a fixed shape: a thin
// start and a thin end (each up to `threshold` single-word packets, while the backlog climbs to
// the threshold and while it is greedily drained at the end) bracket a dense middle (one
// S2MM_PACKET_WORDS packet per descriptor). So the descriptor count is far below one-per-word.

// Descriptors a cap_words capture needs under the backlog-at-threshold packing (thin start + thin
// end bounded by the threshold, dense middle). Clamped at cap_words.
uint32_t dma_wave_cap_ndesc(uint32_t cap_words, uint32_t threshold) {
  if (cap_words == 0) return 0;
  uint64_t two_t = 2ull * (uint64_t)threshold;
  if ((uint64_t)cap_words <= two_t) return cap_words;    // small capture: could be all thin
  uint64_t dense = ((uint64_t)cap_words - two_t + (S2MM_PACKET_WORDS - 1u)) / S2MM_PACKET_WORDS;
  uint64_t n = two_t + dense;
  return (n >= cap_words) ? cap_words : (uint32_t)n;
}

// Inverse: the largest cap_words that fits in `cap_ndesc` descriptors at this threshold.
uint32_t dma_wave_max_cap_words(uint32_t cap_ndesc, uint32_t threshold) {
  uint64_t two_t = 2ull * (uint64_t)threshold;
  if ((uint64_t)cap_ndesc <= two_t) return cap_ndesc;    // all-thin regime: one word per descriptor
  uint64_t m = two_t + (uint64_t)S2MM_PACKET_WORDS * ((uint64_t)cap_ndesc - two_t);
  return (m > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)m;
}

// DAC payload and descriptor footprint (bytes) for one board's stream, matching the layout
// dma_wave_add builds: payload 64-byte aligned, one MM2S_PACKET_WORDS-word packet per descriptor.
uint64_t dma_wave_dac_footprint_data(uint32_t dac_n_words) {
  return align_up_u32(dac_n_words * 4u, DESC_ALIGN);
}
uint64_t dma_wave_dac_footprint_desc(uint32_t dac_n_words) {
  uint32_t n = (dac_n_words + MM2S_PACKET_WORDS - 1u) / MM2S_PACKET_WORDS;
  return (uint64_t)n * DESC_ALIGN;
}

// Capture descriptors that fit in an ADC budget: each costs one 64-byte descriptor slot and one
// S2MM_PACKET_WORDS-word (1024-byte) data buffer, so the tighter of the two bounds.
uint32_t dma_wave_adc_cap_ndesc_budget(uint64_t adc_data_bytes, uint64_t adc_desc_bytes) {
  uint64_t by_data = adc_data_bytes / (S2MM_PACKET_WORDS * 4u);
  uint64_t by_desc = adc_desc_bytes / DESC_ALIGN;
  uint64_t n = (by_data < by_desc) ? by_data : by_desc;
  return (n > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)n;
}

// Reset both directions and clear the region allocator / per-board run state. The MCDMA
// soft-reset (CR bit 2) is global, so resetting here -- before any channel is armed -- avoids
// a later reset wiping an already-armed channel.
int dma_wave_begin(struct dma_ctrl_t *dma, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  reset_direction(dma->reg, MM2S_CTRL);
  reset_direction(dma->reg, S2MM_CTRL);
  for (int b = 0; b < MAX_BOARDS; b++) dma->wave[b].armed = false;
  dma->wave_data_cursor = 0;
  dma->wave_desc_cursor = 0;
  dma->wave_single_board = -1;
  if (verbose) printf("DMA wave: reset both directions; region allocator cleared.\n");
  return 0;
}

// Halt the MCDMA for a coordinated shutdown: soft-reset both directions (CR bit 2 is global,
// so this stops every channel and drains the S2MM downstream path), then clear the software
// run state. Assert FIFO buf_reset only after this, so the engine is not mid-transfer when the
// FIFOs reset (the ex05 halt-reset lesson).
void dma_wave_halt(struct dma_ctrl_t *dma, bool verbose) {
  if (!dma || !dma->ok) return;
  reset_direction(dma->reg, MM2S_CTRL);
  reset_direction(dma->reg, S2MM_CTRL);
  for (int b = 0; b < MAX_BOARDS; b++) dma->wave[b].armed = false;
  dma->wave_data_cursor = 0;
  dma->wave_desc_cursor = 0;
  dma->wave_single_board = -1;
  if (verbose) printf("DMA wave: MCDMA halted (both directions soft-reset); run state cleared.\n");
}

// Carve a region slice for `board`, build its MM2S descriptor and S2MM capture ring, and
// enable (but do not start) both of its channels. dma_wave_start starts every armed board at
// once. Does not reset the directions (dma_wave_begin did).
int dma_wave_add(struct dma_ctrl_t *dma, int board,
                 const uint32_t *dac_words, uint32_t dac_n_words,
                 uint32_t cap_words, uint32_t pack_threshold, uint32_t cap_ndesc_cap, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  if (board < 0 || board >= MAX_BOARDS) { fprintf(stderr, "DMA wave: bad board %d.\n", board); return -1; }
  if (dac_n_words == 0) { fprintf(stderr, "DMA wave: empty DAC stream on board %d.\n", board); return -1; }
  if (dma->wave[board].armed) { fprintf(stderr, "DMA wave: board %d already armed this run.\n", board); return -1; }

  // cap_words == 0 means DAC-only (no ADC capture): the MM2S stream is prebuffered but no
  // S2MM ring is built or armed. The data/descriptor capture slices then have zero length.

  // Data region slice: DAC payload 64-byte aligned, ADC capture 64-byte aligned after it. Each
  // capture descriptor reserves a full S2MM_PACKET_WORDS-word buffer (the engine fills 1..that per
  // packet; compaction reads back the real length), so the capture reserves cap_ndesc buffers.
  uint32_t mm2s_off = align_up_u32(dma->wave_data_cursor, DESC_ALIGN);
  uint32_t mm2s_len = dac_n_words * 4u;
  uint32_t cap_off  = align_up_u32(mm2s_off + mm2s_len, DESC_ALIGN);
  // Reserve descriptors for the packed capture shape (thin start + thin end + dense middle under
  // the backlog-at-threshold drain), capped at this board's share of the ADC region. If the cap
  // bites (the capture is larger than fits), the ring is short and the run overflow-faults partway
  // -- the deliberate "continue anyway" outcome the caller warned about.
  uint32_t cap_ndesc = dma_wave_cap_ndesc(cap_words, pack_threshold);
  if (cap_ndesc > cap_ndesc_cap) cap_ndesc = cap_ndesc_cap;
  if (cap_words > 0 && cap_ndesc == 0) {
    fprintf(stderr, "DMA wave: board %d has no ADC descriptor budget (the DAC consumed the region).\n", board);
    return -1;
  }
  uint32_t cap_buf_bytes = S2MM_PACKET_WORDS * 4u;
  uint64_t data_end = (uint64_t)cap_off + (uint64_t)cap_ndesc * cap_buf_bytes;
  if (data_end > dma->data_size) {
    fprintf(stderr,
      "DMA wave: board %d needs data up to %" PRIu64 " B (DAC %u B + capture %u desc x %u B), but %s is %" PRIu64 " B.\n"
      "  Reduce the run length or enlarge the %s region.\n",
      board, data_end, mm2s_len, cap_ndesc, cap_buf_bytes, DATA_UDMABUF, dma->data_size, DATA_UDMABUF);
    return -1;
  }

  // Descriptor region slice: a chain of MM2S packet descriptors, then the S2MM ring after them.
  uint32_t mm2s_desc_off = align_up_u32(dma->wave_desc_cursor, DESC_ALIGN);
  uint32_t n_mm2s_desc = (dac_n_words + MM2S_PACKET_WORDS - 1u) / MM2S_PACKET_WORDS;
  uint32_t ring_desc_off = mm2s_desc_off + n_mm2s_desc * DESC_ALIGN;
  uint64_t desc_end = (uint64_t)ring_desc_off + (uint64_t)cap_ndesc * DESC_ALIGN;
  if (desc_end > dma->desc_size) {
    fprintf(stderr,
      "DMA wave: board %d needs descriptors up to %" PRIu64 " B (MM2S %u + capture %u), but %s is %" PRIu64 " B.\n"
      "  Reduce the run length or enlarge the %s region.\n",
      board, desc_end, n_mm2s_desc, cap_ndesc, DESC_UDMABUF, dma->desc_size, DESC_UDMABUF);
    return -1;
  }

  volatile uint8_t *r = dma->reg;

  // DAC payload into DDR.
  memcpy(dma->data + mm2s_off, dac_words, mm2s_len);

  // MM2S descriptor chain: split the payload into MM2S_PACKET_WORDS-word packets (each its own
  // SOF|EOF packet), linked head to tail. The MCDMA completes one board's packet then arbitrates
  // to the next board's channel, so the per-board DAC FIFOs fill in lockstep. The DAC FIFO is a
  // plain word stream, so a packet boundary splitting a multi-word command is harmless.
  uint64_t mm2s_desc_phys = dma->desc_phys + mm2s_desc_off;  // first descriptor (CURDESC)
  for (uint32_t k = 0, words_left = dac_n_words; k < n_mm2s_desc; k++) {
    struct mcdma_desc *md = (struct mcdma_desc *)(dma->desc + mm2s_desc_off + (uint64_t)k * DESC_ALIGN);
    uint32_t pkt_words = (words_left < MM2S_PACKET_WORDS) ? words_left : MM2S_PACKET_WORDS;
    uint64_t next_phys = dma->desc_phys + mm2s_desc_off + (uint64_t)((k + 1u) % n_mm2s_desc) * DESC_ALIGN;
    build_desc(md, next_phys, dma->data_phys + mm2s_off + (uint64_t)k * MM2S_PACKET_WORDS * 4u, pkt_words * 4u);
    words_left -= pkt_words;
  }

  // S2MM ring: one receive descriptor per packet, each with a full S2MM_PACKET_WORDS-word buffer.
  // The engine writes a packet's words into a descriptor's buffer and closes it on tlast (or
  // buffer-full), recording the real length; readback compacts the partial buffers. Buffers are
  // laid out in order; the wrap link is never followed (the engine stops at TAILDESC) but keeps
  // the ring well-formed. Skipped entirely for a DAC-only run. No data memset is needed -- the
  // per-descriptor Complete bit gates readback, so unwritten buffer tails are never read.
  if (cap_words > 0) {
    for (uint32_t k = 0; k < cap_ndesc; k++) {
      struct mcdma_desc *rd = (struct mcdma_desc *)(dma->desc + ring_desc_off + (uint64_t)k * DESC_ALIGN);
      uint64_t next_phys = dma->desc_phys + ring_desc_off + (uint64_t)((k + 1u) % cap_ndesc) * DESC_ALIGN;
      build_rx_desc(rd, next_phys, dma->data_phys + cap_off + (uint64_t)k * cap_buf_bytes, cap_buf_bytes);
    }
  }
  __sync_synchronize(); // payload + all descriptors resident before the engines read them

  // MM2S: enable the channel and point it at its descriptor. The transfer does not start until
  // dma_wave_start asserts the common Run/Stop once, after every board is enabled, so the MCDMA
  // arbiter engages all channels together and interleaves them (keeping the per-board DAC FIFOs
  // roughly aligned as they prebuffer behind the leading trigger-wait).
  arm_channel(r, MM2S_CH_BASE(board), MM2S_CHEN, board, mm2s_desc_phys);

  // S2MM capture ring (only when capturing): enable + point at the ring head; started together
  // with the rest in dma_wave_start.
  if (cap_words > 0) {
    uint64_t ring_first_phys = dma->desc_phys + ring_desc_off;
    arm_channel(r, S2MM_CH_BASE(board), S2MM_CHEN, board, ring_first_phys);
  }

  struct dma_wave_run *w = &dma->wave[board];
  w->armed         = true;
  w->board         = board;
  w->mm2s_off      = mm2s_off;
  w->mm2s_len      = mm2s_len;
  w->cap_off       = cap_off;
  w->cap_words     = cap_words;
  w->cap_ndesc     = cap_ndesc;
  w->cap_read      = 0;
  w->cap_desc_read = 0;
  w->cap_word_in_desc = 0;
  w->mm2s_desc_off = mm2s_desc_off;
  w->mm2s_tail_desc_off = mm2s_desc_off + (n_mm2s_desc - 1u) * DESC_ALIGN;
  w->ring_desc_off = ring_desc_off;

  dma->wave_data_cursor = (uint32_t)data_end;
  dma->wave_desc_cursor = (uint32_t)desc_end;

  if (verbose)
    printf("DMA wave board %d armed: DAC %u words (%u B) prebuffered, S2MM ring %u words.\n",
           board, dac_n_words, mm2s_len, cap_words);
  return 0;
}

// Start every armed channel at once: assert each direction's common Run/Stop a single time (so
// the MCDMA arbiter engages all enabled channels together and interleaves them), then kick each
// channel's tail descriptor. Call once after all dma_wave_add calls, before releasing the
// trigger. S2MM is started before MM2S so the capture channels run before any data arrives.
int dma_wave_start(struct dma_ctrl_t *dma, bool verbose) {
  if (!dma || !dma->ok) { fprintf(stderr, "DMA not available.\n"); return -1; }
  volatile uint8_t *r = dma->reg;
  bool any_mm2s = false, any_s2mm = false;
  for (int b = 0; b < MAX_BOARDS; b++) {
    if (!dma->wave[b].armed) continue;
    any_mm2s = true;
    if (dma->wave[b].cap_words > 0) any_s2mm = true;
  }
  if (!any_mm2s) { fprintf(stderr, "DMA wave: no boards armed to start.\n"); return -1; }

  if (any_s2mm) {
    run_direction(r, S2MM_CTRL);
    for (int b = 0; b < MAX_BOARDS; b++) {
      struct dma_wave_run *w = &dma->wave[b];
      if (!w->armed || w->cap_words == 0) continue;
      uint64_t ring_last = dma->desc_phys + w->ring_desc_off + (uint64_t)(w->cap_ndesc - 1u) * DESC_ALIGN;
      trigger_channel(r, S2MM_CH_BASE(b), ring_last);
    }
  }
  run_direction(r, MM2S_CTRL);
  for (int b = 0; b < MAX_BOARDS; b++) {
    struct dma_wave_run *w = &dma->wave[b];
    if (!w->armed) continue;
    trigger_channel(r, MM2S_CH_BASE(b), dma->desc_phys + w->mm2s_tail_desc_off);
  }
  if (verbose) printf("DMA wave: started all armed channels (common Run/Stop asserted once).\n");
  return 0;
}

// Print per-board MCDMA channel status for every armed board: CHEN membership and the
// per-channel status register, for both MM2S and (when capturing) S2MM. Pairs with the caller's
// per-board FIFO-fill print to show where a multi-board run is or is not delivering.
void dma_wave_print_channels(struct dma_ctrl_t *dma) {
  if (!dma || !dma->ok) return;
  volatile uint8_t *r = dma->reg;
  uint32_t mm2s_chen = reg_r(r, MM2S_CHEN);
  uint32_t s2mm_chen = reg_r(r, S2MM_CHEN);
  printf("  MCDMA MM2S CHEN=0x%08X, S2MM CHEN=0x%08X\n", mm2s_chen, s2mm_chen);
  for (int b = 0; b < MAX_BOARDS; b++) {
    struct dma_wave_run *w = &dma->wave[b];
    if (!w->armed) continue;
    uint32_t mm2s_sr = reg_r(r, MM2S_CH_BASE(b) + CH_SR);
    if (w->cap_words > 0) {
      uint32_t s2mm_sr = reg_r(r, S2MM_CH_BASE(b) + CH_SR);
      printf("  board %d: MM2S en=%u SR=0x%08X | S2MM en=%u SR=0x%08X\n",
             b, (mm2s_chen >> b) & 1u, mm2s_sr, (s2mm_chen >> b) & 1u, s2mm_sr);
    } else {
      printf("  board %d: MM2S en=%u SR=0x%08X\n", b, (mm2s_chen >> b) & 1u, mm2s_sr);
    }
  }
}

// Single-board wrapper: one begin + one add + start, remembering the board for the no-board
// accessors. Behaviour matches the former one-run-at-a-time dma_wave_arm.
int dma_wave_arm(struct dma_ctrl_t *dma, int board,
                 const uint32_t *dac_words, uint32_t dac_n_words,
                 uint32_t cap_words, bool verbose) {
  if (dma_wave_begin(dma, verbose) < 0) return -1;
  // Eager threshold + no descriptor cap = the thin-safe one-descriptor-per-word allocation the
  // single-board bench path has always used (it does not drive the packing threshold).
  int rc = dma_wave_add(dma, board, dac_words, dac_n_words, cap_words, UINT32_MAX, cap_words, verbose);
  if (rc != 0) return rc;
  dma->wave_single_board = board;
  return dma_wave_start(dma, verbose);
}

// Completed capture words available past the read cursor, for one board. Walks completed packet
// descriptors from the read cursor and sums each one's real (compacted) length -- the per-packet
// buffers are variable-fill, so this is not a simple contiguous count.
uint32_t dma_wave_avail_board(struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok || board < 0 || board >= MAX_BOARDS || !dma->wave[board].armed) return 0;
  struct dma_wave_run *w = &dma->wave[board];
  uint32_t words = 0;
  uint32_t skip = w->cap_word_in_desc;  // words already consumed from the first descriptor
  for (uint32_t k = w->cap_desc_read; k < w->cap_ndesc; k++) {
    struct mcdma_desc *rd = (struct mcdma_desc *)(dma->desc + w->ring_desc_off + (uint64_t)k * DESC_ALIGN);
    if (!(rd->status & DESC_STAT_CMPLT)) break;
    uint32_t len = (rd->status & DESC_STAT_LEN_MASK) / 4u;
    if (len > skip) words += len - skip;
    skip = 0;
  }
  return words;
}

// Drain up to max_words completed capture words into out, compacting the variable-fill per-packet
// buffers into a contiguous word stream. Splits a descriptor across calls if max_words runs out
// mid-packet, so any max_words works. Returns the word count copied, or -1 on a bad board.
int dma_wave_read_board(struct dma_ctrl_t *dma, int board, uint32_t *out, uint32_t max_words) {
  if (!dma || !dma->ok || board < 0 || board >= MAX_BOARDS || !dma->wave[board].armed) return -1;
  struct dma_wave_run *w = &dma->wave[board];
  uint32_t total = 0;
  while (total < max_words && w->cap_desc_read < w->cap_ndesc) {
    struct mcdma_desc *rd = (struct mcdma_desc *)(dma->desc + w->ring_desc_off + (uint64_t)w->cap_desc_read * DESC_ALIGN);
    if (!(rd->status & DESC_STAT_CMPLT)) break;
    uint32_t len = (rd->status & DESC_STAT_LEN_MASK) / 4u;
    uint32_t left = (len > w->cap_word_in_desc) ? (len - w->cap_word_in_desc) : 0u;
    uint32_t take = (left < max_words - total) ? left : (max_words - total);
    if (take) {
      memcpy(out + total,
             dma->data + w->cap_off + (uint64_t)w->cap_desc_read * (S2MM_PACKET_WORDS * 4u)
               + (uint64_t)w->cap_word_in_desc * 4u,
             (size_t)take * 4u);
      total += take;
      w->cap_word_in_desc += take;
      w->cap_read += take;
    }
    if (w->cap_word_in_desc >= len) {    // this descriptor fully drained -> advance to the next
      w->cap_desc_read++;
      w->cap_word_in_desc = 0;
    } else {
      break;                              // ran out of max_words mid-descriptor
    }
  }
  // Report the read delta so the rate-control backlog tracks what software has consumed.
  if (total) dma_rate_report_read(dma, board, total);
  return (int)total;
}

uint32_t dma_wave_read_total_board(const struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok || board < 0 || board >= MAX_BOARDS || !dma->wave[board].armed) return 0;
  return dma->wave[board].cap_read;
}

uint32_t dma_wave_expected_board(const struct dma_ctrl_t *dma, int board) {
  if (!dma || !dma->ok || board < 0 || board >= MAX_BOARDS || !dma->wave[board].armed) return 0;
  return dma->wave[board].cap_words;
}

bool dma_wave_board_armed(const struct dma_ctrl_t *dma, int board) {
  return dma && dma->ok && board >= 0 && board < MAX_BOARDS && dma->wave[board].armed;
}

void dma_wave_disarm_all(struct dma_ctrl_t *dma) {
  if (!dma) return;
  for (int b = 0; b < MAX_BOARDS; b++) dma->wave[b].armed = false;
  dma->wave_single_board = -1;
}

// --- Single-run wrappers over the per-board accessors (the remembered board) ---

uint32_t dma_wave_avail(struct dma_ctrl_t *dma) {
  if (!dma || dma->wave_single_board < 0) return 0;
  return dma_wave_avail_board(dma, dma->wave_single_board);
}

int dma_wave_read(struct dma_ctrl_t *dma, uint32_t *out, uint32_t max_words) {
  if (!dma || dma->wave_single_board < 0) return -1;
  return dma_wave_read_board(dma, dma->wave_single_board, out, max_words);
}

uint32_t dma_wave_read_total(const struct dma_ctrl_t *dma) {
  if (!dma || dma->wave_single_board < 0) return 0;
  return dma_wave_read_total_board(dma, dma->wave_single_board);
}

uint32_t dma_wave_expected(const struct dma_ctrl_t *dma) {
  if (!dma || dma->wave_single_board < 0) return 0;
  return dma_wave_expected_board(dma, dma->wave_single_board);
}

void dma_wave_disarm(struct dma_ctrl_t *dma) {
  dma_wave_disarm_all(dma);
}

bool dma_wave_is_armed(const struct dma_ctrl_t *dma) {
  return dma && dma->wave_single_board >= 0 && dma_wave_board_armed(dma, dma->wave_single_board);
}


