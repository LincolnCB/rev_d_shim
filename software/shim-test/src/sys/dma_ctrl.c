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

#define DESC_ALIGN      64u             // MCDMA SG descriptor size/alignment

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

// ------------------------------------------------------------- descriptors --

// One single-buffer, self-looping descriptor covering [buf_phys, buf_phys+len), marked
// start- and end-of-packet. MM2S drives tdest from the channel, so no tdest field here.
static void build_desc(struct mcdma_desc *d, uint64_t self_phys, uint64_t buf_phys, uint32_t len) {
  memset(d, 0, sizeof(*d));
  d->nxtdesc         = (uint32_t)self_phys;
  d->nxtdesc_msb     = (uint32_t)(self_phys >> 32);
  d->buffer_addr     = (uint32_t)buf_phys;
  d->buffer_addr_msb = (uint32_t)(buf_phys >> 32);
  d->control         = DESC_CTRL_SOF | DESC_CTRL_EOF | (len & DESC_CTRL_LEN_MASK);
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

// ------------------------------------------------------------- public API --

struct dma_ctrl_t create_dma_ctrl(bool verbose) {
  struct dma_ctrl_t dma;
  memset(&dma, 0, sizeof(dma));
  dma.reg_fd = dma.desc_fd = dma.data_fd = -1;
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
  dma.ok = true;
  return dma;
}

void destroy_dma_ctrl(struct dma_ctrl_t *dma) {
  if (!dma) return;
  if (dma->reg)  munmap((void *)dma->reg, MCDMA_WIN_BYTES);
  if (dma->desc) munmap(dma->desc, dma->desc_size);
  if (dma->data) munmap(dma->data, dma->data_size);
  if (dma->reg_fd  >= 0) close(dma->reg_fd);
  if (dma->desc_fd >= 0) close(dma->desc_fd);
  if (dma->data_fd >= 0) close(dma->data_fd);
  memset(dma, 0, sizeof(*dma));
  dma->reg_fd = dma->desc_fd = dma->data_fd = -1;
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
