# DDR-Backed FIFO Extension for 64-Channel Current Driver

Status: design exploration. Nothing implemented yet.

## 1. System overview

A 64-channel dynamic current driver with per-channel readback, built on a krtkl
snickerdoodle black (Xilinx XC7Z020, dual Cortex-A9 @ 866 MHz, 1 GB LPDDR2,
140 BRAM36 blocks, 53,200 LUTs).

Eight daughter boards, each carrying:

- one 8-channel, 16-bit DAC
- one 8-channel, 16-bit ADC

Each board is driven by a core in the PL that speaks SPI to those chips and is fed
by (DAC) or drains into (ADC) a 32-bit-wide clock-domain-crossing FIFO. One DAC FIFO
and one ADC FIFO per board, 16 streams total. The PS currently reaches these FIFOs
directly over the GP AXI ports.

Sample timing is defined per-update as a clock-cycle delay from the previous update.
The PL clock is set per sequence, between 1 and 50 MHz. The DAC and ADC of a single
board are independent of one another, and boards are independent of each other. The
SPI timing floor caps any single board at roughly 125 kSa/s, but any board may run
arbitrarily slower, pause for extended periods, or sit idle while others run.

Board-to-board start alignment is handled by a separate triggering system and is
explicitly out of scope here. Boards can wait on a trigger rather than a clock count,
and the trigger can be held until the PS reports prebuffering complete.

## 2. Data rates

Per board, at the 125 kSa/s ceiling:

| Direction | Words per update | Bytes per update | Rate per board |
|---|---|---|---|
| DAC (MM2S, PS to PL) | 5 x 32-bit | 20 | 2.5 MB/s |
| ADC (S2MM, PL to PS) | 4 x 32-bit | 16 | 2.0 MB/s |

Across 8 boards at simultaneous maximum: 20 MB/s out, 16 MB/s in, **36 MB/s
aggregate (288 Mbit/s)**. This is the absolute worst case and is not the normal
operating point; real sequences frequently run at hundreds of samples per second,
or in bursts separated by 10 to 100 ms of idle.

Derived figures worth keeping:

- A 4 KiB DAC FIFO holds 204.8 updates, which drains in **1.638 ms** at full rate.
  This is the origin of the "1.6 ms" deadline figure. It scales linearly:
  `drain_time = fifo_depth_bytes / 20 bytes / sample_rate`.
- A 100 ms sequence at full rate on all boards is **2.0 MB of DAC data and 1.6 MB
  of ADC data**, 3.6 MB total.
- 36 MB/s is about 4.5% of one AXI_HP port at 64-bit / 100 MHz (800 MB/s), and
  about 1% of LPDDR2 theoretical bandwidth.
- Available BRAM is 140 x 36 Kb, roughly 630 KB total.

## 3. Hard constraints

1. **DAC FIFOs must never underrun and ADC FIFOs must never overflow.** Either
   condition triggers an immediate sequence halt and a hardware shutdown, reported
   to the PS by error code. Recovery is safe but requires human intervention. Treat
   this as a must-not-happen event, not a degrade-and-continue event.
2. **No continuation after a fault.** The experiment is invalid; the run restarts
   from the beginning.
3. **Per-board independence.** Boards must remain individually addressable,
   startable, pausable, and stoppable. Interleaving the eight boards into a single
   shared stream is rejected (see section 7).
4. **Rate independence.** Any board may run at any rate from ~125 kSa/s down to
   effectively zero, with arbitrary pauses, while other boards run at any other rate.
   The design must not assume a common or fixed cadence.
5. **The existing per-FIFO API shape should survive.** Today userspace writes chunks
   of 5 x 32-bit words (DAC) and reads chunks of 4 x 32-bit words (ADC) per FIFO
   through a misc-device driver with an mmap'd region and no syscalls in the inner
   loop. The upgrade should be as close to in-place as possible.

## 4. Soft preferences

- Push the rate ceiling until it is a hardware limit (SPI timing) rather than a data
  movement or CPU limit.
- Free BRAM if convenient, but it is not currently scarce.
- Keep LUT growth modest. Current utilization is roughly 60% or less; a few
  additional cores is acceptable, a wholesale redesign is not.
- Avoid requiring PREEMPT_RT if possible. It is available and acceptable, but it is
  a learning and maintenance cost.

## 5. What is broken today

For longer or faster sequences, the waveform does not fit in the on-chip BRAM FIFOs,
and the PS cannot refill or drain them in time. The failure mode is the hard shutdown
described above. The current bottleneck is a combination of buffer depth and the
latency and jitter of a Linux userspace thread servicing sixteen FIFOs by hand.

## 6. Core insight driving the design

Because sequences of interest are on the order of 100 ms and total under 4 MB, and
because the trigger can be withheld until prebuffering completes, **the entire
sequence can be resident in LPDDR2 before the run starts.** Software then plays no
part during the run.

This converts the problem from "meet a 1.6 ms service deadline under Linux
scheduling jitter" into "sustain 36 MB/s of descriptor-driven DMA against 800 MB/s of
available port bandwidth." The margin goes from roughly 1x to roughly 20x on
bandwidth, and the deadline-sensitive software path disappears entirely.

Everything below follows from this. If the prebuffered assumption ever breaks (see
section 9), the analysis must be redone.

## 7. Candidate architectures

### Option A: AXI MCDMA, one instance, 16 channels (leading candidate)

One AXI MCDMA instance configured with 8 MM2S and 8 S2MM channels, one channel pair
per board. Channels have independent descriptor queues, independent
start/stop/status, and per-channel interrupts. An AXI4-Stream switch or simple
TDEST demux fans MM2S out to the eight DAC FIFOs; TDEST tagging on the ADC side
routes into S2MM.

- Maps 1:1 onto the eight-board mental model, preserving the current API shape.
- One AXI master pair into a single HP port; far cheaper in LUTs than eight separate
  DMA engines plus a 16-way interconnect.
- The Linux driver's lack of cyclic mode does not matter for prebuffered playback.
- Descriptor count is tiny: with the buffer length register widened to 23 bits
  (8 MB), a 250 KB per-board DAC buffer from contiguous CMA is a single descriptor.
  Mid-run descriptor fetch traffic is effectively zero.
- Interrupts: one per channel at end of sequence, sixteen total per run.
- Main risk: the MCDMA Linux driver is less battle-tested than the plain AXI DMA
  driver, and it implements only `device_prep_slave_sg`. Prototype this early.

### Option B: Eight AXI DMA instances, one per board

Each instance provides MM2S and S2MM, giving natural per-board isolation with the
most mature Linux driver in the family, plus cyclic mode if streaming later matters.

- Likely does not fit. Eight SG-enabled instances at roughly 2 to 3K LUTs each is
  16 to 24K LUTs, plus a 16-master SmartConnect. Against roughly 21K free LUTs at
  60% utilization, this is over budget.
- Worth reconsidering if the board count per Zynq drops, or if LUT usage elsewhere
  can be reduced.

### Option C: PS DMA controller (PL330) via the PL peripheral request interface

Uses no PL LUTs at all. Bandwidth through the central interconnect is far above
36 MB/s.

- Blocked on channel count: the PS DMAC has 8 channels, and the number of PL
  peripheral request interfaces is limited (believed to be 4 — **verify against
  UG585 before pursuing**). Sixteen concurrent PL-driven streams almost certainly
  does not fit.
- Listed for completeness; rank below A and B unless the verification surprises us.

### Option D: Custom datamover per board (AXI DataMover or HLS)

Maximum control, no dependence on Xilinx DMA drivers. Rejected as a first move: it
trades a known driver risk for an unknown RTL and software risk, and the problem does
not need custom behavior.

### Explicitly rejected: interleaving the eight boards into one stream

An earlier line of thinking proposed interleaving all channels into a single
round-robin DMA stream with a PL-side counter demux, to bound inter-channel skew by
construction. **This is rejected.** It was based on a misreading of the constraint as
a cross-board synchronization requirement. There is no such requirement — alignment
is handled by the trigger system, and boards deliberately run at unrelated rates and
pause independently. Interleaving would destroy per-board independence, which is a
hard constraint, in exchange for solving a problem that does not exist.

## 8. PL and PS changes required regardless of option

**PL side:**

- TDEST tagging on the eight ADC streams so S2MM routes to the correct channel;
  TDEST-based demux on the DAC side.
- A `tlast` policy on the ADC streams. Per-sample `tlast` would create 16-byte
  packets and enormous descriptor overhead. Preferred: assert `tlast` at
  end-of-sequence only, so the final partial buffer flushes and the completed-byte
  count is readable. **Open question: does the ADC core emit `tlast` today?**
- Stream width stays 32 bits to match the FIFOs; memory-map width is 64 bits to
  match the HP port. Do not use 32-bit HP mode.
- The CDC FIFOs stay where they are. They become elastic buffers absorbing DMA
  service latency (microseconds) rather than whole waveforms, which means their
  required depth drops by roughly three orders of magnitude. BRAM can be reclaimed
  here if needed elsewhere.
- Keep the existing error and halt path. Consider adding a FIFO-level watchdog that
  flags an approaching underrun before it becomes a shutdown, for diagnostics.

**PS side:**

- Widen the DMA buffer length register to 23 bits. The 14-bit default caps a
  descriptor at 16 KB, which turns a one-descriptor transfer into a long chain.
- Note the Zynq TrustZone default: PS peripherals are secure by default, and
  accesses with `AxPROT[1]=1` return DECERR. Check this if transfers fail
  immediately with no other explanation.

### 8.1 Buffer allocation: use `u-dma-buf` (preferred)

DMA needs physically contiguous memory whose physical address is known to whatever
programs the descriptors. The preferred mechanism is **`u-dma-buf`**
(`ikwzm/udmabuf`), an actively maintained out-of-tree module that does exactly this
job and nothing else.

Why it fits this project specifically:

- Exposes each region as `/dev/udmabufN` with the physical address readable from
  sysfs, which is precisely what descriptor construction needs.
- Supports a **cached** userspace mapping with explicit sync, via the
  `sync_for_cpu` / `sync_for_device` sysfs controls (plus `sync_offset`,
  `sync_size`, `sync_direction`, `sync_mode`). Waveform generation in userspace stays
  fast, and the prebuffered model means one `sync_for_device` per buffer per run — the
  coherency handling collapses to a single call at a known point rather than anything
  in a hot path.
- Permissions: ex05 reaches a non-root `/dev` node via `misc.mode` and deliberately
  avoids udev (this rootfs cannot install udev rules). `u-dma-buf`'s `/dev/udmabufN`
  nodes come up root-owned, so preserving non-root operation for them is an open item
  -- `chmod` from a boot script, or keep root only for the allocation step while the
  register `mmap` stays non-root through the misc driver.
- It is a superset of the allocator we would otherwise write by hand into the existing
  misc driver, and it is somebody else's maintenance burden.

Configuration notes:

- Back it with a **`reserved-memory` node** rather than the general CMA pool. A fixed
  region cannot fail to allocate due to fragmentation, which matters for an instrument
  that must not fail at trigger time. Size it for the longest supported sequence plus
  the descriptor rings; 64 MB is generous for the current 100 ms case.
- The SG descriptor rings also need known physical addresses — give them their own
  small `u-dma-buf` region.
- Follow the repo's out-of-tree kernel-module flow: drop the module under the
  project's `kernel_modules/` (symlinking the vendored copy). The build script
  auto-discovers every subdirectory there -- there is no cfg list to maintain --
  generates the recipe automatically (no hand-written `meta-user/recipes-modules`),
  and appends `KERNEL_MODULE_AUTOLOAD`, so the module loads at boot. Declare the DMA
  regions via device-tree nodes or module parameters, since a bare autoload loads the
  module with none. The only ongoing cost is rebuilding against new kernels.

Alternatives, kept on the table but not preferred: `dma_alloc_coherent` inside a
custom kernel driver (more code we own, no capability advantage), or a bare CMA
allocation via the existing misc driver (same, plus fragmentation risk).

### 8.2 How the MCDMA gets driven (decided: B, proven in ex07)

This is coupled to the choice above and should be decided alongside it.

**A. Linux dmaengine.** Device-tree the MCDMA, use `dmaengine_prep_slave_sg()` from a
consumer driver. Idiomatic and descriptor management is handled. But dmaengine expects
to manage buffers through the kernel DMA API, so it pairs awkwardly with `u-dma-buf`;
going this route probably means `dma_alloc_coherent` in the consumer driver instead.
Risk: the MCDMA path in `xilinx_dma.c` is comparatively lightly exercised.

**B. Direct register control from userspace.** Map the MCDMA control window through
the existing misc driver, allocate with `u-dma-buf`, build descriptor rings and program
channel pointers directly. More code, and we own the SG bookkeeping — but it sidesteps
the driver-maturity question entirely, and it preserves the current architecture
(mmap once, no syscalls in the loop).

**B pairs naturally with `u-dma-buf` and with how this system already works.**
**Proven in ex07:** a userspace program maps the MCDMA control window through a
`pl-reg`-bound misc device (`/dev/mcdma`, non-root), allocates with `u-dma-buf`, builds
SG descriptor rings, and round-trips 2+2 channels byte-for-byte. The register map and
descriptor layout match mainline `xilinx_dma.c` (notably: MCDMA BD `control` at 0x14,
Run/Stop in both the per-channel and common control registers). Evaluate A as a
possible simplification afterward rather than betting on it.

## 9. Deferred: streaming and loopback

Neither is a current requirement. Recorded so the design does not accidentally
foreclose them.

**Reduced-rate streaming.** Rather than cyclic mode, append descriptors ahead of the
queue tail while the run proceeds. The deadline is not the FIFO drain time but the
time to drain everything already queued: 8 MB queued on a DAC channel at 2.5 MB/s is
3.2 seconds of runway, which is enormous compared to any plausible Linux scheduling
jitter. This works with MCDMA as-is.

The real limit is source bandwidth, not DMA. Sustaining 36 MB/s from the SD card or
over the on-board Wi-Fi is not realistic; Gigabit Ethernet on a baseboard would be.
At reduced rates — say 10 kSa/s per board, about 2.9 MB/s aggregate — almost any
source works.

**DAC-side loopback latency.** Latency is dominated by how much data is queued ahead
of the new data, not by the DMA itself: roughly `queued_bytes / consumption_rate`.
Hitting a 100 ms target at full rate means keeping no more than about 250 KB queued
per DAC channel, which is comfortable. Hitting 1 ms would mean about 2.5 KB queued —
below current FIFO depth, so latency would then be set by the PL FIFO itself. That
would require shrinking the FIFOs and adding real-time scheduling, and it directly
trades away underrun margin. The tradeoff is monotonic: less queued data means lower
latency and less safety.

**ADC-side loopback** is not anticipated. ADC data is read after the run.

## 10. Open questions

1. Exact current LUT, BRAM, and FF utilization figures. Option B lives or dies here.
2. Does the ADC core emit `tlast` today, and if not, what is the cost of adding it?
3. Is there a per-board "sequence complete" signal available to the PL?
4. What is the longest sequence that must be supported? This sizes the CMA reservation.
5. Are SG descriptor chains built in the kernel driver or handed down from userspace?
   (See 8.2 — leaning userspace, to be confirmed by ex07.)
   Sizing input needed: longest supported sequence, to fix the `reserved-memory` size.
6. Verify the PS DMAC PL peripheral request interface count in UG585 (section 3 of
   the DMA Controller chapter) to close out Option C.
7. What PetaLinux and Vivado versions are in use? This determines which MCDMA driver
   revision is available and whether the snickerdoodle board files need adaptation.
   (Partly answered in section 11: the repo carries 2024.1 and 2024.2 configs.)

## 11. Repository and build-environment context

This design does not live in a vacuum — it is being prototyped inside the
**`zynq_toolbox`** build framework. An agent picking this up should know the
following, because it constrains how any proposal actually gets implemented.

**Where things live.**

- The de-risking testbed is `projects/ex07_dma/`; the parent system is
  `projects/rev_d_shim/`. Start from `projects/ex07_dma/README.md` (the concrete ex07
  plan) and the repo-root `AGENTS.md` (build conventions and the standing rules).
- Target hardware is the **snickerdoodle black** (`snickerdoodle_black` v1.0). The
  repo carries **PetaLinux/Vivado 2024.1 and 2024.2** configs under
  `cfg/snickerdoodle_black/1.0/petalinux/<ver>/`.

**Build pipeline.** `cores → xpr → xsa → petalinux → sd`, driven by `make`. Builds are
long and require large external tool installs. **Do not run any `make`/build target
unless explicitly asked** — this is a hard rule in this repo.

**Block design.** `block_design.tcl` is Vivado Tcl using repo helpers:
`cell <vlnv> <name> {props} {conns}`, `init_ps`, `module <tcl> <inst> {conns}`,
`wire <pin> <pin>`, and `addr <offset> <range> <target_intf> <addr_space_intf>` (use
`addr`, not `auto_connect_axi`). PL interrupts are concatenated with `xlconcat` into
`IRQ_F2P` (see `projects/ex04_interrupts/`). The current ex07 `block_design.tcl` is
still the **single-channel `axi_dma` loopback scaffold**; converting it to a
parameterized `axi_mcdma` with TDEST routing is the first hardware task.

**Kernel modules.** Each module is a directory under
`projects/<prj>/kernel_modules/<mod>` (usually a symlink into
`examples/kernel_modules/`). `scripts/petalinux/kernel_modules.sh` auto-discovers
every subdirectory there -- there is no cfg list -- builds them out-of-tree, generates
the recipe (no hand-written `meta-user/recipes-modules`), and appends
`KERNEL_MODULE_AUTOLOAD`, so they load automatically at boot. A bare autoload loads
`u-dma-buf` with **no** regions -- declare regions via device-tree nodes or module
parameters. `u-dma-buf` is already vendored under `examples/kernel_modules/u-dma-buf`
and symlinked into ex07.

**Non-root userspace access (established pattern).** PetaLinux's device-tree generator
emits one node per addressed PL IP automatically; a small "misc device" driver
(`pl-reg`, in `projects/ex05_device_driver/`) binds by the auto-generated `compatible`,
names its `/dev` entry from the Vivado instance label in `/__symbols__`, and sets
`misc.mode = 0666` to get non-root access **without udev** (this rootfs cannot install
udev rules). No hand-written `.dtsi` is needed for register windows. Reuse this pattern
for the MCDMA control window.

**Software.** Each program is `projects/<prj>/software/<name>/<name>.c`, cross-compiled
into the rootfs automatically. ex07 starts from `software/xilinx-dma-test/`.

**PetaLinux config** is stored as **patches** (`config.patch`, `rootfs_config.patch`)
against the tool defaults, not as full config files.
