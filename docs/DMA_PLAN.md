# DMA Port Plan: DDR-Backed Buffering for the Rev D Shim

This is the plan for porting the DDR-backed DMA buffering into `rev_d_shim`. The architecture is already decided and proven on hardware in the `ex05_dma` testbed, so this is a port rather than an open design effort. The document is self-contained: it covers the background and constraints, what changes in this project, in what order, and the decisions that gate each step.

## Background: the problem and the core insight

The instrument is a 64-channel dynamic current driver with per-channel readback on a krtkl snickerdoodle black (Xilinx XC7Z020: dual Cortex-A9 at 866 MHz, 1 GB LPDDR2, 140 BRAM36 tiles, 53,200 LUTs). Eight daughter boards each carry an 8-channel 16-bit DAC and an 8-channel 16-bit ADC; a per-board PL core speaks SPI to those chips, fed by (DAC) or draining into (ADC) a 32-bit clock-domain-crossing FIFO. Sample timing is a per-update clock-cycle delay, and the SPI clock is set per sequence between 1 and 50 MHz, capping any one board near 125 kSa/s -- though boards run at unrelated rates and pause independently. Board-to-board start alignment is handled by a separate trigger system and is out of scope, but the trigger can be withheld until the PS reports prebuffering complete, which is the hook the whole design hangs on.

At the 125 kSa/s ceiling a board moves 5 x 32-bit words (20 bytes) per DAC update and 4 x 32-bit words (16 bytes) per ADC sample -- 2.5 MB/s out and 2.0 MB/s in, so 36 MB/s aggregate across eight boards at simultaneous maximum. That is the absolute worst case; real sequences usually run hundreds of samples per second, in bursts separated by idle.

What breaks today: for longer or faster sequences the waveform does not fit in the on-chip BRAM FIFOs, and a Linux userspace thread cannot refill or drain sixteen FIFOs by hand within the timing floor. A DAC FIFO underrun or ADC FIFO overflow trips an immediate hardware shutdown (see Constraints), so the failure is hard, not graceful.

The insight that resolves it: sequences of interest are around 100 ms and under 4 MB total, and the trigger can be held until prebuffering completes, so the entire sequence can be resident in LPDDR2 before the run starts and software plays no part during the run. That turns "meet a ~1.6 ms FIFO-drain deadline under Linux scheduling jitter" into "sustain 36 MB/s of descriptor-driven DMA against ~800 MB/s of HP-port bandwidth" -- a margin of roughly 20x, with the deadline-sensitive software path gone entirely. This prebuffered-sequence assumption is load-bearing; if it ever breaks (see Deferred), the analysis must be redone. The supported tool versions are PetaLinux and Vivado 2024.1 and 2024.2.

## Constraints

Hard constraints, which any implementation must respect:

- DAC FIFOs must never underrun and ADC FIFOs must never overflow. Either trips an immediate sequence halt and hardware shutdown, reported to the PS by error code and recoverable only with human intervention. Treat it as must-not-happen, not degrade-and-continue.
- No continuation after a fault: the run is invalid and restarts from the beginning.
- Per-board independence is non-negotiable -- boards are individually addressable, startable, pausable, and stoppable. Interleaving the eight boards into one shared round-robin stream is rejected: alignment is the trigger system's job and boards deliberately run at unrelated rates, so a shared stream would destroy independence to solve a synchronization problem that does not exist.
- Rate independence: any board may run from ~125 kSa/s down to effectively zero, with arbitrary pauses, while others run at any other rate. No common cadence may be assumed.
- The existing per-FIFO userspace API shape should survive -- mmap once, no syscalls in the inner loop -- so the upgrade stays as close to in-place as possible.

Soft preferences: push the rate ceiling until it is an SPI-timing limit rather than a data-movement or CPU limit; keep LUT growth modest, since LUT is the binding constraint (see The decided design); free BRAM where convenient, as it is not currently scarce; and avoid requiring PREEMPT_RT if possible, since it is a learning and maintenance cost even though it is available.

## The decided design

A single AXI MCDMA with 8 MM2S and 8 S2MM channels (one channel pair per board), driven by direct userspace register control over `u-dma-buf` buffers, with the entire sequence prebuffered into LPDDR2 before the trigger fires. This was built up and confirmed byte-exact on hardware in `projects/ex05_dma/` -- including `pl-irq`-delivered completion interrupts and a halt/clear/reinit reset sequence -- so what follows is a port, not an open design question. Start from `projects/ex05_dma/README.md` for the concrete reference implementation and the repo-root `AGENTS.md` for build conventions.

A single MCDMA maps 1:1 onto the eight-board model, preserves the per-FIFO API shape, and puts one AXI master pair into a single HP port -- measured in ex05 at about 13.9k net-new LUT for the full 8+8 engine (mcdma 8.7k, HP0 SmartConnect 4.4k, GP0 control 0.5k, demux/mux 0.4k). The alternatives were weighed and rejected: eight separate AXI DMA instances would be 16-24k LUT plus a 16-master SmartConnect; the PS DMAC (PL330) falls far short on channel count (roughly four PL peripheral-request interfaces against sixteen streams); and a custom per-board datamover trades a known driver risk for unknown RTL and software risk. The design is tight at eight boards -- the SPI logic alone extrapolates to ~38-40k LUT, landing the whole design near 52-54k (~97-102% of 53,200), so LUT, not BRAM, is the binding constraint. Reclaim comes from dropping PS-side FIFO bridges and a leaner HP0 interconnect once DMA replaces direct FIFO access, while BRAM frees up as the CDC FIFOs shrink to elastic buffers. Driving the engine from userspace rather than the lightly-exercised in-kernel MCDMA `device_prep_slave_sg` path retires the driver-maturity risk; the register map and descriptor layout match mainline `xilinx_dma.c` (notably the BD `control` word at 0x14 and Run/Stop in both the per-channel and common control registers), proven byte-exact in ex05.

## What the DMA drives, and what it does not

Only the two high-throughput streams per board move onto the DMA. Everything else stays on the existing memory-mapped path.

- `dac_cmd_fifo` is fed by an MM2S channel (PS to PL). This is the DAC command and setpoint stream -- five 32-bit words per update -- and it is the dominant DAC-side data sink.
- `adc_data_fifo` is drained by an S2MM channel (PL to PS). This is the ADC sample stream and the dominant data source.

Eight boards give 8 MM2S + 8 S2MM, which is exactly one MCDMA. The other per-board FIFOs are low-throughput and their current shallow buffers are adequate, so they keep the existing `axi_fifo_bridge` mmap interface unchanged:

| FIFO | Depth (today) | Direction | Path after the port |
|---|---|---|---|
| `dac_cmd_fifo` | 2^13 | PS -> DAC core | **MM2S DMA** |
| `adc_data_fifo` | 2^13 | ADC core -> PS | **S2MM DMA** |
| `dac_data_fifo` | 2^12 | DAC debug readback -> PS | mmap (unchanged; rename `dac_debug`) |
| `adc_cmd_fifo` | 2^10 | PS -> ADC core | mmap (unchanged) |
| `trig_cmd_fifo` / `trig_data_fifo` | 2^10 | trigger core | mmap (unchanged) |

## The starting point in this project

The current datapath uses four `base:user:fifo_async` CDC FIFOs per board (32-bit, write in the 100 MHz AXI domain, read in the variable SPI domain), instantiated in `modules/axi_spi_interface.tcl`. The PS reaches every FIFO through `base:user:axi_fifo_bridge` (AXI4-Lite, mmap) on `M_AXI_GP1`, mapped at `0x8000_0000 + board*0x1_0000` for the DAC bridge and `+0x1000` for the ADC bridge. `hw_manager` (`cores/shim/hw_manager/`) owns the fault, halt, and shutdown path and raises `ps_interrupt` into `IRQ_F2P`. `axi_sys_ctrl` already drives per-board `cmd_buf_reset` / `data_buf_reset` masks, which are the exact hook the halt/clear/reinit sequence needs.

Two facts shape the port. First, the ADC path today is a plain FIFO read with an in-band end-of-command marker (`ADC_DBG_CMD_DONE`, top nibble `0x6`); there is no `tlast` or `tdest` sideband, so stream framing is new PL work. Second, the rev_d FIFOs are plain `fifo_async` (write/read data plus enable), not the native-AXIS FIFOs the ex05 testbed used -- but the repo already ships `base:user:axis_fifo_bridge`, which bridges an AXI4-Stream interface to exactly this `fifo_async` interface. That is the adapter between the MCDMA stream side and the existing FIFOs; no new adapter core is needed.

## Coexistence of the mmap and DMA paths

The `dac_cmd_fifo` write port and the `adc_data_fifo` read port each have a single owner today (the `axi_fifo_bridge`). Adding a DMA owner (an `axis_fifo_bridge` on the same FIFO port) puts two producers, or two consumers, on one port -- they must never both be live, or the FIFO contents are corrupted.

Control is a `datapath_mode` register in `axi_sys_ctrl` at the next free word offset (11), laid out as an 8-bit per-board mask that mirrors the block's existing per-board masks (`boot_test_skip`, `debug`, the reset masks). One bit per board covers both of that board's high-rate lanes together, since a board runs a coordinated play-and-capture and is either DMA or mmap as a unit. `datapath_mode[i]` is 0 for mmap and 1 for DMA, and it resets to 0, so power-on reproduces today's behavior exactly and DMA is opt-in per board. Living in `axi_sys_ctrl`, it inherits that block's config lock (`unlock` / `lock_viol`): the mode is set during setup and is immutable while the system is locked and running, so a board's path cannot change mid-run.

Per-board rather than a single global bit costs nothing extra -- the selection mux is per-FIFO either way -- and it buys board-at-a-time bring-up: during Stage 2 you can put one board on the DMA path while the rest stay on the proven mmap path, validate it end to end, then widen.

The mode bit drives a per-board 2:1 selection on the shared FIFO ports and gates the non-owner's handshake, not just the data, so the idle owner physically cannot touch the FIFO: in DMA mode the `axi_fifo_bridge` write path is held off, and in mmap mode the DMA-side bridge/packetizer has `tready`/`rd_en` deasserted. That makes the switch a hard lock, not a convention.

A wrong-mode access -- an mmap poke at a board that is in DMA mode, or vice versa -- must not fail silently and must not crash the program. The gate therefore accepts and discards the access with a normal `OKAY` response (no `SLVERR`, so no CPU external abort and no `SIGBUS`) and raises a `mode_viol` flag into `hw_manager`, like the other faults. That routes the failure through the existing coordinated shutdown -- status word plus `ps_interrupt`, caught by the software's interrupt monitor -- so the operation halts gracefully rather than crashing on the faulting instruction. The `axi_fifo_bridge`'s current `SLVERR`-on-full/empty behavior is scratch to be reworked, not a spec; the `mode_viol` gate is a deliberate accept-and-discard-plus-flag.

A build-time `use_dma` knob was considered as the zero-logic alternative, but it gives no runtime fallback and leaves two build variants; the runtime per-board register keeps the mmap path live as a fallback in a single build.

## ADC stream framing and packetization

For S2MM through the single shared, packet-atomic mux, each ADC stream needs `tdest` and `tlast`, neither of which exists today. `tdest` is trivial: tie it to the board index, so the mux and MCDMA route board *i*'s samples back to channel *i*'s buffer.

`tlast` is the real design, and it is resolved as **4-word chunk-atomic, adaptive packetization**. The atomic unit of ADC data is the 4-word chunk -- one read op captures all eight channels as eight 16-bit SPI words, packed into four 32-bit FIFO words -- so every packet boundary must land on a 4-word boundary; a sample-set straddling a packet would be an internal corruption, and that must not pass silently. The number of chunks per run is highly variable, so packet size is not fixed: a small packetizer core drains the chunks currently available and closes the packet (asserts `tlast` on the last word of a chunk) when the FIFO is about to stall or the packet reaches a cap.

Why adaptive rather than a fixed size: the packet-atomic mux holds a granted board until it sees `tlast`, so a fixed chop strands the mux mid-packet whenever the granted board's FIFO momentarily runs dry -- head-of-line blocking that stalls every other board and risks the ADC-overflow shutdown. Ending the packet exactly when the board's available data runs out releases the mux the instant that board has nothing more ready. It also self-flushes the tail of a run: when the ADC stops and the FIFO drains, the stall condition closes the final packet on its own, so `tlast` needs no separate end-of-sequence signal.

The signal this rests on already exists: `fifo_async` exposes `fifo_count_rd_clk`, the read-side fill count, in the same 100 MHz AXI domain the mux runs in, so available chunks are `fifo_count_rd_clk >> 2`. The count derives from a Gray-synchronized pointer, so it can lag but never over-report -- the safe direction (worst case, a packet closes a chunk early). The packetizer counts words to track chunk boundaries, asserts `tlast` on the fourth word of a chunk when no further whole chunk is available or the packet has reached `MAX` chunks, and drives a full AXIS master bundle (`tdata`, `tvalid`, `tready`, `tlast`, `tdest`) to the mux. It replaces `axis_fifo_bridge` on the ADC read side, since the framing decision needs the FIFO count and read-enable together and `axis_fifo_bridge` is `tdata`-only.

`MAX` has one hard ceiling: it must stay at or under the mux's `ARB_ON_MAX_XFERS` backstop (1024 beats in ex05), or the mux re-arbitrates mid-packet and breaks packet-atomicity -- the ex05 corruption bug. That ceiling is comfortable: at 1024 beats a board holds the port for roughly 10 us, during which each other board (2 MB/s, about one word per 2 us) accumulates only about 5 words, negligible against even a shrunk elastic FIFO. So a generous `MAX` gives large, descriptor-cheap packets under load with no real fairness cost.

Chunk atomicity holds only while the ADC data lane carries nothing but whole 4-word sample-sets -- see the debug-lane cleanup below.

## Debug lanes: cleanup required before completion

The ADC debug words (`ADC_DBG_*` codes in the top nibble) are currently interleaved into the `adc_data` lane -- a post-hoc hack that shared the lane to keep FIFO utilization low. That interleaving breaks the 4-word chunk atomicity the packetizer depends on, so the debug stream must move onto its own dedicated ADC debug lane before this integration is considered complete. In normal operation debug is disabled and the `adc_data` lane is already clean 4-word chunks, so the port proceeds on that assumption for now, but the split is a required closing task, not optional.

The DAC side is the mirror image: `dac_data_fifo` is really only a readback/debug lane, so it should be renamed `dac_debug` to reflect what it is. Both debug lanes stay on the mmap path -- they are low-throughput and not part of the DMA datapath. The original motivation for sharing lanes (BRAM pressure) goes away once DMA lets the data FIFOs shrink, so the clean split is affordable exactly when this work lands.

## Reserved memory

`u-dma-buf` is backed by a single `reserved-memory` region rather than the general CMA pool, so allocation cannot fail from fragmentation at trigger time. The size is set at **64 MB** -- about 6% of the 1 GB LPDDR2 and roughly 16x the largest current sequence (100 ms, under 4 MB), with ample room for the descriptor rings, while leaving the rest of DDR for Linux. That is comfortable, so it is fixed rather than left open.

It stays a single editable value. PetaLinux device-tree sources run through the C preprocessor, so a `#define` at the top of the project's `device_tree.dtsi` (for example `#define SHIM_DMA_RESERVED_SIZE 0x04000000`) is the one source of truth: the `reserved-memory` node's `reg` uses it, the `u-dma-buf` node points at that region by phandle (`memory-region = <&...>`) so it carries no duplicate size, and the software reads the actual size at runtime from `/sys/class/u-dma-buf/udmabuf0/size` rather than hardcoding it. Changing that one `#define` and rebuilding re-sizes the whole path.

## Datapath and platform notes

A few platform facts and datapath rules carry over from the ex05 de-risking and the Zynq platform:

- Stream width stays 32 bits to match the FIFOs; the memory-map side is 64 bits to match the HP port. Do not use 32-bit HP mode.
- Odd-length (5-word, 20-byte) DAC commands are fine on the 64-bit path with no user-side pointer, alignment, or dropped-data handling. The descriptor's byte-granular length register, not the bus width, sets how many 32-bit beats reach the DAC: a 20-byte transfer emits exactly 5 beats with `tlast` on the 5th, and if the length rounds the final DDR read up by 4 bytes, that tail half-word is dropped inside the DMA and never reaches the stream. The only rules are that each buffer's base be 8-byte aligned (already true of `u-dma-buf`'s cache-line-aligned buffers) and its size be rounded up to an 8-byte multiple so the tail over-read stays in-region. Concatenated commands pack with no gaps -- the SPI core re-frames the 32-bit stream into 5-word commands by counting -- so do not give each command its own descriptor at 20-byte spacing, the one layout that would create unaligned bases.
- Widen the MCDMA buffer-length register to 23 bits (8 MB). The 14-bit default caps a descriptor at 16 KB, which would turn a one-descriptor per-board buffer into a long chain.
- The SG descriptor rings need known physical addresses too; give them their own small `u-dma-buf` region alongside the data region.
- `u-dma-buf` is mapped cached, with one `sync_for_device` per buffer before a run and `sync_for_cpu` on readback -- the prebuffered model collapses coherency handling to a single call at a known point rather than anything in a hot path.
- Zynq TrustZone default: PS peripherals are secure by default and accesses with `AxPROT[1]=1` return DECERR. Check this first if DMA transfers fail immediately with no other explanation.
- Optional diagnostic: a FIFO-level watchdog that flags an approaching underrun before it becomes a shutdown, kept alongside the existing error and halt path.

## Software: event-driven run control

Folding DMA completion and faults into `hw_manager` gives the software a single event source, and the prebuffered model means software does nothing during a run -- so the run loop is just an event wait. Today's monitor (`sys_sts.c`, `hw_manager_irq_thread_func`) is observe-only: it `read()`-blocks on `/dev/uio0`, prints the status word, and exits its own loop, with no wire into the worker threads' stop path, so a fault is seen but drives nothing. The clean integration replaces it with an active run-controller:

- Block with `poll()` over the `hw_manager` interrupt fd (the `pl-irq` device, per Stage 1) plus an `eventfd` / self-pipe, so a hardware fault and a local abort (a software-detected error, or SIGINT folded into the eventfd) unblock the same wait.
- On wake, read the sticky status word as the source of truth and dispatch: a normal end (HALTING/HALTED) proceeds to `sync_for_cpu` and readback; a fault code drives the coordinated shutdown, which sets any active worker's existing `request_stop` and runs `hw_power_off`.
- Re-arm the interrupt (`write(fd, &1, 4)`) each cycle. The status register is read only when the doorbell fires -- there is no spin-polling.

This is the piece that makes a fault actually interrupt the operation instead of being noticed late, and it is shared by `waveform` and `static-shims`.

## Staged plan

Each stage is independently testable and leaves the tree in a working state. Per the repo rule, do not run any `make` / build target unless explicitly asked -- these stages call out where a build and a hardware run are needed, for you to run.

**Stage 0 -- Plan and decisions (this document).** The design questions are closed (see Decisions). The earlier `PROJECT_BRIEF.md` and `PROJECT_INSTRUCTIONS.md` were consolidated into this document, with their still-relevant content -- background, constraints, architecture rationale, and datapath notes -- folded into the sections above.

**Stage 1 -- Non-root access and memory plumbing (no datapath change).** rev_d today maps every register and FIFO window through root `/dev/mem` (`software/*/src/sys/map_memory.c`) and takes the `hw_manager` interrupt through generic-uio `/dev/uio0`. This port moves it to non-root -- a migration the project has been meaning to do regardless -- following ex03 for the baseline pattern and ex05 for the fuller, cleaner configuration. Bring in `pl-reg-shim` and `pl-irq-shim`, project-local copies of the ex03/ex04 `pl-reg`/`pl-irq` misc-device drivers forked into `kernel_modules/` so their `of_match_table` lists this project's cores and their `/dev` names are tailored to it. `pl-reg-shim` binds each addressed PL IP by its auto-generated compatible (`xlnx,axi-sys-ctrl-1.0`, `xlnx,axi-sts-register-1.0`, `xlnx,axi-clock-timing-snoop-1.0`, `xlnx,axi-fifo-bridge-1.0`) and publishes each as a world-accessible (`mode 0666`, non-root without udev, which this rootfs cannot install) `/dev` node named from the Vivado instance label, shortened to a stable form (see the mapping below). `pl-irq-shim` (its interrupt sibling, 0666, no kernel bootarg, private compatible `zynq-toolbox,pl-irq-shim`) delivers the `hw_manager` interrupt in place of `/dev/uio0`. Migrate `map_memory.c` and `sys_sts.c` across the programs onto these devices. Separately add `u-dma-buf` for the DDR buffers, backed by a `reserved-memory` node, with the boot-script `chmod 0666` on `/dev/udmabuf*` and its `sync_*` sysfs controls (those come up root-owned with no mode knob). The `kernel_modules/` directory (this project had none) holds the two forked drivers as real sources and `u-dma-buf` as a symlink to `examples/kernel_modules/u-dma-buf` (external source, kept unforked); the build auto-discovers and autoloads each. The MCDMA control window is then a `pl-reg-shim` node like the rest. Verify on hardware that the existing tools run non-root, the `/dev` nodes and reserved region appear, and the SPI datapath is unchanged. The non-root migration is independent of the DMA datapath, so it is worth landing and testing on its own before Stage 2.

`pl-reg-shim` shortens the auto-generated Vivado instance labels into the stable `/dev` names userspace opens. The three singleton control windows are exact renames; the per-board FIFO bridges strip the `axi_spi_interface_` prefix and `_axi_bridge` suffix, so the board index carries through. The mapping is a table at the top of `kernel_modules/pl-reg-shim/petalinux/pl-reg-shim.c`, kept in step with the shim programs' `src/sys/*.h` device-name macros:

| Register window | Vivado instance label | `/dev` name |
|---|---|---|
| System control (`0x40000000`) | `axi_sys_ctrl` | `sys_ctrl` |
| System status (`0x40100000`) | `status_reg` | `sys_sts` |
| SPI clock (`0x40200000`) | `spi_clk_snoop` | `spi_clk` |
| DAC command FIFO, board *b* | `axi_spi_interface_dac_fifo_<b>_axi_bridge` | `dac_fifo_<b>` |
| ADC data FIFO, board *b* | `axi_spi_interface_adc_fifo_<b>_axi_bridge` | `adc_fifo_<b>` |
| Trigger FIFO (`0x80100000`) | `axi_spi_interface_trig_fifo_axi_bridge` | `trig_fifo` |

**Stage 1 follow-up -- board-count awareness (software).** The non-root port made register access fail loudly: `map_pl_reg` exits if a board's `/dev` node is absent, so on a partial-board bitstream (for example the four-board bench build) the FIFO-mapping tools -- `shim-test`, `waveform`, `static-shims` -- exit at the first missing `dac_fifo_<b>`. The old `/dev/mem` path silently mapped a non-existent window instead, so this is a latent assumption surfaced, not a regression. The fix is to have the software learn the active board count at runtime rather than hardcoding eight. The count is discovered by probing which `/dev/dac_fifo_<b>` nodes exist (boards are contiguous `0..N-1`): the auto-generated device tree, and hence the `pl-reg-shim` nodes, already track whatever `num_ch` the bitstream was built with, so this needs no manual sync, no build-time coupling, and no new register. Arrays stay sized to a compile-time `MAX_BOARDS` (eight, the ceiling of 64 channels at eight per board) -- over-allocating to the maximum is free -- so the change stays small: a cached `board_count()` helper, the two mapping loops (`create_dac_ctrl` / `create_adc_ctrl`) zeroing their buffer arrays and iterating to the discovered count, and the central `parse_board_number` validator bounding on that count so every command rejects an absent board cleanly from one place. The many inline "must be 0-7" messages are then the only cosmetic follow-up. This is independent of the DMA datapath and can land any time after Stage 1.

**Stage 2 -- PL datapath (MCDMA plus routing), mmap path preserved.** Add the `axi_mcdma` (8 MM2S + 8 S2MM), an `axis_switch` demux fanning MM2S to the eight DAC-command `axis_fifo_bridge` instances by `tdest`, and a packet-atomic `axis_switch` mux recombining the eight ADC-data streams into S2MM. Reuse the ex05 mux settings that were hard-won there (`HAS_TLAST` and `ARB_ON_TLAST` set together, the `tdest` window in `0x` hex form, `ARB_ON_MAX_XFERS` backstop). Add the ADC packetizer core (4-word chunk-atomic, adaptive off `fifo_count_rd_clk`, `MAX` chunks at or under the mux backstop), the datapath-mode mux against the existing `axi_fifo_bridge`, the HP0 64-bit master, and the 23-bit buffer-length register. Fold the sixteen MCDMA completion/error `introut` lines into `hw_manager` -- it aggregates them into its state machine and status word and keeps raising its single `ps_interrupt`, so DMA completion and faults ride `hw_manager`'s single interrupt (delivered non-root via `pl-irq`, per Stage 1) rather than a separate one. After a build and boot, confirm a byte-exact loopback-style transfer on the DMA-driven FIFOs, and confirm the mmap fallback still works with the mode bit set to PS.

**Stage 3 -- Software.** Port the ex05 userspace MCDMA mover -- register model, SG-ring construction, `u-dma-buf` mapping -- into a shared rev_d library, consumed at two levels. `shim-test` gains low-level DMA test commands for bring-up and debugging; that codebase is bloated and a broader refactor is expected, but the low-level access is worth having there. `waveform` and `static-shims` get the full clean integration: prebuffer the `dac_cmd` sequence and reserve the `adc_data` capture region, set each board's `datapath_mode` to DMA, `sync_for_device`, arm the channels, release the trigger, wait on the folded `hw_manager` interrupt for run-end-or-fault via the event-driven run-controller, then on a normal end `sync_for_cpu` and read back the ADC data. Verify a real waveform plays end to end through the DMA.

**Stage 4 -- Fault and reset coordination.** Make "off" a sequence, not a single pulse: halt the MCDMA channels, then assert `buf_reset` on the DMA-driven FIFOs, then reinit the descriptor rings before the next run. This mirrors the ex05 halt-reset result -- an MCDMA S2MM soft-reset drains whatever is pending downstream, so `buf_reset` is uniquely needed for data stranded upstream in the DAC FIFO while a channel is paused. `hw_manager` sequences the hardware side (FIFO reset only after the MCDMA halt); software reinits the rings. Keep underrun/overflow detection where it already lives -- in the DAC/ADC cores reporting to `hw_manager` -- and do not add a PS-side timeout, since a channel waiting arbitrarily long on a trigger is normal, not a fault. Verify a clean halt and a recoverable restart after an injected fault.

**Stage 5 -- Validation and reclaim.** With the datapath proven, reclaim resources: shrink the DMA-driven CDC FIFOs from whole-waveform depth toward microsecond elastic buffers (freeing most of the BRAM they hold today), and drop redundant PS-side bridges if the mmap fallback is retired. Split the ADC debug words onto a dedicated debug lane and rename `dac_data` to `dac_debug` (see Debug lanes) -- the shrunk buffers make this affordable, and it is required before the integration is complete. Watch LUT, which is the binding constraint at eight boards. Update this document to reflect the integrated state.

## Deferred: streaming and loopback

Neither is a current requirement; both are recorded so the design does not foreclose them.

Reduced-rate streaming: rather than cyclic mode (which the MCDMA driver lacks), append descriptors ahead of the queue tail while the run proceeds. The deadline is not FIFO drain time but the time to drain what is already queued -- 8 MB queued on a DAC channel at 2.5 MB/s is 3.2 s of runway, enormous against Linux scheduling jitter -- so this works with MCDMA as-is. The real limit is source bandwidth: sustaining 36 MB/s from the SD card or on-board Wi-Fi is not realistic, though Gigabit Ethernet on a baseboard would be, and at reduced rates almost any source works.

DAC-side loopback latency is dominated by how much data is queued ahead of new data (`queued_bytes / consumption_rate`), not by the DMA itself. Hitting 100 ms at full rate means keeping under ~250 KB queued per DAC channel, which is comfortable; hitting 1 ms would mean ~2.5 KB, below current FIFO depth, so latency would then be set by the PL FIFO itself and would need shrunk FIFOs plus real-time scheduling, directly trading away underrun margin. ADC-side loopback is not anticipated; ADC data is read after the run.

## Decisions

Every design question is settled. ADC stream framing is 4-word chunk-atomic adaptive packetization off `fifo_count_rd_clk`, which self-flushes the run tail (see ADC stream framing and packetization). Datapath-mode control is a per-board `datapath_mode` mask in `axi_sys_ctrl`, reset to mmap, with a `mode_viol` fault routed through `hw_manager` (see Coexistence). DMA completion and faults fold into `hw_manager`'s single interrupt, delivered non-root via `pl-irq` and consumed by an event-driven run-controller (see Software: event-driven run control). The port migrates rev_d off root `/dev/mem` onto `pl-reg` / `pl-irq` (Stage 1). The `reserved-memory` size is fixed at 64 MB, held as a single `#define` in the project device tree (see Reserved memory). No open questions remain.

## References

- `projects/ex05_dma/README.md` -- the proven reference implementation (MCDMA, `pl-irq`, halt-reset, fault injection).
- `modules/axi_spi_interface.tcl` -- the current per-board FIFOs, bridges, and reset chain.
- `cores/base/axis_fifo_bridge/` -- the AXIS-to-`fifo_async` adapter used on the DMA side.
- `cores/shim/hw_manager/` and `cores/shim/axi_sys_ctrl/` -- the fault/halt path and the buffer-reset masks.
- `examples/kernel_modules/{pl-reg, pl-irq}` -- the ex03/ex04 drivers forked into `kernel_modules/{pl-reg-shim, pl-irq-shim}` (register windows; the `hw_manager` interrupt); `examples/kernel_modules/u-dma-buf` -- symlinked unforked into `kernel_modules/` for the DDR buffers.
- `projects/ex03_device_driver/` and `projects/ex05_dma/` -- the non-root access pattern (ex03 the baseline, ex05 the fuller configuration).
- Vendor documentation: UG585 (Zynq-7000 TRM), PG021 (AXI DMA), PG288 (AXI MCDMA) -- fetch the specific leaf sections; the docs.amd.com landing pages render as navigation only.
