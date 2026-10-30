# DMA Port Plan: DDR-Backed Buffering for the Rev D Shim

This is the plan for porting DDR-backed DMA buffering into `rev_d_shim`. The architecture was proven on hardware in the `ex05_dma` testbed, so this is a port rather than an open design effort. It covers the background and constraints, what changes in this project, in what order, and the decisions behind each step.

## Background: the problem and the core insight

The instrument is a 64-channel dynamic current driver with per-channel readback on a krtkl snickerdoodle black (Xilinx XC7Z020: dual Cortex-A9 at 866 MHz, 1 GB LPDDR2, 140 BRAM36 tiles, 53,200 LUTs). Eight daughter boards each carry an 8-channel 16-bit DAC and an 8-channel 16-bit ADC; a per-board PL core speaks SPI to those chips, fed by (DAC) or draining into (ADC) a 32-bit clock-domain-crossing FIFO. Sample timing is a per-update clock-cycle delay, and the SPI clock is set per sequence between 1 and 30 MHz, capping any one board near 125 kSa/s -- though boards run at unrelated rates and pause independently. Board-to-board start alignment is handled by a separate trigger system and is out of scope, but the trigger can be withheld until the PS reports prebuffering is complete -- the hook the whole design hangs on.

At the 125 kSa/s ceiling a board moves 5 x 32-bit words (20 bytes) per DAC update and 4 x 32-bit words (16 bytes) per ADC sample -- 2.5 MB/s out and 2.0 MB/s in, so 36 MB/s aggregate across eight boards at simultaneous maximum. That is the absolute worst case; real sequences usually run hundreds of samples per second, in bursts separated by idle.

Without DMA, longer or faster sequences don't fit in the on-chip BRAM FIFOs, and a Linux userspace thread can't refill or drain sixteen FIFOs by hand within the timing floor. A DAC underrun or ADC overflow trips an immediate hardware shutdown, so the failure is hard, not graceful.

The resolving insight: sequences of interest are around 100 ms and under 4 MB, and the trigger can be held until prebuffering completes, so the whole sequence can sit in LPDDR2 before the run starts and software plays no part during it. That turns "meet a ~1.6 ms FIFO-drain deadline under Linux scheduling jitter" into "sustain 36 MB/s of descriptor-driven DMA against ~800 MB/s of HP-port bandwidth" -- roughly 20x margin, with no deadline-sensitive software path. This prebuffered-sequence assumption is load-bearing; if it ever breaks (see Deferred), the analysis must be redone. Supported tool versions are PetaLinux and Vivado 2024.1 and 2024.2.

## Constraints

Hard constraints:

- DAC FIFOs must never underrun and ADC FIFOs must never overflow: either trips an immediate halt and hardware shutdown, reported to the PS by error code and recoverable only with human intervention.
- No continuation after a fault: the run is invalid and restarts from the beginning.
- Per-board independence: boards are individually addressable, startable, pausable, and stoppable. A shared round-robin stream across boards is rejected -- alignment is the trigger system's job, and boards deliberately run at unrelated rates.
- Rate independence: any board may run from ~125 kSa/s down to effectively zero, with arbitrary pauses, independent of the others.
- The per-FIFO userspace API shape should survive -- mmap once, no syscalls in the inner loop.

Soft preferences: push the rate ceiling until it's an SPI-timing limit rather than a data-movement or CPU limit; keep LUT growth modest, since LUT is the binding constraint; free BRAM where convenient; and avoid requiring PREEMPT_RT.

## The decided design

A single AXI MCDMA with 8 MM2S and 8 S2MM channels (one pair per board), driven by direct userspace register control over `u-dma-buf` buffers, with the whole sequence prebuffered into LPDDR2 before the trigger fires. `projects/ex05_dma/` confirmed this byte-exact on hardware, including `pl-irq`-delivered completion interrupts and a halt/clear/reinit reset sequence; its README is the reference implementation.

One MCDMA maps 1:1 onto the eight boards, keeps the per-FIFO API shape, and puts one AXI master pair on a single HP port, at about 13.9k LUT for the full 8+8 engine in ex05 (mcdma 8.7k, HP0 SmartConnect 4.4k, GP0 control 0.5k, demux/mux 0.4k). The alternatives lose: eight AXI DMA instances would be 16-24k LUT plus a 16-master SmartConnect, the PS DMAC (PL330) has about four PL request interfaces against sixteen streams, and a custom per-board datamover trades known driver risk for unknown RTL risk. LUT, not BRAM, is the binding constraint: before the reclaim the whole design extrapolated to ~52-54k LUT at eight boards (~97-102% of the device), and the Stage 5 reclaim brings it well under (see `RESOURCE_RECLAIM.md`). Driving the engine from userspace avoids the lightly-exercised in-kernel MCDMA `device_prep_slave_sg` path; the register map and descriptor layout match mainline `xilinx_dma.c` (notably the BD `control` word at 0x14 and Run/Stop in both the per-channel and common control registers).

## What the DMA drives, and what it does not

Only the two high-throughput streams per board move onto the DMA:

- `dac_cmd` is fed by an MM2S channel: the DAC command and setpoint stream, five 32-bit words per update.
- `adc_data` is drained by an S2MM channel: the ADC sample stream.

The other lanes are low-throughput and stay on programmed I/O (PIO) -- direct CPU register access through the `axi_board_fifo_array` window (and the trigger `axi_fifo_bridge`), mapped once with `mmap()`. In PIO mode the two high-rate lanes use the same window:

| FIFO | Depth | Direction | Path |
|---|---|---|---|
| `dac_cmd` | 2^10 | PS -> DAC core | MM2S DMA (PIO in PIO mode) |
| `adc_data` | 2^10 | ADC core -> PS | S2MM DMA (PIO in PIO mode) |
| `dac_debug` | 2^12 | DAC debug words -> PS | PIO |
| `dac_data` | 2^5 | DAC `GET_CAL` readback -> PS | PIO |
| `adc_cmd` | 2^12 | PS -> ADC core | PIO |
| `adc_debug` | 2^12 | ADC debug words -> PS | PIO |
| `trig_cmd` / `trig_data` | 2^10 | trigger core | PIO |

## The starting point in this project

Before the port, each board had four `base:user:fifo_async` CDC FIFOs (32-bit, written in the 100 MHz AXI domain, read in the SPI domain) in `modules/axi_spi_interface.tcl`, reached through `base:user:axi_fifo_bridge` (AXI4-Lite PIO) on `M_AXI_GP1`. `hw_manager` owns the fault, halt and shutdown path and raises `ps_interrupt` into `IRQ_F2P`, and `axi_sys_ctrl`'s per-board `cmd_buf_reset` / `data_buf_reset` masks are the hook the halt/clear/reinit sequence needs.

The ADC path was a plain FIFO read with an in-band end-of-command marker and no `tlast` or `tdest`, so stream framing was new PL work. The rev_d FIFOs are plain `fifo_async` rather than the native-AXIS FIFOs ex05 used, but the repo's `base:user:axis_fifo_bridge` adapts AXI4-Stream to exactly that interface, so no new adapter core was needed.

## Coexistence of the PIO and DMA paths

The `dac_cmd` write port and the `adc_data` read port each have two possible owners -- the PIO lane and the DMA side (an `axis_fifo_bridge` for DAC, the packetizer for ADC) -- and they must never both be live.

Control is `datapath_mode` in `axi_sys_ctrl` (word 11), an 8-bit per-board mask like the block's other per-board masks: 0 is PIO, 1 is DMA, reset 0, so power-on comes up in PIO and DMA is opt-in per board. One bit covers both of a board's high-rate lanes, since a board plays and captures as a unit, and it sits behind `axi_sys_ctrl`'s config lock, so a board's path can't change mid-run. Per-board costs nothing over a global bit (the mux is per-FIFO anyway) and allows board-at-a-time bring-up.

The bit drives a per-board 2:1 selection on the shared FIFO ports and gates the non-owner's handshake as well as its data -- the PIO write path in DMA mode, and the DMA side's `tready` / `rd_en` in PIO mode -- so the switch is a hard lock, not a convention.

A wrong-mode access (a PIO poke at a DMA-mode board) must neither fail silently nor crash the program. The PIO lane in `axi_board_fifo_array` sees the raw access, so it reports it: it always responds `OKAY` (never `SLVERR`, so no CPU external abort or `SIGBUS`), discards the access, and raises `mode_viol`. That folds into `hw_manager` like the other per-board faults, so the run halts through the normal shutdown (status word plus `ps_interrupt`, caught by the run-controller). Genuine full/empty conditions on the owned port still raise `fifo_overflow` / `fifo_underflow`.

Like those flags, `mode_viol` lives in the FIFO's reset domain and clears on a buffer reset or global reset, not on `off`. The `waveform` DMA teardown (Stage 4) pulses those resets on every run exit; in `shim-test`, `hard_reset` clears them.

A build-time `use_dma` knob was the zero-logic alternative, but it gives no runtime fallback and leaves two build variants.

## ADC stream framing and packetization

For S2MM through the shared packet-atomic mux, each ADC stream needs `tdest` and `tlast`. `tdest` is the board index, so the mux and MCDMA route board *i*'s samples to channel *i*'s buffer.

`tlast` uses adaptive word-granular packetization. Packet boundaries can't follow the 4-word sample set (eight 16-bit samples in four 32-bit words), because a single-channel `ADC_RD_CH` read writes one word, so the lane mixes 4-word and 1-word reads. A packet may close on any word and a read may be split across packets; software reassembles samples from the capture by position. The packetizer drains the resident words and closes the packet when the FIFO is about to run dry or the packet hits a cap.

A fixed packet size would strand the mux: it holds a granted board until `tlast`, so a board whose FIFO momentarily runs dry mid-packet would block every other board and risk an ADC-overflow shutdown. Closing the packet when the board's data runs out releases the mux at once, self-flushes the run tail with no end-of-sequence signal, and turns a slow trickle into single-word packets.

The signal is `fifo_async`'s read-side fill count, `fifo_count_rd_clk`, already in the mux's 100 MHz domain. It comes from a Gray-synchronized pointer, so it can lag but never over-report (worst case, a packet closes one word early). The packetizer asserts `tlast` at the last resident word (`fifo_count_rd_clk <= 1`) or at `MAX_PACKET_WORDS`, and drives a full AXIS master (`tdata`, `tvalid`, `tready`, `tlast`, `tdest`). It replaces `axis_fifo_bridge` on the ADC side, which is `tdata`-only.

`MAX_PACKET_WORDS` must stay at or under the mux's `ARB_ON_MAX_XFERS` backstop (1024 beats), or the mux re-arbitrates mid-packet and breaks packet atomicity -- the ex05 corruption bug. At 1024 beats a board holds the port for ~10 us, while each other board accumulates only ~5 words, so a generous cap costs no real fairness.

The S2MM capture is therefore a flat word stream that software slices back into reads, which is why debug words have to stay off it (see Debug lanes).

## Debug lanes

Each core has its own debug output, so debug words never land in the DMA capture. Every board has four read lanes in the board FIFO array: `adc_data` (samples only, both 8-channel and single-channel reads, so DMA-mode calibration still captures through S2MM), `adc_debug`, `dac_debug`, and a small `dac_data` (2^5, one RAMB18) for `GET_CAL` readback only, which keeps a halting overflow like `adc_data`.

The debug lanes stay on PIO in both modes and are 2^12 deep. A debug word arriving at a full debug FIFO is dropped and sets a sticky per-board flag instead of halting; the flags are gathered into one `sys_sts` word and clear when the system powers off. The DMA teardown (which resets `dac_cmd` and `adc_data`) never touches the debug FIFOs: `dac_debug` and `dac_data` share the board's DAC data-reset bit, and `adc_debug` resets with the ADC command FIFO.

## Reserved memory

`u-dma-buf` uses dedicated `reserved-memory` regions rather than the general CMA pool, so allocation can't fail from fragmentation at trigger time: 8 MB of SG descriptors (`udmabuf1`) and 128 MB of data (`udmabuf0`, about 13% of the LPDDR2) for the prebuffered DAC commands and the ADC capture, each shared across boards by a software bump-allocator.

The sizes are `#define`s at the top of the project's `device_tree.dtsi` (`SHIM_DMA_DESC_SIZE`, `SHIM_DMA_DATA_SIZE` and their bases), since PetaLinux device-tree sources go through the C preprocessor. The `reserved-memory` nodes use them, the `u-dma-buf` nodes reference their region by phandle, and software reads the real sizes from `/sys/class/u-dma-buf/udmabufN/size`, so changing a `#define` and rebuilding resizes everything.

The sizes match the capture-packing ratio: one 64-byte descriptor per 256-word (1024-byte) packet means 8 MB of descriptors exactly backs 128 MB of data. The mover rejects an oversized run with a clear message. See Capture descriptor packing.

## Datapath and platform notes

Platform facts and datapath rules carried over from ex05:

- The stream side stays 32 bits to match the FIFOs; the memory side is 64 bits to match the HP port. Don't use 32-bit HP mode.
- Odd-length (5-word, 20-byte) DAC commands are fine on the 64-bit path: the descriptor's byte length, not the bus width, sets how many beats reach the DAC, and any rounded-up tail is dropped inside the DMA. Each buffer's base must be 8-byte aligned and its size rounded up to 8 bytes. Commands pack with no gaps (the SPI core re-frames by counting), so never give each command its own descriptor at 20-byte spacing -- the one layout with unaligned bases.
- The MCDMA buffer-length register is widened to 23 bits (8 MB); the 14-bit default caps a descriptor at 16 KB.
- `u-dma-buf` is mapped non-cached (`O_SYNC`), so descriptor write-back and captured data are coherent with no `sync_for_device` / `sync_for_cpu` calls.
- Zynq TrustZone: PS peripherals are secure by default and accesses with `AxPROT[1]=1` return DECERR. Check this first if DMA transfers fail immediately.
- Optional diagnostic: a FIFO-level watchdog that flags an approaching underrun before it becomes a shutdown.

## Software: event-driven run control

Folding DMA completion and faults into `hw_manager` gives software a single event source, and with prebuffering software does nothing during a run, so the run loop is an event wait. The shared run-controller (`libraries/src/sys/run_ctrl.c`) does it:

- `poll()` over `/dev/hw_manager_irq` plus an `eventfd`, so a hardware fault and a local abort (SIGINT or a software error) unblock the same wait.
- On wake, read the sticky status word and report a benign event, fault, abort, or backstop timeout. The caller loops, draining captures each wake; normal end is a software fact (all expected capture words read), and a fault or abort drives the coordinated shutdown (`request_stop` plus `hw_power_off`).
- Re-arm the interrupt each cycle; the status register is read only when it fires, with no spin-polling.

`waveform` DMA runs use it; `static-shims` stays on PIO (see Stage 3).

## Staged plan

Each stage is independently testable and leaves the tree working. Builds and hardware runs are yours to run (per the repo rule, `make` is never run unprompted).

**Stage 0 -- Plan and decisions (this document).** The design questions are closed (see Decisions); the earlier `PROJECT_BRIEF.md` and `PROJECT_INSTRUCTIONS.md` are folded in above.

**Stage 1 -- Non-root access and memory plumbing (no datapath change).** Before the port, rev_d mapped every window through root `/dev/mem` and took the `hw_manager` interrupt through generic-uio `/dev/uio0`. This stage moves to non-root access, following ex03 (baseline) and ex05 (fuller configuration). `pl-reg-shim` and `pl-irq-shim` are project-local forks of the ex03/ex04 `pl-reg` / `pl-irq` misc-device drivers in `kernel_modules/`, matching this project's cores. `pl-reg-shim` binds each PL IP by its auto-generated compatible (`xlnx,axi-sys-ctrl-1.0`, `xlnx,axi-sts-register-1.0`, `xlnx,axi-clock-timing-snoop-1.0`, `xlnx,axi-board-fifo-array-1.0`, `xlnx,axi-fifo-bridge-1.0`) and publishes each as a mode-0666 `/dev` node (this rootfs has no udev) named from its Vivado instance label. `pl-irq-shim` (0666, private compatible `zynq-toolbox,pl-irq-shim`, no bootarg) delivers the `hw_manager` interrupt in place of `/dev/uio0`, and `map_memory.c` / `sys_sts.c` use these devices. `u-dma-buf` (a symlink to `examples/kernel_modules/u-dma-buf`, kept unforked) provides the DDR buffers, with the boot script `chmod 0666`ing `/dev/udmabuf*` and their `sync_*` sysfs controls. The build auto-discovers and autoloads every module under `kernel_modules/`, and the MCDMA control window is a `pl-reg-shim` node like the rest. Verify that the tools run non-root, the `/dev` nodes and reserved regions appear, and the SPI datapath is unchanged.

`pl-reg-shim`'s name table (top of `kernel_modules/pl-reg-shim/petalinux/pl-reg-shim.c`, kept in step with the `src/sys/*.h` device-name macros) maps instance labels to `/dev` names:

| Register window | Vivado instance label | `/dev` name |
|---|---|---|
| System control (`0x40000000`) | `axi_sys_ctrl` | `sys_ctrl` |
| System status (`0x40100000`) | `status_reg` | `sys_sts` |
| SPI clock (`0x40200000`) | `spi_clk_snoop` | `spi_clk` |
| DAC/ADC FIFOs, all boards (`0x41000000`, a 4 KB page per board) | `axi_spi_interface_board_fifo_array` | `board_fifo` |
| Trigger FIFO (`0x42000000`) | `axi_spi_interface_trig_fifo_axi_bridge` | `trig_fifo` |

**Stage 1 follow-up -- board-count awareness (software).** Non-root access fails loudly on a FIFO window a partial-board bitstream never built (where `/dev/mem` silently poked nothing), so software learns the board count at runtime. `board_count()` counts built boards from the FIFO-present bit (31) in each board's `sys_sts` FIFO status word, with no new register or build coupling. Arrays stay sized to `MAX_BOARDS` (8); the mapping loops iterate to the discovered count, and `parse_board_number` rejects absent boards for every command in one place.

**Stage 2 -- PL datapath (MCDMA plus routing), PIO path preserved.** Add the `axi_mcdma` (8 MM2S + 8 S2MM), an `axis_switch` demux fanning MM2S to the per-board DAC-command `axis_fifo_bridge` instances by `tdest`, and a packet-atomic `axis_switch` mux recombining the ADC streams into S2MM, reusing ex05's hard-won mux settings (`HAS_TLAST` with `ARB_ON_TLAST`, the `tdest` window in `0x` hex form, the `ARB_ON_MAX_XFERS` backstop). Add the ADC packetizer, the datapath-mode mux, the HP0 64-bit master, and the 23-bit buffer-length register. Fold the sixteen MCDMA `introut` lines into `hw_manager`: each is level-high while its status bits are set and doesn't distinguish completion from error, so `hw_manager` pulses its single `ps_interrupt` on any `introut` event during a run (staying in `S_RUNNING`) and software reads the MCDMA status to tell them apart. Confirm a byte-exact transfer on the DMA-driven FIFOs, and that PIO mode still works.

**Stage 3 -- Software.** Port the ex05 userspace MCDMA mover (register model, SG-ring construction, `u-dma-buf` mapping) into a shared library. `shim-test` gets low-level bring-up commands (`dma_mode`, `dma_status`, `dma_channel_test`, `dma_irq_test`, `dma_mode_viol`) and a multi-word bench run (`dma_waveform_test` / `_status` / `_stop`, an 8-channel triangle-times-envelope sequence); both are hardware-validated, and `DMA_TEST_PLAN.md` Stage 3.1 records the command-format and reset facts they pinned down (global MCDMA reset, DAC `LDAC` latch, trigger-wait placement, `ADC_RD` repeat count).

`waveform` gets the full integration, also hardware-validated. A DMA run (the default; `--pio` opts out) sets `datapath_mode` to DMA for all active boards before power-on, synthesizes each board's DAC stream byte-identically to the PIO feed, prebuffers every board at once (per-board runs in a region bump-allocator, with a DAC-only no-capture mode), keeps the ADC command and trigger lanes on their PIO stream threads, captures `adc_data` through S2MM, and runs the run-controller loop to drain the capture and handle faults and aborts. `static-shims` stays PIO by design -- it drives static setpoints interactively and never approaches the rate that motivates DMA.

**Stage 4 -- Fault and reset coordination.** The DMA run teardown is a sequence, not a bare power-off. On every `waveform` DMA run exit (completion, fault, or abort), `dma_wave_halt` soft-resets both MCDMA directions (stopping every channel and draining the S2MM path), then `hw_reset_dma_buffers` pulses `buf_reset` on the active boards' `dac_cmd` and `adc_data` FIFOs, dropping stranded DAC commands and clearing latched lane faults; the rings reinit on the next `dma_wave_begin`. This mirrors ex05: the S2MM soft-reset drains downstream, so `buf_reset` is needed only for data stranded upstream in the DAC FIFO. It's all software on the existing `sys_ctrl` reset masks. Underrun/overflow detection stays in the DAC/ADC cores, and there's no PS-side timeout, since waiting arbitrarily long on a trigger is normal. Verify a clean halt and a recoverable restart after an injected fault.

**Stage 5 -- Validation and reclaim.** With the datapath proven, shrink the DMA-driven CDC FIFOs toward microsecond elastic buffers, cut LUT, and split the debug words off the data lanes (see Debug lanes). LUT scaling is dominated by the per-board DAC controllers and AXI-Lite fabric, not the DMA datapath (well under 100 LUT per board); the knobs and measured savings live in `RESOURCE_RECLAIM.md`.

## Current status and remaining work

Stages 1-4 are implemented and hardware-validated on the four-board bench: non-root access, the MCDMA datapath with its mode mux and folded interrupt, the `waveform` DMA integration behind the run-controller, and the coordinated teardown (validated on the Ctrl-C path; a mid-run hardware fault is still to be tried, see `DMA_TEST_PLAN.md` Stage 4). The ADC command stream is run-length-encoded into the hardware repeat counter, so a uniform read schedule is a couple of command words rather than one per sample.

`waveform` defaults to DMA (`--pio` opts out), writes each ADC sample with its trigger-aligned timepoint in one CSV, calibrates over the matching datapath before each run (`--no-cal` to skip), and reports an `hw_manager` halt mid-run on either datapath. `channel-check` is a one-shot PIO health check, and `static-shims` stays PIO-only.

**Stage 5 -- landed.** The DMA-driven CDC FIFOs are 2^10, and the LUT reclaim is in: DSP multipliers in the timing calcs, a single GP0 SmartConnect for register and FIFO AXI, `reset_sync` in place of the per-FIFO `proc_sys_reset` blocks, and one `axi_board_fifo_array` (a 4 KB page per board, one word per lane) in place of the per-board bridges. The debug lanes are split (with `adc_cmd` at 2^12, since it's PIO in both modes), and `adc_rate_ctrl`'s registers are a per-board rate lane in the array. A CDC and SPI timing pass closes the stage: three real crossing bugs fixed (including `sync_coherent` rebuilt as a handshake, about 1,750 LUT smaller at four boards), the external inputs synchronized, the SPI domain timed at its 30 MHz maximum, and no critical `report_cdc` entries left (the clock-mux select and boot-only DAC MISO are waived in `waivers.xdc`). All of it passes on hardware (`DMA_TEST_PLAN.md` 5.1-5.5). The four-board build is 28,588 LUT (54%); the last eight-board synthesis, after steps 5-6, was 46,535 LUT (87.5%). Still to do: an eight-board build to confirm the reclaim at full size, and tuning the DMA FIFOs toward microsecond depth, since capture packing sets the required accumulation. The hardware changes needed for 30 MHz ADC reads are in `HARDWARE_UPDATE_TODO.md`, and the further LUT knobs in `RESOURCE_RECLAIM.md` are optional headroom.

**Capture descriptor packing.** Implemented and hardware-validated (`docs/DMA_CAPTURE_PACKING.md`). The capture is described per 256-word packet rather than per word, removing the 16x descriptor overhead, with software compacting partial packets on readback. `adc_rate_ctrl` sets the density in hardware: it tracks each board's unread-capture backlog (packetizer emits minus software-reported reads, through the rate lane), and the packetizer packs dense while the backlog is at or above the packet size and drains eagerly below it, so the run tail always flushes. `TAILDESC` credits stay pure backpressure, and a RAM-full overflow is the intended detected error. v1 is the fixed capture ring drained at the end of a `waveform` run; v2 (not started) recycles the ring for live streaming and needs append-at-tail on a running S2MM channel.

**Software channel-cal and shared code.** The 5-point DAC/ADC calibration runs over either transport. Calibration is lost on power-off and `datapath_mode` is locked while powered, so a run calibrates in the same power session and on the datapath it plays on. On a DMA board the DMA transport sends each `DAC_WR_CH` / `SET_CAL` word through MM2S and captures each read through S2MM (the `ADC_RD_CH` itself still goes out on the PIO command lane), reporting each read to `adc_rate_ctrl` so the one-word packets drain eagerly. `waveform` calibrates by default; `channel-check`, `static-shims` and `shim-test` run the PIO cal. Two follow-ups: calibrating once and prepending the per-channel `SET_CAL` commands to the prebuffered DAC stream (which pairs with moving calibration out of the PL, see `RESOURCE_RECLAIM.md`); and a deliberate refactor of the shared code. Common logic lives once under `libraries/` and each tool pulls it in through relative symlinks, leaning on the build flattening every `.c` into one compile; that keeps one copy, but the symlink web is fragile and whole-directory links compile files a tool doesn't use. The sharing mechanism isn't decided yet.

## Deferred: streaming and loopback

Neither is a current requirement; they're recorded so the design doesn't foreclose them.

Reduced-rate streaming: instead of cyclic mode (which the MCDMA driver lacks), append descriptors ahead of the queue tail during the run. The deadline is the time to drain what's already queued -- 8 MB on a DAC channel at 2.5 MB/s is 3.2 s of runway -- so this works with MCDMA as-is. The real limit is source bandwidth: 36 MB/s from the SD card or Wi-Fi isn't realistic, though Gigabit Ethernet would be, and at reduced rates almost any source works.

DAC-side loopback latency is set by how much data is queued ahead (`queued_bytes / consumption_rate`), not by the DMA. 100 ms at full rate means under ~250 KB queued per DAC channel, which is comfortable; 1 ms means ~2.5 KB, below the FIFO depth, which would need shrunk FIFOs plus real-time scheduling at the cost of underrun margin. ADC-side loopback isn't anticipated.

## Decisions

All design questions are settled:
- ADC framing: adaptive word-granular packetization off `fifo_count_rd_clk`, with no sample-set alignment; software recombines reads (see ADC stream framing and packetization).
- Datapath control: a per-board `datapath_mode` mask in `axi_sys_ctrl`, reset to PIO, with `mode_viol` routed through `hw_manager` (see Coexistence).
- Events: DMA completion and faults fold into `hw_manager`'s single interrupt, delivered non-root via `pl-irq` to the run-controller.
- Access: rev_d uses `pl-reg` / `pl-irq` instead of root `/dev/mem` (Stage 1).
- Memory: two `reserved-memory` regions (8 MB descriptors, 128 MB data), each sized by a `#define` in the device tree.

## References

- `projects/ex05_dma/README.md` -- the proven reference implementation (MCDMA, `pl-irq`, halt-reset, fault injection).
- `modules/axi_spi_interface.tcl` -- the per-board FIFOs, the board FIFO array, and the reset chain.
- `cores/base/axis_fifo_bridge/` -- the AXIS-to-`fifo_async` adapter used on the DMA side.
- `cores/shim/hw_manager/` and `cores/shim/axi_sys_ctrl/` -- the fault/halt path and the buffer-reset masks.
- `examples/kernel_modules/{pl-reg, pl-irq}` -- the ex03/ex04 drivers forked into `kernel_modules/{pl-reg-shim, pl-irq-shim}` (register windows; the `hw_manager` interrupt); `examples/kernel_modules/u-dma-buf` -- symlinked unforked into `kernel_modules/` for the DDR buffers.
- `projects/ex03_device_driver/` and `projects/ex05_dma/` -- the non-root access pattern (ex03 the baseline, ex05 the fuller configuration).
- Vendor documentation: UG585 (Zynq-7000 TRM), PG021 (AXI DMA), PG288 (AXI MCDMA) -- fetch the specific leaf sections; the docs.amd.com landing pages render as navigation only.
