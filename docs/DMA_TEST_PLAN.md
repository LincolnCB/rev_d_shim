# DMA Port Test Plan

The companion to `DMA_PLAN.md`: a running checklist of what to verify at each stage of the DMA port, with the same stage numbering. `DMA_PLAN.md` says what to build and why; this says how you know it worked.

Each check records how to run it, what a pass looks like, and a dated result. Host-only checks (device tree, module and core builds) can be marked from the artifacts in `tmp/`; the rest need the board.

Result legend for each check:
- `[ ]` not yet run
- `[PASS]` ran, met the expected result -- add the date and any notes
- `[FAIL]` ran, did not meet it -- add the date, what happened, and the follow-up
- `[N/A]` not applicable in the end (say why)

Unless noted, commands run on the target as a normal, non-root user. Build-artifact checks run on the host from the repo root, against the latest `make petalinux_build` for `snickerdoodle_black 1.0 rev_d_shim` (outputs under `tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/`).

---

## Stage 1 -- Non-root access and memory plumbing

The tools run unprivileged, the DMA `/dev` nodes and reserved DDR appear, and the SPI datapath is unchanged. Stage 1 changes no datapath, so acceptance is "nothing got worse, and the new plumbing is present."

### Steps 1-2 -- kernel_modules wiring and reserved memory (built)

The device-tree and module-build changes.

**1.2.a -- The three modules are in the image (host).**
Run:
```bash
find tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux -path '*lib/modules*/updates/*.ko' | sort
```
Expect: `pl-reg.ko`, `pl-irq.ko`, and `u-dma-buf.ko` all present under `.../updates/`.
Result: `[PASS]` 2026-09-03 -- all three present under `lib/modules/6.6.40-xilinx-.../updates/`.

**1.2.b -- The device tree carries both regions and both udmabuf nodes (host).**
Run: decompile the built dtb and grep the nodes --
```bash
dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/build/tmp/work/zynq_generic_7z020-xilinx-linux-gnueabi/linux-xlnx/*/recipe-sysroot/boot/devicetree/system-top.dtb 2>/dev/null | grep -A6 'shim_dma\|udmabuf'
```
Expect: `shim_dma_desc@30000000` reg `<0x30000000 0x400000>`, `shim_dma_data@30400000` reg `<0x30400000 0x4000000>`, `udmabuf0` -> data region (size `0x4000000`), `udmabuf1` -> desc region (size `0x400000`), memory-region phandles matching.
Result: `[PASS]` 2026-09-03 -- addresses, sizes, and phandles all as intended; `SHIM_DMA_*` macros expanded correctly.

**1.2.c -- The modules load at boot (target).**
Run:
```bash
lsmod | grep -E 'pl_reg|pl_irq|u_dma_buf'
```
Expect: all three listed (loaded names use underscores).
Result: `[PASS]` 2026-09-04 -- `pl_irq`, `pl_reg`, `u_dma_buf` all loaded.

**1.2.d -- u-dma-buf probed both regions without the CMA-alignment failure (target).**
Run:
```bash
dmesg | grep -iE 'u-dma-buf|udmabuf|reserved_mem'
```
Expect: udmabuf0 and udmabuf1 register cleanly, with no `of_reserved_mem_device_init failed return=-22` (the alignment gotcha the dtsi comment calls out).
Result: `[PASS]` 2026-09-04 -- both nodes registered (driver 5.2.0); udmabuf0 phys `0x30400000` size `67108864`, udmabuf1 phys `0x30000000` size `4194304`; no probe error.

**1.2.e -- The DMA device nodes appear with the intended geometry (target).**
Run:
```bash
ls -l /dev/udmabuf0 /dev/udmabuf1
for d in udmabuf0 udmabuf1; do echo "$d:"; cat /sys/class/u-dma-buf/$d/phys_addr /sys/class/u-dma-buf/$d/size; done
```
Expect: `udmabuf0` (data) at `0x30400000`, 64 MB; `udmabuf1` (desc) at `0x30000000`, 4 MB (the sizes of the time; since raised to 128 MB and 8 MB).
Result: `[PASS]` 2026-09-04 -- both present as intended, at mode 0600 until step 5's boot-script `chmod`.

**1.2.f -- The reserved regions took the DDR out of the kernel's hands (target).**
Run:
```bash
dmesg | grep -i 'reserved mem'
cat /proc/iomem | grep -iE 'reserved'   # note: reusable CMA regions do not appear here
```
Expect: dmesg shows both regions initialized `map reusable` at `0x30000000..` (desc) and `0x30400000..` (data). Reusable CMA regions don't appear as reserved in `/proc/iomem`, so dmesg is the authoritative check.
Result: `[PASS]` 2026-09-04 -- both regions initialized at the intended ranges, separate from the global 16 MiB CMA pool.

**1.2.g -- The existing SPI datapath is unchanged (target).**
Run: the usual DAC-out / ADC-in bring-up through `shim-test` (still on `/dev/mem` at this step).
Expect: identical behavior to a pre-port build.
Result: `[PASS]` 2026-09-04 -- datapath behaves as before the port.

Note: `pl-reg` and `pl-irq` load here but bind nothing until steps 3 and 4.

### Step 3 -- Register windows onto pl-reg-shim (map_memory.c migration)

`pl-reg-shim` binds this project's four register cores and publishes each window as a short-named `/dev` node; the shim programs open those nodes instead of `/dev/mem`.

**1.3.a -- Each register window appears as a short-named /dev node (target).**
Run:
```bash
ls -l /dev/sys_ctrl /dev/sys_sts /dev/spi_clk /dev/trig_fifo /dev/board_fifo_*
dmesg | grep pl-reg-shim
```
Expect: every node present at mode 0666, with one `pl-reg-shim` `ready` line per bound window and one FIFO node per built board.
Result: `[PASS]` 2026-09-04 -- all 12 windows bound at 0666 on the 4-board bitstream (per-board FIFO bridge nodes at the time; Stage 5.2 replaced them with one `/dev/board_fifo`).

**1.3.b -- The tools run fully non-root (target).**
Run: as a normal user (no sudo), run a read-only tool that maps the windows, e.g.
```bash
status
```
Expect: it runs without a permission error and without touching `/dev/mem`; a wrong or missing node fails loudly at `open()`.
Result: `[PASS]` 2026-09-04 -- `status` ran as a normal user, no sudo, no permission error.

**1.3.c -- Register reads are sensible and match the pre-port build (target).**
Run:
```bash
status
```
Expect: a valid state code, the SPI clock near its configured frequency, and sane FIFO status, matching a `/dev/mem` build.
Result: `[PASS]` 2026-09-04 -- state `Idle`, SPI clock 30.000 MHz, boards 0-3 present and 4-7 absent.

Note: tools that mapped per-board nodes failed at `open()` for absent boards on this partial-board bitstream, which led to the board-count follow-up (`DMA_PLAN.md` Stage 1).

### Step 4 -- The hw_manager interrupt onto pl-irq-shim (sys_sts.c migration)

The `hw_manager_irq` dtsi node is retagged `zynq-toolbox,pl-irq-shim`, `sys_sts.c`'s monitor opens `/dev/hw_manager_irq` instead of `/dev/uio0`, and the dead `uio_pdrv_genirq.of_id` bootarg is dropped from the kernel command line.

**1.4.a -- The interrupt node is present, non-root, and /dev/uio0 is gone (target).**
Run:
```bash
ls -l /dev/hw_manager_irq
dmesg | grep pl-irq-shim
ls -l /dev/uio0    # expect: No such file or directory
```
Expect: `/dev/hw_manager_irq` at 0666 and bound to `pl-irq-shim`; `/dev/uio0` gone.
Result: `[PASS]` 2026-09-04 -- as expected (irq 49).

**1.4.b -- The interrupt is delivered non-root and wakes the monitor (target).**
Run: as a normal user, a workflow that arms `hw_manager` and causes an interrupt.
Expect: the interrupt and decoded status are reported once per event, with no permission error and no interrupt storm.
Result: `[PASS]` 2026-10-28 -- covered by later stages: `dma_irq_test` took its completion on `/dev/hw_manager_irq` as a normal user (3.0.c), and every DMA `waveform` run waits on it through the run-controller (Stage 3.2), which replaced the observe-only monitor.

**1.4.c -- Re-arm works and there is no interrupt storm (target).**
Expect: after handling, `write(fd, 1)` re-enables delivery, and the line doesn't refire until the source changes -- the loop blocks rather than busy-looping.
Result: `[ ]`

### Step 5 -- Boot-script permissions on the udmabuf nodes

`boot_script.sh` (installed as an `/etc/init.d` service) `chmod 0666`s `/dev/udmabuf*` and the `u-dma-buf` `sync_*` sysfs controls, which come up root-owned (`/dev` 0600, sysfs 0664) with no mode knob.

**1.5.a -- The udmabuf nodes and their sync controls are world-accessible (target).**
Run:
```bash
ls -l /dev/udmabuf0 /dev/udmabuf1
ls -l /sys/class/u-dma-buf/udmabuf0/sync_for_device /sys/class/u-dma-buf/udmabuf0/sync_for_cpu
test -w /sys/class/u-dma-buf/udmabuf1/sync_for_device && echo "udmabuf1 sync writable"
```
Expect: `/dev/udmabuf*` and the `sync_*` files at 0666 (without the boot script they're 0600 / 0664).
Result: `[PASS]` 2026-09-04 -- all at 0666.

**1.5.b -- A non-root program can map and sync a buffer end to end (target).**
Expect: as a normal user, open, `mmap`, write, sync and read back `/dev/udmabuf0` with no error.
Result: `[PASS]` 2026-10-28 -- every DMA run maps both regions, writes the DAC stream and descriptors, and reads the capture back as the normal `shim` user. The mover maps the regions non-cached (`O_SYNC`), so it never needs the `sync_*` controls.

### Stage 1 exit

**1.X -- Non-root, plumbing present, datapath intact.**
Expect: every tool runs non-root; the `/dev/udmabuf*` and `pl-reg`/`pl-irq` nodes are present and correctly owned; the reserved regions read back as intended; a full datapath run is unchanged.
Result: `[PASS]` 2026-10-28 -- 1.2.a-g, 1.3.a-c, 1.4.a-b and 1.5.a-b pass, and every tool runs as the normal `shim` user. 1.4.c is folded into the Stage 3.2 wait check.

---

## Stage 2 -- PL datapath (MCDMA plus routing), PIO path preserved

Stage 2 is built in sub-steps, each leaving a working tree: 2a lands the MCDMA engine, the 64-bit HP0 path and the non-root control window, with the streams wired as a per-channel loopback; 2b rewires the real SPI datapath onto it; 2c folds the interrupts into `hw_manager`.

### Stage 2a -- MCDMA engine, HP0, control window (loopback bring-up)

**2a.1 -- The block design builds with the MCDMA, HP0, and control window (host).**
Run (from the built project under `tmp/snickerdoodle_black/1.0/rev_d_shim`):
```bash
# MCDMA parameters
find . -path '*system_mcdma_0*' -name '*.xci' | head -1 | xargs grep -iE 'c_num_mm2s_channels|c_num_s2mm_channels|c_sg_length_width|c_m_axi_(mm2s|s2mm)_data_width'
# packet-atomic mux + its TDEST window; per-channel demux routing
find . -path '*system_s2mm_mux_0*'  -name '*.xci' | head -1 | xargs grep -iE 'HAS_TLAST|ARB_ON_TLAST|ARB_ON_MAX_XFERS|M00_AXIS_(BASE|HIGH)TDEST'
find . -path '*system_mm2s_demux_0*' -name '*.xci' | head -1 | xargs grep -iE 'M0[0-3]_AXIS_(BASE|HIGH)TDEST'
# HP0 enabled on the PS
find . -path '*system_ps_0*' -name '*.xci' | head -1 | xargs grep -i 'PCW_USE_S_AXI_HP0'
```
Expect: MCDMA 4 MM2S + 4 S2MM (== `board_count`), 23-bit SG length, 64-bit data; `s2mm_mux` with `HAS_TLAST=1`, `ARB_ON_TLAST=1`, `ARB_ON_MAX_XFERS=1024` and an MI window of `[0x0, 0x3]`; `mm2s_demux` routing M00..M03 to TDEST 0..3; `PCW_USE_S_AXI_HP0=1`.
Result: `[PASS]` 2026-09-08 -- all as intended. 33,410 LUT (~63%) at 4 boards, of which the DMA infrastructure is ~11k (mcdma 6,156, HP0 SmartConnect 4,402).

**2a.2 -- The MCDMA node is retagged for pl-reg-shim in the device tree (host).**
Run:
```bash
dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/images/linux/system.dtb 2>/dev/null | grep -iB1 -A5 'axi_mcdma@40400000'
```
Expect: the node carries `compatible = "zynq-toolbox,mcdma-userspace"` (keeping the in-kernel Xilinx driver off it) and keeps its `mcdma` label, so `pl-reg-shim` names it `/dev/mcdma`.
Result: `[PASS]` 2026-09-08 -- as expected, with `reg = <0x40400000 0x10000>`.

**2a.3 -- pl-reg-shim binds the MCDMA control window non-root (target).**
Run:
```bash
dmesg | grep -i 'mcdma'
ls -l /dev/mcdma
```
Expect: `pl-reg-shim` reports `/dev/mcdma ready (mode 0666) ... compatible "zynq-toolbox,mcdma-userspace"`; `/dev/mcdma` is present at mode 0666 (world-rw, no root).
Result: `[PASS]` 2026-10-28 -- every DMA run maps the control window as the normal `shim` user. The mover's only other path is a root-only `/dev/mem` fallback, which would fail for that user.

**2a.4 -- Byte-exact MCDMA loopback round-trip, non-root (target).**
Run:
```bash
mcdma-loopback
mcdma-loopback 1        # a single non-zero channel alone, to isolate the mux
```
Expect: every channel round-trips 2048 bytes with no mismatch, non-root.
Result: `[N/A]` -- the loopback was scaffolding, replaced by the real datapath in 2b before a board was available; the Stage 3.0 byte-exact checks cover it.

**2a.5 -- The existing SPI datapath is unchanged (target).**
Run:
```bash
status
```
Expect: same output as a pre-2a build; the MCDMA on GP0/HP0 doesn't disturb the PIO path.
Result: `[PASS]` 2026-10-28 -- `status` reads Idle, the configured SPI clock, and boards 0-3 present / 4-7 absent, and the PIO tools (`shim-test`, `channel-check`) work on every board.

**On-target bring-up.** Stage 2b replaced the loopback before a board was available, so 2a.4 never ran; the real datapath's byte-exact tests supersede it. 2a.3 and 2a.5 fold into the 2b bring-up, which needs the Stage 3 mover, so on-target validation rolls into Stage 3.

### Stage 2b -- real per-board datapath (host + core tests)

A TDEST demux feeds each board's DAC command FIFO through a write-only `axis_fifo_bridge`, each board's ADC data FIFO drains through the new `adc_packetizer` into a packet-atomic TDEST mux, and a per-board `datapath_mux` hard-locks the shared FIFO ports to PIO or DMA under `datapath_mode[i]` (reset PIO), all inside `axi_spi_interface`.

**2b.1 -- The new and changed datapath cores pass their cocotb tests (host).**
Run (in the cocotb container, no board -- in Docker mode go through make so the script runs inside the container):
```bash
make projects/rev_d_shim/cores/shim/adc_packetizer/tests/test_status PROJECT=rev_d_shim
make projects/rev_d_shim/cores/shim/datapath_mux/tests/test_status PROJECT=rev_d_shim
# VM mode (cocotb on the host) can call the script directly instead:
#   ./scripts/make/test_core.sh rev_d_shim shim adc_packetizer
#   ./scripts/make/test_core.sh rev_d_shim shim datapath_mux
```
Expect: `adc_packetizer` passes its 5 framing tests (reset, cap+tail, mixed 1-/4-word reads, single-word drip, backpressure); `datapath_mux` passes its select/gating tests.
Result: `[PASS]` -- `adc_packetizer` (2026-10-15) and `datapath_mux` (2026-10-06) pass in `make tests`.

**2b.2 -- The block design builds with the real datapath (host).**
Run (from the built project under `tmp/snickerdoodle_black/1.0/rev_d_shim`), and confirm from the block design / synthesized netlist:
```bash
# per-board DMA adapters + select are present; the loopback FIFOs are gone
grep -c 'datapath_mux_\|adc_data_packetizer_\|dac_cmd_dma_bridge_' <bd or netlist>
grep -c 'dma_loop_fifo_' <bd or netlist>   # expect 0
```
Expect: per board a `dac_cmd_dma_bridge_<b>`, an `adc_data_packetizer_<b>` (`MAX_PACKET_WORDS` at or under the mux backstop) and a `datapath_mux_<b>`; no `dma_loop_fifo_*`; the design still fits.
Result: `[PASS]` 2026-09-15 (4-board build) -- per board, packetizer 17 LUT, DMA bridge 2 LUT, `datapath_mux` 51 LUT; no loopback FIFOs. 33,859 LUT (63.6%), only +449 over Stage 2a.

### Stage 2c -- mode_viol fault and MCDMA interrupt fold

The PIO `axi_fifo_bridge` (later the board FIFO array) responds `OKAY` on every access, raising `fifo_overflow` / `fifo_underflow` on a genuine full/empty and `mode_viol` when the port belongs to the DMA side. The per-board `mode_viol` mask folds into `hw_manager` as `STS_MODE_VIOL`, and the MCDMA `introut` lines signal `hw_manager`'s single `ps_interrupt` during a run without leaving `S_RUNNING`.

**2c.1 -- The hw_manager cocotb tests pass, including mode_viol and the DMA doorbell (host).**
Run (in the cocotb container, no board):
```bash
make projects/rev_d_shim/cores/shim/hw_manager/tests/test_status PROJECT=rev_d_shim
# VM mode: ./scripts/make/test_core.sh rev_d_shim shim hw_manager
```
Expect: all pass, including `mode_viol` at every board position and `test_running_dma_introut_doorbell` (one `ps_interrupt` pulse per `introut` event, staying in `S_RUNNING`).
Result: `[PASS]` 2026-09-21 -- 59/59.

**2c.2 -- The block design builds with mode_viol and the introut fold wired (host).**
Run (from the built project under `tmp/snickerdoodle_black/1.0/rev_d_shim`), and confirm from the block design / synthesized netlist:
```bash
# mode_viol aggregation into hw_manager and the two introut concats are present
grep -c 'mode_viol\|dma_mm2s_introut_concat\|dma_s2mm_introut_concat' <bd or netlist>
```
Expect: `mode_viol` wired into `hw_manager`, and the MCDMA `introut` lines concatenated (zero-padded for unused boards) into `hw_manager`'s two `introut` buses.
Result: `[PASS]` 2026-09-21 -- full build completed cleanly, with no unconnected-input criticals.

### Stage 2b/2c acceptance criteria

From the plan:

- Byte-exact transfer on the DMA-driven FIFOs.
- The PIO path still works with a board's `datapath_mode` set to PIO.
- Per-board mode selection: one board on DMA while the rest stay on PIO.
- Wrong-mode access returns `OKAY`, discards, and raises `mode_viol` into the normal coordinated shutdown, not a `SIGBUS`.
- ADC packetizer framing: `tlast` on FIFO-dry or the `MAX_PACKET_WORDS` cap, on any word; an empty board releases the mux; the run tail self-flushes; packets never exceed `ARB_ON_MAX_XFERS`.
- The MCDMA completion/error lines ride `hw_manager`'s single interrupt.

The byte-exact transfer, the on-target `mode_viol` fault, and the interrupt-driven completion pass on target (3.0.a-d, 2026-09-22), along with the host checks. Multi-board mode independence and the packetizer under a real waveform land with Stage 3.

**Stage 2 is complete, host- and hardware-validated.**

---

## Stage 3 -- Software

### Stage 3.0 -- low-level DMA bring-up (shim-test)

The `shim-test` DMA commands (`dma_mode`, `dma_status`, `dma_channel_test`, `dma_irq_test`, `dma_mode_viol`) drive the datapath directly. `dma_channel_test` DMAs a `DAC_WR_CH` through MM2S, arms S2MM, issues a PIO `ADC_RD_CH` on the same channel, and captures the sample back through S2MM; `dma_irq_test` takes the completion on the `hw_manager` interrupt instead of polling; `dma_mode_viol` provokes a wrong-mode access. DMA testing runs at 10 MHz SPI, since ADC data misaligns above that on current hardware.

**3.0.a -- Byte-exact DAC delivery over MM2S (target).**
Run (`dma_mode 0 1` while off, then `ctrl_on`/`pow_on`):
```bash
dma_channel_test 0 1000
dac_last_received_cmd 0
```
Expect: the DAC's last received command is `0x600003e8` (`DAC_WR_CH` channel 0, value 1000) -- the exact bytes crossed DDR -> MM2S -> demux -> DAC.
Result: `[PASS]` 2026-09-17.

**3.0.b -- Full DMA round-trip through the coil (target).**
Run:
```bash
channel_test 0 1000        # PIO reference first (system off, dma_mode 0 0)
# then dma_mode 0 1 / ctrl_on / pow_on:
dma_channel_test 0 1000
```
Expect: no descriptor timeout, one ADC word captured through S2MM, matching the PIO reference within noise.
Result: `[PASS]` 2026-09-17 (10 MHz) -- PIO read 996, DMA read 994, proving MM2S -> DAC -> coil -> ADC -> S2MM -> DDR end to end. It needed two fixes: the non-cached `u-dma-buf` mapping (`O_SYNC`) for coherence, and arming S2MM before the ADC read so the packetizer's continuous drain isn't dropped toward an idle S2MM.

**3.0.c -- Interrupt-driven S2MM completion via the folded doorbell (target, Stage 2c).**
Run (`dma_mode 0 1` while off, then `ctrl_on`/`pow_on`):
```bash
dma_irq_test 0 1000
```
Expect: the same round-trip, with completion delivered by the `hw_manager` interrupt ("Completion delivered by the hw_manager interrupt"). If no interrupt is seen, compare `cat /proc/interrupts | grep pl-irq` before and after: an increment points at the software wake, none at the fold or the MCDMA interrupt enables.
Result: `[PASS]` 2026-09-22 (10 MHz) -- completion arrived on the interrupt and captured 996; the `pl-irq` count went 1 -> 3 (edge-triggered; the tool woke once and found the completion), proving MCDMA `introut` -> `hw_manager` -> `ps_interrupt` -> `/dev/hw_manager_irq`.

**3.0.d -- Wrong-mode access is graceful, not a crash (target, Stage 2c).**
Run (board 0 in DMA mode and running):
```bash
dma_mode_viol 0
```
Expect: the PIO write returns without a `SIGBUS` and `hw_manager` halts with `STS_MODE_VIOL` for board 0; the command prints `PASS`.
Result: `[PASS]` 2026-09-22 -- as expected. The fault latch clears only on a buffer or global reset, not `off`: after `off` / `ctrl_on` / `pow_on` it re-faulted until `hard_reset` pulsed `buf_reset` -- which fed the Stage 4 teardown design.

**Stage 3.0 is complete on hardware (3.0.a-d PASS).**

### Stage 3.1 -- shared MCDMA mover library (multi-word)

The mover (`src/sys/dma_ctrl.c`) is the shared library `waveform` and `static-shims` use through their `src/sys` symlink. On top of the single-word bring-up calls it provides the multi-word run primitives: `dma_wave_arm` lays out a prebuffered DAC stream and an ADC capture ring in the `u-dma-buf` regions and starts both engines without touching the trigger, `dma_wave_avail` / `dma_wave_read` drain the capture behind a read cursor, and `dma_wave_read_total` / `dma_wave_expected` / `dma_wave_disarm` round out the lifecycle. The mover only knows words and FIFO/ring state; triggers, polling, file output and waveform synthesis live in the command layer.

`shim-test` exercises it with `dma_waveform_test`, which synthesizes an 8-channel triangle-times-envelope sequence at the matched DAC/ADC cadence, writes the intended waveform (`<out>.dac_in_A.csv`, amps) and ADC schedule (`<out>.adc_in_t_sec.csv`, the format `waveform --adc` replays), prebuffers the DAC stream, and starts a background collector that streams the readback to `<out>.shim-test_adc_out_A.csv` with no timeout. `dma_waveform_status` reports progress and `dma_waveform_stop` ends a run early.

Facts pinned down here that carry into the run programs:

- The MCDMA soft-reset (control bit 2) is global across MM2S and S2MM: reset both up front, then arm each, or a later reset wipes the armed channel.
- A multi-channel `DAC_WR` latches its outputs only on an LDAC pulse, so each must set LDAC or no current flows. `DAC_WR_CH` uses the immediate write-and-update command, which is why single-word bring-up worked without it.
- Trigger alignment lives in the command stream: the first `DAC_WR` is a trigger-wait, and the ADC stream leads with a trigger-wait `NO_OP`, since an ADC delay is post-read.
- An eight-channel read emits exactly four words, and `ADC_RD` with `REPEAT = R` performs `1 + R` reads, so a capture is `4 * n_reads` words.
- Matched cadence uses `delay = max(dac_min_delay, adc_min_delay)` from the status register, so the capture lines up with the command-stream index.
- S2MM capture in this stage used one one-word descriptor per captured word; capture packing replaces it with per-packet descriptors (see `DMA_CAPTURE_PACKING.md`).

Checks:
- `[PASS]` 2026-09-22 -- `dma_ctrl.c` compiles and links into `waveform` and `static-shims`.
- `[PASS]` 2026-09-22 (4-board, 10 MHz) -- a 128 KB DAC stream (6553 `DAC_WR` updates) played on the trigger with current on the supply, and the matched 6600-read capture came back with the expected triangle-times-envelope shape (full amplitude ch0-3, half ch4-7). The LDAC-latch and global-reset facts above were what closed it.
- `[PASS]` 2026-09-22 -- sizes come from sysfs and an oversized run is rejected with a clear message (a 512 KB DAC run needed 6.7 MB of descriptors against the 4 MB `udmabuf1` of the time).

The per-word ring made the descriptor region the capacity limit (about 256 KB of capture in the 4 MB region of the time); capture packing and the 8 MB / 128 MB regions resolve it.

### Stage 3.2 -- event-driven run-controller

A shared run-controller (`src/sys/run_ctrl.c`: `run_ctrl_create` / `_wait` / `_request_abort` / `_destroy`) replaces the observe-only interrupt monitor. It blocks in `poll()` over `/dev/hw_manager_irq` plus an `eventfd`, so a hardware fault and a local abort (SIGINT or a software error) unblock the same wait, and re-arms the interrupt each cycle with no spin-polling. `run_ctrl_wait` returns `RUN_CTRL_FAULT`, `RUN_CTRL_ABORT`, `RUN_CTRL_EVENT` (a benign doorbell) or `RUN_CTRL_TIMEOUT`; the caller loops it, draining captures each wake. Normal end is a software fact (all expected capture words read), and a fault drives the coordinated shutdown (`request_stop` plus `hw_power_off`).

Checks:
- `[ ]` During a prebuffered run the controller blocks on one wait (no busy-polling of the status register); confirmed by CPU usage and by the `pl-irq` count advancing only on real events.
- `[ ]` A hardware fault (e.g. a `dma_mode_viol`-style violation, or an injected over/underflow) wakes the wait, and the controller reads `STS_*` and runs the coordinated shutdown.
- `[PASS]` 2026-10-21 -- A local abort (SIGINT folded into the `eventfd`) unblocks the same wait and stops the run cleanly: a 100-iteration DMA run aborted with Ctrl-C after the triggers powered off cleanly (5.1.g).
- `[PASS]` 2026-10-28 -- The normal end of a run wakes the wait and proceeds to readback: every completed DMA run reached its trigger count and wrote all its samples (5.2.g, 5.4.c).
- `[PASS]` 2026-10-01 (host) -- `run_ctrl.c` compiles clean and links into `shim-test`, `waveform`, and `static-shims`.

### Stage 3.3 -- waveform DMA integration

Wire the mover and the run-controller into the run programs. For a run on DMA-mode boards: prebuffer each board's `dac_cmd` sequence and reserve its `adc_data` capture region, set the per-board `datapath_mode` to DMA (while the system is off), arm the S2MM capture channels, release the trigger, wait on the run-controller for run-end-or-fault, then on a normal end read back the `adc_data` -- byte-exact against expectation. The PIO path stays the fallback for boards left in PIO mode.

`waveform` runs DMA by default (`--pio` selects PIO). Before power-on it sets `datapath_mode` to DMA for every active board, synthesizes each board's DAC stream with `waveform_build_dac_dma` (byte-identical to the PIO feed apart from the channel data), and prebuffers all boards at once (`dma_wave_begin` / `dma_wave_add`, a per-board run in a region bump-allocator). The ADC command lane stays PIO on its existing stream thread, and `adc_data` is captured through S2MM, one ring per board. The run-controller loop releases the triggers, drains every board's capture into one `<stem>.waveform_adc_out_A.DMA.csv` on each wake, and stops on fault/abort or when every board has its `4 * rows * iters` words. A DAC-only run (no `--adc`) has no capture ring and finishes on the trigger count once the DAC FIFOs drain. `static-shims` stays PIO by design, since static setpoints never approach the rate that motivates DMA.

Checks:
- `[ ]` A real waveform plays end to end through the DMA path on at least one board; the `adc_data` readback matches expectation. 2026-10-28: `dma_wave` plays on board 0 and writes all 6600 samples x 8 channels; the captured values haven't been compared against the DAC input yet.
- `[PASS]` 2026-10-28 -- The `shim-test` -> `waveform` round-trip: `dma_waveform_test` generates `<out>.dac_in_A.csv` and `<out>.adc_in_t_sec.csv`, and `waveform <out>.dac_in_A.csv --adc <out>.adc_in_t_sec.csv` validates both and plays them through DMA at the 216-cycle cadence (5.4.c).
- `[ ]` All active boards run the DMA path together in one run, each board's capture correct.
- `[ ]` The ADC packetizer holds under a real multi-word run: the capture reassembles by position with no lost or misframed words. 2026-10-28: word counts are exact (26400 words) with dense packing engaged (backlog 256-512 words); the content check in the first item would close it.
- `[ ]` A DAC-only DMA run (no `--adc`) drives current and finishes cleanly on the trigger count.
- `[ ]` A fault during the run is caught by the run-controller, which runs the coordinated shutdown; Ctrl-C aborts the same way. The Ctrl-C half passes (5.1.g); a mid-run hardware fault hasn't been tried.
- `[ ]` The PIO fallback produces the same result for a `--pio` run. `dma_wave` is too fast for PIO (5.2.f), so this needs a slower waveform.
- `[PASS]` 2026-10-01 (host) -- `waveform.c`, `waveform_build_dac_dma`, and the extended mover compile clean, including the DAC-only (`cap_words == 0`) path.
- `[PASS]` 2026-10-06 (host) -- timing round-trip fix: `shim-test` writes the DAC CSV at nanosecond precision, and `waveform` validates timing in SPI cycles with the same round-to-nearest conversion as the command synthesis, so a round-trip recovers the exact cadence (216 cycles at 10 MHz) instead of truncating to 215.

---

## Stage 4 -- Fault and reset coordination

The DMA run teardown is a coordinated sequence rather than a bare power-off. On every `waveform` DMA run exit -- completion, fault, or abort -- `dma_wave_halt` soft-resets both MCDMA directions (stopping every channel and draining S2MM), then `hw_reset_dma_buffers` pulses `buf_reset` on the active boards' `dac_cmd` and `adc_data` FIFOs, dropping stranded DAC commands and clearing latched lane faults (`mode_viol`, over/underflow). The rings reinit on the next `dma_wave_begin`. It's all software on the existing `sys_ctrl` masks, so only a rootfs rebuild is needed. As in ex05, the S2MM soft-reset drains downstream, so `buf_reset` is needed only for data stranded upstream in the DAC FIFO. No PS-side timeout is added, since waiting arbitrarily long on a trigger is normal. In `shim-test`, latched-fault recovery is still `hard_reset` (3.0.d).

Checks:
- `[ ]` A clean halt: on a fault (or Ctrl-C) during a DMA run, the MCDMA halts, `buf_reset` clears the DMA FIFOs, and the tool powers off with MM2S/S2MM `SR` showing halted. The Ctrl-C path powers off and the next run starts clean (5.1.g); the `SR` bits and the fault path haven't been checked.
- `[ ]` A recoverable restart after an injected fault: the next DMA run starts clean, with no residue and no re-fault from a still-latched lane fault. So far only after a Ctrl-C abort and a halted `--pio` run (5.1.g), not after a fault in a DMA run.
- `[ ]` No false faults: a run whose triggers are withheld waits indefinitely without declaring a fault or timing out.
- `[PASS]` 2026-10-01 (host) -- `dma_wave_halt`, `hw_reset_dma_buffers`, and the teardown in `run_dma_path` compile clean.

---

## Stage 5 -- Validation and reclaim

Acceptance criteria from the plan:

- The DMA-driven CDC FIFOs shrink from whole-waveform depth toward microsecond elastic buffers, freeing BRAM, with the datapath still byte-exact.
- LUT stays within budget at eight boards (the binding constraint) -- check the post-synth hierarchical utilization report.
- The ADC debug words move onto a dedicated debug lane and the DAC debug words onto `dac_debug`, with `GET_CAL` readback on its own small `dac_data` lane; with debug enabled, the `adc_data` lane carries only samples and the packetizer framing still holds.
- `DMA_PLAN.md` updated to reflect the integrated state.

### Stage 5.1 -- Buffer-reset synchronizers (`reset_sync`)

Each per-board command/data FIFO side and the two trigger FIFO sides had its own `proc_sys_reset` (68 at eight boards, ~21 LUT / 27 FF each). They're replaced one for one by `base:user:reset_sync`, which asserts asynchronously and releases two clock edges after the bit clears, keeping instance names and the command-FIFO chaining. Assertion no longer waits for a clock edge, so a reset lands even with `spi_clk` stopped. Software is untouched (it holds each reset for 1 ms). See `RESOURCE_RECLAIM.md` step 5.

**5.1.a -- The `reset_sync` core passes its cocotb tests (host).**
Run: `make projects/rev_d_shim/cores/base/reset_sync/tests/test_status PROJECT=rev_d_shim` (or `make tests PROJECT=rev_d_shim`).
Expect: all four tests pass -- release after exactly `DEPTH` edges, assertion with the clock parked, held reset ignores clock edges, and a sub-period pulse resets the whole chain.
Result: `[PASS]` 2026-10-21 -- all four tests pass (`make tests`).

**5.1.b -- The block design builds cleanly with the synchronizers (host).**
Run: `make` (or `make xpr` then `make bit`), then check the Vivado log for block-design validation warnings on the `*_rst` cells.
Expect: the project builds; no new critical warnings about reset polarity or clock association on `reset_sync` pins; `axi_spi_interface` holds no `proc_sys_reset` (`grep -c proc_sys_reset` on its hierarchy in the utilization report returns 0).
Result: `[PASS]` 2026-10-21 (four and eight boards) -- builds through bitstream and PetaLinux with no warnings on `reset_sync` and no `proc_sys_reset` under `axi_spi_interface`; the only critical warning was the hold failure in 5.1.d.

**5.1.c -- The LUT/FF saving is as estimated (host).**
Run: read `tmp_reports/snickerdoodle_black/1.0/rev_d_shim/hierarchical_utilization.txt` (eight boards).
Expect: total LUT down about 1,400 from 48,755 and FF down about 1,700 from 42,404, almost all of it in `axi_spi_interface` (5,800 LUT before). Record the numbers in `RESOURCE_RECLAIM.md` step 5.
Result: `[PASS]` 2026-10-21 (eight boards, with 5.2) -- the 68 `reset_sync` instances plus 34 inverters total 102 LUT / 136 FF against ~1,430 / ~1,850 for the `proc_sys_reset` blocks. Combined with 5.2: 46,535 LUT, 39,948 FF.

**5.1.d -- No new timing problems on the reset paths (host, needs `make bit`).**
Run: open the implemented timing summary (or `report_timing_summary` in the GUI).
Expect: timing met as before; no recovery/removal violations on the `*_rst/sync` flops or the FIFO resets they drive.
Result: `[PASS]` 2026-10-21 (four boards) -- every recovery/removal check met. The first build failed hold in the SPI domain (WHS -0.179 ns, 43 endpoints) on short flop -> BRAM and flop -> flop paths with 0.22-0.35 ns of leaf-clock skew (mostly across the Y99/Y100 clock-region boundary); the router's own estimate showed positive hold, so its first pass never fixed them. An aggressive post-place hold-fix `phys_opt_design` only reached -0.114 ns and cost setup slack, so it isn't used. A second `route_design -directive Explore` pass closes hold, and `scripts/vivado/route_hold_retry.tcl` runs it as a post-route hook (all projects) only when WHS is negative. Final: WNS +0.353 ns, WHS +0.051 ns. The final timing is in `system_wrapper_timing_summary_postroute_physopted.rpt`; the `_routed.rpt` predates the hook.

**5.1.e -- Buffer resets still clear every FIFO (target).**
Run: power on with `shim-test`, push a few words into each FIFO (e.g. a DAC command with a long delay, an ADC read), run the buffer resets (`hard_reset`, or `waveform`'s `hw_clear_*` path), then `status`.
Expect: every DAC/ADC command/data FIFO and both trigger FIFOs read empty afterwards, and the hardware is not halted.
Result: `[PASS]` 2026-10-21 (four boards) -- with two words held in each of five DAC/ADC FIFOs and three long `trig_delay` commands in the trigger command FIFO, `hard_reset` emptied all of them and left the state Idle. (`trig_expect_ext` is the more reliable way to hold trigger words, since delays drain on their own.)

**5.1.f -- Latched bridge faults still clear on a buffer reset (target).**
Run: `shim-test` `dma_mode_viol` (or any wrong-mode poke) to latch a `mode_viol`, then a buffer reset (`hard_reset`), then power on again.
Expect: the second power-on does not re-fault -- the sticky flags, which clear on the FIFO's `reset_sync` outputs, cleared with the FIFO.
Result: `[PASS]` 2026-10-21 (four boards) -- with board 2 in DMA mode, `dma_mode_viol 18` halted the system with "Datapath-mode violation" on board 2; after `hard_reset` and `dma_mode 2 0`, `ctrl_on` / `pow_on` came up Running with no re-fault.

**5.1.g -- Back-to-back `waveform` runs start clean (target).**
Run: two DMA runs with `--adc` (the default datapath), then one `--pio` run, back to back; then a DMA run interrupted with Ctrl-C, followed by another DMA run.
Expect: every run completes with correct output; each DMA run's printed peak backlog starts from zero (the `adc_rate_ctrl` backlog clears on the `adc_data` `reset_sync` output); the run after the abort shows no stranded DAC commands and no bad-command fault.
Result: `[PASS]` 2026-10-21 (four boards) -- two DMA runs and a `--pio` run back to back (the PIO run halts on ADC overflow at this cadence, 5.2.f), then the next DMA run armed normally. A 100-iteration DMA run aborted with Ctrl-C after the triggers powered off cleanly, and the following run wrote all 6600 samples with no fault. Peak backlogs were 256, 32 and 512 words, varying with readout latency rather than carrying over.

### Stage 5.2 -- Board FIFO array (`axi_board_fifo_array`)

The eight per-board `axi_board_fifo_bridge` instances, each on its own register SmartConnect port, become one `axi_board_fifo_array` on a single port (14 -> 7 master ports). Lane behavior is unchanged -- always ready, always `OKAY`, wrong-mode access discarded with `mode_viol`, sticky over/underflow flags clearing on their FIFO's reset -- but the map is one 32 KB window at `0x41000000`, a 4 KB page per board, one word per lane. `pl-reg-shim` publishes it as `/dev/board_fifo`, and `board_count()` counts boards from the `sys_sts` FIFO-present bits. See `RESOURCE_RECLAIM.md` step 6 and `cores/base/axi_board_fifo_array/README.md`.

**5.2.a -- The array core passes its cocotb tests (host).**
Run: `make tests PROJECT=rev_d_shim` (or the core's `test_status` target).
Expect: all six tests pass at `NUM_BOARDS` 6 -- per-board write and read routing, unmapped and cross-direction lanes, absent boards, per-board `mode_block` / `mode_viol`, and per-FIFO sticky flags clearing on their own reset.
Result: `[PASS]` 2026-10-21 -- all six tests pass (`make tests`).

**5.2.b -- The block design builds, and the device tree carries one array node (host).**
Run: `make`; then `dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/images/linux/system.dtb 2>/dev/null | grep -B2 -A4 board_fifo`.
Expect: the build completes with no new critical warnings; one node with `compatible = "xlnx,axi-board-fifo-array-1.0"` and `reg = <0x41000000 0x8000>`, and no `axi-board-fifo-bridge` nodes.
Result: `[PASS]` 2026-10-21 -- one `axi_board_fifo_array@41000000` node with the array compatible and `reg = <0x41000000 0x8000>`; no bridge nodes.

**5.2.c -- The LUT saving is as estimated (host).**
Run: read the hierarchical utilization report.
Expect: about 550-650 LUT below step 5, split between `sys_cfg_axi_intercon` (seven fewer master ports) and `axi_spi_interface` (eight bridges -> one array). Record in `RESOURCE_RECLAIM.md` step 6.
Result: `[PASS]` 2026-10-21 (eight boards, built together with Stage 5.1) -- `sys_cfg_axi_intercon` 1,902 -> 1,144 LUT, `axi_board_fifo_array` 295 LUT against 384 for the eight bridges: about -870 LUT, more than estimated.

**5.2.d -- One `/dev/board_fifo` node, non-root (target).**
Run: `ls -l /dev/board_fifo /dev/board_fifo_* 2>&1; dmesg | grep board_fifo`
Expect: `/dev/board_fifo` at `crw-rw-rw-`, bound to `xlnx,axi-board-fifo-array-1.0` with size `0x8000`; no `/dev/board_fifo_<b>` nodes.
Result: `[PASS]` 2026-10-21 (four boards) -- `/dev/board_fifo` is `crw-rw-rw-` and bound to the array (size `0x8000`), with no per-board nodes; `status` reads boards 0-3 present and 4-7 absent through it.

**5.2.e -- Board count still matches the bitstream (target).**
Run: `shim-test` (any board-addressed command with the highest board index and one past it), or `waveform` with a channel count for every board.
Expect: the highest built board is accepted and the next index is rejected cleanly; on the eight-board build all eight boards are accepted.
Result: `[PASS]` 2026-10-21 (four boards) -- `dac_cmd_fifo_sts 3` reads board 3, and `dac_cmd_fifo_sts 4` is rejected with "Must be 0 to 3".

**5.2.f -- PIO lanes reach the right board and FIFO (target).**
Run: `channel-check` across every channel, then a `--pio` `waveform` run with `--adc` over all boards.
Expect: every channel calibrates and reads back on its own board (no cross-board or cross-lane mixups); the PIO capture matches a pre-change run.
Result: `[PASS]` 2026-10-21 (four boards) -- `channel-check 32` calibrates all 32 channels (gains 0.999-1.001; ch 20 flat, `-X-`), and an ADC read and `GET_CAL` on channel 9 land in board 1's lanes. The PIO `waveform` run of `dma_wave` (46.3 kHz) halts on board 0's ADC overflow -- that cadence is beyond what PIO can drain, which is what DMA is for -- so a full PIO capture needs a slower waveform.

**5.2.g -- DMA path and wrong-mode gating unchanged (target).**
Run: a default (DMA) `waveform` run with `--adc`; then `shim-test` `dma_mode_viol` on one board.
Expect: the DMA run plays and captures as before (DMA calibration included); the wrong-mode poke raises `mode_viol` on that board only, halts gracefully, and a buffer reset clears it.
Result: `[PASS]` 2026-10-21 (four boards) -- two back-to-back DMA runs of `dma_wave` (6553 DAC updates, 6600 ADC reads on board 0) calibrated over DMA, played on one trigger and wrote all 6600 samples (peak backlog 256, then 32 words). `dma_mode_viol 18` halted on "Datapath-mode violation" for board 2, and `hard_reset` cleared it.

### Stage 5.3 -- Debug lanes and calibration readback lane

Each board's read lanes split four ways: `adc_data` (samples only), `adc_debug`, `dac_debug`, and a 2^5 `dac_data` (`GET_CAL` readback only). The page becomes `dac_cmd` 0x00, `dac_debug` 0x04, `dac_data` 0x08, `adc_cmd` 0x10, `adc_debug` 0x14, `adc_data` 0x18. Debug FIFOs and `adc_cmd` are 2^12. A full debug FIFO drops the word and sets a sticky flag (status word 89) instead of halting; `adc_debug` status is at words 73-80, `dac_data` at 81-88, and words 18+2b describe `dac_debug`. Software names the lanes `cmd`/`debug`/`data` (`adc_read_data`, `dac_read_debug`, `adc_read_debug`), and `shim-test` gains the debug FIFO status and read commands.

**5.3.a -- Core tests pass (host).**
Run: `make tests PROJECT=rev_d_shim`.
Expect: `axi_board_fifo_array` passes with the six-lane map (per-lane read routing, debug lanes unflagged when empty); `ad5676_dac_ctrl` and `ads816x_adc_ctrl` keep their existing tests and pass the new `test_debug_lane` (debug words only on the debug port, a full debug buffer drops them with `debug_buf_overflow` set and no halt, `GET_CAL` / samples unaffected).
Result: `[PASS]` 2026-10-22 -- `axi_board_fifo_array` 6/6; `ad5676_dac_ctrl` 13 pass + 1 skip (the gated example sim), `ads816x_adc_ctrl` 15 pass + 1 skip, both including `test_debug_lane`.

**5.3.b -- Builds and meets timing; resource cost as estimated (host).**
Run: `make`; read the hierarchical utilization report and `system_wrapper_timing_summary_postroute_physopted.rpt`.
Expect: no new critical warnings; timing met; per board about +80 LUT for `adc_debug` and +60-80 LUT (LUTRAM, no BRAM) for `dac_data`, plus the `dac_debug` / `adc_debug` / `adc_cmd` BRAM growth to 4 RAMB36 each. Record in `RESOURCE_RECLAIM.md`.
Result: `[PASS]` 2026-10-22 (four boards) -- 0 critical warnings, timing met after the hold re-route (WNS +0.233 ns, WHS +0.050 ns). 30,596 LUT (+268), 61 RAMB36 (+40), 8 RAMB18 (+4); `dac_data` maps to a RAMB18 (51 LUT) rather than LUTRAM. Logged as `RESOURCE_RECLAIM.md` step 8.

**5.3.c -- Status and lanes on hardware (target).**
Run: `status`; in `shim-test`, `dac_debug_fifo_sts 1`, `adc_debug_fifo_sts 1`, `dac_data_fifo_sts 1`; `do_dac_get_cal 9` then `read_dac_data 1`.
Expect: `status` lists the cmd, debug and data FIFOs for every board, plus a "debug words dropped: No" line per present board; the calibration readback arrives on `dac_data` only.
Result: `[PASS]` 2026-10-28 -- `status` lists all three FIFOs and the debug-dropped line for each of the four present boards; the `GET_CAL` word lands only in board 1's `dac_data`.

**5.3.d -- Debug output stays off the data lanes (target).**
Run: `set_debug 0xF` (DAC and ADC debug for boards 0-1) before powering on, then `ctrl_on`, `pow_on`, `do_adc_rd_ch 9`, `do_dac_get_cal 9`; then `adc_data_fifo_sts 1`, `read_adc_single 1`, `read_dac_data 1`, `read_adc_dbg 1 --all`, `read_dac_debug 1 --all`.
Expect: `adc_data` holds exactly the one sample and `dac_data` the one calibration word; the debug words (state transitions, command-done, SPI bits) appear only on the debug lanes.
Result: `[PASS]` 2026-10-28 -- one word each in `adc_data` and `dac_data`, debug words only in the debug reads.

**5.3.e -- Full debug lane drops without halting (target).**
Run: with debug on and nothing reading the debug lanes, drive enough commands to fill a debug FIFO (e.g. a long `do_adc_rd` repeat); then `sts`, `adc_debug_fifo_sts 1`, `status`.
Expect: the system keeps running (no halt), the debug FIFO reads full, and the debug-full flag reads Yes for that board; it clears after `off` and power on.
Result: `[PASS]` 2026-10-28 -- a 1,001-read `do_adc_rd` burst with debug on filled board 1's ADC debug FIFO (4096 words, "words dropped: Yes") with the system still Running; turning the system off cleared the flag and left the words in place.

**5.3.f -- DMA run teardown leaves the debug lanes intact (target).**
Run: with debug on, a DMA `waveform` run; afterwards `adc_debug_fifo_sts 0` and `dac_debug_fifo_sts 0`.
Expect: the run completes as before and the debug words from the run are still in both debug FIFOs (the teardown resets only `dac_cmd` and `adc_data`).
Result: `[PASS]` 2026-10-28 -- board 0's debug FIFOs went from 0 words after `hard_reset` to full after two DMA runs with debug on.

**5.3.g -- Debug lanes stream to file (target).**
Run: in `shim-test` with debug on for board 1, `stream_adc_debug 1 /tmp/adc_dbg.txt` and `stream_dac_debug 1 /tmp/dac_dbg.txt`; then `do_adc_rd_ch 9` and `do_dac_get_cal 9`; then `stop_adc_debug_stream 1`, `stop_dac_debug_stream 1`, and look at both files.
Expect: each file holds that lane's debug words for the commands, both debug FIFOs read empty afterwards, and the `adc_data` / `dac_data` words are still in their own lanes.
Result: `[PASS]` 2026-10-28 -- the streams wrote 20 ADC and 38 DAC debug words, both debug FIFOs read empty, and the data lanes kept their one word each. (The streams first failed silently because output paths went through the input-file glob; output files are created up front, with an error if the path can't be.)

### Stage 5.4 -- Capture rate lane (`adc_rate_ctrl` folded into the board FIFO array)

`adc_rate_ctrl` loses its AXI-Lite slave, SmartConnect port (7 -> 6) and `0x40300000` window. Software reaches it through word 7 of each board's FIFO page (`+0x1C`): a write strobes that board's `read_delta_en` with the delta, and a read returns the backlog without popping anything. The lane is never gated by `mode_block` or full flags. The same build carries the `ports.xdc` cleanup for TIMING-28 and TIMING-56.

**5.4.a -- Core tests pass (host).**
Run: `make tests PROJECT=rev_d_shim`.
Expect: `axi_board_fifo_array` passes 7/7, including `test_rate_lane` (one strobe per rate write with the right board and data, no strobe from other lanes or absent boards, backlog readback without a pop, no `mode_block` or full-flag gating); `adc_rate_ctrl` passes its six tests driving `read_delta_en` / `read_delta` and reading `backlog_out` directly.
Result: `[PASS]` 2026-10-27 -- `axi_board_fifo_array` 7/7 (including `test_rate_lane`), `adc_rate_ctrl` 6/6; `ad5676_dac_ctrl` 13 pass + 1 skip and `ads816x_adc_ctrl` 15 pass + 1 skip, unchanged.

**5.4.b -- Builds, meets timing, and drops the window (host).**
Run: `make`; read the hierarchical utilization report, `system_wrapper_timing_summary_postroute_physopted.rpt` and the methodology section of the Vivado log; then `dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/images/linux/system.dtb 2>/dev/null | grep -i rate`.
Expect: no new critical warnings; timing met; no TIMING-28 or TIMING-56 entries; `sys_cfg_axi_intercon` about 70 LUT lower (one fewer master port) and `adc_rate_ctrl` smaller than its 276 LUT with the AXI slave gone; no `adc_rate_ctrl` node in the device tree. Record in `RESOURCE_RECLAIM.md`.
Result: `[PASS]` 2026-10-27 (four boards) -- 0 critical warnings, timing met after the hold re-route (WNS +0.300 ns, WHS +0.050 ns); methodology lists only TIMING-9 (1) and TIMING-18 (90); nothing at `0x40300000` in the device tree. 30,559 LUT (-37): `adc_rate_ctrl` 276 -> 220, `sys_cfg_axi_intercon` 1,147 -> 1,121, the array 313 -> 363 for the backlog read mux. Logged as `RESOURCE_RECLAIM.md` step 9.

**5.4.c -- DMA capture still packs and reports its backlog (target).**
Run: `ls -l /dev/adc_rate_ctrl`; then two back-to-back default (DMA) `waveform` runs with `--adc`.
Expect: `/dev/adc_rate_ctrl` does not exist; each run calibrates over DMA (the per-read deltas go through the rate lane), plays, writes every sample, and prints a peak capture backlog in the same range as Stage 5.1.g (hundreds of words, starting from zero each run).
Result: `[PASS]` 2026-10-28 -- no `/dev/adc_rate_ctrl`; two DMA runs of `dma_wave` calibrated over DMA and wrote all 6600 samples, with peak backlogs of 256 and 512 words.

### Stage 5.5 -- Clock-domain crossing and SPI timing cleanup

A `report_cdc` pass found three real crossing bugs, all fixed: `spi_sts_sync` ran its 33 `sync_coherent` instances backwards (AXI -> SPI); `sync_coherent` used its FIFO's read-side empty flag in its write enable unsynchronized (it's rewritten as a request/acknowledge handshake that refills itself after a one-sided reset); and the board FIFO array and trigger bridge sticky flags reset from SPI-side synchronizers (they use the `*_aclk_rst` ones). `spi_off` is registered before its synchronizer.

The external inputs are synchronized: `Manual_Enable` through a 2-flop synchronizer, `shutdown_sense` sampling a synchronized pin at the end of a 16-cycle dwell per select, and `trigger_core` with a plain 2-flop chain and `blocked` applied after it.

On timing, the SPI MMCM is built at 30 MHz (the maximum runtime rate), `ports.xdc` adds MOSI/CS/LDAC output delays against the forwarded SCK and false paths for DAC MISO (boot clock only) and the slow control pins, and the clock snoop flags anything above 31 MHz. ADC MISO is left unconstrained (see `HARDWARE_UPDATE_TODO.md`).

The rest of `report_cdc` is cleared: each board's DAC debug FIFO gets its own SPI-side reset synchronizer chained off the `dac_data` one, `sync_coherent` keeps its copy enable and reset on the flops' CE and R/S pins (`DIRECT_ENABLE` / `DIRECT_RESET`), and `waivers.xdc` waives the clock-mux select and DAC MISO.

Software caps the SPI clock at `SPI_CLK_MAX_HZ` (30 MHz). The tools set their own 10 MHz clock at startup; `shim-test` doesn't, and warns that the clock may be above the 10 MHz ADC limit.

The ADC core accepts a repeating command's count word any time before the first iteration finishes, and flags `cmd_buf_underflow` only if it's still missing then. `shim-test` writes a repeating `ADC_RD` and its count as two register writes, which hit that window on hardware.

**5.5.a -- Core tests pass (host).**
Run: `make tests PROJECT=rev_d_shim`, plus `make projects/rev_d_shim/cores/base/sync_coherent/tests/test_status PROJECT=rev_d_shim` (`sync_coherent` is a submodule, so `make tests` skips it).
Expect: `sync_coherent` passes its five handshake tests (three clock ratios, output-only reset refill, input-only reset hold, fast-changing `din`, random resets); `shutdown_sense` 4/4; `trigger_core` 10/10; `ads816x_adc_ctrl` 17 + 1 skip, including the two repeat-count tests (count arriving mid-iteration; count missing).
Result: `[PASS]` -- `shutdown_sense` 4/4 and `trigger_core` 10/10 (2026-10-27); `ads816x_adc_ctrl` and `sync_coherent` (2026-10-28).

**5.5.b -- Builds, meets timing at 30 MHz, and the CDC report is clean (host).**
Run: `make`; read the timing summary and methodology sections, then `report_cdc` on the routed checkpoint.
Expect: no critical warnings; timing met with the SPI clock at 33.333 ns; TIMING-18 only on the 8 `ADC_MISO_p` inputs; `report_cdc` critical entries limited to known benign groups. Record in `RESOURCE_RECLAIM.md`.
Result: `[PASS]` 2026-10-27 (four boards) -- 0 critical warnings, timing met after the hold re-route (WNS +0.276 ns, WHS +0.050 ns), 15.9 ns of SPI-domain slack, and SPI outputs with 7.0 ns setup and hold at 30 MHz. Methodology: TIMING-9 (1), TIMING-18 (8) and ULMTCS-1. Critical CDC entries fell from 3,360 to 30 (cleared in 5.5.h). 28,808 LUT (-1,751), logged as `RESOURCE_RECLAIM.md` step 10.

**5.5.c -- Status words read sensibly (target).**
Run: in `shim-test` after `clk_set 10`, `ctrl_on`, `pow_on`; then a few `do_dac_wr_ch` / `do_adc_rd_ch` commands on board 0 and two `force_trig`; then `trig_count`, `dac_last_received_cmd 0`, `adc_last_received_cmd 0`, `dac_cmds_since_reset 0`, `adc_cmds_since_reset 0`, and `status`.
Expect: the trigger count, last-received words and command counts match what was sent -- whole, stable values crossing SPI -> AXI.
Result: `[PASS]` 2026-10-28 -- trigger count 2 after two `force_trig`; last-received commands and command counts match what was sent.

**5.5.d -- Configuration reaches the SPI side across SPI power cycles (target).**
Run: three `off` / `ctrl_on` / `pow_on` cycles in `shim-test`. After each, a DAC write and an ADC read on board 0, and a DAC command whose delay is below the minimum from `get_min_delay_times`.
Expect: every cycle boots cleanly, the write and read work, and the short delay raises "DAC delay too short" each time, showing the timing config reached the SPI side again after each SPI reset (the `sync_coherent` refill).
Result: `[PASS]` 2026-10-28 -- all three `hard_reset` cycles booted, read and wrote, and halted on "DAC delay too short".

**5.5.e -- External inputs still work (target).**
Run: with the system running, drop Manual Enable; restart and trip one board's shutdown sense line if the bench allows it; restart and arm `trig_expect_ext 1` then pulse the external trigger input, followed by `trig_count`.
Expect: Manual Enable halts with "External shutdown"; a tripped sense line halts with "Shutdown sense" naming the right board; the external pulse fires one trigger.
Result: `[PASS]` 2026-10-28 -- trigger count 1 from the external pulse, and dropping Manual Enable halted with "External shutdown triggered". Shutdown sense wasn't tripped on the bench; it's taken as working from its core tests and no spurious halts all session.

**5.5.f -- SPI clock defaults (target).**
Run: `status` right after boot; start `shim-test` (note the startup warning), then `clk_freq`, `clk_set 31`, `clk_set 30` and `clk_set 10`; then start `static-shims` / `channel-check` and a `waveform` run, checking the reported SPI clock.
Expect: 30 MHz after boot; `shim-test` warns and rejects `clk_set 31` (above `SPI_CLK_MAX_HZ`) but accepts 30 and 10; the other tools come up at 10 MHz with calibration and ADC reads working.
Result: `[PASS]` 2026-10-28 -- all as expected; `waveform` ran at 10 MHz and `channel-check 32` calibrated every connected channel (gains 0.999-1.001).

**5.5.g -- Repeating ADC reads from `shim-test` (target).**
Run: in `shim-test`, `stream_adc_data_to_file 1 4004 /tmp/adc_fill.txt` (1,001 reads of 4 words), then `do_adc_rd 1 delay 1000 1000`; once it finishes, `adc_cmd_fifo_sts 1` and `sts`.
Expect: no "ADC command buffer underflow" halt, the ADC command FIFO reads empty, and the file holds the samples from the repeated reads.
Result: `[PASS]` 2026-10-28 -- the system stayed Running, the command FIFO read empty, and the stream wrote all 4,004 words. (Before the `repeat_pending` change this halted on an underflow, since the count word reached the FIFO a moment after the command started.)

**5.5.h -- CDC report is fully accounted for (host).**
Run: `make tests PROJECT=rev_d_shim` plus the `sync_coherent` target from 5.5.a; then `make`, and `report_cdc` and `report_methodology` on the routed checkpoint.
Expect: `sync_coherent` still 5/5; no critical `report_cdc` entries, with the clock-mux select and DAC MISO shown as waived; timing met.

Result: `[PASS]` 2026-10-28 -- 0 critical warnings, timing met (WNS +0.320 ns, WHS +0.050 ns), no critical CDC entries (10 waived endpoints), and TIMING-9 gone, leaving TIMING-18 (ADC MISO) and ULMTCS-1. The two flagged `sync_coherent` bits were the default-1 bits (`thresh_val` 12, `thresh_window` 16), where synthesis had folded reset and enable into a LUT in front of D; with the attributes they're plain FDSE flops. The first DAC debug reset fix just moved the fan-out to the AXI-side reset bit; chaining the debug synchronizer off the `dac_data` one cleared it. 28,588 LUT (`RESOURCE_RECLAIM.md` step 11).
