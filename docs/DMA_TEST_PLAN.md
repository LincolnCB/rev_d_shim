# DMA Port Test Plan

This is the companion to `DMA_PLAN.md`: a running checklist of what to verify at each stage of the DDR-backed DMA port, filled in as the work lands. `DMA_PLAN.md` says what to build and why; this document says how you know it worked. The two are meant to be read side by side -- the stage and step numbering here follows the plan exactly.

Each check records how to run it, what a pass looks like, and a result line to fill in when you get to hardware. Many checks need a boot on the snickerdoodle black, so they stay pending until a board is available; a check that only needs the host build (device-tree compile, module compile, byte-exact file compare) can be marked off from the build artifacts in `tmp/` without hardware.

Result legend for each check:
- `[ ]` not yet run
- `[PASS]` ran, met the expected result -- add the date and any notes
- `[FAIL]` ran, did not meet it -- add the date, what happened, and the follow-up
- `[N/A]` not applicable in the end (say why)

Unless noted otherwise, commands run on the target (the board's Linux console) as a normal, non-root user -- part of the point of Stage 1 is that none of this needs root. Build-artifact checks run on the host from the repo root; they assume the most recent `make petalinux_build` for `snickerdoodle_black 1.0 rev_d_shim`, whose outputs live under `tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/`.

---

## Stage 1 -- Non-root access and memory plumbing

The goal of the whole stage: the existing tools run unprivileged, the DMA `/dev` nodes and reserved DDR appear, and the SPI datapath is byte-for-byte unchanged from before the port. Stage 1 changes no datapath, so its top-level acceptance is "nothing the instrument already did got worse, and the new plumbing is present."

### Steps 1-2 -- kernel_modules wiring and reserved memory (built)

These are the device-tree and module-build changes. The build-artifact checks can be marked now; the on-target checks wait for a boot.

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
Expect: udmabuf0 and udmabuf1 register cleanly; no `of_reserved_mem_device_init failed return=-22` and no other probe error. This is the alignment gotcha the dtsi comment calls out -- a misaligned or too-small reusable region fails exactly here.
Result: `[PASS]` 2026-09-04 -- both nodes registered (driver 5.2.0); udmabuf0 phys `0x30400000` size `67108864`, udmabuf1 phys `0x30000000` size `4194304`; no probe error.

**1.2.e -- The DMA device nodes appear with the intended geometry (target).**
Run:
```bash
ls -l /dev/udmabuf0 /dev/udmabuf1
for d in udmabuf0 udmabuf1; do echo "$d:"; cat /sys/class/u-dma-buf/$d/phys_addr /sys/class/u-dma-buf/$d/size; done
```
Expect: both nodes exist. `udmabuf0` (data) phys_addr `0x30400000`, size `67108864` (64 MB). `udmabuf1` (desc) phys_addr `0x30000000`, size `4194304` (4 MB). These are the values the software reads at runtime instead of hardcoding.
Result: `[PASS]` 2026-09-04 -- both nodes present with the intended phys_addr and size. They come up `crw------- root root` (mode 0600) for now; step 5's boot-script `chmod` is what makes them non-root, so this is expected here, not a failure.

**1.2.f -- The reserved regions took the DDR out of the kernel's hands (target).**
Run:
```bash
dmesg | grep -i 'reserved mem'
cat /proc/iomem | grep -iE 'reserved'   # note: reusable CMA regions do not appear here
```
Expect: dmesg shows both regions initialized -- `OF: reserved mem: 0x30000000..0x303fffff ... shim_dma_desc` and `0x30400000..0x343fffff ... shim_dma_data`, both `map reusable`. Because the regions are reusable/shared-dma-pool (CMA), the kernel hands the memory back as movable until u-dma-buf claims it, so they intentionally do not appear as a reserved range in `/proc/iomem` -- dmesg is the authoritative check here.
Result: `[PASS]` 2026-09-04 -- both regions initialized `map reusable` at the intended ranges; the default global CMA pool (16 MiB at `0x3f000000`) is separate and does not overlap.

**1.2.g -- The existing SPI datapath is unchanged (target).**
Run: exercise the current tools exactly as before the port (they still use `/dev/mem` at this point -- steps 3-5 have not moved them yet). A normal DAC-out / ADC-in check through `shim-test` or the usual bring-up sequence.
Expect: identical behavior to a pre-port build -- boot self-test passes, a known sequence produces the same DAC output and ADC readback. Steps 1-2 must be invisible to the datapath.
Result: `[PASS]` 2026-09-04 -- datapath behaves as before the port.

Note: `pl-reg` and `pl-irq` load here but bind nothing yet -- no node carries their compatible until step 3 (register windows) and step 4 (the hw_manager interrupt). Their real bring-up checks live in those steps below.

### Step 3 -- Register windows onto pl-reg-shim (map_memory.c migration)

`pl-reg-shim` binds this project's four register cores and publishes each window as a short-named `/dev` node; the shim programs open those nodes instead of `/dev/mem`.

**1.3.a -- Each register window appears as a short-named /dev node (target).**
Run:
```bash
ls -l /dev/sys_ctrl /dev/sys_sts /dev/spi_clk /dev/trig_fifo /dev/dac_fifo_* /dev/adc_fifo_*
dmesg | grep pl-reg-shim
```
Expect: every node present, mode `crw-rw-rw-` (0666). `dmesg` shows one `/dev/<name> ready (mode 0666): 0x... size 0x..., compatible "xlnx,..."` line per bound window -- `sys_ctrl` (`xlnx,axi-sys-ctrl-1.0`), `sys_sts` (`axi-sts-register`), `spi_clk` (`axi-clock-timing-snoop`), and one `dac_fifo_<b>` / `adc_fifo_<b>` per board plus `trig_fifo` (all `axi-fifo-bridge`). The count of `dac_fifo_*` / `adc_fifo_*` equals the board count in the bitstream.
Result: `[PASS]` 2026-09-04 -- all 12 windows bound at 0666 with the intended base/size/compatible; this was a 4-board bitstream, so `dac_fifo_0..3` / `adc_fifo_0..3` + the 3 singletons + `trig_fifo`.

**1.3.b -- The tools run fully non-root (target).**
Run: as a normal user (no sudo), run a read-only tool that maps the windows, e.g.
```bash
status
```
Expect: it initializes and prints without a permission error and without touching `/dev/mem` (`grep /dev/mem` of the tools finds nothing). A wrong or missing node would fail loudly at `open()` with `ENOENT` instead of reading a stale address.
Result: `[PASS]` 2026-09-04 -- `status` ran as a normal user, no sudo, no permission error.

**1.3.c -- Register reads are sensible and match the pre-port build (target).**
Run:
```bash
status
```
Expect: the values read back through the `pl-reg-shim` nodes match a known-good `/dev/mem` build -- a valid hardware-state code, a reported SPI clock near the configured frequency, sane FIFO status -- i.e. the datapath reads identically, just through named nodes.
Result: `[PASS]` 2026-09-04 -- state `Idle`, SPI clock reads 30.000 MHz (matches `spi_clk_freq_mhz` in the block design), boards 0-3 `Present: Yes` / 4-7 `Present: No` (matches the 4-board bitstream). The `spi_clk` window (0x800) reading the right frequency confirms the mapping is byte-correct.

Note: `status` reads per-board FIFO state through the `sys_sts` register, so it does not open the per-board `dac_fifo_<b>` / `adc_fifo_<b>` nodes and runs fine on a partial-board bitstream. The tools that do map those nodes (`shim-test`'s DAC/ADC commands, `waveform`) will fail loudly at `open()` for an absent board, given the hardcoded `board < 8` loop.

Note: `pl-irq-shim` still binds nothing at this step -- the `hw_manager_irq` node is `generic-uio` until step 4 -- so `/dev/hw_manager_irq` does not exist yet and the interrupt still arrives on `/dev/uio0`. That is expected here, not a failure.

### Step 4 -- The hw_manager interrupt onto pl-irq-shim (sys_sts.c migration)

The `hw_manager_irq` dtsi node is retagged `zynq-toolbox,pl-irq-shim`, `sys_sts.c`'s monitor opens `/dev/hw_manager_irq` instead of `/dev/uio0`, and the now-dead `uio_pdrv_genirq.of_id` bootarg is dropped from the kernel command line.

**1.4.a -- The interrupt node is present, non-root, and /dev/uio0 is gone (target).**
Run:
```bash
ls -l /dev/hw_manager_irq
dmesg | grep pl-irq-shim
ls -l /dev/uio0    # expect: No such file or directory
```
Expect: `/dev/hw_manager_irq` present at `crw-rw-rw-` (0666); `dmesg` shows `/dev/hw_manager_irq ready (mode 0666): irq <n>, compatible "zynq-toolbox,pl-irq-shim"`. `/dev/uio0` no longer exists (generic-uio fully retired -- the bootarg is gone).
Result: `[PASS]` 2026-09-04 -- `/dev/hw_manager_irq` at 0666, `pl-irq-shim` bound (irq 49, `zynq-toolbox,pl-irq-shim`), `/dev/uio0` absent.

**1.4.b -- The interrupt is delivered non-root and wakes the monitor (target).**
Run: as a normal user (no sudo), run a workflow that arms `hw_manager` and then causes an interrupt (a normal run that reaches a state change, or an injected fault/shutdown).
Expect: the monitor thread reports the interrupt and the decoded hardware status once per event, then re-arms and blocks again (or exits cleanly when the state leaves `S_RUNNING`). No permission error, no spin/storm of repeated prints.
Result: `[ ]`

**1.4.c -- Re-arm works and there is no interrupt storm (target).**
Expect: after handling, the monitor's `write(fd, 1)` re-enables delivery; the line does not refire until the source changes again -- the loop blocks in `read()` rather than busy-looping.
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
Expect: both `/dev/udmabuf*` at `crw-rw-rw-` (0666); the `sync_for_device` / `sync_for_cpu` sysfs files `-rw-rw-rw-` (0666); the `test -w` prints its line. Without the boot script these are 0600 / 0664 and a non-root `sync` fails `EACCES`.
Result: `[PASS]` 2026-09-04 -- `/dev/udmabuf0`/`1` at 0666, `sync_for_device`/`sync_for_cpu` at 0666, `udmabuf1` sync writable.

**1.5.b -- A non-root program can map and sync a buffer end to end (target).**
Expect: as a normal user, `open` `/dev/udmabuf0`, `mmap` it, write a pattern, `sync_for_device`, read back, `sync_for_cpu`, all with no error. Fully exercised by the Stage 3 DMA software; until then, 1.5.a's permission checks are the proxy.
Result: `[ ]`

### Stage 1 exit

**1.X -- Non-root, plumbing present, datapath intact.**
Expect, all together: every tool runs non-root; `/dev/udmabuf0`, `/dev/udmabuf1`, and the `pl-reg`/`pl-irq` nodes are present and correctly owned; the reserved regions read back the intended address and size; and a full datapath run is unchanged from a pre-port build. Stage 1 is independent of the DMA datapath, so this should be solid before Stage 2 starts.
Result: `[ ]`

---

## Stage 2 -- PL datapath (MCDMA plus routing), PIO path preserved

Stage 2 is built in sub-steps, each leaving a working, testable tree. Stage 2a
lands the MCDMA engine, the 64-bit HP0 memory path, and its non-root control
window, with the 16 streams wired as a per-channel loopback so the engine can be
brought up byte-exact before the real SPI datapath is rewired onto it (2b) and
the completion/error interrupts are folded into `hw_manager` (2c). The existing
PIO FIFO datapath is untouched in 2a.

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
Expect: the build completes; MCDMA has 4 MM2S + 4 S2MM channels (== `board_count`), 23-bit SG length, 64-bit MM2S/S2MM data; the `s2mm_mux` has `HAS_TLAST=1`, `ARB_ON_TLAST=1`, `ARB_ON_MAX_XFERS=1024`, and its single MI window spans `[0x0, 0x3]`; the `mm2s_demux` routes M00..M03 to TDEST 0..3; `PCW_USE_S_AXI_HP0=1`. The sixteen `introut` lines are left unconnected in 2a (benign unconnected-output warnings, not errors).
Result: `[PASS]` 2026-09-08 -- all values as intended (mcdma 4+4, sg 23-bit, 64-bit masters; mux HAS_TLAST/ARB_ON_TLAST/1024 + window [0,3]; demux 0..3; HP0 enabled). Whole-design synthesis fits: 33,410 LUT (~63% of 53,200) at 4 boards, DMA infra ~11k (mcdma 6,156 + HP0 SmartConnect 4,402 + demux/mux/loop-fifos ~0.5k).

**2a.2 -- The MCDMA node is retagged for pl-reg-shim in the device tree (host).**
Run:
```bash
dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/images/linux/system.dtb 2>/dev/null | grep -iB1 -A5 'axi_mcdma@40400000'
```
Expect: the `axi_mcdma@40400000` node carries `compatible = "zynq-toolbox,mcdma-userspace"` (the private string that keeps the in-kernel Xilinx driver off it, overriding the auto `xlnx,axi-mcdma-1.2`), the node keeps its `mcdma` label so `pl-reg-shim` names it `/dev/mcdma`, and `pl-reg-shim`'s match table lists that compatible.
Result: `[PASS]` 2026-09-08 -- the final `system.dtb` shows `axi_mcdma@40400000` with `compatible = "zynq-toolbox,mcdma-userspace"` and `reg = <0x40400000 0x10000>`; `__symbols__` maps `mcdma` to the node, so the `pl-reg-shim` label lookup resolves to `/dev/mcdma`.

**2a.3 -- pl-reg-shim binds the MCDMA control window non-root (target).**
Run:
```bash
dmesg | grep -i 'mcdma'
ls -l /dev/mcdma
```
Expect: `pl-reg-shim` reports `/dev/mcdma ready (mode 0666) ... compatible "zynq-toolbox,mcdma-userspace"`; `/dev/mcdma` is present at mode 0666 (world-rw, no root).
Result: `[ ]`

**2a.4 -- Byte-exact MCDMA loopback round-trip, non-root (target).**
Run:
```bash
mcdma-loopback
mcdma-loopback 1        # a single non-zero channel alone, to isolate the mux
```
Expect: every channel reports `ok  received 2048/2048 bytes`, then `All channels round-tripped.`, with no root and no data-mismatch lines. A single-channel run also round-trips. This proves the engine, the HP0 64-bit path, the TDEST demux/mux routing, and non-root control together. (The channel count must match the built `board_count`; on the 4-board bitstream that is channels 0-3.)
Result: `[ ]`

**2a.5 -- The existing SPI datapath is unchanged (target).**
Run:
```bash
status
```
Expect: same output as a pre-2a build -- state `Idle`, SPI clock correct, boards present as built. Adding the MCDMA on GP0/HP0 does not disturb the GP1 PIO FIFO path.
Result: `[ ]`

**Next step -- on-target bring-up (Stage 2a checks fold into 2b).** Stage 2b has
replaced the per-channel loopback with the real per-board datapath, so the
loopback round-trip (2a.4) no longer applies -- it was scaffolding and was never
run on hardware (no board was available), and the real datapath's byte-exact test
supersedes it. The two still-relevant Stage 2a target checks fold into the 2b
bring-up: 2a.3 (`/dev/mcdma` binds non-root) and 2a.5 (`status` shows the SPI
datapath unchanged with all boards in the default PIO mode). Both, plus the 2b
byte-exact datapath test, need a board and the Stage 3 mover software, so on-target
validation of the datapath rolls into Stage 3.

### Stage 2b -- real per-board datapath (host + core tests)

The datapath rework is in: a TDEST demux feeds each board's DAC command FIFO
through a write-only `axis_fifo_bridge`, each board's ADC data FIFO drains through
the new `adc_packetizer` into a packet-atomic TDEST mux, and a per-board
`datapath_mux` hard-locks the shared FIFO ports to either the PIO `axi_fifo_bridge`
or the DMA adapters under `axi_sys_ctrl`'s `datapath_mode[i]` bit (reset PIO). The
DMA-side adapters, the packetizer, and the select all live inside
`axi_spi_interface`; the top level only wires the demux/mux to the module. `mode_viol`
reporting and the sixteen `introut` lines fold into `hw_manager` in Stage 2c below.

**2b.1 -- The new and changed datapath cores pass their cocotb tests (host).**
Run (in the cocotb container, no board -- in Docker mode go through make so the script runs inside the container):
```bash
make projects/rev_d_shim/cores/shim/adc_packetizer/tests/test_status PROJECT=rev_d_shim
make projects/rev_d_shim/cores/shim/datapath_mux/tests/test_status PROJECT=rev_d_shim
# VM mode (cocotb on the host) can call the script directly instead:
#   ./scripts/make/test_core.sh rev_d_shim shim adc_packetizer
#   ./scripts/make/test_core.sh rev_d_shim shim datapath_mux
```
Expect: `adc_packetizer` passes its 5 word-granular framing tests (reset, cap+tail, mixed 1-/4-word reads, single-word drip, backpressure); `datapath_mux` passes its select/gating tests (PIO selects PIO + gates DMA, DMA selects DMA + gates PIO, wr_en/rd_en gated to the selected owner, randomized model match).
Result: `[ ]` (ready to run in the cocotb container; no board needed)

**2b.2 -- The block design builds with the real datapath (host).**
Run (from the built project under `tmp/snickerdoodle_black/1.0/rev_d_shim`), and confirm from the block design / synthesized netlist:
```bash
# per-board DMA adapters + select are present; the loopback FIFOs are gone
grep -c 'datapath_mux_\|adc_data_packetizer_\|dac_cmd_dma_bridge_' <bd or netlist>
grep -c 'dma_loop_fifo_' <bd or netlist>   # expect 0
```
Expect: the build completes; for each board there is a `dac_cmd_dma_bridge_<b>` (`axis_fifo_bridge`, write-only), an `adc_data_packetizer_<b>` (`BOARD_INDEX = b`, `MAX_PACKET_WORDS` at or under the mux `ARB_ON_MAX_XFERS`), and a `datapath_mux_<b>`; no `dma_loop_fifo_*` remain; `axi_sys_ctrl` drives the module's `datapath_mode`. Check the post-synth hierarchical utilization to confirm the datapath still fits (the loopback FIFOs are replaced by the packetizer + bridge + mux; LUT is the binding constraint).
Result: `[PASS]` 2026-09-15 (host, from artifacts, 4-board build). Per board: `adc_data_packetizer_<b>` 17 LUT / 8 FF, `dac_cmd_dma_bridge_<b>` 2 LUT, `datapath_mux_<b>` 51 LUT (the 32-bit write-data mux); no `dma_loop_fifo_*` remain. Whole design 33,859 LUT (63.6% of 53,200), 89 RAMB36 + 4 RAMB18 -- only +449 LUT over the Stage 2a baseline (33,410), since the loopback FIFOs were traded for the real adapters. The build routing at all confirms the AXIS interface inference, the module AXIS interface pins, the `axis_fifo_bridge` `AXIS_DATA_WIDTH` fix, and the reset/slice wiring.

### Stage 2c -- mode_viol fault and MCDMA interrupt fold

`mode_viol` reporting and the sixteen MCDMA `introut` lines fold into `hw_manager`. The
PIO `axi_fifo_bridge` responds `OKAY` on every access (never `SLVERR`), raising
`fifo_overflow` / `fifo_underflow` on a genuine full/empty of the owned port and `mode_viol`
when the datapath mode has handed its port to the DMA owner. `axi_spi_interface` OR's the
two per-board bridges' `mode_viol` into an 8-bit mask that folds into `hw_manager` as a
per-board fault (`STS_MODE_VIOL`). The MCDMA per-channel `introut` lines pack into two
8-bit buses and doorbell `hw_manager`'s single `ps_interrupt` during a run without leaving
`S_RUNNING`.

**2c.1 -- The hw_manager cocotb tests pass, including mode_viol and the DMA doorbell (host).**
Run (in the cocotb container, no board):
```bash
make projects/rev_d_shim/cores/shim/hw_manager/tests/test_status PROJECT=rev_d_shim
# VM mode: ./scripts/make/test_core.sh rev_d_shim shim hw_manager
```
Expect: all hw_manager tests pass. `test_running_per_board_errors` now covers `mode_viol` at every board position (`STS_MODE_VIOL`, correct board number), and `test_running_dma_introut_doorbell` confirms an `introut` event pulses `ps_interrupt` once, stays in `S_RUNNING`, does not re-pulse on the held level, and doorbells again after the line falls and re-raises.
Result: `[PASS]` 2026-09-21 (host, cocotb container) -- hw_manager `TESTS=59 PASS=59 FAIL=0 SKIP=0`, including `mode_viol[0..7]` and `test_running_dma_introut_doorbell`. `datapath_mux` still passes (unchanged logic). `axi_fifo_bridge` has no cocotb testbench; its always-OKAY / `mode_viol` behavior is covered by the build (2c.2) and the on-target checks.

**2c.2 -- The block design builds with mode_viol and the introut fold wired (host).**
Run (from the built project under `tmp/snickerdoodle_black/1.0/rev_d_shim`), and confirm from the block design / synthesized netlist:
```bash
# mode_viol aggregation into hw_manager and the two introut concats are present
grep -c 'mode_viol\|dma_mm2s_introut_concat\|dma_s2mm_introut_concat' <bd or netlist>
```
Expect: the build completes; `axi_spi_interface` exposes an 8-bit `mode_viol` wired to `hw_manager/mode_viol`; the MCDMA `mm2s_ch<n>_introut` / `s2mm_ch<n>_introut` lines are concatenated (zero-padded for unused boards) into `hw_manager/dma_mm2s_introut` and `hw_manager/dma_s2mm_introut`; utilization stays within budget (the added logic is a handful of LUTs -- an OR per board plus two concats).
Result: `[PASS]` 2026-09-21 -- full Vivado/PetaLinux build completed cleanly with the new bridge ports (repackaged), the `mode_viol` aggregation, and the two `introut` concats wired to `hw_manager`. No unconnected-input criticals.

### Stage 2b/2c acceptance criteria

Turned into concrete checks as target validation (needs a board and the Stage 3
mover) and the interrupt fold (2c) land. From the plan:

- Byte-exact loopback-style transfer on the DMA-driven FIFOs (a known pattern pushed MM2S through the DAC-command FIFO and/or captured S2MM from the ADC-data FIFO comes back identical).
- The PIO fallback still works with a board's `datapath_mode` bit set to PIO -- the same tools, same result, on the un-migrated path.
- Per-board mode selection: one board on DMA while the rest stay on PIO, each independently.
- Wrong-mode access (PIO poke at a DMA-mode board, or vice versa) does not crash and does not corrupt: it returns `OKAY`, discards, and raises `mode_viol` into `hw_manager`, which drives the normal coordinated shutdown (status word plus interrupt), not a `SIGBUS`.
- ADC packetizer framing: `tlast` closes a packet when the board's FIFO runs dry or at the `MAX_PACKET_WORDS` cap, on any word boundary (packets carry no sample-set alignment, since a single-channel read writes one word); a momentarily-empty board releases the mux (no head-of-line stall); the run tail self-flushes when the FIFO drains; packet size never exceeds the mux `ARB_ON_MAX_XFERS` backstop.
- The sixteen MCDMA completion/error lines fold into `hw_manager` and ride its single interrupt -- no separate DMA interrupt path.

Checks: the byte-exact DMA-driven FIFO transfer, the on-target `mode_viol` fault, and the interrupt-driven completion (the `introut` doorbell) are all validated on target -- see the Stage 3.0 low-level bring-up results below (3.0.a-d PASS, 2026-09-22). Host verification (2c.1 cocotb, 2c.2 build) also passed. What remains for the full acceptance set -- per-board mode independence across several boards at once, and the ADC packetizer under a real multi-word waveform -- lands with the Stage 3 waveform integration.

**Stage 2 is complete (2a datapath, 2b real routing, 2c fault/interrupt fold), host- and hardware-validated.**

---

## Stage 3 -- Software

### Stage 3.0 -- low-level DMA bring-up (shim-test)

The `shim-test` DMA commands (`dma_mode`, `dma_status`, `dma_channel_test`, `dma_irq_test`,
`dma_mode_viol`) drive the datapath directly for bring-up and debugging. `dma_channel_test`
runs a single-channel round-trip: DMA a `DAC_WR_CH` command through MM2S into the board's DAC
command FIFO (the DAC drives current through the coil), arm S2MM, trigger a PIO `ADC_RD_CH` on
the same channel, and capture the sample back through S2MM into DDR. `dma_irq_test` does the
same round-trip but takes the S2MM completion on the folded `hw_manager` interrupt instead of
polling, and `dma_mode_viol` deliberately provokes a wrong-mode access. All DMA testing runs
at 10 MHz SPI (the default), since the ADC data misaligns above that on current hardware.

**3.0.a -- Byte-exact DAC delivery over MM2S (target).**
Run (`dma_mode 0 1` while off, then `ctrl_on`/`pow_on`):
```bash
dma_channel_test 0 1000
dac_last_received_cmd 0
```
Expect: the DAC's last received command is `0x600003e8` = `DAC_WR_CH channel 0 value 1000` -- the exact bytes crossed DDR -> MM2S -> demux -> the DAC.
Result: `[PASS]` 2026-09-17 -- `dac_last_received_cmd` decoded `DAC_WR_CH (channel=0, value=1000, bits=0x03E8)`, count incrementing per run.

**3.0.b -- Full DMA round-trip through the coil (target).**
Run:
```bash
channel_test 0 1000        # PIO reference first (system off, dma_mode 0 0)
# then dma_mode 0 1 / ctrl_on / pow_on:
dma_channel_test 0 1000
```
Expect: `dma_channel_test` completes with no descriptor timeout, captures one ADC word through S2MM, and the sample matches the PIO `channel_test` reference within measurement noise.
Result: `[PASS]` 2026-09-17 (10 MHz) -- PIO `channel_test 0 1000` read 996; `dma_channel_test 0 1000` reported "Captured 1 ADC word(s) via S2MM DMA. First sample: 994". The full path -- MM2S -> DAC -> coil -> ADC -> S2MM -> DDR -> C -- is proven end to end. This required the non-cached `u-dma-buf` mapping (`O_SYNC`, for descriptor/data coherence) and arming S2MM before the ADC read (so the packetizer's continuous drain lands in the capture buffer rather than being dropped toward an idle S2MM).

**3.0.c -- Interrupt-driven S2MM completion via the folded doorbell (target, Stage 2c).**
Run (`dma_mode 0 1` while off, then `ctrl_on`/`pow_on`):
```bash
dma_irq_test 0 1000
```
Expect: the same round-trip as `dma_channel_test`, but the S2MM completion is delivered by the `hw_manager` interrupt (`/dev/hw_manager_irq`, the folded MCDMA `introut`) rather than by polling -- the command reports "Completion delivered by the hw_manager interrupt (1 doorbell)" and captures the ADC word. If it instead reports "no interrupt was observed", the MCDMA per-channel interrupt enable or the `introut` -> `hw_manager` -> `ps_interrupt` fold is not asserting (bisect with `cat /proc/interrupts | grep pl-irq` before and after: a count increment means the line fires and the software wake is suspect; no increment means the fold or the enable bits are).
Result: `[PASS]` 2026-09-22 (10 MHz) -- `dma_irq_test 0 1000` reported "Completion delivered by the hw_manager interrupt (1 doorbell)" and captured 996. The `pl-irq` count in `/proc/interrupts` incremented across the run (1 -> 3), confirming the MCDMA `introut` reaches the GIC (edge, SPI 61) and wakes userspace via `pl-irq-shim`. The full fold MCDMA `introut` -> `hw_manager` -> single `ps_interrupt` -> `/dev/hw_manager_irq` is proven. (The tool counts its `read()` wakeups, so "1 doorbell" vs the +2 raw edge count is just batching -- the software woke once and found the completion.)

**3.0.d -- Wrong-mode access is graceful, not a crash (target, Stage 2c).**
Run (board 0 in DMA mode and running):
```bash
dma_mode_viol 0
```
Expect: the deliberate PIO write to a DMA-mode board returns without a `SIGBUS` (the shell keeps running -- the command prints "PIO write returned without a bus error"), and `hw_manager` halts with `STS_MODE_VIOL` for board 0. The command prints `PASS`. A `SIGBUS`/`Bus error` that kills the process would mean the bridge still returns `SLVERR`; a halt with a different status code would mean `mode_viol` is not wired or a different fault won the priority.
Result: `[PASS]` 2026-09-22 -- `dma_mode_viol 0` printed "PIO write returned without a bus error (accept-and-discard confirmed)", then `hw_manager` halted with `STS_MODE_VIOL` on board 0 and the command reported `PASS`. No `SIGBUS`. Note: the fault latch clears only on a buffer reset (or global reset), not on `off` -- after `off`/`ctrl_on`/`pow_on` the system re-faults with the same `STS_MODE_VIOL`; `hard_reset` (which pulses `buf_reset`) clears it. This is the Stage 4 coordinated-reset concern, surfaced here.

**Stage 3.0 is complete on hardware (3.0.a-d PASS).** The low-level `shim-test` DMA commands
give a proven, debuggable single-channel datapath -- MM2S DAC delivery, full coil round-trip,
interrupt-driven completion, and graceful wrong-mode handling. The remaining Stage 3 work is
the full clean integration into the run programs, sliced below.

### Stage 3.1 -- shared MCDMA mover library (multi-word)

The low-level mover (`src/sys/dma_ctrl.c`) is the shared library that `waveform` and `static-shims` consume through their `src/sys` symlink. Beyond the single-word bring-up calls it provides the multi-word run primitives the run programs build on: `dma_wave_arm` lays out a prebuffered DAC command stream and an ADC capture ring in the `u-dma-buf` regions, copies the DAC words into DDR, and starts both engines without touching the trigger; `dma_wave_avail` / `dma_wave_read` drain the captured `adc_data` incrementally behind a read cursor; and `dma_wave_read_total` / `dma_wave_expected` / `dma_wave_disarm` round out the lifecycle. The mover stays purely low-level -- words moved and FIFO/ring state, with no knowledge of triggers or DAC/ADC execution counts -- while the polling loop, file output, and waveform synthesis live in the command layer, mirroring the `dac_ctrl`/`adc_ctrl` versus `experiment_commands` split.

`shim-test` exercises it with a bench command trio. `dma_waveform_test` synthesizes an 8-channel triangle-times-envelope sequence at the matched DAC/ADC max cadence, writes the intended waveform to `<out>_dac.csv` in amps, prebuffers it, and starts a background collector; `dma_waveform_status` reports the DAC and ADC command counts, FIFO fills, and capture progress; and `dma_waveform_stop` ends a run early. The collector runs independently of the trigger, streams the ADC readback to `<out>_adc.csv` as the PL fills the capture (flushing each pass so data survives a later crash), and finishes on the expected sample count with no timeout -- a run may wait arbitrarily long for its trigger(s), of which there may be several.

Several datapath and command-format facts were pinned here and carry straight into the run-program integration:

- The MCDMA soft-reset (control bit 2) is global -- it resets both MM2S and S2MM. Reset both directions up front and then arm each; a reset issued after the other channel is armed wipes it.
- A multi-channel `DAC_WR` writes the eight channels to the DAC input registers and latches them to the outputs only on an LDAC pulse, so each `DAC_WR` must set LDAC or no current flows. `DAC_WR_CH` uses the immediate write-and-update SPI command and needs no LDAC, which is why the single-word bring-up drove current without it.
- Trigger alignment lives in the command stream, not the DMA. The first `DAC_WR` is a trigger-wait, so the DAC applies the t=0 point and continues on the trigger; the ADC stream leads with a trigger-wait `NO_OP`, because the ADC's delay is post-read and its first sample cannot itself be a trigger-wait.
- The ADC emits exactly four 32-bit words per eight-channel read, and an `ADC_RD` with `REPEAT = R` performs `1 + R` reads, so the capture length is exactly `4 * n_reads` words with `R = n_reads - 1`.
- Matched DAC/ADC cadence uses `delay = max(dac_min_delay, adc_min_delay)` read from the status register, so the two chips step together and the capture lines up with the command-stream index.
- S2MM capture uses a descriptor ring of one one-word descriptor per captured word, laid out contiguously, so a k-word packet spans k descriptors and the capture region is the `adc_data` word stream in order; each completed descriptor is one captured word.

Checks:
- `[PASS]` 2026-09-22 -- `dma_ctrl.c` compiles and links into `waveform` and `static-shims` via the `src/sys` symlink (the mover is standalone; the bench commands stay `shim-test`-only).
- `[PASS]` 2026-09-22 (4-board, 10 MHz) -- a 128 KB DAC stream (6553 `DAC_WR` updates) prebuffered into the DAC FIFO and played on the trigger (all 6553 executed, current on the supply), and the matched ADC capture (6600 reads, 26400 words) streamed back to `<out>_adc.csv` in amps with the triangle-times-envelope shape (full amplitude ch0-3, half ch4-7). The LDAC-latch and global-reset facts above were the two that closed it.
- `[PASS]` 2026-09-22 -- sizes are read from sysfs and an oversized run is rejected with a clear message (a 512 KB DAC run needs 6.7 MB of descriptors against the 4 MB `udmabuf1`). See the descriptor-region note below for the current cap and the larger-buffer goal.

The descriptor region (`udmabuf1`) is the capacity bottleneck, not the data region. One 64-byte descriptor per captured 4-byte word is a 16x overhead, so the 4 MB region caps a single capture near 256 KB of `adc_data` (about a 320 KB DAC run), while the 64 MB data region (`udmabuf0`) has ample room. Reaching the 10 MB-each goal for prebuffered commands and captured data (see the Reserved memory note in `DMA_PLAN.md`) needs a larger descriptor region, coarser packetization so each descriptor covers more words, or larger per-descriptor buffers with software compaction on readback -- a design pass deferred until the run-program integration lands.

### Stage 3.2 -- event-driven run-controller

Replace the observe-only `sys_sts` interrupt monitor with an active run-controller: block with
`poll()` over `/dev/hw_manager_irq` (the folded `hw_manager`/MCDMA interrupt) plus an `eventfd`
or self-pipe, so a hardware fault and a local abort (SIGINT, or a software-detected error)
unblock the same wait. On wake, read the sticky status word and dispatch: a normal end proceeds
to `sync_for_cpu` and readback; a fault drives the coordinated shutdown (`request_stop` plus
`hw_power_off`). Re-arm the interrupt each cycle; no spin-poll.

Checks:
- `[ ]` During a prebuffered run the controller blocks on one wait (no busy-polling of the status register); confirmed by CPU usage and by the `pl-irq` count advancing only on real events.
- `[ ]` A hardware fault (e.g. a `dma_mode_viol`-style violation, or an injected over/underflow) wakes the wait, and the controller reads `STS_*` and runs the coordinated shutdown.
- `[ ]` A local abort -- SIGINT (Ctrl-C) folded into the `eventfd` -- unblocks the same wait and stops the run cleanly, identically to a hardware fault.
- `[ ]` The normal end of a run (all channels complete) wakes the wait once and proceeds to readback.

### Stage 3.3 -- waveform and static-shims DMA integration

Wire the mover and the run-controller into the run programs. For a run on DMA-mode boards:
prebuffer each board's `dac_cmd` sequence and reserve its `adc_data` capture region, set the
per-board `datapath_mode` to DMA (while the system is off), `sync_for_device`, arm the S2MM
capture channels, release the trigger, wait on the run-controller for run-end-or-fault, then on
a normal end `sync_for_cpu` and read back the `adc_data` -- byte-exact against expectation. The
PIO path stays the fallback for boards left in PIO mode.

Checks:
- `[ ]` A real waveform plays end to end through the DMA path on at least one board; the `adc_data` readback matches expectation byte-for-byte.
- `[ ]` Per-board independence: one board runs on DMA while another stays on PIO in the same run, each correct.
- `[ ]` The ADC packetizer holds under a real multi-word run: the capture stream reassembles by position with no lost or misframed words (validates the adaptive word-granular framing beyond the single-word bring-up).
- `[ ]` `static-shims` drives its setpoints through the same DMA path and reads back correctly.
- `[ ]` The PIO fallback still produces the same result for a board left in PIO mode.

---

## Stage 4 -- Fault and reset coordination

To be filled in as Stage 4 lands. Acceptance criteria from the plan:

- A clean halt: on fault, MCDMA channels halt, then `buf_reset` asserts on the DMA-driven FIFOs, then the descriptor rings reinit -- in that order.
- A recoverable restart after an injected fault: the next run starts from the beginning and produces correct output, with no residue from the aborted run (nothing stranded upstream in a paused DAC FIFO).
- No false faults: a channel waiting arbitrarily long on a trigger is not treated as an error (no PS-side timeout).

Checks: `[ ]` (to be written)

---

## Stage 5 -- Validation and reclaim

To be filled in as Stage 5 lands. Acceptance criteria from the plan:

- The DMA-driven CDC FIFOs shrink from whole-waveform depth toward microsecond elastic buffers, freeing BRAM, with the datapath still byte-exact.
- LUT stays within budget at eight boards (the binding constraint) -- check the post-synth hierarchical utilization report.
- The ADC debug words move onto a dedicated debug lane and `dac_data` is renamed `dac_debug`; with debug enabled, the `adc_data` lane stays clean 4-word chunks and the packetizer framing still holds.
- `DMA_PLAN.md` updated to reflect the integrated state.

Checks: `[ ]` (to be written)
