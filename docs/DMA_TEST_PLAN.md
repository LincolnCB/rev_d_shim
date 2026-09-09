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

## Stage 2 -- PL datapath (MCDMA plus routing), mmap path preserved

Stage 2 is built in sub-steps, each leaving a working, testable tree. Stage 2a
lands the MCDMA engine, the 64-bit HP0 memory path, and its non-root control
window, with the 16 streams wired as a per-channel loopback so the engine can be
brought up byte-exact before the real SPI datapath is rewired onto it (2b) and
the completion/error interrupts are folded into `hw_manager` (2c). The existing
mmap FIFO datapath is untouched in 2a.

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
Expect: same output as a pre-2a build -- state `Idle`, SPI clock correct, boards present as built. Adding the MCDMA on GP0/HP0 does not disturb the GP1 mmap FIFO path.
Result: `[ ]`

**Next step -- on-target Stage 2a bring-up.** The host checks (2a.1, 2a.2) pass
from the build artifacts, so the design and device tree are correct. The
remaining Stage 2a work is a boot on the snickerdoodle black to run the three
target checks, in order: 2a.3 (`/dev/mcdma` binds non-root -- `dmesg | grep -i
mcdma` and `ls -l /dev/mcdma`), 2a.4 (`mcdma-loopback` byte-exact on all four
channels, then `mcdma-loopback 1` alone), and 2a.5 (`status` shows the SPI
datapath unchanged). If 2a.4 round-trips, the MCDMA engine, the HP0 64-bit path,
the TDEST demux/mux routing, and non-root control are all proven in the rev_d
context, and Stage 2b (the real per-board datapath: MM2S demux, S2MM
packet-atomic mux, the new ADC packetizer core, the `axis_fifo_bridge` adapters,
and the per-board `datapath_mode` mux/gating in `axi_sys_ctrl`, defaulting to
mmap) can begin. If a channel fails, the `mcdma-loopback` failure dump (per-`dst`
head bytes, per-channel MCDMA status) tells starvation (all-zero `dst`) apart
from misrouting (another channel's high byte in `dst`).

### Stage 2b/2c acceptance criteria

To be turned into concrete checks as the real datapath (2b) and the interrupt
fold (2c) land. From the plan:

- Byte-exact loopback-style transfer on the DMA-driven FIFOs (a known pattern pushed MM2S through the DAC-command FIFO and/or captured S2MM from the ADC-data FIFO comes back identical).
- The mmap fallback still works with a board's `datapath_mode` bit set to PS -- the same tools, same result, on the un-migrated path.
- Per-board mode selection: one board on DMA while the rest stay on mmap, each independently.
- Wrong-mode access (mmap poke at a DMA-mode board, or vice versa) does not crash and does not corrupt: it returns `OKAY`, discards, and raises `mode_viol` into `hw_manager`, which drives the normal coordinated shutdown (status word plus interrupt), not a `SIGBUS`.
- ADC packetizer framing: packets always close on a 4-word chunk boundary; a momentarily-empty board releases the mux (no head-of-line stall); the run tail self-flushes when the FIFO drains; packet size never exceeds the mux `ARB_ON_MAX_XFERS` backstop.
- The sixteen MCDMA completion/error lines fold into `hw_manager` and ride its single interrupt -- no separate DMA interrupt path.

Checks: `[ ]` (to be written)

---

## Stage 3 -- Software

To be filled in as Stage 3 lands. Acceptance criteria from the plan:

- A real waveform plays end to end through the DMA path: prebuffer the `dac_cmd` sequence, reserve the `adc_data` capture region, set each board's `datapath_mode` to DMA, `sync_for_device`, arm, release the trigger, wait on the folded `hw_manager` interrupt, then on a normal end `sync_for_cpu` and read back the ADC data -- the readback matches expectation byte-for-byte.
- The run-controller wakes once on run-end-or-fault, not by spin-polling; a local abort (SIGINT folded into the eventfd) unblocks the same wait as a hardware fault.
- `shim-test` low-level DMA commands work for bring-up and debugging.
- Shared MCDMA-mover library is used by both `waveform` and `static-shims`.

Checks: `[ ]` (to be written)

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
