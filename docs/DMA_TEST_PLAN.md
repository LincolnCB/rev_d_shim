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
Run: `find tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux -path '*lib/modules*/updates/*.ko' | sort`
Expect: `pl-reg.ko`, `pl-irq.ko`, and `u-dma-buf.ko` all present under `.../updates/`.
Result: `[PASS]` 2026-09-03 -- all three present under `lib/modules/6.6.40-xilinx-.../updates/`.

**1.2.b -- The device tree carries both regions and both udmabuf nodes (host).**
Run: decompile the built dtb and inspect the nodes --
`dtc -I dtb -O dts tmp/snickerdoodle_black/1.0/rev_d_shim/petalinux/build/tmp/work/zynq_generic_7z020-xilinx-linux-gnueabi/linux-xlnx/*/recipe-sysroot/boot/devicetree/system-top.dtb 2>/dev/null | grep -A6 'shim_dma\|udmabuf'`
Expect: `shim_dma_desc@30000000` reg `<0x30000000 0x400000>`, `shim_dma_data@30400000` reg `<0x30400000 0x4000000>`, `udmabuf0` -> data region (size `0x4000000`), `udmabuf1` -> desc region (size `0x400000`), memory-region phandles matching.
Result: `[PASS]` 2026-09-03 -- addresses, sizes, and phandles all as intended; `SHIM_DMA_*` macros expanded correctly.

**1.2.c -- The modules load at boot (target).**
Run: `lsmod | grep -E 'pl_reg|pl_irq|u_dma_buf'`
Expect: all three listed (loaded names use underscores).
Result: `[ ]`

**1.2.d -- u-dma-buf probed both regions without the CMA-alignment failure (target).**
Run: `dmesg | grep -iE 'u-dma-buf|udmabuf|reserved_mem'`
Expect: udmabuf0 and udmabuf1 register cleanly; no `of_reserved_mem_device_init failed return=-22` and no other probe error. This is the alignment gotcha the dtsi comment calls out -- a misaligned or too-small reusable region fails exactly here.
Result: `[ ]`

**1.2.e -- The DMA device nodes appear with the intended geometry (target).**
Run: `ls -l /dev/udmabuf0 /dev/udmabuf1` then
`for d in udmabuf0 udmabuf1; do echo "$d:"; cat /sys/class/u-dma-buf/$d/phys_addr /sys/class/u-dma-buf/$d/size; done`
Expect: both nodes exist. `udmabuf0` (data) phys_addr `0x30400000`, size `67108864` (64 MB). `udmabuf1` (desc) phys_addr `0x30000000`, size `4194304` (4 MB). These are the values the software reads at runtime instead of hardcoding.
Result: `[ ]`

**1.2.f -- The reserved regions took the DDR out of the kernel's hands (target).**
Run: `cat /proc/iomem | grep -iE 'reserved|3000|3040'` (and optionally `dmesg | grep -i 'reserved'`)
Expect: the `0x30000000` and `0x30400000` regions show up as reserved, not general System RAM.
Result: `[ ]`

**1.2.g -- The existing SPI datapath is unchanged (target).**
Run: exercise the current tools exactly as before the port (they still use `/dev/mem` at this point -- steps 3-5 have not moved them yet). A normal DAC-out / ADC-in check through `shim-test` or the usual bring-up sequence.
Expect: identical behavior to a pre-port build -- boot self-test passes, a known sequence produces the same DAC output and ADC readback. Steps 1-2 must be invisible to the datapath.
Result: `[ ]`

Note: `pl-reg` and `pl-irq` load here but bind nothing yet -- no node carries their compatible until step 3 (register windows) and step 4 (the hw_manager interrupt). Their real bring-up checks live in those steps below.

### Step 3 -- Register windows onto pl-reg (map_memory.c migration)

To be filled in when step 3 lands. Expected checks:

**1.3.a -- Each addressed PL block appears as a named /dev node (target).**
Expect: `/dev/<label>` misc devices named from the Vivado instance labels, one per register window that `map_memory.c` used to reach through `/dev/mem`, each mode `0666`. Enumerate against the block design's addressed IPs.
Result: `[ ]`

**1.3.b -- The tools run fully non-root (target).**
Expect: `shim-test` / `waveform` / `static-shims` / `status` open their register windows as a normal user, no `/dev/mem`, no permission error.
Result: `[ ]`

**1.3.c -- Register reads match the old path (target).**
Expect: a spot-check of known registers (e.g. a status/ID word) reads the same value through the `pl-reg` node as it did through `/dev/mem`.
Result: `[ ]`

### Step 4 -- The hw_manager interrupt onto pl-irq (sys_sts.c migration)

To be filled in when step 4 lands. Expected checks:

**1.4.a -- The interrupt is delivered non-root through the pl-irq node (target).**
Expect: the `hw_manager` interrupt arrives via its `/dev/<label>` pl-irq device (replacing `/dev/uio0`), readable/pollable by a normal user; a deliberate status change wakes a `poll()`/`read()`.
Result: `[ ]`

**1.4.b -- Re-arm works and there is no interrupt storm (target).**
Expect: after handling, re-arming (`write` of 1) re-enables delivery; a level-triggered line does not refire until the source is cleared.
Result: `[ ]`

### Step 5 -- Boot-script permissions on the udmabuf nodes

To be filled in when step 5 lands. Expected checks:

**1.5.a -- The udmabuf nodes and their sync controls are world-accessible (target).**
Expect: after the boot service runs, `/dev/udmabuf0` and `/dev/udmabuf1` are mode `0666`, and the `sync_for_cpu` / `sync_for_device` sysfs files under `/sys/class/u-dma-buf/udmabuf*/` are writable by a normal user.
Result: `[ ]`

**1.5.b -- A non-root program can map and sync a buffer end to end (target).**
Expect: open `/dev/udmabuf0`, `mmap` it, write a pattern, `sync_for_device`, read back, `sync_for_cpu`, all as a normal user with no error.
Result: `[ ]`

### Stage 1 exit

**1.X -- Non-root, plumbing present, datapath intact.**
Expect, all together: every tool runs non-root; `/dev/udmabuf0`, `/dev/udmabuf1`, and the `pl-reg`/`pl-irq` nodes are present and correctly owned; the reserved regions read back the intended address and size; and a full datapath run is unchanged from a pre-port build. Stage 1 is independent of the DMA datapath, so this should be solid before Stage 2 starts.
Result: `[ ]`

---

## Stage 2 -- PL datapath (MCDMA plus routing), mmap path preserved

To be filled in as Stage 2 lands. Acceptance criteria from the plan, to turn into concrete checks:

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
