# Capture Packing Design Notes

Design notes for how the S2MM ADC capture packs data into RAM. The v1 mechanism (the `adc_rate_ctrl` core plus the software mover) is implemented and hardware-validated; `DMA_PLAN.md` summarizes it. The v2 streaming prerequisite is still open (see Remaining open items).

## The problem

One mechanism has to handle both extremes: a max-rate burst that fills RAM as fast as possible, and a word-by-word trickle -- without running out of descriptors, and without limiting how much RAM the PL can fill because software reads slowly.

The original S2MM ring used one 64-byte descriptor per 4-byte ADC word, a 16x overhead: the 8 MB descriptor region (`udmabuf1`) ran dry at 512 KB of ADC data while the 128 MB data region (`udmabuf0`) sat ~99.6% empty. Per-packet descriptors plus density control fix that.

## Mechanics this rests on

- The data region (`udmabuf0`, 128 MB) holds samples; the descriptor region (`udmabuf1`, 8 MB) holds 64-byte scatter-gather descriptors.
- Every S2MM packet (every `tlast`) consumes at least one descriptor: the engine closes a descriptor on `tlast` or a full buffer, writes back Complete and the transferred length, and moves on. Descriptors consumed == packet count.
- Freeing descriptors is a software action. The engine never runs past `TAILDESC`, so the `CURDESC`-to-`TAILDESC` gap is a credit window software controls by advancing `TAILDESC` (the standard tail-chasing / continuous-SG pattern).
- The packetizer (`cores/shim/adc_packetizer`) closes a packet when the FIFO is about to run dry (`fifo_count_rd_clk <= 1`) or at `MAX_PACKET_WORDS` (256), so draining a full FIFO yields one dense packet.
- The `adc_data` FIFO is 2^10 words. At the ~500k words/s per-board maximum, filling from 256 to 1024 words takes ~1.5 ms.

## Why 256 words

A full 256-word packet is 1024 B, and 131,072 descriptors x 1024 B is exactly 128 MB, so with full packets both regions run out together -- 128 MB of capture, 256x the per-word ring. Smaller packets make the descriptor region bind first; larger ones (raising `MAX_PACKET_WORDS`) make the data region bind first.

## The control law

Always drain into RAM, and vary only the packing density with the backlog (write frontier minus software read cursor):

- Big backlog (software idle or far behind): pack dense, accumulating toward `MAX_PACKET_WORDS` per descriptor. The latency costs nothing, and it's what lets the whole data region fill.
- Small backlog (software keeping up): pack thin, draining whatever is resident, so software gets data with low latency. Fast draining recycles descriptors as fast as they're spent.

The switch is one hardware comparison (backlog vs a fixed threshold equal to the packet size), not a per-read software decision.

## Mechanism: density separate from flow control

Flow control is the `TAILDESC` credit window, and only that: credits say whether there's RAM/descriptor space to write into. When RAM fills, credits run out, the engine stalls, the FIFO backs up, and an overflow is the detected ADC-overflow error (`hw_manager` shutdown) -- the intended failure mode. Credits never control density.

Density is one comparison against a threshold fixed at the packet size (256 words). At or above it, the packetizer holds `tvalid` and lets the FIFO fill toward `MAX_PACKET_WORDS` before draining a dense packet; below it, it drains eagerly and closes at `fifo_count_rd_clk <= 1`. `MAX_PACKET_WORDS` always caps a packet, and a mode flip mid-accumulation just closes the packet early. Because the threshold equals the packet size, a final sub-packet always sits below it and drains, so the run tail always flushes with no explicit flush. The threshold is a compile-time parameter (`adc_rate_ctrl`'s `PACKET_WORDS`, fed from the same value as `MAX_PACKET_WORDS`), not a register. From reset every backlog is zero, so each board starts in eager drain.

The backlog is one running count per board: the packetizer adds each word it emits, and software subtracts each batch it reads by writing the delta to the board's rate lane (word 7 of its page in `axi_board_fifo_array`, writable during a run and outside the `sys_ctrl` config lock); a saturating subtract floors it at zero. Software never chooses density -- it only reports reads.

That makes it impossible to miss. Idle software stops reporting reads, the backlog climbs on its own, and the core packs at max density until RAM fills. Reporting reads late only makes the backlog look bigger, which biases toward denser packing, never toward overflow. There's no software deadline, and the scanner-side rates never enter in.

Timing `TAILDESC` releases in software to force accumulation would overload credits with a second job and bring back a soft real-time deadline; keeping them separate also leaves a clean base for v2 streaming. The RTL cost is the shared `adc_rate_ctrl` core (per-board counters, the subtract and the compare, with no AXI slave) plus the dense-hold logic in each `adc_packetizer` (18 LUT each).

### Why not compute the backlog in software?

Software could compute it from descriptor completion status and its own read cursor, but the PL needs it continuously, and most of all when software is idle -- exactly when a software-fed value would go stale. The PL can't see reads either, since software reads DDR directly with CPU loads. So each side tracks what it observes -- the PL its writes, software its reads -- and the hardware subtracts them.

The backlog is separate from the `adc_data` FIFO status: the FIFO count is raw data waiting before the DMA (used for packet boundaries and overflow watch), while the backlog is packetized data in RAM waiting for software. Both updates land on the 100 MHz `aclk` the packetizer and AXI control share, so there's no clock crossing, and a scalar per board needs no BRAM. The backlog can read slightly high, since the DMA lands emitted words a little later; that only biases toward denser packing. Software uses descriptor completion, not this counter, to know what it may read.

## Per-packet descriptors and compaction

- Each descriptor's buffer is `MAX_PACKET_WORDS` (1024 B), so a dense packet fills one descriptor and a thin one partially fills it, with the real length written back.
- Readback (`dma_wave_avail_board` / `dma_wave_read_board`) walks completed descriptors and compacts the partial buffers into a contiguous sample stream.
- A packet bigger than its buffer would spill into the next descriptor, which compaction handles via EOF + length; `MAX_PACKET_WORDS` caps packets at the buffer size, so it doesn't arise.

## Knobs and their ceilings

- Backlog threshold: fixed at the packet size by `PACKET_WORDS`; not a runtime knob.
- `MAX_PACKET_WORDS` (256): must stay under the mux `ARB_ON_MAX_XFERS` (1024) and the FIFO depth (1024). Raising it shifts the binding limit from descriptors to data; move it and `PACKET_WORDS` together (the block design feeds both from one value).
- DDR region sizes (`SHIM_DMA_*` defines): the absolute cap. The two regions use 136 MB of the 1 GB; grow them together to keep the balance.

## Plan and branching

v1 -- implemented and hardware-validated: per-packet descriptors with compaction, the per-board backlog, and the accumulate-vs-drain compare in `adc_rate_ctrl`, targeting `waveform` end-of-run readback. The capture ring is fixed and sized to the whole run, so no descriptor recycling is needed. On the four-board bench, DMA runs write every sample with peak backlogs of 32-512 words, so dense mode engages and the tail flushes (`DMA_TEST_PLAN.md` 5.1.g, 5.2.g, 5.4.c).

v2 -- a dev branch after the v1 merge: the ring becomes a recycled circular buffer drained continuously, so capture length is set by drain rate rather than ring size. Density control is unchanged. The new dependency is append-at-tail on a running S2MM channel, best pinned down on a toy testbed (like `ex05_dma`) first.

## Settled decisions

- Overflow is the failure mode: when RAM fills, the FIFO backs up and overflows into the ADC-overflow shutdown. No "park at end-of-region" special case.
- Density is set by a hardware backlog-vs-threshold compare; `TAILDESC` credits are pure backpressure. The threshold equals the packet size, which guarantees the tail flushes.
- The backlog is a 32-bit per-board hardware counter (emitted words minus software-reported reads), reached through the rate lane; the compare is binary.
- v1 targets `waveform` end-of-run readback with a fixed ring; live capture is v2.

## Remaining open items

- v2 prerequisite: does MCDMA S2MM support append-at-tail (advancing `TAILDESC`) on a running channel? Check PG288, the mainline `xilinx_dma.c` cyclic S2MM path, and a small hardware test, in that order.

## Diagnostics and tooling (future work)

- `memory_log` (behind a flag): per-descriptor word counts for a run (DAC MM2S and ADC S2MM), to show the real packing density and check the capacity estimate against what the hardware did.
- `command_log`: replace the scattered `--verbose` prints with a structured command/event log written to a file behind a flag, so bench behavior can be reconstructed afterwards. Applies beyond the DMA path.
