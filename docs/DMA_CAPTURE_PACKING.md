# Capture Packing Design Notes

Design notes for how the S2MM ADC capture packs data into RAM. The mechanism is settled (see
Mechanism); a summary graduates into `DMA_PLAN.md` and the detail stays here. A few
implementation sub-decisions and the v2 streaming prerequisite remain open (see Remaining
open items).

## The problem in one line

The capture has to work at both extremes from one mechanism: a flat-out max-rate burst that
fills RAM as fast as possible, and a word-by-word trickle -- without running out of
descriptors in between, and without capping how much of RAM the PL can fill just because
software is slow to read.

## What's broken today

The S2MM ring uses one 64-byte descriptor per single 4-byte ADC word (`dma_wave_add` in
`software/*/src/sys/dma_ctrl.c`). That is 16x descriptor overhead, so the 4 MB descriptor
region (`udmabuf1`) runs dry at 65,536 words = 256 KB of ADC data, while the 64 MB data
region (`udmabuf0`) sits ~99.6% empty. We run out of descriptors long before RAM fills.
Packing thin unconditionally is the bug.

## Mechanics this rests on (the facts)

- Two DDR regions: data region (`udmabuf0`, 64 MB) holds the samples; descriptor region
  (`udmabuf1`, 4 MB) holds the scatter-gather descriptors (64 B each). Descriptor region is
  the scarce one.
- In scatter-gather S2MM, every packet (every `tlast`) consumes at least one descriptor. The
  engine closes the current descriptor on `tlast` or when its buffer fills, writes back the
  Complete bit and the actual transferred length, then moves to the next descriptor. You
  cannot pack two packets into one descriptor. So descriptors-consumed == packet-count (plus
  spill when a packet is bigger than its buffer).
- Descriptor "freeing" is a software action, not automatic. The engine marks a descriptor
  Complete and leaves it owned by software. The back-channel that hands descriptors back to
  the engine is the `TAILDESC` register: the engine never runs past `TAILDESC`, so the gap
  between `CURDESC` (where the engine is) and `TAILDESC` (where software has released to) is a
  credit window software controls. Advancing `TAILDESC` == returning credits. This is the
  standard tail-chasing / continuous-SG pattern (the plan already assumes append-at-tail for
  MM2S).
- The packetizer (`cores/shim/adc_packetizer`) closes a packet when the FIFO is about to run
  dry (`fifo_count_rd_clk <= 1`) or at `MAX_PACKET_WORDS` (currently 256). Crucially it keeps
  one packet open while the FIFO stays deep, so draining a full FIFO yields one dense packet,
  not many thin ones.
- The adc_data FIFO is 2^13 = 8192 words deep. Per-board ADC rate tops out ~2 MB/s = ~500k
  words/s, so filling from 256 words up to the 8192 ceiling is ~16 ms of headroom at max rate.

## The region-balance number (why 256 is a sweet spot)

At `MAX_PACKET_WORDS = 256`, a full packet buffer is 1024 B. 65,536 descriptors x 1024 B =
exactly 64 MB = the data region. So with full packets the two regions are perfectly matched:
you exhaust both at the same moment, 64 MB of ADC data, a 256x improvement over today's
256 KB. Below 256-word packets the descriptor region binds first; above 256 (raising
`MAX_PACKET_WORDS`) the data region binds first and descriptors become abundant. 256 is the
balance point for the current 4 MB / 64 MB split.

## The control law

Always drain into RAM (RAM is an extended buffer -- fill it). Vary only the packing density,
and tie density to the backlog = (write frontier - software read cursor):

- Big backlog (software idle or far behind): pack dense (accumulate toward `MAX_PACKET_WORDS`
  per descriptor). Software has plenty buffered, so the accumulation latency costs nothing,
  and dense packing is exactly what lets the whole data region fill without exhausting
  descriptors.
- Small backlog (software draining fast, near the frontier): pack thin / eager (drain
  whatever is resident, down to a word) so software gets data with low latency. Thin packing
  is safe here because fast draining recycles descriptors as fast as they are spent.

This unifies the extremes: idle software is the dense, descriptor-efficient, fill-all-of-RAM
case; a hungry fast reader is the thin, low-latency, recycle-as-you-go case. The per-word
exhaustion only happened because we packed thin unconditionally. The switch between the two is
a single comparison in hardware (backlog vs a software-set threshold), not a decision software
has to make per read -- see Mechanism.

## Mechanism: decouple density from flow control

The clean framing splits the job into two orthogonal mechanisms instead of overloading one.

Flow control = the `TAILDESC` credit window. Software advances `TAILDESC` as it drains and
recycles descriptors. Credits reflect only one thing: is there RAM/descriptor space free to
write into. When RAM fills (software behind or idle), credits run out, the engine stalls, the
FIFO accumulates, and if it overflows that is the detected ADC-overflow error (hw_manager
shutdown) -- the intended failure mode, not something to design around. Credits are never
used to control density.

Density is chosen by one comparison: the backlog against a software-set threshold. If
backlog >= threshold the core accumulates -- the packetizer holds `tvalid` and lets the FIFO
fill toward `MAX_PACKET_WORDS` before draining the whole dense packet. If backlog < threshold
it drains eagerly -- forwards whatever is resident now and closes the packet at
`fifo_count_rd_clk <= 1` (today's thin behavior). `MAX_PACKET_WORDS` always caps a packet
regardless. A mode flip mid-accumulation is harmless: if the backlog drops below the
threshold while the FIFO holds, say, 100 words, the packet just closes at 100 and drains. The
threshold is the single software knob, set via AXI.

The backlog it compares against is `write_total - read_total`, two cumulative word counts:
the packetizer bumps `write_total` by the packet size on each `tlast` (it writes the data, so
it knows instantly), and software reports `read_total` -- its cumulative words-consumed --
into a live register as a mechanical part of its drain loop. The core subtracts the two.
Cumulative totals rather than +1/-1 events make it idempotent: a late or missed `read_total`
report only makes the backlog look bigger, and the next report self-corrects. Software never
decides density; it only reports how much it has read.

This is what makes it un-missable. When software goes idle its `read_total` freezes, but the
packetizer keeps bumping `write_total`, so the backlog climbs past the threshold on its own
and the core packs at max density with no software action -- RAM fills densely until full,
then the FIFO accumulates and overflows (the intended error). Because the write side is
tracked in hardware, software reporting `read_total` late only makes the backlog look bigger,
biasing toward denser packing (safe, conserves descriptors), never toward overflow or lost
data. No deadline, and nothing for software to forget. The scanner-side unknowns never enter
in -- density depends only on the read/write gap.

Keeping density out of the credit mechanism is deliberate: the alternative (time `TAILDESC`
releases in software to force FIFO accumulation) overloads credits for two jobs and brings
back a soft real-time deadline. The threshold split has no tight software deadline (hardware
enforces density; the only overflow is the legitimate RAM-full one) and keeps credits as clean
flow-control, which is also the right base for v2 streaming. The RTL cost is small: the backlog
compare plus the fill-count adders per board, tens of LUT on top of the packetizer's current
~17.

The knob is just that backlog threshold -- a set-and-watch value you sweep while testing, not
a per-read decision. (A graded mapping, e.g. an accumulate target of `saturate(backlog >> k)`
instead of a hard accumulate/drain flip, is a possible refinement if the binary form proves
too coarse, but the single threshold is the lean default.) The threshold and the per-board
`read_total` register live in a small live-writable control block -- not behind the
`sys_ctrl` config lock, since `read_total` updates during a run. End-of-run flush falls out
for free: as software catches up, the backlog drops below the threshold, the core switches to
eager drain, and the FIFO tail self-flushes -- no explicit flush write.

### Why the split -- can't software compute the backlog itself?

Software does know what is available to it: it learns the write frontier by reading descriptor
completion status from RAM, and it knows its own read cursor, so it could compute the backlog.
The catch is not computing it but delivering it to the consumer. The density decision runs in
the PL, continuously, and has to be right exactly when software is idle -- that is when it
should ramp to max density and fill RAM. If software fed the backlog to the PL, the write side
would go stale the instant software stopped polling, the moment it is growing fastest. And the
read side is invisible to the PL: software reads the u-dma-buf region straight from DDR with
CPU loads, which the PL never sees, so the PL cannot track reads on its own either. So each
side is tracked by whoever observes it directly and in time -- the PL its own writes, software
its own reads -- and the hardware subtracts them. Neither side can do it alone.

This is not the existing FIFO status register repurposed. That register reports adc_data FIFO
occupancy -- raw data waiting in the PL before the DMA, which the packetizer already reads for
packet boundaries. The backlog is a different pipeline stage: packetized data sitting in RAM,
unread, after the DMA. Both stay and both are useful (the FIFO count for overflow watch, the
backlog for density and for software to monitor); the backlog tracker is additive.

The tracker is shaped exactly like the read/write-pointer fill count on an ordinary PL FIFO,
but over the DDR buffer: `write_total` advances (by the packet size) when the packetizer
completes a packet, `read_total` advances (by the reported amount) when software reports a
read, and the backlog is their difference -- the buffer occupancy. The pointers jump by more
than one, which is just an adder rather than an increment, and both updates land on the same
100 MHz aclk the packetizer and AXI control already share, so there is no clock-domain
crossing and no buffering. It is a scalar per board -- a handful of registers -- so the
soon-to-be-freed BRAM is not needed for it (better left for the FIFO-shrink reclaim).

The packetizer then does with the backlog what it already does with the FIFO count: compare
it to a threshold. It captures the backlog (the occupancy -- the difference, not either
pointer alone) and, if it is at or above the threshold, accumulates to `MAX_PACKET_WORDS`;
otherwise it drains eagerly. One subtlety: `write_total` counts packets the packetizer has
emitted, which the DMA lands in RAM a touch later, so the backlog can read slightly high
versus what is actually readable -- harmless, since an over-estimate only biases toward denser
packing. Software uses the real descriptor-completion status, not this counter, to know what
it may read.

## Shared, needed regardless: per-packet descriptors + compaction

Whatever controls density, the descriptor ring has to change from one-word buffers to
per-packet buffers:
- Each descriptor's buffer sized to `MAX_PACKET_WORDS` (1024 B), so a dense packet lands in
  one descriptor; a thin packet partially fills its buffer and the engine records the real
  length.
- Readback walks completed descriptors, reads each one's actual transferred length, and
  compacts the partially-filled buffers into a contiguous sample stream. Replaces today's
  contiguous `memcpy` in `dma_wave_avail_board` / `dma_wave_read_board`.
- A packet larger than its buffer spills into the next descriptor (engine closes the current
  one on buffer-full without EOF, continues into the next). Compaction handles that via the
  EOF bit + length. Only matters if `start_threshold` or a burst exceeds `MAX_PACKET_WORDS`.

## Knobs and their ceilings

- Backlog threshold: the accumulate-vs-drain level, the single live software knob. Set-and-
  watch; sweep it while testing.
- `MAX_PACKET_WORDS` (RTL param, currently 256): caps a single packet/descriptor. Must stay
  under the mux `ARB_ON_MAX_XFERS` (1024) and below FIFO depth (8192). Raising it makes each
  descriptor denser and shifts the bind point from descriptors to the data region.
- DDR region sizes (`SHIM_DMA_*` defines): the absolute cap. Only 68 MB of 1 GB used today, so
  there is room to grow both regions for more than 64 MB of capture; grow them together to
  keep the balance at the chosen `MAX_PACKET_WORDS`.

## Plan and branching

The work is staged so there is a clean, mergeable stable point partway through. This is a dev
branch; v1 is the first point where it merges back.

v1 -- the first stable merge point. The tool behaves as it did before, but with the capture
backed by the DMA extended buffer: per-packet descriptors + compaction readback, the per-board
`write_total` counter and `read_total` register, the backlog subtract, and the
backlog-vs-threshold compare driving accumulate-vs-drain. The target is `waveform` end-of-run
readback, which keeps the testing self-contained. The capture ring is a fixed ring sized to the
whole run, so no descriptor recycling (append-at-tail) is needed -- per-packet packing makes a
waveform-sized capture fit. This is the whole density-control-core; there is no throwaway
manual-threshold phase. A working `waveform` run on this path ends v1, and the branch merges
back.

v2 -- a new dev branch after the v1 merge. The fixed ring becomes a recycled circular buffer
drained continuously, so capture length is bounded by drain rate rather than ring size (live /
streaming capture). The density mechanism is unchanged -- it only looks at the backlog. The one
new hardware dependency is append-at-tail on a running S2MM channel, whose behavior is best
pinned down on a toy testbed (something like `ex05_dma`) rather than the full system before it
goes into rev_d (see Remaining open items for where to confirm it).

LUT reclaim runs first. The Stage 5 reclaim pass in `DMA_PLAN.md` (shrink the DMA-driven CDC
FIFOs, lean the AXI-Lite interconnects) runs before the v1 core work, to free LUT headroom for
the density-control-core rather than add it to an already-tight design; the FIFO shrink also
frees the BRAM those FIFOs hold.

## Settled decisions

- Overflow is the failure mode, not a thing to avoid: when RAM fills and software has not
  drained, credits run out, the FIFO accumulates, and an overflow is the detected ADC-overflow
  error (hw_manager shutdown). No "park at end-of-region" special case.
- Density is decoupled from flow control: a hardware backlog-vs-threshold compare in the
  packetizer sets packing; `TAILDESC` credits are pure backpressure. The threshold is the
  single live software knob.
- Density is driven by a hardware backlog fill-count (`write_total - read_total`), not by
  software setting density; software only reports `read_total`.
- First target is `waveform` end-of-run readback (v1): fixed ring, no append-at-tail.
  Live/continuous capture is v2.

## Remaining open items

- Counter widths for `write_total` / `read_total` (free-running, wide enough that the
  unsigned difference is always a correct backlog).
- The live control block: where it sits, and global vs per-board threshold (global first).
- Binary accumulate/drain vs a graded `saturate(backlog >> k)` mapping -- binary is the
  default; revisit only if the hard flip proves too coarse on hardware.
- v2 prerequisite (does not block v1): does MCDMA S2MM support append-at-tail (advancing
  `TAILDESC`) on a running channel? Find out from PG288 (the per-channel tail-descriptor
  register behavior on a running channel), the mainline `xilinx_dma.c` cyclic/continuous S2MM
  path, and a small hardware test, in that order.
