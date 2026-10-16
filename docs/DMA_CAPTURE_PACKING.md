# Capture Packing Design Notes

Design notes for how the S2MM ADC capture packs data into RAM. The v1 mechanism is implemented
(the RTL core `adc_rate_ctrl` plus the software mover; cocotb-passed, hardware build pending)
and summarized in `DMA_PLAN.md`, with the detail kept here. The v2 streaming prerequisite
remains open (see Remaining open items).

## The problem in one line

The capture has to work at both extremes from one mechanism: a flat-out max-rate burst that
fills RAM as fast as possible, and a word-by-word trickle -- without running out of
descriptors in between, and without capping how much of RAM the PL can fill just because
software is slow to read.

## What the per-word ring did

The original S2MM ring used one 64-byte descriptor per single 4-byte ADC word (`dma_wave_add`
in `software/*/src/sys/dma_ctrl.c`). That is 16x descriptor overhead, so the 8 MB descriptor
region (`udmabuf1`) ran dry at 131,072 words = 512 KB of ADC data while the 128 MB data region
(`udmabuf0`) sat ~99.6% empty -- descriptors were exhausted long before RAM filled. Packing
thin unconditionally was the bug; per-packet descriptors plus density control (below) fix it.

## Mechanics this rests on (the facts)

- Two DDR regions: data region (`udmabuf0`, 128 MB) holds the samples; descriptor region
  (`udmabuf1`, 8 MB) holds the scatter-gather descriptors (64 B each). The two are balanced at
  the packet size (below), so neither is the sole scarce one.
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
- The adc_data FIFO is 2^10 = 1024 words deep (shrunk from 2^13 in the Stage 5 reclaim, still
  well above `MAX_PACKET_WORDS`). Per-board ADC rate tops out ~2 MB/s = ~500k words/s, so
  filling from a 256-word packet up to the 1024 ceiling is ~1.5 ms of headroom at max rate.

## The region-balance number (why 256 is a sweet spot)

At `MAX_PACKET_WORDS = 256`, a full packet buffer is 1024 B. 131,072 descriptors x 1024 B =
exactly 128 MB = the data region. So with full packets the two regions are perfectly matched:
you exhaust both at the same moment, 128 MB of ADC data, a 256x improvement over the per-word
ring's 512 KB. Below 256-word packets the descriptor region binds first; above 256 (raising
`MAX_PACKET_WORDS`) the data region binds first and descriptors become abundant. 256 is the
balance point for the current 8 MB / 128 MB split.

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
a single comparison in hardware (backlog vs a fixed threshold, the packet size), not a
decision software has to make per read -- see Mechanism.

## Mechanism: decouple density from flow control

The clean framing splits the job into two orthogonal mechanisms instead of overloading one.

Flow control = the `TAILDESC` credit window. Software advances `TAILDESC` as it drains and
recycles descriptors. Credits reflect only one thing: is there RAM/descriptor space free to
write into. When RAM fills (software behind or idle), credits run out, the engine stalls, the
FIFO accumulates, and if it overflows that is the detected ADC-overflow error (hw_manager
shutdown) -- the intended failure mode, not something to design around. Credits are never
used to control density.

Density is chosen by one comparison: the backlog against a threshold fixed at the packet size
(256 words). If backlog >= threshold the core accumulates -- the packetizer holds `tvalid` and
lets the FIFO fill toward `MAX_PACKET_WORDS` before draining the whole dense packet. If
backlog < threshold it drains eagerly -- forwards whatever is resident now and closes the
packet at `fifo_count_rd_clk <= 1` (the thin behavior). `MAX_PACKET_WORDS` always caps a packet
regardless. A mode flip mid-accumulation is harmless: if the backlog drops below the threshold
while the FIFO holds, say, 100 words, the packet just closes at 100 and drains. Tying the
threshold to the packet size is also what makes the run tail always flush: a final sub-packet
is by definition smaller than a packet, so the backlog it represents is below the threshold and
the core drains it eagerly rather than waiting for an accumulation that can never complete. The
threshold is a compile-time parameter (`adc_rate_ctrl`'s `PACKET_WORDS`, driven from the same
value as the packetizer's `MAX_PACKET_WORDS`), not a register -- software never sets it. From
reset every backlog is zero, below the threshold, so each board starts in eager drain and ramps
to dense only as real capture data backs up.

The backlog is a single running count the core holds per board, not two cumulative totals. The
packetizer adds each word it emits (one per beat) as the data is written, and software
subtracts the words it has just read by writing that delta into a per-board register as a
mechanical part of its drain loop; the core's saturating subtract floors the backlog at zero.
Software never decides density -- it only reports how much it read, in word-count deltas.

This is what makes it un-missable. When software goes idle it stops reporting reads, but the
packetizer keeps adding emitted words, so the backlog climbs past the threshold on its own and
the core packs at max density with no software action -- RAM fills densely until full, then the
FIFO accumulates and overflows (the intended error). Because the write side is tracked in
hardware, reporting reads late only makes the backlog look bigger, biasing toward denser
packing (safe, conserves descriptors), never toward overflow or lost data. No deadline, and
nothing for software to forget. The scanner-side unknowns never enter in -- density depends
only on the read/write gap.

Keeping density out of the credit mechanism is deliberate: the alternative (time `TAILDESC`
releases in software to force FIFO accumulation) overloads credits for two jobs and brings
back a soft real-time deadline. The threshold split has no tight software deadline (hardware
enforces density; the only overflow is the legitimate RAM-full one) and keeps credits as clean
flow-control, which is also the right base for v2 streaming. The RTL cost is modest: the shared
`adc_rate_ctrl` AXI-Lite core (per-board backlog registers, the read-delta subtract, and the
backlog-vs-threshold compare against the fixed `PACKET_WORDS` parameter) measures ~447 LUT for
all eight boards, plus a couple of LUT per `adc_packetizer` for the dense-hold logic (18 LUT
each).

The per-board read-delta and backlog registers live in a small live-writable control block (the
`adc_rate_ctrl` AXI-Lite window), not behind the `sys_ctrl` config lock, since the read-delta
register is written during a run; the threshold is not a register at all. End-of-run flush falls
out for free: as software catches up, the backlog drops below the threshold, the core switches
to eager drain,
and the FIFO tail self-flushes -- no explicit flush write.

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

The tracker is shaped like the fill count on an ordinary PL FIFO, but over the DDR buffer: the
backlog rises as the packetizer emits words and falls as software reports reads, so it is the
buffer occupancy. Both updates land on the same 100 MHz aclk the packetizer and AXI control
already share -- the emit side adds one word per beat, the read side subtracts a software-
written delta -- so there is no clock-domain crossing and no buffering. It is a scalar per
board, a handful of registers, so no BRAM is needed for it.

The packetizer then does with the backlog what it already does with the FIFO count: compare it
to the threshold, accumulating to `MAX_PACKET_WORDS` when the backlog is at or above it and
draining eagerly otherwise. One subtlety: the backlog counts words the packetizer has emitted,
which the DMA lands in RAM a touch later, so it can read slightly high versus what is actually
readable -- harmless, since an over-estimate only biases toward denser packing. Software uses
the real descriptor-completion status, not this counter, to know what it may read.

## Shared, needed regardless: per-packet descriptors + compaction

Whatever controls density, the descriptor ring has to change from one-word buffers to
per-packet buffers:
- Each descriptor's buffer sized to `MAX_PACKET_WORDS` (1024 B), so a dense packet lands in
  one descriptor; a thin packet partially fills its buffer and the engine records the real
  length.
- Readback walks completed descriptors, reads each one's actual transferred length, and
  compacts the partially-filled buffers into a contiguous sample stream. Replaces today's
  contiguous `memcpy` in `dma_wave_avail_board` / `dma_wave_read_board`.
- A packet larger than its buffer would spill into the next descriptor (engine closes the
  current one on buffer-full without EOF, continues into the next); compaction handles that via
  the EOF bit + length. In practice `MAX_PACKET_WORDS` caps every packet at the buffer size, so
  this does not arise.

## Knobs and their ceilings

- Backlog threshold: the accumulate-vs-drain level, fixed at the packet size as the
  `adc_rate_ctrl` `PACKET_WORDS` parameter (driven from the same value as the packetizer's
  `MAX_PACKET_WORDS`). Not a register and not a runtime knob.
- `MAX_PACKET_WORDS` (RTL param, currently 256): caps a single packet/descriptor and is the
  packing threshold. Must stay under the mux `ARB_ON_MAX_XFERS` (1024) and below the FIFO depth
  (now 1024). Raising it makes each descriptor denser and shifts the bind point from
  descriptors to the data region; move it and `adc_rate_ctrl`'s `PACKET_WORDS` together (the
  block design feeds both from one value).
- DDR region sizes (`SHIM_DMA_*` defines): the absolute cap. The two regions (8 MB descriptors,
  128 MB data) use 136 MB of the 1 GB, so there is room to grow both for more than 128 MB of
  capture; grow them together to keep the balance at the chosen `MAX_PACKET_WORDS`.

## Plan and branching

The work is staged so there is a clean, mergeable stable point partway through. v1 is the first
point the dev branch merges back.

v1 -- implemented (cocotb-passed, hardware build pending). The tool behaves as it did before,
but with the capture backed by the DMA extended buffer: per-packet descriptors + compaction
readback, the per-board backlog (packetizer emit-count minus software read-deltas), and the
backlog-vs-threshold compare driving accumulate-vs-drain, all in the shared `adc_rate_ctrl`
core. The target is `waveform` end-of-run readback, which keeps the testing self-contained. The
capture ring is a fixed ring sized to the whole run, so no descriptor recycling (append-at-tail)
is needed -- per-packet packing makes a waveform-sized capture fit. What remains for v1 is the
hardware run; the RTL is cocotb-verified and the software host-compiles.

v2 -- a new dev branch after the v1 merge. The fixed ring becomes a recycled circular buffer
drained continuously, so capture length is bounded by drain rate rather than ring size (live /
streaming capture). The density mechanism is unchanged -- it only looks at the backlog. The one
new hardware dependency is append-at-tail on a running S2MM channel, whose behavior is best
pinned down on a toy testbed (something like `ex05_dma`) rather than the full system before it
goes into rev_d (see Remaining open items for where to confirm it).

LUT reclaim ran first. The Stage 5 reclaim in `DMA_PLAN.md` (shrink the DMA-driven CDC FIFOs,
merge the per-board bridges, consolidate the register AXI) landed before the v1 core, freeing
LUT headroom and the BRAM the deep FIFOs held (see `RESOURCE_RECLAIM.md`).

## Settled decisions

- Overflow is the failure mode, not a thing to avoid: when RAM fills and software has not
  drained, credits run out, the FIFO accumulates, and an overflow is the detected ADC-overflow
  error (hw_manager shutdown). No "park at end-of-region" special case.
- Density is decoupled from flow control: a hardware backlog-vs-threshold compare in the
  packetizer sets packing; `TAILDESC` credits are pure backpressure. The threshold is fixed at
  the packet size, which also guarantees the run tail flushes.
- Density is driven by a hardware per-board backlog (the packetizer adds each emitted word,
  software subtracts the words it reads), not by software setting density.
- First target is `waveform` end-of-run readback (v1): fixed ring, no append-at-tail.
  Live/continuous capture is v2.

## Remaining open items

The v1 implementation settled the earlier sub-decisions: the backlog is a 32-bit per-board
register (covers far more than the 128 MB region), the control block is the `adc_rate_ctrl`
AXI-Lite window (per-board read-delta and backlog registers, with the threshold a compile-time
`PACKET_WORDS` parameter rather than a register), and the accumulate/drain compare is the binary
form (no graded mapping). The one item left is the v2 prerequisite.

- v2 prerequisite (does not block v1): does MCDMA S2MM support append-at-tail (advancing
  `TAILDESC`) on a running channel? Find out from PG288 (the per-channel tail-descriptor
  register behavior on a running channel), the mainline `xilinx_dma.c` cyclic/continuous S2MM
  path, and a small hardware test, in that order.

## Diagnostics and tooling (future work)

- `memory_log` file (behind a flag): log the per-descriptor word counts for a run -- for each
  descriptor, the words written (DAC, MM2S) and read (ADC, S2MM). This makes the actual packing
  density and the thin/dense boundaries visible after a run, so the capacity estimate and the
  allocation can be checked against what the hardware really did (how many descriptors ended up
  thin vs dense, where the backlog held). Pairs with the capacity precheck's numbers.
- `command_log` -- rework `--verbose`: today verbose mode is scattered inline prints. The goal is
  a structured, log-style output (a command/event log written to a file behind a flag) that
  captures the sequence of commands and state transitions, so weird behavior on the bench can be
  reconstructed after the fact instead of scraped from interleaved stdout. Applies beyond the DMA
  path; this is where the need surfaced.
