# ADC Rate Control Core

The `adc_rate_ctrl` module holds the per-board capture backlog counters that drive the ADC capture density. It is the hardware half of the capture rate-control law (see `docs/DMA_CAPTURE_PACKING.md`): it tracks how much packetized ADC data is sitting unread in DDR for each board and tells that board's `adc_packetizer` whether to pack dense or drain eagerly, comparing each backlog against a fixed packing threshold set at the packet size.

## Why it exists

The S2MM capture has to work at both extremes from one mechanism: a flat-out burst that fills RAM as fast as possible, and a word-by-word trickle, without exhausting descriptors in between. The packing density is tied to the backlog -- the gap between what the packetizer has written to DDR and what software has read back. When software is idle or far behind, the backlog grows and the packetizer packs dense (one descriptor per `MAX_PACKET_WORDS`), which is what lets the whole data region fill without running out of descriptors. When software is draining fast and near the write frontier, the backlog is small and the packetizer drains eagerly for low latency.

Neither side can compute the backlog alone: the packetizer sees its own writes but not software's reads (which are plain CPU loads from the u-dma-buf region, invisible to the PL), and software polling the write side goes stale the instant it stops. So each side reports its own delta and the hardware keeps the running difference. The packetizer bumps a board's backlog up by one per emitted word (`word_emitted`), and software bumps it down by the number of words it has just read, written as a delta to that board's `READ_DELTA` register. The backlog is their difference -- a fill count over the DDR capture buffer.

## Operation

Each board has a 32-bit backlog counter. On every clock it adds that board's `word_emitted` beat and subtracts any `READ_DELTA` write landing that cycle, floored at zero so a stray over-report can never underflow. `accumulate[b]` asserts while `backlog[b] >= PACKET_WORDS`, and that single bit is what the packetizer compares instead of the raw FIFO count: high means accumulate toward `MAX_PACKET_WORDS`, low means drain eagerly. The threshold is the packet size itself (`PACKET_WORDS`, the same value as the packetizer's `MAX_PACKET_WORDS`), a compile-time parameter rather than a register.

Fixing the threshold to the packet size is what makes a run tail always flush: a final sub-packet holds fewer than `PACKET_WORDS` words, so its backlog is below the threshold and the board drains it eagerly rather than waiting for an accumulation that can never complete. From reset every backlog is zero (below the threshold), so each board starts in eager drain and ramps to dense only as real capture data backs up -- software never writes a threshold.

Both sides report deltas, so the backlog is a plain fill count bounded by the capture region (128 MB = 2^25 words). A 32-bit counter covers a backlog up to 16 GiB, far past that, so the width is fixed rather than parameterized.

Each board's backlog is cleared by its own `backlog_rst_n`, wired to that board's `adc_data` FIFO reset. One buffer reset then flushes the FIFO, resets the packetizer, and zeroes this count together, so the capture bookkeeping stays coherent with the data path across a run bracket.

The read/write pointers jump by more than one (an adder, not an increment), and both updates land on the same 100 MHz `aclk` the packetizer and the AXI control share, so there is no clock-domain crossing. The counters are scalars -- a handful of registers per board -- so they add only tens of LUT on top of the packetizers.

## Register map (32-bit words, shown for the default eight boards)

- `0`..`7` -- `READ_DELTA[b]` (WO): writing `N` subtracts `N` from board `b`'s backlog (saturating at 0). Write-triggered; reading it is meaningless.
- `8`..`15` -- `BACKLOG[b]` (RO): board `b`'s current occupancy in words, for monitoring.

In general `READ_DELTA[b]` is at word `b` and `BACKLOG[b]` is at word `NUM_BOARDS + b`.

## Parameters

- `NUM_BOARDS` (integer): number of boards tracked (default: `8`). Sets the width of the per-board port vectors and the register map (`READ_DELTA` at words `0..NUM_BOARDS-1`, `BACKLOG` at `NUM_BOARDS..2*NUM_BOARDS-1`).
- `PACKET_WORDS` (integer): the packing threshold in words (default: `256`), driven from the same value as the packetizer's `MAX_PACKET_WORDS` so the two cannot drift.
- `AXI_ADDR_WIDTH` (integer): AXI4-Lite address width (default: `16`).

## Ports

### Clock and Reset

- `aclk` (input): clock (the 100 MHz AXI / packetizer domain).
- `aresetn` (input): active-low reset.

### Per-board packing control

- `word_emitted` (input, `NUM_BOARDS`): each board's packetizer emit beat; adds one to that backlog.
- `backlog_rst_n` (input, `NUM_BOARDS`): per-board backlog clear, wired to that board's `adc_data` FIFO reset.
- `accumulate` (output, `NUM_BOARDS`): per-board packing select -- 1 packs dense, 0 drains eagerly.

### AXI4-Lite subordinate

Standard AXI4-Lite subordinate port (`s_axi_*`). Writes never fault (`bresp` is always OKAY); `wstrb` is ignored since every register is a whole 32-bit word.

## Notes

- Software never decides density; it only reports `READ_DELTA`. A late or missing report only makes the backlog look larger, biasing toward denser packing (safe, conserves descriptors), never toward overflow or lost data.
- The backlog is a different pipeline stage than the `adc_data` FIFO occupancy the packetizer already reads: that is raw data waiting in the PL before the DMA; the backlog is packetized data sitting in RAM after the DMA. Both are useful and both stay.
