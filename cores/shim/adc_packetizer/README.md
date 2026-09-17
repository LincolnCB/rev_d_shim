# ADC Packetizer Core

The `adc_packetizer` module frames a plain ADC read-side FIFO stream into AXI4-Stream packets for the S2MM side of the DMA datapath. It replaces `axis_fifo_bridge` on the ADC read port, because the framing decision needs the FIFO's fill count and read-enable together and `axis_fifo_bridge` forwards `tdata` only, with no `tlast` or `tdest`.

## Why it exists

The ADC data lane carries variable-length reads: a full read op captures eight 16-bit channels packed into four 32-bit FIFO words, while a single-channel read (`ADC_RD_CH`) writes just one word. There is no fixed alignment across the stream, so packets are not tied to sample-set boundaries -- a packet may close on any word, and a read may be split across packets. Software reassembles the samples from the captured buffer by position, so a split is harmless.

Packet size is adaptive rather than fixed. The downstream mux is packet-atomic: it holds a granted board until it sees `tlast`, so a fixed chop would strand the mux whenever the granted board's FIFO momentarily runs dry, stalling every other board (head-of-line blocking) and risking the ADC-overflow shutdown. Instead the packetizer keeps draining while words are resident and closes the packet the instant the board's available data runs out, releasing the mux immediately. This also self-flushes the tail of a run: when the ADC stops and the FIFO drains, the "no more words" condition closes the final packet on its own, so `tlast` needs no separate end-of-sequence signal. A slow trickle where the reader keeps pace with the writer simply yields single-word packets.

The framing rests on the FIFO's read-side fill count, `fifo_count_rd_clk`, which lives in this same clock domain. That count derives from a Gray-synchronized pointer, so it can lag but never over-report -- the worst case is closing a packet one word early, which is harmless.

## Operation

`tdest` is tied to `BOARD_INDEX`, so the mux and MCDMA route this board's samples back to its own S2MM buffer.

The packetizer reads the FIFO with first-word-fall-through timing and forwards a word on every accepted beat. It counts the words committed to the current packet and asserts `tlast` when either this is the last resident word (`fifo_count_rd_clk <= 1`, the board ran dry) or the packet has reached `MAX_PACKET_WORDS`. Because the count can lag but never over-reports, the "last resident" test at worst closes a packet one word early; a new packet then carries whatever arrives next.

`MAX_PACKET_WORDS` has one hard ceiling: it must stay at or under the mux's `ARB_ON_MAX_XFERS` backstop (1024 beats in the reference design), or the mux re-arbitrates mid-packet and breaks packet-atomicity. Within that ceiling a generous cap gives large, descriptor-cheap packets under load at no real fairness cost.

## Parameters

- `DATA_WIDTH` (integer): AXIS/FIFO word width (default: `32`).
- `DEST_WIDTH` (integer): `tdest` width (default: `8`).
- `FIFO_COUNT_WIDTH` (integer): width of `fifo_count_rd_clk`, which is the FIFO's `ADDR_WIDTH + 1` (default: `14`, for a 2^13-deep FIFO).
- `MAX_PACKET_WORDS` (integer): packet cap in words; keep it at or under the mux backstop (default: `256`).
- `BOARD_INDEX` (integer): the constant `tdest` value emitted for this board's stream (default: `0`).

## Ports

### Clock and Reset

- `aclk` (input): clock (the FIFO read-side / 100 MHz AXI domain).
- `aresetn` (input): active-low reset.

### FIFO Read Side

- `fifo_rd_data` (input): data from the FIFO read port (first-word-fall-through).
- `fifo_rd_en` (output): read enable; asserted on each accepted beat.
- `fifo_empty` (input): FIFO empty indicator.
- `fifo_count_rd_clk` (input): read-side fill count from `fifo_async`.

### AXIS Manager (to the S2MM mux)

- `m_axis_tdata` (output): stream data.
- `m_axis_tvalid` (output): data valid (asserted whenever the FIFO is non-empty).
- `m_axis_tready` (input): downstream ready.
- `m_axis_tlast` (output): end of packet, asserted on the last resident word or at the cap.
- `m_axis_tdest` (output): constant `BOARD_INDEX`.

## Notes

- The core reads the FIFO with first-word-fall-through timing: `m_axis_tdata` presents the FIFO head directly and `fifo_rd_en` pops on each accepted beat, matching how `axis_fifo_bridge` drives the same `fifo_async` read port.
- Packets carry no sample-set framing, so the S2MM capture buffer is a flat word stream that software slices back into reads. Interleaved debug words on the ADC data lane would land in that buffer as non-sample data, so debug belongs on a separate lane.
