# ADC Packetizer Core

The `adc_packetizer` module frames a plain ADC read-side FIFO stream into chunk-atomic AXI4-Stream packets for the S2MM side of the DMA datapath. It replaces `axis_fifo_bridge` on the ADC read port, because the framing decision needs the FIFO's fill count and read-enable together and `axis_fifo_bridge` forwards `tdata` only, with no `tlast` or `tdest`.

## Why it exists

The ADC data lane carries fixed-size chunks: one ADC read op captures eight 16-bit channels, packed into `CHUNK_WORDS` 32-bit FIFO words (four, for this instrument). That chunk is the atomic unit -- a sample-set that straddled a packet boundary would be an internal corruption -- so every `tlast` lands on the last word of a chunk.

Packet size is adaptive rather than fixed. The downstream mux is packet-atomic: it holds a granted board until it sees `tlast`, so a fixed chop would strand the mux whenever the granted board's FIFO momentarily runs dry, stalling every other board (head-of-line blocking) and risking the ADC-overflow shutdown. Instead the packetizer keeps draining while a whole further chunk is queued and closes the packet the instant the board's available data runs out, releasing the mux immediately. This also self-flushes the tail of a run: when the ADC stops and the FIFO drains, the "no further chunk" condition closes the final packet on its own, so `tlast` needs no separate end-of-sequence signal.

The framing rests on the FIFO's read-side fill count, `fifo_count_rd_clk`, which lives in this same clock domain. That count derives from a Gray-synchronized pointer, so it can lag but never over-report -- the worst case is closing a packet one chunk early, which is harmless.

## Operation

`tdest` is tied to `BOARD_INDEX`, so the mux and MCDMA route this board's samples back to its own S2MM buffer.

The packetizer tracks the word position within the current chunk and the number of chunks completed in the current packet. A chunk is only started (its first word forwarded) when a whole chunk is resident in the FIFO (`avail_chunks >= 1`, where `avail_chunks = fifo_count_rd_clk >> log2(CHUNK_WORDS)`); once started, the remaining words are guaranteed present because the count only grows, so a chunk never stalls mid-way. At the first word of a chunk it snapshots whether a further whole chunk is already queued (`avail_chunks >= 2`). On the last word of a chunk it asserts `tlast` when that snapshot showed no successor, or when the packet has reached `MAX_CHUNKS`.

`MAX_CHUNKS` has one hard ceiling: `MAX_CHUNKS * CHUNK_WORDS` must stay at or under the mux's `ARB_ON_MAX_XFERS` backstop (1024 beats in the reference design), or the mux re-arbitrates mid-packet and breaks packet-atomicity. Within that ceiling a generous cap gives large, descriptor-cheap packets under load at no real fairness cost.

## Parameters

- `DATA_WIDTH` (integer): AXIS/FIFO word width (default: `32`).
- `DEST_WIDTH` (integer): `tdest` width (default: `8`).
- `FIFO_COUNT_WIDTH` (integer): width of `fifo_count_rd_clk`, which is the FIFO's `ADDR_WIDTH + 1` (default: `14`, for a 2^13-deep FIFO).
- `CHUNK_WORDS` (integer): words per atomic ADC chunk; must be a power of two (default: `4`).
- `MAX_CHUNKS` (integer): packet cap in chunks; keep `MAX_CHUNKS * CHUNK_WORDS` at or under the mux backstop (default: `256`).
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
- `m_axis_tvalid` (output): data valid.
- `m_axis_tready` (input): downstream ready.
- `m_axis_tlast` (output): end of packet, asserted only on a chunk boundary.
- `m_axis_tdest` (output): constant `BOARD_INDEX`.

## Notes

- The core reads the FIFO with first-word-fall-through timing: `m_axis_tdata` presents the FIFO head directly and `fifo_rd_en` pops on each accepted beat, matching how `axis_fifo_bridge` drives the same `fifo_async` read port.
- `tvalid` is deasserted at a chunk boundary until a whole chunk is available, so the mux is never handed a partial chunk or empty-FIFO garbage.
- The chunk-atomic guarantee holds only while the ADC data lane carries nothing but whole `CHUNK_WORDS` sample-sets; interleaved debug words on the lane would break it (they belong on a separate debug lane).
