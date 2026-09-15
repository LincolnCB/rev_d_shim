`timescale 1 ns / 1 ps

// Frames an ADC read-side FIFO stream into chunk-atomic AXIS packets for S2MM DMA.
//
// The ADC data lane carries fixed-size chunks -- one ADC read op is eight 16-bit
// channels packed into CHUNK_WORDS 32-bit FIFO words -- and a sample-set must never
// straddle a packet boundary, so tlast only ever lands on the last word of a chunk.
// Packet size is adaptive rather than fixed: the packetizer keeps draining while a
// whole further chunk is queued and closes the packet the moment the board's
// available data runs out, which releases the downstream packet-atomic mux instead
// of stranding it on a momentarily-empty board (head-of-line blocking). A run tail
// self-flushes -- when the FIFO drains, the "no further chunk" condition closes the
// final packet on its own, so no separate end-of-sequence signal is needed.
//
// tdest is tied to BOARD_INDEX so the mux and MCDMA route this board's samples back
// to its own S2MM buffer. MAX_CHUNKS caps a packet so it can never exceed the mux's
// ARB_ON_MAX_XFERS backstop (which would re-arbitrate mid-packet and corrupt framing).
//
// The framing rests on the FIFO's read-side fill count (fifo_count_rd_clk), which is
// in this same clock domain. That count is Gray-synchronized so it can lag but never
// over-report: the worst case is closing a packet one chunk early, which is harmless.
module adc_packetizer #(
  parameter integer DATA_WIDTH       = 32, // AXIS/FIFO word width
  parameter integer DEST_WIDTH       = 8,  // tdest width
  parameter integer FIFO_COUNT_WIDTH = 14, // width of fifo_count_rd_clk (ADDR_WIDTH+1)
  parameter integer CHUNK_WORDS      = 4,  // words per atomic ADC chunk (a power of two)
  parameter integer MAX_CHUNKS       = 256,// packet cap in chunks (<= mux backstop / CHUNK_WORDS)
  parameter integer BOARD_INDEX      = 0   // constant tdest for this board's stream
)(
  input  wire                        aclk,
  input  wire                        aresetn,

  // FIFO read side (fifo_async read port, first-word-fall-through)
  input  wire [DATA_WIDTH-1:0]       fifo_rd_data,
  output wire                        fifo_rd_en,
  input  wire                        fifo_empty,
  input  wire [FIFO_COUNT_WIDTH-1:0] fifo_count_rd_clk,

  // AXIS manager to the S2MM packet-atomic mux
  output wire [DATA_WIDTH-1:0]       m_axis_tdata,
  output wire                        m_axis_tvalid,
  input  wire                        m_axis_tready,
  output wire                        m_axis_tlast,
  output wire [DEST_WIDTH-1:0]       m_axis_tdest
);

  localparam integer CHUNK_SHIFT = $clog2(CHUNK_WORDS);
  // Word-position counter is at least 1 bit wide even for a single-word chunk.
  localparam integer WORD_IDX_WIDTH  = (CHUNK_WORDS <= 1) ? 1 : $clog2(CHUNK_WORDS);
  localparam integer CHUNK_IDX_WIDTH = (MAX_CHUNKS  <= 1) ? 1 : $clog2(MAX_CHUNKS);

  // Parameter validation
  initial begin
    if (DATA_WIDTH <= 0 || DATA_WIDTH % 8 != 0)
      $error("Invalid DATA_WIDTH %0d: must be positive and a multiple of 8.", DATA_WIDTH);
    if (CHUNK_WORDS <= 0 || (CHUNK_WORDS & (CHUNK_WORDS - 1)) != 0)
      $error("Invalid CHUNK_WORDS %0d: must be a positive power of two.", CHUNK_WORDS);
    if (MAX_CHUNKS <= 0)
      $error("Invalid MAX_CHUNKS %0d: must be greater than 0.", MAX_CHUNKS);
    if (FIFO_COUNT_WIDTH <= CHUNK_SHIFT)
      $error("Invalid FIFO_COUNT_WIDTH %0d: must exceed log2(CHUNK_WORDS)=%0d.", FIFO_COUNT_WIDTH, CHUNK_SHIFT);
    if (BOARD_INDEX < 0 || BOARD_INDEX >= (1 << DEST_WIDTH))
      $error("Invalid BOARD_INDEX %0d: must fit in DEST_WIDTH=%0d bits.", BOARD_INDEX, DEST_WIDTH);
  end

  // Whole chunks currently resident in the FIFO (never over-reported).
  wire [FIFO_COUNT_WIDTH-1:0] avail_chunks = fifo_count_rd_clk >> CHUNK_SHIFT;

  reg  [WORD_IDX_WIDTH-1:0]  word_in_chunk;   // 0..CHUNK_WORDS-1, position within the current chunk
  reg  [CHUNK_IDX_WIDTH-1:0] chunks_in_packet;// chunks already completed in the current packet
  reg                        have_successor;  // latched at chunk start: another whole chunk was queued

  wire at_chunk_start = (word_in_chunk == 0);
  wire at_chunk_last  = (word_in_chunk == CHUNK_WORDS - 1);

  // Start a chunk only when a whole chunk is present; once started, its remaining
  // words are guaranteed resident (the count only grows), so mid-chunk never stalls.
  assign m_axis_tvalid = at_chunk_start ? (avail_chunks >= 1) : ~fifo_empty;
  assign m_axis_tdata  = fifo_rd_data;
  assign m_axis_tdest  = BOARD_INDEX[DEST_WIDTH-1:0];

  wire beat       = m_axis_tvalid & m_axis_tready;
  assign fifo_rd_en = beat;

  // Close the packet at a chunk boundary when no further whole chunk is queued, or at the cap.
  // For a single-word chunk, start and last coincide, so decide the successor combinationally.
  wire successor_present = at_chunk_start ? (avail_chunks >= 2) : have_successor;
  wire packet_cap        = (chunks_in_packet == MAX_CHUNKS - 1);
  assign m_axis_tlast    = at_chunk_last & (~successor_present | packet_cap);

  always @(posedge aclk) begin
    if (!aresetn) begin
      word_in_chunk    <= 0;
      chunks_in_packet <= 0;
      have_successor   <= 1'b0;
    end else if (beat) begin
      if (at_chunk_start)
        have_successor <= (avail_chunks >= 2); // snapshot at the first word of the chunk

      if (at_chunk_last) begin
        word_in_chunk    <= 0;
        chunks_in_packet <= m_axis_tlast ? 0 : (chunks_in_packet + 1'b1);
      end else begin
        word_in_chunk <= word_in_chunk + 1'b1;
      end
    end
  end

endmodule
