`timescale 1 ns / 1 ps

// Frames an ADC read-side FIFO stream into AXIS packets for S2MM DMA.
//
// The ADC data lane carries variable-length reads: a full read op is four 32-bit FIFO
// words (eight 16-bit channels), while a single-channel read is one word. There is no
// fixed alignment, so packets are NOT tied to sample-set boundaries -- a packet may
// close on any word and a read may be split across packets. Software reassembles the
// samples from the captured buffer by position, so a split is harmless.
//
// Packet size is adaptive: the packetizer drains while words are resident and closes the
// packet the moment the board runs dry, which releases the downstream packet-atomic mux
// instead of stranding it on a momentarily-empty board (head-of-line blocking). A run
// tail self-flushes -- when the FIFO drains, the "no more words" condition closes the
// final packet on its own, so no separate end-of-sequence signal is needed. A slow
// trickle where the reader keeps pace with the writer simply yields single-word packets.
//
// tdest is tied to BOARD_INDEX so the mux and MCDMA route this board's samples back to
// its own S2MM buffer. MAX_PACKET_WORDS caps a packet so it can never exceed the mux's
// ARB_ON_MAX_XFERS backstop (which would re-arbitrate mid-packet and corrupt framing).
//
// The framing rests on the FIFO's read-side fill count (fifo_count_rd_clk), which is in
// this same clock domain. That count is Gray-synchronized so it can lag but never
// over-report: the worst case is closing a packet one word early, which is harmless.
module adc_packetizer #(
  parameter integer DATA_WIDTH       = 32,  // AXIS/FIFO word width
  parameter integer DEST_WIDTH       = 8,   // tdest width
  parameter integer FIFO_COUNT_WIDTH = 14,  // width of fifo_count_rd_clk (ADDR_WIDTH+1)
  parameter integer MAX_PACKET_WORDS = 256, // packet cap in words (<= mux ARB_ON_MAX_XFERS backstop)
  parameter integer BOARD_INDEX      = 0    // constant tdest for this board's stream
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

  // Packet-length counter is at least 1 bit wide even for a single-word cap.
  localparam integer WORD_IDX_WIDTH = (MAX_PACKET_WORDS <= 1) ? 1 : $clog2(MAX_PACKET_WORDS);

  // Parameter validation
  initial begin
    if (DATA_WIDTH <= 0 || DATA_WIDTH % 8 != 0)
      $error("Invalid DATA_WIDTH %0d: must be positive and a multiple of 8.", DATA_WIDTH);
    if (MAX_PACKET_WORDS <= 0)
      $error("Invalid MAX_PACKET_WORDS %0d: must be greater than 0.", MAX_PACKET_WORDS);
    if (FIFO_COUNT_WIDTH <= 0)
      $error("Invalid FIFO_COUNT_WIDTH %0d: must be greater than 0.", FIFO_COUNT_WIDTH);
    if (BOARD_INDEX < 0 || BOARD_INDEX >= (1 << DEST_WIDTH))
      $error("Invalid BOARD_INDEX %0d: must fit in DEST_WIDTH=%0d bits.", BOARD_INDEX, DEST_WIDTH);
  end

  reg [WORD_IDX_WIDTH-1:0] words_in_packet; // words already committed to the current packet

  // First-word-fall-through read: the head word is presented; a beat pops it.
  assign m_axis_tdata  = fifo_rd_data;
  assign m_axis_tdest  = BOARD_INDEX[DEST_WIDTH-1:0];
  assign m_axis_tvalid = ~fifo_empty;

  wire beat = m_axis_tvalid & m_axis_tready;
  assign fifo_rd_en = beat;

  // Close the packet when this is the last resident word (the board ran dry) or the cap
  // is reached. The count can lag but never over-reports, so "<= 1" at worst closes a
  // packet one word early -- a new packet then carries whatever arrives next.
  wire last_resident = (fifo_count_rd_clk <= 1);
  wire packet_cap    = (words_in_packet == MAX_PACKET_WORDS - 1);
  assign m_axis_tlast = last_resident | packet_cap;

  always @(posedge aclk) begin
    if (!aresetn)
      words_in_packet <= 0;
    else if (beat)
      words_in_packet <= m_axis_tlast ? 0 : (words_in_packet + 1'b1);
  end

endmodule
