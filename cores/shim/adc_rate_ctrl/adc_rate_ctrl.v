`timescale 1 ns / 1 ps

// ADC capture rate control: a per-board backlog fill-count over the DDR capture buffer that
// drives each packetizer's packing density. backlog[b] is the occupancy of board b's
// packetized-but-unread capture in DDR -- the packetizer bumps it up by one for every word it
// emits (word_emitted[b]), and software bumps it down by the number of words it has just read,
// written as a delta to that board's rate lane in the board FIFO array (read_delta_en[b] with
// read_delta). accumulate[b] asserts while backlog[b] >= PACKET_WORDS, telling the packetizer to
// pack dense (fill toward a full packet) rather than drain eagerly. The threshold is the packet
// size itself (PACKET_WORDS, the same value as the packetizer's MAX_PACKET_WORDS) -- a
// compile-time parameter, not a register, so software never sets it, and a final sub-packet
// (fewer than PACKET_WORDS words) always falls below it and so drains rather than stalling.
//
// A board's backlog is zeroed by its own backlog_rst_n, wired to that board's adc_data FIFO
// reset, so one buffer reset clears the FIFO, the packetizer, and this count together. Each
// backlog is also output (board b at [32*b +: 32]) for software to read back through the same
// lane.
//
// Both sides report deltas (the packetizer a +1 beat, software a read count), so backlog is a
// plain fill count: a 32-bit width covers a backlog region up to 16 GiB, far past the 128 MB
// capture region, so the width is fixed rather than parameterized.
module adc_rate_ctrl #
(
  parameter integer NUM_BOARDS     = 8,
  parameter integer PACKET_WORDS   = 256    // packing threshold = packet size (= packetizer MAX_PACKET_WORDS)
)
(
  input  wire                       aclk,
  input  wire                       aresetn,

  // Per-board packing control (one bit per board, same aclk domain as the packetizers)
  input  wire [NUM_BOARDS-1:0]      word_emitted,   // packetizer emit beat: +1 to that backlog
  input  wire [NUM_BOARDS-1:0]      backlog_rst_n,  // per-board backlog clear (adc_data FIFO reset)
  output wire [NUM_BOARDS-1:0]      accumulate,     // 1 = pack dense, 0 = drain eagerly

  // Software read reports and backlog readback (from the board FIFO array's rate lane)
  input  wire [NUM_BOARDS-1:0]      read_delta_en,  // subtract read_delta from that board's backlog
  input  wire [31:0]                read_delta,
  output wire [32*NUM_BOARDS-1:0]   backlog_out
);

  localparam [31:0]  THRESHOLD_WORDS = PACKET_WORDS; // dense once a board has a full packet backlogged

  initial begin
    if (NUM_BOARDS < 1 || NUM_BOARDS > 8)
      $error("Invalid NUM_BOARDS %0d: must be between 1 and 8.", NUM_BOARDS);
    if (PACKET_WORDS < 1)
      $error("Invalid PACKET_WORDS %0d: must be >= 1.", PACKET_WORDS);
  end

  // --- Per-board backlog counters -------------------------------------------
  reg  [31:0] backlog [0:NUM_BOARDS-1];

  genvar b;
  generate
    for (b = 0; b < NUM_BOARDS; b = b + 1) begin : BOARDS
      wire [31:0] dec         = read_delta_en[b] ? read_delta : 32'd0;
      wire [32:0] up          = {1'b0, backlog[b]} + {32'd0, word_emitted[b]}; // backlog + emit beat

      // Fill count: up by the emit beat, down by the reported read delta, floored at 0 so a
      // stray over-report can never underflow. Cleared with the board's adc_data FIFO reset.
      always @(posedge aclk) begin
        if (!aresetn || !backlog_rst_n[b])
          backlog[b] <= 32'd0;
        else if ({1'b0, dec} > up)
          backlog[b] <= 32'd0;
        else
          backlog[b] <= up[31:0] - dec;
      end

      assign accumulate[b] = (backlog[b] >= THRESHOLD_WORDS);
      assign backlog_out[32*b +: 32] = backlog[b];
    end
  endgenerate

endmodule
