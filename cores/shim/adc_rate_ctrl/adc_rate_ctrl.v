`timescale 1 ns / 1 ps

// ADC capture rate-control registers: a per-board backlog fill-count over the DDR capture
// buffer that drives each packetizer's packing density. backlog[b] is the occupancy of board
// b's packetized-but-unread capture in DDR -- the packetizer bumps it up by one for every word
// it emits (word_emitted[b]), and software bumps it down by the number of words it has just
// read, written as a delta to that board's READ_DELTA register. accumulate[b] asserts while
// backlog[b] >= PACKET_WORDS, telling the packetizer to pack dense (fill toward a full packet)
// rather than drain eagerly. The threshold is the packet size itself (PACKET_WORDS, the same
// value as the packetizer's MAX_PACKET_WORDS) -- a compile-time parameter, not a register, so
// software never sets it, and a final sub-packet (fewer than PACKET_WORDS words) always falls
// below it and so drains rather than stalling.
//
// A board's backlog is zeroed by its own backlog_rst_n, wired to that board's adc_data FIFO
// reset, so one buffer reset clears the FIFO, the packetizer, and this count together.
//
// Register map (32-bit words, shown for the default eight boards):
//   0 .. 7       READ_DELTA[b]  WO  write N to subtract N from board b's backlog (saturating at 0)
//   8 .. 15      BACKLOG[b]     RO  board b's current occupancy (words), for monitoring
//
// Both sides report deltas (the packetizer a +1 beat, software a read count), so backlog is a
// plain fill count: a 32-bit width covers a backlog region up to 16 GiB, far past the 128 MB
// capture region, so the width is fixed rather than parameterized.
module adc_rate_ctrl #
(
  parameter integer NUM_BOARDS     = 8,
  parameter integer PACKET_WORDS   = 256,   // packing threshold = packet size (= packetizer MAX_PACKET_WORDS)
  parameter integer AXI_ADDR_WIDTH = 16
)
(
  input  wire                       aclk,
  input  wire                       aresetn,

  // Per-board packing control (one bit per board, same aclk domain as the packetizers)
  input  wire [NUM_BOARDS-1:0]      word_emitted,   // packetizer emit beat: +1 to that backlog
  input  wire [NUM_BOARDS-1:0]      backlog_rst_n,  // per-board backlog clear (adc_data FIFO reset)
  output wire [NUM_BOARDS-1:0]      accumulate,     // 1 = pack dense, 0 = drain eagerly

  // AXI4-Lite subordinate port
  input  wire [AXI_ADDR_WIDTH-1:0]  s_axi_awaddr,  // Write address
  input  wire                       s_axi_awvalid, // Write address valid
  output wire                       s_axi_awready, // Write address ready
  input  wire [31:0]                s_axi_wdata,   // Write data
  input  wire [3:0]                 s_axi_wstrb,   // Write strobe (unused: whole-word registers)
  input  wire                       s_axi_wvalid,  // Write data valid
  output wire                       s_axi_wready,  // Write data ready
  output wire [1:0]                 s_axi_bresp,   // Write response
  output wire                       s_axi_bvalid,  // Write response valid
  input  wire                       s_axi_bready,  // Write response ready
  input  wire [AXI_ADDR_WIDTH-1:0]  s_axi_araddr,  // Read address
  input  wire                       s_axi_arvalid, // Read address valid
  output wire                       s_axi_arready, // Read address ready
  output wire [31:0]                s_axi_rdata,   // Read data
  output wire [1:0]                 s_axi_rresp,   // Read data response
  output wire                       s_axi_rvalid,  // Read data valid
  input  wire                       s_axi_rready   // Read data ready
);

  localparam integer ADDR_LSB        = 2;           // 32-bit word addressing
  localparam integer READ_DELTA_BASE = 0;
  localparam integer BACKLOG_BASE    = NUM_BOARDS;
  localparam [31:0]  THRESHOLD_WORDS = PACKET_WORDS; // dense once a board has a full packet backlogged

  initial begin
    if (NUM_BOARDS < 1 || NUM_BOARDS > 8)
      $error("Invalid NUM_BOARDS %0d: must be between 1 and 8.", NUM_BOARDS);
    if (PACKET_WORDS < 1)
      $error("Invalid PACKET_WORDS %0d: must be >= 1.", PACKET_WORDS);
  end

  // --- AXI4-Lite subordinate handshake (standard single-beat slave) ---------
  reg                       axi_awready;
  reg                       axi_wready;
  reg  [AXI_ADDR_WIDTH-1:0] axi_awaddr;
  reg                       axi_bvalid;
  reg                       axi_arready;
  reg  [AXI_ADDR_WIDTH-1:0] axi_araddr;
  reg                       axi_rvalid;
  reg  [31:0]               axi_rdata;
  reg                       aw_en;

  assign s_axi_awready = axi_awready;
  assign s_axi_wready  = axi_wready;
  assign s_axi_bresp   = 2'b00;   // always OKAY: read-delta writes never fault
  assign s_axi_bvalid  = axi_bvalid;
  assign s_axi_arready = axi_arready;
  assign s_axi_rdata   = axi_rdata;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rvalid  = axi_rvalid;

  // Latch the write address once both address and data are offered.
  always @(posedge aclk) begin
    if (!aresetn) begin
      axi_awready <= 1'b0;
      aw_en       <= 1'b1;
    end else if (!axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en) begin
      axi_awready <= 1'b1;
      aw_en       <= 1'b0;
    end else if (s_axi_bready && axi_bvalid) begin
      axi_awready <= 1'b0;
      aw_en       <= 1'b1;
    end else begin
      axi_awready <= 1'b0;
    end
  end

  always @(posedge aclk) begin
    if (!aresetn)
      axi_awaddr <= {AXI_ADDR_WIDTH{1'b0}};
    else if (!axi_awready && s_axi_awvalid && s_axi_wvalid && aw_en)
      axi_awaddr <= s_axi_awaddr;
  end

  always @(posedge aclk) begin
    if (!aresetn)
      axi_wready <= 1'b0;
    else if (!axi_wready && s_axi_wvalid && s_axi_awvalid && aw_en)
      axi_wready <= 1'b1;
    else
      axi_wready <= 1'b0;
  end

  wire slv_wren = axi_wready && s_axi_wvalid && axi_awready && s_axi_awvalid;
  wire [AXI_ADDR_WIDTH-ADDR_LSB-1:0] wr_word = axi_awaddr[AXI_ADDR_WIDTH-1:ADDR_LSB];

  always @(posedge aclk) begin
    if (!aresetn)
      axi_bvalid <= 1'b0;
    else if (axi_awready && s_axi_awvalid && !axi_bvalid && axi_wready && s_axi_wvalid)
      axi_bvalid <= 1'b1;
    else if (s_axi_bready && axi_bvalid)
      axi_bvalid <= 1'b0;
  end

  // Read address accept.
  always @(posedge aclk) begin
    if (!aresetn) begin
      axi_arready <= 1'b0;
      axi_araddr  <= {AXI_ADDR_WIDTH{1'b0}};
    end else if (!axi_arready && s_axi_arvalid) begin
      axi_arready <= 1'b1;
      axi_araddr  <= s_axi_araddr;
    end else begin
      axi_arready <= 1'b0;
    end
  end

  always @(posedge aclk) begin
    if (!aresetn)
      axi_rvalid <= 1'b0;
    else if (axi_arready && s_axi_arvalid && !axi_rvalid)
      axi_rvalid <= 1'b1;
    else if (axi_rvalid && s_axi_rready)
      axi_rvalid <= 1'b0;
  end

  wire                               slv_rden = axi_arready && s_axi_arvalid && !axi_rvalid;
  wire [AXI_ADDR_WIDTH-ADDR_LSB-1:0] rd_word  = axi_araddr[AXI_ADDR_WIDTH-1:ADDR_LSB];

  // --- Per-board backlog counters -------------------------------------------
  reg  [31:0] backlog [0:NUM_BOARDS-1];

  genvar b;
  generate
    for (b = 0; b < NUM_BOARDS; b = b + 1) begin : BOARDS
      wire        rd_delta_wr = slv_wren && (wr_word == (READ_DELTA_BASE + b));
      wire [31:0] dec         = rd_delta_wr ? s_axi_wdata : 32'd0;
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
    end
  endgenerate

  // Registered read-data mux: a board's backlog, or 0 for an unmapped word.
  integer bi;
  always @(posedge aclk) begin
    if (slv_rden) begin
      axi_rdata <= 32'd0;
      for (bi = 0; bi < NUM_BOARDS; bi = bi + 1)
        if (rd_word == (BACKLOG_BASE + bi))
          axi_rdata <= backlog[bi];
    end
  end

endmodule
