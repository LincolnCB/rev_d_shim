// Thanks to H. Fatih Uǧurdaǧ :)
module fifo_sync #(
  parameter FORCE_BRAM = 0, // Set to 1 to force BRAM usage
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 4,  // FIFO depth = 2^ADDR_WIDTH
  parameter ALMOST_FULL_THRESHOLD = 2, // Adjust as needed
  parameter ALMOST_EMPTY_THRESHOLD = 2 // Adjust as needed
)(
  input  wire                   clk,
  input  wire                   resetn,
  input  wire [DATA_WIDTH-1:0]  wr_data,
  input  wire                   wr_en,
  output wire                   full,
  output wire                   almost_full,

  output wire [ADDR_WIDTH:0]    fifo_count,

  output wire [DATA_WIDTH-1:0]  rd_data,
  input  wire                   rd_en,
  output wire                   empty,
  output wire                   almost_empty
);

  localparam [ADDR_WIDTH:0] FIFO_DEPTH = {1'b1, {ADDR_WIDTH{1'b0}}}; // 2^ADDR_WIDTH
  localparam [ADDR_WIDTH:0] ALMOST_FULL_THR_W = ALMOST_FULL_THRESHOLD[ADDR_WIDTH:0];
  localparam [ADDR_WIDTH:0] ALMOST_EMPTY_THR_W = ALMOST_EMPTY_THRESHOLD[ADDR_WIDTH:0];

  // Validate parameters
  initial begin
    if (FORCE_BRAM != 0 && FORCE_BRAM != 1)
      $error("Invalid value for FORCE_BRAM parameter: %d. Must be 0 or 1.", FORCE_BRAM);
    if (DATA_WIDTH <= 0) 
      $error("Invalid value for DATA_WIDTH parameter: %d. Must be greater than 0.", DATA_WIDTH);
    if (ADDR_WIDTH <= 0)
      $error("Invalid value for ADDR_WIDTH parameter: %d. Must be greater than 0.", ADDR_WIDTH);
    if (ALMOST_FULL_THRESHOLD < 0 || ALMOST_FULL_THRESHOLD > FIFO_DEPTH)
      $error("Invalid value for ALMOST_FULL_THRESHOLD parameter: %d. Must be between 0 and FIFO depth (2^ADDR_WIDTH, ADDR_WIDTH=%d, FIFO_DEPTH=%d).",
             ALMOST_FULL_THRESHOLD, ADDR_WIDTH, FIFO_DEPTH);
    if (ALMOST_EMPTY_THRESHOLD < 0 || ALMOST_EMPTY_THRESHOLD > FIFO_DEPTH)
      $error("Invalid value for ALMOST_EMPTY_THRESHOLD parameter: %d. Must be between 0 and FIFO depth (2^ADDR_WIDTH, ADDR_WIDTH=%d, FIFO_DEPTH=%d).",
             ALMOST_EMPTY_THRESHOLD, ADDR_WIDTH, FIFO_DEPTH);
  end


  // Write and read pointers
  reg [ADDR_WIDTH:0] wr_ptr_bin;
  reg [ADDR_WIDTH:0] rd_ptr_bin;
  reg [ADDR_WIDTH:0] rd_ptr_bin_nxt;

  // FIFO memory (BRAM instance)
  mem_sync #(
    .FORCE_BRAM(FORCE_BRAM),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) mem (
    .clk(clk),
    .wr_addr(wr_ptr_bin[ADDR_WIDTH-1:0]), 
    .wr_data(wr_data),
    .wr_en(wr_en), 
    .rd_addr(rd_ptr_bin_nxt[ADDR_WIDTH-1:0]),
    .rd_data(rd_data)
  );

  // Write logic
  always @(posedge clk) begin
    if (!resetn) begin
      wr_ptr_bin <= 0;
    end else if (wr_en) begin
      wr_ptr_bin <= wr_ptr_bin + 1;
    end
  end

  // Read logic
  always @* rd_ptr_bin_nxt = rd_ptr_bin + {{(ADDR_WIDTH){1'b0}}, (rd_en & ~empty)};
  // Update read pointer on clock edge
  always @(posedge clk) begin
    if (!resetn) begin
      rd_ptr_bin <= 0;
    end else begin
      rd_ptr_bin <= rd_ptr_bin_nxt;
    end
  end

  // Generate full and empty flags
  assign full  = ( (wr_ptr_bin[ADDR_WIDTH] != rd_ptr_bin[ADDR_WIDTH]) &&
           (wr_ptr_bin[ADDR_WIDTH-1:0] == rd_ptr_bin[ADDR_WIDTH-1:0]) );
  assign empty = (wr_ptr_bin == rd_ptr_bin);

  // FIFO count
  assign fifo_count = wr_ptr_bin - rd_ptr_bin;

  // Almost full/empty
  assign almost_full  = (fifo_count >= (FIFO_DEPTH - ALMOST_FULL_THR_W));
  assign almost_empty = (fifo_count <= ALMOST_EMPTY_THR_W);

endmodule

// BRAM module formatted and specced for BRAM utilization in synthesis
module mem_sync #(
  parameter FORCE_BRAM = 0,
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 4
)(
  input  wire                   clk,
  input  wire [ADDR_WIDTH-1:0]  wr_addr,
  input  wire [DATA_WIDTH-1:0]  wr_data,
  input  wire                   wr_en,
  input  wire [ADDR_WIDTH-1:0]  rd_addr,
  output reg  [DATA_WIDTH-1:0]  rd_data
);

  localparam [ADDR_WIDTH:0] BRAM_DEPTH = {1'b1, {ADDR_WIDTH{1'b0}}}; // 2^ADDR_WIDTH

  generate
    if (FORCE_BRAM) begin : gen_bram
      // Forced BRAM usage
      (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:BRAM_DEPTH-1];
      always @(posedge clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
      end
    end else begin : gen_reg
      // Default memory
      reg [DATA_WIDTH-1:0] mem [0:BRAM_DEPTH-1];
      always @(posedge clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
      end
    end
  endgenerate

endmodule
