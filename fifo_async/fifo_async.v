// Thanks to Thomas Witzel, H. Fatih Uǧurdaǧ, and Kutay Bulun :)
module fifo_async #(
  parameter FORCE_BRAM = 0, // Set to 1 to force BRAM usage
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 4,  // FIFO depth = 2^ADDR_WIDTH
  parameter ALMOST_FULL_THRESHOLD = 2, // Adjust as needed
  parameter ALMOST_EMPTY_THRESHOLD = 2 // Adjust as needed
)(
  input  wire                   wr_clk,
  input  wire                   wr_rst_n,
  input  wire [DATA_WIDTH-1:0]  wr_data,
  input  wire                   wr_en,
  output wire [ADDR_WIDTH:0]    fifo_count_wr_clk,
  output wire                   full,
  output wire                   almost_full,

  input  wire                   rd_clk,
  input  wire                   rd_rst_n,
  output wire [DATA_WIDTH-1:0]  rd_data,
  input  wire                   rd_en,
  output wire [ADDR_WIDTH:0]    fifo_count_rd_clk,
  output wire                   empty,
  output wire                   almost_empty
);

  // Cap ALMOST_FULL_THRESHOLD to be less than FIFO depth
  localparam [ADDR_WIDTH:0] ALMOST_FULL_THRESHOLD_CAPPED = (ALMOST_FULL_THRESHOLD < (1 << ADDR_WIDTH)) ? ALMOST_FULL_THRESHOLD[ADDR_WIDTH:0] : (1 << ADDR_WIDTH) - 1;
  localparam [ADDR_WIDTH:0] ALMOST_EMPTY_THRESHOLD_CAPPED = (ALMOST_EMPTY_THRESHOLD < (1 << ADDR_WIDTH)) ? ALMOST_EMPTY_THRESHOLD[ADDR_WIDTH:0] : (1 << ADDR_WIDTH) - 1;
  

  // Function to convert binary to Gray code
  function [ADDR_WIDTH:0] binary_to_gray(input [ADDR_WIDTH:0] bin);
    binary_to_gray = (bin >> 1) ^ bin;
  endfunction

  // Function to convert Gray code to binary
  function [ADDR_WIDTH:0] gray_to_binary(input [ADDR_WIDTH:0] gray);
    integer i;
    begin
      gray_to_binary[ADDR_WIDTH] = gray[ADDR_WIDTH];
      for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
        gray_to_binary[i] = gray[i] ^ gray_to_binary[i+1];
    end
  endfunction

  // Write and read pointers
  reg [ADDR_WIDTH:0] wr_ptr_bin;
  reg [ADDR_WIDTH:0] wr_ptr_gray; // the gray code version of the pointer is used for synchronization

  reg [ADDR_WIDTH:0] rd_ptr_bin;
  reg [ADDR_WIDTH:0] rd_ptr_bin_next;
  reg [ADDR_WIDTH:0] rd_ptr_gray; // the gray code version of the pointer is used for synchronization

  // FIFO memory (BRAM instance)
  mem_async #(
    .FORCE_BRAM(FORCE_BRAM),
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
  ) mem (
    .wr_clk(wr_clk),
    .wr_addr(wr_ptr_bin[ADDR_WIDTH-1:0]),
    .wr_data(wr_data),
    .wr_en(wr_en),

    .rd_clk(rd_clk),
    .rd_addr(rd_ptr_bin_next[ADDR_WIDTH-1:0]),
    .rd_data(rd_data)
  );

  // Write logic
  always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_ptr_bin  <= 0;
      wr_ptr_gray <= 0;
    end else if (wr_en && !full) begin
      wr_ptr_bin  <= wr_ptr_bin + 1;
      wr_ptr_gray <= binary_to_gray(wr_ptr_bin + 1);
    end
  end

  // Read logic
  always @* rd_ptr_bin_next = rd_ptr_bin + {{ADDR_WIDTH{1'b0}}, (rd_en & ~empty)};
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_bin  <= 0;
      rd_ptr_gray <= 0;
    end else begin
      rd_ptr_bin  <= rd_ptr_bin_next;
      rd_ptr_gray <= binary_to_gray(rd_ptr_bin_next);
    end
  end

  // Synchronize pointers across clock domains
  // Use double-flop synchronizers for wr_ptr in read clock domain
  reg [ADDR_WIDTH:0] wr_ptr_gray_rd_clk_sync1, wr_ptr_gray_rd_clk_sync2;
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wr_ptr_gray_rd_clk_sync1 <= 0;
      wr_ptr_gray_rd_clk_sync2 <= 0;
    end else begin
      wr_ptr_gray_rd_clk_sync1 <= wr_ptr_gray;
      wr_ptr_gray_rd_clk_sync2 <= wr_ptr_gray_rd_clk_sync1;
    end
  end
  wire [ADDR_WIDTH:0] wr_ptr_bin_rd_clk;
  assign wr_ptr_bin_rd_clk = gray_to_binary(wr_ptr_gray_rd_clk_sync2);

  // Use double-flop synchronizers for rd_ptr in write clock domain
  reg [ADDR_WIDTH:0] rd_ptr_gray_wr_clk_sync1, rd_ptr_gray_wr_clk_sync2;
  always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rd_ptr_gray_wr_clk_sync1 <= 0;
      rd_ptr_gray_wr_clk_sync2 <= 0;
    end else begin
      rd_ptr_gray_wr_clk_sync1 <= rd_ptr_gray;
      rd_ptr_gray_wr_clk_sync2 <= rd_ptr_gray_wr_clk_sync1;
    end
  end
  wire [ADDR_WIDTH:0] rd_ptr_bin_wr_clk;
  assign rd_ptr_bin_wr_clk = gray_to_binary(rd_ptr_gray_wr_clk_sync2);

  // Generate full and empty flags
  assign full = ( (wr_ptr_bin[ADDR_WIDTH] != rd_ptr_bin_wr_clk[ADDR_WIDTH]) &&
          (wr_ptr_bin[ADDR_WIDTH-1:0] == rd_ptr_bin_wr_clk[ADDR_WIDTH-1:0]) );

  assign empty = (rd_ptr_bin == wr_ptr_bin_rd_clk);

  // ALMOST FULL calculation is done in write clock domain
  assign fifo_count_wr_clk = wr_ptr_bin - rd_ptr_bin_wr_clk;

  // since the ptrs wrap circularily we need to be very careful with the subtractions. Best to have a test
  assign almost_full = (fifo_count_wr_clk >= ((1 << ADDR_WIDTH) - ALMOST_FULL_THRESHOLD_CAPPED));

  // ALMOST EMPTY calculation is done in read clock domain
  assign fifo_count_rd_clk = wr_ptr_bin_rd_clk - rd_ptr_bin;

  assign almost_empty = (fifo_count_rd_clk <= ALMOST_EMPTY_THRESHOLD_CAPPED);

endmodule

// BRAM module formatted and specced to guarantee BRAM utilization in synthesis
module mem_async #(
  parameter FORCE_BRAM = 0,
  parameter DATA_WIDTH = 16,
  parameter ADDR_WIDTH = 4
)(
  input  wire                   wr_clk,
  input  wire [ADDR_WIDTH-1:0]  wr_addr,
  input  wire [DATA_WIDTH-1:0]  wr_data,
  input  wire                   wr_en,

  input  wire                   rd_clk,
  input  wire [ADDR_WIDTH-1:0]  rd_addr,
  output reg  [DATA_WIDTH-1:0]  rd_data
);

  generate
    if (FORCE_BRAM) begin : gen_bram
      // Forced BRAM usage
      (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
      always @(posedge wr_clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
      end
      always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
      end
    end else begin : gen_reg
      // Default memory
      reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
      always @(posedge wr_clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
      end
      always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
      end
    end
  endgenerate

endmodule
