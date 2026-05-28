`timescale 1 ns / 1 ps

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
  input  wire [1:0]             rd_en_type, // 2'b00 = no read, 2'b01 = regular read, 2'b10 = loop def read, 2'b11 = looping read
  output wire [ADDR_WIDTH:0]    fifo_count_rd_clk, // Total readable entries in the FIFO (distance between read start and write pointers)
  output wire [ADDR_WIDTH:0]    fifo_loop_def_available_count_rd_clk, // Available entries left for loop count
  output wire [ADDR_WIDTH:0]    fifo_loop_size_rd_clk, // Size of the current loop (distance between start and end pointers)
  output wire                   empty,
  output wire                   loop_def_empty,
  output wire                   almost_empty,
  output wire                   loop_def_almost_empty
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

  // Function to convert binary to gray code
  function [ADDR_WIDTH:0] binary_to_gray(input [ADDR_WIDTH:0] bin);
    binary_to_gray = (bin >> 1) ^ bin;
  endfunction

  // Function to convert gray code to binary
  function [ADDR_WIDTH:0] gray_to_binary(input [ADDR_WIDTH:0] gray);
    integer i;
    begin
      gray_to_binary[ADDR_WIDTH] = gray[ADDR_WIDTH];
      for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
        gray_to_binary[i] = gray[i] ^ gray_to_binary[i+1];
    end
  endfunction

  // Looping pointers
  localparam [1:0] RD_EN_TYPE_NO_READ = 2'b00;
  localparam [1:0] RD_EN_TYPE_REGULAR = 2'b01;
  localparam [1:0] RD_EN_TYPE_LOOP_DEF = 2'b10;
  localparam [1:0] RD_EN_TYPE_LOOPING = 2'b11;

  // Write pointer
  reg [ADDR_WIDTH:0] wr_ptr_bin;
  // Write pointer is converted to gray code and synchronized to read clock domain for empty calculation
  reg [ADDR_WIDTH:0] wr_ptr_gray;
  wire [ADDR_WIDTH:0] wr_ptr_bin_rd_clk;

  // Read pointer
  reg  [ADDR_WIDTH:0] rd_ptr_bin;
  reg  [ADDR_WIDTH:0] rd_ptr_bin_next; // Next value of read pointer (combinational used for BRAM read address)
  reg  [ADDR_WIDTH:0] rd_ptr_start_bin; // Marks the start of the reading range (used for looping)
  reg  [ADDR_WIDTH:0] rd_ptr_end_bin; // Marks the end of the reading range (used for looping)
  // Read pointer (start) is converted to gray code and synchronized to write clock domain for full calculation
  reg  [ADDR_WIDTH:0] rd_ptr_start_gray; // Write domain only needs to know where it shouldn't overwrite
  wire [ADDR_WIDTH:0] rd_ptr_start_bin_wr_clk;

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

  // Write pointer logic
  always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      wr_ptr_bin  <= 0;
      wr_ptr_gray <= 0;
    end else if (wr_en && !full) begin
      wr_ptr_bin  <= wr_ptr_bin + 1;
      wr_ptr_gray <= binary_to_gray(wr_ptr_bin + 1);
    end
  end

  // Read pointer logic
  always @(*) begin
    case (rd_en_type)
      // No read, hold pointer
      RD_EN_TYPE_NO_READ: begin
        rd_ptr_bin_next = rd_ptr_bin;
      end
      // Regular read, increment if not empty. Loop if aligned to end and NOT start (in the middle of a loop)
      RD_EN_TYPE_REGULAR: begin
        if (rd_ptr_bin == rd_ptr_end_bin && rd_ptr_bin != rd_ptr_start_bin) begin
          rd_ptr_bin_next = rd_ptr_start_bin;
        end else if (empty) begin
          rd_ptr_bin_next = rd_ptr_bin; // Hold if empty
        end else begin
          rd_ptr_bin_next = rd_ptr_bin + 1; // Regular increment
        end
      end
      // Loop def read. Pushes the end pointer forward while leaving the start pointer.
      RD_EN_TYPE_LOOP_DEF: begin
        if (loop_def_empty) begin
          rd_ptr_bin_next = rd_ptr_bin; // Hold if "loop_def" empty (no room for the loop to grow)
        end else begin
          rd_ptr_bin_next = rd_ptr_bin + 1; // Regular increment
        end
      end
      // Looping read, loop back to start if we hit the end.
      RD_EN_TYPE_LOOPING: begin
        if (rd_ptr_bin == rd_ptr_end_bin) begin
          rd_ptr_bin_next = rd_ptr_start_bin; // Loop back to start
        end else if (empty) begin
          rd_ptr_bin_next = rd_ptr_bin; // Hold if empty
        end else begin
          rd_ptr_bin_next = rd_ptr_bin + 1; // Regular increment
        end
      end
    endcase
  end
  // Updating read pointer
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_bin  <= 0;
    end else begin
      rd_ptr_bin  <= rd_ptr_bin_next;
    end
  end

  // Read pointer start logic for looping
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_start_bin <= 0;
    // Only move start pointer on regular reads when already aligned with the read pointer
    end else if (rd_en_type == RD_EN_TYPE_REGULAR && rd_ptr_bin == rd_ptr_start_bin) begin
      rd_ptr_start_bin <= rd_ptr_bin_next;
    end
  end

  // Read pointer end logic for looping
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      rd_ptr_end_bin <= 0;
    // Move the end pointer forward on loop def reads
    end else if (rd_en_type == RD_EN_TYPE_LOOP_DEF) begin
      rd_ptr_end_bin <= rd_ptr_bin_next;
    // For regular reads, only move the end pointer forward if the read pointer is aligned to both the start and end
    end else if (rd_en_type == RD_EN_TYPE_REGULAR && rd_ptr_bin == rd_ptr_start_bin && rd_ptr_bin == rd_ptr_end_bin) begin
      rd_ptr_end_bin <= rd_ptr_bin_next;
    end
  end


  // Synchronize pointers across clock domains
  // Use double-flop synchronizers for wr_ptr in read clock domain
  (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] wr_ptr_gray_rd_clk_sync1, wr_ptr_gray_rd_clk_sync2;
  always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
      wr_ptr_gray_rd_clk_sync1 <= 0;
      wr_ptr_gray_rd_clk_sync2 <= 0;
    end else begin
      wr_ptr_gray_rd_clk_sync1 <= wr_ptr_gray;
      wr_ptr_gray_rd_clk_sync2 <= wr_ptr_gray_rd_clk_sync1;
    end
  end
  assign wr_ptr_bin_rd_clk = gray_to_binary(wr_ptr_gray_rd_clk_sync2);

  // Use double-flop synchronizers for rd_ptr_start in write clock domain
  (* ASYNC_REG = "TRUE" *) reg [ADDR_WIDTH:0] rd_ptr_start_gray_wr_clk_sync1, rd_ptr_start_gray_wr_clk_sync2;
  always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
      rd_ptr_start_gray_wr_clk_sync1 <= 0;
      rd_ptr_start_gray_wr_clk_sync2 <= 0;
    end else begin
      rd_ptr_start_gray_wr_clk_sync1 <= rd_ptr_start_gray;
      rd_ptr_start_gray_wr_clk_sync2 <= rd_ptr_start_gray_wr_clk_sync1;
    end
  end
  assign rd_ptr_start_bin_wr_clk = gray_to_binary(rd_ptr_start_gray_wr_clk_sync2);

  // Generate full flag by making sure the write pointer doesn't lap the read start pointer
  assign full = ( (wr_ptr_bin[ADDR_WIDTH] != rd_ptr_start_bin_wr_clk[ADDR_WIDTH]) &&
          (wr_ptr_bin[ADDR_WIDTH-1:0] == rd_ptr_start_bin_wr_clk[ADDR_WIDTH-1:0]) );

  // Two empty flags:
  // "True" empty is when the read pointer is aligned with the start and the write pointer
  assign empty = (rd_ptr_bin == wr_ptr_bin_rd_clk && rd_ptr_bin == rd_ptr_start_bin);
  // "Loop_def" empty is when the read pointer is aligned with the end and the write pointer, meaning the loop cannot grow further
  assign loop_def_empty = (rd_ptr_bin == wr_ptr_bin_rd_clk && rd_ptr_bin == rd_ptr_end_bin);


  // ALMOST FULL calculation is done in write clock domain
  assign fifo_count_wr_clk = wr_ptr_bin - rd_ptr_start_bin_wr_clk;

  // since the ptrs wrap circularily we need to be very careful with the subtractions. Best to have a test
  assign almost_full = (fifo_count_wr_clk >= (FIFO_DEPTH - ALMOST_FULL_THR_W));

  // ALMOST EMPTY calculation is done in read clock domain
  assign fifo_count_rd_clk = wr_ptr_bin_rd_clk - rd_ptr_start_bin;
  assign fifo_loop_def_available_count_rd_clk = wr_ptr_bin_rd_clk - rd_ptr_end_bin;
  assign fifo_loop_size_rd_clk = rd_ptr_end_bin - rd_ptr_start_bin;

  assign almost_empty = (fifo_count_rd_clk <= ALMOST_EMPTY_THR_W);
  // "Loop_def" almost empty is when the available count for the loop is less than or equal to the threshold
  assign loop_def_almost_empty = (fifo_loop_def_available_count_rd_clk <= ALMOST_EMPTY_THR_W);

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

  localparam [ADDR_WIDTH:0] BRAM_DEPTH = {1'b1, {ADDR_WIDTH{1'b0}}}; // 2^ADDR_WIDTH

  generate
    if (FORCE_BRAM) begin : gen_bram
      // Forced BRAM usage
      (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:BRAM_DEPTH-1];
      always @(posedge wr_clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
      end
      always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
      end
    end else begin : gen_reg
      // Default memory
      reg [DATA_WIDTH-1:0] mem [0:BRAM_DEPTH-1];
      always @(posedge wr_clk) begin
        if(wr_en) mem[wr_addr] <= wr_data;
      end
      always @(posedge rd_clk) begin
        rd_data <= mem[rd_addr];
      end
    end
  endgenerate

endmodule
