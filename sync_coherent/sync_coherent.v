`timescale 1ps/1ps

module sync_coherent #(
  parameter BUF_ADDR_WIDTH = 2, // Buffer address width
  parameter WIDTH = 1   // Width of the input and output signals
)(
  input  wire in_clk,            // Clock signal
  input  wire in_resetn,         // Active low input reset signal
  input  wire out_clk,           // Output clock signal
  input  wire out_resetn,        // Active low output reset signal

  input  wire [WIDTH-1:0] din,   // Input signal to be synchronized
  output reg  [WIDTH-1:0] dout,         // Synchronized output signal
  input  wire [WIDTH-1:0] dout_default  // Default value for output signal
);

  reg  [WIDTH-1:0] prev_din; // Track previous din value
  wire [WIDTH-1:0] fifo_rd_data;
  wire fifo_empty;
  wire fifo_full;
  wire wr_en = ((din != prev_din) && !fifo_full) || fifo_empty; // Write enable condition (only when din changes and FIFO is not full)


  // FIFO to hold the synchronized values
  fifo_async #(
    .DATA_WIDTH(WIDTH),
    .ADDR_WIDTH(BUF_ADDR_WIDTH)
  ) fifo (
    .wr_clk(in_clk),
    .wr_rst_n(in_resetn),
    .wr_data(din),
    .wr_en(wr_en), // Write only if din has changed and FIFO is not full
    .full(fifo_full),
    .rd_clk(out_clk),
    .rd_rst_n(out_resetn),
    .rd_data(fifo_rd_data),
    .rd_en(!fifo_empty), // Read when FIFO is not empty
    .empty(fifo_empty)
  );

  // Output logic
  always @(posedge out_clk) begin
    if (!out_resetn) begin
      dout <= dout_default; // Reset output to default value
    end else if (!fifo_empty) begin
      dout <= fifo_rd_data; // Read from FIFO when not empty
    end
  end

  // Track previous din value
  always @(posedge in_clk) begin
    if (!fifo_full) begin
      prev_din <= din; // Update previous din value only if FIFO is not full
    end
  end

endmodule
