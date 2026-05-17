`timescale 1ps/1ps

module sync_incoherent #(
  parameter DEPTH = 2,  // Depth of the synchronizer (up to 4)
  parameter WIDTH = 1   // Width of the input and output signals
)(
  input  wire clk,               // Clock signal
  input  wire resetn,            // Active low reset signal
  input  wire [WIDTH-1:0] din,   // Input signal to be synchronized
  output wire [WIDTH-1:0] dout   // Synchronized output signal
);

  // Validate parameters
  initial begin
    if (DEPTH < 2 || DEPTH > 4)
      $error("Invalid value for DEPTH parameter: %d. Must be between 2 and 4.", DEPTH);
    if (WIDTH <= 0)
      $error("Invalid value for WIDTH parameter: %d. Must be greater than 0.", WIDTH);
  end

  function integer clogb2 (input integer value);
    for(clogb2 = 0; value > 0; clogb2 = clogb2 + 1) value = value >> 1;
  endfunction

  // Generate per-stage synchronizer registers with explicit stage names.
  generate
    if (DEPTH > 0) begin : sync_chain_0
      (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] async_reg;
      always @(posedge clk) begin
        if (!resetn)
          async_reg <= {WIDTH{1'b0}};
        else
          async_reg <= din;
      end
    end

    if (DEPTH > 1) begin : sync_chain_1
      (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] async_reg;
      always @(posedge clk) begin
        if (!resetn)
          async_reg <= {WIDTH{1'b0}};
        else
          async_reg <= sync_chain_0.async_reg;
      end
    end

    if (DEPTH > 2) begin : sync_chain_2
      (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] async_reg;
      always @(posedge clk) begin
        if (!resetn)
          async_reg <= {WIDTH{1'b0}};
        else
          async_reg <= sync_chain_1.async_reg;
      end
    end

    if (DEPTH > 3) begin : sync_chain_3
      (* ASYNC_REG = "TRUE" *) reg [WIDTH-1:0] async_reg;
      always @(posedge clk) begin
        if (!resetn)
          async_reg <= {WIDTH{1'b0}};
        else
          async_reg <= sync_chain_2.async_reg;
      end
    end

  endgenerate

  // Output is the last active synchronizer stage.
  generate
    if (DEPTH == 2) begin : dout_sel
      assign dout = sync_chain_1.async_reg;
    end else if (DEPTH == 3) begin : dout_sel
      assign dout = sync_chain_2.async_reg;
    end else begin : dout_sel
      assign dout = sync_chain_3.async_reg;
    end
  endgenerate

endmodule
