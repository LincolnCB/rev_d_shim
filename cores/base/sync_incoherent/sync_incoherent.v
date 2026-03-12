`timescale 1ps/1ps

module sync_incoherent #(
  parameter DEPTH = 2,  // Depth of the synchronizer
  parameter WIDTH = 1   // Width of the input and output signals
)(
  input  wire clk,               // Clock signal
  input  wire resetn,            // Active low reset signal
  input  wire [WIDTH-1:0] din,   // Input signal to be synchronized
  output wire [WIDTH-1:0] dout   // Synchronized output signal
);

  // Validate parameters
  initial begin
    if (DEPTH < 2 || DEPTH > 8)
      $error("Invalid value for DEPTH parameter: %d. Must be between 2 and 8.", DEPTH);
    if (WIDTH <= 0)
      $error("Invalid value for WIDTH parameter: %d. Must be greater than 0.", WIDTH);
  end

  function integer clogb2 (input integer value);
    for(clogb2 = 0; value > 0; clogb2 = clogb2 + 1) value = value >> 1;
  endfunction

  // Internal flip-flop chain
  // Use a fixed maximum depth for synthesis compatibility
  reg [WIDTH-1:0] sync_chain [0:7]; // Maximum depth of 8

  // Initial stage logic
  always @(posedge clk) begin
    if (!resetn) begin
      sync_chain[0] <= {WIDTH{1'b0}};
    end else begin
      sync_chain[0] <= din;
    end
  end
  genvar i;
  generate
    for (i = 1; i < 8; i = i + 1) begin : sync_chain_gen
      always @(posedge clk) begin
        if (!resetn) begin
          sync_chain[i] <= {WIDTH{1'b0}};
        end else if (i < DEPTH) begin
          sync_chain[i] <= sync_chain[i-1];
        end
      end
    end
  endgenerate

  // Output is the last flip-flop in the chain
  assign dout = sync_chain[DEPTH-1];

endmodule
