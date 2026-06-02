`timescale 1 ns / 1 ps


module clock_gate #(
  parameter integer WIDTH = 1, // Number of clock gates
  parameter GLOBAL = "FALSE"   // Set to "TRUE" to use a global clock buffer (BUFGCE), "FALSE" for a local clock buffer (BUFR)
)(
  input  wire [WIDTH-1:0] clk,   // Primary clock inputs
  input  wire [WIDTH-1:0] en,    // Clock enable inputs
  output wire [WIDTH-1:0] clk_o  // Gated clock outputs
);


// Generate WIDTH number of clock buffers
genvar i;
generate
  for (i = 0; i < WIDTH; i = i + 1) begin : gen_clock_gate
    if (GLOBAL == "TRUE") begin : g_global_clk
      BUFGCE global_clock_buffer (
        .I(clk[i]),
        .CE(en[i]),
        .O(clk_o[i])
      );
    end else begin : g_local_clk
      BUFR #(
        .BUFR_DIVIDE("1"),
        .SIM_DEVICE("7SERIES")
      ) regional_clock_buffer (
        .I(clk[i]),
        .CE(en[i]),
        .CLR(1'b0),
        .O(clk_o[i])
      );
    end
  end
endgenerate

endmodule
