`timescale 1 ns / 1 ps

module clock_gate #(
  parameter GLOBAL = "FALSE" // Set to "TRUE" to use a global clock buffer (BUFGCE), "FALSE" for a local clock buffer (BUFR)
)(
  input  wire clk,        // Primary clock input
  input  wire en,         // Clock enable input
  output wire clk_gated   // Gated clock output
);

generate
  if (GLOBAL == "TRUE") begin : g_global_clk
    // BUFGCE: Global Clock Buffer with Clock Enable
    //         7 Series
    BUFGCE global_clock_buffer (
      .I(clk),       // 1-bit input: Primary clock
      .CE(en),       // 1-bit input: Clock enable input for I0
      .O(clk_gated)  // 1-bit output: Clock output
    );
  end else begin : g_local_clk
    // BUFR: Regional Clock Buffer for I/O and Logic Resources within a Clock Region
    //       7 Series
    BUFR #(
      .BUFR_DIVIDE("BYPASS"),   // Keep frequency unchanged while enabling CE/CLR behavior
      .SIM_DEVICE("7SERIES")
    ) regional_clock_buffer (
      .I(clk),        // 1-bit input: Clock input
      .CE(en),        // 1-bit input: Clock enable (active high)
      .CLR(1'b0),     // 1-bit input: Async clear (inactive)
      .O(clk_gated)   // 1-bit output: Clock output
    );
  end
endgenerate

endmodule
