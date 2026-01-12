`timescale 1 ns / 1 ps

module clock_gate (
  input  wire clk,        // Primary clock input
  input  wire en,         // Clock enable input
  output wire clk_gated   // Gated clock output
);

// BUFGCE: Global Clock Buffer with Clock Enable
//         7 Series

BUFGCE BUFGCE_inst (
   .I(clk),       // 1-bit input: Primary clock
   .CE(en),       // 1-bit input: Clock enable input for I0
   .O(clk_gated)  // 1-bit output: Clock output
);

endmodule
