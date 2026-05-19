module clock_mux (
  input  wire clk_0_i,   // 1-bit input: Clock input 0
  input  wire clk_1_i,   // 1-bit input: Clock input
  input  wire sel,       // 1-bit input: Clock select signal (0 selects clk_0_i, 1 selects clk_1_i)
  output wire clk_o     // 1-bit output: Selected clock output
);

  // BUFGMUX to select between divided and bypass clock
  BUFGMUX_CTRL clk_mux (
    .O(clk_o),       // 1-bit output: Clock output
    .I0(clk_0_i),    // 1-bit input: Clock input 0 (S=0)
    .I1(clk_1_i),    // 1-bit input: Clock input 1 (S=1)
    .S(sel)          // 1-bit input: Select signal
  );

endmodule
