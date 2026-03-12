module clock_divider (
  input  wire clk_i,
  input  wire slow_en,
  output wire clk_o
);

  wire clk_div8;
  wire clk_bypass;

  // BUFR with divide by 8
  BUFR #(
    .BUFR_DIVIDE("8"),
    .SIM_DEVICE("7SERIES")
  ) bufr_div8 (
    .I(clk_i),
    .O(clk_div8),
    .CE(1'b1),
    .CLR(1'b0)
  );

  // BUFR with bypass (no division)
  BUFR #(
    .BUFR_DIVIDE("BYPASS"),
    .SIM_DEVICE("7SERIES")
  ) bufr_bypass (
    .I(clk_i),
    .O(clk_bypass),
    .CE(1'b1),
    .CLR(1'b0)
  );

  // Mux to select between divided and bypass clock
  assign clk_o = slow_en ? clk_div8 : clk_bypass;

endmodule
