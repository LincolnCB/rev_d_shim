`timescale 1ns / 1ps

// A wrapper for the Xilinx OBUFDS primitive with a parameterized width
module diff_out_buf #
(
  parameter integer DIFF_BUF_WIDTH = 1
)
(
  input   wire  [DIFF_BUF_WIDTH-1:0]   d_in,
  output  wire  [DIFF_BUF_WIDTH-1:0]   diff_out_p,
  output  wire  [DIFF_BUF_WIDTH-1:0]   diff_out_n
);

// Generate DIFF_BUF_WIDTH number of the Xilinx OBUFDS primitives
genvar i;
generate
  for (i = 0; i < DIFF_BUF_WIDTH; i = i + 1)
  begin : gen_diff_out
    OBUFDS
    #(
      .IOSTANDARD("DEFAULT")
    )
    obufds_inst
    (
      .I(d_in[i]),
      .O(diff_out_p[i]),
      .OB(diff_out_n[i])
    );
  end
endgenerate

endmodule
