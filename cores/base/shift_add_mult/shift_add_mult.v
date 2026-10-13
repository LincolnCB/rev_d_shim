`timescale 1ns / 1ps

// Unsigned multiplier with a start/done handshake. The full product is formed with a single
// multiply expression so Vivado maps it onto DSP48 blocks rather than LUT-based iterative
// shift-add logic. One cycle after start is accepted the product is latched into result, and
// done pulses the following cycle; result holds until the next start.
module shift_add_mult #(
  parameter integer MULTIPLICAND_WIDTH = 32,
  parameter integer MULTIPLIER_WIDTH   = 32
) (
  input  wire                                           clk,
  input  wire                                           resetn,
  input  wire                                           start,
  input  wire [MULTIPLICAND_WIDTH-1                 :0] multiplicand,
  input  wire [MULTIPLIER_WIDTH-1                   :0] multiplier,
  output reg  [MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH-1:0] result,
  output reg                                            done
);

  reg busy;

  always @(posedge clk) begin
    if (!resetn) begin
      result <= {(MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH){1'b0}};
      done   <= 1'b0;
      busy   <= 1'b0;
    end else begin
      done <= 1'b0;
      if (!busy) begin
        if (start) begin
          result <= multiplicand * multiplier; // mapped to DSP48
          busy   <= 1'b1;
        end
      end else begin
        busy <= 1'b0;
        done <= 1'b1;
      end
    end
  end

endmodule
