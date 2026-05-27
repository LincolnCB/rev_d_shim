`timescale 1ns / 1ps

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

  // Function to calculate the ceiling of log2(value)
  function integer clogb2 (input integer value);
    for(clogb2 = 0; value > 0; clogb2 = clogb2 + 1) value = value >> 1;
  endfunction

  localparam integer COUNT_WIDTH = clogb2(MULTIPLICAND_WIDTH);

  reg  [COUNT_WIDTH-1:0] mult_count;
  reg  [MULTIPLICAND_WIDTH-1:0] multiplicand_latched;
  reg  [MULTIPLIER_WIDTH-1:0]   multiplier_latched;
  reg  [MULTIPLICAND_WIDTH-1:0] mult_shift;
  reg  [MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH-1:0] mult_accumulator;
  reg         busy;

  always @(posedge clk) begin
    if (!resetn) begin
      done <= 1'b0;
      busy <= 1'b0;
      result <= {(MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH){1'b0}};
      mult_count <= {COUNT_WIDTH{1'b0}};
      mult_shift <= {MULTIPLICAND_WIDTH{1'b0}};
      multiplicand_latched <= {MULTIPLICAND_WIDTH{1'b0}};
      multiplier_latched <= {MULTIPLIER_WIDTH{1'b0}};
      mult_accumulator <= {(MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH){1'b0}};
    end else begin
      if (!busy) begin
        done <= 1'b0;
        if (start) begin
          busy <= 1'b1;
          mult_count <= {COUNT_WIDTH{1'b0}};
          mult_shift <= {MULTIPLICAND_WIDTH{1'b0}};
          mult_accumulator <= {(MULTIPLICAND_WIDTH+MULTIPLIER_WIDTH){1'b0}};
          multiplicand_latched <= multiplicand;
          multiplier_latched <= multiplier;
        end
      end else begin
        if ((mult_shift < multiplicand_latched) && (mult_count < MULTIPLICAND_WIDTH)) begin
          if (multiplicand_latched[mult_count]) begin
            mult_accumulator <= mult_accumulator + ({{MULTIPLICAND_WIDTH{1'b0}}, multiplier_latched} << mult_count);
          end
          mult_shift <= {{(MULTIPLICAND_WIDTH-1){1'b0}}, 1'b1} << mult_count;
          mult_count <= mult_count + 1;
        end else begin
          busy <= 1'b0;
          result <= mult_accumulator;
          done <= 1'b1;
        end
      end
    end
  end

endmodule
