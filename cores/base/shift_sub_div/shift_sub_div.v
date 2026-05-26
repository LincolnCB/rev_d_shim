`timescale 1ns / 1ps

module shift_sub_div #(
  parameter integer DIVIDEND_WIDTH = 32,
  parameter integer DIVISOR_WIDTH  = 32
) (
  input  wire                            clk,
  input  wire                            resetn,
  input  wire                            start,
  input  wire [DIVIDEND_WIDTH-1:0]       dividend,
  input  wire [DIVISOR_WIDTH-1:0]        divisor,
  output reg  [DIVIDEND_WIDTH-1:0]       quotient,
  output reg  [DIVISOR_WIDTH-1:0]        remainder,
  output reg                             div_by_zero,
  output reg                             done
);

  // Function to calculate the ceiling of log2(value)
  function integer clogb2 (input integer value);
    for(clogb2 = 0; value > 0; clogb2 = clogb2 + 1) value = value >> 1;
  endfunction

  localparam integer COUNT_WIDTH = clogb2(DIVIDEND_WIDTH);

  reg [DIVISOR_WIDTH-1:0]  divisor_latched;
  reg [DIVIDEND_WIDTH-1:0] dividend_shift;
  reg [DIVIDEND_WIDTH-1:0] quotient_work;
  reg [DIVISOR_WIDTH:0]    remainder_work;
  reg [COUNT_WIDTH-1:0]    iter_count;
  reg                      busy;

  always @(posedge clk) begin
    if (!resetn) begin
      quotient <= {DIVIDEND_WIDTH{1'b0}};
      remainder <= {DIVISOR_WIDTH{1'b0}};
      div_by_zero <= 1'b0;
      done <= 1'b0;
      divisor_latched <= {DIVISOR_WIDTH{1'b0}};
      dividend_shift <= {DIVIDEND_WIDTH{1'b0}};
      quotient_work <= {DIVIDEND_WIDTH{1'b0}};
      remainder_work <= {(DIVISOR_WIDTH+1){1'b0}};
      iter_count <= {COUNT_WIDTH{1'b0}};
      busy <= 1'b0;
    end else begin
      if (!busy) begin
        done <= 1'b0;
        if (start) begin
          dividend_latched <= dividend;
          divisor_latched <= divisor;
          dividend_shift <= dividend;
          quotient_work <= {DIVIDEND_WIDTH{1'b0}};
          remainder_work <= {(DIVISOR_WIDTH+1){1'b0}};
          iter_count <= {COUNT_WIDTH{1'b0}};

          if (divisor == {DIVISOR_WIDTH{1'b0}}) begin
            // Guard divide-by-zero and complete immediately.
            quotient <= {DIVIDEND_WIDTH{1'b1}};
            remainder <= dividend[DIVISOR_WIDTH-1:0];
            div_by_zero <= 1'b1;
            done <= 1'b1;
            busy <= 1'b0;
          end else begin
            div_by_zero <= 1'b0;
            busy <= 1'b1;
          end
        end
      end else if (busy) begin
        if ((dividend_shift > 0) && (iter_count < DIVIDEND_WIDTH)) begin
          if ({remainder_work[DIVISOR_WIDTH-1:0], dividend_shift[DIVIDEND_WIDTH-1]} >= {1'b0, divisor_latched}) begin
            remainder_work <= {remainder_work[DIVISOR_WIDTH-1:0], dividend_shift[DIVIDEND_WIDTH-1]} - {1'b0, divisor_latched};
            quotient_work <= (quotient_work << 1) | {{(DIVIDEND_WIDTH-1){1'b0}}, 1'b1};
          end else begin
            remainder_work <= {remainder_work[DIVISOR_WIDTH-1:0], dividend_shift[DIVIDEND_WIDTH-1]};
            quotient_work <= (quotient_work << 1);
          end

          dividend_shift <= (dividend_shift << 1);
          iter_count <= iter_count + 1'b1;
        end else begin
          quotient <= quotient_work;
          remainder <= remainder_work[DIVISOR_WIDTH-1:0];
          done <= 1'b1;
          busy <= 1'b0;
        end
      end
    end
  end

endmodule
