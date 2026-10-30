`timescale 1ns / 1ps

// Reset synchronizer: the output asserts asynchronously with the input and releases on a clock
// edge DEPTH cycles after the input deasserts. Asserting asynchronously means the reset lands even
// while the clock is stopped; releasing synchronously keeps the deassertion edge clean for the
// flops it drives. This is the lightweight stand-in for proc_sys_reset where the input is an
// internal, glitch-free reset (a register bit or another reset_sync output) -- no external-pin
// filtering, minimum-width stretching, or bus/peripheral sequencing is needed. Chain two
// (feeding one's rst_out_n into the next's rst_in_n) to order a release across clock domains.
module reset_sync #(
  parameter integer DEPTH = 2  // synchronizer stages (2 to 4)
)(
  input  wire clk,
  input  wire rst_in_n,   // active-low reset in, asynchronous
  output wire rst_out_n   // active-low reset out, released synchronously to clk
);

  // Validate parameters
  initial begin
    if (DEPTH < 2 || DEPTH > 4)
      $error("Invalid value for DEPTH parameter: %0d. Must be between 2 and 4.", DEPTH);
  end

  (* ASYNC_REG = "TRUE" *) reg [DEPTH-1:0] sync;

  always @(posedge clk or negedge rst_in_n) begin
    if (!rst_in_n)
      sync <= {DEPTH{1'b0}};
    else
      sync <= {sync[DEPTH-2:0], 1'b1};
  end

  assign rst_out_n = sync[DEPTH-1];

endmodule
