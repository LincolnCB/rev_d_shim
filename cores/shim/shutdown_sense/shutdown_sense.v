`timescale 1ns / 1ps

module shutdown_sense (
  input wire clk,
  input wire [7:0] shutdown_sense_connected,
  input wire shutdown_sense_en,
  input wire shutdown_sense_pin,
  output reg [2:0] shutdown_sense_sel,
  output reg [7:0] shutdown_sense_o
);

  always @(posedge clk) begin
    if (!shutdown_sense_en) begin
      // Disabled state
      shutdown_sense_sel <= 3'b000;
      shutdown_sense_o <= 8'b00000000;
    end else begin
      // Cycle through shutdown_sense_sel
      shutdown_sense_sel <= shutdown_sense_sel + 1;

      // Latch the corresponding bit of shutdown_sense if connected on that channel
      if (shutdown_sense_pin && shutdown_sense_connected[shutdown_sense_sel]) begin
        shutdown_sense_o[shutdown_sense_sel] <= 1'b1;
      end
    end
  end

endmodule
