`timescale 1ns / 1ps

// Shutdown sense scanner: steps the external sense mux through the eight boards and latches a
// sticky per-board flag when that board's sense line is high. Each select is held for
// DWELL_CYCLES so the external mux, cable and pin synchronizer settle before the single sample
// taken on the window's last cycle.
module shutdown_sense #(
  parameter integer DWELL_CYCLES = 16  // clocks per select (min 4: 2 synchronizer stages plus settling)
)(
  input wire clk,
  input wire [7:0] shutdown_sense_connected,
  input wire shutdown_sense_en,
  input wire shutdown_sense_pin,
  output reg [2:0] shutdown_sense_sel,
  output reg [7:0] shutdown_sense_o
);

  initial begin
    if (DWELL_CYCLES < 4 || DWELL_CYCLES > 256)
      $error("Invalid value for DWELL_CYCLES parameter: %0d. Must be between 4 and 256.", DWELL_CYCLES);
  end

  localparam integer DWELL_LAST_INT = DWELL_CYCLES - 1;
  localparam [7:0] DWELL_LAST = DWELL_LAST_INT[7:0];

  (* ASYNC_REG = "TRUE" *) reg [1:0] pin_sync;
  reg [7:0] dwell;

  always @(posedge clk) begin
    pin_sync <= {pin_sync[0], shutdown_sense_pin};
  end

  always @(posedge clk) begin
    if (!shutdown_sense_en) begin
      // Disabled state
      shutdown_sense_sel <= 3'b000;
      shutdown_sense_o <= 8'b00000000;
      dwell <= 8'd0;
    end else if (dwell == DWELL_LAST) begin
      // End of this select's window: sample, then move to the next select
      dwell <= 8'd0;
      shutdown_sense_sel <= shutdown_sense_sel + 1;
      if (pin_sync[1] && shutdown_sense_connected[shutdown_sense_sel]) begin
        shutdown_sense_o[shutdown_sense_sel] <= 1'b1;
      end
    end else begin
      dwell <= dwell + 8'd1;
    end
  end

endmodule
