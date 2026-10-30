`timescale 1ps/1ps

// Bit-coherent synchronizer: dout (out_clk domain) always holds one whole snapshot of din (in_clk
// domain), never a mix of two values, and follows din within a few cycles of each clock. Values
// that change faster than that are skipped, so it suits configuration and status words, not
// streams. It runs a continuous request/acknowledge loop: the input side loads din into a hold
// register and toggles req, the output side copies the hold register once it sees req change and
// echoes req back as ack, and the input side loads the next snapshot only after that echo. The
// hold register is therefore never changing while it's copied.
//
// Either side can reset on its own. An output reset puts dout at dout_default and the loop
// refills it from the current din without din having to change. An input reset pauses loading,
// so dout keeps the last value until the input side runs again; req and ack_sync aren't reset so
// the loop can't lose track of which toggle was acknowledged.
module sync_coherent #(
  parameter WIDTH = 1   // Width of the input and output signals
)(
  input  wire in_clk,            // Input clock
  input  wire in_resetn,         // Active low input reset (pauses snapshots)
  input  wire out_clk,           // Output clock
  input  wire out_resetn,        // Active low output reset (dout to dout_default)

  input  wire [WIDTH-1:0] din,          // Input signal to be synchronized
  output reg  [WIDTH-1:0] dout,         // Synchronized output signal
  input  wire [WIDTH-1:0] dout_default  // Output value while and after out_resetn is low
);

  // Validate parameters
  initial begin
    if (WIDTH <= 0)
      $error("Invalid value for WIDTH parameter: %d. Must be greater than 0.", WIDTH);
  end

  reg [WIDTH-1:0] hold;  // only copied after a req toggle, which always follows a load
  reg             req;
  reg             ack;
  (* ASYNC_REG = "TRUE" *) reg [1:0] ack_sync;
  (* ASYNC_REG = "TRUE" *) reg [1:0] req_sync;

  initial begin
    req = 1'b0;
    ack_sync = 2'b00;
  end

  // Input side: take the next snapshot once the previous toggle has been echoed back
  always @(posedge in_clk) begin
    ack_sync <= {ack_sync[0], ack};
    if (in_resetn && ack_sync[1] == req) begin
      hold <= din;
      req  <= ~req;
    end
  end

  // Output side: copy the hold register on each new req toggle and echo it as ack.
  // Copy enable and reset stay on the flops' CE and R/S pins, so the hold -> dout crossing has no
  // logic in front of D (otherwise synthesis folds them into a LUT for default-1 bits)
  (* DIRECT_ENABLE = "yes" *) wire copy_en = (req_sync[1] != ack);
  (* DIRECT_RESET = "yes" *)  wire out_rst = !out_resetn;
  always @(posedge out_clk) begin
    if (out_rst) begin
      req_sync <= 2'b00;
      ack      <= 1'b0;
      dout     <= dout_default;
    end else begin
      req_sync <= {req_sync[0], req};
      if (copy_en) begin
        dout <= hold;
        ack  <= req_sync[1];
      end
    end
  end

endmodule
