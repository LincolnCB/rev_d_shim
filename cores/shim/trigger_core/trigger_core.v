`timescale 1 ns / 1 ps

module trigger_core #(
  parameter unsigned TRIGGER_LOCKOUT_DEFAULT = 10000000 // Default lockout period in clock cycles (e.g., 10000000 at 20 MHz SPI clock -> 0.5 seconds)
) (
  input  wire        clk,
  input  wire        resetn,

  // Command FIFO interface
  output wire        cmd_word_rd_en,
  input  wire [31:0] cmd_word,
  input  wire        cmd_buf_empty,

  // Data FIFO interface
  output reg         data_word_wr_en,
  output reg  [31:0] data_word,
  input  wire        data_buf_full,
  input  wire        data_buf_almost_full, // Should indicate that the buffer has less than 2 entries free.

  // External signals
  input  wire        ext_trig,
  input  wire [7:0]  dac_waiting_for_trig,
  input  wire [7:0]  adc_waiting_for_trig,

  // Outputs
  output reg         trig_out,
  output reg [31:0]  trig_counter,
  output reg         data_buf_overflow,
  output reg         bad_cmd
);

  // Trigger lockout minimum
  localparam [27:0] TRIGGER_LOCKOUT_MIN = 4;
  // Validate parameters
  initial begin
    if (TRIGGER_LOCKOUT_DEFAULT < TRIGGER_LOCKOUT_MIN || TRIGGER_LOCKOUT_DEFAULT > 28'hFFFFFFF)
      $error("Invalid value for TRIGGER_LOCKOUT_DEFAULT parameter: %d. Must be between %d and %d.", TRIGGER_LOCKOUT_DEFAULT, TRIGGER_LOCKOUT_MIN, 28'hFFFFFFF);
  end

  // Command encoding
  localparam CMD_SYNC_CH         = 3'd1;
  localparam CMD_SET_LOCKOUT     = 3'd2;
  localparam CMD_EXPECT_EXT_TRIG = 3'd3;
  localparam CMD_DELAY           = 3'd4;
  localparam CMD_FORCE_TRIG      = 3'd5;
  localparam CMD_RESET_COUNT     = 3'd6;
  localparam CMD_CANCEL          = 3'd7;

  // State encoding
  localparam S_RESET       = 3'd0;
  localparam S_IDLE        = 3'd1;
  localparam S_SYNC_CH     = 3'd2;
  localparam S_EXPECT_TRIG = 3'd3;
  localparam S_DELAY       = 3'd4;
  localparam S_ERROR       = 3'd5;

  // State machine info
  reg  [ 2:0] state;
  wire        cmd_done;
  wire        do_next_cmd;
  wire [ 2:0] next_cmd_state;

  // Cancel, count reset, and sync
  wire cancel;
  wire reset_count;
  wire all_waiting;

  // Command decode
  wire [ 2:0] cmd_type = cmd_word[31:29];
  wire        cmd_log_trig = cmd_word[28];
  wire [27:0] cmd_val = cmd_word[27:0];

  // Command execution
  reg [27:0] delay_counter;
  reg [27:0] lockout_counter;
  reg [27:0] ext_trig_counter;
  reg [27:0] trig_lockout;
  reg ext_trig_sync[1:0]; // 2-stage synchronizer for external trigger
  reg  log_trig;
  wire do_log;
  wire do_trig;

  // Trigger timing tracking
  reg [63:0] trig_timer; // 64-bit timer to track trigger timing
  reg [31:0] second_word; // Second word to write to data buffer
  reg        trig_data_second_word; // Flag to indicate if the second word is being written

  // External trigger synchronization
  always @(posedge clk) begin
    if (!resetn) begin
      ext_trig_sync[0] <= 0;
      ext_trig_sync[1] <= 0;
    end else begin
      ext_trig_sync[0] <= ext_trig;
      ext_trig_sync[1] <= ext_trig_sync[0];
    end
  end

  // Checks for cancel / reset count command and synchronization conditions
  assign cancel = !cmd_buf_empty && cmd_type == CMD_CANCEL;
  assign reset_count = !cmd_buf_empty && cmd_type == CMD_RESET_COUNT;
  assign all_waiting = &dac_waiting_for_trig && &adc_waiting_for_trig;

  // Command done logic
  assign cmd_done = (state == S_IDLE && !cmd_buf_empty)
                  || (state == S_SYNC_CH && all_waiting)
                  || (state == S_EXPECT_TRIG && ext_trig_counter == 0)
                  || (state == S_DELAY && delay_counter == 0)
                  || (state != S_ERROR && cancel); // Allow cancel at any time
  assign do_next_cmd = cmd_done && !cmd_buf_empty;
  // Next state from upcoming command
  assign next_cmd_state = cmd_buf_empty ? S_IDLE
                          : (cmd_type == CMD_SYNC_CH) ? (all_waiting ? S_IDLE : S_SYNC_CH) // If all channels are already waiting, go right to idle
                          : (cmd_type == CMD_SET_LOCKOUT) ? (cmd_val >= TRIGGER_LOCKOUT_MIN ? S_IDLE : S_ERROR) // Lockout must be non-zero
                          : (cmd_type == CMD_EXPECT_EXT_TRIG) ? ((cmd_val != 0) ? S_EXPECT_TRIG : S_IDLE) // Zero triggers goes right to idle
                          : (cmd_type == CMD_DELAY) ? ((cmd_val != 0) ? S_DELAY : S_IDLE) // Zero delay goes right to idle
                          : (cmd_type == CMD_FORCE_TRIG) ? S_IDLE
                          : (cmd_type == CMD_RESET_COUNT) ? S_IDLE
                          : (cmd_type == CMD_CANCEL) ? S_IDLE
                          : S_ERROR;
  // State transition logic
  always @(posedge clk) begin
    if (!resetn) state <= S_RESET; // Reset to S_RESET state
    else if (state == S_RESET) state <= S_IDLE; // After reset, go to idle
    else if (cmd_done) state <= next_cmd_state; // Transition based on command done
  end

  //// Command execution logic
  // Set lockout
  always @(posedge clk) begin
    if (!resetn) trig_lockout <= TRIGGER_LOCKOUT_DEFAULT;
    else if (do_next_cmd && cmd_type == CMD_SET_LOCKOUT && cmd_val >= TRIGGER_LOCKOUT_MIN) trig_lockout <= cmd_val;
  end
  // Expected trigger count
  always @(posedge clk) begin
    if (!resetn || cancel || state == S_ERROR) ext_trig_counter <= 0;
    else if (do_next_cmd && cmd_type == CMD_EXPECT_EXT_TRIG) ext_trig_counter <= cmd_val;
    else if (state == S_EXPECT_TRIG && do_trig) ext_trig_counter <= ext_trig_counter - 1;
  end
  // Delay counter, used in delay state
  always @(posedge clk) begin
    if (!resetn || cancel || state == S_ERROR) delay_counter <= 0;
    else if (do_next_cmd && cmd_type == CMD_DELAY) delay_counter <= cmd_val;
    else if (delay_counter > 0) delay_counter <= delay_counter - 1;
  end
  // Lockout counter, used to prevent immediate re-triggering
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) lockout_counter <= 0;
    else if (state == S_EXPECT_TRIG && do_trig) lockout_counter <= trig_lockout; // Set to trig_lockout value after trigger
    else if (lockout_counter > 0) lockout_counter <= lockout_counter - 1; // Decrement lockout counter
  end
  // Trigger pulse generation
  assign do_trig = (do_next_cmd && cmd_type == CMD_FORCE_TRIG) // Force trigger
                    || (do_next_cmd && cmd_type == CMD_SYNC_CH && all_waiting) // Sync channels edge case where all channels are already waiting
                    || (state == S_SYNC_CH && all_waiting) // Sync channels when all are waiting
                    || (state == S_EXPECT_TRIG && ext_trig_counter > 0 && lockout_counter == 0 && ext_trig_sync[1]); // External trigger when expected and lockout is done
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) trig_out <= 0;
    else trig_out <= do_trig; // Trigger pulse
  end
  // Trigger logging
  always @(posedge clk) begin
    if (!resetn) log_trig <= 0;
    else if (do_next_cmd) log_trig <= cmd_log_trig;
  end
  assign do_log = (do_next_cmd && cmd_type == CMD_FORCE_TRIG && cmd_log_trig) // Force trigger with logging
                  || (do_next_cmd && cmd_type == CMD_SYNC_CH && all_waiting && cmd_log_trig) // Sync channels edge case with logging
                  || (state == S_SYNC_CH && all_waiting && log_trig) // Sync channels with logging
                  || (state == S_EXPECT_TRIG && ext_trig_counter > 0 && lockout_counter == 0 && ext_trig_sync[1] && log_trig); // External trigger with logging

  //// Error handling
  // Bad command
  always @(posedge clk) begin
    if (!resetn) bad_cmd <= 0;
    else if (do_next_cmd && next_cmd_state == S_ERROR) bad_cmd <= 1;
  end
  // Data buffer overflow
  always @(posedge clk) begin
    if (!resetn) data_buf_overflow <= 0;
    else if (do_log && (data_buf_full || data_buf_almost_full)) data_buf_overflow <= 1;
  end

  //// Read enable
  assign cmd_word_rd_en = do_next_cmd || reset_count;

  //// Trigger counting
  // Increment trigger count on each trigger event
  always @(posedge clk) begin
    if (!resetn) trig_counter <= 0;
    else if (reset_count) trig_counter <= 0; // Reset counter on reset count command
    else if (do_trig) trig_counter <= trig_counter + 1;
  end

  //// Trigger timing tracking
  always @(posedge clk) begin
    if (!resetn) trig_timer <= 0;
    else if (reset_count) trig_timer <= 0; // Reset timer on reset count command
    else if (trig_timer == 0 && do_log) trig_timer <= 1; // Start timer on first trigger
    else if (trig_timer > 0 && trig_timer < 64'hFFFFFFFFFFFFFFFF) trig_timer <= trig_timer + 1; // Increment timer without overflow
  end

  //// Data buffer write logic
  // Write two sequential 32-bit words to the data buffer each trigger
  always @(posedge clk) begin
    if (!resetn) begin
      data_word_wr_en <= 0;
      data_word <= 32'h0;
      second_word <= 32'h0;
      trig_data_second_word <= 0;
    // If already writing a word
    end else if (data_word_wr_en) begin
      // If already writing the second word, disable write and reset flag
      if (trig_data_second_word) begin
        data_word_wr_en <= 0; // Disable write after second word
        trig_data_second_word <= 0; // Reset second word flag
      // If writing the first word, prepare to write the second word next cycle
      end else begin
        data_word <= second_word; // Write second word
        trig_data_second_word <= 1; // Set flag to write second word next cycle
      end
    // Write first word on trigger if buffer has at least two free entries
    end else if (do_log && !data_buf_full && !data_buf_almost_full) begin
      data_word_wr_en <= 1;
      data_word <= trig_timer[31:0]; // First word is the lower 32 bits of the timer
      second_word <= trig_timer[63:32]; // Second word is the upper 32 bits of the timer
    end
  end

endmodule
