`timescale 1 ns / 1 ps

module shim_ad5676_dac_ctrl #(
  parameter ABS_CAL_MAX = 16'd4096 // Maximum absolute calibration value
)(
  input  wire        clk,
  input  wire        resetn,

  input  wire        boot_test_skip, // Skip the boot test sequence
  input  wire        debug, // Debug mode flag
  input  wire [4:0]  n_cs_high_time, // n_cs high time in clock cycles (max 31)

  input  wire signed [15:0] cal_init_val, // Default calibration value for all channels on reset (in 2's complement)

  output reg         setup_done,

  output wire        cmd_buf_rd_en,
  input  wire [31:0] cmd_buf_word,
  input  wire        cmd_buf_empty,

  output reg         data_buf_wr_en,
  output reg  [31:0] data_word,
  input  wire        data_buf_full,

  input  wire        trigger,
  input  wire        ldac_shared,
  output wire        waiting_for_trig,

  output reg         boot_fail,
  output reg         cmd_buf_underflow,
  output reg         data_buf_overflow,
  output reg         unexp_trig,
  output reg         ldac_misalign,
  output reg         delay_too_short,
  output reg         bad_cmd,
  output reg         cal_oob,
  output reg         dac_val_oob,
  
  output reg [119:0] abs_dac_val_concat,

  output reg         n_cs,
  output wire        mosi,
  input  wire        miso_sck,
  input  wire        miso_resetn,
  input  wire        miso,
  output reg         ldac
);

  ///////////////////////////////////////////////////////////////////////////////
  // SPI Timing Parameters
  ///////////////////////////////////////////////////////////////////////////////

  // SPI command bit width
  localparam integer SPI_CMD_BITS = 24;


  ///////////////////////////////////////////////////////////////////////////////
  // SPI Command Bit Definitions
  ///////////////////////////////////////////////////////////////////////////////

  localparam DAC_TEST_CH   = 3'd5;      // DAC channel for testing
  localparam DAC_TEST_VAL  = 16'h800A;  // Test value for DAC channel
  localparam DAC_MIDRANGE  = 16'h8000;  // Midrange value

  localparam SPI_CMD_REG_WRITE = 4'b0001; // Register write command
  localparam SPI_CMD_REG_READ  = 4'b1001; // Register read command


  ///////////////////////////////////////////////////////////////////////////////
  // State Machine and Command Definitions
  ///////////////////////////////////////////////////////////////////////////////

  // FSM states
  localparam S_RESET     = 4'd0;
  localparam S_INIT      = 4'd1;
  localparam S_TEST_WR   = 4'd2;
  localparam S_REQ_RD    = 4'd3;
  localparam S_TEST_RD   = 4'd4;
  localparam S_SET_MID   = 4'd5;
  localparam S_IDLE      = 4'd6;
  localparam S_DELAY     = 4'd7;
  localparam S_TRIG_WAIT = 4'd8;
  localparam S_DAC_WR    = 4'd9;
  localparam S_DAC_WR_CH = 4'd10;
  localparam S_ERROR     = 4'd15;

  // Command types
  localparam CMD_NO_OP     = 3'd0;
  localparam CMD_SET_CAL   = 3'd1;
  localparam CMD_DAC_WR    = 3'd2;
  localparam CMD_DAC_WR_CH = 3'd3;
  localparam CMD_GET_CAL   = 3'd4;
  localparam CMD_CANCEL    = 3'd7;

  // Command bit positions
  localparam TRIG_BIT = 28;
  localparam CONT_BIT = 27;
  localparam LDAC_BIT = 26;

  // DAC loading stages
  localparam DAC_LOAD_STAGE_INIT = 2'b00;
  localparam DAC_LOAD_STAGE_CAL  = 2'b01;
  localparam DAC_LOAD_STAGE_CONV = 2'b10;

  // Debug codes
  localparam DBG_MISO_DATA        = 4'd1;
  localparam DBG_STATE_TRANSITION = 4'd2;
  localparam DBG_N_CS_TIMER       = 4'd3;
  localparam DBG_SPI_BIT          = 4'd4;
  localparam CAL_DATA             = 4'd8;


  ///////////////////////////////////////////////////////////////////////////////
  // Internal Signals
  ///////////////////////////////////////////////////////////////////////////////

  //// ---- State machine and command control
  // FSM state and previous state
  reg  [ 3:0] state, prev_state;
  // Command flow control
  wire [ 2:0] command;
  wire [31:0] cmd_word;
  wire        next_cmd_ready;
  wire        cmd_done;
  wire        do_next_cmd;
  wire [ 3:0] next_cmd_state;
  wire        cancel_wait;
  wire        error;
  // Command word toggled bits
  reg         do_ldac;
  reg         wait_for_trig;
  wire        trig_wait_done;
  wire        delay_wait_done;
  reg         expect_next;
  // Delay timer and trigger counter
  reg  [24:0] delay_timer, trigger_counter;
  // Calibration values
  reg  signed [15:0] cal_val [0:7];

  //// ---- Calibrated DAC value calculation
  reg         load_pair;
  reg  [1:0]  dac_load_stage;
  reg  signed [15:0] first_dac_val_signed;
  reg  signed [16:0] first_dac_val_cal_signed;
  reg  signed [15:0] second_dac_val_signed;
  reg  signed [16:0] second_dac_val_cal_signed;
  reg  [14:0] abs_dac_val [0:7];
  wire        first_dac_val_cal_signed_oob;
  wire        second_dac_val_cal_signed_oob;

  //// ---- DAC MOSI SPI control
  reg         read_next_dac_val_pair;
  wire        start_spi_cmd;
  reg         dac_wr_done;
  wire        last_dac_channel;
  wire        second_dac_channel_of_pair;
  wire        dac_spi_cmd_done;
  // Chip select control
  reg  [ 4:0] n_cs_timer;
  reg         running_n_cs_timer;
  wire        cs_wait_done;
  // Latched n_cs high time
  reg  [ 4:0] n_cs_high_time_latched;
  // SPI channel index and bit counter
  reg  [ 2:0] dac_channel;
  reg  [ 4:0] spi_bit;
  reg         running_spi_bit;
  // SPI MOSI shift register
  reg  [47:0] mosi_shift_reg;

  //// ---- DAC MISO SPI control
  reg  [14:0] miso_shift_reg;
  wire [15:0] miso_data;
  reg  [ 3:0] miso_bit;
  reg         miso_buf_wr_en;
  // MISO read synchronization
  reg         start_miso_mosi_clk;
  wire        start_miso;
  wire        n_miso_data_ready_mosi_clk;
  wire [15:0] miso_data_mosi_clk;
  // Boot test readback match
  wire        boot_readback_match;

  //// ---- Data buffer write signals
  wire        write_cal;
  wire        debug_miso_data;
  wire        debug_state_transition;
  wire        debug_n_cs_timer;
  wire        debug_spi_bit;
  wire        try_data_write;


  ///////////////////////////////////////////////////////////////////////////////
  // Logic
  ///////////////////////////////////////////////////////////////////////////////

  //// ---- Command word
  assign cmd_word = cmd_buf_empty ? 32'd0 : cmd_buf_word;
  assign command = cmd_word[31:29];
  assign next_cmd_ready = !cmd_buf_empty;
  // Command word read enable
  assign cmd_buf_rd_en = (state != S_ERROR) && next_cmd_ready && (read_next_dac_val_pair || cmd_done || cancel_wait);
  // Command bits processing
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) begin
      do_ldac <= 1'b0;
      wait_for_trig <= 1'b0;
      expect_next <= 1'b0;
    end else if (do_next_cmd) begin
      // Set LDAC, wait_for_trig, and expect_next flags from command bits if NO_OP or DAC_WR command
      if ((command == CMD_NO_OP ) || (command == CMD_DAC_WR)) begin
        do_ldac <= cmd_word[LDAC_BIT];
        wait_for_trig <= cmd_word[TRIG_BIT];
        expect_next <= cmd_word[CONT_BIT];
      // For the single-channel DAC_WR_CH command, always set do_ldac to 1, wait_for_trig to 1, and expect_next to 0
      end else if (command == CMD_DAC_WR_CH) begin
        do_ldac <= 1'b1;
        wait_for_trig <= 1'b1;
        expect_next <= 1'b0;
      // Otherwise set flags to 0
      end else begin
        do_ldac <= 1'b0;
        wait_for_trig <= 1'b0;
        expect_next <= 1'b0;
      end
    end
  end


  //// ---- State machine transitions
  // Allow a cancel command to cancel a delay or trigger wait
  assign cancel_wait =  (state == S_DELAY || state == S_TRIG_WAIT || (state == S_DAC_WR && dac_wr_done))
                        && next_cmd_ready 
                        && command == CMD_CANCEL;
  // Trigger wait is done when trigger counter occurs at 1, or finish immediately if trigger counter was 0
  assign trig_wait_done = (trigger && trigger_counter == 1) || trigger_counter == 0;
  // Delay wait is done when delay timer reaches 0
  assign delay_wait_done = (delay_timer == 0);
  // Current command is finished
  assign cmd_done = (state == S_IDLE && next_cmd_ready) 
                    || (state == S_DELAY && delay_wait_done)
                    || (state == S_TRIG_WAIT && trig_wait_done)
                    || (state == S_DAC_WR && dac_wr_done && (wait_for_trig ? trig_wait_done : delay_wait_done))
                    || (state == S_DAC_WR_CH && dac_wr_done);
  assign do_next_cmd = cmd_done && next_cmd_ready;
  // Next state from upcoming command
  assign next_cmd_state = !next_cmd_ready ? (expect_next ? S_ERROR : S_IDLE) // If buffer is empty, error if expecting next command, otherwise IDLE
                          : (command == CMD_NO_OP) ? (cmd_word[TRIG_BIT] ? S_TRIG_WAIT : S_DELAY) // If command is NO_OP, either wait for trigger or delay depending on TRIG_BIT
                          : (command == CMD_SET_CAL) ? S_IDLE // If command is SET_CAL, go to IDLE
                          : (command == CMD_DAC_WR) ? S_DAC_WR // If command is DAC write, go to DAC_WR state
                          : (command == CMD_DAC_WR_CH) ? S_DAC_WR_CH // If command is single-channel DAC write, go to DAC_WR_CH state
                          : (command == CMD_CANCEL) ? S_IDLE // If command is CANCEL, go to IDLE 
                          : (command == CMD_GET_CAL) ? S_IDLE // If command is GET_CAL, go to IDLE
                          : S_ERROR; // If command is not recognized, go to ERROR state
  // Waiting for trigger flag
  assign waiting_for_trig = (state == S_TRIG_WAIT);
  // State transition
  // Next state
  always @(posedge clk) begin
    if (!resetn)                                                state <= S_RESET; // Reset to initial state
    else if (error)                                             state <= S_ERROR; // Check for error states
    else if (state == S_RESET)                                  state <= boot_test_skip ? S_IDLE : S_INIT; // Skip boot test if requested
    else if (state == S_INIT)                                   state <= S_TEST_WR; // Transition to TEST_WR first in initialization
    else if (state == S_TEST_WR && dac_spi_cmd_done)            state <= S_REQ_RD; // Transition to REQ_RD after writing test value
    else if (state == S_REQ_RD && dac_spi_cmd_done)             state <= S_TEST_RD; // Transition to TEST_RD after requesting read
    else if (state == S_TEST_RD && ~n_miso_data_ready_mosi_clk) state <= boot_readback_match ? S_SET_MID : S_ERROR; // Transition to SET_MID if readback matches, otherwise error
    else if (state == S_SET_MID && dac_wr_done)                 state <= S_IDLE; // Transition to IDLE after setting values to midrange
    else if (cancel_wait)                                       state <= S_IDLE; // Cancel the current wait state if cancel command is received
    else if (cmd_done)                                          state <= next_cmd_state; // Transition to state of next command if command is finished (allows skipping wait state if no wait is needed)
    else if (state == S_DAC_WR && dac_wr_done)                  state <= wait_for_trig ? S_TRIG_WAIT : S_DELAY; // If the DAC write is done, go to the proper wait state
  end
  // Previous state
  always @(posedge clk) begin
    prev_state <= state; // Store the previous state for debugging
  end
  // Setup done
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) setup_done <= 1'b0; // Reset setup done on reset or error
    else if (boot_test_skip) setup_done <= 1'b1; // If boot test is skipped, set setup done immediately
    else if (state == S_SET_MID && dac_wr_done) setup_done <= 1'b1;
  end


  //// ---- Delay timer
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR || cancel_wait) delay_timer <= 25'd0;
    // If the next command is a DAC write or no-op with a delay wait, load the delay timer from command word
    else if (do_next_cmd 
             && ((command == CMD_DAC_WR) || (command == CMD_NO_OP)) 
             && !cmd_word[TRIG_BIT]) begin
      delay_timer <= cmd_word[24:0];
    // Otherwise decrement delay timer to zero if nonzero
    end else if (delay_timer > 0) delay_timer <= delay_timer - 1;
  end


  //// ---- Trigger counter
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR || cancel_wait) trigger_counter <= 25'd0;
    // If the next command is a DAC write or no-op with a trigger wait, load the trigger counter from command word
    else if (do_next_cmd 
             && ((command == CMD_DAC_WR) || (command == CMD_NO_OP)) 
             && cmd_word[TRIG_BIT]) begin
      trigger_counter <= cmd_word[24:0];
    // Single-channel DAC write commands immediately finish
    end else if (do_next_cmd && command == CMD_DAC_WR_CH) begin
      trigger_counter <= 25'd0;
    // Otherwise decrement trigger counter to zero if nonzero
    end else if (trigger_counter > 0 && trigger) trigger_counter <= trigger_counter - 1;
  end


  //// ---- Errors
  // Error flag
  assign error = (state == S_TEST_RD && ~n_miso_data_ready_mosi_clk && ~boot_readback_match) // Readback mismatch (boot fail)
                 || (state != S_TRIG_WAIT && trigger && trigger_counter <= 1) // Unexpected final trigger
                 || (state == S_DAC_WR && !dac_wr_done && !wait_for_trig && delay_wait_done) // Delay too short
                 || ((state == S_DAC_WR || state == S_DAC_WR_CH) && ldac_shared && !ldac) // LDAC misalignment
                 || (do_next_cmd && next_cmd_state == S_ERROR) // Bad command
                 || (((cmd_done && expect_next) || read_next_dac_val_pair) && !next_cmd_ready) // Command buffer underflow
                 || (try_data_write && data_buf_full) // Data buffer overflow
                 || cal_oob // Calibration value out of bounds
                 || dac_val_oob; // DAC value out of bounds
  // Boot check fail
  assign boot_readback_match = (miso_data_mosi_clk == DAC_TEST_VAL); // Readback matches the test value
  always @(posedge clk) begin
    if (!resetn) boot_fail <= 1'b0; // Reset boot fail on reset
    if (state == S_TEST_RD && ~n_miso_data_ready_mosi_clk) boot_fail <= ~boot_readback_match; 
  end
  // Unexpected trigger
  always @(posedge clk) begin
    if (!resetn) unexp_trig <= 1'b0;
    else if (state != S_TRIG_WAIT && trigger && trigger_counter <= 1) unexp_trig <= 1'b1; // Unexpected trigger if triggered while not waiting for the last one
  end
  // Delay too short
  always @(posedge clk) begin
    if (!resetn) delay_too_short <= 1'b0;
    else if (state == S_DAC_WR && !dac_wr_done && !wait_for_trig && delay_wait_done) delay_too_short <= 1'b1; // Delay too short if delay timer is zero before DAC write is done
  end
  // LDAC misalignment
  always @(posedge clk) begin
    if (!resetn) ldac_misalign <= 1'b0;
    else if ((state == S_DAC_WR || state == S_DAC_WR_CH) && ldac_shared && !ldac) ldac_misalign <= 1'b1; // LDAC misalignment if LDAC is shared and another controller is writing to DAC
  end
  // Bad command
  always @(posedge clk) begin
    if (!resetn) bad_cmd <= 1'b0;
    else if (do_next_cmd && next_cmd_state == S_ERROR) bad_cmd <= 1'b1; // Bad command if next command is parsed as ERROR
  end
  // Command buffer underflow
  always @(posedge clk) begin
    if (!resetn) cmd_buf_underflow <= 1'b0;
    else if (((cmd_done && expect_next) || read_next_dac_val_pair) && !next_cmd_ready) cmd_buf_underflow <= 1'b1; // Underflow if expecting buffer item but buffer is empty
  end
  // Data buffer overflow
  always @(posedge clk) begin
    if (!resetn) data_buf_overflow <= 1'b0;
    else if (try_data_write && data_buf_full) data_buf_overflow <= 1'b1;
  end
  // DAC val out of bounds
  assign first_dac_val_cal_signed_oob = (first_dac_val_cal_signed < -16'sd32767 || first_dac_val_cal_signed > 16'sd32767);
  assign second_dac_val_cal_signed_oob = (second_dac_val_cal_signed < -16'sd32767 || second_dac_val_cal_signed > 16'sd32767);
  always @(posedge clk) begin
    if (!resetn) dac_val_oob <= 1'b0; // Reset out of bounds flag on reset
    else begin // Set out of bounds flag if either of the following conditions are met:
      if (dac_load_stage == DAC_LOAD_STAGE_INIT
          && read_next_dac_val_pair && next_cmd_ready 
          && (cmd_word[15:0] == 16'hFFFF || cmd_word[31:16] == 16'hFFFF)) dac_val_oob <= 1'b1; // If incoming DAC value is 0xFFFF
      else if (dac_load_stage == DAC_LOAD_STAGE_CONV && (first_dac_val_cal_signed_oob || second_dac_val_cal_signed_oob)) dac_val_oob <= 1'b1; // If calibrated DAC value is out of bounds
    end
  end


  //// ---- DAC updating
  // LDAC activation
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) ldac <= 1'b0;
    else if (do_ldac && cmd_done && !cancel_wait) begin
      ldac <= 1'b1; // If do_ldac is set, activate LDAC at the end of the command (except for IDLE)
    end
    else ldac <= 1'b0; // Otherwise, deactivate LDAC
  end
  // Update absolute DAC values concatenation
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) abs_dac_val_concat <= 120'd0; // Reset concatenation on reset or error
    else if (ldac) begin
      // Concatenate absolute DAC values when LDAC is asserted
      abs_dac_val_concat <= {abs_dac_val[7], abs_dac_val[6], abs_dac_val[5], 
                             abs_dac_val[4], abs_dac_val[3], abs_dac_val[2], 
                             abs_dac_val[1], abs_dac_val[0]};
    end
  end


  //// ---- Calibration
  // Calibration value set and out-of-bounds
  always @(posedge clk) begin
    if (!resetn) begin
      cal_val[0] <= cal_init_val;
      cal_val[1] <= cal_init_val;
      cal_val[2] <= cal_init_val;
      cal_val[3] <= cal_init_val;
      cal_val[4] <= cal_init_val;
      cal_val[5] <= cal_init_val;
      cal_val[6] <= cal_init_val;
      cal_val[7] <= cal_init_val;
      cal_oob <= 1'b0;
    end else if (do_next_cmd && command == CMD_SET_CAL) begin
      if ($signed(cmd_word[15:0]) <= $signed(ABS_CAL_MAX) && $signed(cmd_word[15:0]) >= -$signed(ABS_CAL_MAX)) begin
        cal_val[cmd_word[18:16]] <= cmd_word[15:0]; // Set calibration value for the channel if within bounds
      end else begin
        cal_oob <= 1'b1; // Set out-of-bounds flag if calibration value is out of range
      end
    end
  end

  //// ---- DAC word sequencing
  // DAC channel count status
  assign last_dac_channel = (dac_channel == 3'd7); // Last channel is when all bits are set
  assign second_dac_channel_of_pair = (dac_channel[0] == 1'b1); // Even channel is when the least significant bit is set (off by 1)
  assign dac_spi_cmd_done = ((state == S_DAC_WR)
                             || (state == S_DAC_WR_CH)
                             || (state == S_TEST_WR)
                             || (state == S_REQ_RD)
                             || (state == S_TEST_RD)
                             || (state == S_SET_MID))
                            && !n_cs && !running_n_cs_timer && spi_bit == 0; // SPI command is done when CS is deasserted and SPI bit counter is zero
  // Read next DAC word from command buffer
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) read_next_dac_val_pair <= 1'b0;
    // If next command is DAC write, immediately read next DAC word (two channels)
    else if (do_next_cmd && command == CMD_DAC_WR) read_next_dac_val_pair <= 1'b1;
    // If done writing to DAC and finished the second channel of the update pair, 
    //   but it's not the last pair, read the next word (pair of channels)
    else if (state == S_DAC_WR
             && dac_spi_cmd_done
             && second_dac_channel_of_pair 
             && !last_dac_channel) read_next_dac_val_pair <= 1'b1;
    else read_next_dac_val_pair <= 1'b0;
  end
  // DAC write done
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) dac_wr_done <= 1'b0;
    else if ((state == S_DAC_WR || state == S_SET_MID) && dac_spi_cmd_done && last_dac_channel) dac_wr_done <= 1'b1; // Ready when all channels are written
    else if (state == S_DAC_WR_CH && dac_spi_cmd_done) dac_wr_done <= 1'b1; // Ready when single channel is written
    else dac_wr_done <= 1'b0; // Not ready otherwise
  end
  // DAC channel index
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) dac_channel <= 3'd0;
    else if (do_next_cmd && command == CMD_DAC_WR) dac_channel <= 3'd0;
    else if (do_next_cmd && command == CMD_DAC_WR_CH) dac_channel <= cmd_word[18:16]; // Set channel from command word for single-channel write
    else if ((state == S_DAC_WR || state == S_SET_MID) && dac_spi_cmd_done) dac_channel <= dac_channel + 1; // Increment channel when timer is done
  end
  // DAC value loading
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) begin
      first_dac_val_signed <= 16'd0;
      first_dac_val_cal_signed <= 17'd0;
      second_dac_val_signed <= 16'd0;
      second_dac_val_cal_signed <= 17'd0;
      abs_dac_val[0] <= 15'd0;
      abs_dac_val[1] <= 15'd0;
      abs_dac_val[2] <= 15'd0;
      abs_dac_val[3] <= 15'd0;
      abs_dac_val[4] <= 15'd0;
      abs_dac_val[5] <= 15'd0;
      abs_dac_val[6] <= 15'd0;
      abs_dac_val[7] <= 15'd0;
      dac_load_stage <= DAC_LOAD_STAGE_INIT; // Reset DAC load stage
    end else 
      case (dac_load_stage)
        DAC_LOAD_STAGE_INIT: begin // Initial stage, waiting for the first DAC value to be loaded
          // Load DAC values in pairs when doing a full 8-channel write
          if (read_next_dac_val_pair && next_cmd_ready) begin
            // Reject DAC value of 0xFFFF
            if (!(cmd_word[15:0] == 16'hFFFF || cmd_word[31:16] == 16'hFFFF))  begin
              first_dac_val_signed <= offset_to_signed(cmd_word[15:0]); // Load first DAC value from command word
              second_dac_val_signed <= offset_to_signed(cmd_word[31:16]); // Load second DAC value from command word
              dac_load_stage <= DAC_LOAD_STAGE_CAL; // Move to next stage
            end
          // Load a single value from the command word for single-channel write
          end else if (do_next_cmd && command == CMD_DAC_WR_CH) begin
            // Reject DAC value of 0xFFFF
            if (cmd_word[15:0] != 16'hFFFF) begin
              first_dac_val_signed <= offset_to_signed(cmd_word[15:0]); // Load DAC value from command word
              second_dac_val_signed <= 16'd0; // No second DAC value
              dac_load_stage <= DAC_LOAD_STAGE_CAL; // Move to next stage
            end
          end
        end
        DAC_LOAD_STAGE_CAL: begin // Second stage, adding calibration and getting absolute values
          // For single-channel write, only handle the one channel
          first_dac_val_cal_signed <= first_dac_val_signed + cal_val[dac_channel]; // Add calibration to first DAC value
          abs_dac_val[dac_channel] <= signed_to_abs(first_dac_val_cal_signed); // Convert first DAC value to absolute
          // For full 8-channel write, handle both channels of the pair
          if (state == S_DAC_WR) begin
            second_dac_val_cal_signed <= second_dac_val_signed + cal_val[dac_channel+1]; // Add calibration to second DAC value
            abs_dac_val[dac_channel + 1] <= signed_to_abs(second_dac_val_cal_signed); // Convert second DAC value to absolute
          end
          dac_load_stage <= DAC_LOAD_STAGE_CONV; // Move to final stage
        end
        DAC_LOAD_STAGE_CONV: begin // Final conversion stage, converting to offset representation
          // Logic is handled in the SPI MOSI control shift register
          // OOB is checked in the DAC val out of bounds section
          dac_load_stage <= DAC_LOAD_STAGE_INIT; // Conversion is done
        end
      endcase
  end


  //// ---- SPI MOSI control
  // Start the next SPI command
  assign start_spi_cmd =  (do_next_cmd && command == CMD_DAC_WR)
                          || (do_next_cmd && command == CMD_DAC_WR_CH)
                          || (state == S_INIT)
                          || (state == S_TEST_WR && dac_spi_cmd_done)
                          || (state == S_REQ_RD && dac_spi_cmd_done)
                          || (state == S_TEST_RD && dac_spi_cmd_done)
                          || (state == S_SET_MID && dac_spi_cmd_done && !last_dac_channel)
                          || (state == S_DAC_WR && dac_spi_cmd_done && !last_dac_channel);
  // Latch ~(Chip Select) high time when coming out of reset
  always @(posedge clk) begin
    if (!resetn) n_cs_high_time_latched <= 5'd0;
    else if (state == S_RESET) n_cs_high_time_latched <= n_cs_high_time;
  end
  // ~(Chip Select) timer
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) n_cs_timer <= 5'd0;
    else if (start_spi_cmd) n_cs_timer <= n_cs_high_time_latched;
    else if (n_cs_timer > 0) n_cs_timer <= n_cs_timer - 1;
    running_n_cs_timer <= (n_cs_timer > 0); // Flag to indicate if CS timer is running
  end
  // ~(Chip Select) (n_cs) has been high for the required time (timer went from nonzero to zero)
  assign cs_wait_done = (running_n_cs_timer && n_cs_timer == 0);
  // ~(Chip Select) (n_cs) signal
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) n_cs <= 1'b1; // Reset n_CS on reset or error
    else if (cs_wait_done) n_cs <= 1'b0; // Assert CS when timer is done
    else if (dac_spi_cmd_done || state == S_IDLE) n_cs <= 1'b1; // Deassert CS when SPI command is done
  end
  // DAC word SPI bit
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) spi_bit <= 5'd0;
    else if (spi_bit > 0) spi_bit <= spi_bit - 1; // Decrement SPI bit counter
    else if (cs_wait_done) spi_bit <= 5'd23; // Load SPI bit counter with 24 bits when CS is done waiting
    running_spi_bit <= (spi_bit > 0); // Flag to indicate if SPI bit counter is running
  end
  // SPI MOSI bit
  assign mosi = mosi_shift_reg[47]; // MOSI is the most significant bit of the shift register
  // SPI MOSI shift register
  always @(posedge clk) begin
    // Reset shift register on reset or error
    if (!resetn || state == S_ERROR) mosi_shift_reg <= 48'd0; 
     // Shift bits out
    else if (spi_bit > 0) mosi_shift_reg <= {mosi_shift_reg[46:0], 1'b0};
    // If just exiting reset load the shift register with the test value for boot-up sequence
    else if (state == S_INIT) begin
      mosi_shift_reg <= {spi_write_cmd(DAC_TEST_CH, DAC_TEST_VAL), 24'h000000};
    // If finished with the test write, load the shift register with two commands: the read request and a write to reset the test value
    end else if (state == S_TEST_WR && dac_spi_cmd_done) begin
      mosi_shift_reg <= {spi_read_cmd(DAC_TEST_CH), spi_write_cmd(DAC_TEST_CH, DAC_MIDRANGE)};
    // If finished with the read request and overwrite, initialize all channels to midrange
    end else if (state == S_TEST_RD && dac_spi_cmd_done) begin
      mosi_shift_reg <= {spi_write_cmd(0, DAC_MIDRANGE), spi_write_cmd(1, DAC_MIDRANGE)};
    // When finished setting midrange values for a channel pair, load the next pair until all channels are set
    end else if (state == S_SET_MID && dac_spi_cmd_done && !last_dac_channel) begin
      mosi_shift_reg <= {spi_write_cmd(dac_channel + 1, DAC_MIDRANGE), spi_write_cmd(dac_channel + 2, DAC_MIDRANGE)};
    // For full 8-channel DAC commands, load the shift register with the first DAC value and the second DAC value
    end else if (state == S_DAC_WR && dac_load_stage == DAC_LOAD_STAGE_CONV) begin
      mosi_shift_reg <= {spi_write_cmd(dac_channel, signed_to_offset(first_dac_val_cal_signed)), 
                        spi_write_cmd(dac_channel + 1, signed_to_offset(second_dac_val_cal_signed))};
    end else if (state == S_DAC_WR_CH && dac_load_stage == DAC_LOAD_STAGE_CONV) begin
      mosi_shift_reg <= {spi_write_cmd(dac_channel, signed_to_offset(first_dac_val_cal_signed)), 24'h000000};
    // Shift once more when transitioning between words (when shift register is loaded with two 24-bit words)
    end else if (running_spi_bit) begin 
       mosi_shift_reg <= {mosi_shift_reg[46:0], 1'b0};
    end
  end
  // Start MISO read in MOSI clock domain
  // (should show up 1 cycle later on readback MISO clock than equivalent MOSI clock cycle, plus 2 for the synchronizer)
  always @(posedge clk) begin
    if (!resetn || state == S_ERROR) start_miso_mosi_clk <= 1'b0; // Reset start MISO read signal on reset or error
    else if (state == S_TEST_RD && spi_bit == 5'd18) start_miso_mosi_clk <= 1'b1;
    else start_miso_mosi_clk <= 1'b0;
  end

  //// ---- SPI MISO control
  // Start MISO synchonization
  sync_incoherent start_miso_sync(
    .clk(miso_sck), // MISO clock
    .resetn(miso_resetn), // Reset for MISO clock domain
    .din(start_miso_mosi_clk), // Start MISO read signal in MOSI clock domain
    .dout(start_miso) // Start MISO read signal in MISO clock domain
  );
  // MISO FIFO
  fifo_async #(
    .DATA_WIDTH  (16), // MISO data width
    .ADDR_WIDTH  (2) // FIFO address width (4 entries)
  ) miso_fifo (
    .wr_clk      (miso_sck), // MISO clock
    .wr_rst_n    (miso_resetn), // Reset for MISO clock domain
    .wr_data     (miso_data), // MISO data to write
    .wr_en       (miso_buf_wr_en), // Write enable for MISO data

    .rd_clk      (clk), // FPGA SCK
    .rd_rst_n    (resetn),
    .rd_data     (miso_data_mosi_clk),
    .rd_en       (~n_miso_data_ready_mosi_clk), // Immediately read MISO data when available in the MOSI clock domain
    .empty       (n_miso_data_ready_mosi_clk)
  );
  // MISO bit counter
  always @(posedge miso_sck) begin
    if (!miso_resetn) miso_bit <= 4'd0; // Reset MISO bit counter on reset
    else if (miso_bit > 0) miso_bit <= miso_bit - 1; // Decrement MISO bit counter
    else if (start_miso) miso_bit <= 4'd15; // Load MISO bit counter with 16 bits when starting MISO read
  end
  // MISO shift register
  always @(posedge miso_sck) begin
    if (!miso_resetn) miso_shift_reg <= 15'd0;
    else if (miso_bit > 1) miso_shift_reg <= {miso_shift_reg[13:0], miso}; // Shift MISO data into the shift register
    else if (start_miso) miso_shift_reg <= {14'd0, miso}; // Start MISO read
  end
  assign miso_data = {miso_shift_reg, miso}; // MISO data is the shift register with the last bit from MISO
  // MISO buffer write enable
  always @(posedge miso_sck) begin
    if (!miso_resetn) miso_buf_wr_en <= 1'b0; // Reset MISO buffer write enable on reset
    else if (miso_bit == 1) miso_buf_wr_en <= 1'b1; // Write MISO data to FIFO when last bit is received
    else miso_buf_wr_en <= 1'b0;
  end

  //// ---- DAC data output
  // Write calibration value to data buffer
  assign write_cal = do_next_cmd && command == CMD_GET_CAL;
  // DEBUG: MISO data ready in MOSI clock domain
  assign debug_miso_data = (state == S_TEST_RD && ~n_miso_data_ready_mosi_clk && debug);
  // DEBUG: State transition
  assign debug_state_transition = (state != prev_state && debug);
  // DEBUG: n_cs_timer start value
  assign debug_n_cs_timer = (!running_n_cs_timer && n_cs_timer > 0 && debug);
  // DEBUG: SPI bit counter when it changes from 0 to nonzero
  assign debug_spi_bit = (!running_spi_bit && spi_bit > 0 && debug); 
  // Attempt to write data to the data buffer if any of the following are true
  assign try_data_write = write_cal
                          || debug_miso_data
                          || debug_state_transition
                          || debug_n_cs_timer
                          || debug_spi_bit;
  // DAC data output write enable
  // Write MISO data to the data buffer when attempting a write and buffer isn't full
  always @(posedge clk) begin
    if (!resetn) data_buf_wr_en <= 1'b0; // Reset data word write enable on reset
    else if (try_data_write && !data_buf_full) data_buf_wr_en <= 1'b1; // Write data word when two words are ready and buffer isn't full
    else data_buf_wr_en <= 1'b0;
  end
  // MISO data word
  always @(posedge clk) begin
    if (!resetn) data_word <= 32'd0; // Reset data word on reset or error
    else if (try_data_write && !data_buf_full) begin
      if (write_cal) begin
        data_word <= {CAL_DATA, 9'd0, cmd_word[18:16], cal_val[cmd_word[18:16]]}; // Write calibration value with channel number and debug code
      end else if (debug_miso_data) begin
        data_word <= {DBG_MISO_DATA, 12'd0, miso_data_mosi_clk[15:0]}; // Write MISO data with debug code
      end else if (debug_state_transition) begin
        data_word <= {DBG_STATE_TRANSITION, 20'd0, prev_state[3:0], state[3:0]}; // Write state transition with debug code
      end else if (debug_n_cs_timer) begin
        data_word <= {DBG_N_CS_TIMER, 23'd0, n_cs_timer}; // Write n_cs timer value with debug code
      end else if (debug_spi_bit) begin
        data_word <= {DBG_SPI_BIT, 23'd0, spi_bit}; // Write SPI bit counter value with debug code
      end
    end else data_word <= 32'd0;
  end

  //// ---- Functions for conversions
  // Convert from offset to signed     
  // Given a 16-bit 0-65535 number, treat 32768 (0x8000) as 0, 0 as -32768, 
  //   and 65535 as disallowed (return 0, handle the error before calling)
  function signed [15:0] offset_to_signed(input [15:0] raw_dac_val);
    begin
      if (raw_dac_val == 16'hFFFF) offset_to_signed = 16'sd0; // Disallowed value, return 0
      else offset_to_signed = $signed(raw_dac_val) - 16'sd32768; // Correct conversion for full 16-bit range
    end
  endfunction
  // Convert the signed value to absolute value
  function [14:0] signed_to_abs(input signed [15:0] signed_val);
    begin
      if (signed_val < 0) begin
        signed_to_abs = -signed_val; // If negative, take the absolute value
      end else begin
        signed_to_abs = signed_val; // If positive, keep the value as is
      end
    end
  endfunction
  // Convert signed value to offset (0-65535) representation
  // Inverse of offset_to_signed: offset = signed_val + 32768 (0x8000)
  //   Should handle out of bounds before calling, but will return 32768 if out of bounds
  function [15:0] signed_to_offset(input signed [16:0] signed_val);
    begin
      if (signed_val < -16'sd32768 || signed_val > 16'sd32767) signed_to_offset = 16'd32768; // If out of bounds, return offset representation of 0
      else signed_to_offset = signed_val + 16'sd32768;
    end
  endfunction
  // SPI command to write to particular DAC channel, waiting for LDAC
  function [23:0] spi_write_cmd(input [2:0] channel, input [15:0] dac_val);
    spi_write_cmd = {SPI_CMD_REG_WRITE, 1'b0, channel, dac_val}; // Construct the SPI command with write command and channel
  endfunction
  // SPI command to read from particular DAC channel on MISO during the next SPI word
  function [23:0] spi_read_cmd(input [2:0] channel);
    spi_read_cmd = {SPI_CMD_REG_READ, 1'b0, channel, 16'b0}; // Construct the SPI command with read command and channel
  endfunction

endmodule
