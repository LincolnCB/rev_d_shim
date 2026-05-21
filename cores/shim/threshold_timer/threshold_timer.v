`timescale 1 ns / 1 ps

module threshold_timer (
  // Inputs
  input   wire         clk              ,
  input   wire         resetn           ,
  input   wire         enable           ,
  input   wire [ 31:0] window           ,
  input   wire [ 14:0] thresh_val       ,
  input   wire         sample_core_setup,
  input   wire [119:0] abs_sample_concat,

  // Outputs
  output  reg         over_thresh  ,
  output  reg         setup_done
);

  reg  [31:0] window_latched;
  reg  [31:0] threshold_timer [0:7];
  reg  [14:0] thresh_val_latched;
  reg  [ 7:0] ch_threshold_viol;
  reg  [ 2:0] state;

  // State encoding
  localparam  IDLE  = 3'd0,
              WAIT  = 3'd1,
              RUNNING = 3'd2,
              OUT_OF_BOUNDS = 3'd3;
  
  always @(posedge clk) begin
    if (!resetn) begin
      // Reset all registers and outputs
      window_latched <= 0;
      thresh_val_latched <= 0;
      over_thresh <= 0;
      setup_done <= 0;
      state <= IDLE;
    end else begin
      case (state)
        // IDLE state, waiting for enable signal to latch inputs and start setup
        IDLE: begin : idle_state
          if (enable) begin
            // Latch inputs
            window_latched <= window;
            thresh_val_latched <= thresh_val;

            state <= WAIT; // Move to WAIT state to wait for sample core setup
          end
        end // IDLE

        // WAIT state, waiting for sample core (DAC/ADC) to finish setting up
        WAIT: begin : wait_state
          if (sample_core_setup) begin
            setup_done <= 1;
            state <= RUNNING;
          end
        end // WAIT

        // RUNNING state, main logic
        RUNNING: begin : running_state
          if (|ch_threshold_viol) begin
            over_thresh <= 1;
            state <= OUT_OF_BOUNDS;
          end
        end // RUNNING

        // OUT_OF_BOUNDS state, latched over_thresh until reset
        OUT_OF_BOUNDS: begin : out_of_bounds_state
          over_thresh <= 1;
        end // OUT_OF_BOUNDS
      endcase
    end
  end

  // Generate threshold violation signals for each channel by tracking net time above threshold
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : gen_threshold_tracking
      always @(posedge clk) begin
        if (!resetn) begin
          threshold_timer[i] <= 0;
          ch_threshold_viol[i] <= 0;
        end else if (state == RUNNING) begin

          // If above threshold, increment timer until window, then set violation flag
          if (abs_sample_concat[((i+1)*15)-1 -: 15] > thresh_val_latched) begin
            if (threshold_timer[i] < window_latched) begin
              threshold_timer[i] <= threshold_timer[i] + 1;
            end else begin
              ch_threshold_viol[i] <= 1;
            end
          // Otherwise, decrement timer if it's above 0 to allow for recovery below threshold
          end else begin
            if (threshold_timer[i] > 0) begin
              threshold_timer[i] <= threshold_timer[i] - 1;
            end
          end

        end
      end
    end
  endgenerate

endmodule

          

          
          

