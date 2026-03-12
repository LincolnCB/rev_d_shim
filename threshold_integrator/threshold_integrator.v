`timescale 1 ns / 1 ps

module threshold_integrator (
  // Inputs
  input   wire         clk              ,
  input   wire         resetn           ,
  input   wire         enable           ,
  input   wire [ 31:0] window           ,
  input   wire [ 14:0] threshold_average,
  input   wire         sample_core_done ,
  input   wire [119:0] abs_sample_concat,

  // Outputs
  output  reg         err_overflow ,
  output  reg         err_underflow,
  output  reg         over_thresh  ,
  output  reg         setup_done
);

  //// Internal signals
  reg [43:0] max_value           ;
  reg [ 4:0] chunk_width         ;
  reg [24:0] chunk_size          ;
  reg [24:0] inflow_chunk_timer  ;
  reg [31:0] outflow_timer       ;
  reg [ 3:0] fifo_in_queue_count ;
  reg [35:0] fifo_din            ;
  wire       fifo_full           ;
  wire       wr_en               ;
  reg [ 3:0] fifo_out_queue_count;
  wire[35:0] fifo_dout           ;
  wire       fifo_empty          ;
  wire       rd_en               ;
  wire[ 7:0] channel_over_thresh ;
  reg [ 2:0] state               ;
  wire[14:0] inflow_value              [0:7];
  reg [35:0] inflow_chunk_sum          [0:7];
  reg [35:0] queued_fifo_in_chunk_sum  [0:7];
  reg [35:0] queued_fifo_out_chunk_sum [0:7];
  reg [14:0] outflow_value             [0:7];
  reg [15:0] outflow_value_plus_one    [0:7];
  reg [19:0] outflow_remainder         [0:7];
  reg signed [16:0] sum_delta          [0:7];
  reg signed [44:0] total_sum          [0:7];

  // Registers for shift-add multiplication
  reg [43:0] window_reg;
  reg [14:0] threshold_average_shift;
  reg [ 4:0] max_value_mult_cnt;

  //// State encoding
  localparam  IDLE          = 3'd0,
              SETUP         = 3'd1,
              WAIT          = 3'd2,
              RUNNING       = 3'd3,
              OUT_OF_BOUNDS = 3'd4,
              ERROR         = 3'd5;

  //// FIFO for rolling integration memory
  fifo_sync #(
    .DATA_WIDTH(36),
    .ADDR_WIDTH(10)
  )
  rolling_sum_mem (
    .clk(clk),
    .resetn(resetn),
    .wr_data(fifo_din),
    .wr_en(wr_en),
    .full(fifo_full),
    .rd_data(fifo_dout),
    .rd_en(rd_en),
    .empty(fifo_empty)
  );

  //// FIFO I/O
  always @* begin
    if (fifo_in_queue_count != 0) begin
      fifo_din = queued_fifo_in_chunk_sum[fifo_in_queue_count - 1];
    end else begin
      fifo_din = 36'b0;
    end
  end
  assign wr_en = (fifo_in_queue_count != 0);
  assign rd_en = (fifo_out_queue_count != 0);

  //// Global logic
  always @(posedge clk) begin : global_logic
    // Reset logic
    if (!resetn) begin : reset_logic
      // Zero all individual signals
      max_value <= 0;
      chunk_width <= 0;
      chunk_size <= 0;
      inflow_chunk_timer <= 0;
      outflow_timer <= 0;
      fifo_in_queue_count <= 0;
      fifo_out_queue_count <= 0;

      // Zero all output signals
      over_thresh <= 0;
      err_overflow <= 0;
      err_underflow <= 0;
      setup_done <= 0;

      // Shift-add multiplication registers
      window_reg <= 0;
      threshold_average_shift <= 0;
      max_value_mult_cnt <= 0;

      // Set initial state
      state <= IDLE;
    end else begin : state_logic
      case (state)

        // IDLE state, waiting for enable signal
        IDLE: begin
          if (enable) begin
            // Calculate chunk_size: (MSB index of window) - 10
            if (window < 32'd2048) begin // window is too small, disallowed
              over_thresh <= 1;
              state <= OUT_OF_BOUNDS;
            end else begin
              // Calculate chunk width based on the window size
              if (window[31]) begin
                chunk_width <= 21;
              end else if (window[30]) begin
                chunk_width <= 20;
              end else if (window[29]) begin
                chunk_width <= 19;
              end else if (window[28]) begin
                chunk_width <= 18;
              end else if (window[27]) begin
                chunk_width <= 17;
              end else if (window[26]) begin
                chunk_width <= 16;
              end else if (window[25]) begin
                chunk_width <= 15;
              end else if (window[24]) begin
                chunk_width <= 14;
              end else if (window[23]) begin
                chunk_width <= 13;
              end else if (window[22]) begin
                chunk_width <= 12;
              end else if (window[21]) begin
                chunk_width <= 11;
              end else if (window[20]) begin
                chunk_width <= 10;
              end else if (window[19]) begin
                chunk_width <= 9;
              end else if (window[18]) begin
                chunk_width <= 8;
              end else if (window[17]) begin
                chunk_width <= 7;
              end else if (window[16]) begin
                chunk_width <= 6;
              end else if (window[15]) begin
                chunk_width <= 5;
              end else if (window[14]) begin
                chunk_width <= 4;
              end else if (window[13]) begin
                chunk_width <= 3;
              end else if (window[12]) begin
                chunk_width <= 2;
              end else begin // window[11] must be 1 here if we already know window >= 2048
                chunk_width <= 1;
              end

              // Prepare for shift-add multiplication in SETUP
              window_reg <= window >> 4; // Only adding every 16th clock cycle
              threshold_average_shift <= threshold_average;
              max_value_mult_cnt <= 0;

              state <= SETUP;
            end
          end
        end // IDLE

        // SETUP state, calculating max_value and chunk size
        SETUP: begin
          // Shift-add multiplication for max_value = threshold_average * window
          if (|threshold_average_shift) begin
            if (threshold_average_shift[0]) begin
              max_value <= max_value + (window_reg << max_value_mult_cnt);
            end
            threshold_average_shift <= threshold_average_shift >> 1;
            max_value_mult_cnt <= max_value_mult_cnt + 1;
          end else begin // Finished shift-add multiplication, set chunk size from chunk width and go to WAIT for sample core
            chunk_size <= (25'b1 << chunk_width) - 1;
            state <= WAIT;
          end
        end // SETUP

        // WAIT state, waiting for sample core (DAC/ADC) to finish setting up
        WAIT: begin
          if (sample_core_done) begin
            // Initialize timers
            inflow_chunk_timer <= chunk_size << 4;
            outflow_timer <= window - 1;
            setup_done <= 1;
            state <= RUNNING;
          end
        end // WAIT

        // RUNNING state, main logic
        RUNNING: begin : running_state

          // Error logic
          if (fifo_full & wr_en) begin
            err_overflow <= 1;
            state <= ERROR;
          end
          if (fifo_empty & rd_en) begin
            err_underflow <= 1;
            state <= ERROR;
          end
          
          // Over threshold logic
          if (|channel_over_thresh) begin
            over_thresh <= 1;
            state <= OUT_OF_BOUNDS;
          end

          // Inflow timer
          if (inflow_chunk_timer == 0) begin // Reset inflow chunk timer
            inflow_chunk_timer <= chunk_size << 4;
            fifo_in_queue_count <= 8;
          end else begin
            inflow_chunk_timer <= inflow_chunk_timer - 1;
          end // Inflow timer

          // Inflow FIFO counter
          if (fifo_in_queue_count != 0) begin
            // FIFO push is done in FIFO I/O always block above
            fifo_in_queue_count <= fifo_in_queue_count - 1;
          end // Inflow FIFO counter

          // Outflow timer
          if (outflow_timer != 0) begin
            if (outflow_timer == 16) begin // Initiate FIFO popping to queue
              fifo_out_queue_count <= 8;
            end
            outflow_timer <= outflow_timer - 1;
          end else begin
            outflow_timer <= chunk_size << 4;
          end // Outflow timer

          // Outflow FIFO counter
          if (fifo_out_queue_count != 0) begin
            // FIFO pop is done in the per-channel logic below
            fifo_out_queue_count <= fifo_out_queue_count - 1;
          end

        end // RUNNING

        OUT_OF_BOUNDS: begin : out_of_bounds_state
          // Stop everything until reset
        end // OUT_OF_BOUNDS

        ERROR: begin : error_state
          // Stop everything until reset
        end // ERROR

      endcase // state
    end
  end // Global logic

  //// Per-channel logic
  genvar i;
  generate // Per-channel logic generate
    for (i = 0; i < 8; i = i + 1) begin : channel_loop
      assign channel_over_thresh[i] = (total_sum[i] > max_value) ? 1 : 0;
      assign inflow_value[i] = abs_sample_concat[((i+1)*15)-1 -: 15];

      always @(posedge clk) begin : channel_logic
        if (!resetn) begin : channel_reset
          // Zero all per-channel signals
          inflow_chunk_sum[i] = 0;
          queued_fifo_in_chunk_sum[i] = 0;
          queued_fifo_out_chunk_sum[i] = 0;
          outflow_value[i] = 0;
          outflow_value_plus_one[i] = 0;
          outflow_remainder[i] = 0;
          total_sum[i] = 0;
          sum_delta[i] = 0;
        end else if (state == RUNNING) begin : channel_running

          //// Inflow logic
          // Only sample every 16th clock cycle
          if (inflow_chunk_timer[3:0] == 0) begin
            // Inflow addition logic
            if (inflow_chunk_timer != 0) begin // Add to chunk sum
              inflow_chunk_sum[i] <= inflow_chunk_sum[i] + inflow_value[i];
            end else begin // Add to chunk sum and move into FIFO queue. Reset chunk sum
              queued_fifo_in_chunk_sum[i] <= inflow_chunk_sum[i] + inflow_value[i];
              inflow_chunk_sum[i] <= 0;
            end
          end

          //// Outflow logic
          // Outflow FIFO logic
          if (fifo_out_queue_count == i + 1) begin
            queued_fifo_out_chunk_sum[i] <= fifo_dout;
          end // Outflow FIFO logic

          // Move queued chunks in to outflow value and remainder
          if (outflow_timer == 0) begin
            outflow_value[i] <= queued_fifo_out_chunk_sum[i] >> chunk_width;
            outflow_value_plus_one[i] <= (queued_fifo_out_chunk_sum[i] >> chunk_width) + 1;
            // Calculate remainder: Note that chunk_size is (25'b1 << chunk_width) - 1, so it can be used to mask out the remainder
            outflow_remainder[i] <= queued_fifo_out_chunk_sum[i] & chunk_size;
          end // FIFO out queue

          //// Sum logic
          // Pipeline the delta to the running total
          if (outflow_timer[3:0] == 0) begin // Only add every 16th clock cycle
            sum_delta[i] <= ((outflow_timer >> 4) < outflow_remainder[i])
                    ? $signed({2'b00, inflow_value[i]}) - $signed({1'b0, outflow_value_plus_one[i]})
                    : $signed({2'b00, inflow_value[i]}) - $signed({2'b00, outflow_value[i]});
          end else if (outflow_timer[3:0] == 4'd15) begin // Add the sum delta to the total sum one cyle after it's calculated
            total_sum[i] <= total_sum[i] + sum_delta[i];
          end // Sum logic
        end // RUNNING
      end // channel_running
    end // channel_loop
  endgenerate // Per-channel logic generate
endmodule
