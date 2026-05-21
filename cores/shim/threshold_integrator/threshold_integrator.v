`timescale 1 ns / 1 ps

module threshold_integrator (
  // Inputs
  input   wire         clk              ,
  input   wire         resetn           ,
  input   wire         enable           ,
  input   wire [ 31:0] window           ,
  input   wire [ 14:0] thresh_val       ,
  input   wire         sample_core_setup,
  input   wire [119:0] abs_sample_concat,

  // Outputs
  output  reg         err_overflow ,
  output  reg         err_underflow,
  output  reg         over_thresh  ,
  output  reg         setup_done
);

  //// Internal signals
  reg  [43:0] max_value           ;
  reg  [ 4:0] chunk_width         ;
  reg  [ 4:0] chunk_width_counter ;
  reg  [19:0] window_width_reg    ;
  reg  [24:0] chunk_mask          ;
  reg  [24:0] timer_init_value    ;
  reg  [24:0] inflow_chunk_timer  ;
  reg  [31:0] outflow_timer       ;
  reg  [ 3:0] fifo_in_queue_count ;
  wire [35:0] fifo_din            ;
  wire        fifo_full           ;
  wire        wr_en               ;
  reg  [ 3:0] fifo_out_queue_count;
  wire [35:0] fifo_dout           ;
  wire        fifo_empty          ;
  wire        rd_en               ;
  wire [ 7:0] channel_over_thresh ;
  reg  [ 2:0] state               ;
  wire [14:0] inflow_value              [0:7];
  reg  [35:0] inflow_chunk_sum          [0:7];
  reg  [35:0] queued_fifo_in_chunk_sum  [0:7];
  reg  [35:0] queued_fifo_out_chunk_sum [0:7];
  reg  [14:0] outflow_value             [0:7];
  reg  [15:0] outflow_value_plus_one    [0:7];
  reg  [24:0] outflow_remainder         [0:7];
  reg  signed [16:0] sum_delta          [0:7];
  reg  signed [44:0] total_sum          [0:7];
  wire        calc_sum_delta;
  reg         add_sum_delta;

  // Registers for shift-add multiplication
  reg  [43:0] window_mult_reg;
  reg  [14:0] thresh_val_shift;
  reg  [ 4:0] max_value_mult_cnt;

  //// State encoding
  localparam  IDLE                 = 3'd0,
              CALC_CHUNK_WIDTH     = 3'd1,
              CALC_MAX_VALUE       = 3'd2,
              WAIT_FOR_SAMPLE_CORE = 3'd3,
              RUNNING              = 3'd4,
              OUT_OF_BOUNDS        = 3'd5,
              ERROR                = 3'd6;

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
    .almost_full(),

    .fifo_count(),

    .rd_data(fifo_dout),
    .rd_en(rd_en),
    .empty(fifo_empty),
    .almost_empty()
  );

  //// FIFO I/O
  assign fifo_din = (fifo_in_queue_count != 0) ? queued_fifo_in_chunk_sum[fifo_in_queue_count - 1] : 36'b0;

  assign wr_en = (fifo_in_queue_count != 0);
  assign rd_en = (fifo_out_queue_count != 0);
  assign calc_sum_delta = (outflow_timer[3:0] == 0);

  //// Global logic
  always @(posedge clk) begin : global_logic
    // Reset logic
    if (!resetn) begin : reset_logic
      // Zero all individual signals
      max_value <= 0;
      chunk_width <= 0;
      chunk_mask <= 0;
      timer_init_value <= 0;
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
      chunk_width_counter <= 0;
      window_width_reg <= 0;
      window_mult_reg <= 0;
      thresh_val_shift <= 0;
      max_value_mult_cnt <= 0;

      // Set initial state
      state <= IDLE;
    end else begin : state_logic
      case (state)

        // IDLE state, waiting for enable signal
        IDLE: begin
          if (enable) begin
            // Calculate chunk size: (MSB index of window) - 10
            if (window < 32'd2048) begin // window is too small, disallowed
              over_thresh <= 1;
              state <= OUT_OF_BOUNDS;
            end else begin
              // Prepare for chunk width shift calculation in CALC_CHUNK_WIDTH
              window_width_reg <= window[31:12]; // Start on bit 12 since it must at least be 2048 (2^11)
              chunk_width_counter <= 1; // Start at 1 since the minimum chunk width is 1 (window of 2048 means chunk width of 1)
              
              // Prepare for shift-add multiplication in CALC_MAX_VALUE
              window_mult_reg <= {12'b0, window >> 4}; // Only adding every 16th clock cycle
              thresh_val_shift <= thresh_val;
              max_value_mult_cnt <= 0;

              state <= CALC_CHUNK_WIDTH;
            end
          end
        end // IDLE

        // CALC_CHUNK_WIDTH state, calculating chunk width by finding the MSB index of window_width_reg
        CALC_CHUNK_WIDTH: begin
          if (window_width_reg != 0) begin // Shift right until window_width_reg is 0, counting the number of shifts to find the MSB index
            window_width_reg <= window_width_reg >> 1;
            chunk_width_counter <= chunk_width_counter + 1;
          end else begin // Finished finding MSB index, calculate chunk width and move to CALC_MAX_VALUE
            chunk_width <= chunk_width_counter;
            state <= CALC_MAX_VALUE;
          end
        end // CALC_CHUNK_WIDTH

        // CALC_MAX_VALUE state, calculating max_value and chunk size
        CALC_MAX_VALUE: begin
          // Shift-add multiplication for max_value = thresh_val * window
          if (|thresh_val_shift) begin
            if (thresh_val_shift[0]) begin
              max_value <= max_value + (window_mult_reg << max_value_mult_cnt);
            end
            thresh_val_shift <= thresh_val_shift >> 1;
            max_value_mult_cnt <= max_value_mult_cnt + 1;
          end else begin // Finished shift-add multiplication, set chunk mask / timer init from chunk width and go to WAIT_FOR_SAMPLE_CORE for sample core
            chunk_mask <= (25'b1 << chunk_width) - 1;
            timer_init_value <= (25'b1 << (chunk_width + 4)) - 1;
            state <= WAIT_FOR_SAMPLE_CORE;
          end
        end // CALC_MAX_VALUE

        // WAIT_FOR_SAMPLE_CORE state, waiting for sample core (DAC/ADC) to finish setting up
        WAIT_FOR_SAMPLE_CORE: begin
          if (sample_core_setup) begin
            // Initialize timers
            inflow_chunk_timer <= timer_init_value;
            outflow_timer <= window - 1;
            setup_done <= 1;
            state <= RUNNING;
          end
        end // WAIT_FOR_SAMPLE_CORE

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
            inflow_chunk_timer <= timer_init_value;
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
            outflow_timer <= {7'b0, timer_init_value};
          end // Outflow timer

          // Outflow FIFO counter
          if (fifo_out_queue_count != 0) begin
            // FIFO pop is done in the per-channel logic below
            fifo_out_queue_count <= fifo_out_queue_count - 1;
          end

          // Sume delta calculation timing logic
          if (calc_sum_delta) begin
            add_sum_delta <= 1;
          end else begin
            add_sum_delta <= 0;
          end

        end // RUNNING

        OUT_OF_BOUNDS: begin : out_of_bounds_state
          // Stop everything until reset
        end // OUT_OF_BOUNDS

        ERROR: begin : error_state
          // Stop everything until reset
        end // ERROR

        default: begin : default_state_error
          state <= ERROR;
        end

      endcase // state
    end
  end // Global logic

  //// Per-channel logic
  genvar i;
  generate // Per-channel logic generate
    for (i = 0; i < 8; i = i + 1) begin : channel_loop
      assign channel_over_thresh[i] = (total_sum[i] > $signed({1'b0, max_value})) ? 1 : 0;
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
              inflow_chunk_sum[i] <= inflow_chunk_sum[i] + {21'b0, inflow_value[i]};
            end else begin // Add to chunk sum and move into FIFO queue. Reset chunk sum
              queued_fifo_in_chunk_sum[i] <= inflow_chunk_sum[i];
              inflow_chunk_sum[i] <= {21'b0, inflow_value[i]};
            end
          end

          //// Outflow logic
          // Outflow FIFO logic
          if (fifo_out_queue_count == i + 1) begin
            queued_fifo_out_chunk_sum[i] <= fifo_dout;
          end // Outflow FIFO logic

          // Move queued chunks in to outflow value and remainder
          if (outflow_timer == 0) begin
            outflow_value[i] <= queued_fifo_out_chunk_sum[i][{1'b0, chunk_width} +: 15];
            outflow_value_plus_one[i] <= {1'b0, queued_fifo_out_chunk_sum[i][{1'b0, chunk_width} +: 15]} + 1;
            // Calculate remainder by masking out by chunk_mask
            outflow_remainder[i] <= queued_fifo_out_chunk_sum[i][24:0] & chunk_mask;
          end // FIFO out queue

          //// Sum logic
          // Pipeline the delta to the running total
          if (calc_sum_delta) begin // Only add every 16th clock cycle
            sum_delta[i] <= ((outflow_timer >> 4) < outflow_remainder[i])
                    ? $signed({2'b00, inflow_value[i]}) - $signed({1'b0, outflow_value_plus_one[i]})
                    : $signed({2'b00, inflow_value[i]}) - $signed({2'b00, outflow_value[i]});
          end else if (add_sum_delta) begin // Add the sum delta to the total sum one cyle after it's calculated
            total_sum[i] <= total_sum[i] + {{28{sum_delta[i][16]}}, sum_delta[i]};
          end // Sum logic
        end // RUNNING
      end // channel_running
    end // channel_loop
  endgenerate // Per-channel logic generate
endmodule
