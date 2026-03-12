`timescale 1ns / 1ps

module ad5676_dac_timing_calc (
  input  wire        clk,
  input  wire        resetn,
  
  input  wire [31:0] spi_clk_freq_hz, // SPI clock frequency in Hz
  input  wire        calc,            // Start calculation signal
  
  output reg  [4:0]  n_cs_high_time,  // Calculated n_cs high time in cycles (minus 1)
                                      //   Range: 3 to 31 (corresponds to 4 to 32 cycles, 80ns to 640ns at 50MHz SPI clock)
  output reg  [24:0] delay_too_short_time, // Minimum delay time for DAC update commands in cycles (minus 1)
  output reg         done,            // Calculation complete
  output reg         lock_viol        // Error if frequency changes during calc
);

  ///////////////////////////////////////////////////////////////////////////////
  // Constants
  ///////////////////////////////////////////////////////////////////////////////

  // Update time (datasheet: time between rising edges of n_cs) and minimum n_cs high time for AD5676
  // Use a power of 2 unit for easier calculations -- calling these NiS
  // 2^30 (0x4000_0000) NiS = 10^9 (1000_000_000) NS = 1 second
  // Round up to make sure we don't underestimate
  // This allows the overall calculation to be: f(Hz) * T(nis) / 2^30
  //   which changes a division into a bit-shift right by 30
  // AD5676: 830 ns  update, 30 ns  min n_cs high
  //      -> 892 nis update, 33 nis min n_cs high
  localparam integer T_UPDATE_NiS = 892;
  localparam integer T_MIN_N_CS_HIGH_NiS = 33;

  // SPI command bit width
  localparam [24:0] SPI_CMD_BITS = 25'd24;

  // The minimum delay time for a DAC update command is equal to 8 * (n_cs_high_time + 1 + SPI_CMD_BITS)

  ///////////////////////////////////////////////////////////////////////////////
  // Internal Signals
  ///////////////////////////////////////////////////////////////////////////////

  // State machine
  localparam S_IDLE           = 3'd0;
  localparam S_CALC_UPDATE    = 3'd1;
  localparam S_CALC_MIN_HIGH  = 3'd2;
  localparam S_CALC_RESULT    = 3'd3;
  localparam S_FIND_NCS       = 3'd4;
  localparam S_FIND_DELAY     = 3'd5;
  localparam S_DONE           = 3'd6;
  localparam S_HALTED         = 3'd7; // Used when lock violation occurs to prevent further calculations until reset

  reg [ 2:0] state;
  reg [31:0] spi_clk_freq_hz_latched;
  
  // Intermediate calculation results
  reg [31:0] min_cycles_for_t_update;
  reg [31:0] min_cycles_for_t_min_n_cs_high;
  reg [31:0] final_result;
  
  // Multiplication state machine (shift-add algorithm)
  reg [ 3:0] mult_count;
  reg [15:0] mult_shift;    // Shift a bit through this to control multiplication
  reg [31:0] multiplicand;  // The constant (T_UPDATE_NiS or T_MIN_N_CS_HIGH_NiS)
  reg [31:0] multiplier;    // The frequency value
  reg [63:0] mult_accumulator;
  wire [63:0] mult_result_rounded_up;

  ///////////////////////////////////////////////////////////////////////////////
  // Logic
  ///////////////////////////////////////////////////////////////////////////////

  assign mult_result_rounded_up = mult_accumulator + 64'h3FFFFFFF; // Add 2^30 - 1 for rounding

  always @(posedge clk) begin
    if (!resetn) begin
      state <= S_IDLE;
      done <= 1'b0;
      lock_viol <= 1'b0;
      n_cs_high_time <= 5'd0;
      delay_too_short_time <= 25'd0;
      spi_clk_freq_hz_latched <= 32'd0;
      min_cycles_for_t_update <= 32'd0;
      min_cycles_for_t_min_n_cs_high <= 32'd0;
      final_result <= 32'd0;
      mult_count <= 4'd0;
      mult_shift <= 16'd0;
      multiplicand <= 32'd0;
      multiplier <= 32'd0;
      mult_accumulator <= 64'd0;
    end else begin
      case (state)
        S_IDLE: begin
          done <= 1'b0;
          lock_viol <= 1'b0;
          if (calc) begin
            spi_clk_freq_hz_latched <= spi_clk_freq_hz;
            state <= S_CALC_UPDATE;
            
            // Setup multiplication for T_UPDATE_NiS * spi_clk_freq_hz
            multiplicand <= T_UPDATE_NiS;
            multiplier <= spi_clk_freq_hz;
            mult_count <= 4'd0;
            mult_shift <= 16'd0;
            mult_accumulator <= 64'd0;
          end
        end

        //// ---- Multiply T_UPDATE_NiS * spi_clk_freq_hz using shift-add
        S_CALC_UPDATE: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end else begin
            if (mult_shift < T_UPDATE_NiS) begin
              // Shift-add multiplication: if current bit of multiplicand is 1, add shifted multiplier
              if (multiplicand[mult_count]) begin
                mult_accumulator <= mult_accumulator + ({32'd0, multiplier} << mult_count);
              end
              mult_shift <= 16'd1 << mult_count;
              mult_count <= mult_count + 1;
            end else begin
              // Multiplication complete, add 2^30 - 1 for rounding, then shift right by 30 bits (equivalent to divide by 2^30)
              // Set the min cycles for t_update to be the result minus SPI command bits (24), min of 0
              min_cycles_for_t_update <= (mult_result_rounded_up[61:30]) > {7'd0, SPI_CMD_BITS} ? (mult_result_rounded_up[61:30] - {7'd0, SPI_CMD_BITS}) : 0;
              
              // Setup multiplication for T_MIN_N_CS_HIGH_NiS * spi_clk_freq_hz_latched
              multiplicand <= T_MIN_N_CS_HIGH_NiS;
              multiplier <= spi_clk_freq_hz_latched;
              mult_count <= 4'd0;
              mult_shift <= 16'd0;
              mult_accumulator <= 64'd0;
              
              state <= S_CALC_MIN_HIGH;
            end
          end
        end

        //// ---- Multiply T_MIN_N_CS_HIGH_NiS * spi_clk_freq_hz using shift-add
        S_CALC_MIN_HIGH: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end else begin
            if (mult_shift < T_MIN_N_CS_HIGH_NiS) begin
              // Shift-add multiplication: if current bit of multiplicand is 1, add shifted multiplier
              if (multiplicand[mult_count]) begin
                mult_accumulator <= mult_accumulator + ({32'd0, multiplier} << mult_count);
              end
              mult_shift <= 16'd1 << mult_count;
              mult_count <= mult_count + 1;
            end else begin
              // Multiplication complete, add 2^30 - 1 for rounding, then shift right by 30 bits (equivalent to divide by 2^30)
              // Ensure n_cs high time is at least 4 cycles to do DAC value loading and calibration
              min_cycles_for_t_min_n_cs_high <= (mult_result_rounded_up[61:30] < 4) ? 32'd4 : mult_result_rounded_up[61:30];
              state <= S_CALC_RESULT;
            end
          end
        end

        //// ---- Calculate the final n_cs_high_time based on the two previous results
        S_CALC_RESULT: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end else begin
            // Calculate final result based on the following logic:
            // final_result = max(min_cycles_for_t_update, min_cycles_for_t_min_n_cs_high);
            if (min_cycles_for_t_update < min_cycles_for_t_min_n_cs_high) begin
              final_result <= min_cycles_for_t_min_n_cs_high;
            end else begin
              final_result <= min_cycles_for_t_update;
            end
            state <= S_FIND_NCS;
          end
        end

        //// ---- Finalize the n_cs_high_time output
        S_FIND_NCS: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end else begin
            // Cap n_cs_high_time at 31 (32 + 24 bits at the maximum 50MHz SPI clock is 1120ns, which is >830ns required)
            if (final_result > 31) begin
              n_cs_high_time <= 5'd31;
            end else begin
              n_cs_high_time <= final_result[4:0] - 1; // Subtract 1 to get n_cs_high_time (3 = 4 cycles, 31 = 32 cycles)
            end
            state <= S_FIND_DELAY;
          end
        end

        //// ---- Calculate the min_delay_time output
        S_FIND_DELAY: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end else begin
            // Delay too short time is [8 * (n_cs_high_time + 1 + SPI_CMD_BITS) - 1]
            //   (subtract 1 to convert to "time too short" threshold)
            delay_too_short_time <= (({20'd0, n_cs_high_time} + 25'd1 + SPI_CMD_BITS) << 3) - 25'd1;
            done <= 1'b1;
            state <= S_DONE;
          end
        end

        //// ---- Stay in this state to check if calc goes low or frequency changes
        S_DONE: begin
          // Check for frequency change during calculation
          if (spi_clk_freq_hz != spi_clk_freq_hz_latched) begin
            lock_viol <= 1'b1;
            state <= S_HALTED;
          end else if (!calc) begin
            // calc went low, reset
            state <= S_IDLE;
          end
        end

        //// ---- If lock violation occurs, stay in this state until calc goes low
        S_HALTED: begin
          if (!calc) begin
            state <= S_IDLE;
          end
        end

        default: begin
          state <= S_IDLE;
        end
      endcase
    end
  end

endmodule
