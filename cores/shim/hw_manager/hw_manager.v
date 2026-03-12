`timescale 1 ns / 1 ps

module hw_manager #(
  // Delays for the various timeouts, default clock frequency is 100 MHz
  parameter integer SHUTDOWN_FORCE_DELAY = 10000000, // 100 ms : Delay after releasing "n_shutdown_force" before pulsing "shutdown_rst"
  parameter integer SHUTDOWN_RESET_PULSE = 10000,    // 100 us : Pulse width for "shutdown_rst"
  parameter integer SHUTDOWN_RESET_DELAY = 10000000, // 100 ms : Delay after pulsing "shutdown_rst" before starting the system
  parameter integer SPI_RESET_WAIT = 100000000,      //   1  s : Delay after starting the SPI clock before checking if the SPI subsystem is initialized to off
  parameter integer SPI_START_WAIT = 300000000       //   3  s : Delay after starting the SPI clock before halting if the SPI subsystem doesn't start
)
(
  input   wire          clk,     // System clock
  input   wire          aresetn, // Active low reset

  // Inputs
  input   wire          ctrl_en,        // Control board enable (turn the system on)
  input   wire          pow_en,         // Power stage enable (turn on power stage)
  input   wire          spi_off,        // SPI system powered off
  input   wire          calc_n_cs_done, // DAC/ADC n_cs timing calculation done
  input   wire          ext_en,         // External enable (deadman shutdown)
  // Pre-start configuration values
  input   wire          lock_viol,            // Configuration lock violation
  input   wire          ctrl_en_oob,          // Control board enable register out of bounds
  input   wire          pow_en_oob,           // Power stage enable register out of bounds
  input   wire          cmd_buf_reset_oob,    // Command buffer reset out of bounds
  input   wire          data_buf_reset_oob,   // Data buffer reset out of bounds
  input   wire          integ_thresh_avg_oob, // Integrator threshold average out of bounds
  input   wire          integ_window_oob,     // Integrator window out of bounds
  input   wire          integ_en_oob,         // Integrator enable register out of bounds
  input   wire          boot_test_skip_oob,   // Boot test skip out of bounds
  input   wire          debug_oob,            // Debug reg out of bounds
  input   wire          mosi_sck_pol_oob,     // MOSI SCK polarity out of bounds
  input   wire          miso_sck_pol_oob,     // MISO SCK polarity out of bounds
  input   wire          dac_cal_init_oob,     // DAC calibration initial value out of bounds
  // Shutdown sense (per board)
  input   wire  [ 7:0]  shutdown_sense_sts, // Shutdown sense
  // Integrator (per board)
  input   wire  [ 7:0]  over_thresh,      // DAC over threshold
  input   wire  [ 7:0]  thresh_underflow, // DAC threshold core FIFO underflow
  input   wire  [ 7:0]  thresh_overflow,  // DAC threshold core FIFO overflow
  // Trigger buffer and commands
  input   wire          bad_trig_cmd,            // Bad trigger command
  input   wire          trig_cmd_buf_overflow,   // Trigger command buffer overflow
  input   wire          trig_data_buf_underflow, // Trigger data buffer underflow
  input   wire          trig_data_buf_overflow,  // Trigger data buffer overflow
  // DAC buffers and commands (per board)
  input   wire  [ 7:0]  dac_boot_fail,          // DAC boot failure
  input   wire  [ 7:0]  bad_dac_cmd,            // Bad DAC command
  input   wire  [ 7:0]  dac_cal_oob,            // DAC calibration out of bounds
  input   wire  [ 7:0]  dac_val_oob,            // DAC value out of bounds
  input   wire  [ 7:0]  dac_cmd_buf_underflow,  // DAC command buffer underflow
  input   wire  [ 7:0]  dac_cmd_buf_overflow,   // DAC command buffer overflow
  input   wire  [ 7:0]  dac_data_buf_underflow, // DAC data buffer underflow
  input   wire  [ 7:0]  dac_data_buf_overflow,  // DAC data buffer overflow
  input   wire  [ 7:0]  unexp_dac_trig,         // Unexpected DAC trigger
  input   wire  [ 7:0]  ldac_misalign,          // LDAC misalignment
  input   wire  [ 7:0]  dac_delay_too_short,    // DAC delay too short
  // ADC buffers and commands (per board)
  input   wire  [ 7:0]  adc_boot_fail,          // ADC boot failure
  input   wire  [ 7:0]  bad_adc_cmd,            // Bad ADC command
  input   wire  [ 7:0]  adc_cmd_buf_underflow,  // ADC command buffer underflow
  input   wire  [ 7:0]  adc_cmd_buf_overflow,   // ADC command buffer overflow
  input   wire  [ 7:0]  adc_data_buf_underflow, // ADC data buffer underflow
  input   wire  [ 7:0]  adc_data_buf_overflow,  // ADC data buffer overflow
  input   wire  [ 7:0]  unexp_adc_trig,         // Unexpected ADC trigger
  input   wire  [ 7:0]  adc_delay_too_short,    // ADC delay too short

  // Outputs
  output  reg           unlock_cfg,        // Lock configuration
  output  reg           spi_clk_gate,      // SPI clock power (negated)
  output  reg           spi_en,            // SPI subsystem enable
  output  reg           shutdown_sense_en, // Shutdown sense enable
  output  reg           block_bufs,        // Block PL side of command/data buffers
  output  reg           n_shutdown_force,  // Shutdown force (negated)
  output  reg           shutdown_rst,      // Shutdown reset
  output  wire  [31:0]  status_word,       // Status - Status word
  output  reg           ps_interrupt       // Interrupt signal
);

  // Validate parameters
  initial begin
    if (SHUTDOWN_FORCE_DELAY < 0)
      $error("Invalid value for SHUTDOWN_FORCE_DELAY parameter: %d. Must be non-negative.", SHUTDOWN_FORCE_DELAY);
    if (SHUTDOWN_RESET_PULSE < 0)
      $error("Invalid value for SHUTDOWN_RESET_PULSE parameter: %d. Must be non-negative.", SHUTDOWN_RESET_PULSE);
    if (SHUTDOWN_RESET_DELAY < 0)
      $error("Invalid value for SHUTDOWN_RESET_DELAY parameter: %d. Must be non-negative.", SHUTDOWN_RESET_DELAY);
    if (SPI_RESET_WAIT < 0)
      $error("Invalid value for SPI_RESET_WAIT parameter: %d. Must be non-negative.", SPI_RESET_WAIT);
    if (SPI_START_WAIT < 0)
      $error("Invalid value for SPI_START_WAIT parameter: %d. Must be non-negative.", SPI_START_WAIT);
  end

  // Internal signals
  reg [ 3:0] state;       // State machine state
  reg [31:0] timer;       // Timer for various timeouts
  reg [ 2:0] board_num;   // Status - Board number (if applicable)
  reg [24:0] status_code; // Status - Status code

  // Concatenated status word
  assign status_word = {board_num, status_code, state};

  // State encoding
  localparam  S_IDLE              = 4'd1,
              S_CONFIRM_SPI_RST   = 4'd2,
              S_POWER_ON_CRTL_BRD = 4'd3,
              S_CONFIRM_SPI_START = 4'd4,
              S_WAIT_FOR_POW_EN   = 4'd5,
              S_POWER_ON_AMP_BRD  = 4'd6,
              S_AMP_POWER_WAIT    = 4'd7,
              S_RUNNING           = 4'd8,
              S_HALTING           = 4'd9,
              S_HALTED            = 4'd10;

  //// Status codes
  // Basic system
  localparam  STS_EMPTY                   = 25'h0000,
              STS_OK                      = 25'h0001,
              STS_PS_SHUTDOWN             = 25'h0002;
  // SPI subsystem
  localparam  STS_SPI_RESET_TIMEOUT       = 25'h0100,
              STS_SPI_START_TIMEOUT       = 25'h0101;
  // Pre-start configuration values
  localparam  STS_LOCK_VIOL               = 25'h0200,
              STS_CTRL_EN_OOB             = 25'h0201,
              STS_POW_EN_OOB              = 25'h0202,
              STS_CMD_BUF_RESET_OOB       = 25'h0203,
              STS_DATA_BUF_RESET_OOB      = 25'h0204,
              STS_INTEG_THRESH_AVG_OOB    = 25'h0205,
              STS_INTEG_WINDOW_OOB        = 25'h0206,
              STS_INTEG_EN_OOB            = 25'h0207,
              STS_BOOT_TEST_SKIP_OOB      = 25'h0208,
              STS_DEBUG_OOB               = 25'h0209,
              STS_MOSI_SCK_POL_OOB        = 25'h020A,
              STS_MISO_SCK_POL_OOB        = 25'h020B,
              STS_DAC_CAL_INIT_OOB        = 25'h020C;
  // Shutdown sense
  localparam  STS_SHUTDOWN_SENSE          = 25'h0300,
              STS_EXT_SHUTDOWN            = 25'h0301;
  // Integrator threshold core
  localparam  STS_OVER_THRESH             = 25'h0400,
              STS_THRESH_UNDERFLOW        = 25'h0401,
              STS_THRESH_OVERFLOW         = 25'h0402;
  // Trigger buffer and commands
  localparam  STS_BAD_TRIG_CMD            = 25'h0500,
              STS_TRIG_CMD_BUF_OVERFLOW   = 25'h0501,
              STS_TRIG_DATA_BUF_UNDERFLOW = 25'h0502,
              STS_TRIG_DATA_BUF_OVERFLOW  = 25'h0503;
  // DAC buffers and commands
  localparam  STS_DAC_BOOT_FAIL           = 25'h0600,
              STS_BAD_DAC_CMD             = 25'h0601,
              STS_DAC_CAL_OOB             = 25'h0602,
              STS_DAC_VAL_OOB             = 25'h0603,
              STS_DAC_CMD_BUF_UNDERFLOW   = 25'h0604,
              STS_DAC_CMD_BUF_OVERFLOW    = 25'h0605,
              STS_DAC_DATA_BUF_UNDERFLOW  = 25'h0606,
              STS_DAC_DATA_BUF_OVERFLOW   = 25'h0607,
              STS_UNEXP_DAC_TRIG          = 25'h0608,
              STS_LDAC_MISALIGN           = 25'h0609,
              STS_DAC_DELAY_TOO_SHORT     = 25'h060A;
  // ADC buffers and commands
  localparam  STS_ADC_BOOT_FAIL           = 25'h0700,
              STS_BAD_ADC_CMD             = 25'h0701,
              STS_ADC_CMD_BUF_UNDERFLOW   = 25'h0702,
              STS_ADC_CMD_BUF_OVERFLOW    = 25'h0703,
              STS_ADC_DATA_BUF_UNDERFLOW  = 25'h0704,
              STS_ADC_DATA_BUF_OVERFLOW   = 25'h0705,
              STS_UNEXP_ADC_TRIG          = 25'h0706,
              STS_ADC_DELAY_TOO_SHORT     = 25'h0707;

  // Main state machine
  always @(posedge clk) begin
    if (!aresetn) begin
      state <= S_IDLE;
      timer <= 0;
      n_shutdown_force <= 0;
      shutdown_rst <= 0;
      shutdown_sense_en <= 0;
      unlock_cfg <= 1;
      spi_clk_gate <= 0;
      spi_en <= 0;
      block_bufs <= 1;
      status_code <= STS_OK;
      board_num <= 0;
      ps_interrupt <= 0;
    end else begin

      // State machine
      case (state)

        // Idle state, hardware shut down, waiting for system enable to go high
        // When enabled, lock the config registers and confirm the SPI subsystem is initialized
        S_IDLE: begin : idle_state
          if (ctrl_en) begin
            // Check for the external enable before proceeding
            if (!ext_en) begin
              state <= S_HALTING;
              status_code <= STS_EXT_SHUTDOWN;
            // Check for out of bounds configuration values
            end else if (
              ctrl_en_oob
              || pow_en_oob
              || cmd_buf_reset_oob
              || data_buf_reset_oob
              || integ_thresh_avg_oob
              || integ_window_oob
              || integ_en_oob
              || boot_test_skip_oob
              || debug_oob
              || mosi_sck_pol_oob
              || miso_sck_pol_oob
              || dac_cal_init_oob
            ) begin
              if (ctrl_en_oob) begin // Control board enable out of bounds
                state <= S_HALTING;
                status_code <= STS_CTRL_EN_OOB;
              end else if (pow_en_oob) begin // Power stage enable out of bounds
                state <= S_HALTING;
                status_code <= STS_POW_EN_OOB;
              end else if (cmd_buf_reset_oob) begin // Command buffer reset out of bounds
                state <= S_HALTING;
                status_code <= STS_CMD_BUF_RESET_OOB;
              end else if (data_buf_reset_oob) begin // Data buffer reset out of bounds
                state <= S_HALTING;
                status_code <= STS_DATA_BUF_RESET_OOB;
              end else if (integ_thresh_avg_oob) begin // Integrator threshold average out of bounds
                state <= S_HALTING;
                status_code <= STS_INTEG_THRESH_AVG_OOB;
              end else if (integ_window_oob) begin // Integrator window out of bounds
                state <= S_HALTING;
                status_code <= STS_INTEG_WINDOW_OOB;
              end else if (integ_en_oob) begin // Integrator enable out of bounds
                state <= S_HALTING;
                status_code <= STS_INTEG_EN_OOB;
              end else if (boot_test_skip_oob) begin // Boot test skip out of bounds
                state <= S_HALTING;
                status_code <= STS_BOOT_TEST_SKIP_OOB;
              end else if (debug_oob) begin // Debug reg out of bounds
                state <= S_HALTING;
                status_code <= STS_DEBUG_OOB;
              end else if (mosi_sck_pol_oob) begin // MOSI SCK polarity out of bounds
                state <= S_HALTING;
                status_code <= STS_MOSI_SCK_POL_OOB;
              end else if (miso_sck_pol_oob) begin // MISO SCK polarity out of bounds
                state <= S_HALTING;
                status_code <= STS_MISO_SCK_POL_OOB;
              end else if (dac_cal_init_oob) begin // DAC calibration initial value out of bounds
                state <= S_HALTING;
                status_code <= STS_DAC_CAL_INIT_OOB;
              end
            end else begin // Lock the cfg registers and start the SPI clock to confirm the SPI subsystem is initialized
              state <= S_CONFIRM_SPI_RST;
              timer <= 0;
              unlock_cfg <= 0;
            end
          end // if (ctrl_en)
        end // S_IDLE

        // Confirm the SPI subsystem is reset to its initial state, off
        // If the SPI subsystem is not reset to off, halt the system
        S_CONFIRM_SPI_RST: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (timer >= 10 && spi_off && calc_n_cs_done) begin
            state <= S_POWER_ON_CRTL_BRD;
            timer <= 0;
            n_shutdown_force <= 1;
          end else if (timer >= SPI_RESET_WAIT) begin
            state <= S_HALTING;
            status_code <= STS_SPI_RESET_TIMEOUT;
          end else begin
            timer <= timer + 1;
          end // if (spi_off)
        end // S_CONFIRM_SPI_RST

        // Wait for a delay between releasing the shutdown force and starting the SPI control subsystem
        // This allows the DAC/ADC etc. to power up and be ready for SPI communication
        // Signals to halt:
        //   timer
        //   n_shutdown_force
        S_POWER_ON_CRTL_BRD: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (timer >= SHUTDOWN_FORCE_DELAY) begin
            state <= S_CONFIRM_SPI_START;
            timer <= 0;
            spi_en <= 1;
            spi_clk_gate <= 1;
          end else begin
            timer <= timer + 1;
          end // if (timer >= SHUTDOWN_FORCE_DELAY)
        end // S_POWER_ON_CRTL_BRD

        // Wait for the SPI subsystem to start before running the system
        // If the SPI subsystem doesn't start in time, halt the system
        S_CONFIRM_SPI_START: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (!spi_off) begin
            state <= S_WAIT_FOR_POW_EN;
            timer <= 0;
            shutdown_rst <= 1;
          end else if (|dac_boot_fail || |adc_boot_fail || timer >= SPI_START_WAIT) begin
            // If the SPI subsystem is still off after the wait, or a channel failed to boot, halt the system
            state <= S_HALTING;
            timer <= 0;
            // Set the status code based on the error condition
            if (|dac_boot_fail) begin
              status_code <= STS_DAC_BOOT_FAIL;
              board_num <= extract_board_num(dac_boot_fail);
            end else if (|adc_boot_fail) begin
              status_code <= STS_ADC_BOOT_FAIL;
              board_num <= extract_board_num(adc_boot_fail);
            end else begin
              status_code <= STS_SPI_START_TIMEOUT;
            end
          end else begin
            timer <= timer + 1;
          end // if (!spi_off)
        end // S_CONFIRM_SPI_START

        // Wait for the power stage enable to go high before powering on the amplifier boards
        S_WAIT_FOR_POW_EN: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (pow_en) begin
            state <= S_POWER_ON_AMP_BRD;
            timer <= 0;
          end // if (pow_en)
        end // S_WAIT_FOR_POW_EN

        // Pulse the shutdown reset for a short time to power on the power stage
        S_POWER_ON_AMP_BRD: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en || !pow_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (timer >= SHUTDOWN_RESET_PULSE) begin
            state <= S_AMP_POWER_WAIT;
            timer <= 0;
            shutdown_rst <= 0;
          end else begin
            timer <= timer + 1;
          end // if (timer >= SHUTDOWN_RESET_PULSE)
        end // S_POWER_ON_AMP_BRD

        // Wait for a delay after pulsing the shutdown reset before starting the system control
        //   (unblocking command/data buffers)
        S_AMP_POWER_WAIT: begin
          if (!ext_en) begin
            state <= S_HALTING;
            status_code <= STS_EXT_SHUTDOWN;
          end else if (!ctrl_en || !pow_en) begin
            state <= S_HALTING;
            status_code <= STS_PS_SHUTDOWN;
          end else if (timer >= SHUTDOWN_RESET_DELAY) begin
            state <= S_RUNNING;
            timer <= 0;
            shutdown_sense_en <= 1;
            block_bufs <= 0;
            ps_interrupt <= 1;
          end else begin
            timer <= timer + 1;
          end // if (timer >= SHUTDOWN_RESET_DELAY)
        end // S_AMP_POWER_WAIT

        // Main running state, check for various error conditions or shutdowns
        S_RUNNING: begin
          // Reset the interrupt before doing anything else
          if (ps_interrupt) begin
            ps_interrupt <= 0;
          end else if ( // Check for various error conditions or shutdowns
              // Manual enables
              !ctrl_en || !pow_en
              // Pre-start configuration values
              || lock_viol
              || mosi_sck_pol_oob
              || miso_sck_pol_oob
              // Shutdown sense
              || |shutdown_sense_sts
              || !ext_en
              // Integrator threshold core
              || |over_thresh
              || |thresh_underflow
              || |thresh_overflow
              // Trigger buffer and commands
              || bad_trig_cmd
              || trig_cmd_buf_overflow
              || trig_data_buf_underflow
              || trig_data_buf_overflow
              // DAC buffers and commands
              || |bad_dac_cmd
              || |dac_cal_oob
              || |dac_val_oob
              || |dac_cmd_buf_underflow
              || |dac_cmd_buf_overflow
              || |dac_data_buf_underflow
              || |dac_data_buf_overflow
              || |unexp_dac_trig
              || |ldac_misalign
              || |dac_delay_too_short
              // ADC buffers and commands
              || |bad_adc_cmd
              || |adc_cmd_buf_underflow
              || |adc_cmd_buf_overflow
              || |adc_data_buf_underflow
              || |adc_data_buf_overflow
              || |unexp_adc_trig
              || |adc_delay_too_short
          ) begin
            //// Set the status code based on the error condition
            // Basic system
            if (!ctrl_en || !pow_en) status_code <= STS_PS_SHUTDOWN;
            // Pre-start configuration values
            else if (lock_viol) status_code <= STS_LOCK_VIOL;
            else if (mosi_sck_pol_oob) status_code <= STS_MOSI_SCK_POL_OOB;
            else if (miso_sck_pol_oob) status_code <= STS_MISO_SCK_POL_OOB;
            // Shutdown sense
            else if (|shutdown_sense_sts) begin
              status_code <= STS_SHUTDOWN_SENSE;
              board_num <= extract_board_num(shutdown_sense_sts);
            end
            else if (!ext_en) status_code <= STS_EXT_SHUTDOWN;
            // Integrator threshold core
            else if (|over_thresh) begin
              status_code <= STS_OVER_THRESH;
              board_num <= extract_board_num(over_thresh);
            end
            else if (|thresh_underflow) begin
              status_code <= STS_THRESH_UNDERFLOW;
              board_num <= extract_board_num(thresh_underflow);
            end
            else if (|thresh_overflow) begin
              status_code <= STS_THRESH_OVERFLOW;
              board_num <= extract_board_num(thresh_overflow);
            end
            // Trigger buffer and commands
            else if (bad_trig_cmd) status_code <= STS_BAD_TRIG_CMD;
            else if (trig_cmd_buf_overflow) status_code <= STS_TRIG_CMD_BUF_OVERFLOW;
            else if (trig_data_buf_underflow) status_code <= STS_TRIG_DATA_BUF_UNDERFLOW;
            else if (trig_data_buf_overflow) status_code <= STS_TRIG_DATA_BUF_OVERFLOW;
            // DAC buffers and commands
            else if (|bad_dac_cmd) begin
              status_code <= STS_BAD_DAC_CMD;
              board_num <= extract_board_num(bad_dac_cmd);
            end
            else if (|dac_cal_oob) begin
              status_code <= STS_DAC_CAL_OOB;
              board_num <= extract_board_num(dac_cal_oob);
            end
            else if (|dac_val_oob) begin
              status_code <= STS_DAC_VAL_OOB;
              board_num <= extract_board_num(dac_val_oob);
            end
            else if (|dac_cmd_buf_underflow) begin
              status_code <= STS_DAC_CMD_BUF_UNDERFLOW;
              board_num <= extract_board_num(dac_cmd_buf_underflow);
            end
            else if (|dac_cmd_buf_overflow) begin
              status_code <= STS_DAC_CMD_BUF_OVERFLOW;
              board_num <= extract_board_num(dac_cmd_buf_overflow);
            end
            else if (|dac_data_buf_underflow) begin
              status_code <= STS_DAC_DATA_BUF_UNDERFLOW;
              board_num <= extract_board_num(dac_data_buf_underflow);
            end
            else if (|dac_data_buf_overflow) begin
              status_code <= STS_DAC_DATA_BUF_OVERFLOW;
              board_num <= extract_board_num(dac_data_buf_overflow);
            end
            else if (|unexp_dac_trig) begin
              status_code <= STS_UNEXP_DAC_TRIG;
              board_num <= extract_board_num(unexp_dac_trig);
            end
            else if (|ldac_misalign) begin
              status_code <= STS_LDAC_MISALIGN;
              board_num <= extract_board_num(ldac_misalign);
            end
            else if (|dac_delay_too_short) begin
              status_code <= STS_DAC_DELAY_TOO_SHORT;
              board_num <= extract_board_num(dac_delay_too_short);
            end
            // ADC buffers and commands
            else if (|bad_adc_cmd) begin
              status_code <= STS_BAD_ADC_CMD;
              board_num <= extract_board_num(bad_adc_cmd);
            end
            else if (|adc_cmd_buf_underflow) begin
              status_code <= STS_ADC_CMD_BUF_UNDERFLOW;
              board_num <= extract_board_num(adc_cmd_buf_underflow);
            end
            else if (|adc_cmd_buf_overflow) begin
              status_code <= STS_ADC_CMD_BUF_OVERFLOW;
              board_num <= extract_board_num(adc_cmd_buf_overflow);
            end
            else if (|adc_data_buf_underflow) begin
              status_code <= STS_ADC_DATA_BUF_UNDERFLOW;
              board_num <= extract_board_num(adc_data_buf_underflow);
            end
            else if (|adc_data_buf_overflow) begin
              status_code <= STS_ADC_DATA_BUF_OVERFLOW;
              board_num <= extract_board_num(adc_data_buf_overflow);
            end
            else if (|unexp_adc_trig) begin
              status_code <= STS_UNEXP_ADC_TRIG;
              board_num <= extract_board_num(unexp_adc_trig);
            end
            else if (|adc_delay_too_short) begin
              status_code <= STS_ADC_DELAY_TOO_SHORT;
              board_num <= extract_board_num(adc_delay_too_short);
            end
            // Set the status code and halt the system
            state <= S_HALTING;
          end // Error/halt state check
        end // S_RUNNING

        // Go to halt the system, set all signals to the initial state and assert the interrupt
        S_HALTING: begin
          state <= S_HALTED; // Proceed to halted state
          timer <= 0;
          n_shutdown_force <= 0;
          shutdown_rst <= 0;
          shutdown_sense_en <= 0;
          unlock_cfg <= 1;
          spi_clk_gate <= 0;
          spi_en <= 0;
          block_bufs <= 1;
          ps_interrupt <= 1;
        end // S_HALTING

        // Wait in the halted state until the BOTH system enables go low
        S_HALTED: begin
          // Reset the interrupt if needed
          if (ps_interrupt) begin
            ps_interrupt <= 0;
          end // if (ps_interrupt)
          // If the BOTH system enables go low, go to S_IDLE and clear the status code
          if (!ctrl_en && !pow_en) begin 
            state <= S_IDLE;
            status_code <= STS_OK;
            board_num <= 0;
            unlock_cfg <= 1;
          end
        end // S_HALTED

        default: begin
          state <= S_HALTED; // Default to halted state
          timer <= 0;
          n_shutdown_force <= 0;
          shutdown_rst <= 0;
          shutdown_sense_en <= 0;
          unlock_cfg <= 1;
          spi_clk_gate <= 0;
          spi_en <= 0;
          block_bufs <= 1;
          status_code <= STS_EMPTY;
          board_num <= 0;
          ps_interrupt <= 1;
        end // default

      endcase // case (state)
    end // if (rst) else
  end // always @(posedge clk)

  // Function to extract the board number from an 8-bit signal
  function [2:0] extract_board_num;
    input [7:0] signal;
    begin
      case (1'b1)
        signal[0]: extract_board_num = 3'd0;
        signal[1]: extract_board_num = 3'd1;
        signal[2]: extract_board_num = 3'd2;
        signal[3]: extract_board_num = 3'd3;
        signal[4]: extract_board_num = 3'd4;
        signal[5]: extract_board_num = 3'd5;
        signal[6]: extract_board_num = 3'd6;
        signal[7]: extract_board_num = 3'd7;
        default: extract_board_num = 3'd0;
      endcase
    end
  endfunction

endmodule
