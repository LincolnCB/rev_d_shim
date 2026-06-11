`timescale 1ns / 1ps

module axi_clock_timing_snoop # (
  parameter integer SOURCE_CLK_FREQ_HZ = 30_000_000,
  parameter integer DIVCLK_DIVIDE_DEFAULT = 1,
  parameter integer CLKFBOUT_MULT_DEFAULT = 33,
  parameter integer CLKFBOUT_FRAC_MULT_DEFAULT = 250,
  parameter integer CLKOUT0_DIVIDE_DEFAULT = 49,
  parameter integer CLKOUT0_FRAC_DIVIDE_DEFAULT = 875,
  parameter integer MAX_CLK_OUT_FREQ_HZ = 100_000_000
)
(
  input  wire        aclk,
  input  wire        aresetn,

  output wire [31:0] source_clk_freq_hz, // Source clock frequency in Hz (for reference)
  output reg  [31:0] clk_out_freq_hz,    // Calculated clock out frequency in Hz
  output reg  [ 3:0] reconfig_state,
  output reg         reconf_in_prog,     // Reconfiguration in progress
  output reg         div_by_zero,        // Indicates if a divide-by-zero error occurred during calculations
  output reg         freq_too_high,      // Indicates if the calculated frequency exceeds the maximum

  // AXI4-Lite subordinate port
  input  wire [10:0]  s_axi_awaddr,  // AXI4-Lite subordinate: Write address
  input  wire         s_axi_awvalid, // AXI4-Lite subordinate: Write address valid
  output wire         s_axi_awready, // AXI4-Lite subordinate: Write address ready
  input  wire [31:0]  s_axi_wdata,   // AXI4-Lite subordinate: Write data
  input  wire [ 3:0]  s_axi_wstrb,   // AXI4-Lite subordinate: Write strobe
  input  wire         s_axi_wvalid,  // AXI4-Lite subordinate: Write data valid
  output wire         s_axi_wready,  // AXI4-Lite subordinate: Write data ready
  output wire [ 1:0]  s_axi_bresp,   // AXI4-Lite subordinate: Write response
  output wire         s_axi_bvalid,  // AXI4-Lite subordinate: Write response valid
  input  wire         s_axi_bready,  // AXI4-Lite subordinate: Write response ready
  input  wire [10:0]  s_axi_araddr,  // AXI4-Lite subordinate: Read address
  input  wire         s_axi_arvalid, // AXI4-Lite subordinate: Read address valid
  output wire         s_axi_arready, // AXI4-Lite subordinate: Read address ready
  output wire [31:0]  s_axi_rdata,   // AXI4-Lite subordinate: Read data
  output wire [ 1:0]  s_axi_rresp,   // AXI4-Lite subordinate: Read data response
  output wire         s_axi_rvalid,  // AXI4-Lite subordinate: Read data valid
  input  wire         s_axi_rready,  // AXI4-Lite subordinate: Read data ready

  // AXI4-Lite manager port -- directly forwarded to/from subordinate port for snooping
  output wire [10:0]  m_axi_awaddr,  // AXI4-Lite manager: Write address
  output wire         m_axi_awvalid, // AXI4-Lite manager: Write address valid
  input  wire         m_axi_awready, // AXI4-Lite manager: Write address ready
  output wire [31:0]  m_axi_wdata,   // AXI4-Lite manager: Write data
  output wire [ 3:0]  m_axi_wstrb,   // AXI4-Lite manager: Write strobe
  output wire         m_axi_wvalid,  // AXI4-Lite manager: Write data valid
  input  wire         m_axi_wready,  // AXI4-Lite manager: Write data ready
  input  wire [ 1:0]  m_axi_bresp,   // AXI4-Lite manager: Write response
  input  wire         m_axi_bvalid,  // AXI4-Lite manager: Write response valid
  output wire         m_axi_bready,  // AXI4-Lite manager: Write response ready
  output wire [10:0]  m_axi_araddr,  // AXI4-Lite manager: Read address
  output wire         m_axi_arvalid, // AXI4-Lite manager: Read address valid
  input  wire         m_axi_arready, // AXI4-Lite manager: Read address ready
  input  wire [31:0]  m_axi_rdata,   // AXI4-Lite manager: Read data
  input  wire [ 1:0]  m_axi_rresp,   // AXI4-Lite manager: Read data response
  input  wire         m_axi_rvalid,  // AXI4-Lite manager: Read data valid
  output wire         m_axi_rready   // AXI4-Lite manager: Read data ready
);

// https://docs.amd.com/r/en-US/pg065-clk-wiz/Port-Descriptions

// Software Reset Register
//  To activate software reset, the value 0x0000_000A must be written to the register.
//  Any other access, read or write, has undefined results.
localparam [10:0] ADDR_SRR = 11'h000;
wire sreset = (s_axi_awvalid && s_axi_wvalid && (s_axi_awaddr == ADDR_SRR) && (s_axi_wdata == 32'h0000_000A));
wire resetn = ~sreset & aresetn; // Combine software reset with external reset, active low


// Clock Configuration Register 0
//   Bit[7:0] = DIVCLK_DIVIDE
//     Eight bit divide value applied to all output clocks.
//   Bit[15:8] = CLKFBOUT_MULT
//     Integer part of multiplier value i.e. For 8.125, this value is 8 = 0x8.
//   Bit[25:16] = CLKFBOUT_FRAC Multiply (3)
//     Fractional part of multiplier value i.e. For 8.125, this value is 125 = 0x7D.
//     The value of CLKFBOUT fractional divide can be from 0 to 875 representing the fractional multiplied by 1000.
localparam [10:0] ADDR_CLK_CFG_0 = 11'h200;
reg  [31:0] axi_clk_cfg_0_reg;
wire [7:0] divclk_divide = axi_clk_cfg_0_reg[7:0];
wire [7:0] clkfbout_mult = axi_clk_cfg_0_reg[15:8];
wire [9:0] clkfbout_frac_mult = axi_clk_cfg_0_reg[25:16];

// Clock Configuration Register 2
//   Bit[7:0] = CLKOUT0_DIVIDE
//     Integer part of clkout0 divide value
//     For example, for 2.250, this value is 2 = 0x2
//   Bit[17:8] = CLKOUT0_FRAC Divide (3)
//     Fractional part of clkout0 divide value
//     For example, for 2.250, this value is 250 = 0xFA
localparam [10:0] ADDR_CLK_CFG_2 = 11'h208;
reg  [31:0] axi_clk_cfg_2_reg;
wire [7:0] clkout0_divide = axi_clk_cfg_2_reg[7:0];
wire [9:0] clkout0_frac_divide = axi_clk_cfg_2_reg[17:8];


// Clock Configuration Register 23 (Load Register)
//   Bit[0] = LOAD / SEN
//     Loads Clock Configuration Register values to the internal register used for dynamic reconfiguration and initiates reconfiguration state machine.
//     This bit should be asserted when the required settings are already written into Clock Configuration Registers.
//     This bit retains to 0, when the dynamic reconfiguration is done and the clock is locked.
//   Bit[1] = SADDR
//     When written 0, default configuration done in the Clocking Wizard GUI is loaded for dynamic reconfiguration.
//     When written 1, setting provided in the Clock Configuration Registers are used for dynamic reconfiguration.
localparam [10:0] ADDR_CLK_CFG_23 = 11'h25C;
wire clk_cfg_load = s_axi_awvalid && s_axi_wvalid && (s_axi_awaddr == ADDR_CLK_CFG_23) && s_axi_wdata[0];
wire clk_cfg_use_default = clk_cfg_load && !s_axi_wdata[1];

// Max fractional value for multiplier/divider (represents 0.875)
localparam [ 9:0] FRAC_MAX = 10'd875;
localparam [63:0] CLK_NUM =
  (64'd1 * SOURCE_CLK_FREQ_HZ) *
  ((64'd1 * CLKFBOUT_MULT_DEFAULT) * 64'd1000 + (64'd1 * CLKFBOUT_FRAC_MULT_DEFAULT));
localparam [63:0] CLK_DENOM =
  (64'd1 * DIVCLK_DIVIDE_DEFAULT) *
  ((64'd1 * CLKOUT0_DIVIDE_DEFAULT) * 64'd1000 + (64'd1 * CLKOUT0_FRAC_DIVIDE_DEFAULT));
localparam [31:0] CLK_FREQ_HZ_DEFAULT = CLK_NUM / CLK_DENOM;
// Make sure the defaults are valid values
initial begin
  // First check basic validity
  if (SOURCE_CLK_FREQ_HZ <= 10_000_000) begin
    $error("SOURCE_CLK_FREQ_HZ must be greater than 10 MHz to ensure timing calculations are meaningful");
  end
  if (DIVCLK_DIVIDE_DEFAULT <= 0) begin
    $error("DIVCLK_DIVIDE_DEFAULT must be greater than 0");
  end
  if (CLKFBOUT_MULT_DEFAULT <= 0) begin
    $error("CLKFBOUT_MULT_DEFAULT must be greater than 0");
  end
  if (CLKFBOUT_FRAC_MULT_DEFAULT > FRAC_MAX) begin
    $error("CLKFBOUT_FRAC_MULT_DEFAULT must be less than or equal to %d", FRAC_MAX);
  end
  if (CLKOUT0_DIVIDE_DEFAULT <= 0) begin
    $error("CLKOUT0_DIVIDE_DEFAULT must be greater than 0");
  end
  if (CLKOUT0_FRAC_DIVIDE_DEFAULT > FRAC_MAX) begin
    $error("CLKOUT0_FRAC_DIVIDE_DEFAULT must be less than or equal to %d", FRAC_MAX);
  end
  // Then check width-based caps
  if (SOURCE_CLK_FREQ_HZ >= (32'hFFFFFFFF)) begin
    $error("SOURCE_CLK_FREQ_HZ must be less than %d to fit within 31 bits for calculations", (1 << 31));
  end
  if (DIVCLK_DIVIDE_DEFAULT >= (1 << 8)) begin
    $error("DIVCLK_DIVIDE_DEFAULT must fit within 8 bits");
  end
  if (CLKFBOUT_MULT_DEFAULT >= (1 << 8)) begin
    $error("CLKFBOUT_MULT_DEFAULT must fit within 8 bits");
  end
  if (CLKFBOUT_FRAC_MULT_DEFAULT >= (1 << 10)) begin
    $error("CLKFBOUT_FRAC_MULT_DEFAULT must fit within 10 bits");
  end
  if (CLKOUT0_DIVIDE_DEFAULT >= (1 << 8)) begin
    $error("CLKOUT0_DIVIDE_DEFAULT must fit within 8 bits");
  end
  if (CLKOUT0_FRAC_DIVIDE_DEFAULT >= (1 << 10)) begin
    $error("CLKOUT0_FRAC_DIVIDE_DEFAULT must fit within 10 bits");
  end
  if (MAX_CLK_OUT_FREQ_HZ >= (32'hFFFFFFFF)) begin
    $error("MAX_CLK_OUT_FREQ_HZ must be less than %d to fit within 31 bits for calculations", (1 << 31));
  end
  if (CLK_FREQ_HZ_DEFAULT >= MAX_CLK_OUT_FREQ_HZ) begin
    $error("Default calculated SPI clock frequency (%d) must be less than MAX_CLK_OUT_FREQ_HZ (%d Hz)", CLK_FREQ_HZ_DEFAULT, MAX_CLK_OUT_FREQ_HZ);
  end
  if (CLK_FREQ_HZ_DEFAULT <= 1_000_000) begin
    $error("Default calculated SPI clock frequency (%d) should be greater than 1 MHz", CLK_FREQ_HZ_DEFAULT);
  end
end

// Local default values with explicit widths
localparam [31:0] SOURCE_CLK_FREQ_HZ_W = SOURCE_CLK_FREQ_HZ;
localparam [ 7:0] DIVCLK_DIVIDE_DEFAULT_W = DIVCLK_DIVIDE_DEFAULT[7:0];
localparam [ 7:0] CLKFBOUT_MULT_DEFAULT_W = CLKFBOUT_MULT_DEFAULT[7:0];
localparam [ 9:0] CLKFBOUT_FRAC_MULT_DEFAULT_W = CLKFBOUT_FRAC_MULT_DEFAULT[9:0];
localparam [ 7:0] CLKOUT0_DIVIDE_DEFAULT_W = CLKOUT0_DIVIDE_DEFAULT[7:0];
localparam [ 9:0] CLKOUT0_FRAC_DIVIDE_DEFAULT_W = CLKOUT0_FRAC_DIVIDE_DEFAULT[9:0];
localparam [31:0] CLK_FREQ_HZ_DEFAULT_W = CLK_FREQ_HZ_DEFAULT;

// Output the source clock frequency as a reference
assign source_clk_freq_hz = SOURCE_CLK_FREQ_HZ_W;

// Reconfiguration state machine states
localparam [3:0] S_IDLE           = 3'd0;
localparam [3:0] S_STARTING       = 3'd1;
localparam [3:0] S_GET_FULL_DIV_1 = 3'd2;
localparam [3:0] S_GET_FULL_DIV_2 = 3'd3;
localparam [3:0] S_GET_FULL_MULT  = 3'd4;
localparam [3:0] S_CALC_MULT      = 3'd5;
localparam [3:0] S_CALC_DIV       = 3'd6;

// Latched values
reg [ 7:0] divclk_divide_latched;
reg [ 7:0] clkfbout_mult_latched;
reg [ 9:0] clkfbout_frac_mult_latched;
reg [ 7:0] clkout0_divide_latched;
reg [ 9:0] clkout0_frac_divide_latched;
// Calculation registers
reg  [25:0] clk_div;  // Multiply by 1000 to preserve fractional precision as integer
// Multiplier and divider inputs and outputs
reg         mult_start;
reg  [17:0] mult_multiplicand;
reg  [31:0] mult_multiplier;
wire [49:0] mult_result;
wire        mult_done;

reg         div_start;
reg  [49:0] div_dividend;
reg  [25:0] div_divisor;
wire [49:0] div_quotient;
wire [25:0] div_remainder;
wire        div_by_zero_calc;
wire        div_done;

shift_add_mult #(
  .MULTIPLICAND_WIDTH(18),
  .MULTIPLIER_WIDTH(32)
) u_shift_add_mult (
  .clk(aclk),
  .resetn(resetn),
  .start(mult_start),
  .multiplicand(mult_multiplicand),
  .multiplier(mult_multiplier),
  .result(mult_result),
  .done(mult_done)
);

shift_sub_div #(
  .DIVIDEND_WIDTH(50),
  .DIVISOR_WIDTH(26)
) u_shift_sub_div (
  .clk(aclk),
  .resetn(resetn),
  .start(div_start),
  .dividend(div_dividend),
  .divisor(div_divisor),
  .quotient(div_quotient),
  .remainder(div_remainder),
  .div_by_zero(div_by_zero_calc),
  .done(div_done)
);


// Capture AXI writes to relevant registers
always @(posedge aclk) begin
  if (!resetn) begin
    axi_clk_cfg_0_reg <= {CLKFBOUT_FRAC_MULT_DEFAULT_W, CLKFBOUT_MULT_DEFAULT_W, DIVCLK_DIVIDE_DEFAULT_W};
    axi_clk_cfg_2_reg <= {CLKOUT0_FRAC_DIVIDE_DEFAULT_W, CLKOUT0_DIVIDE_DEFAULT_W};
  end else begin
    // Clock Configuration Register 0
    if (s_axi_awvalid && s_axi_wvalid && (s_axi_awaddr == ADDR_CLK_CFG_0)) begin
      axi_clk_cfg_0_reg <= s_axi_wdata;
    end
    // Clock Configuration Register 2
    if (s_axi_awvalid && s_axi_wvalid && (s_axi_awaddr == ADDR_CLK_CFG_2)) begin
      axi_clk_cfg_2_reg <= s_axi_wdata;
    end
  end
end

// Calculation state machine
always @(posedge aclk) begin
  if (!resetn) begin
    reconf_in_prog <= 1'b0;
    reconfig_state <= S_IDLE;
    clk_out_freq_hz <= CLK_FREQ_HZ_DEFAULT_W;
    divclk_divide_latched <= DIVCLK_DIVIDE_DEFAULT_W;
    clkfbout_mult_latched <= CLKFBOUT_MULT_DEFAULT_W;
    clkfbout_frac_mult_latched <= CLKFBOUT_FRAC_MULT_DEFAULT_W;
    clkout0_divide_latched <= CLKOUT0_DIVIDE_DEFAULT_W;
    clkout0_frac_divide_latched <= CLKOUT0_FRAC_DIVIDE_DEFAULT_W;
    clk_div <= 26'd0;
    mult_start <= 1'b0;
    div_start <= 1'b0;
    div_by_zero <= 1'b0;
    freq_too_high <= 1'b0;
  end else begin
    case (reconfig_state)

      // Start reconfiguration when load bit is set.
      //   Latch the values to either the current or default values depending on the clk_cfg_use_default bit.
      S_IDLE: begin
        if (clk_cfg_load) begin
          if (clk_cfg_use_default) begin
            // Load default values from parameters
            divclk_divide_latched <= DIVCLK_DIVIDE_DEFAULT_W;
            clkfbout_mult_latched <= CLKFBOUT_MULT_DEFAULT_W;
            clkfbout_frac_mult_latched <= CLKFBOUT_FRAC_MULT_DEFAULT_W;
            clkout0_divide_latched <= CLKOUT0_DIVIDE_DEFAULT_W;
            clkout0_frac_divide_latched <= CLKOUT0_FRAC_DIVIDE_DEFAULT_W;
          end else begin
            // Load latched values from registers
            divclk_divide_latched <= divclk_divide;
            clkfbout_mult_latched <= clkfbout_mult;
            clkfbout_frac_mult_latched <= clkfbout_frac_mult;
            clkout0_divide_latched <= clkout0_divide;
            clkout0_frac_divide_latched <= clkout0_frac_divide;
          end
          reconf_in_prog <= 1'b1;
          div_by_zero <= 1'b0;
          freq_too_high <= 1'b0;
          reconfig_state <= S_STARTING;
        end
      end

      S_STARTING: begin
        // result = clkout0_divide * 1000
        mult_multiplicand <= clkout0_divide_latched;
        mult_multiplier <= 1000;
        mult_start <= 1'b1;
        reconfig_state <= S_GET_FULL_DIV_1;
      end

      S_GET_FULL_DIV_1: begin
        if (mult_done) begin
          // result = divclk_divide * (clkout0_divide * 1000 + clkout0_frac_divide)
          mult_multiplicand <= divclk_divide_latched;
          mult_multiplier <= mult_result + clkout0_frac_divide_latched;
          mult_start <= 1'b1;
          reconfig_state <= S_GET_FULL_DIV_2;
        end else if (mult_start) mult_start <= 1'b0;
      end

      S_GET_FULL_DIV_2: begin
        if (mult_done) begin
          // clk_div = divclk_divide * (clkout0_divide * 1000 + clkout0_frac_divide)
          clk_div <= mult_result;
          // result = clkfbout_mult * 1000
          mult_multiplicand <= clkfbout_mult_latched;
          mult_multiplier <= 1000;
          mult_start <= 1'b1;
          reconfig_state <= S_GET_FULL_MULT;
        end else if (mult_start) mult_start <= 1'b0;
      end

      S_GET_FULL_MULT: begin
        if (mult_done) begin
          // result = source_clk_freq_hz * (clkfbout_mult * 1000 + clkfbout_frac_mult)
          mult_multiplicand <= mult_result + clkfbout_frac_mult_latched;
          mult_multiplier <= source_clk_freq_hz;
          mult_start <= 1'b1;
          reconfig_state <= S_CALC_MULT;
        end else if (mult_start) mult_start <= 1'b0;
      end

      S_CALC_MULT: begin
        if (mult_done) begin
          // result = source_clk_freq_hz * (clkfbout_mult * 1000 + clkfbout_frac_mult)
          //          / (divclk_divide * (clkout0_divide * 1000 + clkout0_frac_divide))
          div_dividend <= mult_result;
          div_divisor <= clk_div;
          div_start <= 1'b1;
          reconfig_state <= S_CALC_DIV;
        end else if (mult_start) mult_start <= 1'b0;
      end

      S_CALC_DIV: begin
        if (div_done) begin
          div_by_zero <= div_by_zero_calc;
          if (!div_by_zero_calc) begin
            if (div_quotient > MAX_CLK_OUT_FREQ_HZ) begin
              freq_too_high <= 1'b1;
              clk_out_freq_hz <= MAX_CLK_OUT_FREQ_HZ;
            end else begin
              clk_out_freq_hz <= div_quotient[31:0];
            end
          end else begin
            clk_out_freq_hz <= CLK_FREQ_HZ_DEFAULT_W;
          end
          reconf_in_prog <= 1'b0;
          reconfig_state <= S_IDLE;
        end else if (div_start) div_start <= 1'b0;
      end

      default: begin
        reconf_in_prog <= 1'b0;
        reconfig_state <= S_IDLE;
        clk_out_freq_hz <= CLK_FREQ_HZ_DEFAULT_W;
        divclk_divide_latched <= DIVCLK_DIVIDE_DEFAULT_W;
        clkfbout_mult_latched <= CLKFBOUT_MULT_DEFAULT_W;
        clkfbout_frac_mult_latched <= CLKFBOUT_FRAC_MULT_DEFAULT_W;
        clkout0_divide_latched <= CLKOUT0_DIVIDE_DEFAULT_W;
        clkout0_frac_divide_latched <= CLKOUT0_FRAC_DIVIDE_DEFAULT_W;
        clk_div <= 26'd0;
        div_by_zero <= 1'b0;
        freq_too_high <= 1'b0;
      end
    endcase
  end
end


// Assign all AXI4-Lite manager signals to directly forward
//  from subordinate port for snooping and vice versa
assign m_axi_awaddr = s_axi_awaddr;
assign m_axi_awvalid = s_axi_awvalid;
assign s_axi_awready = m_axi_awready;
assign m_axi_wdata = s_axi_wdata;
assign m_axi_wstrb = s_axi_wstrb;
assign m_axi_wvalid = s_axi_wvalid;
assign s_axi_wready = m_axi_wready;
assign s_axi_bresp = m_axi_bresp;
assign s_axi_bvalid = m_axi_bvalid;
assign m_axi_bready = s_axi_bready;
assign m_axi_araddr = s_axi_araddr;
assign m_axi_arvalid = s_axi_arvalid;
assign s_axi_arready = m_axi_arready;
assign m_axi_rdata = s_axi_rdata;
assign m_axi_rresp = s_axi_rresp;
assign m_axi_rvalid = s_axi_rvalid;
assign s_axi_rready = m_axi_rready;


endmodule
