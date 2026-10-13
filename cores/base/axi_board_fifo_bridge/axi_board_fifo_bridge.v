`timescale 1 ns / 1 ps

// Per-board PIO register bridge: one AXI4-Lite subordinate fronting one board's DAC and ADC
// command/data FIFOs. One address bit (SEL_BIT) picks the DAC half (0) or ADC half (1), so a
// board occupies a single window instead of two separate subordinates -- that lets all eight
// boards hang off one SmartConnect. This fuses two axi_fifo_bridge instances: a write pushes a
// command FIFO and a read pops a data FIFO, routed by SEL_BIT to the DAC or ADC pair. The DAC
// command write and the ADC data read are the DMA-owned ports (gated by *_mode_block: accept
// and discard when the DMA owns them); the DAC data read and ADC command write are always PIO.
// The bus is always-ready (never hangs) and never faults -- a wrong-mode or full/empty access
// is accepted and discarded, raising the matching sticky flag. Each sticky flag clears on its
// own FIFO's reset.
module axi_board_fifo_bridge #(
  parameter integer AXI_ADDR_WIDTH = 13,
  parameter integer AXI_DATA_WIDTH = 32,
  parameter integer SEL_BIT        = 12  // address bit: 0 = DAC half, 1 = ADC half
)(
  input  wire                        aclk,
  input  wire                        aresetn,          // AXI handshake reset
  input  wire                        dac_cmd_resetn,   // per-FIFO resets (clear sticky flags)
  input  wire                        dac_data_resetn,
  input  wire                        adc_cmd_resetn,
  input  wire                        adc_data_resetn,

  // AXI4-Lite subordinate interface
  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,
  input  wire                        s_axi_awvalid,
  output wire                        s_axi_awready,
  input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,
  input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,
  input  wire                        s_axi_wvalid,
  output wire                        s_axi_wready,
  output reg  [1:0]                  s_axi_bresp,
  output reg                         s_axi_bvalid,
  input  wire                        s_axi_bready,
  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,
  input  wire                        s_axi_arvalid,
  output wire                        s_axi_arready,
  output reg  [AXI_DATA_WIDTH-1:0]   s_axi_rdata,
  output reg  [1:0]                  s_axi_rresp,
  output reg                         s_axi_rvalid,
  input  wire                        s_axi_rready,

  // DAC command FIFO (write side) + DAC data FIFO (read side)
  output wire [AXI_DATA_WIDTH-1:0]   dac_cmd_wr_data,
  output wire                        dac_cmd_wr_en,
  input  wire                        dac_cmd_full,
  input  wire [AXI_DATA_WIDTH-1:0]   dac_data_rd_data,
  output wire                        dac_data_rd_en,
  input  wire                        dac_data_empty,

  // ADC command FIFO (write side) + ADC data FIFO (read side)
  output wire [AXI_DATA_WIDTH-1:0]   adc_cmd_wr_data,
  output wire                        adc_cmd_wr_en,
  input  wire                        adc_cmd_full,
  input  wire [AXI_DATA_WIDTH-1:0]   adc_data_rd_data,
  output wire                        adc_data_rd_en,
  input  wire                        adc_data_empty,

  // DMA-ownership gating (DAC command write, ADC data read)
  input  wire                        dac_wr_mode_block,
  input  wire                        adc_rd_mode_block,
  output wire                        mode_viol,

  // Sticky over/underflow per FIFO
  output reg                         dac_cmd_overflow,
  output reg                         adc_cmd_overflow,
  output reg                         dac_data_underflow,
  output reg                         adc_data_underflow
);

  // Validate parameters
  initial begin
    if (AXI_ADDR_WIDTH <= SEL_BIT)
      $error("AXI_ADDR_WIDTH (%0d) must be greater than SEL_BIT (%0d).", AXI_ADDR_WIDTH, SEL_BIT);
    if (AXI_DATA_WIDTH <= 0 || AXI_DATA_WIDTH % 8 != 0)
      $error("Invalid AXI_DATA_WIDTH %0d. Must be positive and a multiple of 8.", AXI_DATA_WIDTH);
  end

  localparam RESP_OKAY = 2'b00;

  reg dac_wr_mode_viol;
  reg adc_rd_mode_viol;
  assign mode_viol = dac_wr_mode_viol | adc_rd_mode_viol;

  //// Write path -- route by awaddr[SEL_BIT] (0 = DAC command, 1 = ADC command)
  wire wr_is_adc = s_axi_awaddr[SEL_BIT];
  wire try_write = s_axi_awvalid && s_axi_wvalid;
  wire dac_wr_ok = !dac_cmd_full && !dac_wr_mode_block; // DAC command is DMA-owned
  wire adc_wr_ok = !adc_cmd_full;                       // ADC command is always PIO
  assign s_axi_awready  = 1'b1;
  assign s_axi_wready   = 1'b1;
  assign dac_cmd_wr_en  = try_write && !wr_is_adc && dac_wr_ok;
  assign adc_cmd_wr_en  = try_write &&  wr_is_adc && adc_wr_ok;
  assign dac_cmd_wr_data = s_axi_wdata;
  assign adc_cmd_wr_data = s_axi_wdata;

  // Write response: always OKAY, never hang
  always @(posedge aclk) begin
    if (!aresetn) begin
      s_axi_bvalid <= 1'b0;
      s_axi_bresp  <= RESP_OKAY;
    end else if (try_write) begin
      s_axi_bvalid <= 1'b1;
      s_axi_bresp  <= RESP_OKAY;
    end else if (s_axi_bready && s_axi_bvalid) begin
      s_axi_bvalid <= 1'b0;
    end
  end

  // DAC command sticky flags (clear on DAC command FIFO reset)
  always @(posedge aclk) begin
    if (!dac_cmd_resetn) begin
      dac_cmd_overflow <= 1'b0;
      dac_wr_mode_viol <= 1'b0;
    end else if (try_write && !wr_is_adc) begin
      if (dac_wr_mode_block)  dac_wr_mode_viol <= 1'b1; // wrong-mode poke, discarded
      else if (dac_cmd_full)  dac_cmd_overflow <= 1'b1; // genuine overflow, discarded
    end
  end

  // ADC command sticky flag (clear on ADC command FIFO reset)
  always @(posedge aclk) begin
    if (!adc_cmd_resetn)
      adc_cmd_overflow <= 1'b0;
    else if (try_write && wr_is_adc && adc_cmd_full)
      adc_cmd_overflow <= 1'b1;
  end

  //// Read path -- route by araddr[SEL_BIT] (0 = DAC data, 1 = ADC data)
  wire rd_is_adc = s_axi_araddr[SEL_BIT];
  wire try_read  = s_axi_arvalid;
  wire dac_rd_ok = !dac_data_empty;                        // DAC data is always PIO
  wire adc_rd_ok = !adc_data_empty && !adc_rd_mode_block;  // ADC data is DMA-owned
  assign s_axi_arready  = 1'b1;
  assign dac_data_rd_en = try_read && !rd_is_adc && dac_rd_ok;
  assign adc_data_rd_en = try_read &&  rd_is_adc && adc_rd_ok;

  wire any_rd_en = dac_data_rd_en || adc_data_rd_en;

  // Read response: popped data when allowed, else zero; always OKAY, never hang
  always @(posedge aclk) begin
    if (!aresetn) begin
      s_axi_rvalid <= 1'b0;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}};
    end else if (any_rd_en) begin
      s_axi_rvalid <= 1'b1;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= rd_is_adc ? adc_data_rd_data : dac_data_rd_data;
    end else if (try_read) begin
      s_axi_rvalid <= 1'b1;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}}; // empty / wrong-mode read returns zero
    end else if (s_axi_rready && s_axi_rvalid) begin
      s_axi_rvalid <= 1'b0;
    end
  end

  // DAC data sticky flag (clear on DAC data FIFO reset). DAC read is never mode-blocked.
  always @(posedge aclk) begin
    if (!dac_data_resetn)
      dac_data_underflow <= 1'b0;
    else if (try_read && !rd_is_adc && dac_data_empty)
      dac_data_underflow <= 1'b1;
  end

  // ADC data sticky flags (clear on ADC data FIFO reset)
  always @(posedge aclk) begin
    if (!adc_data_resetn) begin
      adc_data_underflow <= 1'b0;
      adc_rd_mode_viol   <= 1'b0;
    end else if (try_read && rd_is_adc) begin
      if (adc_rd_mode_block)    adc_rd_mode_viol   <= 1'b1; // wrong-mode poke, discarded
      else if (adc_data_empty)  adc_data_underflow <= 1'b1; // genuine underflow, discarded
    end
  end

endmodule
