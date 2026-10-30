`timescale 1 ns / 1 ps

// All-board PIO register array: one AXI4-Lite subordinate fronting every board's DAC and ADC
// FIFOs, so the boards share one interconnect port and one handshake instead of a bridge and port
// each. Each board owns a 4 KB page (board = addr[14:12]); inside a page each lane has its own
// word (lane = addr[4:2], aliased across the rest of the page). Lane bit 2 picks DAC/ADC, bits
// 1:0 pick command/debug/data:
//   word 0  dac_cmd    write  (DMA-owned in DMA mode)
//   word 1  dac_debug  read
//   word 2  dac_data   read   (calibration readback)
//   word 4  adc_cmd    write
//   word 5  adc_debug  read
//   word 6  adc_data   read   (DMA-owned in DMA mode)
//   word 7  adc_rate   write: capture read delta (rate_wr_en); read: capture backlog (no pop)
// A write pushes its command FIFO, a read pops its debug/data FIFO. The rate lane is a plain
// register path to the capture rate control, used in both modes. mode_block[b] (the board's
// datapath_mode bit) hands board b's two DMA-owned lanes to the DMA side: a PIO access there is
// accepted and discarded and raises mode_viol[b]. The bus is always ready and always answers
// OKAY -- a full/empty, wrong-mode, unmapped-lane, or absent-board access is discarded (reads
// return zero), raising the matching sticky flag where there is one; the debug lanes have none.
// Each sticky flag clears on its own FIFO's reset. Per-board ports are fixed at eight boards;
// boards at or above NUM_BOARDS are never addressed.
module axi_board_fifo_array #(
  parameter integer NUM_BOARDS     = 8,
  parameter integer AXI_ADDR_WIDTH = 16,
  parameter integer AXI_DATA_WIDTH = 32
)(
  input  wire                          aclk,
  input  wire                          aresetn,          // AXI handshake reset
  input  wire [7:0]                    dac_cmd_resetn,   // per-FIFO resets (clear sticky flags)
  input  wire [7:0]                    dac_data_resetn,
  input  wire [7:0]                    adc_cmd_resetn,
  input  wire [7:0]                    adc_data_resetn,

  // AXI4-Lite subordinate interface
  input  wire [AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
  input  wire                          s_axi_awvalid,
  output wire                          s_axi_awready,
  input  wire [AXI_DATA_WIDTH-1:0]     s_axi_wdata,
  input  wire [AXI_DATA_WIDTH/8-1:0]   s_axi_wstrb,
  input  wire                          s_axi_wvalid,
  output wire                          s_axi_wready,
  output reg  [1:0]                    s_axi_bresp,
  output reg                           s_axi_bvalid,
  input  wire                          s_axi_bready,
  input  wire [AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
  input  wire                          s_axi_arvalid,
  output wire                          s_axi_arready,
  output reg  [AXI_DATA_WIDTH-1:0]     s_axi_rdata,
  output reg  [1:0]                    s_axi_rresp,
  output reg                           s_axi_rvalid,
  input  wire                          s_axi_rready,

  // Command FIFO write sides (one data bus shared by every command FIFO)
  output wire [AXI_DATA_WIDTH-1:0]     cmd_wr_data,
  output wire [7:0]                    dac_cmd_wr_en,
  input  wire [7:0]                    dac_cmd_full,
  output wire [7:0]                    adc_cmd_wr_en,
  input  wire [7:0]                    adc_cmd_full,

  // Read sides (board b's word at [b*AXI_DATA_WIDTH +: AXI_DATA_WIDTH])
  input  wire [8*AXI_DATA_WIDTH-1:0]   dac_debug_rd_data,
  input  wire [7:0]                    dac_debug_empty,
  output wire [7:0]                    dac_debug_rd_en,
  input  wire [8*AXI_DATA_WIDTH-1:0]   dac_data_rd_data,
  input  wire [7:0]                    dac_data_empty,
  output wire [7:0]                    dac_data_rd_en,
  input  wire [8*AXI_DATA_WIDTH-1:0]   adc_debug_rd_data,
  input  wire [7:0]                    adc_debug_empty,
  output wire [7:0]                    adc_debug_rd_en,
  input  wire [8*AXI_DATA_WIDTH-1:0]   adc_data_rd_data,
  input  wire [7:0]                    adc_data_empty,
  output wire [7:0]                    adc_data_rd_en,

  // Capture rate lane: read-delta write strobe (data on cmd_wr_data) and backlog readback
  output wire [7:0]                    rate_wr_en,
  input  wire [8*AXI_DATA_WIDTH-1:0]   rate_backlog,

  // DMA-ownership gating (dac_cmd write, adc_data read), per board
  input  wire [7:0]                    mode_block,
  output wire [7:0]                    mode_viol,

  // Sticky over/underflow per FIFO, per board
  output reg  [7:0]                    dac_cmd_overflow,
  output reg  [7:0]                    adc_cmd_overflow,
  output reg  [7:0]                    dac_data_underflow,
  output reg  [7:0]                    adc_data_underflow
);

  // Validate parameters
  initial begin
    if (NUM_BOARDS < 1 || NUM_BOARDS > 8)
      $error("Invalid NUM_BOARDS %0d. Must be between 1 and 8.", NUM_BOARDS);
    if (AXI_ADDR_WIDTH < 15)
      $error("AXI_ADDR_WIDTH (%0d) must be at least 15 to address eight 4 KB board pages.", AXI_ADDR_WIDTH);
    if (AXI_DATA_WIDTH != 32)
      $error("Invalid AXI_DATA_WIDTH %0d. Must be 32.", AXI_DATA_WIDTH);
  end

  localparam RESP_OKAY = 2'b00;
  localparam [2:0] LANE_DAC_CMD   = 3'd0;
  localparam [2:0] LANE_DAC_DEBUG = 3'd1;
  localparam [2:0] LANE_DAC_DATA  = 3'd2;
  localparam [2:0] LANE_ADC_CMD   = 3'd4;
  localparam [2:0] LANE_ADC_DEBUG = 3'd5;
  localparam [2:0] LANE_ADC_DATA  = 3'd6;
  localparam [2:0] LANE_ADC_RATE  = 3'd7;

  // Mask of boards that exist in this build
  wire [7:0] present = (8'hFF >> (8 - NUM_BOARDS));

  reg [7:0] dac_wr_mode_viol;
  reg [7:0] adc_rd_mode_viol;
  assign mode_viol = dac_wr_mode_viol | adc_rd_mode_viol;

  //// Write path -- decode board page and lane
  wire [2:0] wr_board = s_axi_awaddr[14:12];
  wire [2:0] wr_lane  = s_axi_awaddr[4:2];
  wire       try_write = s_axi_awvalid && s_axi_wvalid;
  wire [7:0] wr_sel    = try_write ? (present & (8'd1 << wr_board)) : 8'd0;
  wire [7:0] wr_dac    = (wr_lane == LANE_DAC_CMD) ? wr_sel : 8'd0;
  wire [7:0] wr_adc    = (wr_lane == LANE_ADC_CMD) ? wr_sel : 8'd0;

  assign s_axi_awready = 1'b1;
  assign s_axi_wready  = 1'b1;
  assign cmd_wr_data   = s_axi_wdata;
  assign dac_cmd_wr_en = wr_dac & ~dac_cmd_full & ~mode_block;
  assign adc_cmd_wr_en = wr_adc & ~adc_cmd_full;
  assign rate_wr_en    = (wr_lane == LANE_ADC_RATE) ? wr_sel : 8'd0;

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

  //// Read path -- decode board page and lane
  wire [2:0] rd_board = s_axi_araddr[14:12];
  wire [2:0] rd_lane  = s_axi_araddr[4:2];
  wire       try_read = s_axi_arvalid;
  wire [7:0] rd_sel   = try_read ? (present & (8'd1 << rd_board)) : 8'd0;
  wire [7:0] rd_dac_dbg = (rd_lane == LANE_DAC_DEBUG) ? rd_sel : 8'd0;
  wire [7:0] rd_dac     = (rd_lane == LANE_DAC_DATA)  ? rd_sel : 8'd0;
  wire [7:0] rd_adc_dbg = (rd_lane == LANE_ADC_DEBUG) ? rd_sel : 8'd0;
  wire [7:0] rd_adc     = (rd_lane == LANE_ADC_DATA)  ? rd_sel : 8'd0;

  assign s_axi_arready   = 1'b1;
  assign dac_debug_rd_en = rd_dac_dbg & ~dac_debug_empty;
  assign dac_data_rd_en  = rd_dac & ~dac_data_empty;
  assign adc_debug_rd_en = rd_adc_dbg & ~adc_debug_empty;
  assign adc_data_rd_en  = rd_adc & ~adc_data_empty & ~mode_block;

  wire        rd_rate   = (rd_lane == LANE_ADC_RATE) && |rd_sel;
  wire        any_rd_en = |{dac_debug_rd_en, dac_data_rd_en, adc_debug_rd_en, adc_data_rd_en} || rd_rate;
  reg  [31:0] rd_word;
  always @(*) begin
    case (rd_lane)
      LANE_DAC_DEBUG: rd_word = dac_debug_rd_data[rd_board*32 +: 32];
      LANE_DAC_DATA:  rd_word = dac_data_rd_data[rd_board*32 +: 32];
      LANE_ADC_DEBUG: rd_word = adc_debug_rd_data[rd_board*32 +: 32];
      LANE_ADC_RATE:  rd_word = rate_backlog[rd_board*32 +: 32];
      default:        rd_word = adc_data_rd_data[rd_board*32 +: 32];
    endcase
  end

  // Read response: popped data when allowed, else zero; always OKAY, never hang
  always @(posedge aclk) begin
    if (!aresetn) begin
      s_axi_rvalid <= 1'b0;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}};
    end else if (try_read) begin
      s_axi_rvalid <= 1'b1;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= any_rd_en ? rd_word : {AXI_DATA_WIDTH{1'b0}};
    end else if (s_axi_rready && s_axi_rvalid) begin
      s_axi_rvalid <= 1'b0;
    end
  end

  //// Sticky flags, one per board per FIFO, each cleared by that FIFO's reset
  genvar b;
  generate
    for (b = 0; b < 8; b = b + 1) begin : g_flags
      // DAC command: wrong-mode poke takes precedence over a genuine overflow
      always @(posedge aclk) begin
        if (!dac_cmd_resetn[b]) begin
          dac_cmd_overflow[b] <= 1'b0;
          dac_wr_mode_viol[b] <= 1'b0;
        end else if (wr_dac[b]) begin
          if (mode_block[b])        dac_wr_mode_viol[b] <= 1'b1;
          else if (dac_cmd_full[b]) dac_cmd_overflow[b] <= 1'b1;
        end
      end

      always @(posedge aclk) begin
        if (!adc_cmd_resetn[b])
          adc_cmd_overflow[b] <= 1'b0;
        else if (wr_adc[b] && adc_cmd_full[b])
          adc_cmd_overflow[b] <= 1'b1;
      end

      // ADC data: wrong-mode poke takes precedence over a genuine underflow
      always @(posedge aclk) begin
        if (!adc_data_resetn[b]) begin
          adc_data_underflow[b] <= 1'b0;
          adc_rd_mode_viol[b]   <= 1'b0;
        end else if (rd_adc[b]) begin
          if (mode_block[b])          adc_rd_mode_viol[b]   <= 1'b1;
          else if (adc_data_empty[b]) adc_data_underflow[b] <= 1'b1;
        end
      end

      always @(posedge aclk) begin
        if (!dac_data_resetn[b])
          dac_data_underflow[b] <= 1'b0;
        else if (rd_dac[b] && dac_data_empty[b])
          dac_data_underflow[b] <= 1'b1;
      end
    end
  endgenerate

endmodule
