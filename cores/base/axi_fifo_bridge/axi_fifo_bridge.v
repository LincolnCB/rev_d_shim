`timescale 1 ns / 1 ps

module axi_fifo_bridge #(
  parameter integer AXI_ADDR_WIDTH = 8,
  parameter integer AXI_DATA_WIDTH = 32,
  parameter         ENABLE_WRITE   = 1, // 1=enable AXI writes to FIFO
  parameter         ENABLE_READ    = 1  // 1=enable AXI reads from FIFO
)(
  input  wire                       aclk,
  input  wire                       wr_resetn,
  input  wire                       rd_resetn,

  // AXI4-Lite subordinate interface
  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_awaddr,  // AXI4-Lite subordinate: Write address
  input  wire                        s_axi_awvalid, // AXI4-Lite subordinate: Write address valid
  output wire                        s_axi_awready, // AXI4-Lite subordinate: Write address ready
  input  wire [AXI_DATA_WIDTH-1:0]   s_axi_wdata,   // AXI4-Lite subordinate: Write data
  input  wire [AXI_DATA_WIDTH/8-1:0] s_axi_wstrb,   // AXI4-Lite subordinate: Write strobe
  input  wire                        s_axi_wvalid,  // AXI4-Lite subordinate: Write data valid
  output wire                        s_axi_wready,  // AXI4-Lite subordinate: Write data ready
  output reg  [1:0]                  s_axi_bresp,   // AXI4-Lite subordinate: Write response
  output reg                         s_axi_bvalid,  // AXI4-Lite subordinate: Write response valid
  input  wire                        s_axi_bready,  // AXI4-Lite subordinate: Write response ready
  input  wire [AXI_ADDR_WIDTH-1:0]   s_axi_araddr,  // AXI4-Lite subordinate: Read address
  input  wire                        s_axi_arvalid, // AXI4-Lite subordinate: Read address valid
  output wire                        s_axi_arready, // AXI4-Lite subordinate: Read address ready
  output reg  [AXI_DATA_WIDTH-1:0]   s_axi_rdata,   // AXI4-Lite subordinate: Read data
  output reg  [1:0]                  s_axi_rresp,   // AXI4-Lite subordinate: Read data response
  output reg                         s_axi_rvalid,  // AXI4-Lite subordinate: Read data valid
  input  wire                        s_axi_rready,  // AXI4-Lite subordinate: Read data ready

  // FIFO write side
  output wire [AXI_DATA_WIDTH-1:0]  fifo_wr_data,
  output wire                       fifo_wr_en,
  input  wire                       fifo_full,

  // FIFO read side
  input  wire [AXI_DATA_WIDTH-1:0]  fifo_rd_data,
  output wire                       fifo_rd_en,
  input  wire                       fifo_empty,

  // Datapath-mode gating: high on the side this bridge does not own (the datapath_mux
  // hands the FIFO port to the DMA adapter), so an access here is a wrong-mode poke to
  // accept-and-discard rather than a real transfer.
  input  wire                       wr_mode_block,
  input  wire                       rd_mode_block,
  output wire                       mode_viol,      // wrong-mode access seen (accepted, discarded)

  // Underflow/overflow signals for the AXI side
  output reg                        fifo_underflow,
  output reg                        fifo_overflow
);

  // Validate parameters
  initial begin
    if (AXI_ADDR_WIDTH <= 0)
      $error("Invalid value for AXI_ADDR_WIDTH parameter: %d. Must be greater than 0.", AXI_ADDR_WIDTH);
    if (AXI_DATA_WIDTH <= 0 || AXI_DATA_WIDTH % 8 != 0)
      $error("Invalid value for AXI_DATA_WIDTH parameter: %d. Must be greater than 0 and a multiple of 8.", AXI_DATA_WIDTH);
    if (ENABLE_WRITE != 0 && ENABLE_WRITE != 1)
      $error("Invalid value for ENABLE_WRITE parameter: %d. Must be 0 or 1.", ENABLE_WRITE);
    if (ENABLE_READ != 0 && ENABLE_READ != 1)
      $error("Invalid value for ENABLE_READ parameter: %d. Must be 0 or 1.", ENABLE_READ);
  end

  // Response signals
  localparam RESP_OKAY = 2'b00;

  // Wrong-mode access flags, latched per direction until reset (mirrors overflow/underflow).
  reg wr_mode_viol;
  reg rd_mode_viol;
  assign mode_viol = wr_mode_viol | rd_mode_viol;


  //// Write logic
  // Always accept the request (no hanging). A write reaches the FIFO only when this side
  // owns it and the FIFO has room; a wrong-mode or full write is accepted and discarded.
  wire   try_write = s_axi_awvalid && s_axi_wvalid;
  wire   write_allowed = !fifo_full && !wr_mode_block && ENABLE_WRITE;
  assign s_axi_awready = 1; // Always ready to accept write requests, not allowed to hang
  assign s_axi_wready  = 1; // Always ready to accept write data, not allowed to hang
  assign fifo_wr_en    = try_write && write_allowed;
  assign fifo_wr_data  = s_axi_wdata;

  // Write response: always OKAY (accept-and-discard, never SLVERR) so a wrong-mode or
  // full-FIFO access cannot fault the CPU. A wrong-mode write raises wr_mode_viol; a
  // genuine full write raises fifo_overflow.
  always @(posedge aclk) begin
    if (!wr_resetn) begin
      s_axi_bvalid <= 1'b0;
      s_axi_bresp  <= RESP_OKAY;
      fifo_overflow <= 1'b0; // Reset overflow flag on reset
      wr_mode_viol  <= 1'b0;
    end else begin
      if (try_write) begin
        s_axi_bvalid <= 1'b1;
        s_axi_bresp  <= RESP_OKAY;
        if (wr_mode_block)  wr_mode_viol  <= 1'b1; // wrong-mode poke, discarded
        else if (fifo_full) fifo_overflow <= 1'b1; // genuine overflow, discarded
      end else if (s_axi_bready && s_axi_bvalid) begin
        s_axi_bvalid <= 1'b0;
      end
    end
  end


  //// Read logic
  // Always accept the request (no hanging). A read pops the FIFO only when this side owns
  // it and the FIFO has data; a wrong-mode or empty read returns zero.
  wire   try_read = s_axi_arvalid;
  wire   read_allowed = !fifo_empty && !rd_mode_block && ENABLE_READ;
  assign s_axi_arready = 1; // Always ready to accept read requests, not allowed to hang
  assign fifo_rd_en    = try_read && read_allowed;

  always @(posedge aclk) begin
    if (!rd_resetn) begin
      s_axi_rvalid <= 1'b0;
      s_axi_rresp  <= RESP_OKAY;
      s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}};
      fifo_underflow <= 1'b0; // Reset underflow flag on reset
      rd_mode_viol   <= 1'b0;
    end else begin
      if (fifo_rd_en) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata  <= fifo_rd_data;
        s_axi_rresp  <= RESP_OKAY;
      end else if (try_read) begin
        s_axi_rvalid <= 1'b1;
        s_axi_rdata  <= {AXI_DATA_WIDTH{1'b0}}; // wrong-mode or empty read returns zero
        s_axi_rresp  <= RESP_OKAY;
        if (rd_mode_block)   rd_mode_viol   <= 1'b1; // wrong-mode poke, discarded
        else if (fifo_empty) fifo_underflow <= 1'b1; // genuine underflow, discarded
      end else if (s_axi_rready && s_axi_rvalid) begin
        s_axi_rvalid <= 1'b0;
      end
    end
  end

endmodule
