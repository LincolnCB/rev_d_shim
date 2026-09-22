`timescale 1 ns / 1 ps

// Per-board 2:1 select between the programmed-I/O (PIO, direct CPU register access) and
// DMA owners of a board's two high-rate FIFO ports -- the DAC command FIFO write side and
// the ADC data FIFO read side -- gated by a single datapath_mode bit (0 = PIO, 1 = DMA).
//
// The select is a hard lock, not a convention: the idle owner is gated on both the data
// path and its handshake, so it physically cannot touch the FIFO. In DMA mode the PIO
// side sees full=1 / empty=1 (its wr_en/rd_en fold to 0) and only the DMA side's wr_en /
// rd_en reach the FIFO; in PIO mode the reverse. So a wrong-mode access can drop data
// but can never corrupt the FIFO. Reporting a wrong-mode access (mode_viol) lives in the
// PIO axi_fifo_bridge, which is where the raw access attempt is still visible; this mux
// only enforces the lock.
//
// The write side carries a real 32-bit data mux (the FIFO has one write-data port), so it
// lives in RTL rather than IPI gates; the read side only muxes the 1-bit read-enable and
// fans the read data out to both owners.
module datapath_mux #(
  parameter integer DATA_WIDTH  = 32,
  parameter integer COUNT_WIDTH = 14  // ADC data FIFO read-side count width (ADDR_WIDTH+1)
)(
  input  wire                   datapath_mode, // 0 = PIO owns, 1 = DMA owns

  // DAC command FIFO write port (to the FIFO)
  output wire [DATA_WIDTH-1:0]  cmd_wr_data,
  output wire                   cmd_wr_en,
  input  wire                   cmd_full,

  // DAC command write, PIO owner (axi_fifo_bridge)
  input  wire [DATA_WIDTH-1:0]  cmd_pio_wr_data,
  input  wire                   cmd_pio_wr_en,
  output wire                   cmd_pio_full,

  // DAC command write, DMA owner (axis_fifo_bridge)
  input  wire [DATA_WIDTH-1:0]  cmd_dma_wr_data,
  input  wire                   cmd_dma_wr_en,
  output wire                   cmd_dma_full,

  // ADC data FIFO read port (from the FIFO)
  input  wire [DATA_WIDTH-1:0]  data_rd_data,
  input  wire                   data_empty,
  input  wire [COUNT_WIDTH-1:0] data_count,
  output wire                   data_rd_en,

  // ADC data read, PIO owner (axi_fifo_bridge)
  output wire [DATA_WIDTH-1:0]  data_pio_rd_data,
  output wire                   data_pio_empty,
  input  wire                   data_pio_rd_en,

  // ADC data read, DMA owner (adc_packetizer)
  output wire [DATA_WIDTH-1:0]  data_dma_rd_data,
  output wire                   data_dma_empty,
  output wire [COUNT_WIDTH-1:0] data_dma_count,
  input  wire                   data_dma_rd_en
);

  // Parameter validation
  initial begin
    if (DATA_WIDTH <= 0 || DATA_WIDTH % 8 != 0)
      $error("Invalid DATA_WIDTH %0d: must be positive and a multiple of 8.", DATA_WIDTH);
    if (COUNT_WIDTH <= 0)
      $error("Invalid COUNT_WIDTH %0d: must be greater than 0.", COUNT_WIDTH);
  end

  // DAC command write: forward the selected owner's data/enable; gate the idle owner by
  // holding its full high so its wr_en folds to zero.
  assign cmd_wr_data  = datapath_mode ? cmd_dma_wr_data : cmd_pio_wr_data;
  assign cmd_wr_en    = datapath_mode ? cmd_dma_wr_en   : cmd_pio_wr_en;
  assign cmd_pio_full = datapath_mode ? 1'b1     : cmd_full;
  assign cmd_dma_full = datapath_mode ? cmd_full : 1'b1;

  // ADC data read: fan the read data to both owners; forward the selected owner's rd_en;
  // gate the idle owner by holding its empty high (and a zero count for the packetizer).
  assign data_rd_en       = datapath_mode ? data_dma_rd_en : data_pio_rd_en;
  assign data_pio_rd_data = data_rd_data;
  assign data_pio_empty   = datapath_mode ? 1'b1       : data_empty;
  assign data_dma_rd_data  = data_rd_data;
  assign data_dma_empty    = datapath_mode ? data_empty : 1'b1;
  assign data_dma_count    = datapath_mode ? data_count : {COUNT_WIDTH{1'b0}};

endmodule
