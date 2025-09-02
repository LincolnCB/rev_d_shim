`timescale 1ns/1ps

module shim_spi_cfg_sync (
  input  wire        aclk,       // AXI domain clock
  input  wire        aresetn,    // Active low reset signal
  input  wire        spi_clk,    // SPI domain clock
  input  wire        spi_resetn, // Active low reset signal for SPI domain

  // Inputs from axi_shim_cfg (AXI domain)
  input  wire        spi_en,
  input  wire        block_bufs,
  input  wire [14:0] integ_thresh_avg,
  input  wire [31:0] integ_window,
  input  wire        integ_en,
  input  wire [ 4:0] dac_n_cs_high_time,
  input  wire [ 7:0] adc_n_cs_high_time,
  input  wire [15:0] boot_test_skip,
  input  wire [15:0] debug,

  // Synchronized outputs to SPI domain
  output wire        spi_en_sync,
  output wire        block_bufs_sync,
  output wire [14:0] integ_thresh_avg_sync,
  output wire [31:0] integ_window_sync,
  output wire        integ_en_sync,
  output wire [ 4:0] dac_n_cs_high_time_sync,
  output wire [ 7:0] adc_n_cs_high_time_sync,
  output wire [15:0] boot_test_skip_sync,
  output wire [15:0] debug_sync
);

  // Default values for registers
  localparam [14:0] integ_thresh_avg_default = 15'h1000;
  localparam [31:0] integ_window_default = 32'h00010000;
  localparam [ 4:0] dac_n_cs_high_time_default = 5'd31; // Max value
  localparam [ 7:0] adc_n_cs_high_time_default = 8'd255; // Max value

  // Synchronize each signal
  // Use sync_coherent for multi-bit data,
  //   sync_incoherent for data where individual bits are not coherent with each other

  // SPI enable (incoherent)
  sync_incoherent #(
    .WIDTH(1),
    .DEPTH(5) // Use deeper synchronizer to give extra delay for this signal
  ) sync_spi_en (
    .clk(spi_clk),
    .resetn(spi_resetn),
    .din(spi_en),
    .dout(spi_en_sync)
  );

  // Block buffers (incoherent)
  sync_incoherent #(
    .WIDTH(1)
  ) sync_block_bufs (
    .clk(spi_clk),
    .resetn(spi_resetn),
    .din(block_bufs),
    .dout(block_bufs_sync)
  );
  
  // Integrator enable (incoherent)
  sync_incoherent #(
    .WIDTH(1)
  ) sync_integ_en (
    .clk(spi_clk),
    .resetn(spi_resetn),
    .din(integ_en),
    .dout(integ_en_sync)
  );

  // Integrator threshold average (coherent)
  sync_coherent #(
    .WIDTH(15)
  ) sync_integ_thresh_avg (
    .in_clk(aclk),
    .in_resetn(aresetn),
    .out_clk(spi_clk),
    .out_resetn(spi_resetn),
    .din(integ_thresh_avg),
    .dout(integ_thresh_avg_sync),
    .dout_default(integ_thresh_avg_default)
  );

  // Integrator window (coherent)
  sync_coherent #(
    .WIDTH(32)
  ) sync_integ_window (
    .in_clk(aclk),
    .in_resetn(aresetn),
    .out_clk(spi_clk),
    .out_resetn(spi_resetn),
    .din(integ_window),
    .dout(integ_window_sync),
    .dout_default(integ_window_default)
  );

  // DAC n_cs_high_time (coherent)
  sync_coherent #(
    .WIDTH(5)
  ) sync_dac_n_cs_high_time (
    .in_clk(aclk),
    .in_resetn(aresetn),
    .out_clk(spi_clk),
    .out_resetn(spi_resetn),
    .din(dac_n_cs_high_time),
    .dout(dac_n_cs_high_time_sync),
    .dout_default(dac_n_cs_high_time_default)
  );

  // ADC n_cs_high_time (coherent)
  sync_coherent #(
    .WIDTH(8)
  ) sync_adc_n_cs_high_time (
    .in_clk(aclk),
    .in_resetn(aresetn),
    .out_clk(spi_clk),
    .out_resetn(spi_resetn),
    .din(adc_n_cs_high_time),
    .dout(adc_n_cs_high_time_sync),
    .dout_default(adc_n_cs_high_time_default)
  );

  // Boot test skip (incoherent)
  sync_incoherent #(
    .WIDTH(16)
  ) sync_boot_test_skip (
    .clk(spi_clk),
    .resetn(spi_resetn),
    .din(boot_test_skip),
    .dout(boot_test_skip_sync)
  );

  // Debug (incoherent)
  sync_incoherent #(
    .WIDTH(16)
  ) sync_debug (
    .clk(spi_clk),
    .resetn(spi_resetn),
    .din(debug),
    .dout(debug_sync)
  );
  
endmodule
