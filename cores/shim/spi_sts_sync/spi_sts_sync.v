`timescale 1ns/1ps

module spi_sts_sync (
  input  wire        aclk,       // AXI domain clock
  input  wire        aresetn,    // Active low reset signal
  input  wire        spi_clk,    // SPI domain clock
  input  wire        spi_resetn, // Active low reset signal for SPI domain

  //// Inputs from SPI domain
  // SPI system status
  input  wire          spi_off,
  // Threshold core status
  input  wire [  7:0]  over_thresh,
  input  wire [  7:0]  thresh_underflow,
  input  wire [  7:0]  thresh_overflow,
  // Trigger channel status
  input  wire [ 31:0]  trig_counter,
  input  wire          bad_trig_cmd,
  input  wire          trig_data_buf_overflow,
  // DAC channel status
  input  wire [  7:0]  dac_boot_fail,
  input  wire [  7:0]  bad_dac_cmd,
  input  wire [  7:0]  dac_cal_oob,
  input  wire [  7:0]  dac_val_oob,
  input  wire [  7:0]  dac_cmd_buf_underflow,
  input  wire [  7:0]  dac_data_buf_overflow,
  input  wire [  7:0]  unexp_dac_trig,
  input  wire [  7:0]  ldac_misalign,
  input  wire [  7:0]  dac_delay_too_short,
  input  wire [255:0]  last_received_dac_cmds_concat,
  input  wire [255:0]  dac_cmds_since_reset_concat,
  // ADC channel status
  input  wire [  7:0]  adc_boot_fail,
  input  wire [  7:0]  bad_adc_cmd,
  input  wire [  7:0]  adc_cmd_buf_underflow,
  input  wire [  7:0]  adc_data_buf_overflow,
  input  wire [  7:0]  unexp_adc_trig,
  input  wire [  7:0]  adc_delay_too_short,
  input  wire [255:0]  last_received_adc_cmds_concat,
  input  wire [255:0]  adc_cmds_since_reset_concat,

  //// Synchronized outputs to AXI domain
  // SPI system status
  output wire        spi_off_sync,
  // Threshold core status
  output wire [  7:0]  over_thresh_sync,
  output wire [  7:0]  thresh_underflow_sync,
  output wire [  7:0]  thresh_overflow_sync,
  // Trigger channel status
  output wire [ 31:0]  trig_counter_sync,
  output wire          bad_trig_cmd_sync,
  output wire          trig_data_buf_overflow_sync,
  // DAC channel status
  output wire [  7:0]  dac_boot_fail_sync,
  output wire [  7:0]  bad_dac_cmd_sync,
  output wire [  7:0]  dac_cal_oob_sync,
  output wire [  7:0]  dac_val_oob_sync,
  output wire [  7:0]  dac_cmd_buf_underflow_sync,
  output wire [  7:0]  dac_data_buf_overflow_sync,
  output wire [  7:0]  unexp_dac_trig_sync,
  output wire [  7:0]  ldac_misalign_sync,
  output wire [  7:0]  dac_delay_too_short_sync,
  output wire [255:0]  last_received_dac_cmds_concat_sync,
  output wire [255:0]  dac_cmds_since_reset_concat_sync,
  // ADC channel status
  output wire [  7:0]  adc_boot_fail_sync,
  output wire [  7:0]  bad_adc_cmd_sync,
  output wire [  7:0]  adc_cmd_buf_underflow_sync,
  output wire [  7:0]  adc_data_buf_overflow_sync,
  output wire [  7:0]  unexp_adc_trig_sync,
  output wire [  7:0]  adc_delay_too_short_sync,
  output wire [255:0]  last_received_adc_cmds_concat_sync,
  output wire [255:0]  adc_cmds_since_reset_concat_sync
);

  //// Synchronize each signal using a sync_incoherent module
  // SPI system on/off status
  sync_incoherent #(
    .WIDTH(1)
  ) sync_spi_off (
    .clk(aclk),
    .resetn(aresetn),
    .din(spi_off),
    .dout(spi_off_sync)
  );

  // Threshold status
  sync_incoherent #(
    .WIDTH(8)
  ) sync_over_thresh (
    .clk(aclk),
    .resetn(aresetn),
    .din(over_thresh),
    .dout(over_thresh_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_thresh_underflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(thresh_underflow),
    .dout(thresh_underflow_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_thresh_overflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(thresh_overflow),
    .dout(thresh_overflow_sync)
  );

  // Trigger channel status
  sync_coherent #(
    .WIDTH(32)
  ) sync_trig_counter (
    .in_clk(aclk),
    .in_resetn(aresetn),
    .out_clk(spi_clk),
    .out_resetn(spi_resetn),
    .din(trig_counter),
    .dout(trig_counter_sync),
    .dout_default(32'd0)
  );
  sync_incoherent #(
    .WIDTH(1)
  ) sync_bad_trig_cmd (
    .clk(aclk),
    .resetn(aresetn),
    .din(bad_trig_cmd),
    .dout(bad_trig_cmd_sync)
  );
  sync_incoherent #(
    .WIDTH(1)
  ) sync_trig_data_buf_overflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(trig_data_buf_overflow),
    .dout(trig_data_buf_overflow_sync)
  );

  // DAC channel status
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_boot_fail (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_boot_fail),
    .dout(dac_boot_fail_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_bad_dac_cmd (
    .clk(aclk),
    .resetn(aresetn),
    .din(bad_dac_cmd),
    .dout(bad_dac_cmd_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_cal_oob (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_cal_oob),
    .dout(dac_cal_oob_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_val_oob (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_val_oob),
    .dout(dac_val_oob_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_cmd_buf_underflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_cmd_buf_underflow),
    .dout(dac_cmd_buf_underflow_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_data_buf_overflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_data_buf_overflow),
    .dout(dac_data_buf_overflow_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_unexp_dac_trig (
    .clk(aclk),
    .resetn(aresetn),
    .din(unexp_dac_trig),
    .dout(unexp_dac_trig_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_ldac_misalign (
    .clk(aclk),
    .resetn(aresetn),
    .din(ldac_misalign),
    .dout(ldac_misalign_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_dac_delay_too_short (
    .clk(aclk),
    .resetn(aresetn),
    .din(dac_delay_too_short),
    .dout(dac_delay_too_short_sync)
  );
  // Generate 8 coherent synchronizers for 32-bit slices of the 256-bit concatenated command history signals
  genvar i;
  generate
    for (i = 0; i < 8; i = i + 1) begin : DAC_CMD_HISTORY_SYNC
      sync_coherent #(
        .WIDTH(32)
      ) sync_last_received_dac_cmds_concat (
        .in_clk(aclk),
        .in_resetn(aresetn),
        .out_clk(spi_clk),
        .out_resetn(spi_resetn),
        .din(last_received_dac_cmds_concat[(i+1)*32-1 -:32]),
        .dout(last_received_dac_cmds_concat_sync[(i+1)*32-1 -:32]),
        .dout_default(32'd0)
      );
      sync_coherent #(
        .WIDTH(32)
      ) sync_dac_cmds_since_reset_concat (
        .in_clk(aclk),
        .in_resetn(aresetn),
        .out_clk(spi_clk),
        .out_resetn(spi_resetn),
        .din(dac_cmds_since_reset_concat[(i+1)*32-1 -:32]),
        .dout(dac_cmds_since_reset_concat_sync[(i+1)*32-1 -:32]),
        .dout_default(32'd0)
      );
    end
  endgenerate

  // ADC channel status
  sync_incoherent #(
    .WIDTH(8)
  ) sync_adc_boot_fail (
    .clk(aclk),
    .resetn(aresetn),
    .din(adc_boot_fail),
    .dout(adc_boot_fail_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_bad_adc_cmd (
    .clk(aclk),
    .resetn(aresetn),
    .din(bad_adc_cmd),
    .dout(bad_adc_cmd_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_adc_cmd_buf_underflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(adc_cmd_buf_underflow),
    .dout(adc_cmd_buf_underflow_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_adc_data_buf_overflow (
    .clk(aclk),
    .resetn(aresetn),
    .din(adc_data_buf_overflow),
    .dout(adc_data_buf_overflow_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_unexp_adc_trig (
    .clk(aclk),
    .resetn(aresetn),
    .din(unexp_adc_trig),
    .dout(unexp_adc_trig_sync)
  );
  sync_incoherent #(
    .WIDTH(8)
  ) sync_adc_delay_too_short (
    .clk(aclk),
    .resetn(aresetn),
    .din(adc_delay_too_short),
    .dout(adc_delay_too_short_sync)
  );
  // Generate 8 coherent synchronizers for 32-bit slices of the 256-bit concatenated command history signals
  genvar j;
  generate
    for (j = 0; j < 8; j = j + 1) begin : ADC_CMD_HISTORY_SYNC
      sync_coherent #(
        .WIDTH(32)
      ) sync_last_received_adc_cmds_concat (
        .in_clk(aclk),
        .in_resetn(aresetn),
        .out_clk(spi_clk),
        .out_resetn(spi_resetn),
        .din(last_received_adc_cmds_concat[(j+1)*32-1 -:32]),
        .dout(last_received_adc_cmds_concat_sync[(j+1)*32-1 -:32]),
        .dout_default(32'd0)
      );
      sync_coherent #(
        .WIDTH(32)
      ) sync_adc_cmds_since_reset_concat (
        .in_clk(aclk),
        .in_resetn(aresetn),
        .out_clk(spi_clk),
        .out_resetn(spi_resetn),
        .din(adc_cmds_since_reset_concat[(j+1)*32-1 -:32]),
        .dout(adc_cmds_since_reset_concat_sync[(j+1)*32-1 -:32]),
        .dout_default(32'd0)
      );
    end
  endgenerate

endmodule
