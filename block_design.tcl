### Create processing system
# Enable UART1 and I2C0
# Pullup for UART1 RX
# Turn off FCLK1-3 and reset1-3
init_ps ps_0 {
  PCW_USE_M_AXI_GP0 1
  PCW_USE_S_AXI_ACP 0
  PCW_EN_CLK1_PORT 0
  PCW_EN_CLK2_PORT 0
  PCW_EN_CLK3_PORT 0
  PCW_EN_RST1_PORT 0
  PCW_EN_RST2_PORT 0
  PCW_EN_RST3_PORT 0
  PCW_UART1_PERIPHERAL_ENABLE 1
  PCW_UART1_UART1_IO {MIO 36 .. 37}
  PCW_I2C0_PERIPHERAL_ENABLE 1
  PCW_MIO_37_PULLUP enabled
  PCW_I2C0_I2C0_IO {MIO 38 .. 39}
} {
  M_AXI_GP0_ACLK ps_0/FCLK_CLK0
}

### AXI Smart Connect
cell xilinx.com:ip:smartconnect:1.0 axi_smc {
  NUM_SI 1
  NUM_MI 1
} {
  S00_AXI /ps_0/M_AXI_GP0
  aclk ps_0/FCLK_CLK0
}

### SPI clock control
## Clocking wizard contains automatic buffer insertion
cell xilinx.com:ip:clk_wiz:6.0 spi_clk {
  PRIMITIVE MMCM
  USE_POWER_DOWN true
  USE_DYN_RECONFIG true
  PRIM_IN_FREQ 10
  JITTER_OPTIONS PS
  CLKIN1_JITTER_PS 100.0
  CLKIN1_UI_JITTER 0.001
  CLKOUT1_REQUESTED_OUT_FREQ 200.000
  MMCM_REF_JITTER1 0.001
} {
  clk_in1 Scanner_10Mhz_In
  s_axi_aclk ps_0/FCLK_CLK0
  s_axi_lite axi_smc/M00_AXI
}
addr 0x40000000 2048 spi_clk/s_axi_lite


### Create I/O buffers for differential signals

## DAC
# (LDAC)
cell lcb:user:differential_out_buffer:1.0 ldac_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  diff_out_p LDAC_p
  diff_out_n LDAC_n
}
# (~DAC_CS)
cell lcb:user:differential_out_buffer:1.0 n_dac_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_out_p n_DAC_CS_p
  diff_out_n n_DAC_CS_n
}
# (DAC_MOSI)
cell lcb:user:differential_out_buffer:1.0 dac_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_out_p DAC_MOSI_p
  diff_out_n DAC_MOSI_n
}
# (DAC_MISO)
cell lcb:user:differential_in_buffer:1.0 dac_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p DAC_MISO_p
  diff_in_n DAC_MISO_n
}

## ADC
# (~ADC_CS)
cell lcb:user:differential_out_buffer:1.0 n_adc_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_out_p n_ADC_CS_p
  diff_out_n n_ADC_CS_n
}
# (ADC_MOSI)
cell lcb:user:differential_out_buffer:1.0 adc_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_out_p ADC_MOSI_p
  diff_out_n ADC_MOSI_n
}
# (ADC_MISO)
cell lcb:user:differential_in_buffer:1.0 adc_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p ADC_MISO_p
  diff_in_n ADC_MISO_n
}

## Clocks
# (MISO_SCK)
cell lcb:user:differential_in_buffer:1.0 miso_sck_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p MISO_SCK_p
  diff_in_n MISO_SCK_n
}
# (~MOSI_SCK)
cell lcb:user:differential_out_buffer:1.0 n_mosi_sck_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  d_in spi_clk/clk_out1
  diff_out_p n_MOSI_SCK_p
  diff_out_n n_MOSI_SCK_n
}
