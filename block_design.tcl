### Create processing system
# Enable UART1 and I2C0
# Pullup for UART1 RX
init_ps ps_0 {
  PCW_USE_M_AXI_GP0 0
  PCW_USE_S_AXI_ACP 0
  PCW_UART1_PERIPHERAL_ENABLE 1
  PCW_UART1_UART1_IO {MIO 36 .. 37}
  PCW_I2C0_PERIPHERAL_ENABLE 1
  PCW_MIO_37_PULLUP enabled
  PCW_I2C0_I2C0_IO {MIO 38 .. 39}
} {}

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
  diff_out_p n_MOSI_SCK_p
  diff_out_n n_MOSI_SCK_n
}
