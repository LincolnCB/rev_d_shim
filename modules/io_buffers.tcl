### Create I/O buffers for differential signals

## DAC
# (LDAC)
create_bd_pin -dir I ldac
create_bd_pin -dir O ldac_p
create_bd_pin -dir O ldac_n
cell lcb:user:differential_out_buffer ldac_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  d_in ldac
  diff_out_p ldac_p
  diff_out_n ldac_n
}
# (~DAC_CS)
create_bd_pin -dir I n_dac_cs
create_bd_pin -dir O n_dac_cs_p
create_bd_pin -dir O n_dac_cs_n
cell lcb:user:differential_out_buffer n_dac_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in n_dac_cs
  diff_out_p n_dac_cs_p
  diff_out_n n_dac_cs_n
}
# (DAC_MOSI)
create_bd_pin -dir I dac_mosi
create_bd_pin -dir O dac_mosi_p
create_bd_pin -dir O dac_mosi_n
cell lcb:user:differential_out_buffer dac_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in dac_mosi
  diff_out_p dac_mosi_p
  diff_out_n dac_mosi_n
}
# (DAC_MISO)
create_bd_pin -dir O dac_miso
create_bd_pin -dir I dac_miso_p
create_bd_pin -dir I dac_miso_n
cell lcb:user:differential_in_buffer dac_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p dac_miso_p
  diff_in_n dac_miso_n
  d_out dac_miso
}

## ADC
# (~ADC_CS)
create_bd_pin -dir I n_adc_cs
create_bd_pin -dir O n_adc_cs_p
create_bd_pin -dir O n_adc_cs_n
cell lcb:user:differential_out_buffer n_adc_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in n_adc_cs
  diff_out_p n_adc_cs_p
  diff_out_n n_adc_cs_n
}
# (ADC_MOSI)
create_bd_pin -dir I adc_mosi
create_bd_pin -dir O adc_mosi_p
create_bd_pin -dir O adc_mosi_n
cell lcb:user:differential_out_buffer adc_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in adc_mosi
  diff_out_p adc_mosi_p
  diff_out_n adc_mosi_n
}
# (ADC_MISO)
create_bd_pin -dir O adc_miso
create_bd_pin -dir I adc_miso_p
create_bd_pin -dir I adc_miso_n
cell lcb:user:differential_in_buffer adc_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p adc_miso_p
  diff_in_n adc_miso_n
  d_out adc_miso
}

## Clocks
# (MISO_SCK)
create_bd_pin -dir O -type clock miso_sck
create_bd_pin -dir I -type clock miso_sck_p
create_bd_pin -dir I -type clock miso_sck_n
cell lcb:user:differential_in_buffer miso_sck_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p miso_sck_p
  diff_in_n miso_sck_n
  d_out miso_sck
}
# (~MOSI_SCK)
create_bd_pin -dir I -type clock n_mosi_sck
create_bd_pin -dir O -type clock n_mosi_sck_p
create_bd_pin -dir O -type clock n_mosi_sck_n
cell lcb:user:differential_out_buffer n_mosi_sck_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  d_in n_mosi_sck
  diff_out_p n_mosi_sck_p
  diff_out_n n_mosi_sck_n
}
