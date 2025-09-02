### ADC Channel Module
## This module implements an ADC channel with an integrator and SPI interface.

# System signals
create_bd_pin -dir I -type clock spi_clk
create_bd_pin -dir I -type reset resetn

# Config parameters
create_bd_pin -dir I -from 8 -to 0 adc_n_cs_high_time
create_bd_pin -dir I boot_test_skip
create_bd_pin -dir I debug

## Status signals
# System status
create_bd_pin -dir O setup_done
# ADC status
create_bd_pin -dir O boot_fail
create_bd_pin -dir O bad_cmd
create_bd_pin -dir O cmd_buf_underflow
create_bd_pin -dir O data_buf_overflow
create_bd_pin -dir O unexp_trig
create_bd_pin -dir O delay_too_short

# Commands and data
create_bd_pin -dir I -from 31 -to 0 adc_cmd
create_bd_pin -dir O adc_cmd_rd_en
create_bd_pin -dir I adc_cmd_empty
create_bd_pin -dir O -from 31 -to 0 adc_data
create_bd_pin -dir O adc_data_wr_en
create_bd_pin -dir I adc_data_full

# Block command and data buffers until HW Manager is ready
create_bd_pin -dir I block_bufs

# Trigger
create_bd_pin -dir I trigger
create_bd_pin -dir O waiting_for_trig

# SPI interface signals
create_bd_pin -dir O n_cs
create_bd_pin -dir O mosi
create_bd_pin -dir I miso_sck
create_bd_pin -dir I miso

##################################################

### ADC SPI Controller
## Block the command buffer if needed (cmd_buf_empty OR block_bufs)
cell xilinx.com:ip:util_vector_logic adc_cmd_empty_blocked {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 adc_cmd_empty
  Op2 block_bufs
}
## Block the data buffer if needed (adc_data_full OR (block_bufs AND NOT debug))
cell xilinx.com:ip:util_vector_logic n_debug {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 debug
}
cell xilinx.com:ip:util_vector_logic block_bufs_and_not_debug {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 block_bufs
  Op2 n_debug/Res
}
cell xilinx.com:ip:util_vector_logic adc_data_full_blocked {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 adc_data_full
  Op2 block_bufs_and_not_debug/Res
}
## MISO clock-domain synchronous reset
cell xilinx.com:ip:proc_sys_reset:5.0 miso_rst {} {
  ext_reset_in resetn
  slowest_sync_clk miso_sck
}
## ADC SPI core
cell lcb:user:shim_ads816x_adc_ctrl adc_spi {} {
  clk spi_clk
  resetn resetn
  boot_test_skip boot_test_skip
  debug debug
  n_cs_high_time adc_n_cs_high_time
  cmd_buf_rd_en adc_cmd_rd_en
  cmd_buf_word adc_cmd
  cmd_buf_empty adc_cmd_empty_blocked/Res
  data_buf_wr_en adc_data_wr_en
  data_word adc_data
  data_buf_full adc_data_full_blocked/Res
  trigger trigger
  setup_done setup_done
  boot_fail boot_fail
  bad_cmd bad_cmd
  cmd_buf_underflow cmd_buf_underflow
  data_buf_overflow data_buf_overflow
  unexp_trig unexp_trig
  delay_too_short delay_too_short
  waiting_for_trig waiting_for_trig
  n_cs n_cs
  mosi mosi
  miso_sck miso_sck
  miso_resetn miso_rst/peripheral_aresetn
  miso miso
}
