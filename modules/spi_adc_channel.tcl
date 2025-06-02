### ADC Channel Module
## This module implements an ADC channel with an integrator and SPI interface.

# System signals
create_bd_pin -dir I -type clock spi_clk
create_bd_pin -dir I -type reset resetn

## Status signals
# System status
create_bd_pin -dir O setup_done
# ADC status
create_bd_pin -dir O bad_cmd
create_bd_pin -dir O cmd_buf_underflow
create_bd_pin -dir O data_buf_overflow
create_bd_pin -dir O unexp_trig

# Commands and data
create_bd_pin -dir I -from 31 -to 0 adc_cmd
create_bd_pin -dir O adc_cmd_rd_en
create_bd_pin -dir I adc_cmd_empty
create_bd_pin -dir O -from 31 -to 0 adc_data
create_bd_pin -dir O adc_data_wr_en
create_bd_pin -dir I adc_data_full

# Trigger
create_bd_pin -dir I trigger

# SPI interface signals
create_bd_pin -dir O n_cs
create_bd_pin -dir O mosi
create_bd_pin -dir I miso_sck
create_bd_pin -dir I miso

# TODO: Temporary constant signal for setup_done
cell xilinx.com:ip:xlconstant:1.1 setup_done_const {
  CONST_VAL 0
  CONST_WIDTH 1
} {
  dout setup_done
}

# Loopback AND of miso_sck and miso to mosi
cell xilinx.com:ip:util_vector_logic mosi_loopback {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 miso
  Op2 miso_sck
  Res mosi
}
