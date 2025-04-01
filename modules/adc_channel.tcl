## ADC Channel Module
# This module implements an ADC channel with an integrator and SPI interface.

# System signals
create_bd_pin -dir I -type clock sck
create_bd_pin -dir I -type reset rst

# Config parameters
create_bd_pin -dir I -from 31 -to 0 integ_window
create_bd_pin -dir I -from 15 -to 0 integ_thresh_avg
create_bd_pin -dir I integ_en
create_bd_pin -dir I spi_en

# Status signals
create_bd_pin -dir O setup_done
create_bd_pin -dir O err_thresh_overflow
create_bd_pin -dir O err_thresh_underflow
create_bd_pin -dir O buf_overflow
create_bd_pin -dir O over_threshold
create_bd_pin -dir O premat_trig
create_bd_pin -dir O premat_div

# Trigger
create_bd_pin -dir I trigger

# SPI interface signals
create_bd_pin -dir O n_cs
create_bd_pin -dir O mosi
create_bd_pin -dir I miso_sck
create_bd_pin -dir I miso


# Enable the integrator if both the SPI and integrator enable signals are high
cell xilinx.com:ip:util_vector_logic spi_and_integ_en {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 integ_en
  Op2 spi_en
}

# Negate integ_en for setup checking
cell xilinx.com:ip:util_vector_logic n_integ_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 integ_en
}

# setup_done if integ_en is low or integrator is done
cell xilinx.com:ip:util_vector_logic setup_done_logic {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 n_integ_en/Res
  Res setup_done
}

# Instantiate the threshold integrator module
cell lcb:user:threshold_integrator:1.0 threshold_core {} {
  clk sck
  rst rst
  enable spi_and_integ_en/Res
  window integ_window
  threshold_average integ_thresh_avg
  err_overflow err_thresh_overflow
  err_underflow err_thresh_underflow
  over_threshold over_threshold
  setup_done setup_done_logic/Op2
}
