# System signals
create_bd_pin -dir I -type clock spi_clk
create_bd_pin -dir I -type reset resetn

# Integrator inputs
create_bd_pin -dir I -from 31 -to 0 integ_window
create_bd_pin -dir I -from 14 -to 0 integ_thresh_avg
create_bd_pin -dir I integ_en
create_bd_pin -dir I sample_core_setup
create_bd_pin -dir I -from 119 -to 0 abs_sample_concat


# Integrator outputs
create_bd_pin -dir O setup_done
create_bd_pin -dir O over_thresh
create_bd_pin -dir O thresh_overflow
create_bd_pin -dir O thresh_underflow

##################################################

### Integrator

# Instantiate the threshold integrator module
cell shim:user:threshold_integrator threshold_core {} {
  clk spi_clk
  resetn resetn
  enable integ_en
  window integ_window
  threshold_average integ_thresh_avg
  sample_core_setup sample_core_setup
  abs_sample_concat abs_sample_concat
  err_overflow thresh_overflow
  err_underflow thresh_underflow
  over_thresh over_thresh
}

# Negate integ_en for setup checking
cell xilinx.com:ip:util_vector_logic n_integ_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 integ_en
}

# Integrator marked done if integrator is done or disabled
cell xilinx.com:ip:util_vector_logic integ_done {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 n_integ_en/Res
  Op2 threshold_core/setup_done
  Res setup_done
}
