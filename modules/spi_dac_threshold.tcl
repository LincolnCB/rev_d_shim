# Get the threshold_core_level flag from the calling context
set threshold_core_level [module_get_upvar threshold_core_level]

# System signals
create_bd_pin -dir I -type clock spi_clk
create_bd_pin -dir I -type reset resetn

# Threshold inputs
create_bd_pin -dir I -from 31 -to 0 thresh_window
create_bd_pin -dir I -from 14 -to 0 thresh_val
create_bd_pin -dir I thresh_en
create_bd_pin -dir I sample_core_setup
create_bd_pin -dir I -from 119 -to 0 abs_sample_concat


# Threshold outputs
create_bd_pin -dir O setup_done
create_bd_pin -dir O over_thresh
create_bd_pin -dir O thresh_overflow
create_bd_pin -dir O thresh_underflow

##################################################

# Constants
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  CONST_VAL 0
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  CONST_VAL 1
} {}

### Threshold core
switch $threshold_core_level {
  0 {
    wire const_0/dout over_thresh
    wire const_0/dout thresh_overflow
    wire const_0/dout thresh_underflow
  }

  1 {
    cell shim:user:threshold_timer threshold_core {} {
      clk spi_clk
      resetn resetn
      enable thresh_en
      window thresh_window
      thresh_val thresh_val
      sample_core_setup sample_core_setup
      abs_sample_concat abs_sample_concat
      over_thresh over_thresh
    }
    wire const_0/dout thresh_overflow
    wire const_0/dout thresh_underflow
  }

  2 {
    cell shim:user:threshold_integrator threshold_core {} {
      clk spi_clk
      resetn resetn
      enable thresh_en
      window thresh_window
      thresh_val thresh_val
      sample_core_setup sample_core_setup
      abs_sample_concat abs_sample_concat
      err_overflow thresh_overflow
      err_underflow thresh_underflow
      over_thresh over_thresh
    }
  }
}

# Negate thresh_en for setup checking
cell xilinx.com:ip:util_vector_logic n_thresh_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 thresh_en
}

# Threshold marked done if core is done or disabled
if {$threshold_core_level == 0} {
  wire const_1/dout setup_done
} else {
  cell xilinx.com:ip:util_vector_logic thresh_setup_done {
    C_SIZE 1
    C_OPERATION or
  } {
    Op1 n_thresh_en/Res
    Op2 threshold_core/setup_done
    Res setup_done
  }
}
