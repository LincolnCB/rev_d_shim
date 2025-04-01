## SPI Clock Domain Module
# This module implements the SPI clock domain containing the DAC and ADC channels.
# It synchronizes the configuration settings across clock domains

##################################################

### Ports

# System ports
create_bd_pin -dir I -type clock clk
create_bd_pin -dir I -type clock sck
create_bd_pin -dir I -type reset rst

# Configuration signals (need synchronization)
create_bd_pin -dir I -from 31 -to 0 trig_lockout
create_bd_pin -dir I -from 15 -to 0 cal_offset
create_bd_pin -dir I -from 15 -to 0 dac_divider
create_bd_pin -dir I -from 15 -to 0 integ_thresh_avg
create_bd_pin -dir I -from 31 -to 0 integ_window
create_bd_pin -dir I integ_en
create_bd_pin -dir I spi_en

# Status signals (need synchronization)
create_bd_pin -dir O -from 7 -to 0 spi_running
create_bd_pin -dir O -from 7 -to 0 dac_over_thresh
create_bd_pin -dir O -from 7 -to 0 adc_over_thresh
create_bd_pin -dir O -from 7 -to 0 dac_thresh_underflow
create_bd_pin -dir O -from 7 -to 0 dac_thresh_overflow
create_bd_pin -dir O -from 7 -to 0 adc_thresh_underflow
create_bd_pin -dir O -from 7 -to 0 adc_thresh_overflow
create_bd_pin -dir O -from 7 -to 0 dac_buf_underflow
create_bd_pin -dir O -from 7 -to 0 adc_buf_underflow
create_bd_pin -dir O -from 7 -to 0 premat_dac_trig
create_bd_pin -dir O -from 7 -to 0 premat_adc_trig
create_bd_pin -dir O -from 7 -to 0 premat_dac_div
create_bd_pin -dir O -from 7 -to 0 premat_adc_div

# Trigger
create_bd_pin -dir I trigger_gated

# SPI interface signals (out)
create_bd_pin -dir O -from 7 -to 0 ldac
create_bd_pin -dir O -from 7 -to 0 n_dac_cs
create_bd_pin -dir O -from 7 -to 0 dac_mosi
create_bd_pin -dir O -from 7 -to 0 n_adc_cs
create_bd_pin -dir O -from 7 -to 0 adc_mosi

# SPI interface signals (in)
create_bd_pin -dir I -from 7 -to 0 miso_sck
create_bd_pin -dir I -from 7 -to 0 dac_miso
create_bd_pin -dir I -from 7 -to 0 adc_miso

##################################################

### Clock domain crossings

## SPI system configuration synchronization
cell lcb:user:spi_cfg_sync:1.0 spi_cfg_sync {
} {
  clk clk
  spi_clk sck
  rst rst
  trig_lockout trig_lockout
  cal_offset cal_offset
  dac_divider dac_divider
  integ_thresh_avg integ_thresh_avg
  integ_window integ_window
  integ_en integ_en
  spi_en spi_en
}

## SPI system status synchronization
cell lcb:user:spi_sts_sync:1.0 spi_sts_sync {
} {
  clk clk
  spi_clk sck
  rst rst
  spi_running_stable spi_running
  dac_over_thresh_stable dac_over_thresh
  adc_over_thresh_stable adc_over_thresh
  dac_thresh_underflow_stable dac_thresh_underflow
  dac_thresh_overflow_stable dac_thresh_overflow
  adc_thresh_underflow_stable adc_thresh_underflow
  adc_thresh_overflow_stable adc_thresh_overflow
  dac_buf_underflow_stable dac_buf_underflow
  adc_buf_overflow_stable adc_buf_underflow
  premat_dac_trig_stable premat_dac_trig
  premat_adc_trig_stable premat_adc_trig
  premat_dac_div_stable premat_dac_div
  premat_adc_div_stable premat_adc_div
}

##################################################

### DAC and ADC Channels

## Channel 1
module dac_ch1 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch1 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 2
module dac_ch2 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch2 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 3
module dac_ch3 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch3 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 4
module dac_ch4 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch4 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 5
module dac_ch5 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch5 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 6
module dac_ch6 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch6 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 7
module dac_ch7 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch7 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

## Channel 8
module dac_ch8 {
  source projects/rev_d_shim/modules/dac_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}
module adc_ch8 {
  source projects/rev_d_shim/modules/adc_channel.tcl
} {
  sck sck
  rst rst
  integ_window spi_cfg_sync/integ_window_stable
  integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
  integ_en spi_cfg_sync/integ_en_stable
  spi_en spi_cfg_sync/spi_en_stable
}

##################################################

### SPI signals

## Outputs
# LDAC
cell xilinx.com:ip:xlconcat:2.1 ldac_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/ldac
  In1 dac_ch2/ldac
  In2 dac_ch3/ldac
  In3 dac_ch4/ldac
  In4 dac_ch5/ldac
  In5 dac_ch6/ldac
  In6 dac_ch7/ldac
  In7 dac_ch8/ldac
  dout ldac
}
# ~DAC_CS
cell xilinx.com:ip:xlconcat:2.1 n_dac_cs_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/n_cs
  In1 dac_ch2/n_cs
  In2 dac_ch3/n_cs
  In3 dac_ch4/n_cs
  In4 dac_ch5/n_cs
  In5 dac_ch6/n_cs
  In6 dac_ch7/n_cs
  In7 dac_ch8/n_cs
  dout n_dac_cs
}
# DAC_MOSI
cell xilinx.com:ip:xlconcat:2.1 dac_mosi_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/mosi
  In1 dac_ch2/mosi
  In2 dac_ch3/mosi
  In3 dac_ch4/mosi
  In4 dac_ch5/mosi
  In5 dac_ch6/mosi
  In6 dac_ch7/mosi
  In7 dac_ch8/mosi
  dout dac_mosi
}
# ~ADC_CS
cell xilinx.com:ip:xlconcat:2.1 n_adc_cs_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/n_cs
  In1 adc_ch2/n_cs
  In2 adc_ch3/n_cs
  In3 adc_ch4/n_cs
  In4 adc_ch5/n_cs
  In5 adc_ch6/n_cs
  In6 adc_ch7/n_cs
  In7 adc_ch8/n_cs
  dout n_adc_cs
}
# ADC_MOSI
cell xilinx.com:ip:xlconcat:2.1 adc_mosi_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/mosi
  In1 adc_ch2/mosi
  In2 adc_ch3/mosi
  In3 adc_ch4/mosi
  In4 adc_ch5/mosi
  In5 adc_ch6/mosi
  In6 adc_ch7/mosi
  In7 adc_ch8/mosi
  dout adc_mosi
}

## Inputs
# MISO_SCK
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch1 {
  DIN_WIDTH 8
  DIN_FROM 0
  DIN_TO 0
} {
  din miso_sck
  dout dac_ch1/miso_sck
  dout adc_ch1/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch2 {
  DIN_WIDTH 8
  DIN_FROM 1
  DIN_TO 1
} {
  din miso_sck
  dout dac_ch2/miso_sck
  dout adc_ch2/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch3 {
  DIN_WIDTH 8
  DIN_FROM 2
  DIN_TO 2
} {
  din miso_sck
  dout dac_ch3/miso_sck
  dout adc_ch3/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch4 {
  DIN_WIDTH 8
  DIN_FROM 3
  DIN_TO 3
} {
  din miso_sck
  dout dac_ch4/miso_sck
  dout adc_ch4/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch5 {
  DIN_WIDTH 8
  DIN_FROM 4
  DIN_TO 4
} {
  din miso_sck
  dout dac_ch5/miso_sck
  dout adc_ch5/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch6 {
  DIN_WIDTH 8
  DIN_FROM 5
  DIN_TO 5
} {
  din miso_sck
  dout dac_ch6/miso_sck
  dout adc_ch6/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch7 {
  DIN_WIDTH 8
  DIN_FROM 6
  DIN_TO 6
} {
  din miso_sck
  dout dac_ch7/miso_sck
  dout adc_ch7/miso_sck
}
cell xilinx.com:ip:xlslice:1.0 miso_sck_ch8 {
  DIN_WIDTH 8
  DIN_FROM 7
  DIN_TO 7
} {
  din miso_sck
  dout dac_ch8/miso_sck
  dout adc_ch8/miso_sck
}
# DAC_MISO
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch1 {
  DIN_WIDTH 8
  DIN_FROM 0
  DIN_TO 0
} {
  din dac_miso
  dout dac_ch1/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch2 {
  DIN_WIDTH 8
  DIN_FROM 1
  DIN_TO 1
} {
  din dac_miso
  dout dac_ch2/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch3 {
  DIN_WIDTH 8
  DIN_FROM 2
  DIN_TO 2
} {
  din dac_miso
  dout dac_ch3/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch4 {
  DIN_WIDTH 8
  DIN_FROM 3
  DIN_TO 3
} {
  din dac_miso
  dout dac_ch4/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch5 {
  DIN_WIDTH 8
  DIN_FROM 4
  DIN_TO 4
} {
  din dac_miso
  dout dac_ch5/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch6 {
  DIN_WIDTH 8
  DIN_FROM 5
  DIN_TO 5
} {
  din dac_miso
  dout dac_ch6/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch7 {
  DIN_WIDTH 8
  DIN_FROM 6
  DIN_TO 6
} {
  din dac_miso
  dout dac_ch7/miso
}
cell xilinx.com:ip:xlslice:1.0 dac_miso_ch8 {
  DIN_WIDTH 8
  DIN_FROM 7
  DIN_TO 7
} {
  din dac_miso
  dout dac_ch8/miso
}
# ADC_MISO
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch1 {
  DIN_WIDTH 8
  DIN_FROM 0
  DIN_TO 0
} {
  din adc_miso
  dout adc_ch1/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch2 {
  DIN_WIDTH 8
  DIN_FROM 1
  DIN_TO 1
} {
  din adc_miso
  dout adc_ch2/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch3 {
  DIN_WIDTH 8
  DIN_FROM 2
  DIN_TO 2
} {
  din adc_miso
  dout adc_ch3/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch4 {
  DIN_WIDTH 8
  DIN_FROM 3
  DIN_TO 3
} {
  din adc_miso
  dout adc_ch4/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch5 {
  DIN_WIDTH 8
  DIN_FROM 4
  DIN_TO 4
} {
  din adc_miso
  dout adc_ch5/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch6 {
  DIN_WIDTH 8
  DIN_FROM 5
  DIN_TO 5
} {
  din adc_miso
  dout adc_ch6/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch7 {
  DIN_WIDTH 8
  DIN_FROM 6
  DIN_TO 6
} {
  din adc_miso
  dout adc_ch7/miso
}
cell xilinx.com:ip:xlslice:1.0 adc_miso_ch8 {
  DIN_WIDTH 8
  DIN_FROM 7
  DIN_TO 7
} {
  din adc_miso
  dout adc_ch8/miso
}


##################################################

### Status signals

## setup_done AND chain
module dac_setup_done {
  source projects/rev_d_shim/modules/8ch_and.tcl
} {
  Op1 dac_ch1/setup_done
  Op2 dac_ch2/setup_done
  Op3 dac_ch3/setup_done
  Op4 dac_ch4/setup_done
  Op5 dac_ch5/setup_done
  Op6 dac_ch6/setup_done
  Op7 dac_ch7/setup_done
  Op8 dac_ch8/setup_done
}
module adc_setup_done {
  source projects/rev_d_shim/modules/8ch_and.tcl
} {
  Op1 adc_ch1/setup_done
  Op2 adc_ch2/setup_done
  Op3 adc_ch3/setup_done
  Op4 adc_ch4/setup_done
  Op5 adc_ch5/setup_done
  Op6 adc_ch6/setup_done
  Op7 adc_ch7/setup_done
  Op8 adc_ch8/setup_done
}
cell xilinx.com:ip:util_vector_logic setup_done_and {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 dac_setup_done/Res
  Op2 adc_setup_done/Res
  Res spi_sts_sync/spi_running
}

## Concatenate error signals
cell xilinx.com:ip:xlconcat:2.1 dac_over_threshold_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/over_threshold
  In1 dac_ch2/over_threshold
  In2 dac_ch3/over_threshold
  In3 dac_ch4/over_threshold
  In4 dac_ch5/over_threshold
  In5 dac_ch6/over_threshold
  In6 dac_ch7/over_threshold
  In7 dac_ch8/over_threshold
  dout spi_sts_sync/dac_over_thresh
}
cell xilinx.com:ip:xlconcat:2.1 adc_over_threshold_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/over_threshold
  In1 adc_ch2/over_threshold
  In2 adc_ch3/over_threshold
  In3 adc_ch4/over_threshold
  In4 adc_ch5/over_threshold
  In5 adc_ch6/over_threshold
  In6 adc_ch7/over_threshold
  In7 adc_ch8/over_threshold
  dout spi_sts_sync/adc_over_thresh
}
cell xilinx.com:ip:xlconcat:2.1 dac_err_thresh_overflow_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/err_thresh_overflow
  In1 dac_ch2/err_thresh_overflow
  In2 dac_ch3/err_thresh_overflow
  In3 dac_ch4/err_thresh_overflow
  In4 dac_ch5/err_thresh_overflow
  In5 dac_ch6/err_thresh_overflow
  In6 dac_ch7/err_thresh_overflow
  In7 dac_ch8/err_thresh_overflow
  dout spi_sts_sync/dac_thresh_overflow
}
cell xilinx.com:ip:xlconcat:2.1 dac_err_thresh_underflow_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/err_thresh_underflow
  In1 dac_ch2/err_thresh_underflow
  In2 dac_ch3/err_thresh_underflow
  In3 dac_ch4/err_thresh_underflow
  In4 dac_ch5/err_thresh_underflow
  In5 dac_ch6/err_thresh_underflow
  In6 dac_ch7/err_thresh_underflow
  In7 dac_ch8/err_thresh_underflow
  dout spi_sts_sync/dac_thresh_underflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_err_thresh_overflow_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/err_thresh_overflow
  In1 adc_ch2/err_thresh_overflow
  In2 adc_ch3/err_thresh_overflow
  In3 adc_ch4/err_thresh_overflow
  In4 adc_ch5/err_thresh_overflow
  In5 adc_ch6/err_thresh_overflow
  In6 adc_ch7/err_thresh_overflow
  In7 adc_ch8/err_thresh_overflow
  dout spi_sts_sync/adc_thresh_overflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_err_thresh_underflow_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/err_thresh_underflow
  In1 adc_ch2/err_thresh_underflow
  In2 adc_ch3/err_thresh_underflow
  In3 adc_ch4/err_thresh_underflow
  In4 adc_ch5/err_thresh_underflow
  In5 adc_ch6/err_thresh_underflow
  In6 adc_ch7/err_thresh_underflow
  In7 adc_ch8/err_thresh_underflow
  dout spi_sts_sync/adc_thresh_underflow
}
cell xilinx.com:ip:xlconcat:2.1 dac_buf_underflow_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/buf_underflow
  In1 dac_ch2/buf_underflow
  In2 dac_ch3/buf_underflow
  In3 dac_ch4/buf_underflow
  In4 dac_ch5/buf_underflow
  In5 dac_ch6/buf_underflow
  In6 dac_ch7/buf_underflow
  In7 dac_ch8/buf_underflow
  dout spi_sts_sync/dac_buf_underflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_buf_overflow_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/buf_overflow
  In1 adc_ch2/buf_overflow
  In2 adc_ch3/buf_overflow
  In3 adc_ch4/buf_overflow
  In4 adc_ch5/buf_overflow
  In5 adc_ch6/buf_overflow
  In6 adc_ch7/buf_overflow
  In7 adc_ch8/buf_overflow
  dout spi_sts_sync/adc_buf_overflow
}

cell xilinx.com:ip:xlconcat:2.1 premat_dac_trig_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/premat_trig
  In1 dac_ch2/premat_trig
  In2 dac_ch3/premat_trig
  In3 dac_ch4/premat_trig
  In4 dac_ch5/premat_trig
  In5 dac_ch6/premat_trig
  In6 dac_ch7/premat_trig
  In7 dac_ch8/premat_trig
  dout spi_sts_sync/premat_dac_trig
}
cell xilinx.com:ip:xlconcat:2.1 premat_adc_trig_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/premat_trig
  In1 adc_ch2/premat_trig
  In2 adc_ch3/premat_trig
  In3 adc_ch4/premat_trig
  In4 adc_ch5/premat_trig
  In5 adc_ch6/premat_trig
  In6 adc_ch7/premat_trig
  In7 adc_ch8/premat_trig
  dout spi_sts_sync/premat_adc_trig
}
cell xilinx.com:ip:xlconcat:2.1 premat_dac_div_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/premat_div
  In1 dac_ch2/premat_div
  In2 dac_ch3/premat_div
  In3 dac_ch4/premat_div
  In4 dac_ch5/premat_div
  In5 dac_ch6/premat_div
  In6 dac_ch7/premat_div
  In7 dac_ch8/premat_div
  dout spi_sts_sync/premat_dac_div
}
cell xilinx.com:ip:xlconcat:2.1 premat_adc_div_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/premat_div
  In1 adc_ch2/premat_div
  In2 adc_ch3/premat_div
  In3 adc_ch4/premat_div
  In4 adc_ch5/premat_div
  In5 adc_ch6/premat_div
  In6 adc_ch7/premat_div
  In7 adc_ch8/premat_div
  dout spi_sts_sync/premat_adc_div
}
