create_bd_pin -dir I -type clock clk
create_bd_pin -dir I -type clock sck
create_bd_pin -dir I -type reset rst

create_bd_pin -dir I -from 31 -to 0 trig_lockout
create_bd_pin -dir I -from 15 -to 0 cal_offset
create_bd_pin -dir I -from 15 -to 0 dac_divider
create_bd_pin -dir I -from 15 -to 0 integ_thresh_avg
create_bd_pin -dir I -from 31 -to 0 integ_window
create_bd_pin -dir I integ_en
create_bd_pin -dir I spi_en

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


## setup_done AND chain
module dac_setup_done {
  source projects/rev_d_shim/modules/8ch_and.tcl
} {
  Op1 dac_ch1/setup_done
  Op2 dac_ch1/setup_done
  Op3 dac_ch1/setup_done
  Op4 dac_ch1/setup_done
  Op5 dac_ch1/setup_done
  Op6 dac_ch1/setup_done
  Op7 dac_ch1/setup_done
  Op8 dac_ch1/setup_done
}
module adc_setup_done {
  source projects/rev_d_shim/modules/8ch_and.tcl
} {
  Op1 adc_ch1/setup_done
  Op2 adc_ch1/setup_done
  Op3 adc_ch1/setup_done
  Op4 adc_ch1/setup_done
  Op5 adc_ch1/setup_done
  Op6 adc_ch1/setup_done
  Op7 adc_ch1/setup_done
  Op8 adc_ch1/setup_done
}
cell xilinx.com:ip:util_vector_logic setup_done_and {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 dac_setup_done/Res
  Op2 adc_setup_done/Res
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
}
cell xilinx.com:ip:xlconcat:2.1 dac_buf_underflow_concat {
  NUM_PORTS 8
} {
  In0 dac_ch1/dac_buf_underflow
  In1 dac_ch2/dac_buf_underflow
  In2 dac_ch3/dac_buf_underflow
  In3 dac_ch4/dac_buf_underflow
  In4 dac_ch5/dac_buf_underflow
  In5 dac_ch6/dac_buf_underflow
  In6 dac_ch7/dac_buf_underflow
  In7 dac_ch8/dac_buf_underflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_buf_overflow_concat {
  NUM_PORTS 8
} {
  In0 adc_ch1/adc_buf_overflow
  In1 adc_ch2/adc_buf_overflow
  In2 adc_ch3/adc_buf_overflow
  In3 adc_ch4/adc_buf_overflow
  In4 adc_ch5/adc_buf_overflow
  In5 adc_ch6/adc_buf_overflow
  In6 adc_ch7/adc_buf_overflow
  In7 adc_ch8/adc_buf_overflow
}
