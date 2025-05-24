## SPI Clock Domain Module
# This module implements the SPI clock domain containing the DAC and ADC channels.
# It synchronizes the configuration settings across clock domains

##################################################

### Ports

# System ports
create_bd_pin -dir I -type clock aclk
create_bd_pin -dir I -type reset aresetn
create_bd_pin -dir I -type clock spi_clk

# Configuration signals (need synchronization)
create_bd_pin -dir I -from 31 -to 0 trig_lockout
create_bd_pin -dir I -from 14 -to 0 integ_thresh_avg
create_bd_pin -dir I -from 31 -to 0 integ_window
create_bd_pin -dir I integ_en
create_bd_pin -dir I spi_en

# Status signals (need synchronization)
create_bd_pin -dir O -from 7 -to 0 spi_off
create_bd_pin -dir O -from 7 -to 0 dac_over_thresh
create_bd_pin -dir O -from 7 -to 0 adc_over_thresh
create_bd_pin -dir O -from 7 -to 0 dac_thresh_underflow
create_bd_pin -dir O -from 7 -to 0 dac_thresh_overflow
create_bd_pin -dir O -from 7 -to 0 adc_thresh_underflow
create_bd_pin -dir O -from 7 -to 0 adc_thresh_overflow
create_bd_pin -dir O -from 7 -to 0 dac_buf_underflow
create_bd_pin -dir O -from 7 -to 0 adc_buf_underflow
create_bd_pin -dir O -from 7 -to 0 unexp_dac_trig
create_bd_pin -dir O -from 7 -to 0 unexp_adc_trig

# Commands and data
for {set i 1} {$i <= 8} {incr i} {
  create_bd_pin -dir I -from 31 -to 0 dac_ch${i}_cmd
  create_bd_pin -dir O dac_ch${i}_cmd_rd_en
  create_bd_pin -dir I dac_ch${i}_cmd_empty

  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_cmd
  create_bd_pin -dir O adc_ch${i}_cmd_rd_en
  create_bd_pin -dir I adc_ch${i}_cmd_empty
  create_bd_pin -dir O -from 31 -to 0 adc_ch${i}_data
  create_bd_pin -dir O adc_ch${i}_data_wr_en
  create_bd_pin -dir I adc_ch${i}_data_full
}

# Trigger
create_bd_pin -dir I trigger_gated

# SPI interface signals (out)
create_bd_pin -dir O ldac
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

## SPI clock domain reset 
# Negate spi_en to give a reset to the SPI clock domain
cell xilinx.com:ip:util_vector_logic n_spi_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 spi_en
}
# Create proc_sys_reset
cell xilinx.com:ip:proc_sys_reset:5.0 spi_rst {} {
  ext_reset_in n_spi_en/Res
  slowest_sync_clk spi_clk
}

## SPI system configuration synchronization
cell lcb:user:spi_cfg_sync:1.0 spi_cfg_sync {
} {
  spi_clk spi_clk
  spi_resetn spi_rst/peripheral_aresetn
  trig_lockout trig_lockout
  integ_thresh_avg integ_thresh_avg
  integ_window integ_window
  integ_en integ_en
  spi_en spi_en
}

## SPI system status synchronization
cell lcb:user:spi_sts_sync:1.0 spi_sts_sync {
} {
  aclk aclk
  aresetn aresetn
  spi_off_stable spi_off
  dac_over_thresh_stable dac_over_thresh
  adc_over_thresh_stable adc_over_thresh
  dac_thresh_underflow_stable dac_thresh_underflow
  dac_thresh_overflow_stable dac_thresh_overflow
  adc_thresh_underflow_stable adc_thresh_underflow
  adc_thresh_overflow_stable adc_thresh_overflow
  dac_buf_underflow_stable dac_buf_underflow
  adc_buf_overflow_stable adc_buf_underflow
  unexp_dac_trig_stable unexp_dac_trig
  unexp_adc_trig_stable unexp_adc_trig
}

##################################################

### DAC and ADC Channels
for {set i 1} {$i <= 8} {incr i} {
  ## DAC Channel
  module dac_channel dac_ch$i {
    spi_clk spi_clk
    aresetn spi_rst/peripheral_aresetn
    integ_window spi_cfg_sync/integ_window_stable
    integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
    integ_en spi_cfg_sync/integ_en_stable
    spi_en spi_cfg_sync/spi_en_stable
    dac_cmd dac_ch${i}_cmd
    dac_cmd_rd_en dac_ch${i}_cmd_rd_en
    dac_cmd_empty dac_ch${i}_cmd_empty
  }
  ## ADC Channel
  module adc_channel adc_ch$i {
    spi_clk spi_clk
    aresetn spi_rst/peripheral_aresetn
    integ_window spi_cfg_sync/integ_window_stable
    integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
    integ_en spi_cfg_sync/integ_en_stable
    spi_en spi_cfg_sync/spi_en_stable
    adc_cmd adc_ch${i}_cmd
    adc_cmd_rd_en adc_ch${i}_cmd_rd_en
    adc_cmd_empty adc_ch${i}_cmd_empty
    adc_data adc_ch${i}_data
    adc_data_wr_en adc_ch${i}_data_wr_en
    adc_data_full adc_ch${i}_data_full
  }
}

##################################################

### SPI signals

## Outputs
# ~DAC_CS
cell xilinx.com:ip:xlconcat:2.1 n_dac_cs_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/n_cs}]
  dout n_dac_cs
}
# DAC_MOSI
cell xilinx.com:ip:xlconcat:2.1 dac_mosi_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/mosi}]
  dout dac_mosi
}
# ~ADC_CS
cell xilinx.com:ip:xlconcat:2.1 n_adc_cs_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/n_cs}]
  dout n_adc_cs
}
# ADC_MOSI
cell xilinx.com:ip:xlconcat:2.1 adc_mosi_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/mosi}]
  dout adc_mosi
}


## Inputs
# MISO_SCK
for {set i 1} {$i <= 8} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 miso_sck_ch$i {
    DIN_WIDTH 8
    DIN_FROM [expr {$i-1}]
    DIN_TO [expr {$i-1}]
  } {
    din miso_sck
    dout dac_ch$i/miso_sck
    dout adc_ch$i/miso_sck
  }
}
# DAC_MISO
for {set i 1} {$i <= 8} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 dac_miso_ch$i {
    DIN_WIDTH 8
    DIN_FROM [expr {$i-1}]
    DIN_TO [expr {$i-1}]
  } {
    din dac_miso
    dout dac_ch$i/miso
  }
}
# ADC_MISO
for {set i 1} {$i <= 8} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 adc_miso_ch$i {
    DIN_WIDTH 8
    DIN_FROM [expr {$i-1}]
    DIN_TO [expr {$i-1}]
  } {
    din adc_miso
    dout adc_ch$i/miso
  }
}


##################################################

### Status signals

## setup_done AND chain
module 8ch_and dac_setup_done {
  [loop_pins i {1 2 3 4 5 6 7 8} {Op$i} {dac_ch$i/setup_done}]
}
module 8ch_and adc_setup_done {
  [loop_pins i {1 2 3 4 5 6 7 8} {Op$i} {adc_ch$i/setup_done}]
}
cell xilinx.com:ip:util_vector_logic setup_done_and {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 dac_setup_done/Res
  Op2 adc_setup_done/Res
}
cell xilinx.com:ip:util_vector_logic setup_done_n {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 setup_done_and/Res
  Res spi_sts_sync/spi_off
}


## Concatenate error signals
cell xilinx.com:ip:xlconcat:2.1 dac_over_threshold_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/over_threshold}]
  dout spi_sts_sync/dac_over_thresh
}
cell xilinx.com:ip:xlconcat:2.1 adc_over_threshold_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/over_threshold}]
  dout spi_sts_sync/adc_over_thresh
}
cell xilinx.com:ip:xlconcat:2.1 dac_err_thresh_overflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/err_thresh_overflow}]
  dout spi_sts_sync/dac_thresh_overflow
}
cell xilinx.com:ip:xlconcat:2.1 dac_err_thresh_underflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/err_thresh_underflow}]
  dout spi_sts_sync/dac_thresh_underflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_err_thresh_overflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/err_thresh_overflow}]
  dout spi_sts_sync/adc_thresh_overflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_err_thresh_underflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/err_thresh_underflow}]
  dout spi_sts_sync/adc_thresh_underflow
}
cell xilinx.com:ip:xlconcat:2.1 dac_buf_underflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/buf_underflow}]
  dout spi_sts_sync/dac_buf_underflow
}
cell xilinx.com:ip:xlconcat:2.1 adc_buf_overflow_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/buf_overflow}]
  dout spi_sts_sync/adc_buf_overflow
}
cell xilinx.com:ip:xlconcat:2.1 unexp_dac_trig_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {dac_ch$i/unexp_trig}]
  dout spi_sts_sync/unexp_dac_trig
}
cell xilinx.com:ip:xlconcat:2.1 unexp_adc_trig_concat {
  NUM_PORTS 8
} {
  [loop_pins i {1 2 3 4 5 6 7 8} {In[expr {$i-1}]} {adc_ch$i/unexp_trig}]
  dout spi_sts_sync/unexp_adc_trig
}
