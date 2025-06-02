## SPI Clock Domain Module
# This module implements the SPI clock domain containing the DAC and ADC channels.
# It synchronizes the configuration settings across clock domains

# Get the board count from the calling context
set board_count [module_get_upvar board_count]

# If the board count is not 8, then error out
if {$board_count < 1 || $board_count > 8} {
  puts "Error: board_count must be between 1 and 8."
  exit 1
}

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

## Status signals (need synchronization)
# SPI system status
create_bd_pin -dir O spi_off
# Integrator threshold status
create_bd_pin -dir O -from 7 -to 0 over_thresh
create_bd_pin -dir O -from 7 -to 0 thresh_underflow
create_bd_pin -dir O -from 7 -to 0 thresh_overflow
# DAC channel status
create_bd_pin -dir O -from 7 -to 0 bad_dac_cmd
create_bd_pin -dir O -from 7 -to 0 dac_cal_oob
create_bd_pin -dir O -from 7 -to 0 dac_val_oob
create_bd_pin -dir O -from 7 -to 0 dac_cmd_buf_underflow
create_bd_pin -dir O -from 7 -to 0 unexp_dac_trig
# ADC channel status
create_bd_pin -dir O -from 7 -to 0 bad_adc_cmd
create_bd_pin -dir O -from 7 -to 0 adc_cmd_buf_underflow
create_bd_pin -dir O -from 7 -to 0 adc_data_buf_overflow
create_bd_pin -dir O -from 7 -to 0 unexp_adc_trig

# Commands and data
for {set i 1} {$i <= $board_count} {incr i} {
  # DAC command channel
  create_bd_pin -dir I -from 31 -to 0 dac_ch${i}_cmd
  create_bd_pin -dir O dac_ch${i}_cmd_rd_en
  create_bd_pin -dir I dac_ch${i}_cmd_empty

  # ADC command channel
  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_cmd
  create_bd_pin -dir O adc_ch${i}_cmd_rd_en
  create_bd_pin -dir I adc_ch${i}_cmd_empty

  # ADC data channel
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

## SPI clock domain crossing reset (first reset)
# Negate spi_en to give a reset to the SPI clock domain
cell xilinx.com:ip:util_vector_logic n_spi_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 spi_en
}
# Create proc_sys_reset
cell xilinx.com:ip:proc_sys_reset:5.0 sync_rst {} {
  ext_reset_in n_spi_en/Res
  slowest_sync_clk spi_clk
}
## SPI system configuration synchronization
cell lcb:user:spi_cfg_sync:1.0 spi_cfg_sync {
} {
  spi_clk spi_clk
  sync_resetn sync_rst/peripheral_aresetn
  trig_lockout trig_lockout
  integ_thresh_avg integ_thresh_avg
  integ_window integ_window
  integ_en integ_en
  spi_en spi_en
}
## SPI system reset
# Negate the stabilized spi_en signal (aligned with all the incoming config signals) for the SPI-system-wide reset
cell xilinx.com:ip:util_vector_logic n_spi_en_stable {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 spi_cfg_sync/spi_en_stable
}
# Create proc_sys_reset for SPI-system-wide reset
cell xilinx.com:ip:proc_sys_reset:5.0 spi_rst {} {
  ext_reset_in n_spi_en_stable/Res
  slowest_sync_clk spi_clk
}



## SPI system status synchronization
cell lcb:user:spi_sts_sync:1.0 spi_sts_sync {
} {
  aclk aclk
  aresetn aresetn
  spi_off_stable spi_off
  over_thresh_stable over_thresh
  thresh_underflow_stable thresh_underflow
  thresh_overflow_stable thresh_overflow
  bad_dac_cmd_stable bad_dac_cmd
  dac_cal_oob_stable dac_cal_oob
  dac_val_oob_stable dac_val_oob
  dac_cmd_buf_underflow_stable dac_cmd_buf_underflow
  unexp_dac_trig_stable unexp_dac_trig
  bad_adc_cmd_stable bad_adc_cmd
  adc_cmd_buf_underflow_stable adc_cmd_buf_underflow
  adc_data_buf_overflow_stable adc_data_buf_overflow
  unexp_adc_trig_stable unexp_adc_trig
}

##################################################

### DAC and ADC Channels
for {set i 1} {$i <= $board_count} {incr i} {
  ## DAC Channel
  module spi_dac_channel dac_ch$i {
    spi_clk spi_clk
    resetn spi_rst/peripheral_aresetn
    integ_window spi_cfg_sync/integ_window_stable
    integ_thresh_avg spi_cfg_sync/integ_thresh_avg_stable
    integ_en spi_cfg_sync/integ_en_stable
    dac_cmd dac_ch${i}_cmd
    dac_cmd_rd_en dac_ch${i}_cmd_rd_en
    dac_cmd_empty dac_ch${i}_cmd_empty
  }
  ## ADC Channel
  module spi_adc_channel adc_ch$i {
    spi_clk spi_clk
    resetn spi_rst/peripheral_aresetn
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

## 0 and 1 constants to fill bits for unused boards
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  WIDTH 8
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  WIDTH 8
} {}


## Outputs

# ~DAC_CS
cell xilinx.com:ip:xlconcat:2.1 n_dac_cs_concat {
  NUM_PORTS 8
} {
  dout n_dac_cs
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire n_dac_cs_concat/In[expr {$i-1}] dac_ch$i/n_cs
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire n_dac_cs_concat/In[expr {$i-1}] const_1/dout
}

# DAC_MOSI
cell xilinx.com:ip:xlconcat:2.1 dac_mosi_concat {
  NUM_PORTS 8
} {
  dout dac_mosi
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire dac_mosi_concat/In[expr {$i-1}] dac_ch$i/mosi
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire dac_mosi_concat/In[expr {$i-1}] const_0/dout
}

# ~ADC_CS
cell xilinx.com:ip:xlconcat:2.1 n_adc_cs_concat {
  NUM_PORTS 8
} {
  dout n_adc_cs
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire n_adc_cs_concat/In[expr {$i-1}] adc_ch$i/n_cs
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire n_adc_cs_concat/In[expr {$i-1}] const_1/dout
}

# ADC_MOSI
cell xilinx.com:ip:xlconcat:2.1 adc_mosi_concat {
  NUM_PORTS 8
} {
  dout adc_mosi
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire adc_mosi_concat/In[expr {$i-1}] adc_ch$i/mosi
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire adc_mosi_concat/In[expr {$i-1}] const_0/dout
}


## Inputs
# MISO_SCK
for {set i 1} {$i <= $board_count} {incr i} {
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
for {set i 1} {$i <= $board_count} {incr i} {
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
for {set i 1} {$i <= $board_count} {incr i} {
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

## spi_off
# setup_done AND chain
for {set i 1} {$i <= $board_count} {incr i} {
  if {$i > 1} {
    # AND gate for each channel's setup_done signal
    cell xilinx.com:ip:util_vector_logic ch_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 dac_ch$i/setup_done
      Op2 adc_ch$i/setup_done
    }
    # Chain the AND gates together
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 ch_${i}_done/Res
      Op2 chs_to_[expr {$i-1}]_done/Res
    }
  } else {
    # For the first channel, no chain.
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 dac_ch$i/setup_done
      Op2 adc_ch$i/setup_done
    }
  }
}
# Negate the final setup_done signal
cell xilinx.com:ip:util_vector_logic setup_done_n {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 chs_to_${board_count}_done/Res
  Res spi_sts_sync/spi_off
}


## over_thresh
cell xilinx.com:ip:xlconcat:2.1 over_thresh_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/over_thresh
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire over_thresh_concat/In[expr {$i-1}] dac_ch$i/over_thresh
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire over_thresh_concat/In[expr {$i-1}] const_0/dout
}

## thresh_underflow
cell xilinx.com:ip:xlconcat:2.1 thresh_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/thresh_underflow
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire thresh_underflow_concat/In[expr {$i-1}] dac_ch$i/thresh_underflow
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire thresh_underflow_concat/In[expr {$i-1}] const_0/dout
}

## thresh_overflow
cell xilinx.com:ip:xlconcat:2.1 thresh_overflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/thresh_overflow
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire thresh_overflow_concat/In[expr {$i-1}] dac_ch$i/thresh_overflow
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire thresh_overflow_concat/In[expr {$i-1}] const_0/dout
}

## bad_dac_cmd
cell xilinx.com:ip:xlconcat:2.1 bad_dac_cmd_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/bad_dac_cmd
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire bad_dac_cmd_concat/In[expr {$i-1}] dac_ch$i/bad_cmd
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire bad_dac_cmd_concat/In[expr {$i-1}] const_0/dout
}

## dac_cal_oob
cell xilinx.com:ip:xlconcat:2.1 dac_cal_oob_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_cal_oob
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire dac_cal_oob_concat/In[expr {$i-1}] dac_ch$i/cal_oob
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire dac_cal_oob_concat/In[expr {$i-1}] const_0/dout
}

## dac_val_oob
cell xilinx.com:ip:xlconcat:2.1 dac_val_oob_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_val_oob
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire dac_val_oob_concat/In[expr {$i-1}] dac_ch$i/dac_val_oob
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire dac_val_oob_concat/In[expr {$i-1}] const_0/dout
}

## dac_cmd_buf_underflow
cell xilinx.com:ip:xlconcat:2.1 dac_cmd_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_cmd_buf_underflow
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire dac_cmd_buf_underflow_concat/In[expr {$i-1}] dac_ch$i/cmd_buf_underflow
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire dac_cmd_buf_underflow_concat/In[expr {$i-1}] const_0/dout
}

## unexp_dac_trig
cell xilinx.com:ip:xlconcat:2.1 unexp_dac_trig_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/unexp_dac_trig
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire unexp_dac_trig_concat/In[expr {$i-1}] dac_ch$i/unexp_trig
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire unexp_dac_trig_concat/In[expr {$i-1}] const_0/dout
}

## bad_adc_cmd
cell xilinx.com:ip:xlconcat:2.1 bad_adc_cmd_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/bad_adc_cmd
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire bad_adc_cmd_concat/In[expr {$i-1}] adc_ch$i/bad_cmd
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire bad_adc_cmd_concat/In[expr {$i-1}] const_0/dout
}

## adc_cmd_buf_underflow
cell xilinx.com:ip:xlconcat:2.1 adc_cmd_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/adc_cmd_buf_underflow
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire adc_cmd_buf_underflow_concat/In[expr {$i-1}] adc_ch$i/cmd_buf_underflow
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire adc_cmd_buf_underflow_concat/In[expr {$i-1}] const_0/dout
}

## adc_data_buf_overflow
cell xilinx.com:ip:xlconcat:2.1 adc_data_buf_overflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/adc_data_buf_overflow
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire adc_data_buf_overflow_concat/In[expr {$i-1}] adc_ch$i/data_buf_overflow
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire adc_data_buf_overflow_concat/In[expr {$i-1}] const_0/dout
}

## unexp_adc_trig
cell xilinx.com:ip:xlconcat:2.1 unexp_adc_trig_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/unexp_adc_trig
}
for {set i 1} {$i <= $board_count} {incr i} {
  wire unexp_adc_trig_concat/In[expr {$i-1}] adc_ch$i/unexp_trig
}
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire unexp_adc_trig_concat/In[expr {$i-1}] const_0/dout
}
