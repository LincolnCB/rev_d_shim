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
# Trigger channel status
create_bd_pin -dir O bad_trig_cmd
create_bd_pin -dir O trig_data_buf_overflow
# DAC channel status
create_bd_pin -dir O -from 7 -to 0 dac_boot_fail
create_bd_pin -dir O -from 7 -to 0 bad_dac_cmd
create_bd_pin -dir O -from 7 -to 0 dac_cal_oob
create_bd_pin -dir O -from 7 -to 0 dac_val_oob
create_bd_pin -dir O -from 7 -to 0 dac_cmd_buf_underflow
create_bd_pin -dir O -from 7 -to 0 unexp_dac_trig
# ADC channel status
create_bd_pin -dir O -from 7 -to 0 adc_boot_fail
create_bd_pin -dir O -from 7 -to 0 bad_adc_cmd
create_bd_pin -dir O -from 7 -to 0 adc_cmd_buf_underflow
create_bd_pin -dir O -from 7 -to 0 adc_data_buf_overflow
create_bd_pin -dir O -from 7 -to 0 unexp_adc_trig

# Commands and data
for {set i 0} {$i < $board_count} {incr i} {
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
# Trigger command channel
create_bd_pin -dir I -from 31 -to 0 trig_cmd
create_bd_pin -dir O trig_cmd_rd_en
create_bd_pin -dir I trig_cmd_empty
# Trigger data channel
create_bd_pin -dir O -from 31 -to 0 trig_data
create_bd_pin -dir O trig_data_wr_en
create_bd_pin -dir I trig_data_full
create_bd_pin -dir I trig_data_almost_full

# External trigger
create_bd_pin -dir I ext_trig

# Block buffers on the SPI side (until HW Manager is ready)
create_bd_pin -dir I block_buffers

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

## 0 and 1 constants to fill bits for unused boards
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  CONST_VAL 0
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  CONST_VAL 1
} {}

##################################################

### Clock domain crossings

## SPI clock domain crossing reset (first reset)
# Create proc_sys_reset for the synchronization reset
cell xilinx.com:ip:proc_sys_reset:5.0 sync_rst_core {
  C_AUX_RESET_HIGH.VALUE_SRC USER
  C_AUX_RESET_HIGH 0
} {
  aux_reset_in spi_en
  slowest_sync_clk spi_clk
}
## SPI system configuration synchronization
cell lcb:user:shim_spi_cfg_sync spi_cfg_sync {} {
  aclk aclk
  aresetn aresetn
  spi_clk spi_clk
  spi_resetn sync_rst_core/peripheral_aresetn
  integ_thresh_avg integ_thresh_avg
  integ_window integ_window
  integ_en integ_en
  spi_en spi_en
  block_buffers block_buffers
}
## SPI system reset
# Create proc_sys_reset for SPI-system-wide reset
cell xilinx.com:ip:proc_sys_reset:5.0 spi_rst_core {
  C_AUX_RESET_HIGH.VALUE_SRC USER
  C_AUX_RESET_HIGH 0
} {
  aux_reset_in spi_cfg_sync/spi_en_sync
  slowest_sync_clk spi_clk
}



## SPI system status synchronization
cell lcb:user:shim_spi_sts_sync spi_sts_sync {} {
  aclk aclk
  aresetn aresetn
  spi_off_sync spi_off
  over_thresh_sync over_thresh
  thresh_underflow_sync thresh_underflow
  thresh_overflow_sync thresh_overflow
  bad_trig_cmd_sync bad_trig_cmd
  trig_data_buf_overflow_sync trig_data_buf_overflow
  dac_boot_fail_sync dac_boot_fail
  bad_dac_cmd_sync bad_dac_cmd
  dac_cal_oob_sync dac_cal_oob
  dac_val_oob_sync dac_val_oob
  dac_cmd_buf_underflow_sync dac_cmd_buf_underflow
  unexp_dac_trig_sync unexp_dac_trig
  adc_boot_fail_sync adc_boot_fail
  bad_adc_cmd_sync bad_adc_cmd
  adc_cmd_buf_underflow_sync adc_cmd_buf_underflow
  adc_data_buf_overflow_sync adc_data_buf_overflow
  unexp_adc_trig_sync unexp_adc_trig
}

##################################################

### Trigger core
## Block the command and data buffers if needed (OR block_buffers_sync with cmd_buf_empty and data_buf_full)
cell xilinx.com:ip:util_vector_logic trig_cmd_empty_blocked {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 trig_cmd_empty
  Op2 spi_cfg_sync/block_buffers_sync
}
cell xilinx.com:ip:util_vector_logic trig_data_full_blocked {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 trig_data_full
  Op2 spi_cfg_sync/block_buffers_sync
}
## Trigger core
cell lcb:user:shim_trigger_core trig_core {
  TRIGGER_LOCKOUT_DEFAULT 5000
} {
  clk spi_clk
  resetn spi_rst_core/peripheral_aresetn
  cmd_word_rd_en trig_cmd_rd_en
  cmd_word trig_cmd
  cmd_buf_empty trig_cmd_empty_blocked/Res
  data_word_wr_en trig_data_wr_en
  data_word trig_data
  data_buf_full trig_data_full_blocked/Res
  data_buf_almost_full trig_data_almost_full
  ext_trig ext_trig
  bad_cmd spi_sts_sync/bad_trig_cmd
  data_buf_overflow spi_sts_sync/trig_data_buf_overflow
} 

##################################################

### DAC and ADC Channels
for {set i 0} {$i < $board_count} {incr i} {
  ## DAC Channel
  module spi_dac_channel dac_ch$i {
    spi_clk spi_clk
    resetn spi_rst_core/peripheral_aresetn
    integ_window spi_cfg_sync/integ_window_sync
    integ_thresh_avg spi_cfg_sync/integ_thresh_avg_sync
    integ_en spi_cfg_sync/integ_en_sync
    dac_cmd dac_ch${i}_cmd
    dac_cmd_rd_en dac_ch${i}_cmd_rd_en
    dac_cmd_empty dac_ch${i}_cmd_empty
    block_buffers spi_cfg_sync/block_buffers_sync
    trigger trig_core/trig_out
  }
  ## ADC Channel
  module spi_adc_channel adc_ch$i {
    spi_clk spi_clk
    resetn spi_rst_core/peripheral_aresetn
    adc_cmd adc_ch${i}_cmd
    adc_cmd_rd_en adc_ch${i}_cmd_rd_en
    adc_cmd_empty adc_ch${i}_cmd_empty
    adc_data adc_ch${i}_data
    adc_data_wr_en adc_ch${i}_data_wr_en
    adc_data_full adc_ch${i}_data_full
    block_buffers spi_cfg_sync/block_buffers_sync
    trigger trig_core/trig_out
  }
}
# Waiting for trigger signals
cell xilinx.com:ip:xlconcat:2.1 dac_waiting_for_trig_concat {
  NUM_PORTS 8
} {
  dout trig_core/dac_waiting_for_trig
}
cell xilinx.com:ip:xlconcat:2.1 adc_waiting_for_trig_concat {
  NUM_PORTS 8
} {
  dout trig_core/adc_waiting_for_trig
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_waiting_for_trig_concat/In${i} dac_ch${i}/waiting_for_trig
  wire adc_waiting_for_trig_concat/In${i} adc_ch${i}/waiting_for_trig
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_waiting_for_trig_concat/In${i} const_1/dout
  wire adc_waiting_for_trig_concat/In${i} const_1/dout
}


##################################################

### SPI signals

## Outputs

# LDAC
## Cascade OR for LDAC
for {set i 0} {$i < $board_count} {incr i} {
  if {$i > 0} {
    # Chain the OR gates together
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_ldac {
      C_SIZE 1
      C_OPERATION or
    } {
      Op1 dac_ch${i}/ldac
      Op2 chs_to_[expr {$i - 1}]_ldac/Res
    }
  } else {
    # For the first channel, no chain.
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_ldac {
      C_SIZE 1
      C_OPERATION or
    } {
      Op1 dac_ch${i}/ldac
      Op2 const_0/dout
    }
  }
}
# Connect the final OR output to the ldac output pin
wire chs_to_[expr {$board_count - 1}]_ldac/Res ldac
# Also connect to the ldac_shared of the dac channels
for {set i 0} {$i < $board_count} {incr i} {
  wire chs_to_[expr {$board_count - 1}]_ldac/Res dac_ch${i}/ldac_shared
}

# ~DAC_CS
cell xilinx.com:ip:xlconcat:2.1 n_dac_cs_concat {
  NUM_PORTS 8
} {
  dout n_dac_cs
}
for {set i 0} {$i < $board_count} {incr i} {
  wire n_dac_cs_concat/In${i} dac_ch${i}/n_cs
}
for {set i $board_count} {$i < 8} {incr i} {
  wire n_dac_cs_concat/In${i} const_1/dout
}

# DAC_MOSI
cell xilinx.com:ip:xlconcat:2.1 dac_mosi_concat {
  NUM_PORTS 8
} {
  dout dac_mosi
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_mosi_concat/In${i} dac_ch${i}/mosi
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_mosi_concat/In${i} const_0/dout
}

# ~ADC_CS
cell xilinx.com:ip:xlconcat:2.1 n_adc_cs_concat {
  NUM_PORTS 8
} {
  dout n_adc_cs
}
for {set i 0} {$i < $board_count} {incr i} {
  wire n_adc_cs_concat/In${i} adc_ch${i}/n_cs
}
for {set i $board_count} {$i < 8} {incr i} {
  wire n_adc_cs_concat/In${i} const_1/dout
}

# ADC_MOSI
cell xilinx.com:ip:xlconcat:2.1 adc_mosi_concat {
  NUM_PORTS 8
} {
  dout adc_mosi
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_mosi_concat/In${i} adc_ch${i}/mosi
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_mosi_concat/In${i} const_0/dout
}

## Inputs
# MISO_SCK
for {set i 0} {$i < $board_count} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 miso_sck_ch$i {
    DIN_WIDTH 8
    DIN_FROM ${i}
    DIN_TO ${i}
  } {
    din miso_sck
    dout dac_ch${i}/miso_sck
    dout adc_ch${i}/miso_sck
  }
}
# DAC_MISO
for {set i 0} {$i < $board_count} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 dac_miso_ch$i {
    DIN_WIDTH 8
    DIN_FROM ${i}
    DIN_TO ${i}
  } {
    din dac_miso
    dout dac_ch${i}/miso
  }
}
# ADC_MISO
for {set i 0} {$i < $board_count} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 adc_miso_ch$i {
    DIN_WIDTH 8
    DIN_FROM ${i}
    DIN_TO ${i}
  } {
    din adc_miso
    dout adc_ch${i}/miso
  }
}


##################################################

### Status signals

## spi_off
# setup_done AND chain
for {set i 0} {$i < $board_count} {incr i} {
  if {$i > 0} {
    # AND gate for each channel's setup_done signal
    cell xilinx.com:ip:util_vector_logic ch_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 dac_ch${i}/setup_done
      Op2 adc_ch${i}/setup_done
    }
    # Chain the AND gates together
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 ch_${i}_done/Res
      Op2 chs_to_[expr {$i - 1}]_done/Res
    }
  } else {
    # For the first channel, no chain.
    cell xilinx.com:ip:util_vector_logic chs_to_${i}_done {
      C_SIZE 1
      C_OPERATION and
    } {
      Op1 dac_ch${i}/setup_done
      Op2 adc_ch${i}/setup_done
    }
  }
}
# Negate the final setup_done signal
cell xilinx.com:ip:util_vector_logic setup_done_n {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 chs_to_[expr {$board_count - 1}]_done/Res
  Res spi_sts_sync/spi_off
}


## over_thresh
cell xilinx.com:ip:xlconcat:2.1 over_thresh_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/over_thresh
}
for {set i 0} {$i < $board_count} {incr i} {
  wire over_thresh_concat/In${i} dac_ch${i}/over_thresh
}
for {set i $board_count} {$i < 8} {incr i} {
  wire over_thresh_concat/In${i} const_0/dout
}

## thresh_underflow
cell xilinx.com:ip:xlconcat:2.1 thresh_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/thresh_underflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire thresh_underflow_concat/In${i} dac_ch${i}/thresh_underflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire thresh_underflow_concat/In${i} const_0/dout
}

## thresh_overflow
cell xilinx.com:ip:xlconcat:2.1 thresh_overflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/thresh_overflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire thresh_overflow_concat/In${i} dac_ch${i}/thresh_overflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire thresh_overflow_concat/In${i} const_0/dout
}

## dac_boot_fail
cell xilinx.com:ip:xlconcat:2.1 dac_boot_fail_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_boot_fail
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_boot_fail_concat/In${i} dac_ch${i}/boot_fail
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_boot_fail_concat/In${i} const_0/dout
}

## bad_dac_cmd
cell xilinx.com:ip:xlconcat:2.1 bad_dac_cmd_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/bad_dac_cmd
}
for {set i 0} {$i < $board_count} {incr i} {
  wire bad_dac_cmd_concat/In${i} dac_ch${i}/bad_cmd
}
for {set i $board_count} {$i < 8} {incr i} {
  wire bad_dac_cmd_concat/In${i} const_0/dout
}

## dac_cal_oob
cell xilinx.com:ip:xlconcat:2.1 dac_cal_oob_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_cal_oob
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_cal_oob_concat/In${i} dac_ch${i}/cal_oob
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_cal_oob_concat/In${i} const_0/dout
}

## dac_val_oob
cell xilinx.com:ip:xlconcat:2.1 dac_val_oob_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_val_oob
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_val_oob_concat/In${i} dac_ch${i}/dac_val_oob
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_val_oob_concat/In${i} const_0/dout
}

## dac_cmd_buf_underflow
cell xilinx.com:ip:xlconcat:2.1 dac_cmd_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/dac_cmd_buf_underflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_cmd_buf_underflow_concat/In${i} dac_ch${i}/cmd_buf_underflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_cmd_buf_underflow_concat/In${i} const_0/dout
}

## unexp_dac_trig
cell xilinx.com:ip:xlconcat:2.1 unexp_dac_trig_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/unexp_dac_trig
}
for {set i 0} {$i < $board_count} {incr i} {
  wire unexp_dac_trig_concat/In${i} dac_ch${i}/unexp_trig
}
for {set i $board_count} {$i < 8} {incr i} {
  wire unexp_dac_trig_concat/In${i} const_0/dout
}

## adc_boot_fail
cell xilinx.com:ip:xlconcat:2.1 adc_boot_fail_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/adc_boot_fail
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_boot_fail_concat/In${i} adc_ch${i}/boot_fail
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_boot_fail_concat/In${i} const_0/dout
}

## bad_adc_cmd
cell xilinx.com:ip:xlconcat:2.1 bad_adc_cmd_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/bad_adc_cmd
}
for {set i 0} {$i < $board_count} {incr i} {
  wire bad_adc_cmd_concat/In${i} adc_ch${i}/bad_cmd
}
for {set i $board_count} {$i < 8} {incr i} {
  wire bad_adc_cmd_concat/In${i} const_0/dout
}

## adc_cmd_buf_underflow
cell xilinx.com:ip:xlconcat:2.1 adc_cmd_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/adc_cmd_buf_underflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_cmd_buf_underflow_concat/In${i} adc_ch${i}/cmd_buf_underflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_cmd_buf_underflow_concat/In${i} const_0/dout
}

## adc_data_buf_overflow
cell xilinx.com:ip:xlconcat:2.1 adc_data_buf_overflow_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/adc_data_buf_overflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_data_buf_overflow_concat/In${i} adc_ch${i}/data_buf_overflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_data_buf_overflow_concat/In${i} const_0/dout
}

## unexp_adc_trig
cell xilinx.com:ip:xlconcat:2.1 unexp_adc_trig_concat {
  NUM_PORTS 8
} {
  dout spi_sts_sync/unexp_adc_trig
}
for {set i 0} {$i < $board_count} {incr i} {
  wire unexp_adc_trig_concat/In${i} adc_ch${i}/unexp_trig
}
for {set i $board_count} {$i < 8} {incr i} {
  wire unexp_adc_trig_concat/In${i} const_0/dout
}
