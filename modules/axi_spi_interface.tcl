### DAC/ADC Command and Data FIFOs Module

# Get the board count from the calling context
set board_count [module_get_upvar board_count]

# If the board count is not 8, then error out
if {$board_count < 1 || $board_count > 8} {
  puts "Error: board_count must be between 1 and 8."
  exit 1
}

##################################################

### Ports

# System signals
create_bd_pin -dir I -type clock aclk
create_bd_pin -dir I -type reset aresetn
create_bd_pin -dir I -from 16 -to 0 command_buffer_reset
create_bd_pin -dir I -from 16 -to 0 data_buffer_reset
create_bd_pin -dir I -type clock spi_clk

# AXI interface
create_bd_intf_pin -mode slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI

# Status concatenated out
create_bd_pin -dir O -from 543 -to 0 cmd_fifo_sts
create_bd_pin -dir O -from 543 -to 0 data_fifo_sts

# DAC/ADC command and data channels
for {set i 0} {$i < $board_count} {incr i} {
  # DAC command channel
  create_bd_pin -dir O -from 31 -to 0 dac_ch${i}_cmd
  create_bd_pin -dir I dac_ch${i}_cmd_rd_en
  create_bd_pin -dir O dac_ch${i}_cmd_empty

  # DAC data channel
  create_bd_pin -dir I -from 31 -to 0 dac_ch${i}_data
  create_bd_pin -dir I dac_ch${i}_data_wr_en
  create_bd_pin -dir O dac_ch${i}_data_full

  # ADC command channel
  create_bd_pin -dir O -from 31 -to 0 adc_ch${i}_cmd
  create_bd_pin -dir I adc_ch${i}_cmd_rd_en
  create_bd_pin -dir O adc_ch${i}_cmd_empty

  # ADC data channel
  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_data
  create_bd_pin -dir I adc_ch${i}_data_wr_en
  create_bd_pin -dir O adc_ch${i}_data_full
}

# Trigger command channel
create_bd_pin -dir O -from 31 -to 0 trig_cmd
create_bd_pin -dir I trig_cmd_rd_en
create_bd_pin -dir O trig_cmd_empty

# Trigger data channel
create_bd_pin -dir I -from 31 -to 0 trig_data
create_bd_pin -dir I trig_data_wr_en
create_bd_pin -dir O trig_data_full
create_bd_pin -dir O trig_data_almost_full

# Overflow and underflow signals
create_bd_pin -dir O -from 7 -to 0 dac_cmd_buf_overflow
create_bd_pin -dir O -from 7 -to 0 dac_data_buf_underflow
create_bd_pin -dir O -from 7 -to 0 adc_cmd_buf_overflow
create_bd_pin -dir O -from 7 -to 0 adc_data_buf_underflow
create_bd_pin -dir O trig_cmd_buf_overflow
create_bd_pin -dir O trig_data_buf_underflow

## 0 and 1 constants to fill bits for unused boards
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  CONST_VAL 0
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  CONST_VAL 1
} {}

#######################################################

# DAC/ADC AXI smart connect
#  (one per board channel, one for the trigger FIFO pair)
cell xilinx.com:ip:smartconnect:1.0 board_ch_axi_intercon {
  NUM_SI 1
  NUM_MI [expr {$board_count + 1}]
} {
  aclk aclk
  S00_AXI S_AXI
  aresetn aresetn
}
# Status word padding for making 32-bit status words
cell xilinx.com:ip:xlconstant:1.1 sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH 17
} {}

## FIFO declarations
# Each FIFO has a 32-bit data width and 10-bit address width
for {set i 0} {$i < $board_count} {incr i} {

  ## DAC/ADC channel smart connect
  cell xilinx.com:ip:smartconnect:1.0 ch${i}_axi_intercon {
    NUM_SI 1
    NUM_MI 2
  } {
    aclk aclk
    S00_AXI board_ch_axi_intercon/M0${i}_AXI
    aresetn aresetn
  }

  ## DAC command FIFO
  # DAC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 dac_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 0}]
    DIN_TO [expr {2 * $i + 0}]
  } {
    din command_buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_cmd_fifo_${i}_spi_clk_rst {} {
    ext_reset_in dac_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_cmd_fifo_${i}_aclk_rst {} {
    ext_reset_in dac_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # DAC command FIFO
  cell lcb:user:fifo_async dac_cmd_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH 10
  } {
    wr_clk aclk
    wr_rst_n dac_cmd_fifo_${i}_aclk_rst/peripheral_aresetn
    rd_clk spi_clk
    rd_rst_n dac_cmd_fifo_${i}_spi_clk_rst/peripheral_aresetn
    rd_data dac_ch${i}_cmd
    rd_en dac_ch${i}_cmd_rd_en
    empty dac_ch${i}_cmd_empty
  }
  # 32-bit DAC command FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 dac_cmd_fifo_${i}_sts_word {
    NUM_PORTS 6
  } {
    In0 dac_cmd_fifo_${i}/fifo_count_wr_clk
    In1 sts_word_padding/dout
    In2 dac_cmd_fifo_${i}/full
    In3 dac_cmd_fifo_${i}/almost_full
    In4 dac_cmd_fifo_${i}/empty
    In5 dac_cmd_fifo_${i}/almost_empty
  }

  ## DAC data FIFO
  # DAC data FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 dac_data_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 0}]
    DIN_TO [expr {2 * $i + 0}]
  } {
    din data_buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_data_fifo_${i}_spi_clk_rst {} {
    ext_reset_in dac_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_data_fifo_${i}_aclk_rst {} {
    ext_reset_in dac_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # DAC data FIFO
  cell lcb:user:fifo_async dac_data_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH 10
  } {
    wr_clk spi_clk
    wr_rst_n dac_data_fifo_${i}_spi_clk_rst/peripheral_aresetn
    rd_clk aclk
    rd_rst_n dac_data_fifo_${i}_aclk_rst/peripheral_aresetn
    wr_data dac_ch${i}_data
    wr_en dac_ch${i}_data_wr_en
    full dac_ch${i}_data_full
    almost_full dac_ch${i}_data_almost_full
  }
  # 32-bit DAC data FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 dac_data_fifo_${i}_sts_word {
    NUM_PORTS 6
  } {
    In0 dac_data_fifo_${i}/fifo_count_rd_clk
    In1 sts_word_padding/dout
    In2 dac_data_fifo_${i}/full
    In3 dac_data_fifo_${i}/almost_full
    In4 dac_data_fifo_${i}/empty
    In5 dac_data_fifo_${i}/almost_empty
  }

  ## DAC FIFO AXI interface
  cell lcb:user:axi_fifo_bridge dac_fifo_${i}_axi_bridge {
    AXI_ADDR_WIDTH 32
    AXI_DATA_WIDTH 32
  } {
    aclk aclk
    aresetn aresetn
    S_AXI ch${i}_axi_intercon/M00_AXI
    fifo_wr_data dac_cmd_fifo_${i}/wr_data
    fifo_wr_en dac_cmd_fifo_${i}/wr_en
    fifo_full dac_cmd_fifo_${i}/full
    fifo_rd_data dac_data_fifo_${i}/rd_data
    fifo_rd_en dac_data_fifo_${i}/rd_en
    fifo_empty dac_data_fifo_${i}/empty
  }


  ## ADC command FIFO
  # ADC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 1}]
    DIN_TO [expr {2 * $i + 1}]
  } {
    din command_buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_cmd_fifo_${i}_spi_clk_rst {} {
    ext_reset_in adc_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_cmd_fifo_${i}_aclk_rst {} {
    ext_reset_in adc_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # ADC command FIFO
  cell lcb:user:fifo_async adc_cmd_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH 10
  } {
    wr_clk aclk
    wr_rst_n adc_cmd_fifo_${i}_aclk_rst/peripheral_aresetn
    rd_clk spi_clk
    rd_rst_n adc_cmd_fifo_${i}_spi_clk_rst/peripheral_aresetn
    rd_data adc_ch${i}_cmd
    rd_en adc_ch${i}_cmd_rd_en
    empty adc_ch${i}_cmd_empty
  }
  # 32-bit ADC command FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_cmd_fifo_${i}_sts_word {
    NUM_PORTS 6
  } {
    In0 adc_cmd_fifo_${i}/fifo_count_wr_clk
    In1 sts_word_padding/dout
    In2 adc_cmd_fifo_${i}/full
    In3 adc_cmd_fifo_${i}/almost_full
    In4 adc_cmd_fifo_${i}/empty
    In5 adc_cmd_fifo_${i}/almost_empty
  }

  ## ADC data FIFO
  # ADC data FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_data_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 1}]
    DIN_TO [expr {2 * $i + 1}]
  } {
    din data_buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_data_fifo_${i}_spi_clk_rst {} {
    ext_reset_in adc_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_data_fifo_${i}_aclk_rst {} {
    ext_reset_in adc_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # ADC data FIFO
  cell lcb:user:fifo_async adc_data_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH 10
  } {
    wr_clk spi_clk
    wr_rst_n adc_data_fifo_${i}_spi_clk_rst/peripheral_aresetn
    rd_clk aclk
    rd_rst_n adc_data_fifo_${i}_aclk_rst/peripheral_aresetn
    wr_data adc_ch${i}_data
    wr_en adc_ch${i}_data_wr_en
    full adc_ch${i}_data_full
    almost_full adc_ch${i}_data_almost_full
  }
  # 32-bit ADC data FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_data_fifo_${i}_sts_word {
    NUM_PORTS 6
  } {
    In0 adc_data_fifo_${i}/fifo_count_rd_clk
    In1 sts_word_padding/dout
    In2 adc_data_fifo_${i}/full
    In3 adc_data_fifo_${i}/almost_full
    In4 adc_data_fifo_${i}/empty
    In5 adc_data_fifo_${i}/almost_empty
  }

  ## ADC FIFO AXI interface
  cell lcb:user:axi_fifo_bridge adc_fifo_${i}_axi_bridge {
    AXI_ADDR_WIDTH 32
    AXI_DATA_WIDTH 32
  } {
    aclk aclk
    aresetn aresetn
    S_AXI ch${i}_axi_intercon/M01_AXI
    fifo_wr_data adc_cmd_fifo_${i}/wr_data
    fifo_wr_en adc_cmd_fifo_${i}/wr_en
    fifo_full adc_cmd_fifo_${i}/full
    fifo_rd_data adc_data_fifo_${i}/rd_data
    fifo_rd_en adc_data_fifo_${i}/rd_en
    fifo_empty adc_data_fifo_${i}/empty
  }
}

## Trigger command FIFO
# Trigger command FIFO resetn
cell xilinx.com:ip:xlslice:1.0 trig_cmd_fifo_rst_slice {
  DIN_WIDTH 17
  DIN_FROM 16
  DIN_TO 16
} {
  din command_buffer_reset
}
cell xilinx.com:ip:proc_sys_reset:5.0 trig_cmd_fifo_spi_clk_rst {} {
  ext_reset_in trig_cmd_fifo_rst_slice/dout
  slowest_sync_clk spi_clk
}
cell xilinx.com:ip:proc_sys_reset:5.0 trig_cmd_fifo_aclk_rst {} {
  ext_reset_in trig_cmd_fifo_rst_slice/dout
  slowest_sync_clk aclk
}
# Trigger command FIFO
cell lcb:user:fifo_async trig_cmd_fifo {
  DATA_WIDTH 32
  ADDR_WIDTH 10
} {
  wr_clk aclk
  wr_rst_n trig_cmd_fifo_aclk_rst/peripheral_aresetn
  rd_clk spi_clk
  rd_rst_n trig_cmd_fifo_spi_clk_rst/peripheral_aresetn
  rd_data trig_cmd
  rd_en trig_cmd_rd_en
  empty trig_cmd_empty
}
# 32-bit trigger command FIFO status word
cell xilinx.com:ip:xlconcat:2.1 trig_cmd_fifo_sts_word {
  NUM_PORTS 6
} {
  In0 trig_cmd_fifo/fifo_count_wr_clk
  In1 sts_word_padding/dout
  In2 trig_cmd_fifo/full
  In3 trig_cmd_fifo/almost_full
  In4 trig_cmd_fifo/empty
  In5 trig_cmd_fifo/almost_empty
}

## Trigger data FIFO
# Trigger data FIFO resetn
cell xilinx.com:ip:xlslice:1.0 trig_data_fifo_rst_slice {
  DIN_WIDTH 17
  DIN_FROM 16
  DIN_TO 16
} {
  din data_buffer_reset
}
cell xilinx.com:ip:proc_sys_reset:5.0 trig_data_fifo_spi_clk_rst {} {
  ext_reset_in trig_data_fifo_rst_slice/dout
  slowest_sync_clk spi_clk
}
cell xilinx.com:ip:proc_sys_reset:5.0 trig_data_fifo_aclk_rst {} {
  ext_reset_in trig_data_fifo_rst_slice/dout
  slowest_sync_clk aclk
}
# Trigger data FIFO
cell lcb:user:fifo_async trig_data_fifo {
  DATA_WIDTH 32
  ADDR_WIDTH 10
} {
  wr_clk spi_clk
  wr_rst_n trig_data_fifo_spi_clk_rst/peripheral_aresetn
  rd_clk aclk
  rd_rst_n trig_data_fifo_aclk_rst/peripheral_aresetn
  wr_data trig_data
  wr_en trig_data_wr_en
  full trig_data_full
  almost_full trig_data_almost_full
}
# 32-bit trigger data FIFO status word
cell xilinx.com:ip:xlconcat:2.1 trig_data_fifo_sts_word {
  NUM_PORTS 6
} {
  In0 trig_data_fifo/fifo_count_rd_clk
  In1 sts_word_padding/dout
  In2 trig_data_fifo/full
  In3 trig_data_fifo/almost_full
  In4 trig_data_fifo/empty
  In5 trig_data_fifo/almost_empty
}

## Trigger FIFO AXI interface
cell lcb:user:axi_fifo_bridge trig_fifo_axi_bridge {
  AXI_ADDR_WIDTH 32
  AXI_DATA_WIDTH 32
} {
  aclk aclk
  aresetn aresetn
  S_AXI board_ch_axi_intercon/M0${board_count}_AXI
  fifo_wr_data trig_cmd_fifo/wr_data
  fifo_wr_en trig_cmd_fifo/wr_en
  fifo_full trig_cmd_fifo/full
  fifo_rd_data trig_data_fifo/rd_data
  fifo_rd_en trig_data_fifo/rd_en
  fifo_empty trig_data_fifo/empty
}

## Status concatenation out
# 32-bit padding for unused boards
cell xilinx.com:ip:xlconstant:1.1 empty_sts_word {
  CONST_VAL 0
  CONST_WIDTH 32
} {}
# Concatenate status words from all command FIFOs
cell xilinx.com:ip:xlconcat:2.1 cmd_fifo_sts_concat {
  NUM_PORTS 17
} {
  dout cmd_fifo_sts
}
# Concatenate status words from all data FIFOs
cell xilinx.com:ip:xlconcat:2.1 data_fifo_sts_concat {
  NUM_PORTS 17
} {
  dout data_fifo_sts
}
# Wire used board command FIFO status words to the concatenation
for {set i 0} {$i < $board_count} {incr i} {
  wire cmd_fifo_sts_concat/In[expr {2 * $i + 0}] dac_cmd_fifo_${i}_sts_word/dout
  wire cmd_fifo_sts_concat/In[expr {2 * $i + 1}] adc_cmd_fifo_${i}_sts_word/dout
}
# Wire unused board command FIFO status words to the concatenation
for {set i $board_count} {$i < 8} {incr i} {
  wire cmd_fifo_sts_concat/In[expr {2 * $i + 0}] empty_sts_word/dout
  wire cmd_fifo_sts_concat/In[expr {2 * $i + 1}] empty_sts_word/dout
}
# Wire trigger command FIFO status word to the concatenation
wire cmd_fifo_sts_concat/In16 trig_cmd_fifo_sts_word/dout

# Wire used board data FIFO status words to the concatenation
for {set i 0} {$i < $board_count} {incr i} {
  wire data_fifo_sts_concat/In[expr {2 * $i + 0}] dac_data_fifo_${i}_sts_word/dout
  wire data_fifo_sts_concat/In[expr {2 * $i + 1}] adc_data_fifo_${i}_sts_word/dout
}
# Wire unused board data FIFO status words to the concatenation
for {set i $board_count} {$i < 8} {incr i} {
  wire data_fifo_sts_concat/In[expr {2 * $i + 0}] empty_sts_word/dout
  wire data_fifo_sts_concat/In[expr {2 * $i + 1}] empty_sts_word/dout
}
# Wire trigger data FIFO status word to the concatenation
wire data_fifo_sts_concat/In16 trig_data_fifo_sts_word/dout

## Overflow and underflow signals
# Concatenate DAC command FIFO overflow signals
cell xilinx.com:ip:xlconcat:2.1 dac_cmd_buf_overflow_concat {
  NUM_PORTS 8
} {
  dout dac_cmd_buf_overflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_cmd_buf_overflow_concat/In${i} dac_fifo_${i}_axi_bridge/fifo_overflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_cmd_buf_overflow_concat/In${i} const_0/dout
}
# Concatenate DAC data FIFO underflow signals
cell xilinx.com:ip:xlconcat:2.1 dac_data_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout dac_data_buf_underflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire dac_data_buf_underflow_concat/In${i} dac_fifo_${i}_axi_bridge/fifo_underflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire dac_data_buf_underflow_concat/In${i} const_0/dout
}
# Concatenate ADC command FIFO overflow signals
cell xilinx.com:ip:xlconcat:2.1 adc_cmd_buf_overflow_concat {
  NUM_PORTS 8
} {
  dout adc_cmd_buf_overflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_cmd_buf_overflow_concat/In${i} adc_fifo_${i}_axi_bridge/fifo_overflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_cmd_buf_overflow_concat/In${i} const_0/dout
}
# Concatenate ADC data FIFO underflow signals
cell xilinx.com:ip:xlconcat:2.1 adc_data_buf_underflow_concat {
  NUM_PORTS 8
} {
  dout adc_data_buf_underflow
}
for {set i 0} {$i < $board_count} {incr i} {
  wire adc_data_buf_underflow_concat/In${i} adc_fifo_${i}_axi_bridge/fifo_underflow
}
for {set i $board_count} {$i < 8} {incr i} {
  wire adc_data_buf_underflow_concat/In${i} const_0/dout
}
# Wire trigger command/data FIFO overflow and underflow signals
wire trig_cmd_buf_overflow trig_fifo_axi_bridge/fifo_overflow
wire trig_data_buf_underflow trig_fifo_axi_bridge/fifo_underflow
