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
create_bd_pin -dir I -from 23 -to 0 buffer_reset
create_bd_pin -dir I -type clock spi_clk

# AXI interface
create_bd_intf_pin -mode slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI

# Status concatenated out
create_bd_pin -dir O -from 767 -to 0 fifo_sts

# DAC/ADC command and data channels
for {set i 1} {$i <= $board_count} {incr i} {
  # DAC command channel
  create_bd_pin -dir O -from 31 -to 0 dac_ch${i}_cmd
  create_bd_pin -dir I dac_ch${i}_cmd_rd_en
  create_bd_pin -dir O dac_ch${i}_cmd_empty

  # ADC command channel
  create_bd_pin -dir O -from 31 -to 0 adc_ch${i}_cmd
  create_bd_pin -dir I adc_ch${i}_cmd_rd_en
  create_bd_pin -dir O adc_ch${i}_cmd_empty

  # ADC data channel
  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_data
  create_bd_pin -dir I adc_ch${i}_data_wr_en
  create_bd_pin -dir O adc_ch${i}_data_full
}

#######################################################

# DAC/ADC AXI smart connect, per board (1 master, board_count slaves)
cell xilinx.com:ip:smartconnect:1.0 board_ch_axi_intercon {
  NUM_SI 1
  NUM_MI $board_count
} {
  aclk aclk
  S00_AXI S_AXI
  aresetn aresetn
}
# Status word padding for making 32-bit status words
cell xilinx.com:ip:xlconstant:1.1 sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH 19
} {}

## FIFO declarations
# Each FIFO has a 32-bit data width and 10-bit address width
for {set i 1} {$i <= $board_count} {incr i} {


  ## DAC/ADC channel smart connect
  cell xilinx.com:ip:smartconnect:1.0 ch${i}_axi_intercon {
    NUM_SI 1
    NUM_MI 3
  } {
    aclk aclk
    S00_AXI board_ch_axi_intercon/M0[expr {$i-1}]_AXI
    aresetn aresetn
  }
  

  ## DAC command FIFO
  # DAC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 dac_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 24
    DIN_FROM [expr {3*($i-1)+0}]
    DIN_TO [expr {3*($i-1)+0}]
  } {
    din buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_cmd_fifo_${i}_spi_clk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in dac_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 dac_cmd_fifo_${i}_aclk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in dac_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # DAC command FIFO
  cell lcb:user:fifo_async_count dac_cmd_fifo_$i {
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
    NUM_PORTS 4
  } {
    In0 dac_cmd_fifo_$i/fifo_count_wr_clk
    In1 sts_word_padding/dout
    In2 dac_cmd_fifo_$i/full
    In3 dac_cmd_fifo_$i/almost_full
  }
  # DAC command FIFO AXI interface
  cell lcb:user:axi_fifo_bridge:1.0 dac_cmd_fifo_${i}_axi_bridge {
    AXI_ADDR_WIDTH 32
    AXI_DATA_WIDTH 32
    ENABLE_READ 0
  } {
    aclk aclk
    aresetn aresetn
    S_AXI ch${i}_axi_intercon/M00_AXI
    fifo_wr_data dac_cmd_fifo_$i/wr_data
    fifo_wr_en dac_cmd_fifo_$i/wr_en
    fifo_full dac_cmd_fifo_$i/full
  }


  ## ADC command FIFO
  # ADC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 24
    DIN_FROM [expr {3*($i-1)+1}]
    DIN_TO [expr {3*($i-1)+1}]
  } {
    din buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_cmd_fifo_${i}_spi_clk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in adc_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_cmd_fifo_${i}_aclk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in adc_cmd_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # ADC command FIFO
  cell lcb:user:fifo_async_count adc_cmd_fifo_$i {
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
    NUM_PORTS 4
  } {
    In0 adc_cmd_fifo_$i/fifo_count_wr_clk
    In1 sts_word_padding/dout
    In2 adc_cmd_fifo_$i/full
    In3 adc_cmd_fifo_$i/almost_full
  }
  # ADC command FIFO AXI interface
  cell lcb:user:axi_fifo_bridge:1.0 adc_cmd_fifo_${i}_axi_bridge {
    AXI_ADDR_WIDTH 32
    AXI_DATA_WIDTH 32
    ENABLE_READ 0
  } {
    aclk aclk
    aresetn aresetn
    S_AXI ch${i}_axi_intercon/M01_AXI
    fifo_wr_data adc_cmd_fifo_$i/wr_data
    fifo_wr_en adc_cmd_fifo_$i/wr_en
    fifo_full adc_cmd_fifo_$i/full
  }


  ## ADC data FIFO
  # ADC data FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_data_fifo_${i}_rst_slice {
    DIN_WIDTH 24
    DIN_FROM [expr {3*($i-1)+2}]
    DIN_TO [expr {3*($i-1)+2}]
  } {
    din buffer_reset
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_data_fifo_${i}_spi_clk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in adc_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk spi_clk
  }
  cell xilinx.com:ip:proc_sys_reset:5.0 adc_data_fifo_${i}_aclk_rst {
    C_AUX_RESET_HIGH.VALUE_SRC USER
    C_AUX_RESET_HIGH 0
  } {
    aux_reset_in adc_data_fifo_${i}_rst_slice/dout
    slowest_sync_clk aclk
  }
  # ADC data FIFO
  cell lcb:user:fifo_async_count adc_data_fifo_$i {
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
  }
  # 32-bit ADC data FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_data_fifo_${i}_sts_word {
    NUM_PORTS 4
  } {
    In0 adc_data_fifo_$i/fifo_count_rd_clk
    In1 sts_word_padding/dout
    In2 adc_data_fifo_$i/empty
    In3 adc_data_fifo_$i/almost_empty
  }
  # ADC data FIFO AXI interface
  cell lcb:user:axi_fifo_bridge:1.0 adc_data_fifo_${i}_axi_bridge {
    AXI_ADDR_WIDTH 32
    AXI_DATA_WIDTH 32
    ENABLE_WRITE 0
  } {
    aclk aclk
    aresetn aresetn
    S_AXI ch${i}_axi_intercon/M02_AXI
    fifo_rd_data adc_data_fifo_$i/rd_data
    fifo_rd_en adc_data_fifo_$i/rd_en
    fifo_empty adc_data_fifo_$i/empty
  }
}

## Status concatenation out
# 32-bit padding for unused boards
cell xilinx.com:ip:xlconstant:1.1 empty_sts_word {
  CONST_VAL 0
  CONST_WIDTH 32
} {}
# Concatenate status words from all FIFOs
cell xilinx.com:ip:xlconcat:2.1 fifo_sts_concat {
  NUM_PORTS 24
} {
  dout fifo_sts
}
# Wire used board status words to the concatenation
for {set i 1} {$i <= $board_count} {incr i} {
  wire fifo_sts_concat/In[expr {3*($i-1)+0}] dac_cmd_fifo_${i}_sts_word/dout
  wire fifo_sts_concat/In[expr {3*($i-1)+1}] adc_cmd_fifo_${i}_sts_word/dout
  wire fifo_sts_concat/In[expr {3*($i-1)+2}] adc_data_fifo_${i}_sts_word/dout
}
# Wire unused board status words to the concatenation
for {set i [expr $board_count+1]} {$i <= 8} {incr i} {
  wire fifo_sts_concat/In[expr {3*($i-1)+0}] empty_sts_word/dout
  wire fifo_sts_concat/In[expr {3*($i-1)+1}] empty_sts_word/dout
  wire fifo_sts_concat/In[expr {3*($i-1)+2}] empty_sts_word/dout
}
