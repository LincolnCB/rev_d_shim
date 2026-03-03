### FIFO Module with 32-bit CFG and STS connections

## Ports to connect:
# Clock
create_bd_pin -dir I aclk
# 32-bit Configuration word
create_bd_pin -dir I -from 31 -to 0 cfg_word
# 32-bit Status word
create_bd_pin -dir O -from 31 -to 0 sts_word
# Subortinate/Slave AXI interface
create_bd_intf_pin -mode slave -vlnv xilinx.com:interface:aximm_rtl:1.0 S_AXI

##################################################

## Synchronous FIFO with AXI Stream interface

## Reset signal handling
# Slice out the first CFG bit to be the FIFO's reset signal
cell xilinx.com:ip:xlslice:1.0 rst_slice {
  DIN_WIDTH 32
  DIN_FROM 0
  DIN_TO 0
} {
  din cfg_word
}
# Negate the reset signal (To make active high reset)
cell xilinx.com:ip:util_vector_logic n_rst {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 rst_slice/dout
}
# Reset manager
cell xilinx.com:ip:proc_sys_reset:5.0 fifo_rst {} {
  ext_reset_in n_rst/Res
  slowest_sync_clk aclk
}

## Custom FIFO to allow for extra status signals
# AXI Stream interface
cell base:user:axi_fifo_bridge axi_fifo_bridge {} {
  aclk aclk
  wr_resetn fifo_rst/peripheral_aresetn
  rd_resetn fifo_rst/peripheral_aresetn
  s_axi S_AXI
}
# FIFO
cell base:user:fifo_sync fifo {
  DATA_WIDTH 32
  ADDR_WIDTH 4
} {
  clk aclk
  resetn fifo_rst/peripheral_aresetn
  wr_data axi_fifo_bridge/fifo_wr_data
  wr_en axi_fifo_bridge/fifo_wr_en
  full axi_fifo_bridge/fifo_full
  rd_data axi_fifo_bridge/fifo_rd_data
  rd_en axi_fifo_bridge/fifo_rd_en
  empty axi_fifo_bridge/fifo_empty
}

## Concatenate the FIFO's status signals to one 32-bit word
# 18 bit pad to make the status word 32 bits
cell xilinx.com:ip:xlconstant:1.1 sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH 18
} {}
# Concatenate the status signals into a 32-bit word
cell xilinx.com:ip:xlconcat:2.1 sts_word {
  NUM_PORTS 7
} {
  In0 fifo/fifo_count
  In1 fifo/full
  In2 axi_fifo_bridge/fifo_overflow
  In3 fifo/fifo_count
  In4 fifo/empty
  In5 axi_fifo_bridge/fifo_underflow
  In6 sts_word_padding/dout
  dout sts_word
}
