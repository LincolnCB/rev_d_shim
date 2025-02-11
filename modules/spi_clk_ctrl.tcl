## SPI clock control module

create_bd_pin -dir I ext_10mhz_in
create_bd_pin -dir O spi_clk
create_bd_pin -dir I s_axi_aclk
create_bd_intf_pin -mode Slave -vlnv xilinx.com:interface:aximm_rtl:1.0 s_axi_clk_ctrl

# cell xilinx.com:ip:clk_wiz:6.0 upstage_clk {
#   CLKIN1_JITTER_PS 100.0
#   CLKIN1_UI_JITTER 0.001
#   CLKOUT1_JITTER 552.815
#   CLKOUT1_PHASE_ERROR 883.386
#   CLKOUT1_REQUESTED_OUT_FREQ 200.000
#   MMCM_CLKFBOUT_MULT_F 62.500
#   MMCM_CLKIN1_PERIOD 100.000
#   MMCM_CLKIN2_PERIOD 10.0
#   MMCM_CLKOUT0_DIVIDE_F 3.125
#   MMCM_REF_JITTER1 0.001
#   PRIM_IN_FREQ 10
#   USE_POWER_DOWN true
# } {
#   clk_in1 ext_10mhz_in
# }
cell xilinx.com:ip:clk_wiz:6.0 upstage_clk {
  PRIMITIVE MMCM
  USE_POWER_DOWN true
  PRIM_IN_FREQ 10
  CLKIN1_JITTER_PS 100.0
  CLKOUT1_REQUESTED_OUT_FREQ 200.000
} {
  clk_in1 ext_10mhz_in
}

cell xilinx.com:ip:clk_wiz:6.0 configurable_clk {
  AXI_DRP false
  CLKIN1_JITTER_PS 50.0
  CLKOUT1_DRIVES BUFG
  CLKOUT1_JITTER 198.242
  CLKOUT1_PHASE_ERROR 155.540
  CLKOUT1_REQUESTED_OUT_FREQ 50.000
  FEEDBACK_SOURCE FDBK_AUTO
  PHASE_DUTY_CONFIG false
  PLL_CLKIN_PERIOD 5.000
  PRIMITIVE PLL
  PRIM_IN_FREQ 200
  USE_POWER_DOWN true
  USE_DYN_RECONFIG true
} {
  s_axi_aclk s_axi_aclk
  clk_in1 upstage_clk/clk_out1
  clk_out1 spi_clk
}
