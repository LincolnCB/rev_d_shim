##################################################

### Create processing system
# Enable M_AXI_GP0 and S_AXI_ACP
# Tie AxUSER pins to 1 for ACP port (to enable coherency)
# Enable UART1 on the correct MIO pins
# UART1 baud rate 921600
# Pullup for UART1 RX
# Enable I2C0 on the correct MIO pins
# Turn off FCLK1-3 and reset1-3
init_ps ps {
  PCW_USE_M_AXI_GP0 1
  PCW_USE_S_AXI_ACP 1
  USE_DEFAULT_ACP_USER_VAL 1
  PCW_UART1_PERIPHERAL_ENABLE 1
  PCW_UART1_UART1_IO {MIO 36 .. 37}
  PCW_UART1_BAUD_RATE 921600
  PCW_MIO_37_PULLUP enabled
  PCW_I2C0_PERIPHERAL_ENABLE 1
  PCW_I2C0_I2C0_IO {MIO 38 .. 39}
  PCW_EN_CLK1_PORT 0
  PCW_EN_CLK2_PORT 0
  PCW_EN_CLK3_PORT 0
  PCW_EN_RST1_PORT 0
  PCW_EN_RST2_PORT 0
  PCW_EN_RST3_PORT 0
} {
  M_AXI_GP0_ACLK ps/FCLK_CLK0
  S_AXI_ACP_ACLK ps/FCLK_CLK0
}

## PS reset core
# Create proc_sys_reset
cell xilinx.com:ip:proc_sys_reset:5.0 ps_rst {} {
  ext_reset_in ps/FCLK_RESET0_N
  slowest_sync_clk ps/FCLK_CLK0
}


### AXI Smart Connect
cell xilinx.com:ip:smartconnect:1.0 axi_smc {
  NUM_SI 1
  NUM_MI 3
} {
  aclk ps/FCLK_CLK0
  S00_AXI /ps/M_AXI_GP0
  aresetn ps_rst/peripheral_aresetn
}

##################################################

### Configuration register
## 32-bit offsets
# +0 Trigger lockout
# +1 Calibration offset (signed, 16b cap)
# +2 DAC divider (unsigned, 16b cap)
# +3 Integrator threshold average (unsigned, 16b cap)
# +4 Integrator window (unsigned, 32b cap)
# +5 Integrator enable (1b cap)
# +6 Hardware enable (1b cap)
# +7 SPI enable (1b cap)
cell lcb:user:axi_shim_cfg:1.0 axi_shim_cfg {
  TRIGGER_LOCKOUT_DEFAULT 250000
  CALIBRATION_OFFSET_DEFAULT 0
  DAC_DIVIDER_DEFAULT 1000
  INTEGRATOR_THRESHOLD_AVERAGE_DEFAULT 16384
  INTEGRATOR_WINDOW_DEFAULT 5000000
  INTEG_EN_DEFAULT 1
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S_AXI axi_smc/M00_AXI
}
addr 0x40000000 128 axi_shim_cfg/S_AXI
  

##################################################

### Hardware manager
cell lcb:user:hw_manager:1.0 hw_manager {
  POWERON_WAIT   250000000
  BUF_LOAD_WAIT  250000000
  SPI_START_WAIT 250000000
  SPI_STOP_WAIT  250000000
} {
  clk ps/FCLK_CLK0
  sys_en axi_shim_cfg/sys_en
  ext_shutdown Shutdown_Button
  n_shutdown_force n_Shutdown_Force
  n_shutdown_rst n_Shutdown_Reset
  trig_lockout_oob axi_shim_cfg/trig_lockout_oob
  cal_offset_oob axi_shim_cfg/cal_offset_oob
  dac_divider_oob axi_shim_cfg/dac_divider_oob
  integ_thresh_avg_oob axi_shim_cfg/integ_thresh_avg_oob
  integ_window_oob axi_shim_cfg/integ_window_oob
  integ_en_oob axi_shim_cfg/integ_en_oob
  sys_en_oob axi_shim_cfg/sys_en_oob
  lock_viol axi_shim_cfg/lock_viol
  unlock_cfg axi_shim_cfg/unlock
}

## Shutdown sense
## Shutdown sense
cell lcb:user:shutdown_sense:1.0 shutdown_sense {
  CLK_FREQ_HZ 100000000
} {
  clk ps/FCLK_CLK0
  rst hw_manager/sys_rst
  shutdown_sense_pin Shutdown_Sense
  shutdown_sense hw_manager/shutdown_sense
  shutdown_sense_sel Shutdown_Sense_Sel
}

##################################################

### SPI clock control
# MMCM (handles down to 10 MHz input)
# Includes power down and dynamic reconfiguration
# Safe clock startup prevents clock output when not locked
cell xilinx.com:ip:clk_wiz:6.0 spi_clk {
  PRIMITIVE MMCM
  USE_POWER_DOWN true
  USE_DYN_RECONFIG true
  USE_SAFE_CLOCK_STARTUP true
  PRIM_IN_FREQ 10
  CLKOUT1_REQUESTED_OUT_FREQ 50.000
  FEEDBACK_SOURCE FDBK_AUTO
  CLKOUT1_DRIVES BUFGCE
} {
  s_axi_aclk ps/FCLK_CLK0
  s_axi_aresetn ps_rst/peripheral_aresetn
  s_axi_lite axi_smc/M02_AXI
  clk_in1 Scanner_10Mhz_In
}
addr 0x40200000 2048 spi_clk/s_axi_lite
# Negate spi_en to power down the clock
cell xilinx.com:ip:util_vector_logic n_spi_en {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 hw_manager/spi_en
  Res spi_clk/power_down
}

##################################################

### SPI clock domain
module spi_clk_domain {
  source projects/rev_d_shim/modules/spi_clk_domain.tcl
} {
  sck spi_clk/clk_out1
  rst hw_manager/sys_rst
  clk ps/FCLK_CLK0
  trig_lockout axi_shim_cfg/trig_lockout
  cal_offset axi_shim_cfg/cal_offset
  dac_divider axi_shim_cfg/dac_divider
  integ_thresh_avg axi_shim_cfg/integ_thresh_avg
  integ_window axi_shim_cfg/integ_window
  integ_en axi_shim_cfg/integ_en
  spi_en hw_manager/spi_en
  spi_running hw_manager/spi_running
  dac_over_thresh hw_manager/dac_over_thresh
  adc_over_thresh hw_manager/adc_over_thresh
  dac_thresh_underflow hw_manager/dac_thresh_underflow
  dac_thresh_overflow hw_manager/dac_thresh_overflow
  adc_thresh_underflow hw_manager/adc_thresh_underflow
  adc_thresh_overflow hw_manager/adc_thresh_overflow
  dac_buf_underflow hw_manager/dac_buf_underflow
  adc_buf_underflow hw_manager/adc_buf_underflow
  premat_dac_trig hw_manager/premat_dac_trig
  premat_adc_trig hw_manager/premat_adc_trig
  premat_dac_div hw_manager/premat_dac_div
  premat_adc_div hw_manager/premat_adc_div
}
## Trigger enable AND gate
cell xilinx.com:ip:util_vector_logic trig_en_and {
  C_SIZE 1
  C_OPERATION and
} {
  Op1 hw_manager/trig_en
  Op2 Trigger_In
  Res spi_clk_domain/trigger_gated
}

##################################################

### DAC and ADC FIFOs
module dac_fifo_0 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_0 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_1 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_1 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_2 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_2 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_3 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_3 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_4 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_4 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_5 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_5 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_6 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_6 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}
module dac_fifo_7 {
  source projects/rev_d_shim/modules/dac_fifo.tcl
} {
}
module adc_fifo_7 {
  source projects/rev_d_shim/modules/adc_fifo.tcl
} {
}

##################################################

### Status register
cell pavel-demin:user:axi_sts_register:1.0 status_reg {
  STS_DATA_WIDTH 2048
} {
  aclk ps/FCLK_CLK0
  S_AXI axi_smc/M01_AXI
  aresetn ps_rst/peripheral_aresetn
}
addr 0x40100000 256 status_reg/S_AXI
## Concatenation
#   31:0    -- 32b Hardware status code (31:29 board num, 28:4 status code, 3:0 internal state)
#   63:32   --     Reserved
#  127:64   -- 64b DAC0 FIFO status (see FIFO module)
#  191:128  -- 64b ADC0 FIFO status
#  255:192  -- 64b DAC1 FIFO status
#  319:256  -- 64b ADC1 FIFO status
#  383:320  -- 64b DAC2 FIFO status
#  447:384  -- 64b ADC2 FIFO status
#  511:448  -- 64b DAC3 FIFO status
#  575:512  -- 64b ADC3 FIFO status
#  639:576  -- 64b DAC4 FIFO status
#  703:640  -- 64b ADC4 FIFO status
#  767:704  -- 64b DAC5 FIFO status
#  831:768  -- 64b ADC5 FIFO status
#  895:832  -- 64b DAC6 FIFO status
#  959:896  -- 64b ADC6 FIFO status
# 1023:960  -- 64b DAC7 FIFO status
# 1087:1024 -- 64b ADC7 FIFO status
# 2047:1088 --     Reserved
cell xilinx.com:ip:xlconstant:1.1 pad_32 {
  CONST_VAL 0
  CONST_WIDTH 32
} {}
## Pad reserved bits
cell xilinx.com:ip:xlconstant:1.1 pad_960 {
  CONST_VAL 0
  CONST_WIDTH 960
} {}
cell xilinx.com:ip:xlconcat:2.1 sts_concat {
  NUM_PORTS 19
} {
  In0  hw_manager/status_word
  In1  pad_32/dout
  In2  dac_fifo_0/fifo_sts_word
  In3  adc_fifo_0/fifo_sts_word
  In4  dac_fifo_1/fifo_sts_word
  In5  adc_fifo_1/fifo_sts_word
  In6  dac_fifo_2/fifo_sts_word
  In7  adc_fifo_2/fifo_sts_word
  In8  dac_fifo_3/fifo_sts_word
  In9  adc_fifo_3/fifo_sts_word
  In10 dac_fifo_4/fifo_sts_word
  In11 adc_fifo_4/fifo_sts_word
  In12 dac_fifo_5/fifo_sts_word
  In13 adc_fifo_5/fifo_sts_word
  In14 dac_fifo_6/fifo_sts_word
  In15 adc_fifo_6/fifo_sts_word
  In16 dac_fifo_7/fifo_sts_word
  In17 adc_fifo_7/fifo_sts_word
  In18 pad_960/dout
  dout status_reg/sts_data
}

##################################################

### Create I/O buffers for differential signals

## DAC
# (LDAC)
cell lcb:user:differential_out_buffer:1.0 ldac_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  d_in spi_clk_domain/ldac
  diff_out_p LDAC_p
  diff_out_n LDAC_n
}
# (~DAC_CS)
cell lcb:user:differential_out_buffer:1.0 n_dac_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in spi_clk_domain/n_dac_cs
  diff_out_p n_DAC_CS_p
  diff_out_n n_DAC_CS_n
}
# (DAC_MOSI)
cell lcb:user:differential_out_buffer:1.0 dac_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in spi_clk_domain/dac_mosi
  diff_out_p DAC_MOSI_p
  diff_out_n DAC_MOSI_n
}
# (DAC_MISO)
cell lcb:user:differential_in_buffer:1.0 dac_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p DAC_MISO_p
  diff_in_n DAC_MISO_n
  d_out spi_clk_domain/dac_miso
}

## ADC
# (~ADC_CS)
cell lcb:user:differential_out_buffer:1.0 n_adc_cs_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in spi_clk_domain/n_adc_cs
  diff_out_p n_ADC_CS_p
  diff_out_n n_ADC_CS_n
}
# (ADC_MOSI)
cell lcb:user:differential_out_buffer:1.0 adc_mosi_obuf {
  DIFF_BUFFER_WIDTH 8
} {
  d_in spi_clk_domain/adc_mosi
  diff_out_p ADC_MOSI_p
  diff_out_n ADC_MOSI_n
}
# (ADC_MISO)
cell lcb:user:differential_in_buffer:1.0 adc_miso_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p ADC_MISO_p
  diff_in_n ADC_MISO_n
  d_out spi_clk_domain/adc_miso
}

## Clocks
# (MISO_SCK)
cell lcb:user:differential_in_buffer:1.0 miso_sck_ibuf {
  DIFF_BUFFER_WIDTH 8
} {
  diff_in_p MISO_SCK_p
  diff_in_n MISO_SCK_n
  d_out spi_clk_domain/miso_sck
}
# (~MOSI_SCK)
cell lcb:user:differential_out_buffer:1.0 n_mosi_sck_obuf {
  DIFF_BUFFER_WIDTH 1
} {
  d_in spi_clk/clk_out1
  diff_out_p n_MOSI_SCK_p
  diff_out_n n_MOSI_SCK_n
}
##################################################
