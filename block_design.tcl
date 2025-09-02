## Variably define the channel count (MUST BE 1 TO 8 INCLUSIVE)
set board_count 1

# If the board count is not 8, then error out
if {$board_count < 1 || $board_count > 8} {
  puts "Error: board_count must be between 1 and 8."
  exit 1
}

## Variably choose whether to use an external clock
set use_ext_clk 0

# If the external clock is not 0 or 1, then error out
if {$use_ext_clk != 0 && $use_ext_clk != 1} {
  puts "Error: use_ext_clk must be 0 or 1."
  exit 1
}

## Variably define the default SPI clock frequency (MHz)
set spi_clk_freq_mhz 20.000

# If the default SPI clock frequency is not between 1 and 50 MHz, then error out
if {$spi_clk_freq_mhz < 1.0 || $spi_clk_freq_mhz > 50.0} {
  puts "Error: spi_clk_freq_mhz must be between 1.0 and 50.0."
  exit 1
}

###############################################################################
#
#   Single-ended ports
#
###############################################################################


#------------------------------------------------------------
# Inputs
#------------------------------------------------------------

# (10Mhz_In)
create_bd_port -dir I -type clk -freq_hz 10000000 Scanner_10Mhz_In
# (Shutdown_Sense)
create_bd_port -dir I -type data Shutdown_Sense
# (Trigger_In)
create_bd_port -dir I Trigger_In
# (Manual_Enable)
create_bd_port -dir I Manual_Enable


#------------------------------------------------------------
# Outputs
#------------------------------------------------------------

# (Shutdown_Sense_Sel0-2)
create_bd_port -dir O -from 2 -to 0 -type data Shutdown_Sense_Sel
# (~Shutdown_Force)
create_bd_port -dir O n_Shutdown_Force
# (~Shutdown_Reset)
create_bd_port -dir O n_Shutdown_Reset



###############################################################################
#
#   Differential ports
#
###############################################################################


#------------------------------------------------------------
# DAC
#------------------------------------------------------------

# (LDAC+)
create_bd_port -dir O -from 0 -to 0 -type data LDAC_p
# (LDAC-)
create_bd_port -dir O -from 0 -to 0 -type data LDAC_n
# (~DAC_CS+)
create_bd_port -dir O -from 7 -to 0 -type data n_DAC_CS_p
# (~DAC_CS-)
create_bd_port -dir O -from 7 -to 0 -type data n_DAC_CS_n
# (DAC_MOSI+)
create_bd_port -dir O -from 7 -to 0 -type data DAC_MOSI_p
# (DAC_MOSI-)
create_bd_port -dir O -from 7 -to 0 -type data DAC_MOSI_n
# (DAC_MISO+)
create_bd_port -dir I -from 7 -to 0 -type data DAC_MISO_p
# (DAC_MISO-)
create_bd_port -dir I -from 7 -to 0 -type data DAC_MISO_n


#------------------------------------------------------------
# ADC
#------------------------------------------------------------

# (~ADC_CS+)
create_bd_port -dir O -from 7 -to 0 -type data n_ADC_CS_p
# (~ADC_CS-)
create_bd_port -dir O -from 7 -to 0 -type data n_ADC_CS_n
# (ADC_MOSI+)
create_bd_port -dir O -from 7 -to 0 -type data ADC_MOSI_p
# (ADC_MOSI-)
create_bd_port -dir O -from 7 -to 0 -type data ADC_MOSI_n
# (ADC_MISO+)
create_bd_port -dir I -from 7 -to 0 -type data ADC_MISO_p
# (ADC_MISO-)
create_bd_port -dir I -from 7 -to 0 -type data ADC_MISO_n


#------------------------------------------------------------
# Clocks
#------------------------------------------------------------

# (SCKO+)
create_bd_port -dir I -from 7 -to 0 MISO_SCK_p
# (SCKO-)
create_bd_port -dir I -from 7 -to 0 MISO_SCK_n
# (~SCKI+)
create_bd_port -dir O -from 0 -to 0 n_MOSI_SCK_p
# (~SCKI-)
create_bd_port -dir O -from 0 -to 0 n_MOSI_SCK_n

###############################################################################

### 0 and 1 constants to fill bits for unused boards
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  CONST_VAL 0
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  CONST_VAL 1
} {}

###############################################################################

### Create processing system
# Enable M_AXI_GP0 and M_AXI_GP1
# Enable UART1 on the correct MIO pins
# UART1 baud rate 921600
# Pullup for UART1 RX
# Enable I2C0 on the correct MIO pins
# Set FCLK0 to 100 MHz
# Turn off FCLK1-3 and reset1-3
init_ps ps {
  PCW_USE_M_AXI_GP0 1
  PCW_USE_M_AXI_GP1 1
  PCW_USE_S_AXI_ACP 0
  PCW_UART1_PERIPHERAL_ENABLE 1
  PCW_UART1_UART1_IO {MIO 36 .. 37}
  PCW_UART1_BAUD_RATE 921600
  PCW_MIO_37_PULLUP enabled
  PCW_I2C0_PERIPHERAL_ENABLE 1
  PCW_I2C0_I2C0_IO {MIO 38 .. 39}
  PCW_FPGA0_PERIPHERAL_FREQMHZ 100
  PCW_EN_CLK1_PORT 0
  PCW_EN_CLK2_PORT 0
  PCW_EN_CLK3_PORT 0
  PCW_EN_RST1_PORT 0
  PCW_EN_RST2_PORT 0
  PCW_EN_RST3_PORT 0
} {
  M_AXI_GP0_ACLK ps/FCLK_CLK0
  M_AXI_GP1_ACLK ps/FCLK_CLK0
}

## PS clock reset core
# Create proc_sys_reset
cell xilinx.com:ip:proc_sys_reset:5.0 ps_rst {} {
  ext_reset_in ps/FCLK_RESET0_N
  slowest_sync_clk ps/FCLK_CLK0
}

### AXI Smart Connect
cell xilinx.com:ip:smartconnect:1.0 sys_cfg_axi_intercon {
  NUM_SI 1
  NUM_MI 3
} {
  aclk ps/FCLK_CLK0
  S00_AXI ps/M_AXI_GP0
  aresetn ps_rst/peripheral_aresetn
}

###############################################################################

### Configuration register
## 32-bit offsets (see shim_axi_sys_ctrl.v)
# +0 System enable (1b cap)
# +1 Buffer reset (26b)
# +2 Integrator threshold average (unsigned, 15b, min 1, max 32767)
# +3 Integrator window (unsigned, 32b, min 2048)
# +4 Integrator enable (1b cap)
# +5 Boot test skip (16b cap)
cell lcb:user:shim_axi_sys_ctrl axi_sys_ctrl {
  INTEG_THRESHOLD_AVERAGE_DEFAULT 16384
  INTEG_WINDOW_DEFAULT 5000000
  INTEG_EN_DEFAULT 1
  MOSI_SCK_POL_DEFAULT 0
  MISO_SCK_POL_DEFAULT 1
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S_AXI sys_cfg_axi_intercon/M00_AXI
}
addr 0x40000000 128 axi_sys_ctrl/S_AXI ps/M_AXI_GP0
  

###############################################################################

### Hardware manager
cell lcb:user:shim_hw_manager hw_manager {} {
  clk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  sys_en axi_sys_ctrl/sys_en
  ext_en Manual_Enable
  sys_en_oob axi_sys_ctrl/sys_en_oob
  cmd_buf_reset_oob axi_sys_ctrl/cmd_buf_reset_oob
  data_buf_reset_oob axi_sys_ctrl/data_buf_reset_oob
  integ_thresh_avg_oob axi_sys_ctrl/integ_thresh_avg_oob
  integ_window_oob axi_sys_ctrl/integ_window_oob
  integ_en_oob axi_sys_ctrl/integ_en_oob
  boot_test_skip_oob axi_sys_ctrl/boot_test_skip_oob
  debug_oob axi_sys_ctrl/debug_oob
  mosi_sck_pol_oob axi_sys_ctrl/mosi_sck_pol_oob
  miso_sck_pol_oob axi_sys_ctrl/miso_sck_pol_oob
  unlock_cfg axi_sys_ctrl/unlock
  n_shutdown_force n_Shutdown_Force
  n_shutdown_rst n_Shutdown_Reset
}

## Shutdown sense
# Set which shutdown sense channels are connected
cell xilinx.com:ip:xlconcat:2.1 shutdown_sense_connected {
  NUM_PORTS 8
} {}
for {set i 0} {$i < $board_count} {incr i} {
  wire shutdown_sense_connected/In${i} const_1/dout
}
for {set i $board_count} {$i < 8} {incr i} {
  wire shutdown_sense_connected/In${i} const_0/dout
}
# Shutdown sense module
cell lcb:user:shim_shutdown_sense shutdown_sense {} {
  clk ps/FCLK_CLK0
  shutdown_sense_en hw_manager/shutdown_sense_en
  shutdown_sense_connected shutdown_sense_connected/dout
  shutdown_sense_pin Shutdown_Sense
  shutdown_sense hw_manager/shutdown_sense
  shutdown_sense_sel Shutdown_Sense_Sel
}

###############################################################################

### SPI clock control
if {$use_ext_clk} {
  # MMCM (handles down to 10 MHz input)
  # Includes power down and dynamic reconfiguration
  # Safe clock startup prevents clock output when not locked
  cell xilinx.com:ip:clk_wiz:6.0 spi_clk {
    PRIMITIVE MMCM
    USE_DYN_RECONFIG true
    USE_SAFE_CLOCK_STARTUP true
    PRIM_IN_FREQ 10
    CLKOUT1_REQUESTED_OUT_FREQ ${spi_clk_freq_mhz}
    FEEDBACK_SOURCE FDBK_AUTO
    CLKOUT1_DRIVES BUFGCE
  } {
    s_axi_aclk ps/FCLK_CLK0
    s_axi_aresetn ps_rst/peripheral_aresetn
    s_axi_lite sys_cfg_axi_intercon/M02_AXI
    clk_in1 Scanner_10Mhz_In
  }
} else {
  # Use FCLK_CLK0 as the clock input
  # (Vivado gives 99999893 Hz as the actual generated frequency)
  cell xilinx.com:ip:clk_wiz:6.0 spi_clk {
    PRIMITIVE MMCM
    USE_DYN_RECONFIG true
    USE_SAFE_CLOCK_STARTUP true
    PRIM_IN_FREQ 99.999893
    CLKOUT1_REQUESTED_OUT_FREQ ${spi_clk_freq_mhz}
    FEEDBACK_SOURCE FDBK_AUTO
    CLKOUT1_DRIVES BUFGCE
  } {
    s_axi_aclk ps/FCLK_CLK0
    s_axi_aresetn ps_rst/peripheral_aresetn
    s_axi_lite sys_cfg_axi_intercon/M02_AXI
    clk_in1 ps/FCLK_CLK0
  }
}
addr 0x40200000 2048 spi_clk/s_axi_lite ps/M_AXI_GP0
  

###############################################################################

### Calculate timing from SPI clock frequency
## Constant block for SPI clock frequency in Hz
cell xilinx.com:ip:xlconstant:1.1 spi_clk_freq_hz_const {
  CONST_VAL [expr {int($spi_clk_freq_mhz * 1000000 + 0.5)}]
  CONST_WIDTH 32
} {}
## Negate unlock_cfg to get calc signal
cell xilinx.com:ip:util_vector_logic n_unlock_cfg {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 axi_sys_ctrl/unlock
}
## Timing calculation cores
cell lcb:user:shim_ad5676_dac_timing_calc dac_timing_calc {} {
  clk spi_clk/clk_out1
  resetn ps_rst/peripheral_aresetn
  spi_clk_freq_hz spi_clk_freq_hz_const/dout
  calc n_unlock_cfg/Res
}
cell lcb:user:shim_ads816x_adc_timing_calc adc_timing_calc {
  ADS_MODEL_ID 8
} {
  clk spi_clk/clk_out1
  resetn ps_rst/peripheral_aresetn
  spi_clk_freq_hz spi_clk_freq_hz_const/dout
  calc n_unlock_cfg/Res
}
## OR the done signals together
cell xilinx.com:ip:util_vector_logic calc_n_cs_done_or {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 dac_timing_calc/done
  Op2 adc_timing_calc/done
  Res hw_manager/calc_n_cs_done
}
## OR the lock violation signals together with axi_sys_ctrl/lock_viol as well (two OR stages)
cell xilinx.com:ip:util_vector_logic calc_n_cs_lock_viol_or {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 dac_timing_calc/lock_viol
  Op2 adc_timing_calc/lock_viol
}
cell xilinx.com:ip:util_vector_logic lock_viol_or {
  C_SIZE 1
  C_OPERATION or
} {
  Op1 calc_n_cs_lock_viol_or/Res
  Op2 axi_sys_ctrl/lock_viol
  Res hw_manager/lock_viol
}
  

### SPI clock domain
module spi_clk_domain spi_clk_domain {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  spi_clk spi_clk/clk_out1
  spi_en hw_manager/spi_en
  integ_thresh_avg axi_sys_ctrl/integ_thresh_avg
  integ_window axi_sys_ctrl/integ_window
  integ_en axi_sys_ctrl/integ_en
  dac_n_cs_high_time dac_timing_calc/n_cs_high_time
  adc_n_cs_high_time adc_timing_calc/n_cs_high_time
  boot_test_skip axi_sys_ctrl/boot_test_skip
  debug axi_sys_ctrl/debug
  spi_off hw_manager/spi_off
  over_thresh hw_manager/over_thresh
  thresh_underflow hw_manager/thresh_underflow
  thresh_overflow hw_manager/thresh_overflow
  bad_trig_cmd hw_manager/bad_trig_cmd
  trig_data_buf_overflow hw_manager/trig_data_buf_overflow
  dac_boot_fail hw_manager/dac_boot_fail
  bad_dac_cmd hw_manager/bad_dac_cmd
  dac_cal_oob hw_manager/dac_cal_oob
  dac_val_oob hw_manager/dac_val_oob
  dac_cmd_buf_underflow hw_manager/dac_cmd_buf_underflow
  dac_data_buf_overflow hw_manager/dac_data_buf_overflow
  unexp_dac_trig hw_manager/unexp_dac_trig
  ldac_misalign hw_manager/ldac_misalign
  dac_delay_too_short hw_manager/dac_delay_too_short
  adc_boot_fail hw_manager/adc_boot_fail
  bad_adc_cmd hw_manager/bad_adc_cmd
  adc_cmd_buf_underflow hw_manager/adc_cmd_buf_underflow
  adc_data_buf_overflow hw_manager/adc_data_buf_overflow
  unexp_adc_trig hw_manager/unexp_adc_trig
  adc_delay_too_short hw_manager/adc_delay_too_short
  ext_trig Trigger_In
  block_bufs hw_manager/block_bufs
}

###############################################################################

### DAC/ADC Command and Data FIFOs Module
module axi_spi_interface axi_spi_interface {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  cmd_buf_reset axi_sys_ctrl/cmd_buf_reset
  data_buf_reset axi_sys_ctrl/data_buf_reset
  spi_clk spi_clk/clk_out1
  S_AXI ps/M_AXI_GP1
}
## Wire channel pins for the module
# DAC and ADC FIFOs
for {set i 0} {$i < $board_count} {incr i} {
  wire axi_spi_interface/dac_ch${i}_cmd spi_clk_domain/dac_ch${i}_cmd
  wire axi_spi_interface/dac_ch${i}_cmd_rd_en spi_clk_domain/dac_ch${i}_cmd_rd_en
  wire axi_spi_interface/dac_ch${i}_cmd_empty spi_clk_domain/dac_ch${i}_cmd_empty
  wire axi_spi_interface/dac_ch${i}_data spi_clk_domain/dac_ch${i}_data
  wire axi_spi_interface/dac_ch${i}_data_wr_en spi_clk_domain/dac_ch${i}_data_wr_en
  wire axi_spi_interface/dac_ch${i}_data_full spi_clk_domain/dac_ch${i}_data_full
  wire axi_spi_interface/adc_ch${i}_cmd spi_clk_domain/adc_ch${i}_cmd
  wire axi_spi_interface/adc_ch${i}_cmd_rd_en spi_clk_domain/adc_ch${i}_cmd_rd_en
  wire axi_spi_interface/adc_ch${i}_cmd_empty spi_clk_domain/adc_ch${i}_cmd_empty
  wire axi_spi_interface/adc_ch${i}_data spi_clk_domain/adc_ch${i}_data
  wire axi_spi_interface/adc_ch${i}_data_wr_en spi_clk_domain/adc_ch${i}_data_wr_en
  wire axi_spi_interface/adc_ch${i}_data_full spi_clk_domain/adc_ch${i}_data_full
}
# Trigger command FIFO
wire axi_spi_interface/trig_cmd spi_clk_domain/trig_cmd
wire axi_spi_interface/trig_cmd_rd_en spi_clk_domain/trig_cmd_rd_en
wire axi_spi_interface/trig_cmd_empty spi_clk_domain/trig_cmd_empty
# Trigger data FIFO
wire axi_spi_interface/trig_data spi_clk_domain/trig_data
wire axi_spi_interface/trig_data_wr_en spi_clk_domain/trig_data_wr_en
wire axi_spi_interface/trig_data_full spi_clk_domain/trig_data_full
wire axi_spi_interface/trig_data_almost_full spi_clk_domain/trig_data_almost_full
## Address assignment
# DAC and ADC FIFOs
for {set i 0} {$i < $board_count} {incr i} {
  addr 0x800${i}0000 128 axi_spi_interface/dac_fifo_${i}_axi_bridge/S_AXI ps/M_AXI_GP1
  addr 0x800${i}1000 128 axi_spi_interface/adc_fifo_${i}_axi_bridge/S_AXI ps/M_AXI_GP1
}
# Trigger command and data FIFOs
addr 0x80100000 128 axi_spi_interface/trig_fifo_axi_bridge/S_AXI ps/M_AXI_GP1

## AXI-domain over/underflow detection
wire axi_spi_interface/dac_cmd_buf_overflow hw_manager/dac_cmd_buf_overflow
wire axi_spi_interface/dac_data_buf_underflow hw_manager/dac_data_buf_underflow
wire axi_spi_interface/adc_cmd_buf_overflow hw_manager/adc_cmd_buf_overflow
wire axi_spi_interface/adc_data_buf_underflow hw_manager/adc_data_buf_underflow
wire axi_spi_interface/trig_cmd_buf_overflow hw_manager/trig_cmd_buf_overflow
wire axi_spi_interface/trig_data_buf_underflow hw_manager/trig_data_buf_underflow

###############################################################################

### Status register
cell pavel-demin:user:axi_sts_register status_reg {
  STS_DATA_WIDTH 2048
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S_AXI sys_cfg_axi_intercon/M01_AXI
}
addr 0x40100000 256 status_reg/S_AXI ps/M_AXI_GP0
## Concatenation
#    31 : 0    --  32b Hardware status code (31:29 board num, 28:4 status code, 3:0 internal state)
#   575 : 32   -- 544b Command FIFO status (32 bits per buffer, ordered as DAC0, ADC0, ..., DAC7, ADC7, Trigger)
#  1119 : 576  -- 544b Data FIFO status (32 bits per buffer, ordered as DAC0, ADC0, ..., DAC7, ADC7, Trigger)
#  1151 : 1120 --  32b Debug 1 (SPI clock locked, spi_off, 28 reserved bits)
#  2047 : 1152 -- RESERVED (0)
cell xilinx.com:ip:xlconcat:2.1 sts_concat {
  NUM_PORTS 5
} {
  In0 hw_manager/status_word
  In1 axi_spi_interface/cmd_fifo_sts
  In2 axi_spi_interface/data_fifo_sts
  dout status_reg/sts_data
}
# Debug 1 tracks the following:
# 0: SPI clock locked
# 1: `spi_off` signal
cell xilinx.com:ip:xlconcat:2.1 debug_1 {
  NUM_PORTS 3
} {
  In0 spi_clk/locked
  In1 hw_manager/spi_off
  dout sts_concat/In3
}
cell xilinx.com:ip:xlconstant:1.1 pad_30 {
  CONST_VAL 0
  CONST_WIDTH 30
} {
  dout debug_1/In2
}
# Pad reserved bits
cell xilinx.com:ip:xlconstant:1.1 pad_sts_reserved {
  CONST_VAL 0
  CONST_WIDTH [expr {2048 - 1152}]
} {
  dout sts_concat/In4
}

## IRQ interrupt concat
cell xilinx.com:ip:xlconcat:2.1 irq_concat {
  NUM_PORTS 1
} {
  In0 hw_manager/ps_interrupt
  dout ps/IRQ_F2P
}

###############################################################################

### Gate the SPI clocks (MISO and MOSI SCK)
cell lcb:user:clock_gate spi_mosi_sck_gate {} {
  clk spi_clk/clk_out1
  en hw_manager/spi_clk_gate
}
cell lcb:user:clock_gate spi_miso_sck_gate {} {
  en hw_manager/spi_clk_gate
}
## Potentially invert the clocks based on register settings
cell xilinx.com:ip:util_vector_logic mosi_sck_pol {
  C_SIZE 1
  C_OPERATION xor
} {
  Op1 spi_mosi_sck_gate/clk_gated
  Op2 axi_sys_ctrl/mosi_sck_pol
}
# Extend the MISO SCK polarity to 8 bits of the same value
cell xilinx.com:ip:xlconcat:2.1 miso_sck_ext {
  NUM_PORTS 8
} {}
for {set i 0} {$i < 8} {incr i} {
  wire miso_sck_ext/In${i} spi_miso_sck_gate/clk_gated
}
cell xilinx.com:ip:util_vector_logic miso_sck_pol {
  C_SIZE 8
  C_OPERATION xor
} {
  Op1 miso_sck_ext/dout
  Op2 axi_sys_ctrl/miso_sck_pol
  Res spi_clk_domain/miso_sck
}

### Create I/O buffers for differential signals
module io_bufs io_bufs {
  ldac spi_clk_domain/ldac
  n_dac_cs spi_clk_domain/n_dac_cs
  dac_mosi spi_clk_domain/dac_mosi
  dac_miso spi_clk_domain/dac_miso
  n_adc_cs spi_clk_domain/n_adc_cs
  adc_mosi spi_clk_domain/adc_mosi
  adc_miso spi_clk_domain/adc_miso
  miso_sck spi_miso_sck_gate/clk
  n_mosi_sck mosi_sck_pol/Res
  ldac_p LDAC_p
  ldac_n LDAC_n
  n_dac_cs_p n_DAC_CS_p
  n_dac_cs_n n_DAC_CS_n
  dac_mosi_p DAC_MOSI_p
  dac_mosi_n DAC_MOSI_n
  dac_miso_p DAC_MISO_p
  dac_miso_n DAC_MISO_n
  n_adc_cs_p n_ADC_CS_p
  n_adc_cs_n n_ADC_CS_n
  adc_mosi_p ADC_MOSI_p
  adc_mosi_n ADC_MOSI_n
  adc_miso_p ADC_MISO_p
  adc_miso_n ADC_MISO_n
  miso_sck_p MISO_SCK_p
  miso_sck_n MISO_SCK_n
  n_mosi_sck_p n_MOSI_SCK_p
  n_mosi_sck_n n_MOSI_SCK_n
}

###############################################################################
