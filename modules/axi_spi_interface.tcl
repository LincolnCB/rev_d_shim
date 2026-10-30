### DAC/ADC Command and Data FIFOs Module

# Get the board count from the calling context
set board_count [module_get_upvar board_count]

# If the board count is not 8, then error out
if {$board_count < 1 || $board_count > 8} {
  puts "Error: board_count must be between 1 and 8."
  exit 1
}

# Get the DAC/ADC/trigger FIFO address widths from the calling context
set dac_cmd_fifo_addr_width [module_get_upvar dac_cmd_fifo_addr_width]
set dac_debug_fifo_addr_width [module_get_upvar dac_debug_fifo_addr_width]
set dac_data_fifo_addr_width [module_get_upvar dac_data_fifo_addr_width]
set adc_cmd_fifo_addr_width [module_get_upvar adc_cmd_fifo_addr_width]
set adc_debug_fifo_addr_width [module_get_upvar adc_debug_fifo_addr_width]
set adc_data_fifo_addr_width [module_get_upvar adc_data_fifo_addr_width]
set trig_cmd_fifo_addr_width [module_get_upvar trig_cmd_fifo_addr_width]
set trig_data_fifo_addr_width [module_get_upvar trig_data_fifo_addr_width]

# Make sure they are all between 10 and 17, except the small calibration readback FIFO
if {$dac_cmd_fifo_addr_width < 10 || $dac_cmd_fifo_addr_width > 17 ||
    $dac_debug_fifo_addr_width < 10 || $dac_debug_fifo_addr_width > 17 ||
    $adc_cmd_fifo_addr_width < 10 || $adc_cmd_fifo_addr_width > 17 ||
    $adc_debug_fifo_addr_width < 10 || $adc_debug_fifo_addr_width > 17 ||
    $adc_data_fifo_addr_width < 10 || $adc_data_fifo_addr_width > 17 ||
    $trig_cmd_fifo_addr_width < 10 || $trig_cmd_fifo_addr_width > 17 ||
    $trig_data_fifo_addr_width < 10 || $trig_data_fifo_addr_width > 17} {
  puts "Error: FIFO address widths must be between 10 and 17."
  exit 1
}
if {$dac_data_fifo_addr_width < 4 || $dac_data_fifo_addr_width > 17} {
  puts "Error: dac_data_fifo_addr_width must be between 4 and 17."
  exit 1
}

# ADC capture packet size (words). One dense S2MM packet / descriptor buffer is this many words,
# and it is also the rate-control packing threshold -- fed to both the packetizer and adc_rate_ctrl
# from here so the two cannot drift.
set adc_packet_words 256

##################################################

### Ports

# System signals
create_bd_pin -dir I -type clock aclk
create_bd_pin -dir I -type reset aresetn
create_bd_pin -dir I -from 16 -to 0 cmd_buf_reset
create_bd_pin -dir I -from 16 -to 0 data_buf_reset
create_bd_pin -dir I -type clock spi_clk
create_bd_pin -dir I -from 7 -to 0 datapath_mode

# All-board DAC/ADC FIFO AXI slave + trigger FIFO slave, fanned to by the top-level register
# SmartConnect (one subordinate for every board's FIFOs, plus the trigger bridge)
create_bd_intf_pin -mode slave -vlnv xilinx.com:interface:aximm_rtl:1.0 board_fifo_axi
create_bd_intf_pin -mode slave -vlnv xilinx.com:interface:aximm_rtl:1.0 trig_axi

# Status concatenated out
create_bd_pin -dir O -from 543 -to 0 cmd_fifo_sts
create_bd_pin -dir O -from 543 -to 0 data_fifo_sts
create_bd_pin -dir O -from 255 -to 0 adc_debug_fifo_sts
create_bd_pin -dir O -from 255 -to 0 dac_data_fifo_sts

# DAC/ADC command, debug and data channels
for {set i 0} {$i < $board_count} {incr i} {
  # DAC command channel
  create_bd_pin -dir O -from 31 -to 0 dac_ch${i}_cmd
  create_bd_pin -dir I dac_ch${i}_cmd_rd_en
  create_bd_pin -dir O dac_ch${i}_cmd_empty

  # DAC debug channel
  create_bd_pin -dir I -from 31 -to 0 dac_ch${i}_debug
  create_bd_pin -dir I dac_ch${i}_debug_wr_en
  create_bd_pin -dir O dac_ch${i}_debug_full

  # DAC data channel (calibration readback)
  create_bd_pin -dir I -from 31 -to 0 dac_ch${i}_data
  create_bd_pin -dir I dac_ch${i}_data_wr_en
  create_bd_pin -dir O dac_ch${i}_data_full
  create_bd_pin -dir O dac_ch${i}_data_almost_full

  # ADC command channel
  create_bd_pin -dir O -from 31 -to 0 adc_ch${i}_cmd
  create_bd_pin -dir I adc_ch${i}_cmd_rd_en
  create_bd_pin -dir O adc_ch${i}_cmd_empty

  # ADC debug channel
  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_debug
  create_bd_pin -dir I adc_ch${i}_debug_wr_en
  create_bd_pin -dir O adc_ch${i}_debug_full

  # ADC data channel
  create_bd_pin -dir I -from 31 -to 0 adc_ch${i}_data
  create_bd_pin -dir I adc_ch${i}_data_wr_en
  create_bd_pin -dir O adc_ch${i}_data_full
  create_bd_pin -dir O adc_ch${i}_data_almost_full

  # DMA datapath AXIS interfaces (DAC command in from MM2S, ADC data out to S2MM)
  create_bd_intf_pin -mode slave  -vlnv xilinx.com:interface:axis_rtl:1.0 dac_ch${i}_dma
  create_bd_intf_pin -mode master -vlnv xilinx.com:interface:axis_rtl:1.0 adc_ch${i}_dma
}

# Trigger command channel
create_bd_pin -dir O -from 31 -to 0 trig_cmd
create_bd_pin -dir I trig_cmd_rd_en
create_bd_pin -dir O trig_cmd_empty
create_bd_pin -dir O trig_cmd_almost_empty

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
create_bd_pin -dir O -from 7 -to 0 mode_viol
create_bd_pin -dir O trig_cmd_buf_overflow
create_bd_pin -dir O trig_data_buf_underflow

## 0 and 1 constants to fill bits for unused boards
cell xilinx.com:ip:xlconstant:1.1 const_0 {
  CONST_VAL 0
} {}
cell xilinx.com:ip:xlconstant:1.1 const_1 {
  CONST_VAL 1
} {}
# 32-bit zero for tying off the unused read side of the DMA write bridges
cell xilinx.com:ip:xlconstant:1.1 const_0_32 {
  CONST_VAL 0
  CONST_WIDTH 32
} {}

#######################################################

# Status word padding for making 32-bit status words
cell xilinx.com:ip:xlconstant:1.1 dac_cmd_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $dac_cmd_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 dac_debug_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $dac_debug_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 dac_data_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $dac_data_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 adc_cmd_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $adc_cmd_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 adc_debug_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $adc_debug_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 adc_data_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $adc_data_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 trig_cmd_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $trig_cmd_fifo_addr_width)}]
} {}
cell xilinx.com:ip:xlconstant:1.1 trig_data_sts_word_padding {
  CONST_VAL 0
  CONST_WIDTH [expr {32 - (6 + $trig_data_fifo_addr_width)}]
} {}


## FIFO declarations
# Each FIFO has a 32-bit data width and 10-bit address width
for {set i 0} {$i < $board_count} {incr i} {

  ## DAC command FIFO
  # DAC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 dac_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 0}]
    DIN_TO [expr {2 * $i + 0}]
  } {
    din cmd_buf_reset
  }
  cell xilinx.com:ip:util_vector_logic n_dac_cmd_fifo_${i}_rst {
    C_SIZE 1
    C_OPERATION not
  } {
    Op1 dac_cmd_fifo_${i}_rst_slice/dout
  }
  cell base:user:reset_sync dac_cmd_fifo_${i}_spi_clk_rst {} {
    clk spi_clk
    rst_in_n n_dac_cmd_fifo_${i}_rst/Res
  }
  # For command buffers, chain the write (aclk) reset off the read (spi_clk) reset so the read
  # side always enters reset first and leaves it first
  cell base:user:reset_sync dac_cmd_fifo_${i}_aclk_rst {} {
    clk aclk
    rst_in_n dac_cmd_fifo_${i}_spi_clk_rst/rst_out_n
  }
  # DAC command FIFO
  cell base:user:fifo_async dac_cmd_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $dac_cmd_fifo_addr_width
  } {
    wr_clk aclk
    wr_rst_n dac_cmd_fifo_${i}_aclk_rst/rst_out_n
    rd_clk spi_clk
    rd_rst_n dac_cmd_fifo_${i}_spi_clk_rst/rst_out_n
    rd_data dac_ch${i}_cmd
    rd_en dac_ch${i}_cmd_rd_en
    empty dac_ch${i}_cmd_empty
  }
  # 32-bit DAC command FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 dac_cmd_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 dac_cmd_fifo_${i}/fifo_count_wr_clk
    In1 dac_cmd_sts_word_padding/dout
    In2 dac_cmd_fifo_${i}/full
    In3 dac_cmd_fifo_${i}/almost_full
    In4 dac_cmd_fifo_${i}/empty
    In5 dac_cmd_fifo_${i}/almost_empty
    In6 const_1/dout
  }

  ## DAC read-side FIFOs (debug and calibration readback), sharing one reset on the board's
  ## DAC data-reset bit so the DMA run teardown (dac_cmd + adc_data resets) leaves them intact
  cell xilinx.com:ip:xlslice:1.0 dac_rd_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 0}]
    DIN_TO [expr {2 * $i + 0}]
  } {
    din data_buf_reset
  }
  cell xilinx.com:ip:util_vector_logic n_dac_rd_fifo_${i}_rst {
    C_SIZE 1
    C_OPERATION not
  } {
    Op1 dac_rd_fifo_${i}_rst_slice/dout
  }
  cell base:user:reset_sync dac_rd_fifo_${i}_spi_clk_rst {} {
    clk spi_clk
    rst_in_n n_dac_rd_fifo_${i}_rst/Res
  }
  # Separate SPI-side sync for the debug FIFO, chained off the data FIFO's so only one flop per
  # domain feeds each FIFO's reset re-sync (avoids CDC-11 fan-out across the crossing)
  cell base:user:reset_sync dac_debug_fifo_${i}_spi_clk_rst {} {
    clk spi_clk
    rst_in_n dac_rd_fifo_${i}_spi_clk_rst/rst_out_n
  }
  cell base:user:reset_sync dac_rd_fifo_${i}_aclk_rst {} {
    clk aclk
    rst_in_n n_dac_rd_fifo_${i}_rst/Res
  }
  # DAC debug FIFO
  cell base:user:fifo_async dac_debug_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $dac_debug_fifo_addr_width
  } {
    wr_clk spi_clk
    wr_rst_n dac_debug_fifo_${i}_spi_clk_rst/rst_out_n
    rd_clk aclk
    rd_rst_n dac_rd_fifo_${i}_aclk_rst/rst_out_n
    wr_data dac_ch${i}_debug
    wr_en dac_ch${i}_debug_wr_en
    full dac_ch${i}_debug_full
  }
  # 32-bit DAC debug FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 dac_debug_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 dac_debug_fifo_${i}/fifo_count_rd_clk
    In1 dac_debug_sts_word_padding/dout
    In2 dac_debug_fifo_${i}/full
    In3 dac_debug_fifo_${i}/almost_full
    In4 dac_debug_fifo_${i}/empty
    In5 dac_debug_fifo_${i}/almost_empty
    In6 const_1/dout
  }
  # DAC data FIFO (calibration readback only)
  cell base:user:fifo_async dac_data_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $dac_data_fifo_addr_width
  } {
    wr_clk spi_clk
    wr_rst_n dac_rd_fifo_${i}_spi_clk_rst/rst_out_n
    rd_clk aclk
    rd_rst_n dac_rd_fifo_${i}_aclk_rst/rst_out_n
    wr_data dac_ch${i}_data
    wr_en dac_ch${i}_data_wr_en
    full dac_ch${i}_data_full
    almost_full dac_ch${i}_data_almost_full
  }
  # 32-bit DAC data FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 dac_data_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 dac_data_fifo_${i}/fifo_count_rd_clk
    In1 dac_data_sts_word_padding/dout
    In2 dac_data_fifo_${i}/full
    In3 dac_data_fifo_${i}/almost_full
    In4 dac_data_fifo_${i}/empty
    In5 dac_data_fifo_${i}/almost_empty
    In6 const_1/dout
  }


  ## ADC command FIFO
  # ADC command FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_cmd_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 1}]
    DIN_TO [expr {2 * $i + 1}]
  } {
    din cmd_buf_reset
  }
  cell xilinx.com:ip:util_vector_logic n_adc_cmd_fifo_${i}_rst {
    C_SIZE 1
    C_OPERATION not
  } {
    Op1 adc_cmd_fifo_${i}_rst_slice/dout
  }
  cell base:user:reset_sync adc_cmd_fifo_${i}_spi_clk_rst {} {
    clk spi_clk
    rst_in_n n_adc_cmd_fifo_${i}_rst/Res
  }
  # For command buffers, chain the write (aclk) reset off the read (spi_clk) reset so the read
  # side always enters reset first and leaves it first
  cell base:user:reset_sync adc_cmd_fifo_${i}_aclk_rst {} {
    clk aclk
    rst_in_n adc_cmd_fifo_${i}_spi_clk_rst/rst_out_n
  }
  # ADC command FIFO
  cell base:user:fifo_async adc_cmd_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $adc_cmd_fifo_addr_width
  } {
    wr_clk aclk
    wr_rst_n adc_cmd_fifo_${i}_aclk_rst/rst_out_n
    rd_clk spi_clk
    rd_rst_n adc_cmd_fifo_${i}_spi_clk_rst/rst_out_n
    rd_data adc_ch${i}_cmd
    rd_en adc_ch${i}_cmd_rd_en
    empty adc_ch${i}_cmd_empty
  }
  # 32-bit ADC command FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_cmd_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 adc_cmd_fifo_${i}/fifo_count_wr_clk
    In1 adc_cmd_sts_word_padding/dout
    In2 adc_cmd_fifo_${i}/full
    In3 adc_cmd_fifo_${i}/almost_full
    In4 adc_cmd_fifo_${i}/empty
    In5 adc_cmd_fifo_${i}/almost_empty
    In6 const_1/dout
  }

  ## ADC debug FIFO, reset with the ADC command FIFO so the DMA run teardown (dac_cmd + adc_data
  ## resets) leaves it intact. Its read (aclk) side releases after the write (spi_clk) side.
  cell base:user:fifo_async adc_debug_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $adc_debug_fifo_addr_width
  } {
    wr_clk spi_clk
    wr_rst_n adc_cmd_fifo_${i}_spi_clk_rst/rst_out_n
    rd_clk aclk
    rd_rst_n adc_cmd_fifo_${i}_aclk_rst/rst_out_n
    wr_data adc_ch${i}_debug
    wr_en adc_ch${i}_debug_wr_en
    full adc_ch${i}_debug_full
  }
  # 32-bit ADC debug FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_debug_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 adc_debug_fifo_${i}/fifo_count_rd_clk
    In1 adc_debug_sts_word_padding/dout
    In2 adc_debug_fifo_${i}/full
    In3 adc_debug_fifo_${i}/almost_full
    In4 adc_debug_fifo_${i}/empty
    In5 adc_debug_fifo_${i}/almost_empty
    In6 const_1/dout
  }

  ## ADC data FIFO
  # ADC data FIFO resetn
  cell xilinx.com:ip:xlslice:1.0 adc_data_fifo_${i}_rst_slice {
    DIN_WIDTH 17
    DIN_FROM [expr {2 * $i + 1}]
    DIN_TO [expr {2 * $i + 1}]
  } {
    din data_buf_reset
  }
  cell xilinx.com:ip:util_vector_logic n_adc_data_fifo_${i}_rst {
    C_SIZE 1
    C_OPERATION not
  } {
    Op1 adc_data_fifo_${i}_rst_slice/dout
  }
  cell base:user:reset_sync adc_data_fifo_${i}_spi_clk_rst {} {
    clk spi_clk
    rst_in_n n_adc_data_fifo_${i}_rst/Res
  }
  cell base:user:reset_sync adc_data_fifo_${i}_aclk_rst {} {
    clk aclk
    rst_in_n n_adc_data_fifo_${i}_rst/Res
  }
  # ADC data FIFO
  cell base:user:fifo_async adc_data_fifo_$i {
    DATA_WIDTH 32
    ADDR_WIDTH $adc_data_fifo_addr_width
  } {
    wr_clk spi_clk
    wr_rst_n adc_data_fifo_${i}_spi_clk_rst/rst_out_n
    rd_clk aclk
    rd_rst_n adc_data_fifo_${i}_aclk_rst/rst_out_n
    wr_data adc_ch${i}_data
    wr_en adc_ch${i}_data_wr_en
    full adc_ch${i}_data_full
    almost_full adc_ch${i}_data_almost_full
  }
  # 32-bit ADC data FIFO status word
  cell xilinx.com:ip:xlconcat:2.1 adc_data_fifo_${i}_sts_word {
    NUM_PORTS 7
  } {
    In0 adc_data_fifo_${i}/fifo_count_rd_clk
    In1 adc_data_sts_word_padding/dout
    In2 adc_data_fifo_${i}/full
    In3 adc_data_fifo_${i}/almost_full
    In4 adc_data_fifo_${i}/empty
    In5 adc_data_fifo_${i}/almost_empty
    In6 const_1/dout
  }

  ## DMA datapath: per-board DAC-command write bridge, ADC-data packetizer, and the
  ## datapath_mode select against the PIO board FIFO array. datapath_mode[i] chooses the
  ## owner of the shared dac_cmd write port and adc_data read port (0 = PIO, 1 = DMA) and
  ## hard-locks the idle owner off the FIFO.
  cell xilinx.com:ip:xlslice:1.0 datapath_mode_${i}_slice {
    DIN_WIDTH 8
    DIN_FROM $i
    DIN_TO $i
  } {
    din datapath_mode
  }
  # DAC command DMA write bridge (AXIS from MM2S -> dac_cmd FIFO write). Write-only, so its
  # read side is tied off; blocking tready (ALWAYS_READY 0) so MM2S backpressures on full.
  cell base:user:axis_fifo_bridge dac_cmd_dma_bridge_${i} {
    AXIS_DATA_WIDTH 32
    ENABLE_WRITE 1
    ENABLE_READ 0
    ALWAYS_READY 0
    ALWAYS_VALID 0
  } {
    aclk aclk
    aresetn dac_cmd_fifo_${i}_aclk_rst/rst_out_n
    s_axis dac_ch${i}_dma
    fifo_rd_data const_0_32/dout
    fifo_empty const_1/dout
  }
  # ADC data DMA packetizer (adc_data FIFO read -> AXIS to the S2MM mux). tdest = board index.
  cell shim:user:adc_packetizer adc_data_packetizer_${i} {
    DATA_WIDTH 32
    DEST_WIDTH 8
    FIFO_COUNT_WIDTH [expr {$adc_data_fifo_addr_width + 1}]
    MAX_PACKET_WORDS $adc_packet_words
    BOARD_INDEX $i
  } {
    aclk aclk
    aresetn adc_data_fifo_${i}_aclk_rst/rst_out_n
    m_axis adc_ch${i}_dma
  }
  # Per-board 2:1 select between the PIO array and the DMA adapters on the two shared
  # FIFO ports, hard-locking the idle owner (full/empty forced on the non-owner). The PIO
  # side is wired to the board FIFO array below.
  cell shim:user:datapath_mux datapath_mux_${i} {
    DATA_WIDTH 32
    COUNT_WIDTH [expr {$adc_data_fifo_addr_width + 1}]
  } {
    datapath_mode datapath_mode_${i}_slice/dout
    cmd_wr_data dac_cmd_fifo_${i}/wr_data
    cmd_wr_en dac_cmd_fifo_${i}/wr_en
    cmd_full dac_cmd_fifo_${i}/full
    cmd_dma_wr_data dac_cmd_dma_bridge_${i}/fifo_wr_data
    cmd_dma_wr_en dac_cmd_dma_bridge_${i}/fifo_wr_en
    cmd_dma_full dac_cmd_dma_bridge_${i}/fifo_full
    data_rd_data adc_data_fifo_${i}/rd_data
    data_empty adc_data_fifo_${i}/empty
    data_count adc_data_fifo_${i}/fifo_count_rd_clk
    data_rd_en adc_data_fifo_${i}/rd_en
    data_dma_rd_data adc_data_packetizer_${i}/fifo_rd_data
    data_dma_empty adc_data_packetizer_${i}/fifo_empty
    data_dma_count adc_data_packetizer_${i}/fifo_count_rd_clk
    data_dma_rd_en adc_data_packetizer_${i}/fifo_rd_en
  }
}

## All-board PIO register array: one AXI-Lite window, a 4 KB page per board, fronting every
## board's six FIFO lanes plus its capture rate lane (see cores/base/axi_board_fifo_array). The dac_cmd write and adc_data
## read reach their FIFOs through each board's datapath_mux; mode_block takes datapath_mode
## directly, so in DMA mode a PIO access to those two lanes is discarded and raises mode_viol.
## Per-board ports are eight wide; boards past board_count are tied off and never addressed.
cell base:user:axi_board_fifo_array board_fifo_array {
  NUM_BOARDS $board_count
  AXI_ADDR_WIDTH 16
  AXI_DATA_WIDTH 32
} {
  aclk aclk
  aresetn aresetn
  S_AXI board_fifo_axi
  mode_block datapath_mode
  mode_viol mode_viol
  dac_cmd_overflow dac_cmd_buf_overflow
  adc_cmd_overflow adc_cmd_buf_overflow
  dac_data_underflow dac_data_buf_underflow
  adc_data_underflow adc_data_buf_underflow
}
# Gather per-board FIFO resets and full/empty flags into the array's 8-bit inputs
foreach {port tie} {
  dac_cmd_resetn  const_0
  dac_data_resetn const_0
  adc_cmd_resetn  const_0
  adc_data_resetn const_0
  dac_cmd_full    const_0
  adc_cmd_full    const_0
  dac_debug_empty const_1
  dac_data_empty  const_1
  adc_debug_empty const_1
  adc_data_empty  const_1
} {
  cell xilinx.com:ip:xlconcat:2.1 board_fifo_array_${port}_concat {
    NUM_PORTS 8
  } {
    dout board_fifo_array/${port}
  }
  for {set i $board_count} {$i < 8} {incr i} {
    wire board_fifo_array_${port}_concat/In${i} ${tie}/dout
  }
}
# Gather per-board read data into the array's 256-bit inputs
foreach port {dac_debug_rd_data dac_data_rd_data adc_debug_rd_data adc_data_rd_data} {
  cell xilinx.com:ip:xlconcat:2.1 board_fifo_array_${port}_concat {
    NUM_PORTS 8
  } {
    dout board_fifo_array/${port}
  }
  for {set i $board_count} {$i < 8} {incr i} {
    wire board_fifo_array_${port}_concat/In${i} const_0_32/dout
  }
}
for {set i 0} {$i < $board_count} {incr i} {
  # The array's sticky flags run on aclk, so they clear from each FIFO's aclk-side reset
  wire board_fifo_array_dac_cmd_resetn_concat/In${i} dac_cmd_fifo_${i}_aclk_rst/rst_out_n
  wire board_fifo_array_dac_data_resetn_concat/In${i} dac_rd_fifo_${i}_aclk_rst/rst_out_n
  wire board_fifo_array_adc_cmd_resetn_concat/In${i} adc_cmd_fifo_${i}_aclk_rst/rst_out_n
  wire board_fifo_array_adc_data_resetn_concat/In${i} adc_data_fifo_${i}_aclk_rst/rst_out_n
  wire board_fifo_array_dac_cmd_full_concat/In${i} datapath_mux_${i}/cmd_pio_full
  wire board_fifo_array_adc_cmd_full_concat/In${i} adc_cmd_fifo_${i}/full
  wire board_fifo_array_dac_debug_empty_concat/In${i} dac_debug_fifo_${i}/empty
  wire board_fifo_array_dac_data_empty_concat/In${i} dac_data_fifo_${i}/empty
  wire board_fifo_array_adc_debug_empty_concat/In${i} adc_debug_fifo_${i}/empty
  wire board_fifo_array_adc_data_empty_concat/In${i} datapath_mux_${i}/data_pio_empty
  wire board_fifo_array_dac_debug_rd_data_concat/In${i} dac_debug_fifo_${i}/rd_data
  wire board_fifo_array_dac_data_rd_data_concat/In${i} dac_data_fifo_${i}/rd_data
  wire board_fifo_array_adc_debug_rd_data_concat/In${i} adc_debug_fifo_${i}/rd_data
  wire board_fifo_array_adc_data_rd_data_concat/In${i} datapath_mux_${i}/data_pio_rd_data
  # Shared command write data, and this board's slice of each enable vector
  wire board_fifo_array/cmd_wr_data datapath_mux_${i}/cmd_pio_wr_data
  wire board_fifo_array/cmd_wr_data adc_cmd_fifo_${i}/wr_data
  foreach {port dest} [list \
    dac_cmd_wr_en   datapath_mux_${i}/cmd_pio_wr_en \
    adc_cmd_wr_en   adc_cmd_fifo_${i}/wr_en \
    dac_debug_rd_en dac_debug_fifo_${i}/rd_en \
    dac_data_rd_en  dac_data_fifo_${i}/rd_en \
    adc_debug_rd_en adc_debug_fifo_${i}/rd_en \
    adc_data_rd_en  datapath_mux_${i}/data_pio_rd_en \
  ] {
    cell xilinx.com:ip:xlslice:1.0 board_fifo_array_${port}_${i}_slice {
      DIN_WIDTH 8
      DIN_FROM $i
      DIN_TO $i
    } {
      din board_fifo_array/${port}
      dout $dest
    }
  }
}

## Trigger command FIFO
# Trigger command FIFO resetn
cell xilinx.com:ip:xlslice:1.0 trig_cmd_fifo_rst_slice {
  DIN_WIDTH 17
  DIN_FROM 16
  DIN_TO 16
} {
  din cmd_buf_reset
}
cell xilinx.com:ip:util_vector_logic n_trig_cmd_fifo_rst {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 trig_cmd_fifo_rst_slice/dout
}
cell base:user:reset_sync trig_cmd_fifo_spi_clk_rst {} {
  clk spi_clk
  rst_in_n n_trig_cmd_fifo_rst/Res
}
cell base:user:reset_sync trig_cmd_fifo_aclk_rst {} {
  clk aclk
  rst_in_n n_trig_cmd_fifo_rst/Res
}
# Trigger command FIFO
cell base:user:fifo_async trig_cmd_fifo {
  DATA_WIDTH 32
  ADDR_WIDTH $trig_cmd_fifo_addr_width
} {
  wr_clk aclk
  wr_rst_n trig_cmd_fifo_aclk_rst/rst_out_n
  rd_clk spi_clk
  rd_rst_n trig_cmd_fifo_spi_clk_rst/rst_out_n
  rd_data trig_cmd
  rd_en trig_cmd_rd_en
  empty trig_cmd_empty
}
# 32-bit trigger command FIFO status word
cell xilinx.com:ip:xlconcat:2.1 trig_cmd_fifo_sts_word {
  NUM_PORTS 7
} {
  In0 trig_cmd_fifo/fifo_count_wr_clk
  In1 trig_cmd_sts_word_padding/dout
  In2 trig_cmd_fifo/full
  In3 trig_cmd_fifo/almost_full
  In4 trig_cmd_fifo/empty
  In5 trig_cmd_fifo/almost_empty
  In6 const_1/dout
}

## Trigger data FIFO
# Trigger data FIFO resetn
cell xilinx.com:ip:xlslice:1.0 trig_data_fifo_rst_slice {
  DIN_WIDTH 17
  DIN_FROM 16
  DIN_TO 16
} {
  din data_buf_reset
}
cell xilinx.com:ip:util_vector_logic n_trig_data_fifo_rst {
  C_SIZE 1
  C_OPERATION not
} {
  Op1 trig_data_fifo_rst_slice/dout
}
cell base:user:reset_sync trig_data_fifo_spi_clk_rst {} {
  clk spi_clk
  rst_in_n n_trig_data_fifo_rst/Res
}
# Chain the read (aclk) reset off the write (spi_clk) reset, so the read side releases after
# the write side
cell base:user:reset_sync trig_data_fifo_aclk_rst {} {
  clk aclk
  rst_in_n trig_data_fifo_spi_clk_rst/rst_out_n
}
# Trigger data FIFO
cell base:user:fifo_async trig_data_fifo {
  DATA_WIDTH 32
  ADDR_WIDTH $trig_data_fifo_addr_width
} {
  wr_clk spi_clk
  wr_rst_n trig_data_fifo_spi_clk_rst/rst_out_n
  rd_clk aclk
  rd_rst_n trig_data_fifo_aclk_rst/rst_out_n
  wr_data trig_data
  wr_en trig_data_wr_en
  full trig_data_full
  almost_full trig_data_almost_full
}
# 32-bit trigger data FIFO status word
cell xilinx.com:ip:xlconcat:2.1 trig_data_fifo_sts_word {
  NUM_PORTS 7
} {
  In0 trig_data_fifo/fifo_count_rd_clk
  In1 trig_data_sts_word_padding/dout
  In2 trig_data_fifo/full
  In3 trig_data_fifo/almost_full
  In4 trig_data_fifo/empty
  In5 trig_data_fifo/almost_empty
  In6 const_1/dout
}

## Trigger FIFO AXI interface
cell base:user:axi_fifo_bridge trig_fifo_axi_bridge {
  AXI_ADDR_WIDTH 32
  AXI_DATA_WIDTH 32
} {
  aclk aclk
  wr_resetn trig_cmd_fifo_aclk_rst/rst_out_n
  rd_resetn trig_data_fifo_aclk_rst/rst_out_n
  S_AXI trig_axi
  fifo_wr_data trig_cmd_fifo/wr_data
  fifo_wr_en trig_cmd_fifo/wr_en
  fifo_full trig_cmd_fifo/full
  fifo_rd_data trig_data_fifo/rd_data
  fifo_rd_en trig_data_fifo/rd_en
  fifo_empty trig_data_fifo/empty
}
# The trigger FIFO is PIO in both datapath modes, so it is never mode-blocked.
wire trig_fifo_axi_bridge/wr_mode_block const_0/dout
wire trig_fifo_axi_bridge/rd_mode_block const_0/dout

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

# Wire used board data FIFO status words to the concatenation (DAC debug, ADC data per board)
for {set i 0} {$i < $board_count} {incr i} {
  wire data_fifo_sts_concat/In[expr {2 * $i + 0}] dac_debug_fifo_${i}_sts_word/dout
  wire data_fifo_sts_concat/In[expr {2 * $i + 1}] adc_data_fifo_${i}_sts_word/dout
}
# Wire unused board data FIFO status words to the concatenation
for {set i $board_count} {$i < 8} {incr i} {
  wire data_fifo_sts_concat/In[expr {2 * $i + 0}] empty_sts_word/dout
  wire data_fifo_sts_concat/In[expr {2 * $i + 1}] empty_sts_word/dout
}
# Wire trigger data FIFO status word to the concatenation
wire data_fifo_sts_concat/In16 trig_data_fifo_sts_word/dout

# ADC debug and DAC data (calibration readback) FIFO status words, one per board
foreach fifo {adc_debug dac_data} {
  cell xilinx.com:ip:xlconcat:2.1 ${fifo}_fifo_sts_concat {
    NUM_PORTS 8
  } {
    dout ${fifo}_fifo_sts
  }
  for {set i 0} {$i < $board_count} {incr i} {
    wire ${fifo}_fifo_sts_concat/In${i} ${fifo}_fifo_${i}_sts_word/dout
  }
  for {set i $board_count} {$i < 8} {incr i} {
    wire ${fifo}_fifo_sts_concat/In${i} empty_sts_word/dout
  }
}

## Overflow and underflow signals
# The per-board DAC/ADC FIFO flags and mode_viol come straight off the board FIFO array (above).
# Wire trigger command/data FIFO overflow and underflow signals
wire trig_cmd_buf_overflow trig_fifo_axi_bridge/fifo_overflow
wire trig_data_buf_underflow trig_fifo_axi_bridge/fifo_underflow

## ADC capture rate control: one adc_rate_ctrl block holds the per-board capture backlog
## counters. Each board's adc_packetizer reports its emit beat (word_emitted) into the block and
## takes back a 1-bit accumulate decision (dense vs eager) -- the block compares each backlog
## against the fixed packet-size threshold (PACKET_WORDS). Software reports its reads and reads
## back the backlog through each board's rate lane in the board FIFO array. The block's per-board
## backlog is cleared by that board's adc_data FIFO reset, so one buffer reset flushes the FIFO,
## the packetizer, and this count together. The block always carries eight boards to match the
## array's port width; unused boards are held in reset at backlog 0.
# Gather each board's packetizer emit beat into the block's write-side input.
cell xilinx.com:ip:xlconcat:2.1 word_emitted_concat {
  NUM_PORTS 8
} {}
for {set i 0} {$i < $board_count} {incr i} {
  wire word_emitted_concat/In${i} adc_data_packetizer_${i}/word_emitted
}
for {set i $board_count} {$i < 8} {incr i} {
  wire word_emitted_concat/In${i} const_0/dout
}
# Gather each board's adc_data FIFO reset as that board's backlog clear; hold unused boards reset.
cell xilinx.com:ip:xlconcat:2.1 backlog_rst_n_concat {
  NUM_PORTS 8
} {}
for {set i 0} {$i < $board_count} {incr i} {
  wire backlog_rst_n_concat/In${i} adc_data_fifo_${i}_aclk_rst/rst_out_n
}
for {set i $board_count} {$i < 8} {incr i} {
  wire backlog_rst_n_concat/In${i} const_0/dout
}
# The rate-control block.
cell shim:user:adc_rate_ctrl adc_rate_ctrl {
  NUM_BOARDS 8
  PACKET_WORDS $adc_packet_words
} {
  aclk aclk
  aresetn aresetn
  word_emitted word_emitted_concat/dout
  backlog_rst_n backlog_rst_n_concat/dout
  read_delta_en board_fifo_array/rate_wr_en
  read_delta board_fifo_array/cmd_wr_data
  backlog_out board_fifo_array/rate_backlog
}
# Fan the per-board accumulate decision out to each packetizer.
for {set i 0} {$i < $board_count} {incr i} {
  cell xilinx.com:ip:xlslice:1.0 accumulate_${i}_slice {
    DIN_WIDTH 8
    DIN_FROM $i
    DIN_TO $i
  } {
    din adc_rate_ctrl/accumulate
  }
  wire accumulate_${i}_slice/dout adc_data_packetizer_${i}/accumulate
}
