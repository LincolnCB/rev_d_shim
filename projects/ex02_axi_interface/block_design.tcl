# No external FPGA ports are used in this project.

############# General setup #############

## Instantiate the processing system

# Create the PS (processing_system7)
# Config:
# - Unused AXI ACP port disabled (or it will complain that the port's clock is not connected)
# Connections:
# - GP AXI 0 (Manager) clock is connected to the processing system's first clock, FCLK_CLK0
init_ps ps {
  PCW_USE_S_AXI_ACP 0
} {
  M_AXI_GP0_ACLK ps/FCLK_CLK0
}


## Create the reset manager
# Create proc_sys_reset
# - Resetn is constant low (active high)
cell xilinx.com:ip:proc_sys_reset ps_rst {} {
  ext_reset_in ps/FCLK_RESET0_N
  slowest_sync_clk ps/FCLK_CLK0
}

################ AXI Interfaces #############

### AXI4 SmartConnect to branch to multiple AXI4 interfaces
#   - One Subordinate/Slave interface
#   - Four Manager/Master interfaces
#   - Connects to the processing system's GP AXI 0 interface, and clock, and the reset manager's reset
cell xilinx.com:ip:smartconnect:1.0 axi_smc {
  NUM_SI 1
  NUM_MI 4
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S00_AXI /ps/M_AXI_GP0
}

### Read/Write "CFG" config register
## Create a read/write register with an AXI4-Lite interface
# - Config register width is 96 bits
#     63:0 are used for NAND example
#     95:64 are used for FIFO control
# - Connect the axi_cfg to the processing system's clock and the reset manager's reset
# - Connect the axi_cfg to the processing system's GP AXI 0 interface through the 
#    first AXI4 SmartConnect Manager/Master port
cell pavel-demin:user:axi_cfg_register axi_cfg {
  CFG_DATA_WIDTH 96
  AXI_ADDR_WIDTH 32
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S_AXI axi_smc/M00_AXI
}
# Assign the address of the axi_cfg in the PS address space
# - Offset: 0x40000000
# - Range: 128 (minimum range size for Vivado, to 0x4000007F)
# - End port: axi_cfg/S_AXI
# - Start port: ps/M_AXI_GP0 (we don't need to include intermediate ports like the axi_smc)
addr 0x40000000 128 axi_cfg/S_AXI ps/M_AXI_GP0

### Read "STS" status register
## Create a read-only status register with an AXI4-Lite interface
# - Status register width is 64 bits
#     31:0 are used for NAND example
#     63:32 are used for FIFO status
# - Connect the axi_sts to the processing system's clock and the reset manager's reset
# - Connect the axi_sts to the processing system's GP AXI 0 interface through the 
#    second AXI4 SmartConnect Manager/Master port
cell pavel-demin:user:axi_sts_register axi_sts {
  STS_DATA_WIDTH 64
  AXI_ADDR_WIDTH 32
} {
  aclk ps/FCLK_CLK0
  aresetn ps_rst/peripheral_aresetn
  S_AXI axi_smc/M01_AXI
}
# Assign the address of the axi_sts in the PS address space
# - Offset: 0x41000000
#    ( Note that offset needs to be at least as big as both of:                                   )
#    ( - The minimum Vivado range size (128 bytes, 0x80)                                          )
#    ( - The pagesize of the virtual memory (4KiB, 0x1000)                                        )
#    ( We use 0x41000000 to get far past both these values because we have address space to spare )
# - Range: 128 (minimum range size for Vivado, to 0x4100007F)
# - End port: axi_sts/S_AXI
# - Start port: ps/M_AXI_GP0
addr 0x41000000 128 axi_sts/S_AXI ps/M_AXI_GP0

### FIFO with AXI4 interface
# Slice off CFG register bits 95:64 for FIFO control
cell xilinx.com:ip:xlslice:1.0 axi_fifo_cfg {
  DIN_WIDTH 96 DIN_FROM 95 DIN_TO 64
} {
  din axi_cfg/cfg_data
}
# Create a FIFO with an AXI4-Lite interface using a custom module
# This command will source its code from the `modules/fifo.tcl` file
# - Connect the clock to the processing system's clock, FCLK_CLK0
# - Connect the cfg word to the sliced FIFO control bits
# - Connect the FIFO's AXI4 interface to the third AXI SmartConnect Manager/Master port
# - The reset signal is handled internally, using the FIFO's cfg word bit 0
module fifo axi_fifo_module {
  aclk ps/FCLK_CLK0
  cfg_word axi_fifo_cfg/dout
  s_axi axi_smc/M02_AXI
}
# Assign the address of the axi_fifo in the PS address space
# - Offset: 0x42000000
# - Range: 128 (minimum range size for Vivado, to 0x4200007F)
# - End port: axi_fifo/s_axi
# - Start port: ps/M_AXI_GP0
addr 0x42000000 128 axi_fifo_module/S_AXI ps/M_AXI_GP0

### BRAM with AXI4 interface
# Create an AXI BRAM controller
# - Set SINGLE_PORT_BRAM to 1 to interface with a single port BRAM
# - Connect the clock to the processing system's clock, FCLK_CLK0
# - Connect the reset to the reset manager's peripheral reset
# - Connect the BRAM controller's AXI interface to the fourth AXI SmartConnect Manager/Master port
cell xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl {
  SINGLE_PORT_BRAM 1
} {
  s_axi_aclk ps/FCLK_CLK0
  s_axi_aresetn ps_rst/peripheral_aresetn
  S_AXI axi_smc/M03_AXI
}
# Create a BRAM block using Vivado's Block Memory Generator
# - Set the memory type to Single_Port_RAM
# - Set the BRAM block to BRAM_Controller mode 
#     (for when interfacing directly with Xilinx BRAM controller. Use Stand_Alone if using your own controller)
# - Set the byte size to 8 bits
# - Set the write width to 32 bits
# - Set the write depth to 16384 (512Kib, 524288 bits)
# - Set the REGISTER_PORTA_OUTPUT_OF_MEMORY_PRIMITIVES to false to avoid extra registers
# - Connect the BRAM controller's port A to the BRAM block's port A
# - The BRAM block will use 16 36Kib blocks in the Zynq
#     ( There are 140 36Kib blocks available in the Zynq-7020           )
#     ( Note that 32x1024 of memory will use the same as 36x1024        )
#     (  because of the available shape options for the BRAM            )
#     ( This means that if you increase the WRITE_DEPTH_A by 1,         )
#     (  it will use 17 blocks, even though it's still under 36Kib x 16 )
#     ( Utlization report is in the `tmp_reports` directory once built  )
#     ( *If you do change this, make sure to update the address range*  )
cell xilinx.com:ip:blk_mem_gen bram {
  MEMORY_TYPE Single_Port_RAM
  USE_BRAM_BLOCK BRAM_Controller
  USE_BYTE_WRITE_ENABLE true
  BYTE_SIZE 8
  WRITE_WIDTH_A 32
  WRITE_DEPTH_A 16384
  REGISTER_PORTA_OUTPUT_OF_MEMORY_PRIMITIVES false
} {
  BRAM_PORTA axi_bram_ctrl/BRAM_PORTA
}
# Assign the address of the axi_bram_ctrl in the PS address space
# - Offset: 0x43000000
# - Range: 65536 = 64K (32 x 16384 / 8, to 0x43010000)
# - End port: axi_bram_ctrl/S_AXI
# - Start port: ps/M_AXI_GP0
addr 0x43000000 64K axi_bram_ctrl/S_AXI ps/M_AXI


############# Simple Register Tests #############
# Uses the first 64 bits of the CFG data and the first 32 bits of the STS data
# This is a simple test to verify that the registers are working
# The first 32 bits of the STS data are the bitwise NAND 
#   of the first 32 bits and the second 32 bits of the CFG data

# Slice off CFG 63:0 for the concatenated NAND input
cell xilinx.com:ip:xlslice:1.0 nand_module_cfg {
  DIN_WIDTH 96 DIN_FROM 63 DIN_TO 0
} {
  din axi_cfg/cfg_data
}
# Add a vector nand gate (see source file)
module nand nand_module {
  nand_din_concat nand_module_cfg/dout
}


############# Concatenate all status signals #############
cell xilinx.com:ip:xlconcat:2.1 sts_concat {
  NUM_PORTS 2
} {
  In0 nand_module/nand_res
  In1 axi_fifo_module/sts_word
  dout axi_sts/sts_data
}
