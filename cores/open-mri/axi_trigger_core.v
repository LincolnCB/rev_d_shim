
`timescale 1 ns / 1 ps

// This module is for managing external triggers
//
module axi_trigger_core #
(
  parameter integer C_S_AXI_DATA_WIDTH = 32,
  parameter integer C_S_AXI_ADDR_WIDTH = 2
)
(
  // Serial Pins for the attenuator
  output wire           trigger_out,
  output wire           stretched_trigger_out,
  input wire            trigger_in,
  input wire            aclk,
  input wire            aresetn,

  // AXI4LITE Interface
  // Do not modify the ports beyond this line

  // Global Clock Signal
  input   wire                                S_AXI_ACLK,
  // Global Reset Signal. This Signal is Active LOW
  input   wire                                S_AXI_ARESETN,
  // Write address (issued by master, acceped by Slave)
  input   wire [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_AWADDR,
  // Write channel Protection type. This signal indicates the
  // privilege and security level of the transaction, and whether
  // the transaction is a data access or an instruction access.
  input   wire [2 : 0]                        S_AXI_AWPROT,
  // Write address valid. This signal indicates that the master signaling
  // valid write address and control information.
  input   wire                                S_AXI_AWVALID,
  // Write address ready. This signal indicates that the slave is ready
  // to accept an address and associated control signals.
  output  wire                                S_AXI_AWREADY,
  // Write data (issued by master, acceped by Slave)
  input   wire [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_WDATA,
  // Write strobes. This signal indicates which byte lanes hold
  // valid data. There is one write strobe bit for each eight
  // bits of the write data bus.
  input   wire [(C_S_AXI_DATA_WIDTH/8)-1 : 0] S_AXI_WSTRB,
  // Write valid. This signal indicates that valid write
  // data and strobes are available.
  input   wire                                S_AXI_WVALID,
  // Write ready. This signal indicates that the slave
  // can accept the write data.
  output  wire                                S_AXI_WREADY,
  // Write response. This signal indicates the status
  // of the write transaction.
  output  wire [1 : 0]                        S_AXI_BRESP,
  // Write response valid. This signal indicates that the channel
  // is signaling a valid write response.
  output  wire                                S_AXI_BVALID,
  // Response ready. This signal indicates that the master
  // can accept a write response.
  input   wire                                S_AXI_BREADY,
  // Read address (issued by master, acceped by Slave)
  input   wire [C_S_AXI_ADDR_WIDTH-1 : 0]     S_AXI_ARADDR,
  // Protection type. This signal indicates the privilege
  // and security level of the transaction, and whether the
  // transaction is a data access or an instruction access.
  input   wire [2 : 0]                        S_AXI_ARPROT,
  // Read address valid. This signal indicates that the channel
  // is signaling valid read address and control information.
  input   wire                                S_AXI_ARVALID,
  // Read address ready. This signal indicates that the slave is
  // ready to accept an address and associated control signals.
  output  wire                                S_AXI_ARREADY,
    // Read data (issued by slave)
  output  wire [C_S_AXI_DATA_WIDTH-1 : 0]     S_AXI_RDATA,
  // Read response. This signal indicates the status of the
  // read transfer.
  output  wire [1 : 0]                        S_AXI_RRESP,
  // Read valid. This signal indicates that the channel is
  // signaling the required read data.
  output  wire                                S_AXI_RVALID,
  // Read ready. This signal indicates that the master can
  // accept the read data and response information.
  input   wire                                S_AXI_RREADY
);
  // AXI4LITE signals
  reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_awaddr;
  reg                            axi_awready;
  reg                            axi_wready;
  reg [1 : 0]                    axi_bresp;
  reg                            axi_bvalid;
  reg [C_S_AXI_ADDR_WIDTH-1 : 0] axi_araddr;
  reg                            axi_arready;
  reg [C_S_AXI_DATA_WIDTH-1 : 0] axi_rdata;
  reg [1 : 0]                    axi_rresp;
  reg                            axi_rvalid;

  // Example-specific design signals
  // local parameter for addressing 32 bit / 64 bit C_S_AXI_DATA_WIDTH
  // ADDR_LSB is used for addressing 32/64 bit registers/memories
  // ADDR_LSB = 2 for 32 bits (n downto 2)
  // ADDR_LSB = 3 for 64 bits (n downto 3)
  localparam integer       ADDR_LSB = (C_S_AXI_DATA_WIDTH/32) + 1;
  localparam integer       OPT_MEM_ADDR_BITS = 3;
  //----------------------------------------------
  //-- Signals for user logic register space example
  //------------------------------------------------
  //-- Number of Slave Registers 4
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg0;
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg1;
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg2;
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg3;
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg4;
  reg [C_S_AXI_DATA_WIDTH-1:0]   slv_reg5;

  wire         slv_reg_rden;
  wire         slv_reg_wren;
  reg [C_S_AXI_DATA_WIDTH-1:0]   reg_data_out;
  integer         byte_index;

  // I/O Connections assignments

  assign S_AXI_AWREADY = axi_awready;
  assign S_AXI_WREADY  = axi_wready;
  assign S_AXI_BRESP   = axi_bresp;
  assign S_AXI_BVALID  = axi_bvalid;
  assign S_AXI_ARREADY = axi_arready;
  assign S_AXI_RDATA   = axi_rdata;
  assign S_AXI_RRESP   = axi_rresp;
  assign S_AXI_RVALID  = axi_rvalid;

  // Implement axi_awready generation
  // axi_awready is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_awready is
  // de-asserted when reset is low.

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_awready <= 1'b0;
    end
    else
    begin
      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID)
      begin
        // slave is ready to accept write address when
        // there is a valid write address and write data
        // on the write address and data bus. This design
        // expects no outstanding transactions.
        axi_awready <= 1'b1;
      end
      else
      begin
        axi_awready <= 1'b0;
      end
    end
  end

  // Implement axi_awaddr latching
  // This process is used to latch the address when both
  // S_AXI_AWVALID and S_AXI_WVALID are valid.

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_awaddr <= 0;
    end
    else
    begin
      if (~axi_awready && S_AXI_AWVALID && S_AXI_WVALID)
      begin
        // Write Address latching
        axi_awaddr <= S_AXI_AWADDR;
      end
    end
  end

  // Implement axi_wready generation
  // axi_wready is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_AWVALID and S_AXI_WVALID are asserted. axi_wready is
  // de-asserted when reset is low.

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_wready <= 1'b0;
    end
    else
    begin
      if (~axi_wready && S_AXI_WVALID && S_AXI_AWVALID)
      begin
        // slave is ready to accept write data when
        // there is a valid write address and write data
        // on the write address and data bus. This design
        // expects no outstanding transactions.
        axi_wready <= 1'b1;
      end
      else
      begin
        axi_wready <= 1'b0;
      end
    end
  end

  // Implement memory mapped register select and write logic generation
  // The write data is accepted and written to memory mapped registers when
  // axi_awready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted. Write strobes are used to
  // select byte enables of slave registers while writing.
  // These registers are cleared when reset (active low) is applied.
  // Slave register write enable is asserted when valid address and data are available
  // and the slave is ready to accept the write address and write data.
  assign slv_reg_wren = axi_wready && S_AXI_WVALID && axi_awready && S_AXI_AWVALID;

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      slv_reg0 <= 8'h00;
      //slv_reg1 <= 8'h10;
      //slv_reg2 <= 8'h20;
      //slv_reg3 <= 8'h30;
      //slv_reg4 <= 8'h40;
      //slv_reg5 <= 8'h50;
      //slv_reg6 <= 8'h60;
      //slv_reg7 <= 8'h70;
      //slv_reg8 <= 8'h80;
      //slv_reg9 <= 8'h90;
      //slv_reg10 <= 8'h77;
    end
    else
    begin
      if (slv_reg_wren)
      begin
        case (axi_awaddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
          4'h0:
            for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1)
              if (S_AXI_WSTRB[byte_index] == 1)
              begin
                // Respective byte enables are asserted as per write strobes
                // Slave register 0
                slv_reg0[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
              end
          4'h1:
            for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1)
              if (S_AXI_WSTRB[byte_index] == 1)
              begin
                // Respective byte enables are asserted as per write strobes
                // Slave register 1
                slv_reg1[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
              end
          4'h2:
            for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1)
              if (S_AXI_WSTRB[byte_index] == 1)
              begin
                // Respective byte enables are asserted as per write strobes
                // Slave register 2
                slv_reg2[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
              end
          4'h3:
            for (byte_index = 0; byte_index <= (C_S_AXI_DATA_WIDTH/8)-1; byte_index = byte_index+1)
              if (S_AXI_WSTRB[byte_index] == 1)
              begin
                // Respective byte enables are asserted as per write strobes
                // Slave register 3
                slv_reg3[(byte_index*8) +: 8] <= S_AXI_WDATA[(byte_index*8) +: 8];
              end
          default:
            begin
              //slv_reg0 <= slv_reg0;
              //slv_reg1 <= slv_reg1;
              //slv_reg2 <= slv_reg2;
              //slv_reg3 <= slv_reg3;
            end
        endcase
      end
    end
  end

  // Implement write response logic generation
  // The write response and response valid signals are asserted by the slave
  // when axi_wready, S_AXI_WVALID, axi_wready and S_AXI_WVALID are asserted.
  // This marks the acceptance of address and indicates the status of
  // write transaction.

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_bvalid <= 0;
      axi_bresp <= 2'b0;
    end
    else
    begin
      if (axi_awready && S_AXI_AWVALID && ~axi_bvalid && axi_wready && S_AXI_WVALID)
      begin
        // indicates a valid write response is available
        axi_bvalid <= 1'b1;
        axi_bresp <= 2'b0; // 'OKAY' response
      end
      else
      begin
        if (S_AXI_BREADY && axi_bvalid)
        begin
          //check if bready is asserted while bvalid is high)
          //(there is a possibility that bready is always asserted high)
          axi_bvalid <= 1'b0;
        end
      end
    end
  end

  // Implement axi_arready generation
  // axi_arready is asserted for one S_AXI_ACLK clock cycle when
  // S_AXI_ARVALID is asserted. axi_awready is
  // de-asserted when reset (active low) is asserted.
  // The read address is also latched when S_AXI_ARVALID is
  // asserted. axi_araddr is reset to zero on reset assertion.

  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_arready <= 1'b0;
      axi_araddr <= 32'b0;
    end
    else
    begin
      if (~axi_arready && S_AXI_ARVALID)
      begin
        // indicates that the slave has acceped the valid read address
        axi_arready <= 1'b1;
        // Read address latching
        axi_araddr <= S_AXI_ARADDR;
      end
      else
      begin
        axi_arready <= 1'b0;
      end
    end
  end

  // Implement axi_arvalid generation
  // axi_rvalid is asserted for one S_AXI_ACLK clock cycle when both
  // S_AXI_ARVALID and axi_arready are asserted. The slave registers
  // data are available on the axi_rdata bus at this instance. The
  // assertion of axi_rvalid marks the validity of read data on the
  // bus and axi_rresp indicates the status of read transaction.axi_rvalid
  // is deasserted on reset (active low). axi_rresp and axi_rdata are
  // cleared to zero on reset (active low).
  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_rvalid <= 0;
      axi_rresp <= 0;
    end
    else
    begin
      if (axi_arready && S_AXI_ARVALID && ~axi_rvalid)
      begin
        // Valid read data is available at the read data bus
        axi_rvalid <= 1'b1;
        axi_rresp <= 2'b0; // 'OKAY' response
      end
      else if (axi_rvalid && S_AXI_RREADY)
      begin
        // Read data is accepted by the master
        axi_rvalid <= 1'b0;
      end
    end
  end

  // Implement memory mapped register select and read logic generation
  // Slave register read enable is asserted when valid address is available
  // and the slave is ready to accept the read address.
  assign slv_reg_rden = axi_arready & S_AXI_ARVALID & ~axi_rvalid;
  always @(*)
  begin
    // Address decoding for reading registers
    case (axi_araddr[ADDR_LSB+OPT_MEM_ADDR_BITS:ADDR_LSB])
      4'h0: reg_data_out <= slv_reg0;
      4'h1: reg_data_out <= slv_reg1;
      4'h2: reg_data_out <= slv_reg2;
      4'h3: reg_data_out <= slv_reg3;
      4'h4: reg_data_out <= slv_reg4;
      4'h5: reg_data_out <= slv_reg5;
      default: reg_data_out <= 0;
    endcase
  end

  // Output register or memory read data
  always @(posedge S_AXI_ACLK)
  begin
    if (S_AXI_ARESETN == 1'b0)
    begin
      axi_rdata <= 0;
    end
    else
    begin
      // When there is a valid read address (S_AXI_ARVALID) with
      // acceptance of read address by the slave (axi_arready),
      // output the read dada
      if (slv_reg_rden)
      begin
        axi_rdata <= reg_data_out; // register read data
      end
    end
  end

  // Add user logic here

  //wire aclk = S_AXI_ACLK;
  //wire aresetn = S_AXI_ARESETN;
  reg trigger_in_reg;
  reg trigger_in_reg2;

  reg trigger_out_reg;
  reg trigger_lockout_reg;
  reg [31:0] trigger_out_ctr;
  reg [31:0] trigger_lockout_ctr;
  reg stretched_trigger_out_reg;
  reg [31:0] stretched_trigger_out_ctr;

  reg [31:0] trigger_counter;

  assign trigger_out = trigger_out_reg;
  assign stretched_trigger_out = stretched_trigger_out_reg;

  always @(posedge aclk)
  begin
    if (~aresetn)
    begin
      trigger_in_reg <= 0;
      trigger_in_reg2 <= 0;
    end
    else
    begin
      trigger_in_reg <= trigger_in;
      trigger_in_reg2 <= trigger_in_reg;
    end
  end

  always @(posedge aclk)
  begin
    if (~aresetn)
    begin
      trigger_out_reg <= 0;
      trigger_out_ctr <= 32'd0;
      trigger_lockout_reg <= 0;
      trigger_lockout_ctr <= 32'd0;
      stretched_trigger_out_reg <= 0;
      stretched_trigger_out_ctr <= 32'd0;
      trigger_counter <= 32'd0;
      slv_reg4 <= 32'd0;
      slv_reg5 <= 32'd0;
    end
    else
    begin
      // Make sure trigger out reg is raised for 2 clock cycles
      if (trigger_out_reg == 0 && trigger_lockout_reg == 0)
      begin
        // inverted for now
        if (trigger_in_reg2 == slv_reg2[0] && slv_reg0[0] == 1'b1)
        begin
          trigger_out_reg <= 1;
          trigger_out_ctr <= 32'd0;
          trigger_lockout_reg <= 1;
          trigger_lockout_ctr <= 32'd0;
          stretched_trigger_out_reg <= 1;
          stretched_trigger_out_ctr <= 0;
          trigger_counter <= trigger_counter + 32'd1;
          slv_reg4 <= slv_reg4 + 32'd1;
        end
      end

      // for now set the lockout to 500us
      if (trigger_lockout_reg == 1)
      begin
        if (trigger_lockout_ctr == slv_reg1)
        begin
          trigger_lockout_reg <= 0;
          trigger_lockout_ctr <= 32'd0;
        end
        else
        begin
          trigger_lockout_ctr <= trigger_lockout_ctr + 32'd1;
        end
      end // if (trigger_lockout_reg == 1)

      // set the trigger out to 1 clock cycle
      if (trigger_out_reg == 1)
      begin
        //if (trigger_out_ctr == 32'd1)
        //begin
        trigger_out_reg <= 0;
        slv_reg5 <= slv_reg5 + 32'd1;

        //trigger_out_ctr <= 32'd0;
        //end
        //else
        //begin
        //trigger_out_ctr <= trigger_out_ctr + 32'd1;
        //end
      end // if (trigger_out_reg == 1)

      // set the stretched trigger out to 10ms
      if (stretched_trigger_out_reg == 1)
      begin
        if (stretched_trigger_out_ctr == 32'd1428572)
        begin
          stretched_trigger_out_reg <= 0;
          stretched_trigger_out_ctr <= 0;
        end
        else
        begin
          stretched_trigger_out_ctr <= stretched_trigger_out_ctr + 32'd1;
        end
      end // if (stretched_trigger_out_reg == 1)
    end // else: !if(~aresetn)
  end // always @ (posedge aclk)
endmodule
