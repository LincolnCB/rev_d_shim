
`timescale 1 ns / 1 ps
// AXI4-Lite status register with alert functionality
// Created based off of Pavel Demin's axi_sts_register core

module axi_sts_alert_reg #
(
  parameter integer STS_DATA_WIDTH = 1024,
  parameter integer AXI_DATA_WIDTH = 32,
  parameter integer AXI_ADDR_WIDTH = 16
)
(
  // System signals
  input  wire                      aclk,
  input  wire                      aresetn,

  // Status bits
  input  wire [STS_DATA_WIDTH-1:0] sts_data,

  // Alert bit (if sts_data has changed since last read)
  output reg                       alert,

  // Slave side
  input  wire [AXI_ADDR_WIDTH-1:0] s_axi_awaddr,  // AXI4-Lite slave: Write address
  input  wire                      s_axi_awvalid, // AXI4-Lite slave: Write address valid
  output wire                      s_axi_awready, // AXI4-Lite slave: Write address ready
  input  wire [AXI_DATA_WIDTH-1:0] s_axi_wdata,   // AXI4-Lite slave: Write data
  input  wire                      s_axi_wvalid,  // AXI4-Lite slave: Write data valid
  output wire                      s_axi_wready,  // AXI4-Lite slave: Write data ready
  output wire [1:0]                s_axi_bresp,   // AXI4-Lite slave: Write response
  output wire                      s_axi_bvalid,  // AXI4-Lite slave: Write response valid
  input  wire                      s_axi_bready,  // AXI4-Lite slave: Write response ready
  input  wire [AXI_ADDR_WIDTH-1:0] s_axi_araddr,  // AXI4-Lite slave: Read address
  input  wire                      s_axi_arvalid, // AXI4-Lite slave: Read address valid
  output wire                      s_axi_arready, // AXI4-Lite slave: Read address ready
  output wire [AXI_DATA_WIDTH-1:0] s_axi_rdata,   // AXI4-Lite slave: Read data
  output wire [1:0]                s_axi_rresp,   // AXI4-Lite slave: Read data response
  output wire                      s_axi_rvalid,  // AXI4-Lite slave: Read data valid
  input  wire                      s_axi_rready   // AXI4-Lite slave: Read data ready
);
  // Register to hold the last read status data
  reg [STS_DATA_WIDTH-1:0] last_read_sts_data;

  // Function to calculate the ceiling of log2(value)
  function integer clogb2 (input integer value);
    for(clogb2 = 0; value > 0; clogb2 = clogb2 + 1) value = value >> 1;
  endfunction

  // Local parameters for address calculation
  localparam integer ADDR_LSB = clogb2(AXI_DATA_WIDTH/8 - 1); // LSB for address alignment
  localparam integer STS_SIZE = STS_DATA_WIDTH/AXI_DATA_WIDTH; // Number of status words
  localparam integer STS_WIDTH = STS_SIZE > 1 ? clogb2(STS_SIZE-1) : 1; // Bits needed for status word index

  // Internal registers for read valid and read data
  reg int_rvalid_reg, int_rvalid_next;
  reg [AXI_DATA_WIDTH-1:0] int_rdata_reg, int_rdata_next;

  // Array of status words split from sts_data
  wire [AXI_DATA_WIDTH-1:0] int_data_mux [STS_SIZE-1:0];

  genvar j, k;

  // Generate block to assign each status word from sts_data
  generate
    for(j = 0; j < STS_SIZE; j = j + 1)
    begin : WORDS
      assign int_data_mux[j] = sts_data[j*AXI_DATA_WIDTH+AXI_DATA_WIDTH-1:j*AXI_DATA_WIDTH];
    end
  endgenerate

  // Synchronous process for read valid and read data registers
  always @(posedge aclk)
  begin
    if(!aresetn)
    begin
      int_rvalid_reg <= 1'b0;
      int_rdata_reg <= {(AXI_DATA_WIDTH){1'b0}};
    end
    else
    begin
      int_rvalid_reg <= int_rvalid_next;
      int_rdata_reg <= int_rdata_next;
    end
  end

  // Combinatorial logic for read valid and read data
  always @*
  begin
    int_rvalid_next = int_rvalid_reg;
    int_rdata_next = int_rdata_reg;

    // If read address is valid, update next read data and set valid
    if(s_axi_arvalid)
    begin
      int_rvalid_next = 1'b1;
      int_rdata_next = int_data_mux[s_axi_araddr[ADDR_LSB+STS_WIDTH-1:ADDR_LSB]];
    end

    // If read data is accepted, clear valid
    if(s_axi_rready & int_rvalid_reg)
    begin
      int_rvalid_next = 1'b0;
    end
  end

  // If reading from the status register, update the corresponding section of the last read status data
  always @(posedge aclk)
  begin
    if(!aresetn)
      last_read_sts_data <= {(STS_DATA_WIDTH){1'b0}};
    else if(s_axi_arvalid && s_axi_arready)
      last_read_sts_data[s_axi_araddr[ADDR_LSB+STS_WIDTH-1:ADDR_LSB]*AXI_DATA_WIDTH +: AXI_DATA_WIDTH] <= int_rdata_reg;
  end

  // Alert logic: set alert if the status data has changed since last read
  always @(posedge aclk)
  begin
    if(!aresetn) alert <= 1'b0;
    else alert <= (sts_data != last_read_sts_data); // Set alert if current status data differs from last read
  end

  // AXI response signals
  assign s_axi_rresp = 2'd0; // OKAY response
  assign s_axi_arready = 1'b1; // Always ready for read address
  assign s_axi_rdata = int_rdata_reg; // Read data output
  assign s_axi_rvalid = int_rvalid_reg; // Read valid output

endmodule
