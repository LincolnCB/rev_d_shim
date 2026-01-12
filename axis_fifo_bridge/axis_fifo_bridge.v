`timescale 1 ns / 1 ps

module axis_fifo_bridge #(
  parameter integer AXIS_DATA_WIDTH = 32,
  parameter         ENABLE_WRITE    = 1, // 1=enable AXIS writes to FIFO
  parameter         ENABLE_READ     = 1, // 1=enable AXIS reads from FIFO
  parameter         ALWAYS_READY    = "TRUE", // If "TRUE", s_axis_tready is always high
  parameter         ALWAYS_VALID    = "TRUE"  // If "TRUE", m_axis_tvalid is always high
)(
  input  wire                       aclk,
  input  wire                       aresetn,

  // AXIS subordinate (write) interface
  input  wire [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
  input  wire                       s_axis_tvalid,
  output wire                       s_axis_tready,

  // AXIS manager (read) interface
  output wire [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
  output wire                       m_axis_tvalid,
  input  wire                       m_axis_tready,

  // FIFO write side
  output wire [AXIS_DATA_WIDTH-1:0] fifo_wr_data,
  output wire                       fifo_wr_en,
  input  wire                       fifo_full,

  // FIFO read side
  input  wire [AXIS_DATA_WIDTH-1:0] fifo_rd_data,
  output wire                       fifo_rd_en,
  input  wire                       fifo_empty,

  // Underflow/overflow signals for the AXIS side
  output reg                        fifo_underflow,
  output reg                        fifo_overflow
);

  // Write logic (AXIS subordinate to FIFO)
  wire write_allowed = ENABLE_WRITE && !fifo_full;
  assign fifo_wr_data = s_axis_tdata;
  assign fifo_wr_en   = s_axis_tvalid && write_allowed;

  // s_axis_tready logic
  generate
    if (ALWAYS_READY == "TRUE") begin : GEN_ALWAYS_READY
      assign s_axis_tready = 1'b1;
    end else begin : GEN_BLOCKING_READY
      assign s_axis_tready = write_allowed;
    end
  endgenerate

  // Overflow flag
  always @(posedge aclk) begin
    if (!aresetn) begin
      fifo_overflow <= 1'b0;
    end else if (s_axis_tvalid && !write_allowed) begin
      if (fifo_full) fifo_overflow <= 1'b1;
    end
  end

  // Read logic (FIFO to AXIS manager)
  wire read_allowed = ENABLE_READ && !fifo_empty;
  assign fifo_rd_en = m_axis_tready && read_allowed;

  // m_axis_tdata/valid logic
  generate
    if (ALWAYS_VALID == "TRUE") begin : GEN_ALWAYS_VALID
      assign m_axis_tdata  = fifo_empty ? {AXIS_DATA_WIDTH{1'b0}} : fifo_rd_data;
      assign m_axis_tvalid = 1'b1;
    end else begin : GEN_BLOCKING_VALID
      assign m_axis_tdata  = fifo_rd_data;
      assign m_axis_tvalid = !fifo_empty;
    end
  endgenerate

  // Underflow flag
  always @(posedge aclk) begin
    if (!aresetn) begin
      fifo_underflow <= 1'b0;
    end else if (m_axis_tready && !read_allowed) begin
      if (fifo_empty) fifo_underflow <= 1'b1;
    end
  end

endmodule
