***Updated 2025-06-20***
# AXIS FIFO Bridge Core

The `axis_fifo_bridge` module bridges an AXI4-Stream (AXIS) subordinate/manager interface and a simple FIFO interface. It allows AXIS-based systems to write data to and read data from a FIFO, with configurable support for write and read operations, and flexible handshake behavior.

## Features

- AXI4-Stream subordinate (input) and manager (output) interfaces.
- Simple FIFO interface for data transfer.
- Configurable enable/disable for write and read paths.
- Optional always-ready and/or always-valid AXIS handshake.
- Error indication on FIFO full (write) or empty (read).
- Parameterizable data width.
- Overflow and underflow indication signals.

## Parameters

- `AXIS_DATA_WIDTH` (integer): AXIS data width (default: 32).
- `ENABLE_WRITE` (bit): Enable AXIS writes to FIFO (default: 1).
- `ENABLE_READ` (bit): Enable AXIS reads from FIFO (default: 1).
- `ALWAYS_READY` (string): If `"TRUE"`, `s_axis_tready` is always high (default: `"TRUE"`).
- `ALWAYS_VALID` (string): If `"TRUE"`, `m_axis_tvalid` is always high (default: `"TRUE"`).

## Ports

### Clock and Reset

- `aclk` (input): AXIS clock.
- `aresetn` (input): Active-low reset.

### AXIS Subordinate (Write) Interface

- `s_axis_tdata` (input): Write data.
- `s_axis_tvalid` (input): Write data valid.
- `s_axis_tready` (output): Write data ready (see `ALWAYS_READY`).

### AXIS Manager (Read) Interface

- `m_axis_tdata` (output): Read data.
- `m_axis_tvalid` (output): Read data valid (see `ALWAYS_VALID`).
- `m_axis_tready` (input): Read data ready.

### FIFO Write Side

- `fifo_wr_data` (output): Data to write to FIFO.
- `fifo_wr_en` (output): Write enable for FIFO.
- `fifo_full` (input): FIFO full indicator.

### FIFO Read Side

- `fifo_rd_data` (input): Data read from FIFO.
- `fifo_rd_en` (output): Read enable for FIFO.
- `fifo_empty` (input): FIFO empty indicator.

### Status Signals

- `fifo_underflow` (output): Indicates attempted read when FIFO is empty.
- `fifo_overflow` (output): Indicates attempted write when FIFO is full.

## Operation

### Write Path

- AXIS write requests are accepted when `s_axis_tvalid` is high and `s_axis_tready` is asserted.
- If `ENABLE_WRITE` is set and FIFO is not full, data is written to the FIFO.
- If FIFO is full or writes are disabled, no data is written.
- If FIFO is full and a write is attempted, `fifo_overflow` is asserted.
- `s_axis_tready` is always high if `ALWAYS_READY` is `"TRUE"`, otherwise it reflects FIFO status and enable.

### Read Path

- AXIS read requests are accepted when `m_axis_tready` is high and `m_axis_tvalid` is asserted.
- If `ENABLE_READ` is set and FIFO is not empty, data is read from the FIFO and presented on `m_axis_tdata`.
- If FIFO is empty or reads are disabled, no data is read.
- If FIFO is empty and a read is attempted, `fifo_underflow` is asserted.
- `m_axis_tvalid` is always high if `ALWAYS_VALID` is `"TRUE"`, otherwise it reflects FIFO status and enable.

### Handshake Behavior

- If `ALWAYS_READY` is `"TRUE"`, `s_axis_tready` is always high (subordinate interface never stalls).
- If `ALWAYS_VALID` is `"TRUE"`, `m_axis_tvalid` is always high (manager interface always presents data, zero if FIFO empty).

## Notes

- The module does not decode or use any AXIS sideband signals except `tdata`, `tvalid`, `tready`.
- Overflow and underflow signals are asserted when the AXIS side attempts to write to a full FIFO or read from an empty FIFO, respectively.
- No support for AXIS sideband signals (`tlast`, `tkeep`, etc.) or burst/multi-beat semantics.
- The AXIS interface handshake can be configured for always-ready/valid or backpressure-aware operation.
- Data width is parameterizable.
