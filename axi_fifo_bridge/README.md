***Updated 2025-06-17***
# AXI FIFO Bridge Core

The `axi_fifo_bridge` module bridges an AXI4-Lite subordinate interface and a simple FIFO interface. It allows AXI-based systems to write data to and read data from a FIFO, with configurable support for write and read operations.

## Features

- AXI4-Lite subordinate interface for register access.
- Simple FIFO interface for data transfer.
- Configurable enable/disable for write and read paths.
- Always-ready AXI handshake (no hanging).
- Error response on FIFO full (write) or empty (read).
- Parameterizable address and data widths.
- Overflow and underflow indication signals.

## Parameters

- `AXI_ADDR_WIDTH` (integer): AXI address width (default: 8).
- `AXI_DATA_WIDTH` (integer): AXI data width (default: 32).
- `ENABLE_WRITE` (bit): Enable AXI writes to FIFO (default: 1).
- `ENABLE_READ` (bit): Enable AXI reads from FIFO (default: 1).

## Ports

### Clock and Reset

- `aclk` (input): AXI clock.
- `aresetn` (input): Active-low reset.

### AXI4-Lite Subordinate Interface

- `s_axi_awaddr` (input): Write address.
- `s_axi_awvalid` (input): Write address valid.
- `s_axi_awready` (output): Write address ready (always high).
- `s_axi_wdata` (input): Write data.
- `s_axi_wstrb` (input): Write strobes.
- `s_axi_wvalid` (input): Write data valid.
- `s_axi_wready` (output): Write data ready (always high).
- `s_axi_bresp` (output): Write response (`OKAY` or `SLVERR`).
- `s_axi_bvalid` (output): Write response valid.
- `s_axi_bready` (input): Write response ready.
- `s_axi_araddr` (input): Read address.
- `s_axi_arvalid` (input): Read address valid.
- `s_axi_arready` (output): Read address ready (always high).
- `s_axi_rdata` (output): Read data.
- `s_axi_rresp` (output): Read response (`OKAY` or `SLVERR`).
- `s_axi_rvalid` (output): Read valid.
- `s_axi_rready` (input): Read ready.

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

- AXI write requests are always accepted (`s_axi_awready` and `s_axi_wready` are always high).
- If `ENABLE_WRITE` is set and FIFO is not full, data is written to the FIFO.
- If FIFO is full or writes are disabled, an AXI `SLVERR` response is returned
- If FIFO is full, `fifo_overflow` is asserted.
- Write response (`s_axi_bvalid`/`s_axi_bresp`) is asserted after each write attempt.

### Read Path

- AXI read requests are always accepted (`s_axi_arready` is always high).
- If `ENABLE_READ` is set and FIFO is not empty, data is read from the FIFO and returned.
- If FIFO is empty or reads are disabled, an AXI `SLVERR` response is returned with zero data.
- If FIFO is empty, `fifo_underflow` is asserted.
- Read response (`s_axi_rvalid`/`s_axi_rresp`) is asserted after each read attempt.

### AXI Responses

- `OKAY` (2'b00): Operation successful.
- `SLVERR` (2'b10): Error (FIFO full on write, FIFO empty on read, or operation disabled).

## Notes

- The module does not decode addresses; all accesses are treated as FIFO operations.
- The AXI interface is always ready, so the master must handle error responses.
- No support for burst or multi-beat transactions (AXI4-Lite only).
- Overflow and underflow signals are asserted when the AXI side attempts to write to a full FIFO or read from an empty FIFO, respectively.
