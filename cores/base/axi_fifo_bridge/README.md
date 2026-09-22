***Updated 2026-09-21***
# AXI FIFO Bridge Core

The `axi_fifo_bridge` module bridges an AXI4-Lite subordinate interface and a simple FIFO interface. It allows AXI-based systems to write data to and read data from a FIFO, with configurable support for write and read operations.

## Features

- AXI4-Lite subordinate interface for register access.
- Simple FIFO interface for data transfer.
- Configurable enable/disable for write and read paths.
- Always-ready AXI handshake (no hanging).
- Always-`OKAY` responses (accept-and-discard) so a full/empty or wrong-mode access never faults the CPU.
- Overflow and underflow indication for genuine full/empty accesses.
- Datapath-mode gating with a `mode_viol` flag for wrong-mode accesses (the `datapath_mux` hands a shared FIFO port to a DMA owner).
- Parameterizable address and data widths.

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
- `s_axi_bresp` (output): Write response (always `OKAY`).
- `s_axi_bvalid` (output): Write response valid.
- `s_axi_bready` (input): Write response ready.
- `s_axi_araddr` (input): Read address.
- `s_axi_arvalid` (input): Read address valid.
- `s_axi_arready` (output): Read address ready (always high).
- `s_axi_rdata` (output): Read data.
- `s_axi_rresp` (output): Read response (always `OKAY`).
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

### Datapath-Mode Gating

- `wr_mode_block` (input): High when this bridge does not own the write port (a DMA owner does); a write here is a wrong-mode poke, accepted and discarded.
- `rd_mode_block` (input): High when this bridge does not own the read port; a read here is a wrong-mode poke, accepted and discarded (returns zero).
- `mode_viol` (output): Latched high once a wrong-mode write or read is seen; cleared by the corresponding reset.

### Status Signals

- `fifo_underflow` (output): Indicates an attempted read of an empty FIFO that this bridge owns.
- `fifo_overflow` (output): Indicates an attempted write to a full FIFO that this bridge owns.

## Operation

### Write Path

- AXI write requests are always accepted (`s_axi_awready` and `s_axi_wready` are always high).
- If `ENABLE_WRITE` is set, this bridge owns the port (`wr_mode_block` low), and the FIFO is not full, data is written to the FIFO.
- Otherwise the write is accepted and discarded: a wrong-mode write (`wr_mode_block` high) asserts `mode_viol`; a genuine full write asserts `fifo_overflow`.
- The write response (`s_axi_bvalid`/`s_axi_bresp`) is always `OKAY`, asserted after each write attempt.

### Read Path

- AXI read requests are always accepted (`s_axi_arready` is always high).
- If `ENABLE_READ` is set, this bridge owns the port (`rd_mode_block` low), and the FIFO is not empty, data is read from the FIFO and returned.
- Otherwise the read is accepted and returns zero: a wrong-mode read (`rd_mode_block` high) asserts `mode_viol`; a genuine empty read asserts `fifo_underflow`.
- The read response (`s_axi_rvalid`/`s_axi_rresp`) is always `OKAY`, asserted after each read attempt.

### AXI Responses

- `OKAY` (2'b00): The bridge never returns `SLVERR`. Every access completes, so a full/empty FIFO or a wrong-mode access cannot raise a CPU external abort (`SIGBUS`). The full/empty/wrong-mode conditions surface through `fifo_overflow`, `fifo_underflow`, and `mode_viol` instead.

## Notes

- The module does not decode addresses; all accesses are treated as FIFO operations.
- The AXI interface is always ready and always responds `OKAY`; the master reads the status flags rather than the response code to detect an anomaly.
- No support for burst or multi-beat transactions (AXI4-Lite only).
- `fifo_overflow` / `fifo_underflow` flag genuine full/empty accesses on the owned port; `mode_viol` flags a wrong-mode access when the `datapath_mux` has handed the port to a DMA owner. All three latch until the corresponding reset.
