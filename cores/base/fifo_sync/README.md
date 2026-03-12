***Updated 2025-06-04***
# FIFO Sync Core

The `fifo_sync` module implements a synchronous FIFO with programmable almost-full and almost-empty thresholds. It is designed for high-throughput data buffering within a single clock domain.

## Inputs and Outputs

### Inputs

- `clk`: System clock.
- `resetn`: Active-low synchronous reset.
- `wr_data [DATA_WIDTH-1:0]`: Data input for write.
- `wr_en`: Write enable.
- `rd_en`: Read enable.

### Outputs

- `full`: FIFO full flag.
- `almost_full`: FIFO almost full flag.
- `rd_data [DATA_WIDTH-1:0]`: Data output for read.
- `empty`: FIFO empty flag.
- `almost_empty`: FIFO almost empty flag.

## Parameters

- `FORCE_BRAM` (default: 0): Set to 1 to force the use of block RAM for FIFO memory. Otherwise, Vivado can choose.
- `DATA_WIDTH` (default: 16): Data width of FIFO.
- `ADDR_WIDTH` (default: 4): Address width; FIFO depth is `2^ADDR_WIDTH`.
- `ALMOST_FULL_THRESHOLD` (default: 2): Threshold for almost full flag.
- `ALMOST_EMPTY_THRESHOLD` (default: 2): Threshold for almost empty flag.

## Operation

### FIFO Memory

- Uses a single-port block RAM (`bram`) for storage.
- Write and read pointers are managed synchronously in the same clock domain.

### Pointer Management

- Write and read pointers are binary counters, extended by one bit to distinguish full/empty conditions.
- Write pointer increments on `wr_en`.
- Read pointer increments on `rd_en` if FIFO is not empty.

### Full and Empty Flags

- **Full**: Asserted when the write pointer is one cycle ahead of the read pointer (with MSB difference).
- **Empty**: Asserted when write and read pointers are equal.

### Almost Full/Empty Flags

- **Almost Full**: Asserted when the FIFO fill count is greater than or equal to `2^ADDR_WIDTH - ALMOST_FULL_THRESHOLD`.
- **Almost Empty**: Asserted when the FIFO fill count is less than or equal to `ALMOST_EMPTY_THRESHOLD`.

### FIFO Count

- Fill count is computed as the difference between write and read pointers.
- Used for almost-full and almost-empty flag generation.

### Reset Behavior

- All pointers are reset to zero on deassertion of `resetn`.

## BRAM Implementation

- The `bram` submodule is structured for block RAM inference in synthesis tools.
- Single clock for both read and write operations.

## Usage Notes

- The FIFO is intended for use within a single clock domain.
