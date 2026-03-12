***Updated 2025-06-04***
# FIFO Async Count Core

The `fifo_async_count` module implements an asynchronous FIFO with independent read and write clock domains, Gray code pointer synchronization, and programmable almost-full/almost-empty thresholds. It is suitable for safe data transfer between different clock domains.

## Inputs and Outputs

### Inputs

- **Write Side**
  - `wr_clk`: Write clock.
  - `wr_rst_n`: Active-low write reset.
  - `wr_data [DATA_WIDTH-1:0]`: Data input for write.
  - `wr_en`: Write enable.

- **Read Side**
  - `rd_clk`: Read clock.
  - `rd_rst_n`: Active-low read reset.
  - `rd_en`: Read enable.

### Outputs

- **Write Side**
  - `fifo_count_wr_clk [ADDR_WIDTH:0]`: FIFO fill count in write clock domain.
  - `full`: FIFO full flag (write domain).
  - `almost_full`: FIFO almost full flag (write domain).

- **Read Side**
  - `rd_data [DATA_WIDTH-1:0]`: Data output for read.
  - `fifo_count_rd_clk [ADDR_WIDTH:0]`: FIFO fill count in read clock domain.
  - `empty`: FIFO empty flag (read domain).
  - `almost_empty`: FIFO almost empty flag (read domain).

## Parameters

- `FORCE_BRAM` (default: 0): Set to 1 to force the use of block RAM for FIFO memory. Otherwise, Vivado can choose.
- `DATA_WIDTH` (default: 16): Data width of FIFO.
- `ADDR_WIDTH` (default: 4): Address width; FIFO depth is `2^ADDR_WIDTH`.
- `ALMOST_FULL_THRESHOLD` (default: 2): Threshold for almost full flag.
- `ALMOST_EMPTY_THRESHOLD` (default: 2): Threshold for almost empty flag.

## Operation

### FIFO Memory

- Uses a dual-port block RAM (`bram_async`) for storage.
- Write and read pointers are managed independently in their respective clock domains.

### Pointer Management and Synchronization

- Write and read pointers are maintained in both binary and Gray code.
- Gray code pointers are synchronized across clock domains using double-flop synchronizers to avoid metastability.
- Binary pointers are recovered from synchronized Gray code for accurate count and flag generation.

### Full and Empty Flags

- **Full**: Asserted when the write pointer is one cycle ahead of the synchronized read pointer (in write domain).
- **Empty**: Asserted when the read pointer matches the synchronized write pointer (in read domain).

### Almost Full/Empty Flags

- **Almost Full**: Asserted when the FIFO fill count in the write domain reaches or exceeds `2^FIFO_DEPTH - ALMOST_FULL_THRESHOLD`.
- **Almost Empty**: Asserted when the FIFO fill count in the read domain is less than or equal to `ALMOST_EMPTY_THRESHOLD`.

### FIFO Counts

- `fifo_count_wr_clk`: Number of entries in FIFO (write domain).
- `fifo_count_rd_clk`: Number of entries in FIFO (read domain).
- Counts are computed using synchronized pointers and handle pointer wrap-around.

### Reset Behavior

- Both write and read sides have independent active-low resets.
- All pointers and synchronizers are reset to zero.

## BRAM Implementation

- The `bram_async` submodule is structured so Vivado infers block RAM for efficient FPGA implementation.
- Separate clocks for write and read ports allow true asynchronous operation.

## Usage Notes

- The FIFO is intended to be safe for crossing unrelated clock domains.
