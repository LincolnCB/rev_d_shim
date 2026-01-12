***Updated 2024-08-01***
# Synchronous Coherent Signal Synchronizer Core

The `sync_coherent` module synchronizes coherent data between two different clock domains using an asynchronous FIFO. It is parameterizable for signal width and FIFO buffer depth.

## Inputs and Outputs

### Inputs

- **Clocks and Reset**
  - `in_clk`: Source (input) clock domain.
  - `in_resetn`: Active-low reset for the input domain.
  - `out_clk`: Destination (output) clock domain.
  - `out_resetn`: Active-low reset for the output domain.

- **Data and Defaults**
  - `din [WIDTH-1:0]`: Input signal to be synchronized from the source domain.
  - `dout_default [WIDTH-1:0]`: Default value for the output signal after reset.

### Outputs

- `dout [WIDTH-1:0]`: Synchronized output signal in the destination clock domain.

## Operation

- The module detects changes on the input signal (`din`).
- The module pushes new values into an asynchronous FIFO when the input signal changes and the FIFO is not full, or when the FIFO is empty.
- If the FIFO is full, the tracked previous value (`prev_din`) is retained, which should prevent missing a new value.
- The output (`dout`) is updated in the destination clock domain whenever new data is available in the FIFO.
- On reset, the output is set to the provided default value (`dout_default`).

## Parameters

- `WIDTH`: Width of the signal to be synchronized (default: 1).
- `BUF_ADDR_WIDTH`: Address width of the FIFO buffer (default: 2).

## Notes

- The `sync_coherent` module is suitable for synchronizing configuration or control signals across clock domains, ensuring data coherence and glitch-free operation.
- For more details, refer to the Verilog source code.
- Requires an asynchronous FIFO implementation (`fifo_async`).

