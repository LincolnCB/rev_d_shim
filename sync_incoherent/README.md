***Updated 2024-08-01***
# Synchronous Incoherent Signal Synchronizer Core

The `sync_incoherent` module synchronizes signals between two clock domains using a multi-stage flip-flop chain. Bits synchronized are not guaranteed to be coherent with each other -- this module is better suited for a wide array of independent bits. It is parameterizable for signal width and synchronizer depth.

## Inputs and Outputs

### Inputs

- **Clocks and Reset**
  - `clk`: Clock signal for the synchronizer.
  - `resetn`: Active-low reset signal.

- **Data**
  - `din [WIDTH-1:0]`: Input signal to be synchronized.

### Outputs

- `dout [WIDTH-1:0]`: Synchronized output signal.

## Operation

- The module uses a chain of flip-flops (with configurable depth) to transfer the input signal (`din`) into the clock domain of `clk`.
- On reset, all flip-flops in the chain are cleared.
- The output (`dout`) is taken from the last flip-flop in the chain, providing a metastability-hardened version of the input.
- The synchronizer depth is clamped between 2 and 8 stages for reliability.

## Parameters

- `WIDTH`: Width of the signal to be synchronized (default: 1).
- `DEPTH`: Number of flip-flop stages in the synchronizer chain (default: 2, min: 2, max: 8).

## Notes

- The `sync_incoherent` module is suitable for synchronizing status or data signals across clock domains where strict data coherence is not required.
- This approach reduces the risk of metastability but does not guarantee glitch-free or coherent transfer of multi-bit signals.
- For more details, refer to the Verilog source code.
- No FIFO is required; only flip-flop registers are used.
