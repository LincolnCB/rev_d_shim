# Reset Synchronizer Core

The `reset_sync` module brings an active-low reset into a clock domain: the output asserts asynchronously as soon as the input asserts, and releases on a clock edge `DEPTH` cycles after the input deasserts. Asserting asynchronously means the reset takes effect even while the clock is stopped; releasing synchronously keeps the deassertion edge clean for the flops downstream.

It is the lightweight stand-in for `proc_sys_reset` when the input is an internal, glitch-free reset -- a software-written register bit, or another `reset_sync` output. `proc_sys_reset` exists to filter an external reset pin, stretch it to a minimum width, and sequence bus/peripheral resets, and costs around 20 LUT and 27 FF per instance for it; `reset_sync` is `DEPTH` flip-flops and no LUT. The project uses one per FIFO side for the per-board command/data FIFO buffer resets in `modules/axi_spi_interface.tcl`.

## Inputs and Outputs

### Inputs

- `clk`: clock of the domain the reset is released into.
- `rst_in_n`: active-low reset in. Asserts the output immediately (asynchronously).

### Outputs

- `rst_out_n`: active-low reset out, released synchronously to `clk`.

## Operation

- While `rst_in_n` is low, every stage of the flop chain is held at 0, so `rst_out_n` is low regardless of `clk`.
- After `rst_in_n` goes high, a 1 shifts through the chain on each rising edge of `clk`, and `rst_out_n` goes high on the `DEPTH`-th edge.
- Chaining two instances (one's `rst_out_n` into the next's `rst_in_n`, each on its own clock) orders the release across domains: the second releases only after the first has. Assertion still propagates through both asynchronously.
- The input must be glitch-free, since any low pulse resets the chain. A flop output, or a single-input inversion of one, qualifies.

## Parameters

- `DEPTH`: number of synchronizer stages, which is also the release latency in `clk` cycles (default: 2, min: 2, max: 4).
