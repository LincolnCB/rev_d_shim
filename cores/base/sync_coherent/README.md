***Updated 2024-08-01***
# Synchronous Coherent Signal Synchronizer Core

The `sync_coherent` module carries a multi-bit value between two clock domains so that the output always holds one whole snapshot of the input, never a mix of two values. It uses a request/acknowledge handshake and is parameterizable for signal width.

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

- The input side loads `din` into a hold register and toggles a request bit. The output side synchronizes the request (two `ASYNC_REG` flops), copies the hold register into `dout` when it sees the request change, and echoes the request back as an acknowledge. The input side synchronizes the acknowledge and loads the next snapshot only after the echo arrives, so the hold register never changes while it's being copied.
- The loop runs continuously, so `dout` follows `din` within a few cycles of each clock (one round trip plus a fresh snapshot). Values that change faster than that are skipped; `dout` only ever shows values that were actually on `din`.
- An output reset (`out_resetn` low) holds `dout` at `dout_default`. After it's released, the loop refills `dout` from the current `din` without `din` having to change.
- An input reset (`in_resetn` low) pauses snapshots: `dout` keeps its last value until the input side runs again. The request and its synchronized acknowledge aren't reset, so either side can reset on its own without the loop losing track of which toggle was acknowledged.

## Parameters

- `WIDTH`: Width of the signal to be synchronized (default: 1).

## Notes

- Suited to configuration and status words that need bit coherence across domains, not to streams where every value matters (use `fifo_async` for those).
- The hold register to `dout` path is a multi-bit crossing gated by the synchronized request, which Vivado's CDC report treats as an enable-controlled structure. The copy enable and output reset carry `DIRECT_ENABLE` / `DIRECT_RESET` so every `dout` bit stays a plain flop fed straight from the hold register; without them, synthesis folds the reset and enable into a LUT in front of D for bits whose `dout_default` is 1, and the CDC report flags those bits as unknown circuitry.

