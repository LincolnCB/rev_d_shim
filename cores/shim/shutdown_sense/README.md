***Updated 2025-08-01***
# Hardware Shutdown Sense Core

The `shutdown_sense` module monitors the onboard shutdown mux and latches a high value if a shutdown is detected on any enabled channel.

## Inputs and Outputs

- **Inputs:**
  - `clk`: Clock signal for synchronous operation.
  - `shutdown_sense_connected [7:0]`: Indicates which shutdown sense channels are connected (active).
  - `shutdown_sense_en`: Enable signal to activate the shutdown sense functionality.
  - `shutdown_sense_pin`: Input signal indicating a shutdown condition.

- **Outputs:**
  - `shutdown_sense_sel [2:0]`: Selects which bit of the `shutdown_sense` register to update (cycles through 0–7).
  - `shutdown_sense [7:0]`: Stores the latched shutdown states for each channel.

## Parameters

- `DWELL_CYCLES` (default 16, min 4, max 256): clock cycles each select is held before it's sampled.

## Operation

- When `shutdown_sense_en` is low, the module resets: `shutdown_sense_sel` is set to `000` and `shutdown_sense` is cleared to `0`.
- When enabled, `shutdown_sense_sel` holds each channel for `DWELL_CYCLES` clock cycles, then increments, cycling through all 8 channels (128 cycles per full scan at the default, 1.28 us at 100 MHz).
- `shutdown_sense_pin` passes through a 2-flop synchronizer and is sampled once, on the last cycle of each channel's window, so the external mux, cable and synchronizer have the rest of the window to settle after the select changes.
- For each selected channel, if the synchronized pin is high **and** the corresponding bit in `shutdown_sense_connected` is set, the respective bit in `shutdown_sense` is latched high (`1`).
- Only channels marked as connected in `shutdown_sense_connected` will be detected and latched.
