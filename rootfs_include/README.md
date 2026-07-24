# Rev D Shim Amplifier

This system includes multiple command-line tools for controlling, driving, and diagnosing the shim hardware. These all currently need to be run with sudo privileges in order to handle memory mapping.

| Tool | Type | What it does |
|---|---|---|
| `reset-shims` | One-shot utility | Safely resets the shim hardware to a known idle state — powers off and pulses the command/data buffer resets. Good for recovering between runs without a power cycle. |
| `status` | One-shot, read-only | Reads the FPGA system status register and prints a human-readable snapshot (hardware state, trigger count, clock frequencies, timing debug, per-board FIFO status). Observes only, never drives hardware. |
| `static-shims` | Interactive | Drives the shims with **static (DC)** currents. Set per-channel currents manually, from files, or on triggers. |
| `waveform` | One-shot, file-driven | Plays a multi-channel **waveform** from a CSV on the DACs, optionally reads back ADC samples, and logs trigger times. Configurable clock, lockout, and iteration count. |
| `shim-test` | Interactive | Manual test bench for **every aspect** of the shim driver — exercise the system, clock, DACs, ADCs, trigger, and streaming/experiment features by hand. For bring-up, debugging, and validation. |

## At a glance

- **Diagnostics / recovery:** `status` (inspect the hardware) and `reset-shims` (recover to idle).
- **Driving the hardware:** `static-shims` for DC currents and `waveform` for time-varying playback.
- **Development / debugging:** `shim-test` as the low-level interactive driver test tool.

Use the `-h` or `--help` option with any command to see its usage and available options.
