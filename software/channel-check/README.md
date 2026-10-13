# channel-check

One-shot channel health check for the Rev D shim amplifier. Boots the hardware in PIO mode and runs a single calibration pass over every configured channel, printing one result line per channel so you can scroll through a board's channels at a glance.

## Usage

```bash
channel-check [--verbose] <channel_count>
```

- `channel_count` -- number of active shim channels (1-64).
- `--verbose` -- enable detailed per-sample logging.

Each channel prints its measured offset and slope followed by a status marker:

- `---` connected / linear
- `-X-` flat near zero (likely disconnected)
- `~E~` nonlinear (likely oscillations)
- `-F-` read failure during calibration

Calibration drives the DACs and reads the ADCs over the programmed-I/O FIFO bridge, so the tool holds every board in PIO (it clears `datapath_mode` before power-on). It runs the same 5-point linearity regression as the interactive `channel_cal` in `shim-test` / `static-shims`, but a single pass rather than the full converging loop. On a hardware halt it prints the decoded `hw_manager` status and exits non-zero; otherwise it powers the system off and exits 0.

The hardware layer is reused from `static-shims` (symlinked `src/` and `include/` files), so the two stay in step; only the single-pass entry point (`hw_calibrate_iters`) is specific to this tool.
