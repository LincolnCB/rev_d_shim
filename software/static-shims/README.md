# static-shims

Interactive command-line tool for driving the Rev D shim amplifier with **static** (DC) currents. It boots the FPGA hardware, then accepts single-character commands to set per-channel currents manually, from files, or on external/forced triggers.

## Usage

```bash
static-shims [--verbose] <channel_count>
```

- `channel_count` — number of active shim channels (1–64).
- `--verbose` — enable detailed logging.

On start it initializes hardware, configures the clock, and drops into a `static-shims>` prompt. `Ctrl+C` (or `Q`) powers the system off safely before exiting.

## Commands

**General**
| Cmd | Action |
|---|---|
| `H` | Brief help |
| `?` | Extended guide (file format + commands) |
| `Q` | Quit (powers off) |
| `S` | Print system/hardware status |
| `P` | Power amplifier on |
| `X` | Hard reset: power off, unload file, clear buffers |
| `Z` | Zero all currents, unload file, clear buffers |
| `I [file]` | Read currents from ADC (optionally dump to file) |

**Manual**
| Cmd | Action |
|---|---|
| `n x` | Set channel `n` to `x` amperes |
| `U x1 x2...` | Update all channels to given amps |
| `B x1 x2...` | Buffer all channels, wait for trigger |
| `C` | Run calibration |
| `D t` | Set trigger lockout time (ms, default 10.0) |
| `T [n]` | Trigger next shim row `n` times (default 1) |

**File**
| Cmd | Action |
|---|---|
| `L [file]` | Load shim block file (or reload last) |
| `A [data]` | Load inline array as a shim block (`/` = newline) |
| `E` | Exit loaded file, reset trigger counter |
| `R` | Reset buffers, restart loaded file |

## Layout

- `static-shims.c` — entry point / REPL loop and boot sequence.
- `src/`, `include/` — `commands/` (command parsing + execution), `input/` (line parsing), `file_handling/` (shim block files), and `static_hardware/` (`hw_*` hardware control API, wrapping the FPGA `sys`/`clk`/`dac`/`adc`/`trigger` controllers).

Max 64 channels; currents limited to ±5.0 A.
