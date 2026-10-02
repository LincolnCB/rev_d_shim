# waveform

Command-line tool that plays a multi-channel waveform on the Rev D shim amplifier's DACs, optionally reads back ADC samples, and logs hardware trigger times. It streams commands to the FPGA over the DAC/ADC/trigger control interfaces and powers the hardware safely on and off around the run.

## Usage

```
waveform <file.csv> [OPTIONS]
```

| Argument / flag | Type | Default | Meaning |
|---|---|---|---|
| `<file.csv>` | path | — (required) | DAC waveform input file |
| `-a`, `--adc <path>` | path | none | ADC read-timestamp file (enables readback) |
| `-l`, `--lockout <float>` | ms | `10.0` | Trigger lockout time in milliseconds |
| `-c`, `--clk_MHz <float>` | MHz | `30.0` | SPI clock frequency |
| `-i`, `--iters <int>` | int | `1` | Number of times to replay the file(s) |
| `-d`, `--dma` | — | off | Run the prebuffered MCDMA datapath on all active boards (see [DMA datapath](#dma-datapath)) |
| `-h`, `--help` | — | — | Show usage |

> Note: all timestamps **inside the CSV files are in seconds**. The `--lockout` flag is the one exception and is given in milliseconds.

## Input (DAC) CSV format

One data row per line: the first column is a timestamp in **seconds**, and each remaining column is a channel current in **amps**.

```
# comments start with '#'; blank lines are ignored
# t_sec, ch0_amps, ch1_amps, ...
0.000,   0.0,   0.0
0.001,   0.5,  -0.5
0.002,   1.0,  -1.0
```

- Fields may be separated by commas, spaces, or tabs.
- There is **no text header row** — a header must be commented out (`#`) or it
  will fail to parse as numbers.
- Channel count is `fields - 1` and must be between `1` and `64`. Channels map
  onto 8-channel boards; unused channels on an active board are driven to 0.
- Currents must be within `[-3.0, 3.0]` A or validation fails.
- Timestamps must be non-negative.

### Trigger points

A row is a **trigger point** (the start of a new sweep) if it is the first row, or if its timestamp is *lower* than the previous row's (time has reset). Within a sweep, each row is applied as a delay from the previous row. The smallest gap between consecutive rows in a sweep must be at least the hardware's minimum DAC/ADC delay for the selected clock, or the run is rejected.

## ADC (readback) CSV format

A plain list of read timestamps in **seconds**, one per line, no header and no channel columns:

```
0.0005
0.0015
0.0025
```

The same trigger-point rule applies, and the ADC file's trigger count **must match** the DAC file's trigger count.

## Output files

Written next to the input file:

| File | When | Contents |
|---|---|---|
| `<input>.trig_t_sec.csv` | always | one hardware trigger time (seconds) per line |
| `<input>.adc_out_A.csv` | PIO with `--adc` | active-channel readback amps, one sample per line |
| `<input>.adc_out_A.board<b>.csv` | DMA with `--adc` | per-board readback amps (ch0..ch7), one sample per line |

Press `Ctrl+C` to stop early; the hardware is powered off before exit.

## DMA datapath

With `--dma`, every active board runs the prebuffered MCDMA datapath instead of streaming DAC commands word-by-word from software. The tool sets each board's `datapath_mode` to DMA before power-on (the register is locked once the system is running), synthesizes each board's full DAC command stream, and prebuffers it into DDR so the MCDMA fills the DAC FIFO behind the leading trigger-wait -- no software feeds the DAC during the run. The ADC command and trigger lanes stay on the programmed-I/O path; the ADC *data* lane is captured through the MCDMA into DDR and drained to one CSV per board.

During the run the tool waits on the `hw_manager` interrupt (via a `poll()` over `/dev/hw_manager_irq` plus a local-abort path), so a hardware fault or `Ctrl+C` stops the run through the same wait, with no busy-polling. A run finishes when every board has captured its expected sample count (or, with no `--adc` file, when the trigger count is reached and the DAC FIFOs have drained). On any exit the MCDMA is halted and the DMA-driven FIFOs are cleared before power-off, so a later run starts from a clean state.

The DMA path requires a DMA-enabled bitstream (the MCDMA block design) with `u-dma-buf` loaded; without them the tool reports the missing device and exits. The non-DMA (default) path is unchanged. `static-shims` has no DMA mode -- its static setpoint rate never approaches the FIFO-drain floor that motivates DMA.
