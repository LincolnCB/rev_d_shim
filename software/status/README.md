# status

Command-line diagnostic tool that reads the Rev D shim amplifier's FPGA system status register (mapped at `0x40100000`) and prints a human-readable snapshot of the hardware state. It is read-only — it only observes and reports, and never drives or reconfigures the hardware.

## Usage

```bash
status
```

No arguments. Run it on the target to dump the current status of the shim hardware.

## What it reports

- **Hardware status** — decoded state (`S_IDLE`…`S_HALTED`), status code (`STS_*`), and the board number associated with the current status.
- **Trigger count** — number of hardware triggers seen.
- **Clock frequencies** — SPI clock and SPI source clock, in Hz.
- **Timing debug register** — clock-locked/SPI-off flags, DAC/ADC `~CS` high times, and the SPI clock snoop reconfiguration state machine state.
- **Minimum delay times** — DAC and ADC "delay too short" thresholds, in SPI clock cycles.
- **Per-board FIFO status (boards 0–7)** — DAC and ADC command/data FIFO status (word count, full / almost-full / empty / almost-empty / present flags), the last received command word, and the command count since reset.
- **Trigger FIFOs** — command and data FIFO status.
