# shim-test

`shim-test` is an interactive command-line tool for manually testing every aspect of the Rev D shim amplifier driver. It maps the shim hardware into memory and initializes control/status interfaces for the system, SPI clock, DACs, ADCs, and trigger, then drops into a REPL-style command loop.

From that prompt you can exercise and inspect all parts of the driver by hand — configuring and controlling the system, driving the DACs, reading the ADCs, and working with the trigger and streaming/experiment features — making it useful for bring-up, debugging, and validation.

## Usage

```bash
shim-test [--verbose]
```

Type `help` at the `Command>` prompt to list all available commands.
