# AXI Board FIFO Array Core

The `axi_board_fifo_array` module is the programmed-I/O (PIO) register path to every board's DAC and ADC FIFOs, in one AXI4-Lite subordinate. A write pushes a command FIFO and a read pops a debug or data FIFO. Each board's page also carries the ADC capture rate lane, a plain register path to `adc_rate_ctrl`. Fronting all boards from one core means they share a single interconnect port and a single AXI handshake, rather than one bridge and one SmartConnect master port per board (about 70 LUT per port at eight boards).

## Address map

Each board owns one 4 KB page, so board `b`'s lanes sit at `b * 0x1000` plus the lane offset, and the whole array spans 32 KB. Within a page, each lane has its own word. For the FIFO lanes, lane bit 2 picks DAC or ADC, and bits 1:0 pick command, debug or data; word 7 is the rate lane:

| Offset | Lane | Access | DMA-owned in DMA mode |
|---|---|---|---|
| `0x00` | `dac_cmd` | write (push) | yes |
| `0x04` | `dac_debug` | read (pop) | no |
| `0x08` | `dac_data` | read (pop), calibration readback | no |
| `0x10` | `adc_cmd` | write (push) | no |
| `0x14` | `adc_debug` | read (pop) | no |
| `0x18` | `adc_data` | read (pop) | yes |
| `0x1C` | `adc_rate` | write: capture read delta; read: capture backlog (no pop) | no |

The lane is decoded from `addr[4:2]` and the board from `addr[14:12]`; other address bits are ignored, so the lane words alias across the rest of the page. Any other lane offset, a write to a read lane, a read from a write lane, or any access to a board at or above `NUM_BOARDS` is accepted and discarded (reads return zero) without touching a FIFO or raising a flag.

## Operation

- The bus is always ready and always answers `OKAY`, so the CPU never faults on a FIFO access. A write to a full command FIFO or a read from an empty `dac_data`/`adc_data` FIFO is discarded (the read returns zero) and raises that FIFO's sticky overflow/underflow flag. A read from an empty debug FIFO just returns zero; the debug lanes carry no flags.
- `mode_block[b]` is board `b`'s `datapath_mode` bit. While set, the board's two DMA-owned lanes (`dac_cmd` write, `adc_data` read) belong to the DMA adapters, so a PIO access there is discarded and raises `mode_viol[b]` instead of an over/underflow. The board's other lanes stay PIO in both modes.
- Each sticky flag clears on its own FIFO's reset (`*_resetn[b]`), so a buffer reset clears both the FIFO and its flags. `mode_viol[b]` clears on the `dac_cmd` and `adc_data` resets.
- All command FIFOs share one write-data bus (`cmd_wr_data`); only the addressed FIFO's write enable fires. Read data comes in as one concatenated bus per lane, board `b` at bits `[32*b +: 32]`.
- The rate lane is not a FIFO and is never gated: a write strobes `rate_wr_en[b]` with the delta on `cmd_wr_data` regardless of `mode_block` or the command FIFO full flags, and a read returns board `b`'s slice of `rate_backlog` without popping anything.

## Parameters

- `NUM_BOARDS`: boards present (default: 8, min: 1, max: 8). The per-board ports are always eight wide; boards at or above `NUM_BOARDS` are never addressed, so their inputs can be tied off.
- `AXI_ADDR_WIDTH`: AXI address width (default: 16, min: 15 to reach eight pages).
- `AXI_DATA_WIDTH`: AXI data width (must be 32).

## Ports

### Clock and resets

- `aclk`, `aresetn`: AXI clock and handshake reset.
- `dac_cmd_resetn`, `dac_data_resetn`, `adc_cmd_resetn`, `adc_data_resetn` (8 bits each): per-board FIFO resets, active low, clearing that FIFO's sticky flags.

### FIFO sides

- `cmd_wr_data` (32 bits): shared command write data.
- `dac_cmd_wr_en` / `dac_cmd_full`, `adc_cmd_wr_en` / `adc_cmd_full` (8 bits each): command FIFO write enables and full flags.
- `adc_data_rd_data` / `dac_data_rd_data` / `adc_debug_rd_data` / `dac_debug_rd_data` (256 bits each), with matching `*_empty` and `*_rd_en` (8 bits each): read sides.

### Capture rate lane

- `rate_wr_en` (8 bits, out): per-board read-delta write strobe, data on `cmd_wr_data`.
- `rate_backlog` (256 bits, in): every board's capture backlog from `adc_rate_ctrl`, board `b` at `[32*b +: 32]`.

### Gating and flags

- `mode_block` (8 bits, in): per-board DMA ownership of the `dac_cmd` and `adc_data` lanes.
- `mode_viol` (8 bits, out): per-board sticky wrong-mode access flag.
- `dac_cmd_overflow`, `adc_cmd_overflow`, `dac_data_underflow`, `adc_data_underflow` (8 bits each, out): per-board sticky FIFO flags.
