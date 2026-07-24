# Agent Instructions — Rev. D Shim Amplifier System

Check folder READMEs for detailed context if needed.

## Build System
**Never run `make` without explicit user instruction.** Requires Vivado, PetaLinux, and the [Zynq Toolbox](https://github.com/LincolnCB/zynq_toolbox) (external repo); builds are long and resource-intensive.

## Architecture
Zynq-7000 FPGA project (target: Snickerdoodle Black) controlling up to 8 magnetic shim amplifier boards.

**PL (FPGA/RTL):**
- `block_design.tcl` — top-level Vivado design, key vars: `board_count`, `threshold_core_level`, `use_ext_clk`, FIFO address widths
- `modules/*.tcl` — sub-modules sourced by `block_design.tcl`, each using `module_get_upvar` for inherited scope
- `cores/base/` — FIFOs, clock domain sync (`sync_coherent`, `sync_incoherent`), diff I/O buffers
- `cores/shim/` — shim-specific cores (below)
- `cores/pavel-demin/` — upstream cores (Red Pitaya Notes)

**PS (ARM Linux):**
- `software/*/` — C programs using `/dev/mem` + `mmap` for AXI register access; `src/` + `include/` per program

## Key Shim Cores
| Core | Role |
|---|---|
| `hw_manager` | Central FSM: power-up, SPI init, monitoring, shutdown |
| `axi_sys_ctrl` | AXI4-Lite regs for PS config (11 regs, 0x00–0x28) |
| `ad5676_dac_ctrl` | AD5676 16-bit 8-ch DAC control (per board) |
| `ads816x_adc_ctrl` | ADS816x 16-bit 8-ch ADC control (per board) |
| `trigger_core` | Trigger orchestration: sync, external gating, forced, logging |
| `threshold_integrator` | Rolling-sum safety core (BRAM chunked FIFO) |
| `threshold_timer` | Simpler time-above-threshold safety core |
| `spi_cfg_sync` / `spi_sts_sync` | Config/status sync across AXI ↔ SPI clock domains |

## Clock Domains
AXI/PS clock ↔ SPI clock, crossed via:
- `sync_coherent` (async FIFO) — multi-bit, bit-coherent data
- `sync_incoherent` (double-flop) — control signals/masks, no coherency needed
- `spi_en` uses a 4-stage synchronizer for extra margin

## Key Conventions

**RTL**
- Resets: `aresetn`/`resetn`, active-low, synchronous to their clock domain
- Per-board signals: always `[7:0]`
- Command words: 32 bits — upper bits = code/type, then modifier flags, value near the end (DAC/ADC/trigger cores)
- DAC/ADC values: **offset binary** on the wire (0x8000 = 0); software uses signed 16-bit, cores convert
- FSM states: 1-indexed (`S_IDLE = 4'd1`)
- Sim vs. silicon timing: overridden per-test via `parameters.json` (sim uses shorter delays)

**⚠️ FIFO address widths** — set in `block_design.tcl` (`dac_cmd_fifo_addr_width`, etc.) **must match** the software `.h` files (`dac_ctrl.h`, `adc_ctrl.h`, `trigger_ctrl.h`). Mismatch → silent AXI crashes.

**Software**
- `map_32bit_memory()` wraps `/dev/mem` mmap → `volatile uint32_t *`
- Each subsystem: `create_<name>()` maps memory, returns struct of register pointers
- `bool verbose` threaded through most functions, gates `printf`

**Docs** — each core's `README.md` has `***Updated YYYY-MM-DD***`; keep current when modifying cores.

**Testing (cocotb)** — `cores/<family>/<core>/tests/src/`: `testbench.py` (`@cocotb.test()` fns), `<core>_base.py` (test class), `<core>_coverage.py` (coverage). `parameters.json` overrides params for sim.
