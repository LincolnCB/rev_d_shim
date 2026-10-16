# Resource Reclaim Progress (rev_d_shim, toward the 8-board build)

Tracks LUT and BRAM reductions as we work toward fitting the full 8-board `rev_d_shim` build
on the xc7z020. The method is a loop: make a change together, run the specific build, then
record the resulting LUT/BRAM here against the previous step. Each entry says what changed and
what it saved, so the running total and the remaining gap are always visible.

Numbers come from the post-synth hierarchical utilization report
(`tmp_reports/[board]/[board_ver]/rev_d_shim/hierarchical_utilization.txt`, from
`report_utilization -hierarchical`). Record the board count with every entry -- the design is
dominated by per-board blocks, so 4-board and 8-board numbers are not comparable directly.

## Device budget (xc7z020clg400-3)

| Resource | Available |
|---|---|
| LUT | 53,200 |
| FF | 106,400 |
| RAMB36 | 140 (= 280 RAMB18) |
| DSP | 220 |

## Why this is needed for 8 boards

At 4 boards the design is 33,692 LUT (63%) and 89 RAMB36 (64%) -- comfortable. But both scale
with board count, and BRAM is the harder wall:

- BRAM: the per-board FIFOs are 21 RAMB36/board (see the baseline breakdown). Eight boards is
  21 x 8 = 168, plus the shared trig FIFOs (2) and the MCDMA (3 at four channels, more at
  eight) -- roughly 176 RAMB36 against 140 available. The current whole-waveform FIFO depths
  overflow BRAM outright at 8 boards. Shrinking the now-DMA-backed FIFOs to elastic buffers is
  mandatory, not optional.
- LUT: the per-board SPI pool, `axi_spi_interface`, and the MCDMA scale with boards; the
  earlier scaling analysis put 8 boards in the mid-90s percent of the 53,200 LUT, so headroom
  is tight. Reclaim here buys margin for the growth and for the capture-packing
  density-control core.

So BRAM reduction comes first (it unblocks the 8-board build at all), then LUT reclaim for
headroom. The FIFO shrink also directly serves the capture-packing work: see
`DMA_CAPTURE_PACKING.md`.

## Baseline -- 4 boards

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-06.
Report: `tmp_reports/snickerdoodle_black/1.0/rev_d_shim/hierarchical_utilization.txt`.

Totals: 33,692 LUT, 32,966 FF, 89 RAMB36, 4 RAMB18, 0 DSP.

Top blocks by LUT:

| Block | LUT | FF | RAMB36 | RAMB18 | scales with boards |
|---|---|---|---|---|---|
| spi_clk_domain | 12,516 | 9,681 | 0 | 0 | yes (per-channel SPI: DAC/ADC ctrl, threshold, telemetry) |
| mcdma | 6,063 | 7,381 | 3 | 4 | yes (per channel) |
| axi_spi_interface | 5,480 | 5,896 | 86 | 0 | yes (per-board FIFOs + bridges) |
| axi_mem_intercon (HP0 SmartConnect) | 4,402 | 4,580 | 0 | 0 | no |
| spi_clk_gen | 1,360 | 1,645 | 0 | 0 | no |
| sys_cfg_axi_intercon | 866 | 893 | 0 | 0 | ~no |
| axi_sys_ctrl | 539 | 1,202 | 0 | 0 | no |
| spi_clk_snoop | 534 | 665 | 0 | 0 | no |
| status_reg | 479 | 33 | 0 | 0 | ~no |
| adc_timing_calc | 446 | 390 | 0 | 0 | no (shared) |
| dac_timing_calc | 432 | 379 | 0 | 0 | no (shared) |
| hw_manager | 372 | 67 | 0 | 0 | ~no |
| s2mm_mux | 89 | 38 | 0 | 0 | yes (SI count) |
| mm2s_demux | 57 | 82 | 0 | 0 | yes |

Remaining blocks (ps, ps_rst, shutdown_sense, the concat/gate glue) are under ~25 LUT each.

BRAM breakdown (the 89 RAMB36 + 4 RAMB18):

| FIFO | depth | RAMB36/board | path | x4 boards |
|---|---|---|---|---|
| adc_cmd_fifo | 2^10 | 1 | PIO | 4 |
| adc_data_fifo | 2^13 | 8 | **DMA (S2MM)** | 32 |
| dac_cmd_fifo | 2^13 | 8 | **DMA (MM2S)** | 32 |
| dac_data_fifo | 2^12 | 4 | PIO (debug readback) | 16 |
| (per board subtotal) | | 21 | | 84 |
| trig_cmd_fifo | 2^10 | 1 | PIO (shared) | 1 |
| trig_data_fifo | 2^10 | 1 | PIO (shared) | 1 |
| mcdma | -- | 3 (+4 RAMB18) | -- | 3 (+4 RAMB18) |
| **total** | | | | **89 RAMB36 + 4 RAMB18** |

The two DMA-backed FIFOs (`adc_data` 8, `dac_cmd` 8 = 16 RAMB36/board, 64 at four boards) are
the bulk and the first shrink target: DMA now does the whole-waveform buffering, so these only
need to be elastic CDC buffers (deep enough for the packetizer to accumulate up to
`MAX_PACKET_WORDS` plus margin). `dac_data` (debug, PIO) is a secondary target.

## Reclaim log

| Step | Change | Boards | LUT | RAMB36 | RAMB18 | dLUT | dRAMB36 | Build |
|---|---|---|---|---|---|---|---|---|
| 0 | baseline | 4 | 33,692 | 89 | 4 | -- | -- | ok |
| 1 | all FIFO depths -> 2^10 (`dac_cmd` 2^13->2^10, `dac_data` 2^12->2^10, `adc_data` 2^13->2^10; `adc_cmd` / trig already 2^10) + matching software `*_FIFO_WORDCOUNT` macros | 4 | 33,604 | 21 | 4 | -88 | -68 | ok |
| 2 | (reverted -- net loss) AXI3->AXI4-Lite converters at both PS GP boundaries to force low-area SmartConnect. `sys_cfg` did go all-Lite (866->603) but the standalone converter cost 532 (`gp0`) / 518 (`gp1`); the board tree stayed chained (`board_ch` masters still full AXI4, `ch*` unchanged at 406). The AXI3->Lite conversion is ~500 LUT whether inside SmartConnect or standalone, so pulling it out just duplicates it. | 4 | 34,181 | 21 | 4 | +577 | 0 | reverted |
| 3 | current `dma` datapath at branch HEAD: per-board DAC+ADC FIFO bridges merged into one `axi_board_fifo_bridge`, shared `adc_rate_ctrl` capture rate-control core added, DAC/ADC timing calcs + clk snoop map their multipliers to DSP (0 -> 10). dLUT/dRAMB36 vs step 1. | 4 | 31,526 | 21 | 4 | -2,078 | 0 | ok |

## Current build -- 4 boards (`dma` branch)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-15. Report: `tmp_reports/snickerdoodle_black/1.0/rev_d_shim/hierarchical_utilization.txt`.

Totals: 31,526 LUT (59%), 29,983 FF, 21 RAMB36, 4 RAMB18, 10 DSP. Net against the FIFO-shrink step (step 1, 33,604 LUT) is -2,078 LUT at 4 boards, with BRAM unchanged at 21 RAMB36. This build carries the current `dma`-branch datapath: the per-board DAC and ADC FIFO bridges merged into one `axi_board_fifo_bridge`, the shared `adc_rate_ctrl` capture rate-control core added, and the DAC/ADC timing calculators plus the clock snoop now map their multipliers to DSP slices (0 -> 10 DSP) rather than LUTs.

Top blocks by LUT:

| Block | LUT | FF | RAMB36 | DSP | scales with boards |
|---|---|---|---|---|---|
| spi_clk_domain | 12,516 | 9,681 | 0 | 0 | yes (per-channel SPI) |
| mcdma | 6,063 | 7,381 | 3 | 0 | yes (per channel) |
| axi_mem_intercon (HP0 SmartConnect) | 4,402 | 4,580 | 0 | 0 | no |
| axi_spi_interface | 3,209 | 3,057 | 18 | 0 | yes (per-board FIFOs + merged bridge + rate_ctrl) |
| sys_cfg_axi_intercon | 1,473 | 1,353 | 0 | 0 | ~no |
| spi_clk_gen | 1,360 | 1,645 | 0 | 0 | no |
| axi_sys_ctrl | 544 | 1,202 | 0 | 0 | no |
| status_reg | 463 | 33 | 0 | 0 | ~no |
| spi_clk_snoop | 385 | 459 | 0 | 2 | no |
| hw_manager | 373 | 67 | 0 | 0 | ~no |
| adc_timing_calc | 273 | 190 | 0 | 4 | no (shared) |
| dac_timing_calc | 262 | 181 | 0 | 4 | no (shared) |
| s2mm_mux | 89 | 38 | 0 | 0 | yes (SI count) |
| mm2s_demux | 57 | 82 | 0 | 0 | yes |

The DMA datapath itself is cheap inside `axi_spi_interface`: `adc_rate_ctrl` 447 LUT (one shared core), four `adc_packetizer` at 18 LUT each, four `axi_board_fifo_bridge` at 44 LUT each. Against the 2026-10-06 baseline `axi_spi_interface` fell 5,480 -> 3,209 (merged bridge; RAMB36 86 -> 18 from the FIFO shrink), so the merged bridge and the DSP mapping together more than paid for the rate-control core -- the capture density-control feature landed net-negative on LUTs. `sys_cfg_axi_intercon` grew 866 -> 1,473 as the register/clock-config SmartConnect gained master ports, the `adc_rate_ctrl` AXI-Lite subordinate among them.

## 8-board result (post-synthesis)

Measured on the 8-board build (snickerdoodle_black 1.0, Vivado 2024.2, synthesized 2026-10-07):
52,556 LUT, 47,129 FF, 37 RAMB36, 4 RAMB18.

- LUT: 52,556 / 53,200 = **98.8%**. It fits the LUT count, but this is a post-synthesis estimate
  (the report is `Design State: Synthesized`, not implemented). At 98.8% post-synth there is
  almost no slack for place-and-route and timing closure, so a full bitstream (`make bit`) is
  where that density is actually tested -- synthesis fitting does not guarantee implementation
  routes or closes timing.
- BRAM: 37 / 140 RAMB36 (26%) -- the FIFO shrink removed the wall entirely.
- FF: 47,129 / 106,400 (44%) -- not a constraint.

Scaling of the dominant blocks from 4 to 8 boards: `spi_clk_domain` 12,516 -> 23,913,
`axi_spi_interface` 5,408 -> 10,155, `mcdma` 6,063 -> 8,661 (eight channels); the fixed blocks
(HP0 SmartConnect 4,402, clock gen, sys_ctrl, timing calcs) are unchanged.

For real implementation headroom, the register path is the main no-behavior-change reclaim:
`sys_cfg` (866) plus `board_ch` + eight `ch*` (inside `axi_spi_interface`) is on the order of
5k LUT at eight boards. The protocol-converter route was measured and rejected (step 2 in the
log); the remaining lever is a custom single-beat AXI decoder (reclaim menu item 2), estimated
to drop the register path toward a few hundred LUT and bring the design toward ~93-95% -- a much
safer place to implement. The cost is the address-workflow change (addresses move from the
Vivado address editor into core parameters).

These 8-board figures predate the current 4-board datapath above (merged `axi_board_fifo_bridge`, the timing calcs and clock snoop moved to DSP, the added `adc_rate_ctrl` core, and the larger `sys_cfg_axi_intercon`), so the per-block scaling here is stale -- a fresh 8-board synthesis is needed to update it.

## Reclaim menu (planned order)

Rough order by return-over-risk; BRAM first because it gates the 8-board build.

1. **BRAM -- shrink the DMA-backed FIFOs** (`adc_data` 2^13, `dac_cmd` 2^13) toward elastic
   depth, and consider `dac_data` (2^12 debug). This is the step we do first, at 4 boards, to
   measure the per-board saving before scaling to 8. Target depth must still cover the
   packetizer accumulation (`MAX_PACKET_WORDS` + CDC margin) -- see `DMA_CAPTURE_PACKING.md`.
2. **Register-path custom AXI decoder** -- the register/FIFO SmartConnects (`sys_cfg` 866, plus
   `board_ch` + eight `ch*`, ~5k LUT total at eight boards) wrap a full AXI4 crossbar and an
   AXI3->AXI4-Lite converter around single-beat register windows. The classic `axi_interconnect`
   (shared-lite) is not an option -- discontinued in Vivado 2024.2 -- and a boundary protocol
   converter to force SmartConnect's low-area mode was measured as a net loss (step 2 in the
   log). The remaining lever is a custom single-beat AXI decoder: one minimal AXI slave facing
   the PS plus parameterized address decode to the Lite subordinates, sidestepping the ~500-LUT
   generic conversion. Largest no-behavior-change win; cost is moving per-subordinate addresses
   from the Vivado address editor into core parameters.
3. **HP0 SmartConnect (`axi_mem_intercon`, 4,402 LUT)** -- strip register slices / cut
   outstanding depth, or route SG descriptor fetches through a GP port to drop it from three
   masters to two.
4. **spi_sts_sync telemetry** -- the continuously cross-domain-synced "last command" /
   "commands since reset" arrays are observability, not function; gate behind debug or capture
   on a read handshake.
5. **Per-board PIO bridge consolidation** -- after DMA, each per-board bridge is half-idle
   (only `adc_cmd` writes and `dac_debug` reads remain on PIO); collapse to one narrow bridge.
6. **DAC calibration into the prebuffered software stream** -- lets the PL DAC controller drop
   its calibration adder; largest structural saving, but changes the calibration flow and is
   only clean once the PIO DAC path is retired.
7. **Threshold core time-mux** -- over-current safety logic; time-multiplex one comparator
   across a board's eight channels. Scrutinize as a safety path, not a casual trim.
