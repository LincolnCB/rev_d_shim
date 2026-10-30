# Resource Reclaim Progress (rev_d_shim, toward the 8-board build)

Tracks LUT and BRAM reductions toward fitting the full 8-board `rev_d_shim` build on the xc7z020: make a change, run the build, and record the result here against the previous step.

Numbers come from the post-synth hierarchical utilization report (`tmp_reports/[board]/[board_ver]/rev_d_shim/hierarchical_utilization.txt`). Record the board count with every entry -- the design is dominated by per-board blocks, so 4-board and 8-board numbers aren't directly comparable.

## Device budget (xc7z020clg400-3)

| Resource | Available |
|---|---|
| LUT | 53,200 |
| FF | 106,400 |
| RAMB36 | 140 (= 280 RAMB18) |
| DSP | 220 |

## Why this is needed for 8 boards

The 4-board baseline was 33,692 LUT (63%) and 89 RAMB36 (64%), and both scale with board count. At eight boards the whole-waveform FIFOs alone would need about 176 RAMB36 of the 140 available (21 per board plus the trigger FIFOs and the MCDMA), so step 1's FIFO shrink came first. LUT is the next limit: the earlier scaling analysis put eight boards in the mid-90s percent, so the later steps buy margin for that growth and for the capture-packing core.

## Baseline -- 4 boards

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-06.

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
| adc_data_fifo | 2^13 | 8 | DMA (S2MM) | 32 |
| dac_cmd_fifo | 2^13 | 8 | DMA (MM2S) | 32 |
| dac_data_fifo | 2^12 | 4 | PIO (debug readback) | 16 |
| (per board subtotal) | | 21 | | 84 |
| trig_cmd_fifo | 2^10 | 1 | PIO (shared) | 1 |
| trig_data_fifo | 2^10 | 1 | PIO (shared) | 1 |
| mcdma | -- | 3 (+4 RAMB18) | -- | 3 (+4 RAMB18) |
| total | | | | 89 RAMB36 + 4 RAMB18 |

The two DMA-backed FIFOs (64 RAMB36 at four boards) were the first shrink target: with DMA doing the whole-waveform buffering, they only need to be elastic CDC buffers (`MAX_PACKET_WORDS` plus margin).

## Reclaim log

| Step | Change | Boards | LUT | RAMB36 | RAMB18 | dLUT | dRAMB36 | Build |
|---|---|---|---|---|---|---|---|---|
| 0 | baseline | 4 | 33,692 | 89 | 4 | -- | -- | ok |
| 1 | all FIFO depths -> 2^10 (`dac_cmd` 2^13->2^10, `dac_data` 2^12->2^10, `adc_data` 2^13->2^10; `adc_cmd` / trig already 2^10) + matching software `*_FIFO_WORDCOUNT` macros | 4 | 33,604 | 21 | 4 | -88 | -68 | ok |
| 2 | (reverted -- net loss) AXI3->AXI4-Lite converters at both PS GP boundaries to force low-area SmartConnect. `sys_cfg` went all-Lite (866 -> 603), but each standalone converter cost ~520 LUT; the conversion costs ~500 LUT inside SmartConnect or out. | 4 | 34,181 | 21 | 4 | +577 | 0 | reverted |
| 3 | `dma` datapath at the time: per-board DAC+ADC FIFO bridges merged into one `axi_board_fifo_bridge`, shared `adc_rate_ctrl` added, timing-calc and clock-snoop multipliers on DSP (0 -> 10). dLUT/dRAMB36 vs step 1. | 4 | 31,526 | 21 | 4 | -2,078 | 0 | ok |
| 4 | step-3 datapath at `board_count` 8. Baseline for the 8-board steps; not comparable to the 4-board rows. | 8 | 48,755 | 37 | 4 | -- | -- | ok |
| 5 | 68 per-FIFO `proc_sys_reset` blocks (~21 LUT / 27 FF each) replaced one for one by `base:user:reset_sync` (2 FF), keeping names and command-FIFO chaining. | 8 | 46,535 (with step 6) | 37 | 4 | about -1,350 | 0 | ok (synth) |
| 6 | eight per-board `axi_board_fifo_bridge` instances and their SmartConnect ports replaced by one `axi_board_fifo_array` on one port (14 -> 7 master ports); a 4 KB page per board. Built with step 5, so the split is estimated; together -2,220 LUT / -2,456 FF vs step 4. | 8 | 46,535 | 37 | 4 | about -870 | 0 | ok (synth) |
| 7 | steps 5 + 6 at `board_count` 4. dLUT vs step 3. | 4 | 30,328 | 21 | 4 | -1,198 | 0 | ok (synth) |
| 8 | debug lanes split (not a reclaim): new `adc_debug` (2^12), old `dac_data` becomes `dac_debug`, a separate 2^5 `dac_data` for `GET_CAL`, `adc_cmd` 2^10 -> 2^12, 17 more status words. dLUT/dRAMB36 vs step 7. | 4 | 30,596 | 61 | 8 | +268 | +40 | ok |
| 9 | `adc_rate_ctrl` folded into the array as a per-board rate lane (word 7), dropping its AXI-Lite slave and SmartConnect port. dLUT vs step 8. | 4 | 30,559 | 61 | 8 | -37 | 0 | ok |
| 10 | CDC and SPI timing cleanup (`DMA_TEST_PLAN.md` 5.5): `sync_coherent` rewritten as a handshake, `spi_sts_sync` turned to run SPI -> AXI, SPI MMCM built and timed at 30 MHz. dLUT vs step 9. | 4 | 28,808 | 61 | 8 | -1,751 | 0 | ok |
| 11 | ADC repeat-count fix and `report_cdc` cleanup (not a reclaim): separate DAC debug reset synchronizers, `DIRECT_ENABLE` / `DIRECT_RESET` in `sync_coherent`, `waivers.xdc`. dLUT vs step 10. | 4 | 28,588 | 61 | 8 | -220 | 0 | ok |

## 4-board build (step 11)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, implemented 2026-10-28 (WNS +0.320 ns, WHS +0.050 ns; no critical `report_cdc` entries).

Totals: 28,588 LUT (54%), 28,108 FF, 61 RAMB36, 8 RAMB18, 10 DSP -- 220 LUT below step 10, all in `spi_clk_domain` (10,277 -> 10,053), including `spi_cfg_sync` 30 -> 22 LUT from the `sync_coherent` attributes.

## 4-board build (step 10)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, implemented 2026-10-27 (WNS +0.276 ns, WHS +0.050 ns after the hold re-route, SPI domain timed at 30 MHz).

Totals: 28,808 LUT (54%), 28,096 FF, 788 LUTRAM, 61 RAMB36, 8 RAMB18, 10 DSP -- 1,751 LUT, 1,450 FF and 488 LUTRAM below step 9, nearly all in `spi_clk_domain` (12,044 -> 10,277 LUT, 584 -> 96 LUTRAM). Each of the 40 `sync_coherent` instances was a 4-deep LUTRAM FIFO; as handshakes, `spi_cfg_sync` and `spi_sts_sync` total 115 LUT. About 24 of the 40 are live at four boards (absent boards' inputs are constant), so at ~73 LUT per live instance eight boards should save around 2,900 LUT -- an estimate until an eight-board build.

## 4-board build (step 8)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, implemented 2026-10-22 (WNS +0.233 ns, WHS +0.050 ns after the hold re-route).

Totals: 30,596 LUT (58%), 29,688 FF, 61 RAMB36, 8 RAMB18, 10 DSP -- 268 LUT, 965 FF and 40 RAMB36 above step 7. `axi_spi_interface` rose 2,342 -> 2,984 LUT (83 LUT and 4 RAMB36 per `adc_debug`, 51 LUT and a RAMB18 per `dac_data`, the array 207 -> 313 for two more read lanes) and `status_reg` 463 -> 561 for the extra status words, mostly offset by `spi_clk_domain` falling 12,516 -> 12,044. BRAM grows 10 RAMB36 per board (`dac_debug` 1 -> 4, `adc_cmd` 1 -> 4, `adc_debug` 0 -> 4) plus a RAMB18 for `dac_data`.

At eight boards this projects to about 47,000 LUT (88%) and 117 of 140 RAMB36 (84%), so BRAM becomes the closer limit. Dropping the two debug FIFOs to 2^11 would save 2 RAMB36 each if needed.

## 4-board build (step 7)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-21.

Totals: 30,328 LUT (57%), 28,723 FF, 21 RAMB36, 4 RAMB18, 10 DSP -- 1,198 LUT and 1,260 FF below step 3. `axi_spi_interface` fell 3,209 -> 2,342 LUT (`reset_sync` plus the 207-LUT array in place of four bridges) and `sys_cfg_axi_intercon` 1,473 -> 1,147 (10 -> 7 master ports). Both savings scale with board count, so they're smaller here than at eight.

## 4-board build (step 3)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-15.

Totals: 31,526 LUT (59%), 29,983 FF, 21 RAMB36, 4 RAMB18, 10 DSP -- 2,078 LUT below step 1 at the same BRAM. This build merged the per-board DAC and ADC FIFO bridges into one `axi_board_fifo_bridge`, added the shared `adc_rate_ctrl` core, and moved the timing-calc and clock-snoop multipliers to DSP (0 -> 10).

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

Inside `axi_spi_interface` the DMA datapath is cheap: `adc_rate_ctrl` 447 LUT, `adc_packetizer` 18 LUT each, `axi_board_fifo_bridge` 44 each. Against the baseline, `axi_spi_interface` fell 5,480 -> 3,209 LUT (RAMB36 86 -> 18 from the FIFO shrink), so the merged bridge and DSP mapping more than paid for the rate-control core. `sys_cfg_axi_intercon` grew 866 -> 1,473 as it gained master ports, `adc_rate_ctrl`'s among them.

## 8-board build (steps 4-6)

Build: snickerdoodle_black 1.0, `rev_d_shim`, Vivado 2024.2, synthesized 2026-10-16.

Totals: 48,755 LUT (91.6%), 42,404 FF (40%), 37 RAMB36 + 4 RAMB18 (28% of BRAM), 10 DSP, post-synthesis -- placement and timing closure at this density need a full `make bit`. The earlier 8-board synthesis (2026-10-07) was 52,556 LUT (98.8%); nearly all of the drop is `axi_spi_interface` (10,155 -> 5,800) from merging the per-board bridges.

Top blocks by LUT:

| Block | LUT | FF | RAMB36 | DSP | scales with boards |
|---|---|---|---|---|---|
| spi_clk_domain | 23,913 | 17,225 | 0 | 0 | yes -- eight DAC ctrl at 1,910, eight ADC ctrl at ~650, `spi_sts_sync` 2,741 |
| mcdma | 8,661 | 9,319 | 3 (+4 RAMB18) | 0 | yes (per channel) |
| axi_spi_interface | 5,800 | 5,655 | 34 | 0 | yes (per-board FIFOs, resets, bridges) |
| axi_mem_intercon (HP0 SmartConnect) | 4,402 | 4,580 | 0 | 0 | no |
| sys_cfg_axi_intercon | 1,902 | 1,636 | 0 | 0 | yes (one master port per board) |
| spi_clk_gen | 1,360 | 1,645 | 0 | 0 | no |
| axi_sys_ctrl | 539 | 1,218 | 0 | 0 | no |
| status_reg | 526 | 33 | 0 | 0 | ~no |
| hw_manager | 400 | 67 | 0 | 0 | ~no |
| spi_clk_snoop | 385 | 459 | 0 | 2 | no |
| adc_timing_calc | 273 | 190 | 0 | 4 | no (shared) |
| dac_timing_calc | 262 | 181 | 0 | 4 | no (shared) |
| s2mm_mux | 199 | 68 | 0 | 0 | yes (SI count) |
| mm2s_demux | 72 | 90 | 0 | 0 | yes |

Inside `axi_spi_interface` the DMA datapath is cheap (eight `adc_packetizer` at 18 LUT, eight `axi_board_fifo_bridge` at 48, one `adc_rate_ctrl` at 566); the bulk is per-FIFO overhead -- 34 `fifo_async` at 83 LUT each, and 68 `proc_sys_reset` blocks at ~21 LUT / 27 FF each, the target of step 5. In `sys_cfg_axi_intercon` each master port costs ~70 LUT on top of ~800 fixed, so the eight per-board ports are ~560 LUT.

After steps 5 and 6 (synthesized 2026-10-21): 46,535 LUT (87.5%), 39,948 FF, 37 RAMB36 + 4 RAMB18, 10 DSP -- 2,220 LUT and 2,456 FF below step 4. `axi_spi_interface` fell 5,800 -> 4,368 LUT (68 `reset_sync` instances plus 34 inverters at 68 LUT / 136 FF, against ~1,430 / ~1,850; the array at 295 LUT against 384 for eight bridges), and `sys_cfg_axi_intercon` 1,902 -> 1,144 with seven fewer ports.

## Reclaim menu

Rough order by return-over-risk. Items 1, 2 and 6 have landed (log steps 1, 5 and 6); the rest are optional headroom.

1. **BRAM -- shrink the DMA-backed FIFOs** (step 1, done): `adc_data` and `dac_cmd` 2^13 -> 2^10, still above the packetizer accumulation -- see `DMA_CAPTURE_PACKING.md`.
2. **Per-FIFO reset synchronizers** (step 5, done): `proc_sys_reset`'s pin filtering, pulse stretching and sequencing buy nothing for a software-written reset bit, so `base:user:reset_sync` (async assert, `DEPTH`-flop sync release) replaces it one for one, keeping the command-FIFO chaining. No behavior, address, or software change.
3. **Register-path custom AXI decoder** -- the register SmartConnect (about 1,100 LUT: ~800 fixed plus ~70 per master port) wraps a full AXI4 crossbar and an AXI3->AXI4-Lite converter around single-beat register windows. The classic `axi_interconnect` is discontinued in Vivado 2024.2, and forcing SmartConnect's low-area mode with a boundary converter was a net loss (step 2). A minimal custom decoder would sidestep the ~500-LUT conversion, at the cost of moving subordinate addresses from the address editor into core parameters.
4. **HP0 SmartConnect (`axi_mem_intercon`, 4,402 LUT)** -- strip register slices / cut outstanding depth, or route SG descriptor fetches through a GP port to drop it from three masters to two.
5. **spi_sts_sync telemetry** -- the "last command" / "commands since reset" arrays are observability, not function; gate behind debug or capture on a read handshake. The step-10 handshake rewrite already cut this to 85 LUT at four boards.
6. **Per-board PIO bridge consolidation** (step 6, done): `axi_board_fifo_array` fronts every board's FIFOs on one SmartConnect port with one handshake, and carries the debug and rate lanes.
7. **DAC calibration into the prebuffered software stream** -- lets the PL DAC controller drop its calibration adder; the largest structural saving, but it changes the calibration flow and is only clean once the PIO DAC path is retired.
8. **Threshold core time-mux** -- time-multiplex one over-current comparator across a board's eight channels. It's a safety path, so scrutinize it rather than trim casually.
