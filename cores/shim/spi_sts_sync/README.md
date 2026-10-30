***Updated 2024-08-01***
# SPI System Status Synchronization Core

The `spi_sts_sync` module synchronizes a variety of status signals from the SPI clock domain into the AXI (PS) clock domain.

## Inputs and Outputs

### Inputs

- **Clocks and Reset**
  - `aclk`: AXI (PS) clock signal.
  - `aresetn`: Active-low reset signal for the AXI domain.
  - `spi_clk`: SPI domain clock signal.
  - `sync_resetn`: Active-low reset signal for the SPI domain synchronization.

- **SPI Domain Status Inputs**
  - `spi_off`: Indicates the SPI subsystem is powered off.
  - `over_thresh [7:0]`: Threshold core over-threshold status.
  - `thresh_underflow [7:0]`: Threshold core threshold underflow status.
  - `thresh_overflow [7:0]`: Threshold core threshold overflow status.
  - `bad_trig_cmd`: Bad trigger command detected.
  - `trig_data_buf_overflow`: Trigger data buffer overflow.
  - `dac_boot_fail [7:0]`: DAC boot failure status.
  - `bad_dac_cmd [7:0]`: Bad DAC command detected.
  - `dac_cal_oob [7:0]`: DAC calibration out-of-bounds.
  - `dac_val_oob [7:0]`: DAC value out-of-bounds.
  - `dac_cmd_buf_underflow [7:0]`: DAC command buffer underflow.
  - `dac_data_buf_overflow [7:0]`: DAC data (calibration readback) buffer overflow.
  - `dac_debug_buf_overflow [7:0]`: DAC debug buffer overflow (sticky, non-halting).
  - `unexp_dac_trig [7:0]`: Unexpected DAC trigger.
  - `adc_boot_fail [7:0]`: ADC boot failure status.
  - `bad_adc_cmd [7:0]`: Bad ADC command detected.
  - `adc_cmd_buf_underflow [7:0]`: ADC command buffer underflow.
  - `adc_data_buf_overflow [7:0]`: ADC data buffer overflow.
  - `adc_debug_buf_overflow [7:0]`: ADC debug buffer overflow (sticky, non-halting).
  - `unexp_adc_trig [7:0]`: Unexpected ADC trigger.

### Outputs

- **AXI Domain Synchronized Outputs**
  - `spi_off_sync`: Synchronized SPI off status.
  - `over_thresh_sync [7:0]`: Synchronized threshold core over-threshold status.
  - `thresh_underflow_sync [7:0]`: Synchronized threshold core threshold underflow status.
  - `thresh_overflow_sync [7:0]`: Synchronized threshold core threshold overflow status.
  - `bad_trig_cmd_sync`: Synchronized bad trigger command status.
  - `trig_data_buf_overflow_sync`: Synchronized trigger data buffer overflow status.
  - `dac_boot_fail_sync [7:0]`: Synchronized DAC boot failure status.
  - `bad_dac_cmd_sync [7:0]`: Synchronized bad DAC command status.
  - `dac_cal_oob_sync [7:0]`: Synchronized DAC calibration out-of-bounds status.
  - `dac_val_oob_sync [7:0]`: Synchronized DAC value out-of-bounds status.
  - `dac_cmd_buf_underflow_sync [7:0]`: Synchronized DAC command buffer underflow status.
  - `dac_data_buf_overflow_sync [7:0]`: Synchronized DAC data buffer overflow status.
  - `dac_debug_buf_overflow_sync [7:0]`: Synchronized DAC debug buffer overflow status.
  - `unexp_dac_trig_sync [7:0]`: Synchronized unexpected DAC trigger status.
  - `adc_boot_fail_sync [7:0]`: Synchronized ADC boot failure status.
  - `bad_adc_cmd_sync [7:0]`: Synchronized bad ADC command status.
  - `adc_cmd_buf_underflow_sync [7:0]`: Synchronized ADC command buffer underflow status.
  - `adc_data_buf_overflow_sync [7:0]`: Synchronized ADC data buffer overflow status.
  - `adc_debug_buf_overflow_sync [7:0]`: Synchronized ADC debug buffer overflow status.
  - `unexp_adc_trig_sync [7:0]`: Synchronized unexpected ADC trigger status.

## Operation

- Each input signal from the SPI domain is synchronized to the AXI clock domain: the multi-bit counters and command history (`trig_counter`, `last_received_*_cmds_concat`, `*_cmds_since_reset_concat`) through `sync_coherent` so each word arrives whole, and the per-board flags through `sync_incoherent`.
- `spi_off` is a combinational AND of every board's `setup_done`, so it's registered once in the SPI domain (reset high with `sync_resetn`) before its synchronizer.
- On AXI domain reset (`aresetn` low), all synchronized output signals are set to their default values (zeros).

