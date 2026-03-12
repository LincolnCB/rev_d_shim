***Updated 2024-08-01***
# SPI System Status Synchronization Core

The `spi_sts_sync` module synchronizes a variety of status signals from the SPI clock domain into the AXI (PS) clock domain.

## Inputs and Outputs

### Inputs

- **Clocks and Reset**
  - `aclk`: AXI (PS) clock signal.
  - `aresetn`: Active-low reset signal for the AXI domain.
  - `spi_clk`: SPI domain clock signal.
  - `spi_resetn`: Active-low reset signal for the SPI domain.

- **SPI Domain Status Inputs**
  - `spi_off`: Indicates the SPI subsystem is powered off.
  - `over_thresh [7:0]`: Integrator over-threshold status.
  - `thresh_underflow [7:0]`: Integrator threshold underflow status.
  - `thresh_overflow [7:0]`: Integrator threshold overflow status.
  - `bad_trig_cmd`: Bad trigger command detected.
  - `trig_data_buf_overflow`: Trigger data buffer overflow.
  - `dac_boot_fail [7:0]`: DAC boot failure status.
  - `bad_dac_cmd [7:0]`: Bad DAC command detected.
  - `dac_cal_oob [7:0]`: DAC calibration out-of-bounds.
  - `dac_val_oob [7:0]`: DAC value out-of-bounds.
  - `dac_cmd_buf_underflow [7:0]`: DAC command buffer underflow.
  - `dac_data_buf_overflow [7:0]`: DAC data buffer overflow.
  - `unexp_dac_trig [7:0]`: Unexpected DAC trigger.
  - `adc_boot_fail [7:0]`: ADC boot failure status.
  - `bad_adc_cmd [7:0]`: Bad ADC command detected.
  - `adc_cmd_buf_underflow [7:0]`: ADC command buffer underflow.
  - `adc_data_buf_overflow [7:0]`: ADC data buffer overflow.
  - `unexp_adc_trig [7:0]`: Unexpected ADC trigger.

### Outputs

- **AXI Domain Synchronized Outputs**
  - `spi_off_sync`: Synchronized SPI off status.
  - `over_thresh_sync [7:0]`: Synchronized integrator over-threshold status.
  - `thresh_underflow_sync [7:0]`: Synchronized integrator threshold underflow status.
  - `thresh_overflow_sync [7:0]`: Synchronized integrator threshold overflow status.
  - `bad_trig_cmd_sync`: Synchronized bad trigger command status.
  - `trig_data_buf_overflow_sync`: Synchronized trigger data buffer overflow status.
  - `dac_boot_fail_sync [7:0]`: Synchronized DAC boot failure status.
  - `bad_dac_cmd_sync [7:0]`: Synchronized bad DAC command status.
  - `dac_cal_oob_sync [7:0]`: Synchronized DAC calibration out-of-bounds status.
  - `dac_val_oob_sync [7:0]`: Synchronized DAC value out-of-bounds status.
  - `dac_cmd_buf_underflow_sync [7:0]`: Synchronized DAC command buffer underflow status.
  - `dac_data_buf_overflow_sync [7:0]`: Synchronized DAC data buffer overflow status.
  - `unexp_dac_trig_sync [7:0]`: Synchronized unexpected DAC trigger status.
  - `adc_boot_fail_sync [7:0]`: Synchronized ADC boot failure status.
  - `bad_adc_cmd_sync [7:0]`: Synchronized bad ADC command status.
  - `adc_cmd_buf_underflow_sync [7:0]`: Synchronized ADC command buffer underflow status.
  - `adc_data_buf_overflow_sync [7:0]`: Synchronized ADC data buffer overflow status.
  - `unexp_adc_trig_sync [7:0]`: Synchronized unexpected ADC trigger status.

## Operation

- Each input signal from the SPI domain is synchronized to the AXI clock domain using the `sync_coherent` module.
- On AXI domain reset (`aresetn` low), all synchronized output signals are set to their default values (zeros).

