***Updated 2025-09-22***
# SPI Configuration Synchronization Core

The `spi_cfg_sync` module synchronizes configuration signals from the AXI clock domain to the SPI clock domain.

## Inputs and Outputs

### Inputs

- **Clocks and Reset**
  - `aclk`: AXI domain clock signal.
  - `aresetn`: Active-low reset for AXI domain.
  - `spi_clk`: SPI domain clock signal.
  - `spi_resetn`: Active-low reset for SPI domain.

- **AXI Domain Configuration Inputs**
  - `integ_thresh_avg [14:0]`: Integration threshold average configuration.
  - `integ_window [31:0]`: Integration window configuration.
  - `integ_en`: Integration enable signal.
  - `spi_en`: SPI enable signal.
  - `block_bufs`: Block buffers enable signal.
  - `dac_n_cs_high_time [4:0]`: DAC chip select high time configuration.
  - `adc_n_cs_high_time [7:0]`: ADC chip select high time configuration.
  - `boot_test_skip [15:0]`: Boot test skip mask (per-core).
  - `debug [15:0]`: Debug mask (per-core).
  - `dac_cal_init [15:0]`: DAC calibration initialization value (signed).

### Outputs

- **SPI Domain Synchronized Outputs**
  - `integ_thresh_avg_sync [14:0]`: Synchronized integration threshold average.
  - `integ_window_sync [31:0]`: Synchronized integration window.
  - `integ_en_sync`: Synchronized integration enable.
  - `spi_en_sync`: Synchronized SPI enable.
  - `block_bufs_sync`: Synchronized block buffers enable.
  - `dac_n_cs_high_time_sync [4:0]`: Synchronized DAC chip select high time.
  - `adc_n_cs_high_time_sync [7:0]`: Synchronized ADC chip select high time.
  - `boot_test_skip_sync [15:0]`: Synchronized boot test skip mask.
  - `debug_sync [15:0]`: Synchronized debug mask.
  - `dac_cal_init_sync [15:0]`: Synchronized DAC calibration initialization value (signed).

## Operation

- Each AXI domain input is synchronized to the SPI clock domain using either `sync_coherent` or `sync_incoherent` modules.
- **Coherent synchronization** is used for multi-bit data that must be coherent across bits.
- **Incoherent synchronization** is used for data where individual bits are not coherent with each other.
- Default values are provided for each output in case of reset:
  - `integ_thresh_avg_sync`: 0x1000
  - `integ_window_sync`: 0x00010000
  - `integ_en_sync`: 0
  - `spi_en_sync`: 0 (uses deeper synchronizer for extra delay)
  - `block_bufs_sync`: 1
  - `dac_n_cs_high_time_sync`: 31 (capped max)
  - `adc_n_cs_high_time_sync`: 255 (capped max)
  - `boot_test_skip_sync`: 0
  - `debug_sync`: 0
  - `dac_cal_init_sync`: 0

## Notes

- The `sync_coherent` module is used for timing-critical multi-bit configuration signals that need coherency across bits.
- The `sync_incoherent` module is used for control signals and masks where bit coherency is not required.
- The `spi_en` signal uses a deeper synchronizer (5 stages) to provide extra delay for this critical signal.
- For more details, refer to the Verilog source code.
