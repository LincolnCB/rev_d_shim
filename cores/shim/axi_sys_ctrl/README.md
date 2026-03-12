***Updated 2025-09-22***
# AXI Shim Config Core

The `axi_sys_ctrl` module provides a configurable interface for managing system parameters and operation via an AXI4-Lite interface.

## Register Map
| Address Offset | 32-bit Offset | Register                | Width   | Description                                   | Default Value         | Range/Notes                  |
|:--------------:|:-------------:|:-----------------------|:--------|:----------------------------------------------|:----------------------|:-----------------------------|
| 0x00           | 0             | `sys_en`               | 1 bit   | System enable (locks configuration)            | 0                     | 0 or 1                       |
| 0x04           | 1             | `cmd_buf_reset`        | 17 bits | Command buffer reset                           | 0                     | Not locked by `sys_en`       |
| 0x08           | 2             | `data_buf_reset`       | 17 bits | Data buffer reset                              | 0                     | Not locked by `sys_en`       |
| 0x0C           | 3             | `integ_thresh_avg`     | 15 bits | Integration threshold average                  | 16384                 | 1 to 32767                   |
| 0x10           | 4             | `integ_window`         | 32 bits | Integration window                            | 5000000               | 2048 to 0xFFFFFFFF           |
| 0x14           | 5             | `integ_en`             | 1 bit   | Integration enable                            | 1                     | 0 or 1                       |
| 0x18           | 6             | `boot_test_skip`       | 16 bits | Boot test skip mask (per-core)                | 0                     | 0 to 0xFFFF                  |
| 0x1C           | 7             | `debug`               | 16 bits | Debug mask (per-core)               | 0                     | 0 to 0xFFFF                  |
| 0x20           | 8             | `mosi_sck_pol`        | 1 bit   | MOSI SCK polarity                            | 0                     | 0 or 1                       |
| 0x24           | 9             | `miso_sck_pol`        | 1 bit   | MISO SCK polarity                            | 1                     | 0 or 1                       |
| 0x28           | 10            | `dac_cal_init`        | 16 bits | DAC calibration initialization value (signed) | -32                   | -4096 to 4096               |

- **Inputs**:
  - `aclk`: AXI clock signal.
  - `aresetn`: Active-low reset signal.
  - `unlock`: Signal to unlock the configuration registers.
  - AXI4-Lite signals: `s_axi_awaddr`, `s_axi_awvalid`, `s_axi_wdata`, `s_axi_wstrb`, `s_axi_wvalid`, `s_axi_bready`, `s_axi_araddr`, `s_axi_arvalid`, `s_axi_rready`.

- **Outputs**:
  - `sys_en`: System enable signal.
  - `cmd_buf_reset`: 17-bit command buffer reset configuration.
  - `data_buf_reset`: 17-bit data buffer reset configuration.
  - `integ_thresh_avg`: 15-bit integration threshold average configuration.
  - `integ_window`: 32-bit integration window configuration.
  - `integ_en`: Integration enable signal.
  - `boot_test_skip`: 16-bit boot test skip mask.
  - `debug`: 16-bit debug mask.
  - `mosi_sck_pol`: MOSI SCK polarity signal.
  - `miso_sck_pol`: MISO SCK polarity signal.
  - `dac_cal_init`: 16-bit signed DAC calibration initialization value.
  - Out-of-bounds signals: `sys_en_oob`, `cmd_buf_reset_oob`, `data_buf_reset_oob`, `integ_thresh_avg_oob`, `integ_window_oob`, `integ_en_oob`, `boot_test_skip_oob`, `debug_oob`, `mosi_sck_pol_oob`, `miso_sck_pol_oob`, `dac_cal_init_oob`.
  - `lock_viol`: Signal indicating a lock violation.
  - AXI4-Lite signals: `s_axi_awready`, `s_axi_wready`, `s_axi_bresp`, `s_axi_bvalid`, `s_axi_arready`, `s_axi_rdata`, `s_axi_rresp`, `s_axi_rvalid`.

## Operation

- On reset (`aresetn` low), all internal and output configuration values are initialized to their parameterized default values, capped to their valid ranges.
- Output configuration values are updated to the AXI-written internal variables and locked when the `sys_en` bit is set high by the AXI interface. Once locked, any attempt to modify the values results in a lock violation, indicated by latching high the `lock_viol` signal.
- Each internal configuration value is checked against a parameterized range defined in local parameters. If a value is outside its valid range, a corresponding "out-of-bounds" signal is asserted.
- The `cmd_buf_reset` and `data_buf_reset` signals are not locked by the `sys_en` signal. While reset is active, they are set to all ones, but default to all zeros otherwise.
- The `boot_test_skip` register allows skipping boot tests for selected cores, with each bit corresponding to a core.
- The `debug` register enables debug mode for selected cores, with each bit corresponding to a core.
- The `mosi_sck_pol` and `miso_sck_pol` registers control the SPI clock polarity for MOSI and MISO signals respectively.
- The `dac_cal_init` register provides a signed calibration initialization value for DAC cores, ranging from -4096 to +4096.
- The `unlock` signal can be used to clear the lock and allow modifications to the configuration registers if `sys_en` has been set low.
- The module supports AXI4-Lite read and write operations for accessing and modifying configuration values. Write responses include error codes to indicate out-of-bounds violations or lock violations.

