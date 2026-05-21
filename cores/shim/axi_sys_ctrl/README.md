***Updated 2026-05-15***
# AXI Shim Config Core

The `axi_sys_ctrl` module provides a configurable interface for managing system parameters and operation via an AXI4-Lite interface.

## Register Map
| Address Offset | 32-bit Offset | Register                | Width   | Description                                   | Default Value         | Range/Notes                  |
|:--------------:|:-------------:|:-----------------------|:--------|:----------------------------------------------|:----------------------|:-----------------------------|
| 0x00           | 0             | `ctrl_en`              | 1 bit   | System enable (locks configuration)            | 0                     | 0 or 1                       |
| 0x04           | 1             | `pow_en`               | 1 bit   | Power enable                                   | 0                     | Not locked by `ctrl_en`      |
| 0x08           | 2             | `cmd_buf_reset`        | 17 bits | Command buffer reset                           | 0                     | Not locked by `ctrl_en`      |
| 0x0C           | 3             | `data_buf_reset`       | 17 bits | Data buffer reset                              | 0                     | Not locked by `ctrl_en`      |
| 0x10           | 4             | `thresh_val`           | 15 bits | Threshold average                              | 16384                 | 1 to 32767                   |
| 0x14           | 5             | `thresh_window`        | 32 bits | Threshold window                               | 5000000               | 2048 to 0xFFFFFFFF           |
| 0x18           | 6             | `thresh_en`            | 1 bit   | Threshold enable                               | 1                     | 0 or 1                       |
| 0x1C           | 7             | `boot_test_skip`       | 16 bits | Boot test skip mask (per-core)                 | 0                     | 0 to 0xFFFF                  |
| 0x20           | 8             | `debug`                | 16 bits | Debug mask (per-core)                          | 0                     | 0 to 0xFFFF                  |
| 0x24           | 9             | `dac_cal_init`         | 16 bits | DAC calibration initialization value (signed)  | 0                     | -32768 to 32767              |
| 0x28           | 10            | `do_dac_pre_delay`     | 1 bit   | DAC command timing: 1 = end of write delay, 0 = start | 1              | 0 or 1                       |

- **Inputs**:
  - `aclk`: AXI clock signal.
  - `aresetn`: Active-low reset signal.
  - `unlock`: Signal to unlock the configuration registers.
  - AXI4-Lite signals: `s_axi_awaddr`, `s_axi_awvalid`, `s_axi_wdata`, `s_axi_wstrb`, `s_axi_wvalid`, `s_axi_bready`, `s_axi_araddr`, `s_axi_arvalid`, `s_axi_rready`.

- **Outputs**:
  - `ctrl_en`: System enable signal.
  - `pow_en`: Power enable signal.
  - `cmd_buf_reset`: 17-bit command buffer reset configuration.
  - `data_buf_reset`: 17-bit data buffer reset configuration.
  - `thresh_val`: 15-bit threshold average configuration.
  - `thresh_window`: 32-bit threshold window configuration.
  - `thresh_en`: Threshold enable signal.
  - `boot_test_skip`: 16-bit boot test skip mask.
  - `debug`: 16-bit debug mask.
  - `dac_cal_init`: 16-bit signed DAC calibration initialization value.
  - `do_dac_pre_delay`: DAC command timing control.
  - Out-of-bounds signals: `ctrl_en_oob`, `pow_en_oob`, `cmd_buf_reset_oob`, `data_buf_reset_oob`, `thresh_val_oob`, `thresh_window_oob`, `thresh_en_oob`, `boot_test_skip_oob`, `debug_oob`, `dac_cal_init_oob`, `do_dac_pre_delay_oob`.
  - `lock_viol`: Signal indicating a lock violation.
  - AXI4-Lite signals: `s_axi_awready`, `s_axi_wready`, `s_axi_bresp`, `s_axi_bvalid`, `s_axi_arready`, `s_axi_rdata`, `s_axi_rresp`, `s_axi_rvalid`.

## Operation

- On reset (`aresetn` low), all internal and output configuration values are initialized to their parameterized default values, capped to their valid ranges.
- Output configuration values are updated to the AXI-written internal variables and locked when the `ctrl_en` bit is set high by the AXI interface. Once locked, any attempt to modify the values results in a lock violation, indicated by latching high the `lock_viol` signal.
- Each internal configuration value is checked against a parameterized range defined in local parameters. If a value is outside its valid range, a corresponding "out-of-bounds" signal is asserted.
- The `cmd_buf_reset` and `data_buf_reset` signals are not locked by the `sys_en` signal. While reset is active, they are set to all ones, but default to all zeros otherwise.
- The `boot_test_skip` register allows skipping boot tests for selected cores, with each bit corresponding to a core.
- The `debug` register enables debug mode for selected cores, with each bit corresponding to a core.
- The `dac_cal_init` register provides a signed calibration initialization value for DAC cores.
- The `do_dac_pre_delay` register controls whether the DAC command is issued at the end (`1`) or start (`0`) of the write delay.
- The `unlock` signal can be used to clear the lock and allow modifications to the configuration registers if `sys_en` has been set low.
- The module supports AXI4-Lite read and write operations for accessing and modifying configuration values. Write responses include error codes to indicate out-of-bounds violations or lock violations.

