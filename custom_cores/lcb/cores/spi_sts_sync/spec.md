# SPI System Status Synchronization Core
The `spi_sts_sync` module ensures synchronization of status signals from the SPI clock domain to the PS clock domain. It operates as follows:

- **Inputs and Outputs**:
  - **Inputs**:
    - `clk`: PS clock signal.
    - `spi_clk`: SPI clock signal.
    - `rst`: Reset signal (active high).
    - `spi_running`: Indicates if the SPI system is running.
    - `dac_over_thresh`: 8-bit input signal for DAC over-threshold status.
    - `adc_over_thresh`: 8-bit input signal for ADC over-threshold status.
    - `dac_thresh_underflow`: 8-bit input signal for DAC threshold underflow status.
    - `dac_thresh_overflow`: 8-bit input signal for DAC threshold overflow status.
    - `adc_thresh_underflow`: 8-bit input signal for ADC threshold underflow status.
    - `adc_thresh_overflow`: 8-bit input signal for ADC threshold overflow status.
    - `dac_buf_underflow`: 8-bit input signal for DAC buffer underflow status.
    - `adc_buf_overflow`: 8-bit input signal for ADC buffer overflow status.
    - `unexp_dac_trig`: 8-bit input signal for unexpected DAC trigger status.
    - `unexp_adc_trig`: 8-bit input signal for unexpected ADC trigger status.
  - **Outputs**:
    - `spi_running_stable`: Synchronized and stable SPI running status.
    - `dac_over_thresh_stable`: 8-bit synchronized and stable DAC over-threshold status.
    - `adc_over_thresh_stable`: 8-bit synchronized and stable ADC over-threshold status.
    - `dac_thresh_underflow_stable`: 8-bit synchronized and stable DAC threshold underflow status.
    - `dac_thresh_overflow_stable`: 8-bit synchronized and stable DAC threshold overflow status.
    - `adc_thresh_underflow_stable`: 8-bit synchronized and stable ADC threshold underflow status.
    - `adc_thresh_overflow_stable`: 8-bit synchronized and stable ADC threshold overflow status.
    - `dac_buf_underflow_stable`: 8-bit synchronized and stable DAC buffer underflow status.
    - `adc_buf_overflow_stable`: 8-bit synchronized and stable ADC buffer overflow status.
    - `unexp_dac_trig_stable`: 8-bit synchronized and stable unexpected DAC trigger status.
    - `unexp_adc_trig_stable`: 8-bit synchronized and stable unexpected ADC trigger status.

- **Operation**:
  - Each input signal is synchronized to the AXI clock domain using a 3-stage synchronizer.
  - Stability flags are generated for each signal to indicate when the synchronized value is stable.
  - When stability flags are asserted, the synchronized values are latched into the corresponding stable output registers.
  - On reset (`rst` high), all stable output registers are cleared to their default values.
