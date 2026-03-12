***Updated 2025-09-21***
# ADS816x ADC Timing Calculator Core

The `ads816x_adc_timing_calc` module provides hardware-optimized timing calculations for ADS816x series ADCs, determining the required n_cs high time based on SPI clock frequency and ADC model specifications.

## Overview

This module calculates the minimum number of SPI clock cycles that the chip select (n_cs) signal must remain high between ADC conversions to meet timing requirements. The calculation considers both conversion time and cycle time constraints for different ADS816x models. To divide, the module scales the nanosecond timing requirements to a power-of-two unit (NiS) to allow division via bit-shifting. The output represents cycles minus 1 (i.e., output of 3 means 4 actual cycles).

## Parameters

| Parameter      | Type    | Default | Description                                    | Valid Values              |
|:---------------|:--------|:--------|:-----------------------------------------------|:--------------------------|
| `ADS_MODEL_ID` | integer | 8       | ADC model identifier for timing selection     | 6 (ADS8166), 7 (ADS8167), 8 (ADS8168) |

## ADC Model Specifications

| Model    | Conversion Time | Cycle Time | NiS Conv | NiS Cycle | Bit Width Conv | Bit Width Cycle |
|:---------|:---------------:|:----------:|:--------:|:---------:|:--------------:|:---------------:|
| ADS8168  | 660 ns          | 1000 ns    | 709      | 1074      | 10 bits        | 11 bits         |
| ADS8167  | 1200 ns         | 2000 ns    | 1289     | 2148      | 11 bits        | 12 bits         |
| ADS8166  | 2500 ns         | 4000 ns    | 2685     | 4295      | 12 bits        | 13 bits         |

*NiS (Like GiB vs GB) units: 2^30 NiS = 10^9 NS = 1 second*

## Interface

### Inputs
- `clk`: System clock signal
- `resetn`: Active-low reset signal
- `spi_clk_freq_hz` (32-bit): SPI clock frequency in Hz
- `calc`: Start calculation signal (active high)

### Outputs
- `n_cs_high_time` (8-bit): Calculated n_cs high time in SPI clock cycles minus 1 (2-255, representing 3-256 actual cycles)
- `done`: Calculation complete signal (active high)
- `lock_viol`: Lock violation signal, asserted if SPI frequency changes during calculation

## Operation

### State Machine
The module implements a 5-state finite state machine:

1. **S_IDLE (0)**: Waiting for calculation request
2. **S_CALC_CONV (1)**: Calculate minimum cycles for conversion time
3. **S_CALC_CYCLE (2)**: Calculate minimum cycles for cycle time  
4. **S_CALC_RESULT (3)**: Determine final result from both calculations
5. **S_DONE (4)**: Output final result and wait for calc deassertion

### Calculation Method

The module uses optimized shift-add multiplication to avoid DSP resource usage and includes automatic rounding:

1. **Conversion Time Calculation**:
   ```
   min_cycles_conv = max(3, (T_CONV_NiS × spi_clk_freq_hz + 2^30 - 1) >> 30)
   ```

2. **Cycle Time Calculation**:
   ```
   min_cycles_cycle = max(0, ((T_CYCLE_NiS × spi_clk_freq_hz + 2^30 - 1) >> 30) - 16)
   ```
   *Note: 16 represents the on-the-fly command bits*

3. **Final Result**:
   ```
   n_cs_high_time = min(255, max(min_cycles_conv, min_cycles_cycle) - 1)
   ```
   *Note: Output is cycles minus 1, so actual cycles = n_cs_high_time + 1*

### Performance Optimizations

- **NiS Scaling**: Uses 2^30 scaling factor to replace division with bit shifting
- **Automatic Rounding**: Adds 2^30 - 1 before bit shifting to ensure proper rounding up
- **Optimized Multiplication**: Shift-add algorithm iterates only for significant bits:
  - ADS8168: 10-11 iterations instead of 32
  - ADS8167: 11-12 iterations instead of 32  
  - ADS8166: 12-13 iterations instead of 32
- **Resource Efficient**: Uses wire for intermediate rounding calculations, no DSP blocks required

### Error Handling

- **Lock Violation**: If `spi_clk_freq_hz` changes during calculation, `lock_viol` is asserted and the state machine returns to idle
- **Early Termination**: If `calc` signal is deasserted during calculation, the module returns to idle state
- **Saturation**: Output is capped at 255 cycles maximum

### Timing Constraints

- **Minimum Conversion Time**: Ensures at least 3 cycles for conversion
- **Cycle Time Accounting**: Subtracts 16-bit command transmission time from cycle requirement
- **Output Format**: Returns cycles minus 1 (actual cycles = output + 1, range 3-256 cycles)
- **Maximum Frequency**: At 50MHz SPI clock with output 255, actual cycles are 256, total time is 5440ns (exceeds maximum 4000ns ADC requirement)

## Usage Example

```systemverilog
// Instantiate ADS8168 timing calculator
ads816x_adc_timing_calc #(
    .ADS_MODEL_ID(8)  // ADS8168
) adc_timing_calc (
    .clk(clk),
    .resetn(resetn),
    .spi_clk_freq_hz(32'd20_000_000),  // 20MHz SPI clock
    .calc(start_calc),
    .n_cs_high_time(cs_high_cycles),
    .done(calc_done),
    .lock_viol(freq_changed)
);
```

## Implementation Notes

- All timing constants are compile-time parameters based on `ADS_MODEL_ID`
- Multiplication optimization reduces calculation time by ~50%
- Module maintains frequency lock during calculation to ensure accuracy
- Reset values initialize all registers to safe defaults
- **Output Format**: n_cs_high_time represents cycles minus 1 for hardware efficiency
- Automatic rounding ensures conservative timing calculations
