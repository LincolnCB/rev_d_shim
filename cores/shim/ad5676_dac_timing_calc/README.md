***Updated 2025-09-21***
# AD5676 DAC Timing Calculator Core

The `ad5676_dac_timing_calc` module provides hardware-optimized timing calculations for the AD5676 DAC, determining the required n_cs high time based on SPI clock frequency and DAC timing specifications.

## Overview

This module calculates the minimum number of SPI clock cycles that the chip select (n_cs) signal must remain high between DAC updates to meet timing requirements. The calculation considers both update time and minimum n_cs high time constraints specified in the AD5676 datasheet. To divide, the module scales the nanosecond timing requirements to a power-of-two unit (NiS) to allow division via bit-shifting. The output represents cycles minus 1 (i.e., output of 3 means 4 actual cycles).

## DAC Specifications

| Parameter                    | Value   | NiS Value | Bit Width | Description                                    |
|:-----------------------------|:-------:|:---------:|:---------:|:-----------------------------------------------|
| Update Time (T_UPDATE)      | 830 ns  | 892       | 10 bits   | Time between rising edges of n_cs             |
| Min n_cs High Time           | 30 ns   | 33        | 6 bits    | Minimum high time for n_cs signal             |
| SPI Command Width            | 24 bits | -         | -         | Command transmission time consideration        |

*NiS (Like GiB vs GB) units: 2^30 NiS = 10^9 NS = 1 second*

## Interface

### Inputs
- `clk`: System clock signal
- `resetn`: Active-low reset signal
- `spi_clk_freq_hz` (32-bit): SPI clock frequency in Hz
- `calc`: Start calculation signal (active high)

### Outputs
- `n_cs_high_time` (5-bit): Calculated n_cs high time in SPI clock cycles minus 1 (3-31, representing 4-32 actual cycles)
- `done`: Calculation complete signal (active high)
- `lock_viol`: Lock violation signal, asserted if SPI frequency changes during calculation

## Operation

### State Machine
The module implements a 5-state finite state machine:

1. **S_IDLE (0)**: Waiting for calculation request
2. **S_CALC_UPDATE (1)**: Calculate minimum cycles for update time
3. **S_CALC_MIN_HIGH (2)**: Calculate minimum cycles for n_cs high time
4. **S_CALC_RESULT (3)**: Determine final result from both calculations
5. **S_DONE (4)**: Output final result and wait for calc deassertion

### Calculation Method

The module uses optimized shift-add multiplication to avoid DSP resource usage and includes automatic rounding:

1. **Update Time Calculation**:
   ```
   min_cycles_update = (T_UPDATE_NiS × spi_clk_freq_hz + 2^30 - 1) >> 30
   // Only used if result > 24 (SPI command bits)
   ```

2. **Minimum High Time Calculation**:
   ```
   min_cycles_min_high = max(4, (T_MIN_N_CS_HIGH_NiS × spi_clk_freq_hz + 2^30 - 1) >> 30)
   ```
   *Note: Minimum of 4 cycles ensures adequate time for DAC value loading and calibration*

3. **Final Result**:
   ```
   n_cs_high_time = min(31, max(min_cycles_update, min_cycles_min_high) - 1)
   ```
   *Note: Output is cycles minus 1, so actual cycles = n_cs_high_time + 1*

### Performance Optimizations

- **NiS Scaling**: Uses 2^30 scaling factor to replace division with bit shifting
- **Automatic Rounding**: Adds 2^30 - 1 before bit shifting to ensure proper rounding up
- **Optimized Multiplication**: Shift-add algorithm iterates only for significant bits:
  - Update time calculation: 10 iterations instead of 32
  - Min high time calculation: 6 iterations instead of 32
- **Resource Efficient**: Uses wire for intermediate rounding calculations, no DSP blocks required
- **~70% Reduction**: In multiplication cycles compared to full 32-bit iteration

### Error Handling

- **Lock Violation**: If `spi_clk_freq_hz` changes during calculation, `lock_viol` is asserted and the state machine returns to idle
- **Early Termination**: If `calc` signal is deasserted during calculation, the module returns to idle state
- **Saturation**: Output is capped at 31 cycles maximum
- **Minimum Enforcement**: Ensures at least 4 cycles for proper DAC operation

### Timing Constraints

- **Update Time Requirement**: Ensures sufficient time between n_cs rising edges (830ns)
- **Minimum High Time**: Guarantees n_cs remains high for at least 30ns
- **Command Overhead**: Accounts for 24-bit SPI command transmission
- **Output Format**: Returns cycles minus 1 (actual cycles = output + 1, range 4-32 cycles)
- **Maximum Frequency**: At 50MHz SPI clock with output 31, actual cycles are 32, total time is 1120ns (exceeds 830ns requirement)

## Usage Example

```systemverilog
// Instantiate AD5676 timing calculator
ad5676_dac_timing_calc dac_timing_calc (
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

- All timing constants are pre-calculated compile-time values optimized for AD5676
- Multiplication optimization reduces calculation time by approximately 70%
- Module maintains frequency lock during calculation to ensure accuracy
- Reset values initialize all registers to safe defaults
- **Output Format**: n_cs_high_time represents cycles minus 1 for hardware efficiency
- Hardware-efficient implementation uses minimal FPGA resources
- Automatic rounding ensures conservative timing calculations
