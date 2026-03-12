***Updated 2025-06-30***
# Threshold Integrator Core

The `threshold_integrator` module is a safety core designed for the Rev D shim firmware. It captures the absolute values of DAC and ADC inputs/outputs and maintains a running sum over a user-defined window. If any channel's sum exceeds a user-defined threshold, it sends a fault signal to the system.

A rolling average over a window is equal to the sum over the window divided by the window width. Instead of checking if the rolling average is larger than the target average (which requires division), we can multiply both by the window width, and check if a rolling sum is over the (window width x target average), which can be precomputed. The rolling sum can be calculated by adding values to the sum when the values enter the window, and subtracting them when they exit the window. This requires storing the values in a FIFO.

For the Zynq 7020, each FIFO will utilize a BRAM block. Each BRAM block can hold 36 kibibits (36 x 1024 bits) of data. For 8 channels, that's a maximum of 128 entries per channel, or 7 bits of information. Thus, per channel, the FIFO can hold 36 + 7 = 43 bits of data. To do so, the core needs to compress multiple samles into a single FIFO entry, which is referred to as a "chunk". A chunk is a sum of 2^(chunk width) samples, where the chunk width is determined by the window size -- the total data that needs to be stored in the FIFO per channel is equal to the bits of the window size plus the bits of the sample (15 bits). The chunk width is calculated to be the most efficient power of 2 that can hold this required amount of data.

However, the maximum widnow size is 32 bits, which gives a maximum total data size of 47 bits (32 + 15). This is 4 bits past what the FIFO could hold for 1024 entries, even if the chunk size was maximized to 21 bits. To accomodate this, as well as allowing the core to be optimized with pipelining, the core's sample rate is set to 1/16th of the clock rate. This gives the missing 4 bits of data.

Finally, when using chunks and subtracting values from the running sum, we no longer know the exact value to subtract on each cycle -- we only have the sum over the chunk size. We instead calculate the average value by dividing the chunk sum by the chunk size (a power of 2 shift), and then subtracting that value from the running sum each cycle. To prevent drift, we also need to account for the remainder of that division. We calculate that remainder (just the bits that were not included in the average) and subtract an extra 1 from the running sum if the outflow timer is below the remainder value. This ensures that the total value of the chunk sum is exactly removed from the running sum by the point the chunk will have exited the window.

## Inputs and Outputs

### Inputs:
- **Clock and Reset**:
  - `clk`: Clock signal for synchronous operation.
  - `resetn`: Active-low reset signal to initialize the module.

- **Control Signals**:
  - `enable`: Enable signal to start the integrator.
  - `sample_core_done`: Signal indicating that the DAC/ADC core is ready.

- **Configuration Signals**:
  - `window`: 32-bit unsigned value defining the integration window size (range: \(2^{11}\) to \(2^{32} - 1\)). Note that this value will be rounded down to the nearest multiple of 16.
  - `threshold_average`: 15-bit unsigned value defining the threshold average (absolute, range: 0 to \(2^{15} - 1\)).

- **Input Data**:
  - `abs_sample_concat`: 120-bit concatenated input containing 8 channels of 15-bit unsigned absolute values.

### Outputs:
- **Status Signals**:
  - `over_thresh`: Signal indicating that the running sum has exceeded the threshold.
  - `err_overflow`: Signal indicating a FIFO overflow error.
  - `err_underflow`: Signal indicating a FIFO underflow error.
  - `setup_done`: Signal indicating that the setup phase is complete.

## Operation

### States:
- **IDLE**: Waits for the `enable` signal to go high. Calculates the chunk bit-width based on the window size.
- **SETUP**: Performs shift-add multiplication to compute the maximum allowed value (`max_value = threshold_average * (window >> 4)`). Sets up the chunk size (power of 2, 0 indexed) and transitions to WAIT.
- **WAIT**: Waits for the `sample_core_done` signal before initializing timers and transitioning to the RUNNING state.
- **RUNNING**: Executes the main logic for inflow, outflow, and running sum calculations. Monitors for threshold violations and FIFO errors.
- **OUT_OF_BOUNDS**: Halts operation if the running sum exceeds the threshold.
- **ERROR**: Halts operation if a FIFO overflow or underflow occurs.

### Setup Phase:
1. **Chunk Width Calculation**: Determines the number of samples to aggregate into a single FIFO entry "chunk" based on the most significant bit of the window. The chunk size will be a power of 2, so calculate and store the `chunk_width`.
2. **Max Value Calculation**: Uses shift-add multiplication to compute `max_value = threshold_average * (window >> 4)`.
3. **Wait for Core Ready**: Waits for `sample_core_done` to initialize timers and transition to the running state.

### Running Phase:
- **Inflow Logic**:
  - For each channel, captures the absolute value of the input every 16th clock cycle.
  - Aggregates 2^(chunk width) values into a chunk sum.
  - Pushes 8 chunk sums (one per channel) into the FIFO when the inflow chunk timer resets.
- **Outflow Logic**:
  - Pops 8 chunk sums from the FIFO when the outflow timer reaches 16 (one per channel).
  - Break down the queued chunk sums into outflow values and remainders.
  - Updates the running total sum for each channel using the difference between inflow and outflow values, accounting for remainders with an optional extra 1 added to the outflow value if the outflow chunk timer is below the remainder value (resulting in fully accounting for the remainder by the end of the sum).
- **Threshold Check**:
  - If any running total sum exceeds the threshold, transitions to the `OUT_OF_BOUNDS` state.

### Error Handling:
- If a FIFO overflow or underflow occurs, transitions to the `ERROR` state and asserts the corresponding error signal.

## Core Specifications

- **Reset Behavior**:
  - All internal signals and outputs are cleared.
  - FIFO is reset.
  - State machine returns to `IDLE`.

### Notes:
- The FIFO is used to manage the running sum efficiently, with a depth of 1024 and a width of 36 bits.
- Sample clustering reduces the required FIFO depth while maintaining precision.
- The minimum window size is 2048 to ensure sufficient processing time for FIFO operations.

### References:
- [7 Series Memory Resources](https://docs.amd.com/v/u/en-US/ug473_7Series_Memory_Resources)
