**Updated 2026-05-29**
# ADS816x ADC Control Core

The `ads816x_adc_ctrl` module implements command-driven control for the Texas Instruments ADS816x ADC family (ADS8168, ADS8167, ADS8166) in the Rev D shim firmware. It handles the boot-time On-The-Fly register test, SPI transactions, command sequencing, sample ordering, error detection, synchronization for up to 8 ADC channels.

## Supported Devices

- ADS8168 (`ADS_MODEL_ID = 8`)
- ADS8167 (`ADS_MODEL_ID = 7`)
- ADS8166 (`ADS_MODEL_ID = 6`)

Set the target device using the `ADS_MODEL_ID` parameter.

## Inputs and Outputs

### Inputs

- `clk`, `resetn`: Main clock and active-low reset.
- `boot_test_skip`: Skips the boot-time readback step if asserted.
- `debug`: Enables debug mode (debug outputs to the data buffer).
- `n_cs_high_time [7:0]`: Chip select high time in clock cycles (max 255), latched when exiting reset.
- `cmd_word [31:0]`: Command word from buffer.
- `cmd_buf_empty`: Indicates command buffer is empty.
- `trigger`: External trigger signal.
- `miso_sck`, `miso_resetn`, `miso`: SPI MISO clock, reset, and data.
- `data_buf_full`: Indicates data buffer is full.

### Outputs

- `setup_done`: Indicates successful boot/setup after the boot sequence or skipped readback completes.
- `cmd_buf_rd_en`: Enables reading the next command word.
- `waiting_for_trig`: Indicates waiting for trigger.
- `data_buf_wr_en`: Enables writing a data word to the output buffer.
- `data_word [31:0]`: Packed ADC data (two samples per word).
- `last_received_cmd [31:0]`: Most recently accepted command word.
- `cmds_since_reset [31:0]`: Saturating count of commands accepted since reset.
- Error flags: `boot_fail`, `cmd_buf_underflow`, `data_buf_overflow`, `unexp_trig`, `delay_too_short`, `bad_cmd`.
- `n_cs`: SPI chip select (active low).
- `mosi`: SPI MOSI data.

## State Machine

The state machine in `ads816x_adc_ctrl` uses the following codes:

| State Code | Name         | Description                                                                 |
|:----------:|:-------------|:----------------------------------------------------------------------------|
| `0`        | `S_RESET`    | Reset state; waits for `resetn` deassertion.                                |
| `1`        | `S_INIT`     | Initialization; starts SPI register write for boot test.                    |
| `2`        | `S_SET_OTF`  | Necessary initialization: writes On-The-Fly mode to ADC                     |
| `3`        | `S_REQ_RD`   | Boot test: requests readback of On-The-Fly register.                        |
| `4`        | `S_TEST_RD`  | Boot test: reads back register value and checks for match.                  |
| `5`        | `S_IDLE`     | Idle; waits for new command from buffer.                                    |
| `6`        | `S_DELAY`    | Delay timer; waits for specified cycles before next command.                |
| `7`        | `S_TRIG_WAIT`| Waits for external trigger signal.                                          |
| `8`        | `S_ADC_RD`   | Performs ADC read sequence for all channels.                                |
| `9`        | `S_ADC_RD_CH`| Immediately and simply read a single ADC channel.                           |
| `15`       | `S_ERROR`    | Error state; indicates boot/readback failure or invalid command/condition.  |

State transitions are managed based on command type, trigger, delay, and error conditions.

### Boot Test

If `boot_test_skip` is asserted, the core still performs the On-The-Fly register write during boot, but it skips the readback step. `setup_done` is asserted after the register write completes, and the core transitions from `S_SET_OTF` to `S_IDLE`.

Otherwise, the core performs a boot test by writing to the On-The-Fly mode register and reading it back to verify correct operation. If the readback does not match the expected value, the core transitions to `S_ERROR`.

The boot test sequence is as follows:

1. **`S_RESET -> S_INIT`**: After exiting reset, the core prepares to write the On-The-Fly mode register.

2. **`S_INIT -> S_SET_OTF`**: Writes the On-The-Fly mode command to the ADC.
  - SPI command composition:
    - `[23:19]` = `00001` (SPI register write command)
    - `[18:8]`  = `00000101010` (address for On-The-Fly mode register, `0x2A`)
    - `[7:0]`   = `00000001` (data to set On-The-Fly mode)
  - SPI word written:
    ```
    00001_00000101010_00000001
    0x082A01
    ```

3. **`S_SET_OTF -> S_REQ_RD`**: After write completion, requests readback of the On-The-Fly mode register.
  - SPI command composition:
    - `[23:19]` = `00010` (SPI register read command)
    - `[18:8]`  = `00000101010` (address for On-The-Fly mode register, `0x2A`)
    - `[7:0]`   = `00000000` (data field, unused for read)
  - SPI word written:
    ```
    00010_00000101010_00000000
    0x102A00
    ```

4. **`S_REQ_RD -> S_TEST_RD`**: After read request completion, reads back the register value by writing a 16-bit dummy command to clock out the read data.
  - SPI command composition:
    - `[15:0]` = `0000000000000000` (dummy command; all zeros)
  - SPI word written:
    ```
    0000000000000000
    0x0000
    ```
  - SPI response composition:
    - `[15:8]` = `00000001` (should match the On-The-Fly mode value written earlier)
    - `[7:0]`  = `00000000` (unused)
  - SPI response expected (MISO):
    ```
    00000001_00000000
    0x0100
    ```

5. **`S_TEST_RD -> S_IDLE`**: If readback matches expected value, setup is complete (`setup_done` is set); otherwise, transitions to `S_ERROR` and sets `boot_fail`.

When `boot_test_skip` is asserted, step 4 is skipped and the core transitions directly from `S_SET_OTF` to `S_IDLE` after asserting `setup_done`.


## Operation

The core operates based on 32-bit command words read from the command buffer. Each command is executed fully before the next is read (except for CANCEL, which can interrupt waits).

### Command Types

- **NO_OP (`3'd0`)**: Delay or trigger wait.
- **SET_ORD (`3'd1`)**: Set sample order for ADC channels.
- **ADC_RD (`3'd2`)**: Read ADC samples.
- **ADC_RD_CH (`3'd3`)**: Read a single ADC channel.
- **CANCEL (`3'd7`)**: Cancel current wait or delay.

### Command Word Structure

- `[31:29]` — Command code (3 bits).
#### NO_OP (`3'd0`)
- `[28]` — **TRIGGER WAIT**: If set, waits for external trigger (`trigger` input); otherwise, uses value as delay timer.
- `[27]` — **CONTINUE**: If set, expects next command immediately after current completes; if not, returns to IDLE unless buffer underflow.
- `[26]` — **REPEAT**: If set, the command is set to be repeated. The repeat count is the next received command buffer word.
- `[24:0]` — **Value**: If TRIGGER WAIT is set, this is the trigger counter (number of triggers to wait for); otherwise, it is the delay timer (number of clock cycles to wait, zero is allowed).

This command does not initiate SPI activity. If TRIGGER WAIT is set, the core waits for the specified number of external trigger events. Otherwise, it waits for the delay timer to expire. The trigger counter represents the exact number of triggers to wait for, so a value of 0 means finish immediately (no triggers), and a value of 1 means wait for one trigger. The core transitions to the next command if present; if CONTINUE is set and there is no next command, it goes to ERROR, otherwise returns to IDLE. If REPEAT is set, the command is set to be repeated. The repeat count is the next received command buffer word. The command is repeated until the count expires.

**State transitions:**
- `S_IDLE -> S_TRIG_WAIT/S_DELAY -> S_IDLE/S_ERROR/next_cmd_state`

#### SET_ORD (`3'd1`)
- `[23:21]` — Channel 7 index
- `[20:18]` — Channel 6 index
- `[17:15]` — Channel 5 index
- `[14:12]` — Channel 4 index
- `[11:9]`  — Channel 3 index
- `[8:6]`   — Channel 2 index
- `[5:3]`   — Channel 1 index
- `[2:0]`   — Channel 0 index

Configures the sample order for subsequent `ADC_RD` commands. The default order is 0 through 7.

**State transitions:**
- `S_IDLE -> S_IDLE/next_cmd_state`

#### ADC_RD (`3'd2`)
- `[28]` — **TRIGGER WAIT**
- `[27]` — **CONTINUE**
- `[26]` — **REPEAT**: If set, the command is set to be repeated. The repeat count is the next received command buffer word.
- `[24:0]` — **Value**: If TRIGGER WAIT is set, this is the trigger counter (number of triggers to wait for) after ADC read; otherwise, it is the delay timer (number of clock cycles to wait after ADC read, zero is allowed).

Initiates an ADC read sequence for 8 channels in the configured order. In On-The-Fly mode, the core sends 8 SPI words of the form `{2'b10, ch, 11'd0}` (where `ch` is the channel index), followed by a dummy word to clock out the last sample, for a total of 9 transactions. Each MISO word returns the sample for the previously requested channel, so the first word is garbage. After all samples are read, the core either waits for the specified number of triggers or delay cycles, depending on the TRIGGER WAIT flag. The trigger counter represents the exact number of triggers to wait for, so a value of 0 means finish immediately (no triggers), and a value of 1 means wait for one trigger. If REPEAT is set, the command is set to be repeated. The repeat count is the next received command buffer word. The command is repeated until the count expires.

**State transitions:**
- `S_IDLE -> S_ADC_RD -> S_TRIG_WAIT/S_DELAY -> S_IDLE/S_ERROR/next_cmd_state`

#### ADC_RD_CH (`3'd3`)
- `[2:0]` — **Channel**: ADC channel to read (0-7)

Immediately and simply reads a single ADC channel. Has no delays or trigger waits after completion. This command always behaves as a trigger-wait command internally, but the wait counter is set to zero so it completes immediately.

**State transitions:**
- `S_IDLE -> S_ADC_RD_CH -> S_IDLE/next_cmd_state`

#### CANCEL (`3'd7`)
- `[28:0]` — Unused.

Cancels the current wait or delay if issued while the core is in DELAY or TRIG_WAIT state (or just finishing ADC_RD, about to transition to one of those). This is the only command that can interrupt a wait. After canceling, the core returns to IDLE.

**State transitions:**
- `S_TRIG_WAIT/S_DELAY/S_ADC_RD -> S_IDLE`


## Data Buffer Output

### ADC Samples

During and shortly after the `S_ADC_RD` state, the core receives 16-bit SIGNED integer ADC samples on MISO. The samples are packed by pairs into 32-bit words for output to the data buffer. For a pair of samples (N, N+1), the output word is formatted as
```
[data_word] = {sample N+1 [15:0], sample N [15:0]}
```

For single-channel reads, the lower 16 bits contain the converted sample and the upper 16 bits are zero.

### Debug Mode
If `debug` is asserted, the core outputs debug information in addition to ADC samples. On a given clock cycle, the core will choose what to output in the following priority order:
1. If an ADC sample pair is ready, output that.
2. If a single ADC sample is ready, output that.
3. If a MISO data word is read during boot test, output a word using Debug Code 1 (`DBG_MISO_DATA`), including the MISO data.
4. If a state transition occurs, output a word using Debug Code 2 (`DBG_STATE_TRANSITION`), including the previous and current state.
5. If a repeat starts, output a word using Debug Code 3 (`DBG_REPEAT_BIT`), including the repeated command and repeat counter.
6. If the chip select timer starts, output a word using Debug Code 4 (`DBG_N_CS_TIMER`), including the timer value.
7. If the SPI bit counter changes from 0 to nonzero, output a word using Debug Code 5 (`DBG_SPI_BIT`), including the SPI bit value.
8. If a command finishes, output a word using Debug Code 6 (`DBG_CMD_DONE`), including the command state and repeat counter.

Debug output format:
- `[31:28]` - Debug Code (see below)
- Remaining bits: debug data (varies by code, see below)

| Debug Code | Name                   | Data Format                                 |
|:----------:|:-----------------------|:--------------------------------------------|
| 1          | `DBG_MISO_DATA`        | `[15:0]` MISO data                          |
| 2          | `DBG_STATE_TRANSITION` | `[7:4]` prev_state, `[3:0]` state           |
| 3          | `DBG_REPEAT_BIT`       | `[21:16]` repeated command, `[15:0]` repeat counter |
| 4          | `DBG_N_CS_TIMER`       | `[7:0]` n_cs_timer value                    |
| 5          | `DBG_SPI_BIT`          | `[4:0]` spi_bit value                       |
| 6          | `DBG_CMD_DONE`         | `[22]` next_cmd_ready, `[21:16]` command bits, `[15:0]` repeat counter |

If none of the above conditions are met, the output word is zero and nothing is written to the data buffer.

## Errors

- Boot readback mismatch (`boot_fail`)
- Unexpected trigger (trigger when not in `S_TRIG_WAIT`) (`unexp_trig`)
- Delay too short (`delay_too_short`)
- Invalid commands (`bad_cmd`)
- Buffer underflow (expect next command with command buffer empty) (`cmd_buf_underflow`)
- Underflow while starting a repeat (`cmd_buf_underflow`)
- Buffer overflow (try to write to data buffer with data buffer full) (`data_buf_overflow`)

## Notes

- SPI timing and chip select are managed to meet ADS816x requirements.
- The `n_cs_high_time` input is latched when the state machine is in `S_RESET` and clamped to a minimum of 3 cycles.
- The `min_delay_time` input is latched when the state machine is in `S_RESET` and clamped to a minimum of 171 cycles.
- Asynchronous FIFO and synchronizer modules are used for safe cross-domain data transfer.
- Data buffer output is packed as two samples per 32-bit word.

## References

- [ADS8168 Datasheet](https://www.ti.com/product/ADS8168)
- [ADS8167 Datasheet](https://www.ti.com/product/ADS8167)
- [ADS8166 Datasheet](https://www.ti.com/product/ADS8166)

---
*See the source code for detailed implementation and signal descriptions.*
