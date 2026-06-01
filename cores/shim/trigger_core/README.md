**Updated 2026-05-29**
# Trigger Core (`trigger_core`)

The `trigger_core` module provides command-driven trigger orchestration for the Rev D shim firmware. It can synchronize ADC/DAC trigger consumers, gate external triggers with a programmable lockout, generate forced triggers, and optionally log trigger timestamps.

## Parameters

- `TRIGGER_LOCKOUT_DEFAULT`: Default external-trigger lockout in `clk` cycles.
  - Valid range is 4 to `28'hFFFFFFF`.
  - Values outside this range raise a simulation-time `$error`.

## Inputs and Outputs

### Inputs

- `clk`, `resetn`: Main clock and active-low reset.
- `cmd_word [31:0]`: Command word from command FIFO.
- `cmd_buf_empty`: Command FIFO empty flag.
- `data_buf_full`: Data FIFO full flag.
- `data_buf_almost_full`: Data FIFO almost-full flag. This is used to guarantee room for two log words.
- `ext_trig`: External trigger input (internally synchronized with a 2-flop synchronizer).
- `dac_waiting_for_trig [7:0]`: Per-channel DAC waiting flags.
- `adc_waiting_for_trig [7:0]`: Per-channel ADC waiting flags.

### Outputs

- `cmd_word_rd_en`: Command FIFO read strobe.
- `data_word_wr_en`: Data FIFO write strobe.
- `data_word [31:0]`: Trigger log payload.
- `trig_out`: One-cycle internal trigger pulse output.
- `trig_counter [31:0]`: Trigger event count.
- `data_buf_overflow`: Sticky overflow flag when logging cannot write two words.
- `bad_cmd`: Sticky invalid-command/invalid-argument flag.

## Command Encoding

- `[31:29]`: Command type.
- `[28]`: Log-enable bit for trigger-producing commands.
- `[27:0]`: Command value.

### Command Types

- `SYNC_CH (3'd1)`
  - Waits until all `dac_waiting_for_trig` bits and all `adc_waiting_for_trig` bits are high.
  - Emits one trigger pulse when all channels are ready.
  - If already ready, can complete immediately and still trigger.
  - If log-enable is set, this trigger is logged.

- `SET_LOCKOUT (3'd2)`
  - Sets the external-trigger lockout period to `[27:0]`.
  - Values below 4 are rejected and push the FSM to `S_ERROR` (sets `bad_cmd`).

- `EXPECT_EXT_TRIG (3'd3)`
  - Arms external-trigger counting for `[27:0]` events.
  - A value of 0 completes immediately.
  - Each accepted external trigger decrements the internal counter and emits `trig_out`.
  - If log-enable is set, each accepted trigger is logged.

- `DELAY (3'd4)`
  - Waits `[27:0]` clock cycles.
  - A value of 0 completes immediately.

- `FORCE_TRIG (3'd5)`
  - Emits an immediate trigger pulse.
  - If log-enable is set, this trigger is logged.

- `RESET_COUNT (3'd6)`
  - Resets `trig_counter` and the internal 64-bit timestamp counter.
  - This command has side effects as soon as it reaches the FIFO head (`cmd_word_rd_en` also asserts on this command).

- `CANCEL (3'd7)`
  - Can cancel active waits and complete commands from any non-error state.
  - Causes command completion and transition back to `S_IDLE`.

## State Machine

The FSM states are:

- `S_RESET (0)`: Reset state.
- `S_IDLE (1)`: Wait for next command.
- `S_SYNC_CH (2)`: Wait for all ADC/DAC channels to be ready for sync trigger.
- `S_EXPECT_TRIG (3)`: Wait for programmed number of accepted external triggers.
- `S_DELAY (4)`: Delay countdown.
- `S_ERROR (5)`: Error state.

`cmd_done` is asserted for:

- command available in `S_IDLE`
- sync complete in `S_SYNC_CH`
- external-trigger count exhausted in `S_EXPECT_TRIG`
- delay counter exhausted in `S_DELAY`
- `CANCEL` command from any non-error state

## Trigger Generation and Lockout

Triggers are produced by:

- `FORCE_TRIG`
- `SYNC_CH` completion
- accepted external trigger events in `S_EXPECT_TRIG`

External-trigger acceptance requires:

- `state == S_EXPECT_TRIG`
- `ext_trig_counter > 0`
- `lockout_counter == 0`
- synchronized `ext_trig` high

After each accepted external trigger, `lockout_counter` reloads from `trig_lockout` and counts down to prevent rapid retriggering.

## Logging Format

If a trigger event has logging enabled (`do_log`), two FIFO words are written in consecutive cycles:

1. lower 32 bits of internal 64-bit timer
2. upper 32 bits of internal 64-bit timer

Writes are attempted only when both `data_buf_full == 0` and `data_buf_almost_full == 0`. Otherwise `data_buf_overflow` is set.

## Counters and Timer

- `trig_counter` increments on every emitted trigger (`trig_out` event source).
- `trig_counter` resets on `RESET_COUNT` or global reset.
- internal 64-bit timer:
  - resets on global reset and `RESET_COUNT`
  - starts at 1 on first logged trigger
  - increments each cycle while nonzero, saturating at `64'hFFFFFFFFFFFFFFFF`

## Error Handling

- `bad_cmd` is set when:
  - command decode is invalid, or
  - `SET_LOCKOUT` value is below minimum (4)
- `data_buf_overflow` is set when a log event occurs and FIFO cannot accept two words

Both flags are sticky until reset.

## Notes

- Command FIFO read behavior is intentional: `cmd_word_rd_en = do_next_cmd || reset_count`.
- `CANCEL` is allowed broadly (any non-error state), not just delay/expect states.
- External trigger detection is level-sensitive after synchronization, with lockout preventing immediate retriggering.

---
*See source code for detailed implementation and signal descriptions.*
