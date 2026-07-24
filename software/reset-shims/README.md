# reset-shims

Small utility that safely resets the Rev D Shim hardware to a known idle state.

On run it:
1. Maps the system control register (`0x40000000`) via `/dev/mem`.
2. Powers the system off (control and power boards disabled).
3. Pulses the 17-bit command and data buffer resets (assert, brief delay, release).

Useful for recovering the buffers/state between runs without a full power cycle.

## Usage

```bash
reset-shims
```
