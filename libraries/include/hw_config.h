#ifndef HW_CONFIG_H
#define HW_CONFIG_H

// Board/signal constants shared across the shim software. Dependency-free (just values), so it
// is cheap to include anywhere a tool needs them, and is the single source of truth for these.

// Maximum number of channels the hardware supports (8 boards * 8 channels).
#define HW_MAX_CHANNELS 64
// Fastest SPI clock the bitstream is built and timed for (block_design.tcl, ports.xdc). Software
// never requests more; the PL clock snoop flags anything above about 31 MHz.
#define SPI_CLK_MAX_HZ 30000000u
// Maximum absolute current in amps for a DAC channel at full scale (+/-32767).
#define HW_MAX_ABS_AMPS 5.0
// Settle time in microseconds between a calibration DAC update and the first ADC read, so the
// DAC output and coil current stabilize before sampling. Used only by channel_cal.
#define CAL_SETTLE_DELAY 200

#endif // HW_CONFIG_H
