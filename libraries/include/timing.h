#ifndef TIMING_H
#define TIMING_H

#include <stdbool.h>
#include <stdint.h>

// Software mirror of the PL timing calculators (ad5676_dac_timing_calc and ads816x_adc_timing_calc
// with ADS_MODEL_ID 8), so tools can predict the hardware's frequency-dependent minimum command
// delays without powering on -- and search for the slowest SPI clock a given timing needs. The
// hardware values (read from sys_sts after power-on) stay the source of truth for validation.

// SPI clock range the search covers, and its step (results land on whole 0.1 MHz values).
#define TIMING_CLK_MIN_HZ  1000000u
#define TIMING_CLK_MAX_HZ  50000000u
#define TIMING_CLK_STEP_HZ 100000u
// The clock generator may land up to this far below a requested frequency.
#define TIMING_CLK_TOL_PCT 1u

// Minimum delay between DAC update commands, in SPI clock cycles, at spi_clk_hz.
uint32_t timing_dac_min_delay_cycles(uint32_t spi_clk_hz);

// Minimum delay between ADC read commands, in SPI clock cycles, at spi_clk_hz.
uint32_t timing_adc_min_delay_cycles(uint32_t spi_clk_hz);

// Returns true if the caller's timing is playable at spi_clk_hz.
typedef bool (*timing_fits_fn)(uint32_t spi_clk_hz, void *arg);

// Binary-search the 0.1 MHz grid over [TIMING_CLK_MIN_HZ, TIMING_CLK_MAX_HZ] for the slowest clock
// where fits() holds, assuming a faster clock never makes the timing worse. Each candidate is
// tested TIMING_CLK_TOL_PCT below itself, so the result still fits if the generator lands low.
// Returns that clock in Hz (already rounded up to the next 0.1 MHz), or 0 if none fits.
uint32_t timing_find_min_clk_hz(timing_fits_fn fits, void *arg);

#endif // TIMING_H
