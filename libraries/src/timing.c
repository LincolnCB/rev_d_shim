#include "timing.h"

// Datasheet times in NiS (2^30 NiS = 1 s, rounded up), as in the RTL, so f * T >> 30 matches the
// hardware's cycle counts bit for bit.
#define DAC_T_UPDATE_NIS     892u  // AD5676: 830 ns between n_cs rising edges
#define DAC_T_MIN_NCS_NIS    33u   // AD5676: 30 ns min n_cs high
#define DAC_SPI_CMD_BITS     24u
#define DAC_NCS_MAX          31u
#define ADC_T_CONV_NIS       709u  // ADS8168: 660 ns conversion
#define ADC_T_CYCLE_NIS      1074u // ADS8168: 1000 ns cycle
#define ADC_OTF_CMD_BITS     16u
#define ADC_NCS_MAX          255u
#define NCS_MIN              3u

// ceil(f * t_nis / 2^30), the RTL's rounded-up shift-add result.
static uint32_t nis_to_cycles(uint32_t spi_clk_hz, uint32_t t_nis) {
  return (uint32_t)(((uint64_t)spi_clk_hz * t_nis + 0x3FFFFFFFull) >> 30);
}

uint32_t timing_dac_min_delay_cycles(uint32_t spi_clk_hz) {
  uint32_t upd = nis_to_cycles(spi_clk_hz, DAC_T_UPDATE_NIS);
  upd = (upd > DAC_SPI_CMD_BITS) ? upd - DAC_SPI_CMD_BITS : 0;
  uint32_t high = nis_to_cycles(spi_clk_hz, DAC_T_MIN_NCS_NIS);
  if (high < NCS_MIN) high = NCS_MIN;
  uint32_t ncs = (upd > high) ? upd : high;
  if (ncs > DAC_NCS_MAX) ncs = DAC_NCS_MAX;
  // Eight channel updates per DAC write
  return 8u * (ncs + DAC_SPI_CMD_BITS);
}

uint32_t timing_adc_min_delay_cycles(uint32_t spi_clk_hz) {
  uint32_t conv = nis_to_cycles(spi_clk_hz, ADC_T_CONV_NIS);
  if (conv < NCS_MIN) conv = NCS_MIN;
  uint32_t cyc = nis_to_cycles(spi_clk_hz, ADC_T_CYCLE_NIS);
  cyc = (cyc > ADC_OTF_CMD_BITS) ? cyc - ADC_OTF_CMD_BITS : 0;
  uint32_t ncs = (conv > cyc) ? conv : cyc;
  if (ncs > ADC_NCS_MAX) ncs = ADC_NCS_MAX;
  // Nine transactions per read: the last channel's data returns on a trailing dummy word
  return 9u * (ncs + ADC_OTF_CMD_BITS);
}

// A grid step's clock as the generator may actually produce it (TIMING_CLK_TOL_PCT low).
static uint32_t step_worst_hz(uint32_t step) {
  uint32_t hz = step * TIMING_CLK_STEP_HZ;
  return hz - hz / 100u * TIMING_CLK_TOL_PCT;
}

uint32_t timing_find_min_clk_hz(timing_fits_fn fits, void *arg) {
  uint32_t lo = TIMING_CLK_MIN_HZ / TIMING_CLK_STEP_HZ;
  uint32_t hi = TIMING_CLK_MAX_HZ / TIMING_CLK_STEP_HZ;
  if (!fits(step_worst_hz(hi), arg)) return 0;
  // Invariant: grid point hi fits; find the lowest one that does
  while (lo < hi) {
    uint32_t mid = lo + (hi - lo) / 2u;
    if (fits(step_worst_hz(mid), arg)) hi = mid;
    else lo = mid + 1u;
  }
  return hi * TIMING_CLK_STEP_HZ;
}
