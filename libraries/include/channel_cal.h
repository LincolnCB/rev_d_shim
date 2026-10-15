#ifndef CHANNEL_CAL_H
#define CHANNEL_CAL_H

#include <stdbool.h>

#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "sys_sts.h"
#include "dma_ctrl.h"

// Unified per-channel DAC/ADC calibration, shared by every tool that calibrates.
// A channel is calibrated by sweeping a symmetric DAC ramp, averaging the ADC reading
// at each point, fitting a line, and folding the fitted offset back into the channel's
// SET_CAL register over a few correction passes. The sweep/fit/correct logic is identical
// across transports; only the three hardware primitives it needs -- write a DAC value,
// read an ADC sample, set a calibration value -- differ, and those sit behind cal_transport_t.
//
// Transport notes:
//  - CAL_XPORT_PIO drives the DAC command and ADC data FIFOs directly over the PIO bridge.
//  - CAL_XPORT_DMA pushes the DAC write / SET_CAL words through MM2S and drains the ADC
//    sample through S2MM, matching the datapath_mode that gates those two lanes to the DMA
//    engine. The ADC command lane stays PIO in both modes, so issuing the read is identical.
//    A DMA ctx therefore requires a mapped dma handle; a DMA run is many small round-trips,
//    so it is slower than a PIO run -- tune averages/iterations down if that matters.
//
// The caller owns hardware setup: the system must be running (and in the matching
// datapath_mode for the chosen transport) with drained command/data FIFOs before the run.

typedef enum {
  CAL_XPORT_PIO = 0, // DAC write + ADC data over the PIO FIFO bridge
  CAL_XPORT_DMA = 1  // DAC write + ADC data over MCDMA (MM2S / S2MM)
} cal_transport_t;

// Low-level handles the calibration needs. dma may be NULL for a PIO-only tool; it is
// required (and must be mapped) for a CAL_XPORT_DMA run. Populate it with a helper near
// the tool's own hardware struct (see channel_cal_ctx_from_* in the tools).
typedef struct {
  struct dac_ctrl_t *dac;  // DAC command FIFO (DAC_WR_CH / SET_CAL)
  struct adc_ctrl_t *adc;  // ADC command FIFO (ADC_RD_CH) + PIO data readback
  struct sys_sts_t  *sts;  // hardware status + FIFO fill counts
  struct dma_ctrl_t *dma;  // MCDMA mover, required for CAL_XPORT_DMA
} channel_cal_ctx_t;

// Shape of the calibration sweep. Defaults below match the historical routine.
typedef struct {
  int    samples_per_ramp;    // DAC points per ramp (>= 2)
  double abs_scale_per_ramp;  // ramp spans [-scale, +scale] in DAC codes
  int    averages_per_sample; // ADC reads averaged at each ramp point (>= 1)
  int    correction_iters;    // offset-correction passes per channel (>= 1)
} channel_cal_params_t;

#define CHANNEL_CAL_DEFAULT_SAMPLES   5
#define CHANNEL_CAL_DEFAULT_SCALE     3000.0
#define CHANNEL_CAL_DEFAULT_AVERAGES  15
#define CHANNEL_CAL_DEFAULT_ITERS     4

// The historical defaults (5-point +/-3000 ramp, 15 averages, 4 correction passes).
channel_cal_params_t channel_cal_default_params(void);

// Calibrate ch_count channels starting at absolute channel ch_start (board = ch/8,
// channel = ch%8). Prints one result line per channel. Returns 0 if every channel
// calibrated cleanly, negative if any channel failed or the hardware halted mid-run.
int channel_cal_run(const channel_cal_ctx_t *ctx, cal_transport_t xport,
                    int ch_start, int ch_count,
                    channel_cal_params_t params, bool verbose);

// channel_cal_run with the default sweep shape.
int channel_cal_run_default(const channel_cal_ctx_t *ctx, cal_transport_t xport,
                            int ch_start, int ch_count, bool verbose);

#endif // CHANNEL_CAL_H
