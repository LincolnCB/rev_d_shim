***TODO OUT OF DATE -- IN PROGRESS***

---

# Hardware
- [ ] Hardware completed in [[Verilog]]
- [ ] Hardware completed in [[RHDL]]

## Main Clock Domain
### Interfaces
- PS AXI Interface
	- IRQ in/out
		- #unfinished
	- In
		- [MMCM dynamic reconfig AXI interface](https://docs.amd.com/r/en-US/pg065-clk-wiz/Register-Space)
		- 32b - Trigger lockout
		- 16b - Calibration offset
		- 32b - Integrator threshold
		- 32b - Integrator timeframe
		- 1b - Enable
		- 1b - Integrator enable
	- Out
		- 416b - FIFO status (8x52b)
		  (Round up to 8x64b)
			- 24b - Write count
			- 24b - Read count
			- 1b - Empty
			- 1b - Full
			- 1b - Underflow
			- 1b - Overflow
		- 32b - Hardware status
			- 1b - Stopped
			- 31b - Status code
- DMA Interface
	- In
		- 8x DAC RAM to AXI stream
	- Out
		- 8x ADC AXI to RAM stream
- SPI Clock Domain Interface
	- In
		- 8x 16-bit ADC FIFO
	- Out
		- 1b - SPI Clk
		- 1b - SPI CD rst/en
		- 32b - Trigger lockout
		- 16b - Calibration offset
		- 8x 16-bit DAC FIFO
- External
	- In
		- 1b - Scanner clock
		- 1b - Shutdown sense
		- 3b - Shutdown sense select
	- Out
		- 1b - Shutdown force
		- 1b - Shutdown reset

### Cores
- [[SPI clock core]]
- Rst/En Core
- Hardware sense safety core
- AXI DMA Bridge
- Integrator safety core

## SPI Clock Domain
### Interfaces
- Main Clock Domain Interface
	- In
		- 1b - SPI clk
		- 1b - Rst/en
		- 32b - Trigger lockout
		- 16b - Calibration offset
		- 8x 16-bit DAC FIFO
	- Out
		- 8x 16-bit ADC FIFO
- External
	- In
		- 1b - Trigger
		- 8b - 8x DAC MISO SDA
		- 8b - 8x ADC MISO SDA
		- 8b - 8x MISO SCK
	- Out
		- 1b - MOSI SCK
		- 8b - 8x DAC MOSI SDA
		- 8b - 8x DAC CS
		- 8b - 8x DAC LDAC
		- 8b - 8x ADC SDA
		- 8b - 8x ADC CS
### Cores
- Trigger core
- 8x DAC core
- 8x ADC core


# Summary

- Software defined SPI clock core
	- Frequency and phase defined relative to 10 MHz scanner input clock
	- Handle clock domain crossings and proper update rate
- Emergency stop core (SPI clock domain)
	- Latches high on activation
	- Activates from:
		- Software interrupt
		- PL safety cores (see below)
		- Empty DAC FIFO
		- External E-stop button
	- Deactivates from Rst/En core
	- Sends:
		- RST signal to everything on SPI clock domain
		- Interrupt to software
		- Shutdown_Force pin on activation
		- ~Shutdown_Reset on deactivation
- Rst/En core
	- Does a buffer drain/reset
	- Resets the E-stop core
- Trigger core
	- Software-defined trigger lockout
	- Force trigger from PS
- Safety cores:
	- Integrator: Integrates both DAC and ADC separately over software-defined time relative to software-defined total threshold, triggers E-stop if passed
	- Shutdown sense: Cycles shutdown_sense_sel bits to strobe shutdown_sense across all DACs, triggering E-stop if any DAC has thermally latched
- 50 kHz DAC (AD5676) sampling rate. 
	- Each 8ch DAC gets an update every 20$\mu$s.
	- One update is a 8 24-bit command words. $\frac{24 \times 8}{20 \,\mu \text{s}}=9.6 \text{ MHz}$ SPI clock minimum
	- Preload DAC before first trigger, trigger goes to ~LDACn and starts next cycle
- 50 kHz ADC (AD7689) sampling rate
	- Same deal
- FIFO streaming to and from DMA (and to/from SD card)
	- $\frac{16 \times 8}{20 \,\mu \text{s}}=6.4 \text{ Mbps}$ stream in and out minimum, can be averaged over 25ms
	- Store 25ms of DAC on-chip (160 kbit) of memory buffer (for 32x1024 bit blocks, each DAC would use 5 BRAM blocks)
- Static shimming functionality
	- Use the extra 4 bits per 32 bits in the BRAM width to indicate trigger breaks.
	- Still load the next commands, just don't send LDAC. 
- Buffer drain/reset
	- Doesn't need to be done fast, 1 second is perfectly fine during scan breaks
	- Triggered from software or hardware
- Waveform is generated/loaded beforehand
- Calibration core
	- Measure offset and gain error using ADC and DAC via streaming, calculate in software
	- Write offset and slope adjustments to a calibration core that will adjust the DAC words as they're loaded from buffer
	- Datasheet maximum offset error is $\frac{\pm1.5\,\text{mV}}{4\,\text{V}} \times 2^{16}\,\text{LSB} = \pm25\,\text{LSB}$, 5 bits for signed offset error
	- Datasheet maximum gain error is $\pm 0.06\% \times 2^{16} \,\text{LSB} = \pm 30 \,\text{LSB}$, 5 bits for signed gain error
	- Core/software should handle 1 LSB precision for both
	- Ignore this offset in the DAC integrator core
	- Do NOT use this for shimming!

Software goals:
- One main CLI tool that stays activated the entire time.
	- PL input registers:
		- E-stop and reset
		- SPI clock freq and phase
		- Buffer drain
		- Integrator threshold and timeframe
		- Calibration offset and slope (per DAC. per channel?)
		- Trigger lockout
	- PL output registers:
		- FIFO status
		- Stopped
		- Reading any values set above
	- Commands (any set values can be from a config file loaded on init):
		- Stop: Manually E-stop. Should also be triggered by ctrl+c
		- Reset: Pulse reset to reset the system, including E-stop. Do we need to re-set register values as part of this?
		- Calibrate: Run calibration and set calibration values
		- Empty buffer: Drain/reset the buffer
		- Set/read clock
		- Set/read safety threshold
		- Set/read calibration
		- Set/read lockout
		- Manual trigger
		- Static shim: simple commands to manually set shim levels
		- Start stream: connect a DAC waveform input file and ADC sample output file and start a second stream that keeps the DMA flowing
		- Stop stream: stop the streams and reset the system
		- Read temp: Read temperatures off of I2C bus
- Backup CLI safety tool that just sends a shutdown in case the first crashes in a weird way

Milestones:
- Parity with original OCRA code -- DAC playback works with load on the bench
- Software-defined SPI clock
- Trigger core
- E-stop system with input from software interrupt
- E-stop error codes and interrupt to software
- **Experiment:** Noise test --> RF noise floor variance changes less than 2% when DACs and ADCs operate during image encode readout window
- New trigger core with force trigger from software (for debugging, static shim).
- DAC streaming with DMA on the bench
- **Experiment:** Successfully update slice-by-slice shims in 2D EPI acquisitions and verify that currents settle in < 1ms and cause no image detectable image artifacts
- **Experiment:** Play out an arbitrary waveform during an MRI acquisition using a local encoding coil and verify that image encoding occurs as expected.  For example, play out trapezoid diffusion encode waveforms in a triggered 2D spin-echo EPI sequence (classic Stejskal-Tanner experiment).
- **Experiment:** Create Pulseq acquisition with chemical shift imaging to map the applied field field in phantom at every point during readout to measure the field and verify that it follows the current accurately; use for image recon calibration if needed.
- ADC streaming with DMA on the bench
- Buffer drain
- Calibration core with software tool
- Integrator
- Shutdown sense
