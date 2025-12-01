#!/usr/bin/env python3
"""
Unified DAC Waveform Converter

Converts .npy, .csv, or .mat files to DAC waveform files for the rev_d_shim project.
Handles optional time vector in the input file.

- For .mat: Prompts for variable names for channels and (optionally) time.
- For .csv/.npy: Prompts if the first column is a time vector.

Author: Consolidated from waveform_from_npy.py, waveform_from_csv.py, waveform_from_mat.py
"""

# --- User Option: Dump time+channelA as .npy for import_npy ---
DUMP_NPY = False  # Set True to enable dumping time+channelA as .npy
DUMP_NPY_FILENAME = None  # Set to None for auto-naming, or provide a string

import sys
import os
import numpy as np
import math
import csv

try:
  import scipy.io
except ImportError:
  scipy = None

MAX_DELAY = 2**25 - 1  # Maximum delay value supported by hardware

def prompt(msg, default=None, type_=str, allow_empty=False):
  while True:
    val = input(msg)
    if val == '' and default is not None:
      return default
    if val == '' and allow_empty:
      return ''
    try:
      return type_(val)
    except Exception:
      print(f"Invalid input. Expected {type_.__name__}.")

def prompt_yes_no(msg, default=False):
  while True:
    val = input(f"{msg} [{'Y/n' if default else 'y/N'}]: ").strip().lower()
    if val == '' and default is not None:
      return default
    if val in ('y', 'yes'): return True
    if val in ('n', 'no'): return False
    if val == '': return default
    print("Please enter y or n.")

def current_to_dac_value(current_amps):
  arr = np.asarray(current_amps)
  # arr shape: [n_bd, 8, n_samples] or [8, n_samples] or [n_samples]
  dac = np.round(arr * 32767 / 5.1).astype(int)
  dac = np.clip(dac, -32767, 32767)
  return dac

def calculate_sample_delay(sample_rate_ksps, spi_clock_freq_mhz):
  sample_rate_hz = sample_rate_ksps * 1000
  spi_clock_freq_hz = spi_clock_freq_mhz * 1e6
  cycles_per_sample = spi_clock_freq_hz / sample_rate_hz
  delay = int(cycles_per_sample)
  return max(1, min(MAX_DELAY, delay))

def create_zeroed_samples(sample_count, n_channels):
  return [[0]*n_channels for _ in range(sample_count)]

def trim_and_zero_channels(samples_A):
  """
  Interactively prompt user for channel range to keep, zero and trim others, and pad to multiple of 8.
  Returns (samples_A, ch_start, ch_end, n_channels, n_bd)
  """
  print(f"Imported data shape: {samples_A.shape}")
  total_channels = samples_A.shape[1] if samples_A.ndim > 1 else 1
  print(f"Total channels in data: {total_channels}")
  if total_channels > 1:
    while True:
      ch_range = input(f"Enter channel range to keep (e.g. 0-7, default 0-{total_channels-1}): ").strip()
      if ch_range == '':
        ch_start, ch_end = 0, total_channels
        break
      if '-' in ch_range:
        parts = ch_range.split('-')
        try:
          ch_start = int(parts[0])
          ch_end = int(parts[1]) + 1
          if 0 <= ch_start < ch_end <= total_channels:
            break
        except Exception:
          pass
      print(f"Invalid range. Please enter as start-end, e.g. 0-7.")
    # Zero and trim channels outside selected range
    if ch_start > 0:
      samples_A[:, :ch_start] = 0.0
    if ch_end < total_channels:
      samples_A[:, ch_end:] = 0.0
    samples_A = samples_A[:, ch_start:ch_end]
    print(f"Trimmed to channels {ch_start}-{ch_end-1}, new shape: {samples_A.shape}")
    n_channels = samples_A.shape[1]
    n_bd = (n_channels + 7) // 8
    if n_channels < n_bd * 8:
      print(f"Padding from {n_channels} to {n_bd * 8} channels with zeros")
      padding = np.zeros((samples_A.shape[0], n_bd * 8 - n_channels))
      samples_A = np.concatenate([samples_A, padding], axis=1)
      n_channels = samples_A.shape[1]
  else:
    ch_start, ch_end = 0, 1
    n_bd = 1
    n_channels = 1
  # Reshape to [n_bd, 8, n_samples]
  n_samples = samples_A.shape[0]
  bd_samples_A = samples_A.T.reshape(n_bd, 8, n_samples)
  print(f"Final array shape: {bd_samples_A.shape}")
  return bd_samples_A

def write_waveform_file(filename, time, samples, spi_clock_freq_mhz, src_filename, is_zeroed=False):
  # samples: [8, n_samples], time: [n_samples] in clock cycles
  n_samples = samples.shape[1]
  max_delay = MAX_DELAY
  try:
    with open(filename, 'w') as f:
      waveform_type = "Zeroed DAC Waveform" if is_zeroed else "DAC Waveform"
      f.write(f"# {waveform_type} File\n")
      f.write(f"# Source file: {src_filename}\n")
      f.write(f"# SPI clock frequency: {spi_clock_freq_mhz:.6g} MHz\n")
      f.write(f"# Number of samples: {n_samples}\n")
      f.write(f"# Board: {filename}\n")
      f.write(f"# Channels: 8\n")
      f.write("# Format: T 1 <ch0-ch7> (trigger) / D <delay> <ch0-ch7> (delay)\n")

      prev_time = None
      prev_vals = None
      for i in range(n_samples):
        t = time[i]
        vals = samples[:, i]
        if i == 0 or t == 0:
          f.write(f"T 1" + ''.join(f" {v}" for v in vals) + "\n")
          prev_time = t
          prev_vals = vals.copy()
          continue
        if prev_time is None:
          raise ValueError("First sample must have time == 0")
        if t < prev_time:
          raise ValueError(f"Time decreased at sample {i}: {t} < {prev_time}")
        delay = t - prev_time
        if delay <= 0:
          raise ValueError(f"Non-positive delay at sample {i}: {delay}")
        # If values are the same as previous, skip (accumulate delay)
        if prev_vals is not None and np.array_equal(vals, prev_vals) and i != n_samples - 1:
          # Only skip if not the final sample
          continue
        delay_left = delay
        # If delay > max_delay, emit D commands with max_delay-1000 until delay fits
        while delay_left > max_delay:
          emit_delay = max_delay - 1000
          f.write(f"D {emit_delay}" + ''.join(f" {v}" for v in vals) + "\n")
          prev_time += emit_delay
          delay_left = t - prev_time
        f.write(f"D {delay_left}" + ''.join(f" {v}" for v in vals) + "\n")
        prev_time = t
        prev_vals = vals.copy()
    print(f"Waveform file written to: {filename}")
  except IOError as e:
    print(f"Error writing file {filename}: {e}")
    sys.exit(1)

def calculate_dac_durations(time):
  """
  Given a time vector in clock cycles, return a list of durations (in clock cycles) for each trigger segment.
  Each duration is the last non-zero time before the next time==0 (or end).
  """
  time = np.asarray(time)
  zero_idxs = np.where(time == 0)[0]
  durations = []
  for i, idx in enumerate(zero_idxs):
    if i + 1 < len(zero_idxs):
      next_idx = zero_idxs[i+1]
      seg = time[idx:next_idx]
    else:
      seg = time[idx:]
    # Find last nonzero time in this segment
    nonzero = seg[seg > 0]
    if len(nonzero) > 0:
      duration = int(nonzero[-1])
    else:
      duration = 0
    durations.append(duration)
  return durations

def write_adc_readout_file(filename, durations_cycles, adc_sample_rate_ksps, extra_time_ms, spi_clock_freq_mhz):
  try:
    with open(filename, 'w') as f:
      # Convert extra_cycles back to true extra_time_ms (in case of rounding)
      extra_cycles = int(round(extra_time_ms * 1e-3 * spi_clock_freq_mhz * 1e6))
      true_extra_time_ms = extra_cycles / (spi_clock_freq_mhz * 1e6) * 1e3
      # Convert durations_cycles to ms for comment
      durations_ms = [cycles / (spi_clock_freq_mhz * 1e6) * 1e3 for cycles in durations_cycles]
      f.write("# ADC Readout Command File\n")
      f.write(f"# Duration{'s' if len(durations_ms) > 1 else ''} (ms): {[f'{d:.3f}' for d in durations_ms]}\n")
      f.write(f"# Extra sample time: {true_extra_time_ms:.6g} ms\n")
      f.write(f"# ADC sample rate: {adc_sample_rate_ksps:.6g} ksps\n")
      f.write(f"# SPI clock frequency: {spi_clock_freq_mhz:.6g} MHz\n")
      f.write("O 0 1 2 3 4 5 6 7\n")
      for i, dur_cycles in enumerate(durations_cycles):
        total_cycles = dur_cycles + extra_cycles
        adc_delay_value = calculate_sample_delay(adc_sample_rate_ksps, spi_clock_freq_mhz)
        # Estimate total samples as total_cycles // adc_delay_value
        total_samples = max(1, total_cycles // adc_delay_value)
        f.write("T 1\n")
        repeat_count = total_samples - 1
        f.write(f"D {adc_delay_value} {repeat_count}\n")
    print(f"ADC readout file written to: {filename}")
  except IOError as e:
    print(f"Error writing ADC readout file {filename}: {e}")
    sys.exit(1)

def get_adc_readout_parameters(dac_params):
  print("\n--- ADC Readout Parameters ---")
  default_adc_rate = dac_params['sample_rate'] if dac_params['sample_rate'] is not None else 50
  while True:
    try:
      if default_adc_rate is not None:
        adc_sample_rate_input = input(f"ADC sample rate (ksps, default {default_adc_rate:.6g}): ").strip()
        if adc_sample_rate_input == '':
          adc_sample_rate = default_adc_rate
        else:
          adc_sample_rate = float(adc_sample_rate_input)
      else:
        adc_sample_rate_input = input("ADC sample rate (ksps, required): ").strip()
        if adc_sample_rate_input == '':
          print("ADC sample rate is required when waveform sample rate is unknown.")
          continue
        adc_sample_rate = float(adc_sample_rate_input)
      if adc_sample_rate > 0:
        break
      print("ADC sample rate must be positive")
    except ValueError:
      print("Please enter a valid number")
  while True:
    try:
      extra_time = float(input("Extra sample time after DAC completes (ms): "))
      if extra_time >= 0:
        break
      print("Extra sample time must be non-negative")
    except ValueError:
      print("Please enter a valid number")
  return {
    'adc_sample_rate': adc_sample_rate,
    'adc_extra_time': extra_time
  }


def import_mat(filename, has_time, sample_rate=None):
  if scipy is None:
    print("scipy.io is required for .mat files.")
    sys.exit(1)
  mat = scipy.io.loadmat(filename)
  var_names = [k for k in mat.keys() if not k.startswith('__')]
  print("Variables in .mat file:")
  for i, name in enumerate(var_names):
    arr = mat[name]
    print(f"  [{i}] {name}: shape {arr.shape}, dtype {arr.dtype}")
  # Select shim channel variable
  while True:
    idx = input(f"Select variable for shim channels (0-{len(var_names)-1}): ").strip()
    if idx.isdigit() and 0 <= int(idx) < len(var_names):
      shim_var = var_names[int(idx)]
      break
    print("Invalid selection.")
  data = np.array(mat[shim_var])
  if has_time:
    while True:
      idx = input(f"Select variable for time points (0-{len(var_names)-1}): ").strip()
      if idx.isdigit() and 0 <= int(idx) < len(var_names):
        time_var = var_names[int(idx)]
        break
      print("Invalid selection.")
    time = np.array(mat[time_var]).squeeze()
    if data.ndim == 1:
      data = data[:, np.newaxis]
    # Find which axis in data matches the time length
    match_axis = None
    for axis, dim in enumerate(data.shape):
      if dim == time.shape[0]:
        match_axis = axis
        break
    if match_axis is None:
      print(f"Error: None of the shim array axes match the time array length ({time.shape[0]}). Shim shape: {data.shape}, time shape: {time.shape}")
      sys.exit(1)
    # Move the matching axis to axis 0 (samples), the other to axis 1 (channels)
    if match_axis != 0:
      data = np.moveaxis(data, match_axis, 0)
    # Now data.shape[0] == time.shape[0]
  else:
    # Generate time vector from sample_rate
    if sample_rate is None:
      raise ValueError("sample_rate must be provided if no time vector")
    n_samples = data.shape[0]
    time = np.arange(n_samples) / (sample_rate * 1e3)  # seconds
  channels = data.shape[1] if data.ndim > 1 else 1
  if channels < 1 or channels > 64:
    print(f"Error: Number of channels must be 1-64, got {channels}")
    sys.exit(1)
  n_bd = (channels + 7) // 8
  if channels < n_bd * 8:
    print(f"Padding from {channels} to {n_bd * 8} channels with zeros")
    padding = np.zeros((data.shape[0], n_bd * 8 - channels))
    data = np.concatenate([data, padding], axis=1)
  n_channels = data.shape[1]
  return time, data

def import_csv(filename, has_time, sample_rate=None):
  with open(filename, newline='') as f:
    reader = csv.reader(f)
    rows = [row for row in reader if row and any(cell.strip() for cell in row)]
  if not rows:
    print("No data found in CSV.")
    sys.exit(1)
  arr = np.array([[float(x) for x in row] for row in rows])
  if has_time:
    time = arr[:, 0]
    data = arr[:, 1:]
  else:
    data = arr
    if sample_rate is None:
      raise ValueError("sample_rate must be provided if no time vector")
    n_samples = data.shape[0]
    time = np.arange(n_samples) / (sample_rate * 1e3)  # seconds
  channels = data.shape[1]
  if channels > 64:
    print(f"Error: Number of channels must be <= 64, got {channels}")
    sys.exit(1)
  n_bd = math.ceil(channels / 8)
  if channels < n_bd * 8:
    print(f"Padding from {channels} to {n_bd * 8} channels with zeros")
    padding = np.zeros((data.shape[0], n_bd * 8 - channels))
    data = np.concatenate([data, padding], axis=1)
  n_channels = data.shape[1]
  return time, data

def import_npy(filename, has_time, sample_rate=None):
  arr = np.load(filename)
  if arr.ndim == 1:
    arr = arr[:, np.newaxis]
  if has_time:
    time = arr[:, 0]
    data = arr[:, 1:]
  else:
    data = arr
    if sample_rate is None:
      raise ValueError("sample_rate must be provided if no time vector")
    n_samples = data.shape[0]
    time = np.arange(n_samples) / (sample_rate * 1e3)  # seconds
  channels = data.shape[1]
  if channels < 1 or channels > 64:
    print(f"Error: Number of channels must be 1-64, got {channels}")
    sys.exit(1)
  n_bd = (channels + 7) // 8
  if channels < n_bd * 8:
    print(f"Padding from {channels} to {n_bd * 8} channels with zeros")
    padding = np.zeros((data.shape[0], n_bd * 8 - channels))
    data = np.concatenate([data, padding], axis=1)
  n_channels = data.shape[1]
  return time, data

def main():
  # Get filename
  if len(sys.argv) > 1:
    filename = sys.argv[1]
  else:
    filename = input("Path to input file (.npy/.csv/.mat): ").strip()
  if not os.path.exists(filename):
    print(f"File not found: {filename}")
    sys.exit(1)
  ext = os.path.splitext(filename)[1].lower()

  # Ask if there is a time vector
  has_time = prompt_yes_no("Does the file include a time vector column?", default=True)

  # Get SPI clock and sample rate
  spi_clock_freq = prompt("SPI clock frequency (MHz, default 50): ", default=50, type_=float)
  if not has_time:
    sample_rate = prompt("Sample rate (ksps): ", type_=float)
  else:
    sample_rate = None
  
  # Get additional options
  create_adc_readout = prompt_yes_no("Create ADC readout command file?", default=True)
  create_zero_waveform = prompt_yes_no("Create equivalent zeroed waveform?", default=True)
  zero_at_end = prompt_yes_no("Zero at the end?", default=True)
  params = {
    'sample_rate': sample_rate,
    'spi_clock_freq': spi_clock_freq,
    'create_adc_readout': create_adc_readout,
    'create_zero_waveform': create_zero_waveform,
    'zero_at_end': zero_at_end
  }
  if create_adc_readout:
    adc_params = get_adc_readout_parameters(params)
    params.update(adc_params)
  
  # Load data and process by filetype
  if ext == '.mat':
    time, samples_A = import_mat(filename, has_time, sample_rate=sample_rate)
  elif ext == '.csv':
    time, samples_A = import_csv(filename, has_time, sample_rate=sample_rate)
  elif ext == '.npy':
    time, samples_A = import_npy(filename, has_time, sample_rate=sample_rate)
  else:
    print(f"Unsupported file extension: {ext}")
    sys.exit(1)
  

  # If zero_at_end, append a final zero sample if not already present
  if zero_at_end:
    # Check if last sample is already all zeros
    if not np.all(samples_A[-1] == 0):
      zero_row = np.zeros((1, samples_A.shape[1]))
      samples_A = np.vstack([samples_A, zero_row])
      # Prompt for time (in ms) to wait before the final zero
      while True:
        try:
          extra_zero_time_ms = float(input("Time (ms) to wait before final zero sample: "))
          if extra_zero_time_ms >= 0:
            break
          print("Time must be non-negative")
        except ValueError:
          print("Please enter a valid number")
      # Set the final time entry
      time = np.append(time, time[-1] + extra_zero_time_ms / 1e3)

  # --- Dump time+channelA as .npy if enabled ---
  if DUMP_NPY:
    # Compose array: first column is time, rest are channel A values
    arr = np.column_stack((time, samples_A))
    if DUMP_NPY_FILENAME:
      npy_out = DUMP_NPY_FILENAME
    else:
      base = os.path.splitext(os.path.basename(filename))[0]
      npy_out = f"{base}_dump.npy"
    np.save(npy_out, arr)
    print(f"[DUMP_NPY] Saved time+channelA as: {npy_out}")

  # Convert and trim channels and time
  bd_samples_A = trim_and_zero_channels(samples_A)
  bd_samples_DAC = current_to_dac_value(bd_samples_A)

  # Convert time vector to integer clock cycles after trimming
  spi_clock_freq = params['spi_clock_freq']
  time_cycles = np.round(time * spi_clock_freq * 1e6).astype(int)
  # Ensure time_cycles is monotonic and starts at zero
  time_cycles = time_cycles - time_cycles[0]

  # Get output filename
  if bd_samples_DAC.size == 0:
    print("No samples to write")
    sys.exit(1)
  default_filename = os.path.splitext(os.path.basename(filename))[0]
  if params['sample_rate'] is not None:
    default_filename = f"{default_filename}_{params['sample_rate']:.0f}ksps"
  outname = input(f"Output filename (default: {default_filename}.[wfm/rdout]): ").strip()
  if not outname:
    outname = default_filename

  # Write waveform files
  n_bd = bd_samples_DAC.shape[0]
  n_samples = bd_samples_DAC.shape[2]
  for bd in range(n_bd):
    wfm_filename = f"{outname}_bd{bd}.wfm" if n_bd > 1 else (outname if outname.endswith('.wfm') else f"{outname}.wfm")
    write_waveform_file(wfm_filename, time_cycles, bd_samples_DAC[bd], params['spi_clock_freq'], filename, is_zeroed=False)

  if params.get('create_zero_waveform', False):
    # Make zero waveform: one trigger and one D per duration, D delay = duration
    zero_samples = np.zeros_like(bd_samples_DAC[0])
    zero_filename = f"{outname}_zero.wfm"
    durations_cycles = calculate_dac_durations(time_cycles)
    try:
      with open(zero_filename, 'w') as f:
        f.write("# Zeroed DAC Waveform File (trigger and D per duration)\n")
        f.write(f"# Source file: {filename}\n")
        f.write(f"# SPI clock frequency: {params['spi_clock_freq']:.6g} MHz\n")
        f.write(f"# Board: {zero_filename}\n")
        f.write(f"# Channels: 8\n")
        f.write("# Format: T 1 <ch0-ch7> (trigger) / D <delay> <ch0-ch7> (delay)\n")
        for dur in durations_cycles:
          f.write("T 1" + ''.join(f" 0" for _ in range(zero_samples.shape[0])) + "\n")
          f.write(f"D {dur}" + ''.join(f" 0" for _ in range(zero_samples.shape[0])) + "\n")
      print(f"Zeroed waveform file written to: {zero_filename}")
    except IOError as e:
      print(f"Error writing zeroed waveform file {zero_filename}: {e}")
      sys.exit(1)
  
  # Write ADC readout file if requested
  if params.get('create_adc_readout', False):
    # Use trigger/duration segmentation based on time_cycles
    durations_cycles = calculate_dac_durations(time_cycles)
    rdout_filename = outname if outname.endswith('.rdout') else f"{outname}.rdout"
    write_adc_readout_file(
      rdout_filename,
      durations_cycles,
      params['adc_sample_rate'],
      params['adc_extra_time'],
      params['spi_clock_freq']
    )
  print("Waveform generation complete!")

if __name__ == "__main__":
  main()
