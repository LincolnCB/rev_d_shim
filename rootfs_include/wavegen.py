#!/usr/bin/env python3
"""
DAC Waveform Generator

This script generates waveform files for the DAC system in the format expected
by the shim-test command handler. The file format consists of lines starting
with 'D' (delay mode) or 'T' (trigger mode) followed by a timing value and
optional 8-channel values.

Format: [D|T] <value> [ch0 ch1 ch2 ch3 ch4 ch5 ch6 ch7]
- D/T: Delay or Trigger mode
- value: 32-bit unsigned integer (max 33554431)
- ch0-ch7: Optional signed 16-bit integers (-32767 to 32767)

Author: Generated for rev_d_shim project
"""

import math
import sys
import os

def get_user_input():
    """Get user input for waveform generation parameters."""
    print("DAC Waveform Generator")
    print("=" * 50)
    
    # Get clock frequency first
    while True:
        try:
            clock_freq = float(input("System clock frequency (MHz): "))
            if clock_freq > 0:
                break
            print("Clock frequency must be positive")
        except ValueError:
            print("Please enter a valid number")
    
    # Get waveform type
    while True:
        waveform_type = input("Select waveform type (sine/trap/trapezoid): ").strip().lower()
        if waveform_type in ['sine', 'trap', 'trapezoid']:
            # Normalize trap to trapezoid
            if waveform_type == 'trap':
                waveform_type = 'trapezoid'
            break
        print("Please enter 'sine' or 'trap'/'trapezoid'")
    
    # Get waveform-specific parameters
    if waveform_type == 'sine':
        params = get_sine_parameters(clock_freq)
    elif waveform_type == 'trapezoid':
        params = get_trapezoid_parameters(clock_freq)
    else:
        print("Unsupported waveform type")
        sys.exit(1)
    
    params['type'] = waveform_type
    params['clock_freq'] = clock_freq
    return params

def get_sine_parameters(clock_freq):
    """Get parameters specific to sine wave generation."""
    print("\n--- Sine Wave Parameters ---")
    
    while True:
        try:
            sample_rate = float(input("Sample rate (kHz, max 100): "))
            if 0 < sample_rate <= 100:
                break
            print("Sample rate must be between 0 and 100 kHz")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            duration = float(input("Duration (ms): "))
            if duration > 0:
                break
            print("Duration must be positive")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            frequency = float(input("Frequency (kHz): "))
            if frequency > 0:
                break
            print("Frequency must be positive")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            amplitude = float(input("Amplitude (volts, 0 to 4): "))
            if 0 <= amplitude <= 4:
                break
            print("Amplitude must be between 0 and 4 volts")
        except ValueError:
            print("Please enter a valid number")
    
    return {
        'sample_rate': sample_rate,
        'duration': duration,
        'frequency': frequency,
        'amplitude': amplitude
    }

def get_trapezoid_parameters(clock_freq):
    """Get parameters specific to trapezoid wave generation."""
    print("\n--- Trapezoid Wave Parameters ---")
    
    while True:
        try:
            sample_rate = float(input("Sample rate (kHz, max 100): "))
            if 0 < sample_rate <= 100:
                break
            print("Sample rate must be between 0 and 100 kHz")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            rise_time = float(input("Rise time (ms, max 100): "))
            if 0 < rise_time <= 100:
                break
            print("Rise time must be between 0 and 100 ms")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            flat_time = float(input("Flat time (ms, max 100): "))
            if 0 < flat_time <= 100:
                break
            print("Flat time must be between 0 and 100 ms")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            fall_time = float(input("Fall time (ms, max 100): "))
            if 0 < fall_time <= 100:
                break
            print("Fall time must be between 0 and 100 ms")
        except ValueError:
            print("Please enter a valid number")
    
    while True:
        try:
            amplitude = float(input("Amplitude (volts, 0 to 4): "))
            if 0 <= amplitude <= 4:
                break
            print("Amplitude must be between 0 and 4 volts")
        except ValueError:
            print("Please enter a valid number")
    
    return {
        'sample_rate': sample_rate,
        'rise_time': rise_time,
        'flat_time': flat_time,
        'fall_time': fall_time,
        'amplitude': amplitude
    }

def voltage_to_dac_value(voltage, amplitude):
    """
    Convert sine wave value to signed 16-bit DAC value.
    
    The amplitude parameter (0-4V) scales to the full DAC range.
    The sine wave value (-1 to +1) then uses the full +/- range.
    
    Args:
        voltage: Sine wave value (-1 to +1)
        amplitude: Maximum amplitude (0 to 4V)
    
    Returns:
        Signed 16-bit integer DAC value
    """
    # Scale amplitude (0-4V) to DAC range (0-32767)
    max_dac_value = int(amplitude * 32767 / 4.0)
    
    # Apply sine wave value (-1 to +1) to get full +/- range
    dac_value = int(voltage * max_dac_value)
    
    # Clamp to valid 16-bit signed range
    return max(-32767, min(32767, dac_value))

def generate_sine_wave(params):
    """
    Generate sine wave samples for 8 channels.
    
    Args:
        params: Dictionary with waveform parameters
    
    Returns:
        List of tuples, each containing 8 channel values for one sample
    """
    sample_rate_hz = params['sample_rate'] * 1000  # Convert kHz to Hz
    duration_s = params['duration'] / 1000         # Convert ms to seconds
    frequency_hz = params['frequency'] * 1000      # Convert kHz to Hz
    amplitude_v = params['amplitude']
    
    # Calculate number of samples
    num_samples = int(sample_rate_hz * duration_s)
    if num_samples == 0:
        num_samples = 1
    
    samples = []
    
    for i in range(num_samples):
        # Calculate time for this sample
        t = i / sample_rate_hz
        
        # Generate sine wave value (-1 to +1)
        sine_value = math.sin(2 * math.pi * frequency_hz * t)
        
        # Convert to DAC value using amplitude
        dac_value = voltage_to_dac_value(sine_value, amplitude_v)
        
        # Generate the same value for all 8 channels
        # (Could be modified to generate different waveforms per channel)
        channel_values = [dac_value] * 8
        
        samples.append(channel_values)
    
    return samples

def generate_trapezoid_wave(params):
    """
    Generate trapezoid wave samples for 8 channels.
    
    The trapezoid consists of:
    1. Rise phase: linear ramp up from 0 to amplitude
    2. Flat phase: constant amplitude (implemented with a single long delay)
    3. Fall phase: linear ramp down from amplitude to 0
    
    Args:
        params: Dictionary with waveform parameters
    
    Returns:
        List of sample data, where each sample is either:
        - (delay_cycles, [ch0, ch1, ..., ch7]) for regular samples
        - (long_delay_cycles, [ch0, ch1, ..., ch7]) for the flat section
    """
    sample_rate_hz = params['sample_rate'] * 1000  # Convert kHz to Hz
    sample_period_s = 1.0 / sample_rate_hz
    
    rise_time_s = params['rise_time'] / 1000        # Convert ms to seconds
    flat_time_s = params['flat_time'] / 1000        # Convert ms to seconds
    fall_time_s = params['fall_time'] / 1000        # Convert ms to seconds
    amplitude_v = params['amplitude']
    
    # Calculate number of samples for each phase
    rise_samples = max(1, int(rise_time_s / sample_period_s))
    fall_samples = max(1, int(fall_time_s / sample_period_s))
    
    # Calculate how many sample periods the flat time represents
    flat_sample_periods = flat_time_s / sample_period_s
    
    samples = []
    
    # Phase 1: Rise (linear ramp from 0 to amplitude)
    for i in range(rise_samples):
        # Linear interpolation from 0 to 1
        ramp_value = i / (rise_samples - 1) if rise_samples > 1 else 1.0
        
        # Convert to DAC value
        dac_value = voltage_to_dac_value(ramp_value, amplitude_v)
        
        # All channels get the same value
        channel_values = [dac_value] * 8
        samples.append(channel_values)
    
    # Phase 2: Flat section (single long delay with amplitude value)
    # The flat section is implemented as a single sample with a long delay
    # This keeps the raster time accurate while minimizing sample count
    flat_dac_value = voltage_to_dac_value(1.0, amplitude_v)  # Full amplitude
    flat_channel_values = [flat_dac_value] * 8
    
    # This sample will have a special long delay calculated later
    samples.append(('FLAT_SECTION', flat_channel_values, flat_sample_periods))
    
    # Phase 3: Fall (linear ramp from amplitude to 0)
    for i in range(fall_samples):
        # Linear interpolation from 1 to 0
        ramp_value = 1.0 - (i / (fall_samples - 1)) if fall_samples > 1 else 0.0
        
        # Convert to DAC value
        dac_value = voltage_to_dac_value(ramp_value, amplitude_v)
        
        # All channels get the same value
        channel_values = [dac_value] * 8
        samples.append(channel_values)
    
    return samples

def calculate_sample_delay(sample_rate_khz, clock_freq_mhz):
    """
    Calculate the delay value for the given sample rate.
    
    The delay value represents the number of clock cycles between samples.
    
    Args:
        sample_rate_khz: Sample rate in kHz
        clock_freq_mhz: System clock frequency in MHz
    
    Returns:
        Delay value as integer
    """
    # Convert frequencies to Hz
    sample_rate_hz = sample_rate_khz * 1000
    clock_freq_hz = clock_freq_mhz * 1000000
    
    # Calculate clock cycles per sample
    cycles_per_sample = clock_freq_hz / sample_rate_hz
    
    # Convert to integer delay value
    delay = int(cycles_per_sample)
    
    # Ensure delay is within valid range (1 to 33554431)
    return max(1, min(33554431, delay))

def write_waveform_file(filename, samples, sample_rate_khz, clock_freq_mhz, waveform_type):
    """
    Write samples to a DAC waveform file.
    
    Args:
        filename: Output filename
        samples: List of channel value tuples or special flat section entries
        sample_rate_khz: Sample rate in kHz
        clock_freq_mhz: System clock frequency in MHz
        waveform_type: Type of waveform (for comments)
    """
    delay_value = calculate_sample_delay(sample_rate_khz, clock_freq_mhz)
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            f.write("# DAC Waveform File\n")
            f.write(f"# Generated by wavegen.py\n")
            f.write(f"# Waveform type: {waveform_type}\n")
            f.write(f"# Sample rate: {sample_rate_khz} kHz\n")
            f.write(f"# Clock frequency: {clock_freq_mhz} MHz\n")
            f.write(f"# Normal delay value: {delay_value}\n")
            f.write("# Format: First command: T 1 <ch0-ch7>, Subsequent commands: D <delay> <ch0-ch7>\n")
            f.write("# Note: First command waits for trigger, subsequent commands use delay timing\n")
            f.write("#\n")
            
            # Count actual samples for reporting
            actual_sample_count = 0
            
            # Write samples
            for i, sample_data in enumerate(samples):
                # Determine if this is the first command (should be trigger type)
                is_first_command = (i == 0)
                
                # Check if this is a special flat section
                if (isinstance(sample_data, tuple) and len(sample_data) == 3 and 
                    sample_data[0] == 'FLAT_SECTION'):
                    
                    # Special flat section handling
                    _, channel_values, flat_sample_periods = sample_data
                    
                    # Calculate the long delay for the flat section
                    # This is the delay UNTIL the next sample (the fall begins)
                    flat_delay = int(flat_sample_periods * delay_value)
                    flat_delay = max(1, min(33554431, flat_delay))  # Clamp to valid range
                    
                    f.write(f"# Flat section: {flat_sample_periods:.2f} sample periods = {flat_delay} cycles\n")
                    
                    if is_first_command:
                        # First command: Trigger type waiting for 1 trigger
                        line = f"T 1"
                        f.write("# First command: Trigger type waiting for 1 trigger\n")
                    else:
                        # Regular delay command
                        line = f"D {flat_delay}"
                    
                    for ch_val in channel_values:
                        line += f" {ch_val}"
                    line += "\n"
                    f.write(line)
                    actual_sample_count += 1
                    
                else:
                    # Regular sample
                    channel_values = sample_data
                    
                    if is_first_command:
                        # First command: Trigger type waiting for 1 trigger
                        line = f"T 1"
                        f.write("# First command: Trigger type waiting for 1 trigger\n")
                    else:
                        # Regular delay command
                        line = f"D {delay_value}"
                    
                    for ch_val in channel_values:
                        line += f" {ch_val}"
                    line += "\n"
                    f.write(line)
                    actual_sample_count += 1
        
        print(f"Waveform file written to: {filename}")
        print(f"Number of samples: {actual_sample_count}")
        print(f"Normal sample delay: {delay_value}")
        
    except IOError as e:
        print(f"Error writing file {filename}: {e}")
        sys.exit(1)

def main():
    """Main function."""
    # Get user input
    params = get_user_input()
    
    # Generate waveform samples
    if params['type'] == 'sine':
        samples = generate_sine_wave(params)
    elif params['type'] == 'trapezoid':
        samples = generate_trapezoid_wave(params)
    else:
        print("Unsupported waveform type")
        sys.exit(1)
    
    if not samples:
        print("No samples generated")
        sys.exit(1)
    
    # Get output filename based on waveform type
    if params['type'] == 'sine':
        default_filename = f"sine_wave_{params['frequency']}khz_{params['amplitude']}v.wfm"
    elif params['type'] == 'trapezoid':
        default_filename = f"trap_wave_{params['rise_time']}ms_rise_{params['flat_time']}ms_flat_{params['fall_time']}ms_fall_{params['amplitude']}v.wfm"
    else:
        default_filename = "waveform.wfm"
    
    filename = input(f"Output filename (default: {default_filename}): ").strip()
    if not filename:
        filename = default_filename
    
    # Write waveform file
    write_waveform_file(filename, samples, params['sample_rate'], params['clock_freq'], params['type'])
    
    print("Waveform generation complete!")

if __name__ == "__main__":
    main()
