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
        waveform_type = input("Select waveform type (sine/trapezoid): ").strip().lower()
        if waveform_type in ['sine', 'trapezoid']:
            break
        print("Please enter 'sine' or 'trapezoid'")
    
    if waveform_type == 'trapezoid':
        print("Trapezoid waveform generation is not yet implemented.")
        sys.exit(1)
    
    # Get parameters for sine wave
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
        'type': waveform_type,
        'clock_freq': clock_freq,
        'sample_rate': sample_rate,
        'duration': duration,
        'frequency': frequency,
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

def write_waveform_file(filename, samples, sample_rate_khz, clock_freq_mhz):
    """
    Write samples to a DAC waveform file.
    
    Args:
        filename: Output filename
        samples: List of channel value tuples
        sample_rate_khz: Sample rate in kHz
        clock_freq_mhz: System clock frequency in MHz
    """
    delay_value = calculate_sample_delay(sample_rate_khz, clock_freq_mhz)
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            f.write("# DAC Waveform File\n")
            f.write(f"# Generated by wavegen.py\n")
            f.write(f"# Samples: {len(samples)}\n")
            f.write(f"# Sample rate: {sample_rate_khz} kHz\n")
            f.write(f"# Clock frequency: {clock_freq_mhz} MHz\n")
            f.write(f"# Delay value: {delay_value}\n")
            f.write("# Format: D <delay> <ch0> <ch1> <ch2> <ch3> <ch4> <ch5> <ch6> <ch7>\n")
            f.write("#\n")
            
            # Write samples
            for i, channel_values in enumerate(samples):
                # Use delay mode ('D') for all samples
                # Format: D <delay> <ch0> <ch1> <ch2> <ch3> <ch4> <ch5> <ch6> <ch7>
                line = f"D {delay_value}"
                for ch_val in channel_values:
                    line += f" {ch_val}"
                line += "\n"
                f.write(line)
        
        print(f"Waveform file written to: {filename}")
        print(f"Number of samples: {len(samples)}")
        print(f"Sample delay: {delay_value}")
        
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
    else:
        print("Unsupported waveform type")
        sys.exit(1)
    
    if not samples:
        print("No samples generated")
        sys.exit(1)
    
    # Get output filename
    default_filename = f"sine_wave_{params['frequency']}khz_{params['amplitude']}v.wfm"
    filename = input(f"Output filename (default: {default_filename}): ").strip()
    if not filename:
        filename = default_filename
    
    # Write waveform file
    write_waveform_file(filename, samples, params['sample_rate'], params['clock_freq'])
    
    print("Waveform generation complete!")

if __name__ == "__main__":
    main()
