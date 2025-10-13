#!/usr/bin/env python3
"""
DAC Waveform Generator from Numpy Array

This script generates waveform files for the DAC system from numpy array files.
The numpy array should be in the format [samples]x[channels] where channels can
be 1-8. If fewer than 8 channels, zeros are added to pad to 8 channels.

The values in the numpy array should be between -5.1 and 5.1 representing 
current in amps. These are converted to signed 16-bit DAC values.

The output file format consists of lines starting with 'T' (trigger mode) for
the first command, followed by 'D' (delay mode) commands with timing values
and 8-channel values.

Format: [T|D] <value> [ch0 ch1 ch2 ch3 ch4 ch5 ch6 ch7]
- T/D: Trigger or Delay mode  
- value: 32-bit unsigned integer (max 33554431) for delay cycles
- ch0-ch7: Signed 16-bit integers (-32767 to 32767)

Author: Generated for rev_d_shim project
"""

import numpy as np
import sys
import os

def get_user_input(numpy_file=None):
    """Get user input for waveform generation parameters."""
    print("DAC Waveform Generator from Numpy Array")
    print("=" * 50)
    
    # Get numpy file path from argument or prompt
    if numpy_file is None:
        while True:
            numpy_file = input("Path to numpy array file (.npy): ").strip()
            if os.path.exists(numpy_file):
                break
            print(f"File not found: {numpy_file}")
    else:
        # Validate provided file path
        if not os.path.exists(numpy_file):
            print(f"Error: File not found: {numpy_file}")
            sys.exit(1)
        print(f"Using numpy file: {numpy_file}")
    
    # Get SPI clock frequency
    while True:
        try:
            spi_clock_freq = float(input("SPI clock frequency (MHz): "))
            if spi_clock_freq > 0:
                break
            print("SPI clock frequency must be positive")
        except ValueError:
            print("Please enter a valid number")
    
    # Get sample rate
    while True:
        try:
            sample_rate = float(input("Sample rate (ksps): "))
            if sample_rate > 0:
                break
            print("Sample rate must be positive")
        except ValueError:
            print("Please enter a valid number")
    
    # Ask about ADC readout file
    while True:
        adc_input = input("Create ADC readout command file? [y/N]: ").strip().lower()
        if adc_input in ['', 'n', 'no']:
            create_adc_readout = False
            break
        elif adc_input in ['y', 'yes']:
            create_adc_readout = True
            break
        print("Please enter 'y' for yes or 'n' for no")
    
    # Ask about equivalent zeroed waveform
    while True:
        zero_input = input("Create equivalent zeroed waveform? [y/N]: ").strip().lower()
        if zero_input in ['', 'n', 'no']:
            create_zero_waveform = False
            break
        elif zero_input in ['y', 'yes']:
            create_zero_waveform = True
            break
        print("Please enter 'y' for yes or 'n' for no")
    
    # Ask about zeroing the final point
    while True:
        zero_end_input = input("Zero at the end? [y/N]: ").strip().lower()
        if zero_end_input in ['', 'n', 'no']:
            zero_at_end = False
            break
        elif zero_end_input in ['y', 'yes']:
            zero_at_end = True
            break
        print("Please enter 'y' for yes or 'n' for no")
    
    params = {
        'numpy_file': numpy_file,
        'spi_clock_freq': spi_clock_freq,
        'sample_rate': sample_rate,
        'create_adc_readout': create_adc_readout,
        'create_zero_waveform': create_zero_waveform,
        'zero_at_end': zero_at_end
    }
    
    # Get ADC readout parameters if requested
    if create_adc_readout:
        adc_params = get_adc_readout_parameters(params)
        params.update(adc_params)
    
    return params

def get_adc_readout_parameters(dac_params):
    """Get parameters for ADC readout file generation."""
    print("\n--- ADC Readout Parameters ---")
    
    # Default ADC sample rate to match DAC sample rate
    default_adc_rate = dac_params['sample_rate']
    
    while True:
        try:
            adc_sample_rate_input = input(f"ADC sample rate (ksps, default {default_adc_rate:.6g}): ").strip()
            if adc_sample_rate_input == '':
                adc_sample_rate = default_adc_rate
            else:
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

def load_and_validate_numpy_array(filename):
    """
    Load numpy array and validate its format.
    
    Args:
        filename: Path to numpy array file
        
    Returns:
        Numpy array with shape [samples, 8] where values are between -5.1 and 5.1
    """
    try:
        # Load numpy array
        data = np.load(filename)
        print(f"Loaded array with shape: {data.shape}")
        
        # Validate dimensions
        if data.ndim != 2:
            print(f"Error: Array must be 2D, got {data.ndim}D")
            sys.exit(1)
        
        samples, channels = data.shape
        
        if channels < 1 or channels > 8:
            print(f"Error: Number of channels must be between 1 and 8, got {channels}")
            sys.exit(1)
        
        # Check value range
        min_val = np.min(data)
        max_val = np.max(data)
        print(f"Value range: {min_val:.6f} to {max_val:.6f}")
        
        if min_val < -5.1 or max_val > 5.1:
            print(f"Warning: Values outside recommended range [-5.1, 5.1]")
            print(f"  Min: {min_val:.6f}, Max: {max_val:.6f}")
            
            # Ask user if they want to continue
            while True:
                continue_input = input("Continue anyway? [y/N]: ").strip().lower()
                if continue_input in ['', 'n', 'no']:
                    print("Aborting due to value range")
                    sys.exit(1)
                elif continue_input in ['y', 'yes']:
                    break
                print("Please enter 'y' for yes or 'n' for no")
        
        # Pad with zeros if needed to make 8 channels
        if channels < 8:
            print(f"Padding from {channels} to 8 channels with zeros")
            padding = np.zeros((samples, 8 - channels))
            data = np.concatenate([data, padding], axis=1)
        
        return data
        
    except Exception as e:
        print(f"Error loading numpy array: {e}")
        sys.exit(1)

def current_to_dac_value(current_amps):
    """
    Convert current value in amps to signed 16-bit DAC value.
    
    The current range -5.1A to +5.1A maps to the full DAC range.
    
    Args:
        current_amps: Current value in amps (-5.1 to +5.1)
    
    Returns:
        Signed 16-bit integer DAC value (-32767 to +32767)
    """
    # Scale current (-5.1A to +5.1A) to DAC range (-32767 to +32767)
    dac_value = int(current_amps * 32767 / 5.1)
    
    # Clamp to valid 16-bit signed range
    return max(-32767, min(32767, dac_value))

def calculate_sample_delay(sample_rate_ksps, spi_clock_freq_mhz):
    """
    Calculate the delay value for the given sample rate.
    
    The delay value represents the number of SPI clock cycles between samples.
    
    Args:
        sample_rate_ksps: Sample rate in ksps (kilosamples per second)
        spi_clock_freq_mhz: SPI clock frequency in MHz
    
    Returns:
        Delay value as integer (number of clock cycles)
    """
    # Convert frequencies to Hz
    sample_rate_hz = sample_rate_ksps * 1000  # ksps to Hz
    spi_clock_freq_hz = spi_clock_freq_mhz * 1000000  # MHz to Hz
    
    # Calculate clock cycles per sample
    cycles_per_sample = spi_clock_freq_hz / sample_rate_hz
    
    # Convert to integer delay value
    delay = int(cycles_per_sample)
    
    # Ensure delay is within valid range (1 to 33554431)
    return max(1, min(33554431, delay))

def convert_numpy_to_samples(data, zero_at_end=False):
    """
    Convert numpy array to list of channel value tuples.
    
    Args:
        data: Numpy array with shape [samples, 8] and values in amps
        zero_at_end: If True, set the final sample to zero on all channels
        
    Returns:
        List of tuples, each containing 8 channel DAC values
    """
    samples = []
    
    for i in range(data.shape[0]):
        channel_values = []
        for j in range(8):
            # If this is the last sample and zero_at_end is True, use 0 current
            if zero_at_end and i == data.shape[0] - 1:
                current_amps = 0.0
            else:
                current_amps = data[i, j]
            
            dac_value = current_to_dac_value(current_amps)
            channel_values.append(dac_value)
        
        samples.append(channel_values)
    
    return samples

def create_zeroed_samples(sample_count):
    """
    Create zeroed samples with the same count as the original waveform.
    
    Args:
        sample_count: Number of samples to create
        
    Returns:
        List of tuples, each containing 8 zero channel values
    """
    samples = []
    
    for i in range(sample_count):
        # All channels set to 0 (zero current)
        channel_values = [0, 0, 0, 0, 0, 0, 0, 0]
        samples.append(channel_values)
    
    return samples

def write_waveform_file(filename, samples, sample_rate_ksps, spi_clock_freq_mhz, numpy_filename, is_zeroed=False):
    """
    Write samples to a DAC waveform file.
    
    Args:
        filename: Output filename
        samples: List of channel value tuples  
        sample_rate_ksps: Sample rate in ksps
        spi_clock_freq_mhz: SPI clock frequency in MHz
        numpy_filename: Original numpy file name for reference
        is_zeroed: Whether this is a zeroed equivalent waveform
    """
    delay_value = calculate_sample_delay(sample_rate_ksps, spi_clock_freq_mhz)
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            waveform_type = "Zeroed DAC Waveform" if is_zeroed else "DAC Waveform"
            f.write(f"# {waveform_type} File from Numpy Array\n")
            f.write(f"# Generated by waveform_from_npy.py\n")
            f.write(f"# Source file: {numpy_filename}\n")
            if is_zeroed:
                f.write("# Note: This is an equivalent zeroed waveform (all channels = 0)\n")
            f.write(f"# Sample rate: {sample_rate_ksps:.6g} ksps\n")
            f.write(f"# SPI clock frequency: {spi_clock_freq_mhz:.6g} MHz\n")
            f.write(f"# Number of samples: {len(samples)}\n")
            f.write(f"# Delay value: {delay_value} cycles\n")
            if not is_zeroed:
                f.write("# Value range: -5.1A to +5.1A maps to -32767 to +32767 DAC\n")
            else:
                f.write("# All values: 0 (zero current)\n")
            f.write("# Format: T 1 <ch0-ch7> (first command - trigger) / D <delay> <ch0-ch7> (subsequent commands - delay)\n")
            f.write("# Note: First command waits for trigger, subsequent commands use delay timing\n")
            f.write("#\n")
            
            # Write samples
            for i, channel_values in enumerate(samples):
                # Determine if this is the first command (should be trigger type)
                is_first_command = (i == 0)
                
                if is_first_command:
                    # First command: Trigger type waiting for 1 trigger
                    line = f"T 1"
                    f.write("# First command: Trigger type waiting for 1 trigger\n")
                else:
                    # Regular delay command
                    line = f"D {delay_value}"
                
                # Add channel values
                for ch_val in channel_values:
                    line += f" {ch_val}"
                line += "\n"
                f.write(line)
        
        print(f"Waveform file written to: {filename}")
        print(f"Number of samples: {len(samples)}")
        print(f"Sample delay: {delay_value} cycles")
        
    except IOError as e:
        print(f"Error writing file {filename}: {e}")
        sys.exit(1)

def calculate_dac_duration(sample_count, sample_rate_ksps):
    """
    Calculate the total duration of the DAC waveform in milliseconds.
    
    Args:
        sample_count: Number of samples in the waveform
        sample_rate_ksps: Sample rate in ksps
    
    Returns:
        Duration in milliseconds
    """
    # Duration = samples / (sample_rate_ksps * 1000) * 1000 = samples / sample_rate_ksps
    return sample_count / sample_rate_ksps

def write_adc_readout_file(filename, dac_duration_ms, adc_sample_rate_ksps, extra_time_ms, spi_clock_freq_mhz):
    """
    Write an ADC readout command file.
    
    The file format follows the ADC command structure:
    - T <value>: Trigger command (waits for trigger)
    - L <count>: Loop command (repeats next command)
    - D <delay>: Delay command (reads with delay timing)
    - O <ch0> <ch1> ... <ch7>: Order command (sets channel order)
    
    Args:
        filename: Output filename
        dac_duration_ms: Duration of DAC waveform in milliseconds
        adc_sample_rate_ksps: ADC sample rate in ksps
        extra_time_ms: Extra sampling time after DAC completes in milliseconds
        spi_clock_freq_mhz: SPI clock frequency in MHz
    """
    # Calculate total sampling time
    total_sample_time_ms = dac_duration_ms + extra_time_ms
    
    # Calculate delay value for ADC sample rate (convert ksps to kHz for compatibility)
    adc_delay_value = calculate_sample_delay(adc_sample_rate_ksps, spi_clock_freq_mhz)
    
    # Calculate total number of samples needed
    total_samples = int(adc_sample_rate_ksps * total_sample_time_ms)
    if total_samples < 1:
        total_samples = 1
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            f.write("# ADC Readout Command File\n")
            f.write(f"# Generated by waveform_from_npy.py\n")
            f.write(f"# DAC duration: {dac_duration_ms:.6g} ms\n")
            f.write(f"# Extra sample time: {extra_time_ms:.6g} ms\n")
            f.write(f"# Total sample time: {total_sample_time_ms:.6g} ms\n")
            f.write(f"# ADC sample rate: {adc_sample_rate_ksps:.6g} ksps\n")
            f.write(f"# SPI clock frequency: {spi_clock_freq_mhz:.6g} MHz\n")
            f.write(f"# ADC delay value: {adc_delay_value}\n")
            f.write(f"# Total samples: {total_samples}\n")
            f.write("# Format: T <value> (trigger) / L <count> (loop) / D <delay> (delay/read) / O <ch0> <ch1> ... <ch7> (order)\n")
            f.write("#\n")
            
            # Write channel order command (sets ADC channel order to 0 1 2 3 4 5 6 7)
            f.write("# Order command: set ADC channel order to 0 1 2 3 4 5 6 7\n")
            f.write("O 0 1 2 3 4 5 6 7\n")
            
            # Write trigger command (waits for 1 trigger to start)
            f.write("# Trigger command: wait for 1 trigger to start sampling\n")
            f.write("T 1\n")
            
            # Write loop command and delay command
            f.write(f"# Loop command: repeat next command {total_samples} times\n")
            f.write(f"L {total_samples}\n")
            f.write(f"# Delay command: read ADC with {adc_delay_value} cycle delay\n")
            f.write(f"D {adc_delay_value}\n")
        
        print(f"ADC readout file written to: {filename}")
        print(f"Total samples: {total_samples}")
        print(f"ADC delay value: {adc_delay_value}")
        print(f"Total sampling time: {total_sample_time_ms:.6g} ms")
        
    except IOError as e:
        print(f"Error writing ADC readout file {filename}: {e}")
        sys.exit(1)

def main():
    """Main function."""
    # Check for command line arguments
    numpy_file = None
    if len(sys.argv) == 2:
        numpy_file = sys.argv[1]
        # Check for help request
        if numpy_file in ['-h', '--help']:
            print("Usage: waveform_from_npy.py [numpy_file.npy]")
            print()
            print("Generate DAC waveform files from numpy array files.")
            print()
            print("Arguments:")
            print("  numpy_file.npy    Path to numpy array file (optional, will prompt if not provided)")
            print()
            print("The numpy array should have shape [samples, channels] where:")
            print("  - samples: Number of time samples")
            print("  - channels: 1-8 channels (zeros added if < 8)")
            print("  - values: -5.1 to +5.1 representing current in amps")
            print()
            print("Options available during execution:")
            print("  - ADC readout file generation")
            print("  - Equivalent zeroed waveform (same timing, all values = 0)")
            sys.exit(0)
    elif len(sys.argv) > 2:
        print("Usage: waveform_from_npy.py [numpy_file.npy]")
        print("Use -h or --help for more information")
        sys.exit(1)
    
    # Get user input
    params = get_user_input(numpy_file)
    
    # Load and validate numpy array
    print(f"\nLoading numpy array from: {params['numpy_file']}")
    data = load_and_validate_numpy_array(params['numpy_file'])
    
    print(f"Final array shape: {data.shape}")
    
    # Convert to DAC samples
    print("Converting to DAC values...")
    if params['zero_at_end']:
        print("Final sample will be set to zero on all channels")
    samples = convert_numpy_to_samples(data, params['zero_at_end'])
    
    if not samples:
        print("No samples to write")
        sys.exit(1)
    
    # Get output filename
    numpy_basename = os.path.splitext(os.path.basename(params['numpy_file']))[0]
    default_filename = f"{numpy_basename}_{params['sample_rate']:.0f}ksps"
    
    filename = input(f"Output filename (default: {default_filename}.[wfm/rdout]): ").strip()
    if not filename:
        filename = default_filename
    
    # Write waveform file
    wfm_filename = filename if filename.endswith('.wfm') else f"{filename}.wfm"
    write_waveform_file(wfm_filename, samples, params['sample_rate'], 
                       params['spi_clock_freq'], params['numpy_file'])
    
    # Generate equivalent zeroed waveform if requested
    if params.get('create_zero_waveform', False):
        # Create zeroed samples with same count
        zero_samples = create_zeroed_samples(len(samples))
        
        # Write zeroed waveform file
        zero_filename = filename if filename.endswith('_zero.wfm') else f"{filename}_zero.wfm"
        write_waveform_file(zero_filename, zero_samples, params['sample_rate'], 
                           params['spi_clock_freq'], params['numpy_file'], is_zeroed=True)
    
    # Generate ADC readout file if requested
    if params.get('create_adc_readout', False):
        # Calculate DAC duration
        dac_duration_ms = calculate_dac_duration(len(samples), params['sample_rate'])
        
        # Write ADC readout file
        rdout_filename = filename if filename.endswith('.rdout') else f"{filename}.rdout"
        write_adc_readout_file(
            rdout_filename, 
            dac_duration_ms, 
            params['adc_sample_rate'], 
            params['adc_extra_time'], 
            params['spi_clock_freq']
        )
    
    print("Waveform generation complete!")

if __name__ == "__main__":
    main()
