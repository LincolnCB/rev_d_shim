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
    
    # Get clock frequency
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
    
    # Get polarity alternation preference
    while True:
        polarity_input = input("Alternate channel polarity? [Y/n]: ").strip().lower()
        if polarity_input in ['', 'y', 'yes']:
            alternate_polarity = True
            break
        elif polarity_input in ['n', 'no']:
            alternate_polarity = False
            break
        print("Please enter 'y' for yes or 'n' for no")
    
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
    params['alternate_polarity'] = alternate_polarity
    
    # Ask about ADC readout file
    while True:
        adc_input = input("Create ADC readout command file? [y/N]: ").strip().lower()
        if adc_input in ['', 'n', 'no']:
            params['create_adc_readout'] = False
            break
        elif adc_input in ['y', 'yes']:
            params['create_adc_readout'] = True
            adc_params = get_adc_readout_parameters(params)
            params.update(adc_params)
            break
        print("Please enter 'y' for yes or 'n' for no")
    
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
            amplitude = float(input("Amplitude (amps, 0 to 4): "))
            if 0 <= amplitude <= 4:
                break
            print("Amplitude must be between 0 and 4 amps")
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
            amplitude = float(input("Amplitude (amps, 0 to 4): "))
            if 0 <= amplitude <= 4:
                break
            print("Amplitude must be between 0 and 4 amps")
        except ValueError:
            print("Please enter a valid number")
    
    return {
        'sample_rate': sample_rate,
        'rise_time': rise_time,
        'flat_time': flat_time,
        'fall_time': fall_time,
        'amplitude': amplitude
    }

def get_adc_readout_parameters(dac_params):
    """Get parameters for ADC readout file generation."""
    print("\n--- ADC Readout Parameters ---")
    
    # Default ADC sample rate to match DAC sample rate
    default_adc_rate = dac_params['sample_rate']
    
    while True:
        try:
            adc_sample_rate_input = input(f"ADC sample rate (kHz, default {default_adc_rate:.6g}): ").strip()
            if adc_sample_rate_input == '':
                adc_sample_rate = default_adc_rate
            else:
                adc_sample_rate = float(adc_sample_rate_input)
            
            if 0 < adc_sample_rate <= 100:
                break
            print("ADC sample rate must be between 0 and 100 kHz")
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

def current_to_dac_value(current, amplitude):
    """
    Convert sine wave value to signed 16-bit DAC value.
    
    The amplitude parameter (0-4A) scales to the full DAC range.
    The sine wave value (-1 to +1) then uses the full +/- range.
    
    Args:
        current: Sine wave value (-1 to +1)
        amplitude: Maximum amplitude (0 to 4A)
    
    Returns:
        Signed 16-bit integer DAC value
    """
    # Scale amplitude (0-4A) to DAC range (0-32767)
    max_dac_value = int(amplitude * 32767 / 4.0)
    
    # Apply sine wave value (-1 to +1) to get full +/- range
    dac_value = int(current * max_dac_value)
    
    # Clamp to valid 16-bit signed range
    return max(-32767, min(32767, dac_value))

def generate_sine_wave(params):
    """
    Generate sine wave samples for 8 channels.
    
    The sine wave duration is determined by sample_rate * duration,
    where duration is specified in milliseconds and sample_rate in kHz.
    Total samples = (sample_rate_kHz * 1000) * (duration_ms / 1000) = sample_rate_kHz * duration_ms
    
    Args:
        params: Dictionary with waveform parameters including:
                - duration: Duration in milliseconds
                - sample_rate: Sample rate in kHz
                - frequency: Sine wave frequency in kHz
                - amplitude: Amplitude in amps (0-4A)
    
    Returns:
        List of tuples, each containing 8 channel values for one sample
    """
    sample_rate_hz = params['sample_rate'] * 1000  # Convert kHz to Hz
    duration_s = params['duration'] / 1000         # Convert ms to seconds
    frequency_hz = params['frequency'] * 1000      # Convert kHz to Hz
    amplitude_a = params['amplitude']
    
    # Calculate number of samples
    num_samples = int(sample_rate_hz * duration_s)
    if num_samples == 0:
        num_samples = 1
    
    samples = []
    
    for i in range(num_samples):
        # Calculate time for this sample (offset by one sample period so first sample isn't at t=0)
        t = (i + 1) / sample_rate_hz
        
        # Generate sine wave value (-1 to +1)
        sine_value = math.sin(2 * math.pi * frequency_hz * t)
        
        # Convert to DAC value using amplitude
        base_dac_value = current_to_dac_value(sine_value, amplitude_a)
        
        # Generate channel values based on polarity preference
        channel_values = []
        for ch in range(8):
            if params['alternate_polarity'] and ch % 2 == 1:
                # Odd channels get negative polarity when alternating is enabled
                channel_values.append(-base_dac_value)
            else:
                # Even channels or all channels when not alternating
                channel_values.append(base_dac_value)
        
        samples.append(channel_values)
    
    return samples

def generate_trapezoid_wave(params):
    """
    Generate trapezoid wave samples for 8 channels.
    
    The trapezoid consists of:
    1. Rise phase: linear ramp up from initial value to amplitude
    2. Flat phase: constant amplitude with extended delay
    3. Fall phase: linear ramp down from amplitude to 0
    
    Args:
        params: Dictionary with waveform parameters
    
    Returns:
        List of channel value lists for each sample
    """
    sample_rate_hz = params['sample_rate'] * 1000  # Convert kHz to Hz
    sample_period_s = 1.0 / sample_rate_hz
    
    rise_time_s = params['rise_time'] / 1000        # Convert ms to seconds
    flat_time_s = params['flat_time'] / 1000        # Convert ms to seconds
    fall_time_s = params['fall_time'] / 1000        # Convert ms to seconds
    amplitude_a = params['amplitude']
    
    # Calculate number of samples for each phase
    rise_samples = max(1, int(rise_time_s / sample_period_s))
    flat_samples = max(1, int(flat_time_s / sample_period_s))
    fall_samples = max(1, int(fall_time_s / sample_period_s))
    
    samples = []
    
    # For consistent step sizing, calculate max value based on rise samples
    # This ensures nice integer steps like 1000, 2000, 3000, etc.
    # We'll use a reasonable max value that gives clean steps
    if rise_samples <= 5:
        max_dac_value = rise_samples * 1000  # e.g., 5000 for 5 samples
    else:
        # For larger sample counts, use standard current conversion
        max_dac_value = current_to_dac_value(1.0, amplitude_a)
    
    # Phase 1: Rise (linear ramp up to amplitude)
    for i in range(rise_samples):
        # Calculate ramp value: starts from 1/rise_samples, goes to rise_samples/rise_samples = 1.0
        step_value = int((i + 1) * max_dac_value / rise_samples)
        
        # Generate channel values based on polarity preference
        channel_values = []
        for ch in range(8):
            if params['alternate_polarity'] and ch % 2 == 1:
                # Odd channels get negative polarity when alternating is enabled
                channel_values.append(-step_value)
            else:
                # Even channels or all channels when not alternating
                channel_values.append(step_value)
        samples.append(channel_values)
    
    # Phase 2: Flat section with first fall value and extended delay
    # The flat section uses the first fall value and extends the delay
    if fall_samples > 0:
        first_fall_value = int((fall_samples - 1) * max_dac_value / fall_samples)
    else:
        first_fall_value = max_dac_value
    
    flat_channel_values = []
    for ch in range(8):
        if params['alternate_polarity'] and ch % 2 == 1:
            # Odd channels get negative polarity when alternating is enabled
            flat_channel_values.append(-first_fall_value)
        else:
            # Even channels or all channels when not alternating
            flat_channel_values.append(first_fall_value)
    
    # Add flat section as a special marker with sample count info
    samples.append(('FLAT_SECTION', flat_channel_values, flat_samples))
    
    # Phase 3: Fall (linear ramp down from amplitude to 0, skipping first fall value)
    for i in range(1, fall_samples):  # Start from 1 to skip first fall value
        # Calculate ramp value: starts from (fall_samples-2)/fall_samples, goes down to 0
        step_value = int((fall_samples - 1 - i) * max_dac_value / fall_samples)
        
        # Generate channel values based on polarity preference
        channel_values = []
        for ch in range(8):
            if params['alternate_polarity'] and ch % 2 == 1:
                # Odd channels get negative polarity when alternating is enabled
                channel_values.append(-step_value)
            else:
                # Even channels or all channels when not alternating
                channel_values.append(step_value)
        samples.append(channel_values)
    
    # Add one extra zero sample at the end
    zero_channel_values = []
    for ch in range(8):
        zero_channel_values.append(0)
    samples.append(zero_channel_values)
    
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

def calculate_dac_duration(params):
    """
    Calculate the total duration of the DAC waveform in milliseconds.
    
    Args:
        params: Dictionary with waveform parameters
    
    Returns:
        Duration in milliseconds
    """
    if params['type'] == 'sine':
        return params['duration']
    elif params['type'] == 'trapezoid':
        return params['rise_time'] + params['flat_time'] + params['fall_time']
    else:
        return 0

def write_adc_readout_file(filename, dac_duration_ms, adc_sample_rate_khz, extra_time_ms, clock_freq_mhz):
    """
    Write an ADC readout command file.
    
    The file format follows the ADC command structure:
    - T <value>: Trigger command (waits for trigger)
    - L <count>: Loop command (repeats next command)
    - D <delay>: Delay command (reads with delay timing)
    
    Args:
        filename: Output filename
        dac_duration_ms: Duration of DAC waveform in milliseconds
        adc_sample_rate_khz: ADC sample rate in kHz
        extra_time_ms: Extra sampling time after DAC completes in milliseconds
        clock_freq_mhz: System clock frequency in MHz
    """
    # Calculate total sampling time
    total_sample_time_ms = dac_duration_ms + extra_time_ms
    
    # Calculate delay value for ADC sample rate
    adc_delay_value = calculate_sample_delay(adc_sample_rate_khz, clock_freq_mhz)
    
    # Calculate total number of samples needed
    total_samples = int(adc_sample_rate_khz * total_sample_time_ms)
    if total_samples < 1:
        total_samples = 1
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            f.write("# ADC Readout Command File\n")
            f.write(f"# Generated by wavegen.py\n")
            f.write(f"# DAC duration: {dac_duration_ms:.6g} ms\n")
            f.write(f"# Extra sample time: {extra_time_ms:.6g} ms\n")
            f.write(f"# Total sample time: {total_sample_time_ms:.6g} ms\n")
            f.write(f"# ADC sample rate: {adc_sample_rate_khz:.6g} kHz\n")
            f.write(f"# Clock frequency: {clock_freq_mhz:.6g} MHz\n")
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

def write_waveform_file(filename, samples, sample_rate_khz, clock_freq_mhz, waveform_type, params=None):
    """
    Write samples to a DAC waveform file.
    
    Args:
        filename: Output filename
        samples: List of channel value tuples or special flat section entries
        sample_rate_khz: Sample rate in kHz
        clock_freq_mhz: System clock frequency in MHz
        waveform_type: Type of waveform (for comments)
        params: Optional waveform parameters for additional comment information
    """
    delay_value = calculate_sample_delay(sample_rate_khz, clock_freq_mhz)
    
    try:
        with open(filename, 'w') as f:
            # Write header comment
            f.write("# DAC Waveform File\n")
            f.write(f"# Generated by wavegen.py\n")
            f.write(f"# Waveform type: {waveform_type}\n")
            
            # Round sample rate and clock frequency to nearest millionth
            sample_rate_str = f"{sample_rate_khz:.6g}" if isinstance(sample_rate_khz, (int, float)) else str(sample_rate_khz)
            clock_freq_str = f"{clock_freq_mhz:.6g}" if isinstance(clock_freq_mhz, (int, float)) else str(clock_freq_mhz)
            
            f.write(f"# Sample rate: {sample_rate_str} kHz\n")
            f.write(f"# Clock frequency: {clock_freq_str} MHz\n")
            
            # Add duration information for sine waves
            if params and waveform_type == 'sine':
                duration_ms = params.get('duration', 'unknown')
                frequency_khz = params.get('frequency', 'unknown')
                amplitude_a = params.get('amplitude', 'unknown')
                
                # Round values to nearest millionth (6 decimal places)
                duration_str = f"{duration_ms:.6g}" if isinstance(duration_ms, (int, float)) else str(duration_ms)
                frequency_str = f"{frequency_khz:.6g}" if isinstance(frequency_khz, (int, float)) else str(frequency_khz)
                amplitude_str = f"{amplitude_a:.6g}" if isinstance(amplitude_a, (int, float)) else str(amplitude_a)
                
                f.write(f"# Sine wave duration: {duration_str} ms\n")
                f.write(f"# Sine wave frequency: {frequency_str} kHz\n")
                f.write(f"# Sine wave amplitude: {amplitude_str} A\n")
                if isinstance(duration_ms, (int, float)) and isinstance(sample_rate_khz, (int, float)):
                    total_samples = int(sample_rate_khz * duration_ms)
                    f.write(f"# Expected total samples: {total_samples}\n")
            
            # Add duration information for trapezoid waves
            elif params and waveform_type == 'trapezoid':
                rise_time = params.get('rise_time', 'unknown')
                flat_time = params.get('flat_time', 'unknown') 
                fall_time = params.get('fall_time', 'unknown')
                amplitude_a = params.get('amplitude', 'unknown')
                total_duration = 'unknown'
                
                # Round values to nearest millionth (6 decimal places)
                rise_str = f"{rise_time:.6g}" if isinstance(rise_time, (int, float)) else str(rise_time)
                flat_str = f"{flat_time:.6g}" if isinstance(flat_time, (int, float)) else str(flat_time)
                fall_str = f"{fall_time:.6g}" if isinstance(fall_time, (int, float)) else str(fall_time)
                amplitude_str = f"{amplitude_a:.6g}" if isinstance(amplitude_a, (int, float)) else str(amplitude_a)
                
                if all(isinstance(t, (int, float)) for t in [rise_time, flat_time, fall_time]):
                    total_duration = rise_time + flat_time + fall_time
                    total_str = f"{total_duration:.6g}"
                else:
                    total_str = str(total_duration)
                
                f.write(f"# Trapezoid rise time: {rise_str} ms\n")
                f.write(f"# Trapezoid flat time: {flat_str} ms\n")
                f.write(f"# Trapezoid fall time: {fall_str} ms\n")
                f.write(f"# Trapezoid total duration: {total_str} ms\n")
                f.write(f"# Trapezoid amplitude: {amplitude_str} A\n")
            
            f.write(f"# Normal delay value: {delay_value}\n")
            f.write("# Format: T 1 <ch0-ch7> (update to values after trigger) / D <delay> <ch0-ch7> (update to values after delay)\n")
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
                    _, channel_values, flat_sample_count = sample_data
                    
                    # Calculate the extended delay for the flat section
                    # This is the delay to hold the flat value, plus normal delay to next sample
                    flat_delay = delay_value * flat_sample_count + delay_value
                    flat_delay = max(1, min(33554431, flat_delay))  # Clamp to valid range
                    
                    f.write(f"# Flat section: {flat_sample_count} samples + 1 transition = {flat_delay} cycles total\n")
                    
                    if is_first_command:
                        # First command: Trigger type waiting for 1 trigger
                        line = f"T 1"
                        f.write("# First command: Trigger type waiting for 1 trigger\n")
                    else:
                        # Regular delay command with extended delay
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
        default_filename = f"sine_wave_{params['frequency']}khz_{params['amplitude']}a"
    elif params['type'] == 'trapezoid':
        default_filename = f"trap_wave_{params['rise_time']}ms_rise_{params['flat_time']}ms_flat_{params['fall_time']}ms_fall_{params['amplitude']}a"
    else:
        default_filename = "waveform"
    
    filename = input(f"Output filename (default: {default_filename}.[wfm/rdout]): ").strip()
    if not filename:
        filename = default_filename
    
    # Write waveform file
    write_waveform_file(f"{filename}.wfm", samples, params['sample_rate'], params['clock_freq'], params['type'], params)
    
    # Generate ADC readout file if requested
    if params.get('create_adc_readout', False):
        # Calculate DAC duration
        dac_duration_ms = calculate_dac_duration(params)
        
        # Write ADC readout file
        write_adc_readout_file(
            f"{filename}.rdout", 
            dac_duration_ms, 
            params['adc_sample_rate'], 
            params['adc_extra_time'], 
            params['clock_freq']
        )
    
    print("Waveform generation complete!")

if __name__ == "__main__":
    main()
