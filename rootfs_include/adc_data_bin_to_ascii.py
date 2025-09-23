#!/usr/bin/env python3
"""
Convert ADC data files from old binary format to new ASCII format.

Old format: Binary file with 32-bit words (little-endian)
New format: ASCII text file with 8 samples per line, space-separated

Each 32-bit word contains two 16-bit samples:
- Bits 15:0 are sample 1
- Bits 31:16 are sample 2
- Samples are in offset format and need to be converted to signed

The offset_to_signed conversion is:
- If offset value == 0xFFFF, return 0
- Otherwise, return (int16_t)(offset_value - 32767)
"""

import os
import sys
import struct
import argparse
from pathlib import Path

def adc_offset_to_signed(offset_val):
    """Convert ADC offset format to signed value."""
    if offset_val == 0xFFFF:
        return 0
    else:
        # Convert to signed 16-bit, clamping the result
        signed_val = offset_val - 32767
        return max(-32767, min(32767, signed_val))

def convert_adc_file(input_file, output_file, verbose=False):
    """Convert a binary ADC data file to ASCII format."""
    
    if verbose:
        print(f"Converting {input_file} -> {output_file}")
    
    try:
        # Read binary data
        with open(input_file, 'rb') as f:
            binary_data = f.read()
        
        # Check if file size is valid (multiple of 4 bytes)
        if len(binary_data) % 4 != 0:
            print(f"Warning: {input_file} size ({len(binary_data)} bytes) is not a multiple of 4")
            # Truncate to nearest multiple of 4
            binary_data = binary_data[:len(binary_data) - (len(binary_data) % 4)]
        
        word_count = len(binary_data) // 4
        if verbose:
            print(f"Processing {word_count} words ({len(binary_data)} bytes)")
        
        # Unpack as little-endian 32-bit unsigned integers
        words = struct.unpack(f'<{word_count}I', binary_data)
        
        # Convert and write ASCII output
        with open(output_file, 'w') as f:
            samples_on_line = 0
            
            for word in words:
                # Extract two 16-bit samples from the 32-bit word
                sample1_offset = word & 0xFFFF         # Bits 15:0
                sample2_offset = (word >> 16) & 0xFFFF # Bits 31:16
                
                # Convert from offset format to signed
                sample1_signed = adc_offset_to_signed(sample1_offset)
                sample2_signed = adc_offset_to_signed(sample2_offset)
                
                # Write sample1
                if samples_on_line > 0:
                    f.write(' ')
                f.write(str(sample1_signed))
                samples_on_line += 1
                
                # Check if we need a new line
                if samples_on_line >= 8:
                    f.write('\n')
                    samples_on_line = 0
                
                # Write sample2
                if samples_on_line > 0:
                    f.write(' ')
                f.write(str(sample2_signed))
                samples_on_line += 1
                
                # Check if we need a new line
                if samples_on_line >= 8:
                    f.write('\n')
                    samples_on_line = 0
            
            # Add final newline if needed
            if samples_on_line > 0:
                f.write('\n')
        
        sample_count = word_count * 2
        if verbose:
            print(f"Converted {sample_count} samples successfully")
        
        return True
        
    except Exception as e:
        print(f"Error converting {input_file}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Convert ADC data files from binary to ASCII format')
    parser.add_argument('input', help='Input binary file or directory')
    parser.add_argument('output', nargs='?', help='Output ASCII file or directory (optional for single file)')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('-r', '--recursive', action='store_true', help='Process directories recursively')
    parser.add_argument('--suffix', default='_ascii', help='Suffix to add to converted files (default: _ascii)')
    
    args = parser.parse_args()
    
    input_path = Path(args.input)
    
    if not input_path.exists():
        print(f"Error: Input path {input_path} does not exist")
        return 1
    
    success_count = 0
    total_count = 0
    
    if input_path.is_file():
        # Single file conversion
        if args.output:
            output_path = Path(args.output)
        else:
            # Generate output filename by adding suffix before extension
            stem = input_path.stem
            suffix = input_path.suffix if input_path.suffix else ''
            output_path = input_path.parent / f"{stem}{args.suffix}{suffix}"
            if suffix == '':
                output_path = input_path.parent / f"{stem}{args.suffix}.txt"
        
        total_count = 1
        if convert_adc_file(input_path, output_path, args.verbose):
            success_count = 1
            print(f"Converted: {input_path} -> {output_path}")
        else:
            print(f"Failed to convert: {input_path}")
    
    elif input_path.is_dir():
        # Directory conversion
        if args.output:
            output_dir = Path(args.output)
            output_dir.mkdir(parents=True, exist_ok=True)
        else:
            output_dir = input_path
        
        # Find all files to convert
        if args.recursive:
            pattern = '**/*'
        else:
            pattern = '*'
        
        files_to_convert = []
        for file_path in input_path.glob(pattern):
            if file_path.is_file():
                files_to_convert.append(file_path)
        
        if not files_to_convert:
            print(f"No files found in {input_path}")
            return 0
        
        print(f"Found {len(files_to_convert)} files to process")
        
        for file_path in files_to_convert:
            # Generate output path
            relative_path = file_path.relative_to(input_path)
            stem = relative_path.stem
            suffix = relative_path.suffix if relative_path.suffix else ''
            
            # Create output filename with suffix
            if suffix:
                output_filename = f"{stem}{args.suffix}{suffix}"
            else:
                output_filename = f"{stem}{args.suffix}.txt"
            
            output_path = output_dir / relative_path.parent / output_filename
            output_path.parent.mkdir(parents=True, exist_ok=True)
            
            total_count += 1
            if convert_adc_file(file_path, output_path, args.verbose):
                success_count += 1
                if not args.verbose:
                    print(f"Converted: {relative_path}")
            else:
                print(f"Failed: {relative_path}")
    
    print(f"\nConversion complete: {success_count}/{total_count} files successful")
    
    return 0 if success_count == total_count else 1

if __name__ == '__main__':
    sys.exit(main())
