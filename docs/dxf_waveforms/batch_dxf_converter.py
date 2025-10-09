#!/usr/bin/env python3
"""
Batch DXF to NumPy Converter

This script processes all DXF files in a directory and converts them to numpy arrays.

Usage:
    python batch_dxf_converter.py <dxf_directory> [--num_points NUM_POINTS] [--output_dir OUTPUT_DIR]

Example:
    python batch_dxf_converter.py tmp/dxf_lines/ --num_points 1000 --output_dir numpy_arrays/
"""

import argparse
import numpy as np
import os
from pathlib import Path
import sys

# Import our DXF processing functions
from dxf_to_numpy import process_dxf_file


def process_directory(dxf_dir, output_dir=None, force_single_line=False):
    """
    Process all DXF files in a directory and convert to waveforms.
    
    Args:
        dxf_dir (str): Directory containing DXF files
        output_dir (str): Output directory for waveform files (optional)
        force_single_line (bool): Force single line generation
    
    Returns:
        dict: Dictionary mapping DXF filenames to waveform arrays
    """
    dxf_path = Path(dxf_dir)
    
    if not dxf_path.exists() or not dxf_path.is_dir():
        print(f"Error: Directory '{dxf_dir}' not found")
        return {}
    
    # Find all DXF files
    dxf_files = list(dxf_path.glob("*.dxf"))
    
    if not dxf_files:
        print(f"No DXF files found in {dxf_dir}")
        return {}
    
    print(f"Found {len(dxf_files)} DXF files")
    
    # Setup output directory
    if output_dir:
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
    else:
        output_path = dxf_path
    
    results = {}
    
    # Process each DXF file
    for dxf_file in sorted(dxf_files):
        print(f"\nProcessing: {dxf_file.name}")
        
        # Process the DXF file
        waveform_array = process_dxf_file(str(dxf_file), force_single_line)
        
        if waveform_array.size == 0:
            print(f"  Warning: No waveform generated for {dxf_file.name}")
            continue
        
        print(f"  Generated waveform with {len(waveform_array)} samples")
        
        # Save waveform to file
        output_file = output_path / f"{dxf_file.stem}.wfm.npy"
        np.save(output_file, waveform_array)
        print(f"  Saved waveform to: {output_file}")
        
        results[dxf_file.name] = waveform_array
    
    return results


def create_summary_report(results, output_file="conversion_summary.txt"):
    """
    Create a summary report of the conversion process.
    
    Args:
        results (dict): Dictionary of conversion results
        output_file (str): Path to save the summary report
    """
    with open(output_file, 'w') as f:
        f.write("DXF to Waveform Conversion Summary\n")
        f.write("=" * 42 + "\n\n")
        
        for filename, waveform_array in results.items():
            f.write(f"File: {filename}\n")
            f.write(f"  Waveform samples: {len(waveform_array)}\n")
            f.write(f"  Array shape: {waveform_array.shape}\n")
            f.write(f"  Y range: {waveform_array.min():.6f} to {waveform_array.max():.6f}\n")
            f.write("\n")
    
    print(f"Summary report saved to: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Batch convert DXF files to waveform arrays')
    parser.add_argument('dxf_directory', help='Directory containing DXF files')
    parser.add_argument('--output_dir', '-o', 
                       help='Output directory for waveform files (default: same as input)')
    parser.add_argument('--single_line', '-s', action='store_true',
                       help='Force creation of single line waveforms for all files')
    parser.add_argument('--summary', action='store_true',
                       help='Create a summary report of the conversion')
    
    args = parser.parse_args()
    
    print("Batch DXF to Waveform Converter")
    print("=" * 32)
    
    # Process all DXF files in the directory
    results = process_directory(
        args.dxf_directory, 
        args.output_dir,
        args.single_line
    )
    
    if not results:
        print("No files were processed successfully")
        sys.exit(1)
    
    print(f"\nProcessing complete! Converted {len(results)} files")
    
    # Create summary report if requested
    if args.summary:
        summary_file = "conversion_summary.txt"
        if args.output_dir:
            summary_file = str(Path(args.output_dir) / summary_file)
        create_summary_report(results, summary_file)


if __name__ == "__main__":
    main()
