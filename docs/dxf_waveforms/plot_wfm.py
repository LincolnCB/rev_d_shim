#!/usr/bin/env python3
"""
Waveform Visualizer

This script visualizes waveform arrays generated from DXF files.

Usage:
    python plot_wfm.py <waveform_file> [--output OUTPUT_FILE]
    python plot_wfm.py --batch <directory_with_wfm_files>

Example:
    python plot_wfm.py line_1.wfm.npy
    python plot_wfm.py --batch ./
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def plot_waveform(wfm_file, output_file=None, ax=None, title=None, color='blue'):
    """
    Plot a waveform from a 1D numpy array with sample indices as x-axis.
    
    Args:
        wfm_file (str): Path to the waveform numpy file
        output_file (str): Output image file (optional)
        ax: Matplotlib axis to plot on (optional)
        title (str): Plot title (optional)
        color (str): Line color
        
    Returns:
        matplotlib axis object
    """
    try:
        waveform = np.load(wfm_file)
    except Exception as e:
        print(f"Error loading {wfm_file}: {e}")
        return None
    
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        standalone = True
    else:
        standalone = False
    
    # Use sample indices as x coordinates
    x_coords = np.arange(len(waveform))
    
    # Plot the waveform
    ax.plot(x_coords, waveform, color=color, linewidth=1.5, alpha=0.8)
    
    # Set labels and formatting
    if title:
        ax.set_title(title)
    else:
        ax.set_title(f'Waveform: {Path(wfm_file).stem}')
    
    ax.set_xlabel('Sample Index')
    ax.set_ylabel('Y (scaled)')
    ax.set_ylim(-5.1, 5.1)  # Fixed y-axis range
    ax.grid(True, alpha=0.3)
    
    if standalone:
        plt.tight_layout()
        if output_file:
            plt.savefig(output_file, dpi=300, bbox_inches='tight')
            print(f"Waveform plot saved as {output_file}")
        else:
            plt.savefig(f'{Path(wfm_file).stem}_plot.png', dpi=300, bbox_inches='tight')
            print(f"Waveform plot saved as {Path(wfm_file).stem}_plot.png")
        
        plt.show()
    
    return ax



def plot_batch_waveforms(directory, output_file=None):
    """
    Plot all waveform arrays in a directory.
    
    Args:
        directory (str): Directory containing .wfm.npy files
        output_file (str): Output image file (optional)
    """
    # Look for waveform files
    wfm_files = list(Path(directory).glob("*.wfm.npy"))
    
    if not wfm_files:
        print(f"No *.wfm.npy files found in {directory}")
        return
    
    wfm_files = sorted(wfm_files)
    n_files = len(wfm_files)
    
    # Calculate subplot layout (add 1 for superimposed plot)
    cols = min(3, n_files)
    rows = ((n_files + 1) + cols - 1) // cols  # +1 for superimposed plot
    
    fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 4*rows))
    if (n_files + 1) == 1:
        axes = [axes]
    elif rows == 1:
        axes = [axes] if cols == 1 else axes
    else:
        axes = axes.flatten()
    
    colors = plt.cm.tab10(np.linspace(0, 1, n_files))
    
    # Plot individual waveforms
    for i, wfm_file in enumerate(wfm_files):
        ax = axes[i]
        title = wfm_file.stem.replace('.wfm', '')
        plot_waveform(str(wfm_file), ax=ax, title=title, color=colors[i])
    
    # Create superimposed plot in the next available subplot
    superimposed_ax = axes[n_files]
    superimposed_ax.set_title('All Waveforms Superimposed')
    superimposed_ax.set_xlabel('Sample Index')
    superimposed_ax.set_ylabel('Y (scaled)')
    superimposed_ax.set_ylim(-5.1, 5.1)
    superimposed_ax.grid(True, alpha=0.3)
    
    # Plot all waveforms on the superimposed subplot
    for i, wfm_file in enumerate(wfm_files):
        try:
            waveform = np.load(str(wfm_file))
            x_coords = np.arange(len(waveform))
            label = wfm_file.stem.replace('.wfm', '')
            superimposed_ax.plot(x_coords, waveform, color=colors[i], 
                               linewidth=1.5, alpha=0.7, label=label)
        except Exception as e:
            print(f"Error loading {wfm_file} for superimposed plot: {e}")
    
    # Add legend to superimposed plot
    superimposed_ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    # Hide unused subplots
    for i in range(n_files + 1, len(axes)):
        axes[i].set_visible(False)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Batch plot saved as {output_file}")
    else:
        plt.savefig(f'{Path(directory).name}_batch_plot.png', dpi=300, bbox_inches='tight')
        print(f"Batch plot saved as {Path(directory).name}_batch_plot.png")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize waveforms from .wfm.npy files')
    parser.add_argument('input', nargs='?', help='Waveform file or directory')
    parser.add_argument('--batch', help='Directory containing .wfm.npy files')
    parser.add_argument('--output', '-o', help='Output image file')
    
    args = parser.parse_args()
    
    if args.batch:
        plot_batch_waveforms(args.batch, args.output)
    elif args.input:
        input_path = Path(args.input)
        if input_path.is_file():
            plot_waveform(str(input_path), args.output)
        elif input_path.is_dir():
            plot_batch_waveforms(str(input_path), args.output)
        else:
            print(f"Error: {args.input} not found")
            sys.exit(1)
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
