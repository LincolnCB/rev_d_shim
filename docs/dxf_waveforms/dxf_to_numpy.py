#!/usr/bin/env python3
"""
DXF to Waveform Converter

This script converts DXF files into waveform arrays with configurable number of equidistant y samples scaled by 0.34.
Supports both single file processing and batch processing of entire directories.

Usage:
    # Single file mode
    python dxf_to_numpy.py <dxf_file> [--output OUTPUT_FILE] [--num_samples NUM]
    
    # Batch mode
    python dxf_to_numpy.py --batch <directory> [--output OUTPUT_DIR] [--num_samples NUM]

Examples:
    # Process single file with default 1000 samples
    python dxf_to_numpy.py tmp/dxf_lines/line_1.dxf --output line_1.wfm.npy
    
    # Process single file with 500 samples
    python dxf_to_numpy.py tmp/dxf_lines/line_1.dxf --num_samples 500 --output line_1.wfm.npy
    
    # Process all DXF files in directory with 2000 samples each
    python dxf_to_numpy.py --batch tmp/dxf_lines/ --num_samples 2000 --output numpy_arrays/
    
    # Batch processing creates individual files plus a concatenated array
    # The concatenated array has shape [samples, files] for easy plotting
"""

import argparse
import numpy as np
import ezdxf
from pathlib import Path
import sys


def extract_geometry_from_dxf(dxf_file):
    """
    Extract all LINE, ARC, and ELLIPSE entities from a DXF file.
    
    Args:
        dxf_file (str): Path to the DXF file
        
    Returns:
        tuple: (lines, arcs, ellipses) where:
               lines: List of tuples containing (start_point, end_point) for each line
               arcs: List of dictionaries containing arc parameters
               ellipses: List of dictionaries containing ellipse parameters
    """
    try:
        # Load the DXF file
        doc = ezdxf.readfile(dxf_file)
        modelspace = doc.modelspace()
        
        lines = []
        arcs = []
        ellipses = []
        
        # Extract all LINE entities
        for entity in modelspace.query('LINE'):
            start_point = (entity.dxf.start.x, entity.dxf.start.y)
            end_point = (entity.dxf.end.x, entity.dxf.end.y)
            lines.append((start_point, end_point))
            
        # Extract all ARC entities
        for entity in modelspace.query('ARC'):
            center = (entity.dxf.center.x, entity.dxf.center.y)
            radius = entity.dxf.radius
            start_angle = entity.dxf.start_angle * np.pi / 180  # Convert to radians
            end_angle = entity.dxf.end_angle * np.pi / 180      # Convert to radians
            
            arcs.append({
                'center': center,
                'radius': radius,
                'start_angle': start_angle,
                'end_angle': end_angle
            })
            
        # Extract all ELLIPSE entities
        for entity in modelspace.query('ELLIPSE'):
            # Get ellipse parameters
            center = (entity.dxf.center.x, entity.dxf.center.y)
            major_axis = (entity.dxf.major_axis.x, entity.dxf.major_axis.y)
            ratio = entity.dxf.ratio  # ratio of minor axis to major axis
            start_param = entity.dxf.start_param
            end_param = entity.dxf.end_param
            
            ellipses.append({
                'center': center,
                'major_axis': major_axis,
                'ratio': ratio,
                'start_param': start_param,
                'end_param': end_param
            })
            
        return lines, arcs, ellipses
        
    except Exception as e:
        print(f"Error reading DXF file {dxf_file}: {e}")
        return [], [], []


def extract_lines_from_dxf(dxf_file):
    """
    Extract all LINE entities from a DXF file (backwards compatibility).
    
    Args:
        dxf_file (str): Path to the DXF file
        
    Returns:
        list: List of tuples containing (start_point, end_point) for each line
    """
    lines, _, _ = extract_geometry_from_dxf(dxf_file)
    return lines





def get_segment_endpoints(seg_type, seg_data):
    """
    Get the start and end points of a segment.
    
    Args:
        seg_type (str): 'line', 'arc', or 'ellipse'
        seg_data: Segment data
        
    Returns:
        tuple: (start_point, end_point) as (x, y) tuples
    """
    if seg_type == 'line':
        return seg_data[0], seg_data[1]
    elif seg_type == 'arc':
        center = seg_data['center']
        radius = seg_data['radius']
        start_angle = seg_data['start_angle']
        end_angle = seg_data['end_angle']
        is_reversed = seg_data.get('reversed', False)
        
        # Calculate start and end points
        start_pt = (
            center[0] + radius * np.cos(start_angle),
            center[1] + radius * np.sin(start_angle)
        )
        end_pt = (
            center[0] + radius * np.cos(end_angle),
            center[1] + radius * np.sin(end_angle)
        )
        
        # If reversed, swap the logical start and end points
        if is_reversed:
            return end_pt, start_pt
        else:
            return start_pt, end_pt
    else:  # ellipse
        center = seg_data['center']
        major_axis = seg_data['major_axis']
        ratio = seg_data['ratio']
        start_param = seg_data['start_param']
        end_param = seg_data['end_param']
        is_reversed = seg_data.get('reversed', False)
        
        # Calculate major axis length and rotation
        major_length = np.sqrt(major_axis[0]**2 + major_axis[1]**2)
        minor_length = major_length * ratio
        rotation = np.arctan2(major_axis[1], major_axis[0])
        
        # Calculate start and end points
        cos_rot = np.cos(rotation)
        sin_rot = np.sin(rotation)
        
        # Start point
        x_local = major_length * np.cos(start_param)
        y_local = minor_length * np.sin(start_param)
        start_pt = (
            x_local * cos_rot - y_local * sin_rot + center[0],
            x_local * sin_rot + y_local * cos_rot + center[1]
        )
        
        # End point
        x_local = major_length * np.cos(end_param)
        y_local = minor_length * np.sin(end_param)
        end_pt = (
            x_local * cos_rot - y_local * sin_rot + center[0],
            x_local * sin_rot + y_local * cos_rot + center[1]
        )
        
        # If reversed, swap the logical start and end points
        if is_reversed:
            return end_pt, start_pt
        else:
            return start_pt, end_pt


def organize_path_segments(lines, arcs, ellipses, target_start=(-7.413, 0), target_end=(7.413, 0), tolerance=0.05, quiet=False):
    """
    Organize line segments, circular arcs, and elliptical arcs into a continuous path.
    
    Args:
        lines (list): List of line segments as (start_point, end_point) tuples
        arcs (list): List of circular arc parameter dictionaries
        ellipses (list): List of ellipse parameter dictionaries
        target_start (tuple): Target starting point
        target_end (tuple): Target ending point  
        tolerance (float): Distance tolerance for connecting segments
        quiet (bool): If True, suppress gap warnings
        
    Returns:
        list: Ordered list of segments, each being ('line', segment), ('arc', params), or ('ellipse', params)
    """
    def distance(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    def normalize_segment(seg_type, seg_data):
        """Normalize segment orientation to go from left to right when possible"""
        start_pt, end_pt = get_segment_endpoints(seg_type, seg_data)
        
        # If segment goes from right to left (positive x to negative x), reverse it
        if start_pt[0] > end_pt[0]:
            if seg_type == 'line':
                return seg_type, (end_pt, start_pt)
            elif seg_type == 'arc':
                new_arc = seg_data.copy()
                # Don't swap angles, just mark as reversed for rasterization
                new_arc['reversed'] = True
                return seg_type, new_arc
            else:  # ellipse
                new_ellipse = seg_data.copy()
                # Don't swap parameters, just mark as reversed for rasterization
                new_ellipse['reversed'] = True
                return seg_type, new_ellipse
        
        return seg_type, seg_data
    
    # Convert to segment format and normalize orientations
    available_segments = []
    for line in lines:
        seg_type, seg_data = normalize_segment('line', line)
        available_segments.append((seg_type, seg_data))
    for arc in arcs:
        seg_type, seg_data = normalize_segment('arc', arc)
        available_segments.append((seg_type, seg_data))
    for ellipse in ellipses:
        seg_type, seg_data = normalize_segment('ellipse', ellipse)
        available_segments.append((seg_type, seg_data))
    
    if not available_segments:
        return []
    
    # Build connectivity graph - find which segments connect to which
    segment_endpoints = []
    for i, (seg_type, seg_data) in enumerate(available_segments):
        start_pt, end_pt = get_segment_endpoints(seg_type, seg_data)
        segment_endpoints.append((i, start_pt, end_pt))
    
    # Find starting segment (closest start point to target_start)
    best_start_seg = None
    best_start_distance = float('inf')
    best_start_reverse = False
    
    for i, start_pt, end_pt in segment_endpoints:
        # Check normal orientation
        d1 = distance(target_start, start_pt)
        if d1 < best_start_distance:
            best_start_distance = d1
            best_start_seg = i
            best_start_reverse = False
            
        # Check reversed orientation
        d2 = distance(target_start, end_pt)
        if d2 < best_start_distance:
            best_start_distance = d2
            best_start_seg = i
            best_start_reverse = True
    
    if best_start_seg is None:
        return []
    
    # Build the continuous path
    ordered_path = []
    used_segments = set()
    
    # Add the starting segment
    seg_type, seg_data = available_segments[best_start_seg]
    if best_start_reverse:
        if seg_type == 'line':
            start_pt, end_pt = seg_data
            seg_data = (end_pt, start_pt)
        elif seg_type == 'arc':
            new_arc = seg_data.copy()
            new_arc['start_angle'], new_arc['end_angle'] = seg_data['end_angle'], seg_data['start_angle']
            seg_data = new_arc
        else:  # ellipse
            new_ellipse = seg_data.copy()
            new_ellipse['start_param'], new_ellipse['end_param'] = seg_data['end_param'], seg_data['start_param']
            seg_data = new_ellipse
    
    ordered_path.append((seg_type, seg_data))
    used_segments.add(best_start_seg)
    
    # Get the current endpoint
    _, current_endpoint = get_segment_endpoints(seg_type, seg_data)
    
    # Continue adding segments that connect
    while len(used_segments) < len(available_segments):
        next_segment = None
        next_index = -1
        next_reverse = False
        best_connection_distance = float('inf')
        
        for i, (seg_type, seg_data) in enumerate(available_segments):
            if i in used_segments:
                continue
                
            start_pt, end_pt = get_segment_endpoints(seg_type, seg_data)
            
            # Check if this segment's start connects to current endpoint
            d1 = distance(current_endpoint, start_pt)
            if d1 < best_connection_distance:
                best_connection_distance = d1
                next_segment = (seg_type, seg_data)
                next_index = i
                next_reverse = False
            
            # Check if this segment's end connects to current endpoint (reverse needed)
            d2 = distance(current_endpoint, end_pt)
            if d2 < best_connection_distance:
                best_connection_distance = d2
                next_segment = (seg_type, seg_data)
                next_index = i
                next_reverse = True
        
        if next_segment is None:
            break
            
        # Check if connection is good enough
        if best_connection_distance > tolerance:
            if not quiet:
                print(f"Warning: Gap ({best_connection_distance:.3f}) between segments")
        
        # Add the next segment (possibly reversed)
        seg_type, seg_data = next_segment
        if next_reverse:
            if seg_type == 'line':
                start_pt, end_pt = seg_data
                seg_data = (end_pt, start_pt)
            elif seg_type == 'arc':
                # For arcs, we need to reverse the direction while preserving the arc geometry
                new_arc = seg_data.copy()
                # Don't swap angles, just mark as reversed for rasterization
                new_arc['reversed'] = True
                seg_data = new_arc
            else:  # ellipse
                # For ellipses, we need to reverse the direction while preserving the arc geometry
                new_ellipse = seg_data.copy()
                # Don't swap parameters, just mark as reversed for rasterization
                new_ellipse['reversed'] = True
                seg_data = new_ellipse
        
        ordered_path.append((seg_type, seg_data))
        used_segments.add(next_index)
        
        # Update current endpoint
        _, current_endpoint = get_segment_endpoints(seg_type, seg_data)
    
    # Add closing segment to target_end if needed
    final_distance = distance(current_endpoint, target_end)
    if final_distance > tolerance:
        if not quiet:
            print(f"Adding closing segment: gap of {final_distance:.3f} to target end")
        ordered_path.append(('line', (current_endpoint, target_end)))
    
    return ordered_path


def organize_segments_by_x_range(ordered_segments):
    """
    Organize segments by their x-coordinate ranges for direct evaluation.
    
    Args:
        ordered_segments (list): List of ('line', segment) or ('ellipse', params) tuples
        
    Returns:
        list: List of tuples (x_start, x_end, seg_type, seg_data) sorted by x_start
    """
    x_ranges = []
    
    for seg_type, seg_data in ordered_segments:
        start_pt, end_pt = get_segment_endpoints(seg_type, seg_data)
        x_start, x_end = min(start_pt[0], end_pt[0]), max(start_pt[0], end_pt[0])
        
        # For vertical segments, add a tiny spread to avoid division by zero
        if abs(x_end - x_start) < 1e-10:
            x_mid = (x_start + x_end) / 2
            x_start = x_mid - 1e-6
            x_end = x_mid + 1e-6
        
        x_ranges.append((x_start, x_end, seg_type, seg_data))
    
    # Sort by x_start to enable efficient lookup
    x_ranges.sort(key=lambda x: x[0])
    
    return x_ranges


def evaluate_segment_at_x(x, seg_type, seg_data):
    """
    Evaluate a segment's y-value at a given x-coordinate.
    
    Args:
        x (float): X-coordinate to evaluate at
        seg_type (str): 'line', 'arc', or 'ellipse'
        seg_data: Segment data
        
    Returns:
        float: Y-coordinate at the given x
    """
    if seg_type == 'line':
        start_pt, end_pt = seg_data
        x1, y1 = start_pt
        x2, y2 = end_pt
        
        # Linear interpolation: y = y1 + (y2-y1) * (x-x1)/(x2-x1)
        if abs(x2 - x1) < 1e-10:  # Vertical line
            return (y1 + y2) / 2  # Return midpoint y
        
        t = (x - x1) / (x2 - x1)
        return y1 + t * (y2 - y1)
        
    elif seg_type == 'arc':
        center = seg_data['center']
        radius = seg_data['radius']
        start_angle = seg_data['start_angle']
        end_angle = seg_data['end_angle']
        is_reversed = seg_data.get('reversed', False)
        
        cx, cy = center
        
        # For a circle: (x-cx)^2 + (y-cy)^2 = r^2
        # Solve for y: y = cy ± sqrt(r^2 - (x-cx)^2)
        discriminant = radius**2 - (x - cx)**2
        if discriminant < 0:
            return cy  # Point outside circle, return center y
        
        y_offset = np.sqrt(discriminant)
        y_upper = cy + y_offset
        y_lower = cy - y_offset
        
        # Determine which y value is correct based on the arc angles
        # Calculate the angles corresponding to both y values
        angle_upper = np.arctan2(y_upper - cy, x - cx)
        angle_lower = np.arctan2(y_lower - cy, x - cx)
        
        # Normalize angles to [0, 2π)
        angle_upper = angle_upper % (2 * np.pi)
        angle_lower = angle_lower % (2 * np.pi)
        
        # Normalize start and end angles
        start_norm = start_angle % (2 * np.pi)
        end_norm = end_angle % (2 * np.pi)
        
        if start_norm > end_norm:
            end_norm += 2 * np.pi
        
        # Check which angle is within the arc range
        def angle_in_range(angle, start, end):
            if angle < start:
                angle += 2 * np.pi
            return start <= angle <= end
        
        if angle_in_range(angle_upper, start_norm, end_norm):
            return y_upper
        elif angle_in_range(angle_lower, start_norm, end_norm):
            return y_lower
        else:
            # Fallback: return the y closest to the arc midpoint
            mid_angle = (start_angle + end_angle) / 2
            mid_y = cy + radius * np.sin(mid_angle)
            return y_upper if abs(y_upper - mid_y) < abs(y_lower - mid_y) else y_lower
            
    else:  # ellipse
        center = seg_data['center']
        major_axis = seg_data['major_axis']
        ratio = seg_data['ratio']
        start_param = seg_data['start_param']
        end_param = seg_data['end_param']
        is_reversed = seg_data.get('reversed', False)
        
        cx, cy = center
        major_length = np.sqrt(major_axis[0]**2 + major_axis[1]**2)
        minor_length = major_length * ratio
        rotation = np.arctan2(major_axis[1], major_axis[0])
        
        cos_rot = np.cos(rotation)
        sin_rot = np.sin(rotation)
        
        # For a rotated ellipse, we need to solve for the parameter t where x matches
        # The parametric equations are:
        # x = cx + major_length * cos(t) * cos(rot) - minor_length * sin(t) * sin(rot)
        # y = cy + major_length * cos(t) * sin(rot) + minor_length * sin(t) * cos(rot)
        
        # Use a higher resolution sampling approach with better interpolation
        num_samples = 1000  # Much higher resolution
        t_vals = np.linspace(start_param, end_param, num_samples)
        
        x_vals = []
        y_vals = []
        
        for t in t_vals:
            x_local = major_length * np.cos(t)
            y_local = minor_length * np.sin(t)
            
            x_global = x_local * cos_rot - y_local * sin_rot + cx
            y_global = x_local * sin_rot + y_local * cos_rot + cy
            
            x_vals.append(x_global)
            y_vals.append(y_global)
        
        x_vals = np.array(x_vals)
        y_vals = np.array(y_vals)
        
        # Sort by x values to ensure proper interpolation
        sort_indices = np.argsort(x_vals)
        x_vals_sorted = x_vals[sort_indices]
        y_vals_sorted = y_vals[sort_indices]
        
        # Handle the case where the ellipse might not be monotonic in x
        # Find the closest x values and interpolate between them
        if len(x_vals_sorted) < 2:
            return cy  # Fallback to center y
        
        # Check if x is within the ellipse's x range
        if x <= x_vals_sorted[0]:
            return y_vals_sorted[0]
        elif x >= x_vals_sorted[-1]:
            return y_vals_sorted[-1]
        else:
            # Linear interpolation using sorted values
            return np.interp(x, x_vals_sorted, y_vals_sorted)



def create_single_line_waveform(y=0, num_samples=1000, y_scale=0.34):
    """
    Create a simple horizontal line waveform.
    
    Args:
        y (float): Y coordinate for the line (default: 0)
        num_samples (int): Number of waveform samples (default: 1000)
        y_scale (float): Scale factor for y values (default: 0.34)
        
    Returns:
        numpy.ndarray: Array of shape (num_samples,) with scaled y values
    """
    return np.full(num_samples, y * y_scale)


def process_dxf_file(dxf_file, force_single_line=False, quiet=False, num_samples=1000):
    """
    Process a DXF file and convert it to a waveform array.
    
    Args:
        dxf_file (str): Path to the DXF file
        force_single_line (bool): If True, create a single line waveform
        quiet (bool): If True, suppress gap warnings
        num_samples (int): Number of waveform samples (default: 1000)
        
    Returns:
        numpy.ndarray: Array of waveform samples
    """
    if force_single_line:
        return create_single_line_waveform(num_samples=num_samples)
    
    # Extract geometry from DXF file
    lines, arcs, ellipses = extract_geometry_from_dxf(dxf_file)
    
    if not lines and not arcs and not ellipses:
        if not quiet:
            print(f"No geometry found in {dxf_file}, creating default single line")
        return create_single_line_waveform(num_samples=num_samples)
    
    if not quiet:
        print(f"Found {len(lines)} lines, {len(arcs)} arcs, and {len(ellipses)} ellipses in {dxf_file}")
    
    # Organize geometry into a continuous path
    ordered_segments = organize_path_segments(lines, arcs, ellipses, (-7.413, 0), (7.413, 0), quiet=quiet)
    
    if not ordered_segments:
        if not quiet:
            print(f"Could not organize segments into a path, creating default single line")
        return create_single_line_waveform(num_samples=num_samples)
    
    # Convert segments directly to waveform using mathematical evaluation
    waveform_array = create_waveform_array_direct(ordered_segments, num_samples=num_samples)
    
    return waveform_array


def create_waveform_array_direct(ordered_segments, num_samples=1000, y_scale=0.34, x_start=-7.413, x_end=7.413):
    """
    Convert an ordered list of segments to a waveform array by direct evaluation at sample points.
    
    Args:
        ordered_segments (list): List of ('line', segment) or ('arc', params) or ('ellipse', params) tuples
        num_samples (int): Number of samples for the waveform (default: 1000)
        y_scale (float): Scale factor for y values (default: 0.34)
        x_start (float): Starting x coordinate for sampling (default: -7.413)
        x_end (float): Ending x coordinate for sampling (default: 7.413)
        
    Returns:
        numpy.ndarray: Array of shape (num_samples,) containing scaled y values
    """
    if not ordered_segments:
        return np.zeros(num_samples)
    
    # Organize segments by their x-ranges for efficient lookup
    x_ranges = organize_segments_by_x_range(ordered_segments)
    
    # Create equidistant x coordinates for sampling
    x_sample = np.linspace(x_start, x_end, num_samples)
    y_waveform = np.zeros(num_samples)
    
    # For each sample point, find which segment contains it and evaluate
    for i, x in enumerate(x_sample):
        # Find the segment that contains this x coordinate
        segment_found = False
        
        for x_seg_start, x_seg_end, seg_type, seg_data in x_ranges:
            if x_seg_start <= x <= x_seg_end:
                # Evaluate this segment at x
                y = evaluate_segment_at_x(x, seg_type, seg_data)
                y_waveform[i] = y * y_scale
                segment_found = True
                break
        
        # If no segment contains this x, interpolate between nearest segments
        if not segment_found:
            # Find the closest segments on either side
            left_seg = None
            right_seg = None
            left_x = -float('inf')
            right_x = float('inf')
            
            for x_seg_start, x_seg_end, seg_type, seg_data in x_ranges:
                seg_center = (x_seg_start + x_seg_end) / 2
                if seg_center <= x and seg_center > left_x:
                    left_seg = (seg_center, seg_type, seg_data)
                    left_x = seg_center
                elif seg_center > x and seg_center < right_x:
                    right_seg = (seg_center, seg_type, seg_data)
                    right_x = seg_center
            
            # Interpolate between the segments
            if left_seg and right_seg:
                left_y = evaluate_segment_at_x(left_seg[0], left_seg[1], left_seg[2])
                right_y = evaluate_segment_at_x(right_seg[0], right_seg[1], right_seg[2])
                
                # Linear interpolation
                t = (x - left_x) / (right_x - left_x) if right_x != left_x else 0
                y = left_y + t * (right_y - left_y)
                y_waveform[i] = y * y_scale
            elif left_seg:
                # Extrapolate from left segment
                y = evaluate_segment_at_x(left_seg[0], left_seg[1], left_seg[2])
                y_waveform[i] = y * y_scale
            elif right_seg:
                # Extrapolate from right segment
                y = evaluate_segment_at_x(right_seg[0], right_seg[1], right_seg[2])
                y_waveform[i] = y * y_scale
            else:
                # No segments available, keep as zero
                y_waveform[i] = 0
    
    return y_waveform


def process_directory(dxf_dir, force_single_line=False, quiet=False, num_samples=1000):
    """
    Process all DXF files in a directory and convert to waveforms.
    
    Args:
        dxf_dir (str): Directory containing DXF files
        force_single_line (bool): Force single line generation for all files
        quiet (bool): Suppress processing messages
        num_samples (int): Number of waveform samples (default: 1000)
    
    Returns:
        tuple: (filenames, waveforms) where:
               filenames: List of DXF filenames (without extension)
               waveforms: List of waveform arrays
    """
    dxf_path = Path(dxf_dir)
    
    if not dxf_path.exists() or not dxf_path.is_dir():
        print(f"Error: Directory '{dxf_dir}' not found")
        return [], []
    
    # Find all DXF files
    dxf_files = list(dxf_path.glob("*.dxf"))
    
    if not dxf_files:
        print(f"No DXF files found in {dxf_dir}")
        return [], []
    
    if not quiet:
        print(f"Found {len(dxf_files)} DXF files")
    
    filenames = []
    waveforms = []
    
    # Process each DXF file
    for dxf_file in sorted(dxf_files):
        if not quiet:
            print(f"Processing: {dxf_file.name}")
        
        # Process the DXF file
        waveform_array = process_dxf_file(str(dxf_file), force_single_line, quiet, num_samples)
        
        if waveform_array.size == 0:
            if not quiet:
                print(f"  Warning: No waveform generated for {dxf_file.name}")
            continue
        
        if not quiet:
            print(f"  Generated waveform with {len(waveform_array)} samples")
        
        filenames.append(dxf_file.stem)
        waveforms.append(waveform_array)
    
    return filenames, waveforms


def create_concatenated_array(waveforms):
    """
    Concatenate waveforms into a 2D array for easy plotting.
    
    Args:
        waveforms (list): List of 1D waveform arrays
        
    Returns:
        numpy.ndarray: 2D array with shape [samples, batch_size] where each column
                      represents one waveform. This allows plt.plot(array) to show
                      all waveforms overlaid.
    """
    if not waveforms:
        return np.array([])
    
    # Stack waveforms as columns (samples on rows, files on columns)
    # This way plt.plot(concatenated_array) will plot all waveforms
    return np.column_stack(waveforms)


def main():
    parser = argparse.ArgumentParser(description='Convert DXF files to waveform arrays')
    
    # Create mutually exclusive group for single file vs batch processing
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('dxf_file', nargs='?', help='Path to a single DXF file')
    input_group.add_argument('--batch', '-b', metavar='DIR', 
                           help='Directory containing DXF files to process in batch mode')
    
    parser.add_argument('--output', '-o', help='Output waveform file path (optional)')
    parser.add_argument('--num_samples', '-n', type=int, default=1000,
                       help='Number of waveform samples (default: 1000)')
    parser.add_argument('--single_line', '-s', action='store_true',
                       help='Force creation of single line waveform')
    parser.add_argument('--preview', '-p', action='store_true',
                       help='Show preview of the waveform')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='Suppress gap warnings')
    
    args = parser.parse_args()
    
    # Handle batch processing mode
    if args.batch:
        # Validate batch directory
        if not Path(args.batch).exists():
            print(f"Error: Batch directory '{args.batch}' not found")
            sys.exit(1)
        
        if not args.quiet:
            print(f"Processing DXF files in directory: {args.batch}")
        
        # Process all DXF files in the directory
        filenames, waveforms = process_directory(args.batch, args.single_line, args.quiet, args.num_samples)
        
        if not waveforms:
            print("Error: No waveforms generated from batch processing")
            sys.exit(1)
        
        print(f"\nBatch processing complete! Generated {len(waveforms)} waveforms")
        
        # Create concatenated array
        concatenated_array = create_concatenated_array(waveforms)
        print(f"Concatenated array shape: {concatenated_array.shape} [samples x files]")
        
        # Show preview if requested
        if args.preview:
            print(f"\nProcessed files: {filenames}")
            print(f"Concatenated array Y range: {concatenated_array.min():.6f} to {concatenated_array.max():.6f}")
            print(f"Individual waveform shapes: {[w.shape for w in waveforms]}")
        
        # Save files
        batch_dir = Path(args.batch)
        if args.output:
            output_base = Path(args.output)
        else:
            output_base = batch_dir / "batch_waveforms"
        
        # Save individual waveforms
        output_base.mkdir(exist_ok=True)
        for filename, waveform in zip(filenames, waveforms):
            individual_path = output_base / f"{filename}.wfm.npy"
            np.save(individual_path, waveform)
        
        # Save concatenated array
        concatenated_path = output_base / "concatenated_waveforms.npy"
        np.save(concatenated_path, concatenated_array)
        
        print(f"Saved {len(waveforms)} individual waveforms to: {output_base}/")
        print(f"Saved concatenated array to: {concatenated_path}")
        
    else:
        # Single file processing mode
        if not args.dxf_file:
            print("Error: Either provide a DXF file or use --batch with a directory")
            sys.exit(1)
        
        # Check if DXF file exists
        if not Path(args.dxf_file).exists():
            print(f"Error: DXF file '{args.dxf_file}' not found")
            sys.exit(1)
        
        if not args.quiet:
            print(f"Processing DXF file: {args.dxf_file}")
        
        # Process the DXF file
        waveform_array = process_dxf_file(args.dxf_file, args.single_line, args.quiet, args.num_samples)
        
        if waveform_array.size == 0:
            print("Error: No waveform generated")
            sys.exit(1)
        
        print(f"Generated waveform with {len(waveform_array)} samples")
        print(f"Waveform array shape: {waveform_array.shape}")
        
        # Show preview if requested
        if args.preview:
            print("\nFirst 10 waveform samples:")
            print(waveform_array[:10])
            print("\nLast 10 waveform samples:")
            print(waveform_array[-10:])
            print(f"\nWaveform Y range: {waveform_array.min():.6f} to {waveform_array.max():.6f}")
        
        # Save waveform to file
        if args.output:
            # Use provided output path but ensure .wfm.npy extension
            output_path = Path(args.output)
            if not str(output_path).endswith('.wfm.npy'):
                output_path = output_path.with_suffix('.wfm.npy')
        else:
            # Default waveform filename based on input
            input_path = Path(args.dxf_file)
            output_path = input_path.with_suffix('.wfm.npy')
        
        np.save(output_path, waveform_array)
        print(f"Saved waveform array to: {output_path}")


if __name__ == "__main__":
    main()
