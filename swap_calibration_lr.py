#!/usr/bin/env python3
"""
Swap LEFT↔RIGHT in stereo calibration YAML file

REASON:
Dataset capture (2025-11-10): Camera 0 SLAVE → /left/, Camera 1 MASTER → /right/
Runtime (current): Camera 1 MASTER → LEFT, Camera 0 SLAVE → RIGHT

This script swaps the calibration parameters so they match the runtime camera assignment.
"""

import yaml
import sys
import os
from datetime import datetime

def swap_calibration_lr(input_yaml, output_yaml=None):
    """Swap left↔right camera parameters in calibration file"""

    print(f"Loading calibration from: {input_yaml}")

    with open(input_yaml, 'r') as f:
        calib = yaml.safe_load(f)

    print(f"Original calibration:")
    print(f"  Image size: {calib['image_width']}x{calib['image_height']}")
    print(f"  Baseline: {calib['baseline_mm']:.3f} mm")
    print(f"  Date: {calib['calibration_date']}")

    # Swap camera matrices
    temp = calib['camera_matrix_left']
    calib['camera_matrix_left'] = calib['camera_matrix_right']
    calib['camera_matrix_right'] = temp

    # Swap distortion coefficients
    temp = calib['distortion_coefficients_left']
    calib['distortion_coefficients_left'] = calib['distortion_coefficients_right']
    calib['distortion_coefficients_right'] = temp

    # Swap rectification transforms
    temp = calib['rectification_transform_left']
    calib['rectification_transform_left'] = calib['rectification_transform_right']
    calib['rectification_transform_right'] = temp

    # Swap projection matrices
    temp = calib['projection_matrix_left']
    calib['projection_matrix_left'] = calib['projection_matrix_right']
    calib['projection_matrix_right'] = temp

    # Invert rotation matrix R (transpose)
    # R_new = R^T because swapping cameras inverts the transformation
    R = calib['rotation_matrix']
    R_data = R['data']
    # Transpose 3x3 matrix stored row-major
    R_transposed = [
        R_data[0], R_data[3], R_data[6],  # First row
        R_data[1], R_data[4], R_data[7],  # Second row
        R_data[2], R_data[5], R_data[8]   # Third row
    ]
    calib['rotation_matrix']['data'] = R_transposed

    # Invert translation vector T (negate)
    # T_new = -T because swapping cameras inverts the baseline direction
    T = calib['translation_vector']
    T_data = T['data']
    calib['translation_vector']['data'] = [-T_data[0], -T_data[1], -T_data[2]]

    # Update metadata
    calib['calibration_date'] = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    calib['calibration_method'] = "opencv_stereo_calibrate_SWAPPED_LR"

    # Add note about swap
    calib['swap_note'] = "LEFT↔RIGHT swapped to match runtime camera assignment (Cam1→LEFT, Cam0→RIGHT)"

    # Determine output filename
    if output_yaml is None:
        base = os.path.splitext(input_yaml)[0]
        output_yaml = f"{base}_swapped.yaml"

    # Save swapped calibration
    print(f"\nSaving swapped calibration to: {output_yaml}")

    with open(output_yaml, 'w') as f:
        yaml.dump(calib, f, default_flow_style=None, sort_keys=False)

    print(f"✓ Swapped calibration saved successfully!")
    print(f"\nTo test:")
    print(f"  sudo ln -sf {output_yaml} /unlook_calib/default.yaml")
    print(f"  unlook  # Run scan and check epipolar alignment")

    return output_yaml

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: ./swap_calibration_lr.py <input.yaml> [output.yaml]")
        print("Example: ./swap_calibration_lr.py /unlook_calib/default.yaml")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(input_file):
        print(f"Error: File not found: {input_file}")
        sys.exit(1)

    swap_calibration_lr(input_file, output_file)
