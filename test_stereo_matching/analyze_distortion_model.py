#!/usr/bin/env python3
"""
Analyze calibration distortion model and flags
"""

import cv2
import numpy as np

def analyze_calibration():
    print('=' * 80)
    print('CALIBRATION DISTORTION MODEL ANALYSIS')
    print('=' * 80)

    fs = cv2.FileStorage('/unlook_calib/default.yaml', cv2.FILE_STORAGE_READ)

    distCoeffsLeft = fs.getNode('distortion_coeffs_left').mat()
    distCoeffsRight = fs.getNode('distortion_coeffs_right').mat()

    print('\nDistortion Coefficients (Left):')
    print(f'  Shape: {distCoeffsLeft.shape}')
    print(f'  Values: {distCoeffsLeft.ravel()}')

    print('\nDistortion Coefficients (Right):')
    print(f'  Shape: {distCoeffsRight.shape}')
    print(f'  Values: {distCoeffsRight.ravel()}')

    # Analyze which coefficients are non-zero
    print('\n' + '=' * 80)
    print('ACTIVE DISTORTION COEFFICIENTS')
    print('=' * 80)

    coeff_names = ['k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6',
                   's1', 's2', 's3', 's4', 'tx', 'ty']

    print('\nLeft camera:')
    for i, (name, val) in enumerate(zip(coeff_names, distCoeffsLeft.ravel())):
        status = '✓ ACTIVE' if abs(val) > 1e-10 else '✗ ZERO'
        print(f'  {name:4s} = {val:12.6f}  {status}')

    print('\nRight camera:')
    for i, (name, val) in enumerate(zip(coeff_names, distCoeffsRight.ravel())):
        status = '✓ ACTIVE' if abs(val) > 1e-10 else '✗ ZERO'
        print(f'  {name:4s} = {val:12.6f}  {status}')

    # Count active coefficients
    active_left = np.sum(np.abs(distCoeffsLeft.ravel()) > 1e-10)
    active_right = np.sum(np.abs(distCoeffsRight.ravel()) > 1e-10)

    print('\n' + '=' * 80)
    print('SUMMARY')
    print('=' * 80)
    print(f'\nActive coefficients:')
    print(f'  Left:  {active_left}/14')
    print(f'  Right: {active_right}/14')

    print('\nCalibration flags used (from C++ code):')
    print('  ✓ CALIB_RATIONAL_MODEL (enables k4, k5, k6)')
    print('  ✓ CALIB_FIX_K3 (sets k3 = 0)')
    print('  ✓ CALIB_FIX_K4 (sets k4 = 0)')
    print('  ✓ CALIB_FIX_K5 (sets k5 = 0)')
    print('  ✓ CALIB_ZERO_TANGENT_DIST (sets p1, p2 = 0)')
    print('  ✓ CALIB_FIX_ASPECT_RATIO')
    print('  ✓ CALIB_SAME_FOCAL_LENGTH')

    print('\nEFFECTIVE DISTORTION MODEL:')
    print('  - Radial: k1, k2, k6 (3 coefficients)')
    print('  - Tangential: NONE (p1, p2 = 0)')
    print('  - Thin prism: NONE (s1, s2, s3, s4 = 0)')
    print('  - Tilted sensor: NONE (tx, ty = 0)')

    print('\nPROBLEM ANALYSIS:')
    print('  ⚠ Only using k1, k2, k6 for radial distortion')
    print('  ⚠ Tangential distortion (p1, p2) is DISABLED')
    print('  ⚠ This may be INSUFFICIENT for wide-angle 6mm lenses')
    print('')
    print('RECOMMENDATION:')
    print('  Try recalibrating WITHOUT these flags:')
    print('    - Remove CALIB_ZERO_TANGENT_DIST')
    print('    - Remove CALIB_FIX_K3 (or at least test)')
    print('  This will enable p1, p2, k3 for better lens modeling')

    fs.release()

if __name__ == '__main__':
    analyze_calibration()
