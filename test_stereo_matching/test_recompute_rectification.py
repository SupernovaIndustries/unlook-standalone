#!/usr/bin/env python3
"""
Test: RECOMPUTE rectification instead of using precomputed from YAML
This tests if the precomputed rectification parameters are the problem
"""

import numpy as np
import cv2
import sys

def load_calibration_basic(calib_path='/unlook_calib/default.yaml'):
    """Load ONLY intrinsics and extrinsics, NOT precomputed rectification"""
    print(f'Loading calibration from: {calib_path}')

    fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f'ERROR: Failed to open {calib_path}')
        sys.exit(1)

    # Load intrinsics
    cameraMatrixLeft = fs.getNode('camera_matrix_left').mat()
    cameraMatrixRight = fs.getNode('camera_matrix_right').mat()
    distCoeffsLeft = fs.getNode('distortion_coeffs_left').mat()
    distCoeffsRight = fs.getNode('distortion_coeffs_right').mat()

    # Load extrinsics
    R = fs.getNode('rotation_matrix').mat()
    T = fs.getNode('translation_vector').mat()

    # Load metadata
    width = int(fs.getNode('image_width').real())
    height = int(fs.getNode('image_height').real())
    imageSize = (width, height)
    baseline_mm = fs.getNode('baseline_mm').real()

    fs.release()

    print(f'  Image size: {width}x{height}')
    print(f'  Baseline: {baseline_mm:.2f} mm')
    print(f'  Loaded intrinsics and extrinsics (NOT precomputed rectification)')

    return {
        'cameraMatrixLeft': cameraMatrixLeft,
        'cameraMatrixRight': cameraMatrixRight,
        'distCoeffsLeft': distCoeffsLeft,
        'distCoeffsRight': distCoeffsRight,
        'R': R,
        'T': T,
        'imageSize': imageSize,
        'baseline_mm': baseline_mm
    }

def main():
    # Paths
    scan_dir = '/home/alessandro/unlook_debug/scan_20251115_033519'
    left_path = f'{scan_dir}/00_raw_frame00_left.png'
    right_path = f'{scan_dir}/00_raw_frame00_right.png'
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('TEST: RECOMPUTE RECTIFICATION (alpha=1)')
    print('=' * 80)

    # Load calibration (WITHOUT precomputed rectification)
    calib = load_calibration_basic()

    # RECOMPUTE stereoRectify with alpha=1
    print('\nRecomputing stereoRectify with alpha=1...')
    R1, R2, P1, P2, Q, validRoi1, validRoi2 = cv2.stereoRectify(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        calib['imageSize'],
        calib['R'], calib['T'],
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=1,
        newImageSize=calib['imageSize']
    )

    print('  ✓ Recomputed R1, R2, P1, P2, Q')
    print(f'  Q[3,2] = {Q[3,2]:.6f} → baseline = {-1.0/Q[3,2]:.2f} mm')

    # Create rectification maps
    print('\nCreating rectification maps...')
    map1L, map2L = cv2.initUndistortRectifyMap(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        R1, P1,
        calib['imageSize'], cv2.CV_16SC2
    )

    map1R, map2R = cv2.initUndistortRectifyMap(
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        R2, P2,
        calib['imageSize'], cv2.CV_16SC2
    )

    # Load images
    print(f'\nLoading images...')
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print('ERROR: Failed to load images')
        sys.exit(1)

    # Apply rectification
    print('Applying RECOMPUTED rectification...')
    rectL = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

    # Save rectified images
    cv2.imwrite(f'{output_dir}/rectified_left_recomputed.png', rectL)
    cv2.imwrite(f'{output_dir}/rectified_right_recomputed.png', rectR)
    print(f'  Saved: {output_dir}/rectified_left_recomputed.png')
    print(f'  Saved: {output_dir}/rectified_right_recomputed.png')

    # Check alignment
    print('\nChecking epipolar alignment...')
    orb = cv2.ORB_create(nfeatures=500)
    kp1, desc1 = orb.detectAndCompute(rectL, None)
    kp2, desc2 = orb.detectAndCompute(rectR, None)

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)
    good_matches = matches[:min(100, len(matches))]

    y_errors = []
    for m in good_matches:
        pt1 = kp1[m.queryIdx].pt
        pt2 = kp2[m.trainIdx].pt
        y_diff = abs(pt1[1] - pt2[1])
        y_errors.append(y_diff)

    if len(y_errors) > 0:
        y_errors = np.array(y_errors)
        print(f'  Matches: {len(good_matches)}')
        print(f'  Mean Y error: {y_errors.mean():.2f} px')
        print(f'  Median Y error: {np.median(y_errors):.2f} px')
        print(f'  Max Y error: {y_errors.max():.2f} px')

        if y_errors.mean() < 2.0:
            print(f'\n  ✓ EXCELLENT: Recomputed rectification works!')
        elif y_errors.mean() < 5.0:
            print(f'\n  ✓ GOOD: Improvement with recomputed rectification')
        else:
            print(f'\n  ✗ SAME PROBLEM: Recomputing didn\'t help')
            print(f'    → Problem is in calibration data itself!')

    print('\n' + '=' * 80)
    print('TEST COMPLETED')
    print('=' * 80)

if __name__ == '__main__':
    main()
