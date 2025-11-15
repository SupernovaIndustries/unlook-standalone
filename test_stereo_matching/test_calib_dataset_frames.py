#!/usr/bin/env python3
"""
Test rectification quality on frames from calibration dataset itself
These frames SHOULD have <1px error if calibration is valid
"""

import numpy as np
import cv2
import sys

def load_calibration(calib_path='/unlook_calib/default.yaml'):
    """Load calibration parameters"""
    fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f'ERROR: Failed to open {calib_path}')
        sys.exit(1)

    cameraMatrixLeft = fs.getNode('camera_matrix_left').mat()
    cameraMatrixRight = fs.getNode('camera_matrix_right').mat()
    distCoeffsLeft = fs.getNode('distortion_coeffs_left').mat()
    distCoeffsRight = fs.getNode('distortion_coeffs_right').mat()
    R = fs.getNode('rotation_matrix').mat()
    T = fs.getNode('translation_vector').mat()

    width = int(fs.getNode('image_width').real())
    height = int(fs.getNode('image_height').real())
    imageSize = (width, height)

    fs.release()

    return {
        'cameraMatrixLeft': cameraMatrixLeft,
        'cameraMatrixRight': cameraMatrixRight,
        'distCoeffsLeft': distCoeffsLeft,
        'distCoeffsRight': distCoeffsRight,
        'R': R,
        'T': T,
        'imageSize': imageSize
    }

def test_frame(calib, frame_idx):
    """Test rectification on a calibration dataset frame"""
    left_path = f'/unlook_calib_dataset/dataset_20251110_112023/left/frame_{frame_idx:03d}.png'
    right_path = f'/unlook_calib_dataset/dataset_20251110_112023/right/frame_{frame_idx:03d}.png'

    print(f'\nTesting frame_{frame_idx:03d}.png...')

    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print(f'  ✗ Failed to load frame {frame_idx}')
        return None

    # Recompute rectification
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        calib['imageSize'],
        calib['R'], calib['T'],
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=1,
        newImageSize=calib['imageSize']
    )

    map1L, map2L = cv2.initUndistortRectifyMap(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        R1, P1, calib['imageSize'], cv2.CV_16SC2
    )

    map1R, map2R = cv2.initUndistortRectifyMap(
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        R2, P2, calib['imageSize'], cv2.CV_16SC2
    )

    rectL = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

    # Check alignment
    orb = cv2.ORB_create(nfeatures=500)
    kp1, desc1 = orb.detectAndCompute(rectL, None)
    kp2, desc2 = orb.detectAndCompute(rectR, None)

    if desc1 is None or desc2 is None or len(kp1) < 10 or len(kp2) < 10:
        print(f'  ⚠ Insufficient features (checkerboard)')
        return None

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)
    good_matches = matches[:min(50, len(matches))]

    y_errors = []
    for m in good_matches:
        pt1 = kp1[m.queryIdx].pt
        pt2 = kp2[m.trainIdx].pt
        y_diff = abs(pt1[1] - pt2[1])
        y_errors.append(y_diff)

    if len(y_errors) > 0:
        y_errors = np.array(y_errors)
        mean_err = y_errors.mean()
        print(f'  Matches: {len(y_errors)}')
        print(f'  Mean Y error: {mean_err:.2f} px', end='')

        if mean_err < 1.0:
            print(' ✓ EXCELLENT')
        elif mean_err < 2.0:
            print(' ✓ GOOD')
        elif mean_err < 5.0:
            print(' ⚠ ACCEPTABLE')
        else:
            print(' ✗ POOR')

        return mean_err

    return None

def main():
    print('=' * 80)
    print('TEST: CALIBRATION DATASET FRAMES')
    print('=' * 80)
    print('Testing rectification on frames used for calibration')
    print('Expected: <1px error if calibration is valid')

    calib = load_calibration()

    # Test several frames
    test_indices = [0, 5, 10, 15, 20, 25, 30, 35, 40, 45]

    errors = []
    for idx in test_indices:
        err = test_frame(calib, idx)
        if err is not None:
            errors.append(err)

    if len(errors) > 0:
        print('\n' + '=' * 80)
        print('SUMMARY')
        print('=' * 80)
        print(f'Tested frames: {len(errors)}')
        print(f'Average error: {np.mean(errors):.2f} px')
        print(f'Min error: {np.min(errors):.2f} px')
        print(f'Max error: {np.max(errors):.2f} px')

        if np.mean(errors) < 2.0:
            print('\n✓ Calibration is VALID on calibration dataset')
            print('  → Problem is that SCAN frames are different from calibration')
            print('  → Cameras may have moved OR different focus/lighting')
        else:
            print('\n✗ Calibration has errors even on calibration dataset!')
            print('  → Need to recalibrate with better dataset')

if __name__ == '__main__':
    main()
