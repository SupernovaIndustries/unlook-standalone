#!/usr/bin/env python3
"""
Quick check for epipolar alignment in rectified images
"""

import cv2
import numpy as np

def check_alignment(left_path, right_path):
    print(f'Checking rectification alignment...')
    print(f'  Left: {left_path}')
    print(f'  Right: {right_path}')

    left = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if left is None or right is None:
        print('ERROR: Failed to load images')
        return

    # Detect ORB features
    orb = cv2.ORB_create(nfeatures=500)
    kp1, desc1 = orb.detectAndCompute(left, None)
    kp2, desc2 = orb.detectAndCompute(right, None)

    # Match features
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desc1, desc2)
    matches = sorted(matches, key=lambda x: x.distance)

    # Take best matches
    good_matches = matches[:min(100, len(matches))]

    # Calculate vertical errors
    y_errors = []
    for m in good_matches:
        pt1 = kp1[m.queryIdx].pt
        pt2 = kp2[m.trainIdx].pt
        y_diff = abs(pt1[1] - pt2[1])
        y_errors.append(y_diff)

    if len(y_errors) > 0:
        y_errors = np.array(y_errors)
        print(f'\nEpipolar Alignment (ORB features):')
        print(f'  Matches: {len(good_matches)}')
        print(f'  Mean Y error: {y_errors.mean():.2f} px')
        print(f'  Median Y error: {np.median(y_errors):.2f} px')
        print(f'  Max Y error: {y_errors.max():.2f} px')
        print(f'  Std Y error: {y_errors.std():.2f} px')

        # Check if aligned
        if y_errors.mean() < 2.0:
            print(f'\n  ✓ EXCELLENT: Mean error < 2px')
        elif y_errors.mean() < 5.0:
            print(f'\n  ⚠ GOOD: Mean error < 5px')
        elif y_errors.mean() < 10.0:
            print(f'\n  ⚠ ACCEPTABLE: Mean error < 10px')
        else:
            print(f'\n  ✗ POOR: Mean error > 10px')
    else:
        print('ERROR: No matches found')

if __name__ == '__main__':
    check_alignment(
        'rectified_left_sgbm.png',
        'rectified_right_sgbm.png'
    )
