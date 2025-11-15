#!/usr/bin/env python3
"""
Quick summary of all test results
"""

import os

print('=' * 80)
print('STEREO MATCHING TEST RESULTS SUMMARY')
print('=' * 80)

print('\n1. SGBM STANDARD (OpenCV)')
print('-' * 80)
print('  Valid pixels: 37.2%')
print('  Mean disparity: 119.44')
print('  Z range: -193549 to -685 mm (VERY NOISY!)')
print('  Conclusion: Poor quality, large depth range indicates noise')

print('\n2. RECTIFICATION QUALITY')
print('-' * 80)
print('  Using PRECOMPUTED rectification from YAML:')
print('    Mean Y error: 20.82 px ✗')
print('  ')
print('  Using RECOMPUTED rectification (stereoRectify):')
print('    Mean Y error: 18.77 px ✗')
print('  ')
print('  Conclusion: PROBLEM IS IN CALIBRATION DATA ITSELF!')
print('              Not in precomputed parameters or code loading')

print('\n3. ROOT CAUSE ANALYSIS')
print('-' * 80)
print('  ✓ Distortion coeffs ARE being loaded correctly now')
print('  ✓ Code is correct (matches OpenCV sample)')
print('  ✗ Calibration has 15-20px residual epipolar error')
print('  ')
print('  This means:')
print('  - Either calibration dataset has issues')
print('  - Or cameras moved after calibration')
print('  - Or distortion model doesn\'t fit lens accurately')

print('\n4. NEXT STEPS')
print('-' * 80)
print('  Option A: NEW CALIBRATION with fresh dataset')
print('    - Capture new checkerboard images')
print('    - Ensure perfect rigidity')
print('    - Use more images (50+ pairs)')
print('  ')
print('  Option B: Test with DIFFERENT FRAMES')
print('    - Use frames from calibration dataset itself')
print('    - Should have 0.25px error if calibration is valid')
print('  ')
print('  Option C: Check HARDWARE')
print('    - Verify cameras haven\'t moved')
print('    - Check mount rigidity')
print('    - Test with known-good calibration target')

print('\n' + '=' * 80)
print('FILES GENERATED')
print('=' * 80)

files = [
    'disparity_sgbm_standard.png',
    'point_cloud_sgbm_standard.ply',
    'rectified_left_sgbm.png',
    'rectified_right_sgbm.png',
    'rectified_left_recomputed.png',
    'rectified_right_recomputed.png'
]

for f in files:
    if os.path.exists(f):
        size = os.path.getsize(f) / 1024 / 1024
        print(f'  ✓ {f} ({size:.1f} MB)')
    else:
        print(f'  ✗ {f} (not found)')

print()
