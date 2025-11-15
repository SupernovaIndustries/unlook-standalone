#!/usr/bin/env python3
"""
Compare stereo matching results from different algorithms
"""

import cv2
import numpy as np
import sys
import os

def analyze_disparity(disp_path, name):
    """Analyze disparity map and print statistics"""
    if not os.path.exists(disp_path):
        print(f"  ✗ {name}: File not found")
        return None

    # Read disparity image
    disp_color = cv2.imread(disp_path)
    if disp_color is None:
        print(f"  ✗ {name}: Failed to load")
        return None

    # Convert to grayscale for analysis
    disp_gray = cv2.cvtColor(disp_color, cv2.COLOR_BGR2GRAY)

    # Statistics
    valid_pixels = np.count_nonzero(disp_gray > 0)
    total_pixels = disp_gray.size
    valid_percent = 100.0 * valid_pixels / total_pixels

    valid_disp = disp_gray[disp_gray > 0]
    if len(valid_disp) == 0:
        print(f"  ✗ {name}: No valid disparities")
        return None

    stats = {
        'name': name,
        'path': disp_path,
        'valid_pixels': valid_pixels,
        'valid_percent': valid_percent,
        'min': valid_disp.min(),
        'max': valid_disp.max(),
        'mean': valid_disp.mean(),
        'std': valid_disp.std()
    }

    print(f"  ✓ {name}:")
    print(f"      Valid pixels: {valid_pixels}/{total_pixels} ({valid_percent:.1f}%)")
    print(f"      Range: {stats['min']:.1f} - {stats['max']:.1f}")
    print(f"      Mean: {stats['mean']:.1f} ± {stats['std']:.1f}")

    return stats

def analyze_ply(ply_path, name):
    """Analyze PLY file and print statistics"""
    if not os.path.exists(ply_path):
        print(f"  ✗ {name}: File not found")
        return None

    # Count vertices
    with open(ply_path, 'r') as f:
        for line in f:
            if line.startswith('element vertex'):
                num_vertices = int(line.split()[-1])
                print(f"  ✓ {name}: {num_vertices} vertices")
                return {'name': name, 'vertices': num_vertices}

    print(f"  ✗ {name}: Invalid PLY format")
    return None

def main():
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('STEREO MATCHING RESULTS COMPARISON')
    print('=' * 80)

    # Analyze disparity maps
    print('\nDisparity Map Analysis:')
    print('-' * 80)

    sgbm_stats = analyze_disparity(
        f'{output_dir}/disparity_sgbm_standard.png',
        'SGBM Standard'
    )

    census_stats = analyze_disparity(
        f'{output_dir}/disparity_census_binary.png',
        'Census Binary SGBM'
    )

    census_python_stats = analyze_disparity(
        f'{output_dir}/disparity_sgm_census_python.png',
        'SGM-Census Python'
    )

    # Analyze PLY files
    print('\nPoint Cloud Analysis:')
    print('-' * 80)

    sgbm_ply = analyze_ply(
        f'{output_dir}/point_cloud_sgbm_standard.ply',
        'SGBM Standard'
    )

    census_ply = analyze_ply(
        f'{output_dir}/point_cloud_census_binary.ply',
        'Census Binary SGBM'
    )

    census_python_ply = analyze_ply(
        f'{output_dir}/point_cloud_sgm_census_python.ply',
        'SGM-Census Python'
    )

    # Comparison summary
    print('\n' + '=' * 80)
    print('SUMMARY')
    print('=' * 80)

    all_stats = [s for s in [sgbm_stats, census_stats, census_python_stats] if s is not None]

    if len(all_stats) > 0:
        print('\nValid Pixel Percentage Ranking:')
        sorted_stats = sorted(all_stats, key=lambda x: x['valid_percent'], reverse=True)
        for i, s in enumerate(sorted_stats, 1):
            print(f"  {i}. {s['name']}: {s['valid_percent']:.1f}%")

        print('\nMean Disparity Ranking:')
        sorted_stats = sorted(all_stats, key=lambda x: x['mean'], reverse=True)
        for i, s in enumerate(sorted_stats, 1):
            print(f"  {i}. {s['name']}: {s['mean']:.1f}")
    else:
        print('No valid results to compare')

    print('\nRecommendations:')
    print('  1. Compare disparity maps visually for quality assessment')
    print('  2. View PLY files in MeshLab to check 3D reconstruction quality')
    print('  3. Check for cone/frustum pattern in point clouds')
    print('  4. Verify epipolar alignment in rectified images')

if __name__ == '__main__':
    main()
