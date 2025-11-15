#!/usr/bin/env python3
"""
SGBM with MAXIMUM/OPTIMIZED parameters for best quality
"""

import numpy as np
import cv2
import sys

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, verts, colors):
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

def load_calibration(calib_path='/unlook_calib/default.yaml'):
    fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f'ERROR: Failed to open {calib_path}')
        sys.exit(1)

    cameraMatrixLeft = fs.getNode('camera_matrix_left').mat()
    cameraMatrixRight = fs.getNode('camera_matrix_right').mat()
    distCoeffsLeft = fs.getNode('distortion_coeffs_left').mat()
    distCoeffsRight = fs.getNode('distortion_coeffs_right').mat()
    R1 = fs.getNode('rectification_transform_left').mat()
    R2 = fs.getNode('rectification_transform_right').mat()
    P1 = fs.getNode('projection_matrix_left').mat()
    P2 = fs.getNode('projection_matrix_right').mat()
    Q = fs.getNode('disparity_to_depth_matrix').mat()

    width = int(fs.getNode('image_width').real())
    height = int(fs.getNode('image_height').real())
    imageSize = (width, height)
    baseline_mm = fs.getNode('baseline_mm').real()

    fs.release()

    return {
        'cameraMatrixLeft': cameraMatrixLeft,
        'cameraMatrixRight': cameraMatrixRight,
        'distCoeffsLeft': distCoeffsLeft,
        'distCoeffsRight': distCoeffsRight,
        'R1': R1, 'R2': R2, 'P1': P1, 'P2': P2, 'Q': Q,
        'imageSize': imageSize, 'baseline_mm': baseline_mm
    }

def create_rectification_maps(calib):
    map1Left, map2Left = cv2.initUndistortRectifyMap(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        calib['R1'], calib['P1'], calib['imageSize'], cv2.CV_16SC2
    )
    map1Right, map2Right = cv2.initUndistortRectifyMap(
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        calib['R2'], calib['P2'], calib['imageSize'], cv2.CV_16SC2
    )
    return map1Left, map2Left, map1Right, map2Right

def main():
    scan_dir = '/home/alessandro/unlook_debug/scan_20251115_033519'
    left_path = f'{scan_dir}/00_raw_frame00_left.png'
    right_path = f'{scan_dir}/00_raw_frame00_right.png'
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('SGBM OPTIMIZED - MAXIMUM QUALITY PARAMETERS')
    print('=' * 80)

    calib = load_calibration()
    map1L, map2L, map1R, map2R = create_rectification_maps(calib)

    print(f'\nLoading images...')
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print('ERROR: Failed to load images')
        sys.exit(1)

    print('Applying rectification...')
    rectL = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

    # OPTIMIZED SGBM PARAMETERS - MAXIMUM QUALITY
    print('\nCreating SGBM matcher with OPTIMIZED parameters...')

    min_disp = 0
    num_disp = 16 * 12  # 192 disparities (increased from 160)
    block_size = 3      # Smaller block for more detail

    # Optimized P1, P2 for quality
    P1 = 8 * 1 * block_size ** 2     # Reduced for smoother
    P2 = 32 * 1 * block_size ** 2    # Reduced for smoother

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=P1,
        P2=P2,
        disp12MaxDiff=1,           # Strict left-right consistency
        uniquenessRatio=5,         # Lower = more permissive (was 10)
        speckleWindowSize=200,     # Larger = more aggressive filtering (was 100)
        speckleRange=32,           # Same
        preFilterCap=63,           # Maximum pre-filter
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # Best quality mode
    )

    print(f'  Block size: {block_size}x{block_size}')
    print(f'  Disparity range: {min_disp} to {min_disp + num_disp}')
    print(f'  P1: {P1}, P2: {P2}')
    print(f'  Uniqueness ratio: 5')
    print(f'  Speckle window: 200')
    print(f'  Mode: SGBM_3WAY (highest quality)')

    print('\nComputing disparity...')
    disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    # Apply WLS filter for additional smoothing
    print('Applying WLS post-filter...')
    left_matcher = stereo
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

    disp_left = left_matcher.compute(rectL, rectR)
    disp_right = right_matcher.compute(rectR, rectL)

    # WLS filter parameters
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    wls_filter.setLambda(8000)    # Smoothing strength
    wls_filter.setSigmaColor(1.5)  # Edge preservation

    filtered_disp = wls_filter.filter(disp_left, rectL, disparity_map_right=disp_right)
    filtered_disp = filtered_disp.astype(np.float32) / 16.0

    # Statistics
    valid_disp = filtered_disp[filtered_disp > min_disp]
    print(f'\nDisparity statistics (WLS filtered):')
    print(f'  Valid pixels: {len(valid_disp)} / {filtered_disp.size} ({100.0*len(valid_disp)/filtered_disp.size:.1f}%)')
    if len(valid_disp) > 0:
        print(f'  Min: {valid_disp.min():.2f}')
        print(f'  Max: {valid_disp.max():.2f}')
        print(f'  Mean: {valid_disp.mean():.2f}')

    # Save disparity map
    disp_normalized = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp_colored = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(f'{output_dir}/disparity_sgbm_optimized.png', disp_colored)
    print(f'\nSaved: {output_dir}/disparity_sgbm_optimized.png')

    # Generate 3D point cloud
    print('Generating 3D point cloud...')
    points = cv2.reprojectImageTo3D(filtered_disp, calib['Q'])

    imgL_color = cv2.imread(left_path)
    rectL_color = cv2.remap(imgL_color, map1L, map2L, cv2.INTER_LINEAR)
    colors = cv2.cvtColor(rectL_color, cv2.COLOR_BGR2RGB)

    mask = filtered_disp > min_disp
    out_points = points[mask]
    out_colors = colors[mask]

    ply_filename = f'{output_dir}/point_cloud_sgbm_optimized.ply'
    write_ply(ply_filename, out_points, out_colors)
    print(f'Saved: {ply_filename}')
    print(f'  Points: {len(out_points)}')

    if len(out_points) > 0:
        print(f'\n3D Point statistics:')
        print(f'  X range: {out_points[:,0].min():.2f} to {out_points[:,0].max():.2f} mm')
        print(f'  Y range: {out_points[:,1].min():.2f} to {out_points[:,1].max():.2f} mm')
        print(f'  Z range: {out_points[:,2].min():.2f} to {out_points[:,2].max():.2f} mm')

    print('\n' + '=' * 80)
    print('SGBM OPTIMIZED TEST COMPLETED')
    print('=' * 80)

if __name__ == '__main__':
    main()
