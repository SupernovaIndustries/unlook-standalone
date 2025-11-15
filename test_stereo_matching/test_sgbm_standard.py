#!/usr/bin/env python3
"""
Test SGBM stereo matching with Unlook calibration data
Loads ALL calibration parameters from default.yaml and tests standard OpenCV SGBM
"""

import numpy as np
import cv2
import sys

# PLY header template
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
    """Write PLY file with vertices and colors"""
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])
    with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

def load_calibration(calib_path='/unlook_calib/default.yaml'):
    """Load ALL calibration parameters from YAML file"""
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

    # Load precomputed rectification transforms (CRITICAL!)
    R1 = fs.getNode('rectification_transform_left').mat()
    R2 = fs.getNode('rectification_transform_right').mat()
    P1 = fs.getNode('projection_matrix_left').mat()
    P2 = fs.getNode('projection_matrix_right').mat()
    Q = fs.getNode('disparity_to_depth_matrix').mat()

    # Load metadata
    width = int(fs.getNode('image_width').real())
    height = int(fs.getNode('image_height').real())
    imageSize = (width, height)
    baseline_mm = fs.getNode('baseline_mm').real()

    fs.release()

    print(f'  Image size: {width}x{height}')
    print(f'  Baseline: {baseline_mm:.2f} mm')
    print(f'  Camera matrix left fx: {cameraMatrixLeft[0,0]:.2f}')
    print(f'  Camera matrix right fx: {cameraMatrixRight[0,0]:.2f}')
    print(f'  Distortion coeffs left: {distCoeffsLeft.ravel()[:3]}')
    print(f'  Distortion coeffs right: {distCoeffsRight.ravel()[:3]}')

    return {
        'cameraMatrixLeft': cameraMatrixLeft,
        'cameraMatrixRight': cameraMatrixRight,
        'distCoeffsLeft': distCoeffsLeft,
        'distCoeffsRight': distCoeffsRight,
        'R': R,
        'T': T,
        'R1': R1,
        'R2': R2,
        'P1': P1,
        'P2': P2,
        'Q': Q,
        'imageSize': imageSize,
        'baseline_mm': baseline_mm
    }

def create_rectification_maps(calib):
    """Create rectification maps using precomputed calibration parameters"""
    print('Creating rectification maps...')

    # Use CV_16SC2 format (matching C++ code)
    map1Left, map2Left = cv2.initUndistortRectifyMap(
        calib['cameraMatrixLeft'], calib['distCoeffsLeft'],
        calib['R1'], calib['P1'],
        calib['imageSize'], cv2.CV_16SC2
    )

    map1Right, map2Right = cv2.initUndistortRectifyMap(
        calib['cameraMatrixRight'], calib['distCoeffsRight'],
        calib['R2'], calib['P2'],
        calib['imageSize'], cv2.CV_16SC2
    )

    return map1Left, map2Left, map1Right, map2Right

def main():
    # Paths
    scan_dir = '/home/alessandro/unlook_debug/scan_20251115_033519'
    left_path = f'{scan_dir}/00_raw_frame00_left.png'
    right_path = f'{scan_dir}/00_raw_frame00_right.png'
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('SGBM STANDARD TEST - UNLOOK CALIBRATION')
    print('=' * 80)

    # Load calibration (ALL parameters)
    calib = load_calibration()

    # Create rectification maps
    map1L, map2L, map1R, map2R = create_rectification_maps(calib)

    # Load images
    print(f'\nLoading images...')
    print(f'  Left: {left_path}')
    print(f'  Right: {right_path}')
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print('ERROR: Failed to load images')
        sys.exit(1)

    print(f'  Image size: {imgL.shape[1]}x{imgL.shape[0]}')

    # Apply rectification
    print('\nApplying rectification...')
    rectL = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

    # Save rectified images
    cv2.imwrite(f'{output_dir}/rectified_left_sgbm.png', rectL)
    cv2.imwrite(f'{output_dir}/rectified_right_sgbm.png', rectR)
    print(f'  Saved: {output_dir}/rectified_left_sgbm.png')
    print(f'  Saved: {output_dir}/rectified_right_sgbm.png')

    # Create SGBM matcher - optimized parameters for our scanner
    print('\nCreating SGBM matcher...')
    window_size = 5
    min_disp = 0
    num_disp = 16 * 10  # 160 disparities (must be divisible by 16)

    # P1, P2 penalties - using OpenCV recommended values
    P1 = 8 * 3 * window_size ** 2
    P2 = 32 * 3 * window_size ** 2

    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        P1=P1,
        P2=P2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
    )

    print(f'  Window size: {window_size}')
    print(f'  Min disparity: {min_disp}')
    print(f'  Num disparities: {num_disp}')
    print(f'  P1: {P1}')
    print(f'  P2: {P2}')
    print(f'  Mode: SGBM_3WAY')

    # Compute disparity
    print('\nComputing disparity...')
    disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    # Statistics
    valid_disp = disp[disp > min_disp]
    print(f'\nDisparity statistics:')
    print(f'  Valid pixels: {len(valid_disp)} / {disp.size} ({100.0*len(valid_disp)/disp.size:.1f}%)')
    print(f'  Min: {valid_disp.min():.2f}')
    print(f'  Max: {valid_disp.max():.2f}')
    print(f'  Mean: {valid_disp.mean():.2f}')

    # Save disparity map
    disp_normalized = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp_colored = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(f'{output_dir}/disparity_sgbm_standard.png', disp_colored)
    print(f'\nSaved: {output_dir}/disparity_sgbm_standard.png')

    # Generate 3D point cloud using calibration Q matrix
    print('\nGenerating 3D point cloud...')
    points = cv2.reprojectImageTo3D(disp, calib['Q'])

    # Load color image for point cloud
    imgL_color = cv2.imread(left_path)
    rectL_color = cv2.remap(imgL_color, map1L, map2L, cv2.INTER_LINEAR)
    colors = cv2.cvtColor(rectL_color, cv2.COLOR_BGR2RGB)

    # Filter invalid points
    mask = disp > min_disp
    out_points = points[mask]
    out_colors = colors[mask]

    # Write PLY file
    ply_filename = f'{output_dir}/point_cloud_sgbm_standard.ply'
    write_ply(ply_filename, out_points, out_colors)
    print(f'Saved: {ply_filename}')
    print(f'  Points: {len(out_points)}')

    # 3D statistics
    print(f'\n3D Point statistics:')
    print(f'  X range: {out_points[:,0].min():.2f} to {out_points[:,0].max():.2f} mm')
    print(f'  Y range: {out_points[:,1].min():.2f} to {out_points[:,1].max():.2f} mm')
    print(f'  Z range: {out_points[:,2].min():.2f} to {out_points[:,2].max():.2f} mm')

    print('\n' + '=' * 80)
    print('SGBM STANDARD TEST COMPLETED')
    print('=' * 80)

if __name__ == '__main__':
    main()
