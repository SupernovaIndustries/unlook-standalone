#!/usr/bin/env python3
"""
Test Census-based stereo matching (StereoBinarySGBM from opencv_contrib)
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

    # Load precomputed rectification transforms
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

    return {
        'cameraMatrixLeft': cameraMatrixLeft,
        'cameraMatrixRight': cameraMatrixRight,
        'distCoeffsLeft': distCoeffsLeft,
        'distCoeffsRight': distCoeffsRight,
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
    # Check if opencv_contrib stereo module is available
    try:
        # Test if cv2.stereo exists
        _ = cv2.stereo.StereoBinarySGBM_create
        print('✓ OpenCV contrib stereo module is available')
    except AttributeError:
        print('✗ OpenCV contrib stereo module NOT available')
        print('  StereoBinarySGBM (Census-based) requires opencv-contrib-python')
        print('  Install with: pip3 install opencv-contrib-python==4.6.0.66')
        sys.exit(1)

    # Paths
    scan_dir = '/home/alessandro/unlook_debug/scan_20251115_033519'
    left_path = f'{scan_dir}/00_raw_frame00_left.png'
    right_path = f'{scan_dir}/00_raw_frame00_right.png'
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('CENSUS BINARY SGBM TEST - UNLOOK CALIBRATION')
    print('=' * 80)

    # Load calibration
    calib = load_calibration()

    # Create rectification maps
    map1L, map2L, map1R, map2R = create_rectification_maps(calib)

    # Load images
    print(f'\nLoading images...')
    imgL = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
    imgR = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)

    if imgL is None or imgR is None:
        print('ERROR: Failed to load images')
        sys.exit(1)

    # Apply rectification
    print('\nApplying rectification...')
    rectL = cv2.remap(imgL, map1L, map2L, cv2.INTER_LINEAR)
    rectR = cv2.remap(imgR, map1R, map2R, cv2.INTER_LINEAR)

    # Create StereoBinarySGBM matcher (Census-based)
    print('\nCreating StereoBinarySGBM matcher (Census transform)...')
    min_disp = 0
    num_disp = 16 * 10  # 160 disparities

    # Binary SGBM uses different penalty values
    P1 = 100
    P2 = 1000

    stereo = cv2.stereo.StereoBinarySGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=9,
        P1=P1,
        P2=P2,
        disp12MaxDiff=1,
        preFilterCap=0,
        uniquenessRatio=5,
        speckleWindowSize=100,
        speckleRange=32,
        mode=cv2.stereo.StereoBinarySGBM_MODE_SGBM
    )

    print(f'  Min disparity: {min_disp}')
    print(f'  Num disparities: {num_disp}')
    print(f'  Block size: 9')
    print(f'  P1: {P1}')
    print(f'  P2: {P2}')
    print(f'  Mode: Binary SGBM (Census transform)')

    # Compute disparity
    print('\nComputing disparity with Census transform...')
    disp = stereo.compute(rectL, rectR).astype(np.float32) / 16.0

    # Statistics
    valid_disp = disp[disp > min_disp]
    print(f'\nDisparity statistics:')
    print(f'  Valid pixels: {len(valid_disp)} / {disp.size} ({100.0*len(valid_disp)/disp.size:.1f}%)')
    if len(valid_disp) > 0:
        print(f'  Min: {valid_disp.min():.2f}')
        print(f'  Max: {valid_disp.max():.2f}')
        print(f'  Mean: {valid_disp.mean():.2f}')

    # Save disparity map
    disp_normalized = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp_colored = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(f'{output_dir}/disparity_census_binary.png', disp_colored)
    print(f'\nSaved: {output_dir}/disparity_census_binary.png')

    # Generate 3D point cloud
    print('\nGenerating 3D point cloud...')
    points = cv2.reprojectImageTo3D(disp, calib['Q'])

    # Load color image
    imgL_color = cv2.imread(left_path)
    rectL_color = cv2.remap(imgL_color, map1L, map2L, cv2.INTER_LINEAR)
    colors = cv2.cvtColor(rectL_color, cv2.COLOR_BGR2RGB)

    # Filter invalid points
    mask = disp > min_disp
    out_points = points[mask]
    out_colors = colors[mask]

    # Write PLY file
    ply_filename = f'{output_dir}/point_cloud_census_binary.ply'
    write_ply(ply_filename, out_points, out_colors)
    print(f'Saved: {ply_filename}')
    print(f'  Points: {len(out_points)}')

    if len(out_points) > 0:
        print(f'\n3D Point statistics:')
        print(f'  X range: {out_points[:,0].min():.2f} to {out_points[:,0].max():.2f} mm')
        print(f'  Y range: {out_points[:,1].min():.2f} to {out_points[:,1].max():.2f} mm')
        print(f'  Z range: {out_points[:,2].min():.2f} to {out_points[:,2].max():.2f} mm')

    print('\n' + '=' * 80)
    print('CENSUS BINARY SGBM TEST COMPLETED')
    print('=' * 80)

if __name__ == '__main__':
    main()
