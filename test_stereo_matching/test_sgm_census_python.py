#!/usr/bin/env python3
"""
Python implementation of Unlook SGM-Census algorithm
Replicates the C++ SGMCensus implementation for testing and validation
"""

import numpy as np
import cv2
import sys
from numba import jit

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

@jit(nopython=True)
def compute_census_transform(image, census_window=7):
    """
    Compute Census Transform (7x7 window, 64-bit descriptor)
    Matches C++ implementation in SGMCensus.cpp
    """
    height, width = image.shape
    radius = census_window // 2
    census = np.zeros((height, width), dtype=np.uint64)

    for y in range(radius, height - radius):
        for x in range(radius, width - radius):
            descriptor = np.uint64(0)
            center_val = image[y, x]
            shift_count = 0
            total_bits = census_window * census_window

            # Iterate in same order as C++ code
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    # Skip center pixel
                    if shift_count != total_bits // 2:
                        descriptor <<= 1  # Left shift first
                        pixel_val = image[y + dy, x + dx]
                        # bit = 1 if pixel < center
                        if pixel_val < center_val:
                            descriptor |= 1
                    shift_count += 1

            census[y, x] = descriptor

    return census

@jit(nopython=True)
def hamming_distance(a, b):
    """Compute Hamming distance between two 64-bit integers"""
    xor = a ^ b
    count = 0
    while xor:
        count += xor & 1
        xor >>= 1
    return count

def compute_matching_cost(census_left, census_right, num_disparities, vertical_range=8):
    """
    Compute matching cost using Hamming distance
    With vertical search to compensate for epipolar error
    """
    height, width = census_left.shape
    D = num_disparities

    cost_volume = np.full((height, width, D), 255, dtype=np.uint8)

    print(f'  Computing matching cost with vertical range ±{vertical_range}px...')

    for y in range(height):
        left_row = census_left[y, :]

        for x in range(width):
            left_desc = left_row[x]

            for d in range(D):
                xr = x - d
                if xr < 0 or xr >= width:
                    continue

                # Search in vertical window ±vertical_range
                min_cost = 255
                for dy in range(-vertical_range, vertical_range + 1):
                    yr = y + dy
                    if yr < 0 or yr >= height:
                        continue

                    right_desc = census_right[yr, xr]
                    cost = hamming_distance(left_desc, right_desc)
                    min_cost = min(min_cost, cost)

                cost_volume[y, x, d] = min_cost

    return cost_volume

def aggregate_costs_sgm(cost_volume, P1=10, P2=120, use_8_paths=True):
    """
    SGM cost aggregation with 8 paths
    Simplified version of the C++ implementation
    """
    height, width, D = cost_volume.shape

    print(f'  Aggregating costs with SGM (P1={P1}, P2={P2}, {"8" if use_8_paths else "4"} paths)...')

    # Define path directions
    if use_8_paths:
        paths = [
            (0, 1),   # Right
            (1, 0),   # Down
            (1, 1),   # Diagonal down-right
            (1, -1),  # Diagonal down-left
            (0, -1),  # Left
            (-1, 0),  # Up
            (-1, -1), # Diagonal up-left
            (-1, 1)   # Diagonal up-right
        ]
    else:
        paths = [(0, 1), (1, 0), (1, 1), (1, -1)]

    aggregated_cost = np.zeros((height, width, D), dtype=np.uint32)

    for path_idx, (dy, dx) in enumerate(paths):
        # For each path, aggregate costs
        path_cost = cost_volume.copy().astype(np.uint32)

        # Determine iteration order based on path direction
        if dy >= 0:
            y_range = range(height)
        else:
            y_range = range(height - 1, -1, -1)

        if dx >= 0:
            x_range = range(width)
        else:
            x_range = range(width - 1, -1, -1)

        for y in y_range:
            for x in x_range:
                # Previous pixel in path
                py = y - dy
                px = x - dx

                if py < 0 or py >= height or px < 0 or px >= width:
                    # First pixel in path
                    continue

                prev_costs = path_cost[py, px, :]
                min_prev = prev_costs.min()

                for d in range(D):
                    cost = cost_volume[y, x, d]

                    # L(p,d) = C(p,d) + min(
                    #   L(p-r,d),           # Same disparity
                    #   L(p-r,d-1) + P1,    # Disparity ±1
                    #   L(p-r,d+1) + P1,
                    #   min_k(L(p-r,k)) + P2  # Any other disparity
                    # ) - min_k(L(p-r,k))

                    # Same disparity
                    cost0 = prev_costs[d]

                    # Disparity - 1
                    cost1 = prev_costs[d - 1] + P1 if d > 0 else prev_costs[d] + P2

                    # Disparity + 1
                    cost2 = prev_costs[d + 1] + P1 if d < D - 1 else prev_costs[d] + P2

                    # Any other disparity
                    cost3 = min_prev + P2

                    path_cost[y, x, d] = cost + min(cost0, cost1, cost2, cost3) - min_prev

        # Accumulate path costs
        aggregated_cost += path_cost

    return aggregated_cost

def select_disparities_wta(aggregated_cost):
    """
    Winner-Takes-All disparity selection
    """
    print('  Selecting disparities (WTA)...')
    disparity = np.argmin(aggregated_cost, axis=2).astype(np.int16)
    return disparity

def main():
    # Paths
    scan_dir = '/home/alessandro/unlook_debug/scan_20251115_033519'
    left_path = f'{scan_dir}/00_raw_frame00_left.png'
    right_path = f'{scan_dir}/00_raw_frame00_right.png'
    output_dir = '/home/alessandro/unlook-standalone/test_stereo_matching'

    print('=' * 80)
    print('SGM-CENSUS PYTHON TEST - UNLOOK ALGORITHM')
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

    # SGM-Census parameters (matching C++ defaults)
    census_window = 7
    num_disparities = 160
    vertical_range = 8  # ±8 pixels for epipolar error compensation
    P1 = 10
    P2 = 120

    # Compute Census Transform
    print(f'\nComputing Census Transform ({census_window}x{census_window})...')
    census_left = compute_census_transform(rectL, census_window)
    census_right = compute_census_transform(rectR, census_window)
    print('  ✓ Census transform completed')

    # Compute Matching Cost
    print(f'\nComputing matching cost (Hamming distance)...')
    cost_volume = compute_matching_cost(census_left, census_right, num_disparities, vertical_range)
    print('  ✓ Matching cost completed')

    # SGM Aggregation
    print(f'\nSGM cost aggregation...')
    aggregated_cost = aggregate_costs_sgm(cost_volume, P1, P2, use_8_paths=True)
    print('  ✓ SGM aggregation completed')

    # WTA Selection
    disparity = select_disparities_wta(aggregated_cost)
    print('  ✓ WTA selection completed')

    # Statistics
    valid_disp = disparity[disparity > 0]
    print(f'\nDisparity statistics:')
    print(f'  Valid pixels: {len(valid_disp)} / {disparity.size} ({100.0*len(valid_disp)/disparity.size:.1f}%)')
    if len(valid_disp) > 0:
        print(f'  Min: {valid_disp.min():.2f}')
        print(f'  Max: {valid_disp.max():.2f}')
        print(f'  Mean: {valid_disp.mean():.2f}')

    # Save disparity map (scale to 16-bit for compatibility)
    disparity_16bit = (disparity.astype(np.float32) * 16.0).astype(np.int16)
    disp_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disp_colored = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
    cv2.imwrite(f'{output_dir}/disparity_sgm_census_python.png', disp_colored)
    print(f'\nSaved: {output_dir}/disparity_sgm_census_python.png')

    # Generate 3D point cloud
    print('\nGenerating 3D point cloud...')
    disp_float = disparity.astype(np.float32)
    points = cv2.reprojectImageTo3D(disp_float, calib['Q'])

    # Load color image
    imgL_color = cv2.imread(left_path)
    rectL_color = cv2.remap(imgL_color, map1L, map2L, cv2.INTER_LINEAR)
    colors = cv2.cvtColor(rectL_color, cv2.COLOR_BGR2RGB)

    # Filter invalid points
    mask = disparity > 0
    out_points = points[mask]
    out_colors = colors[mask]

    # Write PLY file
    ply_filename = f'{output_dir}/point_cloud_sgm_census_python.ply'
    write_ply(ply_filename, out_points, out_colors)
    print(f'Saved: {ply_filename}')
    print(f'  Points: {len(out_points)}')

    if len(out_points) > 0:
        print(f'\n3D Point statistics:')
        print(f'  X range: {out_points[:,0].min():.2f} to {out_points[:,0].max():.2f} mm')
        print(f'  Y range: {out_points[:,1].min():.2f} to {out_points[:,1].max():.2f} mm')
        print(f'  Z range: {out_points[:,2].min():.2f} to {out_points[:,2].max():.2f} mm')

    print('\n' + '=' * 80)
    print('SGM-CENSUS PYTHON TEST COMPLETED')
    print('=' * 80)

if __name__ == '__main__':
    main()
