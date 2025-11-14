#!/usr/bin/env python3
"""
Analyze disparity and 3D data exported from Unlook scanner
Helps diagnose cone/frustum pattern issues
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

def load_data(filepath):
    """Load disparity_3d_data.txt"""
    print(f"Loading data from: {filepath}")

    # Format: x y disparity X Y Z
    data = np.loadtxt(filepath, comments='#')

    x_pixel = data[:, 0].astype(int)
    y_pixel = data[:, 1].astype(int)
    disparity = data[:, 2]
    X = data[:, 3]
    Y = data[:, 4]
    Z = data[:, 5]

    print(f"Loaded {len(data)} points")
    return x_pixel, y_pixel, disparity, X, Y, Z

def analyze_data(x_pixel, y_pixel, disparity, X, Y, Z):
    """Analyze disparity and 3D data"""

    print("\n=== DISPARITY ANALYSIS ===")
    valid_disp = disparity[disparity > 0]
    print(f"Valid disparity pixels: {len(valid_disp)}/{len(disparity)} ({100*len(valid_disp)/len(disparity):.1f}%)")
    if len(valid_disp) > 0:
        print(f"Disparity range: [{valid_disp.min():.2f}, {valid_disp.max():.2f}] pixels")
        print(f"Disparity mean: {valid_disp.mean():.2f} pixels")
        print(f"Disparity median: {np.median(valid_disp):.2f} pixels")

    print("\n=== 3D COORDINATE ANALYSIS ===")
    # Filter out OpenCV invalid marker (10000)
    valid_mask = (Z < 10000) & (Z > 0)
    valid_X = X[valid_mask]
    valid_Y = Y[valid_mask]
    valid_Z = Z[valid_mask]

    print(f"Valid 3D points: {len(valid_Z)}/{len(Z)} ({100*len(valid_Z)/len(Z):.1f}%)")

    if len(valid_Z) > 0:
        print(f"\nX range: [{valid_X.min():.2f}, {valid_X.max():.2f}] mm")
        print(f"Y range: [{valid_Y.min():.2f}, {valid_Y.max():.2f}] mm")
        print(f"Z range: [{valid_Z.min():.2f}, {valid_Z.max():.2f}] mm")

        print(f"\nZ statistics:")
        print(f"  Mean: {valid_Z.mean():.2f} mm")
        print(f"  Median: {np.median(valid_Z):.2f} mm")
        print(f"  Std dev: {valid_Z.std():.2f} mm")

        # Check for cone pattern: X/Y spread should be constant, not proportional to Z
        print(f"\n=== CONE PATTERN DETECTION ===")

        # Sample points at different depths
        depth_bins = [300, 400, 500, 600, 700, 800, 900]
        print("Analyzing XY spread at different depths:")
        print("(If spread increases with depth → CONE pattern!)")

        for depth in depth_bins:
            mask = (valid_Z > depth - 50) & (valid_Z < depth + 50)
            if mask.sum() > 10:
                pts_at_depth = valid_X[mask]
                x_spread = pts_at_depth.max() - pts_at_depth.min()
                y_spread_pts = valid_Y[mask]
                y_spread = y_spread_pts.max() - y_spread_pts.min()
                print(f"  Depth ~{depth}mm: X spread = {x_spread:.1f}mm, Y spread = {y_spread:.1f}mm, N={mask.sum()}")

    return valid_mask

def plot_data(x_pixel, y_pixel, disparity, X, Y, Z, valid_mask):
    """Create diagnostic plots"""

    fig = plt.figure(figsize=(15, 10))

    # 1. Disparity map
    ax1 = fig.add_subplot(2, 3, 1)
    img_height = y_pixel.max() + 1
    img_width = x_pixel.max() + 1
    disp_img = np.zeros((img_height, img_width))
    for i in range(len(disparity)):
        if disparity[i] > 0:
            disp_img[y_pixel[i], x_pixel[i]] = disparity[i]
    ax1.imshow(disp_img, cmap='jet')
    ax1.set_title('Disparity Map')
    ax1.set_xlabel('X pixel')
    ax1.set_ylabel('Y pixel')
    plt.colorbar(ax1.images[0], ax=ax1, label='Disparity (pixels)')

    # 2. Disparity histogram
    ax2 = fig.add_subplot(2, 3, 2)
    valid_disp = disparity[disparity > 0]
    if len(valid_disp) > 0:
        ax2.hist(valid_disp, bins=50, edgecolor='black')
        ax2.set_title('Disparity Histogram')
        ax2.set_xlabel('Disparity (pixels)')
        ax2.set_ylabel('Count')
        ax2.grid(True, alpha=0.3)

    # 3. 3D point cloud (top view XZ)
    ax3 = fig.add_subplot(2, 3, 3)
    valid_X = X[valid_mask]
    valid_Z = Z[valid_mask]
    if len(valid_Z) > 0:
        scatter = ax3.scatter(valid_X, valid_Z, c=valid_Z, cmap='viridis', s=1, alpha=0.5)
        ax3.set_title('Point Cloud Top View (X-Z)')
        ax3.set_xlabel('X (mm)')
        ax3.set_ylabel('Z depth (mm)')
        ax3.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax3, label='Depth (mm)')

    # 4. 3D point cloud (side view YZ)
    ax4 = fig.add_subplot(2, 3, 4)
    valid_Y = Y[valid_mask]
    if len(valid_Z) > 0:
        scatter = ax4.scatter(valid_Y, valid_Z, c=valid_Z, cmap='viridis', s=1, alpha=0.5)
        ax4.set_title('Point Cloud Side View (Y-Z)')
        ax4.set_xlabel('Y (mm)')
        ax4.set_ylabel('Z depth (mm)')
        ax4.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax4, label='Depth (mm)')

    # 5. Z depth histogram
    ax5 = fig.add_subplot(2, 3, 5)
    if len(valid_Z) > 0:
        ax5.hist(valid_Z, bins=50, edgecolor='black')
        ax5.set_title('Depth (Z) Histogram')
        ax5.set_xlabel('Depth (mm)')
        ax5.set_ylabel('Count')
        ax5.grid(True, alpha=0.3)

        # Mark target range
        ax5.axvline(200, color='r', linestyle='--', label='Min depth (200mm)')
        ax5.axvline(1000, color='r', linestyle='--', label='Max depth (1000mm)')
        ax5.legend()

    # 6. 3D scatter plot
    ax6 = fig.add_subplot(2, 3, 6, projection='3d')
    if len(valid_Z) > 0:
        # Sample for performance
        n_samples = min(10000, len(valid_Z))
        indices = np.random.choice(len(valid_Z), n_samples, replace=False)
        ax6.scatter(valid_X[indices], valid_Y[indices], valid_Z[indices],
                   c=valid_Z[indices], cmap='viridis', s=1, alpha=0.5)
        ax6.set_title('3D Point Cloud (sampled)')
        ax6.set_xlabel('X (mm)')
        ax6.set_ylabel('Y (mm)')
        ax6.set_zlabel('Z depth (mm)')

    plt.tight_layout()
    plt.savefig('disparity_analysis.png', dpi=150, bbox_inches='tight')
    print(f"\nSaved plot to: disparity_analysis.png")
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_disparity_3d.py <path_to_disparity_3d_data.txt>")
        print("Example: python3 analyze_disparity_3d.py /home/alessandro/unlook_debug/scan_*/disparity_3d_data.txt")
        sys.exit(1)

    filepath = sys.argv[1]

    try:
        x_pixel, y_pixel, disparity, X, Y, Z = load_data(filepath)
        valid_mask = analyze_data(x_pixel, y_pixel, disparity, X, Y, Z)
        plot_data(x_pixel, y_pixel, disparity, X, Y, Z, valid_mask)
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
