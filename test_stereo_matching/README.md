# Stereo Matching Test Suite

Test suite for comparing different stereo matching algorithms using Unlook calibration data.

## Overview

This test suite evaluates three stereo matching approaches:

1. **SGBM Standard** - OpenCV's standard Semi-Global Block Matching
2. **Census Binary SGBM** - Census transform-based matching (requires opencv-contrib)
3. **SGM-Census Python** - Python recreation of Unlook's C++ SGM-Census algorithm

All tests use the **same calibration** from `/unlook_calib/default.yaml` and **same input frames** from the most recent scan.

## Files

- `test_sgbm_standard.py` - OpenCV SGBM with standard cost function
- `test_census_binary.py` - StereoBinarySGBM using Census transform (opencv_contrib)
- `test_sgm_census_python.py` - Python implementation of Unlook SGM-Census
- `run_all_tests.sh` - Run all tests sequentially
- `compare_results.py` - Compare results and generate statistics
- `stereo_match_official.py` - Original OpenCV sample (reference)
- `stereo_contrib.hpp` - OpenCV contrib stereo module header (reference)

## Requirements

```bash
# Required
pip3 install numpy opencv-python

# Optional (for Census Binary SGBM test)
pip3 install opencv-contrib-python==4.6.0.66

# Optional (for faster Python SGM-Census)
pip3 install numba
```

## Usage

### Run all tests
```bash
cd /home/alessandro/unlook-standalone/test_stereo_matching
./run_all_tests.sh
```

### Run individual test
```bash
python3 test_sgbm_standard.py
python3 test_census_binary.py
python3 test_sgm_census_python.py
```

### Compare results
```bash
python3 compare_results.py
```

## Output Files

Each test generates:
- **Disparity map** (colored visualization): `disparity_<algorithm>.png`
- **Point cloud** (PLY format): `point_cloud_<algorithm>.ply`
- **Rectified images** (first test only): `rectified_left_sgbm.png`, `rectified_right_sgbm.png`

## Viewing Results

### Disparity Maps
```bash
# View with any image viewer
eog disparity_sgbm_standard.png
```

### Point Clouds
```bash
# Install MeshLab
sudo apt install meshlab

# View PLY file
meshlab point_cloud_sgbm_standard.ply
```

## Algorithm Details

### SGBM Standard
- **Cost function**: Census + absolute difference
- **Optimization**: Semi-Global Matching (8 paths)
- **Penalties**: P1=600, P2=2400 (adaptive)
- **Parameters**: window_size=5, num_disparities=160

### Census Binary SGBM (opencv_contrib)
- **Cost function**: Binary Census transform (Hamming distance)
- **Optimization**: Semi-Global Matching
- **Penalties**: P1=100, P2=1000
- **Parameters**: block_size=9, num_disparities=160

### SGM-Census Python
- **Cost function**: 64-bit Census transform (7x7 window)
- **Matching**: Hamming distance with vertical search (±8px)
- **Optimization**: SGM 8-path aggregation
- **Penalties**: P1=10, P2=120
- **Features**: Epipolar error compensation

## Calibration Loading

All scripts load **ALL parameters** from `/unlook_calib/default.yaml`:
- Camera matrices (left/right)
- Distortion coefficients (left/right)
- Rotation and translation (R, T)
- **Precomputed rectification** (R1, R2, P1, P2)
- **Disparity-to-depth matrix (Q)**
- Baseline (69.46mm)

This ensures **exact consistency** with the C++ Unlook scanner code.

## Troubleshooting

### Error: "Failed to open calibration file"
- Check that `/unlook_calib/default.yaml` symlink exists
- Verify it points to a valid calibration file

### Error: "opencv_contrib stereo module NOT available"
- Census Binary SGBM requires `opencv-contrib-python`
- Install with: `pip3 install opencv-contrib-python==4.6.0.66`

### SGM-Census Python is slow
- Install `numba` for JIT compilation: `pip3 install numba`
- Or reduce `num_disparities` parameter in the script

### No valid disparities
- Check that input images are rectified correctly
- Verify calibration quality
- Try adjusting SGBM parameters (P1, P2, window_size)

## Expected Results

**CRITICAL BUG FIXED**: The C++ code was loading distortion coefficients with wrong key names:
- **Wrong**: `distortion_coefficients_left/right`
- **Correct**: `distortion_coeffs_left/right`

This meant **distortion coefficients were NOT loaded** in HandheldScanPipeline, causing poor rectification quality despite good calibration.

After fixing this bug, we expect:
- **Better rectification**: <2px vertical error (was 15px)
- **No cone pattern**: Linear XY spread should disappear
- **Higher valid pixel %**: More accurate stereo matching
- **Consistent depth**: Z values should be stable

## Next Steps

1. Rebuild C++ code with the distortion coefficient fix
2. Run new scan and compare with Python test results
3. Verify cone pattern is eliminated
4. Validate 3D reconstruction quality

## References

- OpenCV SGBM: https://docs.opencv.org/4.6.0/d2/d85/classcv_1_1StereoSGBM.html
- OpenCV contrib stereo: https://github.com/opencv/opencv_contrib/tree/4.6.0/modules/stereo
- Unlook SGM-Census: `/home/alessandro/unlook-standalone/src/stereo/SGMCensus.cpp`
