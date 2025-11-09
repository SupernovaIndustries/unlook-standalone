#!/bin/bash

# Run offline scan test with correct library paths

# Set library path
export LD_LIBRARY_PATH=/home/alessandro/unlook-standalone/build/src:/home/alessandro/unlook-standalone/build/src/calibration:/home/alessandro/unlook-standalone/build/src/stereo:/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/build/src/libcamera:/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH

# Default arguments
FRAMES_DIR="${1:-/home/alessandro/unlook_debug/scan_20251109_023628}"
CALIB_FILE="${2:-/unlook_calib/default.yaml}"

echo "=== Running offline scan test ==="
echo "Frames directory: $FRAMES_DIR"
echo "Calibration file: $CALIB_FILE"
echo ""

# Run the test
./build/test_offline_scan "$FRAMES_DIR" "$CALIB_FILE"
