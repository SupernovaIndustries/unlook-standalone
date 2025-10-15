#!/bin/bash
# Unlook Gesture Recognition Test Script
#
# Usage:
#   ./test_gesture.sh                    # Test with webcam
#   ./test_gesture.sh video.mp4          # Test with video file
#   ./test_gesture.sh --help             # Show help

# Set library paths
export LD_LIBRARY_PATH="$PWD/third-party/onnxruntime/onnxruntime-linux-aarch64-1.16.3/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$PWD/third-party/libcamera-sync-fix/build/src/libcamera:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$PWD/third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$PWD/build/src:$LD_LIBRARY_PATH"

# Model paths
export GESTURE_MODEL_PATH="$PWD/third-party/hand-gesture-recognition-using-onnx/model"

echo "=== Unlook Gesture Recognition Test ==="
echo ""
echo "Library paths configured:"
echo "  - ONNX Runtime: $(ls third-party/onnxruntime/*/lib/libonnxruntime.so* 2>/dev/null | head -1)"
echo "  - Models: $GESTURE_MODEL_PATH"
echo ""

# Check if executable exists
if [ ! -f "build/examples/gesture_test_camera" ]; then
    echo "ERROR: gesture_test_camera not found!"
    echo "Please build first: cd build && make gesture_test_camera"
    exit 1
fi

# Run the test program with REAL hardware cameras
echo "Starting gesture recognition test with REAL IMX296 cameras..."
echo "Hardware Sync: XVS/XHS enabled"
echo "Press CTRL+C to stop"
echo ""

cd build/examples
if [ "$1" == "--help" ] || [ "$1" == "-h" ]; then
    ./gesture_test_camera --help
else
    ./gesture_test_camera "$@"
fi
