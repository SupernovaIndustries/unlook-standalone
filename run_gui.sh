#!/bin/bash
# Unlook Scanner GUI - Gesture Recognition Edition
#
# Launch the complete GUI with gesture recognition integrated

# Get script directory (absolute path)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set library paths (ORDER MATTERS!)
export LD_LIBRARY_PATH="$SCRIPT_DIR/build/src:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SCRIPT_DIR/build/src/pointcloud:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SCRIPT_DIR/third-party/onnxruntime/onnxruntime-linux-aarch64-1.16.3/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SCRIPT_DIR/third-party/libcamera-sync-fix/build/src/libcamera:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="$SCRIPT_DIR/third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH"

# Model paths for gesture recognition
export GESTURE_MODEL_PATH="$SCRIPT_DIR/third-party/hand-gesture-recognition-using-onnx/model"

echo "=========================================="
echo " Unlook 3D Scanner - Gesture Recognition"
echo "=========================================="
echo ""
echo "Hardware:"
echo "  • 2x IMX296 Cameras (Hardware Sync)"
echo "  • ONNX Runtime 1.16.3 (ARM64)"
echo "  • Gesture Recognition Ready"
echo ""
echo "Features:"
echo "  • Camera Preview"
echo "  • Depth Test"
echo "  • GESTURE RECOGNITION (NEW!)"
echo "  • Options"
echo ""
echo "Starting GUI..."
echo ""

# Check if executable exists
if [ ! -f "$SCRIPT_DIR/build/src/gui/unlook_scanner" ]; then
    echo "ERROR: GUI not found!"
    echo "Please build first: cd $SCRIPT_DIR/build && make unlook_scanner"
    exit 1
fi

# Run GUI from script directory
cd "$SCRIPT_DIR/build"
./src/gui/unlook_scanner "$@"
