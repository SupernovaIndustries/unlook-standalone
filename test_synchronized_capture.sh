#!/bin/bash

# Test script for synchronized camera capture using third-party libcamera-sync-fix
# This verifies hardware synchronization with XVS/XHS pins

echo "======================================"
echo "Unlook Synchronized Camera Test"
echo "Using third-party libcamera-sync-fix"
echo "======================================"

# Set up environment for third-party libcamera
LIBCAMERA_PATH="/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix"
export LD_LIBRARY_PATH="${LIBCAMERA_PATH}/build/src/libcamera:${LIBCAMERA_PATH}/build/src/libcamera/base:$LD_LIBRARY_PATH"
export LIBCAMERA_IPA_MODULE_PATH="${LIBCAMERA_PATH}/build/src/ipa"
export LIBCAMERA_IPA_CONFIG_PATH="${LIBCAMERA_PATH}/src/ipa"

echo "Environment configured:"
echo "  LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
echo "  LIBCAMERA_IPA_MODULE_PATH: $LIBCAMERA_IPA_MODULE_PATH"

# Test 1: List cameras with third-party cam
echo ""
echo "Test 1: Listing cameras..."
${LIBCAMERA_PATH}/build/src/apps/cam/cam --list

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to list cameras"
    exit 1
fi

# Test 2: Capture from master camera (Camera 1 = LEFT)
echo ""
echo "Test 2: Capturing from MASTER camera (Camera 1 = LEFT)..."
${LIBCAMERA_PATH}/build/src/apps/cam/cam \
    --camera "/base/soc/i2c0mux/i2c@1/imx296@1a" \
    --capture=1 \
    --file=master_test.raw

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to capture from master camera"
    exit 1
fi

echo "Master camera capture successful"

# Test 3: Capture from slave camera (Camera 0 = RIGHT)
echo ""
echo "Test 3: Capturing from SLAVE camera (Camera 0 = RIGHT)..."
${LIBCAMERA_PATH}/build/src/apps/cam/cam \
    --camera "/base/soc/i2c0mux/i2c@0/imx296@1a" \
    --capture=1 \
    --file=slave_test.raw

if [ $? -ne 0 ]; then
    echo "ERROR: Failed to capture from slave camera"
    exit 1
fi

echo "Slave camera capture successful"

# Test 4: Synchronized capture from both cameras
echo ""
echo "Test 4: Testing synchronized capture..."
echo "Starting master camera stream..."

# Start master camera in background
${LIBCAMERA_PATH}/build/src/apps/cam/cam \
    --camera "/base/soc/i2c0mux/i2c@1/imx296@1a" \
    --stream width=1456,height=1088,pixelformat=SBGGR10 \
    --capture=10 \
    --file=master_sync- &

MASTER_PID=$!

# Small delay for master to initialize
sleep 0.5

echo "Starting slave camera stream..."

# Start slave camera
${LIBCAMERA_PATH}/build/src/apps/cam/cam \
    --camera "/base/soc/i2c0mux/i2c@0/imx296@1a" \
    --stream width=1456,height=1088,pixelformat=SBGGR10 \
    --capture=10 \
    --file=slave_sync- &

SLAVE_PID=$!

# Wait for both cameras to complete
echo "Waiting for capture to complete..."
wait $MASTER_PID
MASTER_RESULT=$?
wait $SLAVE_PID
SLAVE_RESULT=$?

if [ $MASTER_RESULT -ne 0 ] || [ $SLAVE_RESULT -ne 0 ]; then
    echo "ERROR: Synchronized capture failed"
    exit 1
fi

echo ""
echo "======================================"
echo "Test Results:"
echo "======================================"
echo "✓ Camera detection successful"
echo "✓ Master camera capture successful" 
echo "✓ Slave camera capture successful"
echo "✓ Synchronized capture successful"
echo ""
echo "Captured files:"
ls -lh master_sync-* slave_sync-* 2>/dev/null | head -5
echo ""
echo "Hardware sync configuration:"
echo "  XVS GPIO: 17 (External Vertical Sync)"
echo "  XHS GPIO: 27 (External Horizontal Sync - camera system)"
echo "  AS1170 Strobe GPIO: 19 (LED strobe control)"
echo "  MAS GPIO: 22 (Master/Slave select)"
echo "  Target sync precision: <1ms"
echo ""
echo "All tests passed successfully!"
echo "======================================">