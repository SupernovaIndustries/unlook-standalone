#!/bin/bash

# Build and Test Script for Real-time Performance
# Validates 30 FPS continuous capture with proper grayscale images

set -e

echo "================================================"
echo "Unlook 3D Scanner - Performance Test Build"
echo "Target: 30 FPS @ VGA Resolution"
echo "================================================"

# Build the project with optimizations
echo -e "\n[BUILD] Building with Release optimizations..."
./build.sh -t Release -j$(nproc)

if [ $? -ne 0 ]; then
    echo "[ERROR] Build failed!"
    exit 1
fi

# Compile the performance test
echo -e "\n[BUILD] Compiling performance test..."
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Compile test program
echo -e "\n[BUILD] Building test executable..."
g++ -O3 -march=native -std=c++17 \
    -I../include \
    -I/usr/local/include \
    -I/usr/include/opencv4 \
    ../test_realtime_performance.cpp \
    -o test_realtime_performance \
    -L. -lunlook \
    -L/usr/local/lib \
    -lopencv_core -lopencv_imgproc -lopencv_highgui \
    -lcamera -lcamera-base \
    -lpthread \
    -Wl,-rpath,. -Wl,-rpath,/usr/local/lib

if [ $? -ne 0 ]; then
    echo "[ERROR] Test compilation failed!"
    exit 1
fi

echo -e "\n[SUCCESS] Build completed successfully!"

# Run the performance test
echo -e "\n================================================"
echo "Starting Performance Test"
echo "================================================"
echo "This test will:"
echo "  1. Initialize cameras with 15ms exposure, 3x gain"
echo "  2. Start continuous capture at 30 FPS"
echo "  3. Convert SBGGR10 to grayscale with NEON"
echo "  4. Display live preview windows"
echo "  5. Report FPS and latency statistics"
echo ""
echo "Press Ctrl+C to stop the test"
echo "================================================"
echo ""

# Check if running with sudo (may be needed for real-time priorities)
if [ "$EUID" -ne 0 ]; then 
    echo "[INFO] Running without root. For best performance, consider: sudo $0"
fi

# Set performance governor if available
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo "[INFO] Current CPU governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
    if [ "$EUID" -eq 0 ]; then
        echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
        echo "[INFO] Set CPU governor to 'performance' mode"
    fi
fi

# Run the test
echo -e "\n[TEST] Starting real-time performance test...\n"
./test_realtime_performance

# Restore CPU governor if changed
if [ "$EUID" -eq 0 ] && [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
    echo ondemand | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
    echo "[INFO] Restored CPU governor to 'ondemand' mode"
fi

echo -e "\n[COMPLETE] Performance test finished"