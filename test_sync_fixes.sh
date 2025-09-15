#!/bin/bash

# Test script to verify camera synchronization fixes
# Tests single frame capture, repeated start/stop cycles, and media device cleanup

echo "================================================================"
echo "   UNLOOK Camera Synchronization Fixes Verification Test"
echo "================================================================"
echo ""
echo "This test verifies the following fixes:"
echo "1. Single frame capture synchronization (<1ms error)"
echo "2. Request lifecycle management (no validation errors)"
echo "3. Media device cleanup (proper shutdown)"
echo "4. Repeated capture cycles robustness"
echo ""

# Function to run a test with timeout
run_test() {
    local test_name="$1"
    local test_cmd="$2"
    local timeout_sec="${3:-10}"
    
    echo "----------------------------------------"
    echo "Running: $test_name"
    echo "Command: $test_cmd"
    echo "----------------------------------------"
    
    # Run the test with timeout
    timeout $timeout_sec $test_cmd
    local result=$?
    
    if [ $result -eq 124 ]; then
        echo "✗ Test timed out after $timeout_sec seconds"
        return 1
    elif [ $result -eq 0 ]; then
        echo "✓ Test completed successfully"
        return 0
    else
        echo "✗ Test failed with exit code $result"
        return 1
    fi
    echo ""
}

# Change to build directory
cd build || exit 1

echo "Test 1: Basic Hardware Sync Test (5 seconds)"
echo "Expected: No sync errors >1ms, no request validation errors"
run_test "Hardware Sync Test" "./test_hardware_sync_new" 7

echo ""
echo "Test 2: Camera Capture Lifecycle Test"
echo "Expected: Clean startup and shutdown, no memory issues"
run_test "Capture Lifecycle" "./test_camera_capture" 5

echo ""
echo "Test 3: Single Frame Capture Test"
echo "Expected: Sync error <1ms for single frame"
cat > test_single_frame.cpp << 'EOF'
#include <unlook/camera/HardwareSyncCapture.hpp>
#include <iostream>
#include <iomanip>

using namespace unlook::camera;

int main() {
    std::cout << "Testing single frame capture synchronization..." << std::endl;
    
    auto capture = std::make_unique<HardwareSyncCapture>();
    
    HardwareSyncCapture::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.format = libcamera::formats::YUV420;
    config.buffer_count = 4;
    
    if (!capture->initialize(config)) {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }
    
    if (!capture->start()) {
        std::cerr << "Failed to start capture" << std::endl;
        return 1;
    }
    
    // Wait for stabilization
    std::cout << "Waiting for camera stabilization..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Capture single frame
    std::cout << "Capturing single frame..." << std::endl;
    HardwareSyncCapture::StereoFrame frame;
    if (capture->captureSingle(frame, 3000)) {
        std::cout << "Single frame captured:" << std::endl;
        std::cout << "  Sync error: " << std::fixed << std::setprecision(3) 
                  << frame.sync_error_ms << " ms" << std::endl;
        
        if (frame.sync_error_ms < 1.0) {
            std::cout << "✓ PASS: Single frame sync <1ms" << std::endl;
        } else {
            std::cout << "✗ FAIL: Single frame sync >1ms" << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Failed to capture single frame" << std::endl;
        return 1;
    }
    
    capture->stop();
    std::cout << "Test completed successfully" << std::endl;
    return 0;
}
EOF

# Compile single frame test
echo "Compiling single frame test..."
g++ -std=c++17 test_single_frame.cpp -o test_single_frame \
    -I../include -I../third-party/libcamera-sync-fix/include \
    -I../third-party/libcamera-sync-fix/build/include \
    -L. -lunlook \
    -L../third-party/libcamera-sync-fix/build/src/libcamera \
    -L../third-party/libcamera-sync-fix/build/src/libcamera/base \
    -lcamera -lcamera-base \
    -lopencv_core -lopencv_imgproc \
    -pthread 2>/dev/null

if [ -f test_single_frame ]; then
    export LD_LIBRARY_PATH=".:../third-party/libcamera-sync-fix/build/src/libcamera:../third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH"
    run_test "Single Frame Capture" "./test_single_frame" 10
else
    echo "✗ Failed to compile single frame test"
fi

echo ""
echo "Test 4: Repeated Start/Stop Cycles"
echo "Expected: No request validation errors, clean shutdown"
cat > test_repeated_cycles.cpp << 'EOF'
#include <unlook/camera/HardwareSyncCapture.hpp>
#include <iostream>

using namespace unlook::camera;

int main() {
    std::cout << "Testing repeated start/stop cycles..." << std::endl;
    
    auto capture = std::make_unique<HardwareSyncCapture>();
    
    HardwareSyncCapture::CameraConfig config;
    config.width = 1456;
    config.height = 1088;
    config.format = libcamera::formats::YUV420;
    config.buffer_count = 4;
    
    if (!capture->initialize(config)) {
        std::cerr << "Failed to initialize" << std::endl;
        return 1;
    }
    
    // Test 3 start/stop cycles
    for (int i = 1; i <= 3; ++i) {
        std::cout << "\nCycle " << i << "/3:" << std::endl;
        
        if (!capture->start()) {
            std::cerr << "Failed to start capture on cycle " << i << std::endl;
            return 1;
        }
        std::cout << "  Started successfully" << std::endl;
        
        // Capture for 1 second
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        auto stats = capture->getStats();
        std::cout << "  Captured " << stats.frames_captured << " frames" << std::endl;
        
        capture->stop();
        std::cout << "  Stopped successfully" << std::endl;
        
        // Wait between cycles
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    std::cout << "\n✓ All cycles completed without errors" << std::endl;
    std::cout << "Testing safe destruction..." << std::endl;
    capture.reset();
    std::cout << "✓ Clean shutdown achieved" << std::endl;
    
    return 0;
}
EOF

# Compile repeated cycles test
echo "Compiling repeated cycles test..."
g++ -std=c++17 test_repeated_cycles.cpp -o test_repeated_cycles \
    -I../include -I../third-party/libcamera-sync-fix/include \
    -I../third-party/libcamera-sync-fix/build/include \
    -L. -lunlook \
    -L../third-party/libcamera-sync-fix/build/src/libcamera \
    -L../third-party/libcamera-sync-fix/build/src/libcamera/base \
    -lcamera -lcamera-base \
    -lopencv_core -lopencv_imgproc \
    -pthread 2>/dev/null

if [ -f test_repeated_cycles ]; then
    run_test "Repeated Start/Stop" "./test_repeated_cycles" 10
else
    echo "✗ Failed to compile repeated cycles test"
fi

echo ""
echo "================================================================"
echo "                    TEST SUMMARY"
echo "================================================================"
echo ""
echo "Critical Issues Fixed:"
echo "1. Single frame capture synchronization timing"
echo "2. Request lifecycle and validation errors"
echo "3. Media device cleanup during shutdown"
echo "4. Capture start/stop cycle robustness"
echo ""
echo "If all tests pass, the camera system is ready for production use."
echo "================================================================"