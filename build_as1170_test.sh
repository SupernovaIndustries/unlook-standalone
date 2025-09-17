#!/bin/bash

echo "=== Building AS1170 Communication Test ==="

# Set build directory
BUILD_DIR="build"

# Create build directory if it doesn't exist
mkdir -p $BUILD_DIR

# Build the test
cd $BUILD_DIR

# Compile the AS1170 communication test
echo "Compiling AS1170 communication test..."

g++ -std=c++17 -O2 \
    -I../include \
    -I../third-party/libcamera-sync-fix/include \
    -L./src \
    -L../third-party/libcamera-sync-fix/build/src/libcamera \
    -L../third-party/libcamera-sync-fix/build/src/libcamera/base \
    ../test_as1170_communication.cpp \
    -lunlook \
    -lopencv_core \
    -lopencv_imgproc \
    -lopencv_imgcodecs \
    -li2c \
    -lpthread \
    -o test_as1170_communication

if [ $? -eq 0 ]; then
    echo "✓ AS1170 communication test compiled successfully!"
    echo ""
    echo "To run the test:"
    echo "cd build"
    echo "LD_LIBRARY_PATH=src:../third-party/libcamera-sync-fix/build/src/libcamera:../third-party/libcamera-sync-fix/build/src/libcamera/base:\$LD_LIBRARY_PATH ./test_as1170_communication"
else
    echo "✗ Compilation failed!"
    exit 1
fi