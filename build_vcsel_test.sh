#!/bin/bash

# Build script for VCSEL integration test

echo "========================================="
echo "  Building VCSEL Integration Test"
echo "========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if build directory exists
if [ ! -d "build" ]; then
    echo -e "${YELLOW}Build directory not found. Running main build first...${NC}"
    ./build.sh
    if [ $? -ne 0 ]; then
        echo -e "${RED}Main build failed. Please fix build errors first.${NC}"
        exit 1
    fi
fi

cd build

# Compile the test program
echo -e "${GREEN}Compiling VCSEL integration test...${NC}"
g++ -std=c++17 \
    -I../include \
    -I../third-party/libcamera-sync-fix/include \
    -L./src \
    -L./src/hardware \
    -L../third-party/libcamera-sync-fix/build/src/libcamera \
    -L../third-party/libcamera-sync-fix/build/src/libcamera/base \
    ../test_vcsel_integration.cpp \
    -o test_vcsel_integration \
    -lunlook_hardware \
    -lcamera \
    -lcamera-base \
    -lpthread \
    -latomic

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo ""
    echo "To run the test:"
    echo "  cd build"
    echo "  sudo LD_LIBRARY_PATH=src:src/hardware:../third-party/libcamera-sync-fix/build/src/libcamera:../third-party/libcamera-sync-fix/build/src/libcamera/base:\$LD_LIBRARY_PATH ./test_vcsel_integration"
    echo ""
    echo "Note: sudo is required for GPIO and I2C access"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi