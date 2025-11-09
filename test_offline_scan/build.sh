#!/bin/bash

# Quick build script for offline scan test

set -e

echo "=== Building test_offline_scan ==="

# Create build directory
mkdir -p build
cd build

# Configure
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j4

echo ""
echo "✅ Build complete!"
echo ""
echo "Run with: ./run.sh"
echo "   or:    cd build && ./test_offline_scan"
