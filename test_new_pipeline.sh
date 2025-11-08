#!/bin/bash
# Test script for NEW backend pipeline

echo "Testing NEW backend pipeline..."

# Create test directory
TEST_DIR="/home/alessandro/unlook_debug/test_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"

echo "Test output will be saved to: $TEST_DIR"

# Run the scanner with debug output
export UNLOOK_DEBUG_DIR="$TEST_DIR"
export UNLOOK_DEBUG_LEVEL=2

# Launch unlook (you'll need to manually test)
echo "Please run: unlook"
echo "Then:"
echo "1. Go to Handheld Scan tab"
echo "2. Click START"
echo "3. Check for progress updates"
echo "4. Verify debug images in $TEST_DIR"
