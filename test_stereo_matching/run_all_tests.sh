#!/bin/bash
# Run all stereo matching tests and compare results

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "================================================================================"
echo "UNLOOK STEREO MATCHING COMPARISON TEST SUITE"
echo "================================================================================"
echo ""

# Make scripts executable
chmod +x test_sgbm_standard.py
chmod +x test_census_binary.py
chmod +x test_sgm_census_python.py

# Test 1: SGBM Standard
echo "================================================================================"
echo "TEST 1: OpenCV SGBM Standard"
echo "================================================================================"
if python3 test_sgbm_standard.py; then
    echo "✓ SGBM Standard test completed successfully"
else
    echo "✗ SGBM Standard test failed"
fi
echo ""

# Test 2: Census Binary SGBM (opencv_contrib)
echo "================================================================================"
echo "TEST 2: StereoBinarySGBM (Census-based, opencv_contrib)"
echo "================================================================================"
if python3 test_census_binary.py; then
    echo "✓ Census Binary SGBM test completed successfully"
else
    echo "✗ Census Binary SGBM test failed (may require opencv-contrib-python)"
fi
echo ""

# Test 3: SGM-Census Python (Unlook algorithm)
echo "================================================================================"
echo "TEST 3: SGM-Census Python (Unlook algorithm recreation)"
echo "================================================================================"
if python3 test_sgm_census_python.py; then
    echo "✓ SGM-Census Python test completed successfully"
else
    echo "✗ SGM-Census Python test failed"
fi
echo ""

# Summary
echo "================================================================================"
echo "TEST SUITE COMPLETED"
echo "================================================================================"
echo ""
echo "Generated files:"
echo "  - rectified_left_sgbm.png, rectified_right_sgbm.png"
echo "  - disparity_sgbm_standard.png"
echo "  - disparity_census_binary.png (if opencv_contrib available)"
echo "  - disparity_sgm_census_python.png"
echo "  - point_cloud_sgbm_standard.ply"
echo "  - point_cloud_census_binary.ply (if opencv_contrib available)"
echo "  - point_cloud_sgm_census_python.ply"
echo ""
echo "View PLY files with: meshlab <filename>.ply"
echo "Compare disparity maps visually or run compare_results.py"
echo ""
