#!/bin/bash

# ========================================================
# MIGRATION SCRIPT: Transition to NEW Backend Pipeline
# ========================================================
# This script helps migrate from old VCSELStereoMatcher to
# the new modular backend with CalibrationValidation
# ========================================================

set -e

echo "================================================="
echo "UNLOOK BACKEND MIGRATION - Phase 5"
echo "================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if running from correct directory
if [ ! -f "CMakeLists.txt" ]; then
    echo -e "${RED}ERROR: Run this script from unlook-standalone root directory${NC}"
    exit 1
fi

echo -e "${BLUE}Step 1: Backing up original files...${NC}"
# Create backup directory
BACKUP_DIR="backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"

# Backup original HandheldScanPipeline
if [ -f "src/api/HandheldScanPipeline.cpp" ]; then
    cp src/api/HandheldScanPipeline.cpp "$BACKUP_DIR/HandheldScanPipeline_ORIGINAL.cpp"
    echo -e "${GREEN}✓ Backed up original HandheldScanPipeline.cpp${NC}"
fi

# Backup GUI widget
if [ -f "src/gui/handheld_scan_widget.cpp" ]; then
    cp src/gui/handheld_scan_widget.cpp "$BACKUP_DIR/handheld_scan_widget_ORIGINAL.cpp"
    echo -e "${GREEN}✓ Backed up original handheld_scan_widget.cpp${NC}"
fi

echo -e "\n${BLUE}Step 2: Checking NEW backend components...${NC}"

# Check if new components exist
COMPONENTS_OK=true

# Check CalibrationValidation
if [ -f "include/unlook/calibration/CalibrationValidation.hpp" ]; then
    echo -e "${GREEN}✓ CalibrationValidation found${NC}"
else
    echo -e "${RED}✗ CalibrationValidation missing${NC}"
    COMPONENTS_OK=false
fi

# Check RectificationEngine
if [ -f "include/unlook/stereo/RectificationEngine.hpp" ]; then
    echo -e "${GREEN}✓ RectificationEngine found${NC}"
else
    echo -e "${RED}✗ RectificationEngine missing${NC}"
    COMPONENTS_OK=false
fi

# Check DisparityComputer
if [ -f "include/unlook/stereo/DisparityComputer.hpp" ]; then
    echo -e "${GREEN}✓ DisparityComputer found${NC}"
else
    echo -e "${RED}✗ DisparityComputer missing${NC}"
    COMPONENTS_OK=false
fi

# Check DebugOutputManager
if [ -f "include/unlook/stereo/DebugOutputManager.hpp" ]; then
    echo -e "${GREEN}✓ DebugOutputManager found${NC}"
else
    echo -e "${RED}✗ DebugOutputManager missing${NC}"
    COMPONENTS_OK=false
fi

if [ "$COMPONENTS_OK" = false ]; then
    echo -e "${RED}ERROR: Some backend components are missing!${NC}"
    echo "Please ensure all Phase 1-4 components are built first."
    exit 1
fi

echo -e "\n${BLUE}Step 3: Installing NEW HandheldScanPipeline...${NC}"

# Check if NEW version exists
if [ -f "src/api/HandheldScanPipeline_NEW.cpp" ]; then
    echo -e "${YELLOW}Found HandheldScanPipeline_NEW.cpp${NC}"

    # Ask user to confirm
    read -p "Replace old pipeline with NEW backend? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        mv src/api/HandheldScanPipeline.cpp src/api/HandheldScanPipeline_OLD.cpp
        cp src/api/HandheldScanPipeline_NEW.cpp src/api/HandheldScanPipeline.cpp
        echo -e "${GREEN}✓ Installed NEW HandheldScanPipeline${NC}"
    else
        echo -e "${YELLOW}Skipping pipeline replacement${NC}"
    fi
else
    echo -e "${RED}HandheldScanPipeline_NEW.cpp not found!${NC}"
fi

echo -e "\n${BLUE}Step 4: Checking calibration files...${NC}"

# Check calibration directory
CALIB_DIR="/unlook_calib"
if [ -d "$CALIB_DIR" ]; then
    echo "Found calibration directory: $CALIB_DIR"
    ls -la "$CALIB_DIR"/*.yaml 2>/dev/null || echo "No calibration files found"
else
    echo -e "${YELLOW}Calibration directory not found at $CALIB_DIR${NC}"
    echo "Creating directory..."
    sudo mkdir -p "$CALIB_DIR"
    sudo chmod 755 "$CALIB_DIR"
fi

# Check for 1280x720 calibration
if [ -f "$CALIB_DIR/calib-20251108_171118.yaml" ]; then
    echo -e "${GREEN}✓ Found 1280x720 calibration file${NC}"
else
    echo -e "${YELLOW}⚠ 1280x720 calibration not found${NC}"
    echo "You may need to recalibrate cameras at 1280x720 resolution"
fi

echo -e "\n${BLUE}Step 5: Build configuration check...${NC}"

# Check CMakeLists.txt for new components
if grep -q "CalibrationValidation" src/calibration/CMakeLists.txt 2>/dev/null; then
    echo -e "${GREEN}✓ CalibrationValidation in build${NC}"
else
    echo -e "${YELLOW}⚠ CalibrationValidation not in CMakeLists.txt${NC}"
fi

if grep -q "RectificationEngine\|DisparityComputer\|DebugOutputManager" src/stereo/CMakeLists.txt 2>/dev/null; then
    echo -e "${GREEN}✓ New stereo components in build${NC}"
else
    echo -e "${YELLOW}⚠ New stereo components not in CMakeLists.txt${NC}"
fi

echo -e "\n${BLUE}Step 6: Creating test script...${NC}"

# Create test script
cat > test_new_pipeline.sh << 'EOF'
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
EOF

chmod +x test_new_pipeline.sh
echo -e "${GREEN}✓ Created test_new_pipeline.sh${NC}"

echo -e "\n${BLUE}Step 7: Summary and recommendations...${NC}"

echo "================================================="
echo "MIGRATION STATUS"
echo "================================================="

if [ "$COMPONENTS_OK" = true ]; then
    echo -e "${GREEN}✓ All backend components present${NC}"
else
    echo -e "${RED}✗ Some components missing${NC}"
fi

echo -e "\n${YELLOW}NEXT STEPS:${NC}"
echo "1. Review the changes in HandheldScanPipeline.cpp"
echo "2. Apply GUI integration patches from GUI_INTEGRATION_PATCH.cpp"
echo "3. Rebuild the project: ./build.sh --clean"
echo "4. Test with: ./test_new_pipeline.sh"
echo "5. Verify calibration validation works"
echo "6. Check GPU acceleration if available"

echo -e "\n${YELLOW}IMPORTANT NOTES:${NC}"
echo "• Ensure calibration files are at 1280x720 resolution"
echo "• Old VCSELStereoMatcher is replaced with DisparityComputer"
echo "• CalibrationValidation MUST pass before processing"
echo "• Debug images will be saved to /home/alessandro/unlook_debug/"

echo -e "\n${GREEN}Migration preparation complete!${NC}"
echo "Backup saved to: $BACKUP_DIR"