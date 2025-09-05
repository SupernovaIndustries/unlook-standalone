#!/bin/bash

# Unlook 3D Scanner - Build Script
# Builds the camera system with automatic libcamera-sync detection

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
BUILD_TYPE="Release"
BUILD_DIR="build"
JOBS=$(nproc)
CLEAN_BUILD=false
INSTALL=false
VERBOSE=false

# Print usage
usage() {
    echo "Usage: $0 [options]"
    echo "Options:"
    echo "  -h, --help          Show this help message"
    echo "  -t, --type TYPE     Build type (Debug/Release/RelWithDebInfo) [default: Release]"
    echo "  -j, --jobs N        Number of parallel jobs [default: $(nproc)]"
    echo "  -c, --clean         Clean build directory before building"
    echo "  -i, --install       Install after building"
    echo "  -v, --verbose       Verbose build output"
    echo "  --deps              Build all dependencies (libcamera-sync-fix, etc.)
  --cross ARCH        Cross-compile for architecture (rpi4, rpi5, cm5)
  --package           Create installation package after build"
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            usage
            ;;
        -t|--type)
            BUILD_TYPE="$2"
            shift 2
            ;;
        -j|--jobs)
            JOBS="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -i|--install)
            INSTALL=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        --deps)
            BUILD_DEPS=true
            shift
            ;;
        --cross)
            CROSS_ARCH="$2"
            shift 2
            ;;
        --package)
            CREATE_PACKAGE=true
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            usage
            ;;
    esac
done

echo -e "${GREEN}=== Unlook 3D Scanner Build ===${NC}"
echo "Build type: $BUILD_TYPE"
echo "Parallel jobs: $JOBS"

# Handle cross-compilation setup
if [ -n "$CROSS_ARCH" ]; then
    echo "Cross-compilation target: $CROSS_ARCH"
    case "$CROSS_ARCH" in
        rpi4|cm4)
            echo "Setting up for Raspberry Pi 4/CM4..."
            CMAKE_CROSS_ARGS="-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY"
            TARGET_ARCH="cortex-a72"
            ;;
        rpi5|cm5)
            echo "Setting up for Raspberry Pi 5/CM5 with ARM Cortex-A76..."
            CMAKE_CROSS_ARGS="-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=aarch64"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_PROGRAM=NEVER"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=ONLY"
            CMAKE_CROSS_ARGS="$CMAKE_CROSS_ARGS -DTARGET_ARM_ARCH=cortex-a76"
            TARGET_ARCH="cortex-a76"
            ;;
        *)
            echo -e "${RED}Unsupported cross-compilation target: $CROSS_ARCH${NC}"
            echo "Supported targets: rpi4, cm4, rpi5, cm5"
            exit 1
            ;;
    esac
fi

# Check for required third-party libcamera-sync-fix build
echo -e "\n${YELLOW}Checking for required libcamera-sync-fix...${NC}"
if [ -f "third-party/libcamera-sync-fix/build/src/libcamera/libcamera.so.0.5.1" ]; then
    echo -e "${GREEN}Found third-party libcamera-sync-fix build${NC}"
else
    echo -e "${YELLOW}Building required third-party libcamera-sync-fix...${NC}"
    
    # Check if configure script exists
    if [ -f "third-party/libcamera-sync-fix/configure_sync_build.sh" ]; then
        cd third-party/libcamera-sync-fix
        ./configure_sync_build.sh
        echo -e "${YELLOW}Building libcamera-sync-fix with ninja...${NC}"
        cd build
        ninja -j$JOBS
        cd ../../..
        echo -e "${GREEN}libcamera-sync-fix build complete${NC}"
    else
        echo -e "${RED}ERROR: configure_sync_build.sh not found!${NC}"
        echo -e "${RED}Please ensure third-party/libcamera-sync-fix is properly set up${NC}"
        exit 1
    fi
fi

# Verify the build is complete
if [ ! -f "third-party/libcamera-sync-fix/build/src/libcamera/libcamera.so.0.5.1" ]; then
    echo -e "${RED}ERROR: libcamera-sync-fix build failed!${NC}"
    echo -e "${RED}This project requires the custom libcamera-sync-fix version for hardware synchronization.${NC}"
    exit 1
fi

# Clean build directory if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "\n${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
fi

# Create build directory
if [ ! -d "$BUILD_DIR" ]; then
    mkdir -p "$BUILD_DIR"
fi

# Configure with CMake
echo -e "\n${YELLOW}Configuring with CMake...${NC}"
cd "$BUILD_DIR"

CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"

if [ "$VERBOSE" = true ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_VERBOSE_MAKEFILE=ON"
fi

# Add cross-compilation arguments if specified
if [ -n "$CMAKE_CROSS_ARGS" ]; then
    CMAKE_ARGS="$CMAKE_ARGS $CMAKE_CROSS_ARGS"
    echo -e "${YELLOW}Cross-compilation arguments: $CMAKE_CROSS_ARGS${NC}"
fi

# CM5 specific optimizations
if [ -n "$TARGET_ARCH" ] && [ "$TARGET_ARCH" = "cortex-a76" ]; then
    CMAKE_ARGS="$CMAKE_ARGS -DENABLE_CM5_OPTIMIZATIONS=ON"
    echo -e "${YELLOW}Enabling CM5 Cortex-A76 optimizations${NC}"
fi

cmake .. $CMAKE_ARGS

# Build
echo -e "\n${YELLOW}Building...${NC}"
if [ "$VERBOSE" = true ]; then
    make -j$JOBS VERBOSE=1
else
    make -j$JOBS
fi

echo -e "${GREEN}Build completed successfully!${NC}"

# Install if requested
if [ "$INSTALL" = true ]; then
    echo -e "\n${YELLOW}Installing...${NC}"
    sudo make install
    echo -e "${GREEN}Installation completed!${NC}"
fi

# Create installation package if requested
if [ "$CREATE_PACKAGE" = true ]; then
    echo -e "\n${YELLOW}Creating installation package...${NC}"
    
    # Add CPack configuration to create packages
    echo "Creating DEB and TGZ packages..."
    make package
    
    if [ -n "$CROSS_ARCH" ]; then
        PACKAGE_SUFFIX="_${CROSS_ARCH}"
    else
        PACKAGE_SUFFIX="_$(uname -m)"
    fi
    
    # Rename packages with architecture suffix
    for pkg in *.deb *.tar.gz *.rpm 2>/dev/null; do
        if [ -f "$pkg" ]; then
            base_name=$(basename "$pkg" | cut -d. -f1)
            extension=$(echo "$pkg" | cut -d. -f2-)
            mv "$pkg" "${base_name}${PACKAGE_SUFFIX}.${extension}"
        fi
    done
    
    echo -e "${GREEN}Packages created:${NC}"
    ls -la *${PACKAGE_SUFFIX}.* 2>/dev/null || echo "No packages found"
fi

# Print executable locations and usage
echo -e "\n${GREEN}=== Build Complete ===${NC}"
if [ -n "$CROSS_ARCH" ]; then
    echo "Cross-compiled for: $CROSS_ARCH ($TARGET_ARCH)"
fi
echo "Executables are in: $PWD/bin/"
echo ""
echo "Main applications:"
echo "  ./bin/unlook_scanner_gui      # Main touch interface"
echo "  ./bin/stereo_camera_example   # Basic stereo example"
echo ""
echo "Testing and validation:"
echo "  ./bin/test_camera_sync        # Camera synchronization test"
echo "  ./bin/test_sync_precision     # Timing precision validation"
echo "  ./bin/test_boofcv_calibration # Calibration system test"
echo ""
echo "Hardware tests (requires IMX296 cameras):"
echo "  ./test_synchronized_capture.sh # Full system validation"
echo ""
echo "Mock mode (no hardware required):"
echo "  ./bin/unlook_scanner_gui --mock"

cd ..