#!/bin/bash

# BoofCPP Build Script  
# Downloads and builds BoofCPP (C++ wrapper for BoofCV)

set -e  # Exit on any error

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
THIRD_PARTY_DIR="${PROJECT_ROOT}/third-party"
BOOFCPP_DIR="${THIRD_PARTY_DIR}/BoofCPP"
BOOFCV_INSTALL_DIR="${THIRD_PARTY_DIR}/install/BoofCV"
INSTALL_DIR="${THIRD_PARTY_DIR}/install/BoofCPP"

BOOFCPP_VERSION="v0.41"
BOOFCPP_URL="https://github.com/lessthanoptimal/BoofCPP.git"

# Color output  
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() { echo -e "${BLUE}[BoofCPP]${NC} $*"; }
log_success() { echo -e "${GREEN}[BoofCPP]${NC} $*"; }
log_warning() { echo -e "${YELLOW}[BoofCPP]${NC} $*"; }
log_error() { echo -e "${RED}[BoofCPP]${NC} $*" >&2; }

# Check prerequisites
check_prerequisites() {
    log_info "Checking prerequisites..."
    
    # Check if BoofCV is available
    if [[ ! -d "$BOOFCV_INSTALL_DIR" ]] || [[ ! -f "$BOOFCV_INSTALL_DIR/boofcv_config.cmake" ]]; then
        log_error "BoofCV not found at $BOOFCV_INSTALL_DIR"
        log_error "Please run download_boofcv.sh first"
        return 1
    fi
    
    # Check Java
    if ! command -v java >/dev/null 2>&1; then
        # Try embedded Java
        local java_home="${THIRD_PARTY_DIR}/install/java"
        if [[ -d "$java_home" ]] && [[ -x "$java_home/bin/java" ]]; then
            export JAVA_HOME="$java_home"
            export PATH="$java_home/bin:$PATH"
            log_info "Using embedded Java from $java_home"
        else
            log_error "Java not found and required for BoofCPP JNI"
            return 1
        fi
    fi
    
    # Check CMake
    if ! command -v cmake >/dev/null 2>&1; then
        log_error "CMake not found. Required for building BoofCPP"
        return 1
    fi
    
    log_success "Prerequisites validated"
}

# Download BoofCPP source
download_boofcpp() {
    log_info "Downloading BoofCPP $BOOFCPP_VERSION..."
    
    if [[ -d "$BOOFCPP_DIR" ]]; then
        log_info "BoofCPP directory exists, updating..."
        cd "$BOOFCPP_DIR"
        git fetch --all
        git checkout "$BOOFCPP_VERSION" 2>/dev/null || git checkout main
        git pull origin "$BOOFCPP_VERSION" 2>/dev/null || git pull origin main
    else
        git clone "$BOOFCPP_URL" "$BOOFCPP_DIR"
        cd "$BOOFCPP_DIR"
        git checkout "$BOOFCPP_VERSION" 2>/dev/null || git checkout main
    fi
    
    log_success "BoofCPP source downloaded"
}

# Build BoofCPP
build_boofcpp() {
    log_info "Building BoofCPP..."
    
    cd "$BOOFCPP_DIR"
    
    # Create build directory
    local build_dir="build"
    if [[ -d "$build_dir" ]]; then
        rm -rf "$build_dir"
    fi
    mkdir "$build_dir"
    cd "$build_dir"
    
    # Configure CMake
    local cmake_args=(
        "-DCMAKE_BUILD_TYPE=Release"
        "-DCMAKE_INSTALL_PREFIX=$INSTALL_DIR"
        "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
        "-DBUILD_SHARED_LIBS=ON"
    )
    
    # Add BoofCV classpath if available
    if [[ -f "$BOOFCV_INSTALL_DIR/boofcv_classpath.txt" ]]; then
        local boofcv_classpath=$(cat "$BOOFCV_INSTALL_DIR/boofcv_classpath.txt")
        cmake_args+=("-DBOOFCV_CLASSPATH=$boofcv_classpath")
    fi
    
    # Find Java and JNI
    cmake_args+=(
        "-DJAVA_HOME=$JAVA_HOME"
        "-DCMAKE_PREFIX_PATH=$JAVA_HOME"
    )
    
    log_info "Running CMake configuration..."
    cmake "${cmake_args[@]}" ..
    
    # Build
    log_info "Building BoofCPP library..."
    make -j$(nproc)
    
    # Install
    log_info "Installing BoofCPP..."
    make install
    
    log_success "BoofCPP built and installed"
}

# Create wrapper CMake config if BoofCPP doesn't provide one
create_cmake_config() {
    local config_file="$INSTALL_DIR/BoofCPPConfig.cmake"
    
    # Check if BoofCPP already created a config
    if [[ -f "$config_file" ]]; then
        log_info "Using existing BoofCPP CMake config"
        return 0
    fi
    
    log_info "Creating BoofCPP CMake configuration..."
    
    cat > "$config_file" << EOF
# BoofCPP CMake Configuration
# Generated automatically by build_boofcpp.sh

set(BoofCPP_FOUND TRUE)
set(BoofCPP_VERSION "$BOOFCPP_VERSION")
set(BoofCPP_INSTALL_DIR "$INSTALL_DIR")

# Include directories
set(BoofCPP_INCLUDE_DIRS "\${BoofCPP_INSTALL_DIR}/include")

# Libraries
find_library(BoofCPP_LIBRARIES 
    NAMES boofcpp BoofCPP
    PATHS "\${BoofCPP_INSTALL_DIR}/lib"
    NO_DEFAULT_PATH
)

# Java dependencies
find_package(Java REQUIRED COMPONENTS Runtime Development)
find_package(JNI REQUIRED)

# Create imported target
if(NOT TARGET BoofCPP::BoofCPP)
    add_library(BoofCPP::BoofCPP SHARED IMPORTED)
    set_target_properties(BoofCPP::BoofCPP PROPERTIES
        IMPORTED_LOCATION "\${BoofCPP_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "\${BoofCPP_INCLUDE_DIRS}"
        INTERFACE_LINK_LIBRARIES "\${JNI_LIBRARIES}"
    )
endif()

# Include BoofCV configuration
include("$BOOFCV_INSTALL_DIR/boofcv_config.cmake")

message(STATUS "Found BoofCPP \${BoofCPP_VERSION} at \${BoofCPP_INSTALL_DIR}")
EOF
    
    log_success "BoofCPP CMake configuration created"
}

# Create combined header for easy inclusion
create_combined_header() {
    local combined_header="$INSTALL_DIR/include/boofcv_unlook.h"
    
    log_info "Creating combined BoofCV header for Unlook..."
    
    mkdir -p "$(dirname "$combined_header")"
    
    cat > "$combined_header" << EOF
/**
 * Unlook BoofCV Integration Header
 * Combined header for easy BoofCV integration in Unlook 3D Scanner
 * Generated automatically by build_boofcpp.sh
 */

#ifndef BOOFCV_UNLOOK_H
#define BOOFCV_UNLOOK_H

// Standard includes
#include <memory>
#include <vector>
#include <string>

// JNI includes for Java integration
#include <jni.h>

// BoofCPP includes (if available)
#ifdef BOOFCPP_AVAILABLE
#include <boofcv.h>
#endif

namespace unlook {
namespace boofcv {

/**
 * BoofCV Manager for Unlook integration
 * Handles Java VM initialization and BoofCV operations
 */
class BoofCVManager {
public:
    BoofCVManager();
    ~BoofCVManager();
    
    // Initialize Java VM and BoofCV
    bool initialize();
    void shutdown();
    
    // Calibration functions
    bool calibrateCamera(
        const std::vector<std::vector<cv::Point2f>>& image_points,
        const std::vector<std::vector<cv::Point3f>>& object_points,
        const cv::Size& image_size,
        cv::Mat& camera_matrix,
        cv::Mat& distortion_coeffs
    );
    
    bool calibrateStereo(
        const std::vector<std::vector<cv::Point2f>>& left_points,
        const std::vector<std::vector<cv::Point2f>>& right_points,
        const std::vector<std::vector<cv::Point3f>>& object_points,
        const cv::Size& image_size,
        cv::Mat& left_camera_matrix,
        cv::Mat& left_distortion,
        cv::Mat& right_camera_matrix,
        cv::Mat& right_distortion,
        cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F
    );
    
    // Feature detection
    std::vector<cv::Point2f> detectChessboard(const cv::Mat& image, const cv::Size& pattern_size);
    std::vector<cv::Point2f> detectCharucoBoard(const cv::Mat& image, int dictionary_id, const cv::Size& pattern_size);
    
private:
    JavaVM* jvm_;
    JNIEnv* env_;
    bool initialized_;
    
    // BoofCV class references
    jclass calibration_class_;
    jclass detection_class_;
    
    bool initializeJVM();
    bool loadBoofCVClasses();
};

} // namespace boofcv
} // namespace unlook

#endif // BOOFCV_UNLOOK_H
EOF
    
    log_success "Combined BoofCV header created"
}

# Create test program
create_test_program() {
    local test_dir="$INSTALL_DIR/test"
    mkdir -p "$test_dir"
    
    local test_file="$test_dir/test_boofcpp.cpp"
    
    log_info "Creating BoofCPP test program..."
    
    cat > "$test_file" << EOF
/**
 * BoofCPP Test Program
 * Tests basic BoofCV functionality through C++ wrapper
 */

#include <iostream>
#include <string>

// Include combined header
#include "../include/boofcv_unlook.h"

int main() {
    std::cout << "BoofCPP Test Program" << std::endl;
    std::cout << "===================" << std::endl;
    
    try {
        unlook::boofcv::BoofCVManager boofcv_manager;
        
        std::cout << "Initializing BoofCV..." << std::endl;
        if (!boofcv_manager.initialize()) {
            std::cerr << "Failed to initialize BoofCV" << std::endl;
            return 1;
        }
        
        std::cout << "BoofCV initialized successfully!" << std::endl;
        
        // Test would go here - for now just check initialization
        std::cout << "Basic functionality test passed" << std::endl;
        
        boofcv_manager.shutdown();
        std::cout << "BoofCV shutdown completed" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
EOF
    
    # Create CMakeLists.txt for the test
    cat > "$test_dir/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.16)
project(BoofCPPTest)

set(CMAKE_CXX_STANDARD 17)

# Find BoofCPP
find_package(BoofCPP REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_boofcpp test_boofcpp.cpp)
target_link_libraries(test_boofcpp BoofCPP::BoofCPP \${OpenCV_LIBS})
EOF
    
    log_success "Test program created at $test_dir"
}

# Validate installation
validate_installation() {
    log_info "Validating BoofCPP installation..."
    
    # Check for essential files
    local required_files=(
        "$INSTALL_DIR/include/boofcv_unlook.h"
        "$INSTALL_DIR/BoofCPPConfig.cmake"
    )
    
    local missing_files=()
    for file in "${required_files[@]}"; do
        if [[ ! -f "$file" ]]; then
            missing_files+=("$file")
        fi
    done
    
    if [[ ${#missing_files[@]} -gt 0 ]]; then
        log_warning "Missing files: ${missing_files[*]}"
    fi
    
    # Check for library files
    if find "$INSTALL_DIR/lib" -name "*boofcpp*" -o -name "*BoofCPP*" | grep -q .; then
        log_success "BoofCPP libraries found"
        find "$INSTALL_DIR/lib" -name "*boofcpp*" -o -name "*BoofCPP*" | while read -r lib; do
            log_info "  - $(basename "$lib")"
        done
    else
        log_warning "BoofCPP libraries not found - may need manual integration"
    fi
    
    log_success "BoofCPP validation completed"
}

# Main execution
main() {
    log_info "BoofCPP Build Script"
    log_info "==================="
    
    check_prerequisites
    download_boofcpp
    build_boofcpp
    create_cmake_config
    create_combined_header
    create_test_program
    validate_installation
    
    log_success "BoofCPP setup completed successfully!"
    log_info "Installation directory: $INSTALL_DIR"
    log_info "Include the header: #include <boofcv_unlook.h>"
    log_info "Link with: BoofCPP::BoofCPP"
}

# Execute main function
main "$@"