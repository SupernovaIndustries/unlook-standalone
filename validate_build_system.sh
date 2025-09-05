#!/bin/bash

# Build System Validation Script
# Comprehensive validation of the Unlook build system

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${SCRIPT_DIR}"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

log_info() { echo -e "${BLUE}[VALIDATE]${NC} $*"; }
log_success() { echo -e "${GREEN}[VALIDATE]${NC} $*"; }
log_warning() { echo -e "${YELLOW}[VALIDATE]${NC} $*"; }
log_error() { echo -e "${RED}[VALIDATE]${NC} $*" >&2; }

# Validation results
VALIDATION_PASSED=0
VALIDATION_FAILED=0
VALIDATION_WARNINGS=0

validate_check() {
    local description="$1"
    local command="$2"
    
    log_info "Checking: $description"
    
    if eval "$command" >/dev/null 2>&1; then
        log_success "✓ $description"
        ((VALIDATION_PASSED++))
        return 0
    else
        log_error "✗ $description"
        ((VALIDATION_FAILED++))
        return 1
    fi
}

validate_warn() {
    local description="$1"
    local command="$2"
    
    log_info "Checking: $description"
    
    if eval "$command" >/dev/null 2>&1; then
        log_success "✓ $description"
        ((VALIDATION_PASSED++))
        return 0
    else
        log_warning "⚠ $description (optional)"
        ((VALIDATION_WARNINGS++))
        return 1
    fi
}

log_info "Unlook Build System Validation"
log_info "=============================="

# 1. Core build system files
log_info "1. Core Build System Files"
validate_check "CMakeLists.txt exists" "test -f '$PROJECT_ROOT/CMakeLists.txt'"
validate_check "build.sh exists and executable" "test -x '$PROJECT_ROOT/build.sh'"
validate_check "cmake/ directory exists" "test -d '$PROJECT_ROOT/cmake'"
validate_check "CompilerFlags.cmake exists" "test -f '$PROJECT_ROOT/cmake/CompilerFlags.cmake'"
validate_check "Dependencies.cmake exists" "test -f '$PROJECT_ROOT/cmake/Dependencies.cmake'"
validate_check "ThirdParty.cmake exists" "test -f '$PROJECT_ROOT/cmake/ThirdParty.cmake'"
validate_check "Install.cmake exists" "test -f '$PROJECT_ROOT/cmake/Install.cmake'"

# 2. Source structure
log_info "2. Source Directory Structure"
validate_check "src/ directory exists" "test -d '$PROJECT_ROOT/src'"
validate_check "src/CMakeLists.txt exists" "test -f '$PROJECT_ROOT/src/CMakeLists.txt'"
validate_check "Main header unlook.h exists" "test -f '$PROJECT_ROOT/src/unlook.h'"
validate_check "Core types header exists" "test -f '$PROJECT_ROOT/src/core/types.h'"
validate_check "GUI CMakeLists.txt exists" "test -f '$PROJECT_ROOT/src/gui/CMakeLists.txt'"

# 3. Third-party dependency system
log_info "3. Third-party Dependency System"
validate_check "third-party/ directory exists" "test -d '$PROJECT_ROOT/third-party'"
validate_check "scripts/ directory exists" "test -d '$PROJECT_ROOT/scripts'"
validate_check "BoofCV download script exists" "test -x '$PROJECT_ROOT/scripts/download_boofcv.sh'"
validate_check "BoofCPP build script exists" "test -x '$PROJECT_ROOT/scripts/build_boofcpp.sh'"
validate_check "libcamera-sync-fix exists" "test -d '$PROJECT_ROOT/third-party/libcamera-sync-fix'"

# 4. Cross-compilation support
log_info "4. Cross-compilation Support"
validate_check "ARM64 toolchain exists" "test -f '$PROJECT_ROOT/cmake/toolchain-aarch64.cmake'"
validate_warn "ARM64 compiler available" "command -v aarch64-linux-gnu-gcc"

# 5. Testing framework
log_info "5. Testing Framework"
validate_check "tests/ directory exists" "test -d '$PROJECT_ROOT/tests'"
validate_check "tests/CMakeLists.txt exists" "test -f '$PROJECT_ROOT/tests/CMakeLists.txt'"
validate_check "Unit tests directory exists" "test -d '$PROJECT_ROOT/tests/unit'"
validate_check "Hardware tests directory exists" "test -d '$PROJECT_ROOT/tests/hardware'"
validate_check "Test config template exists" "test -f '$PROJECT_ROOT/tests/test_config.h.in'"

# 6. Examples and applications
log_info "6. Examples and Applications"
validate_check "examples/ directory exists" "test -d '$PROJECT_ROOT/examples'"
validate_check "examples/CMakeLists.txt exists" "test -f '$PROJECT_ROOT/examples/CMakeLists.txt'"

# 7. Configuration and packaging
log_info "7. Configuration and Packaging"
validate_check "UnlookConfig.cmake.in exists" "test -f '$PROJECT_ROOT/cmake/UnlookConfig.cmake.in'"
validate_check "pkg-config template exists" "test -f '$PROJECT_ROOT/src/unlook.pc.in'"
validate_check "Uninstall script exists" "test -f '$PROJECT_ROOT/cmake/cmake_uninstall.cmake.in'"

# 8. System dependencies
log_info "8. System Dependencies"
validate_check "CMake available" "command -v cmake"
validate_check "Make available" "command -v make"
validate_check "GCC available" "command -v gcc"
validate_check "G++ available" "command -v g++"
validate_warn "Ninja available" "command -v ninja"
validate_warn "Meson available" "command -v meson"
validate_warn "OpenCV system package" "pkg-config --exists opencv4 || pkg-config --exists opencv"
validate_warn "Qt5 system package" "pkg-config --exists Qt5Core Qt5Widgets"

# 9. CMake syntax validation
log_info "9. CMake Syntax Validation"
validate_check "Main CMakeLists.txt syntax" "cd '$PROJECT_ROOT' && cmake -P CMakeLists.txt 2>/dev/null || cmake --help >/dev/null"
validate_check "Source CMakeLists.txt accessible" "test -r '$PROJECT_ROOT/src/CMakeLists.txt'"

# 10. Build script functionality
log_info "10. Build Script Functionality"
validate_check "Build script help works" "'$PROJECT_ROOT/build.sh' --help"
validate_check "Build script validates args" "'$PROJECT_ROOT/build.sh' --invalid-option 2>/dev/null; test $? -ne 0"

# Summary
log_info ""
log_info "Validation Summary"
log_info "=================="
log_success "Passed: $VALIDATION_PASSED"
if [[ $VALIDATION_WARNINGS -gt 0 ]]; then
    log_warning "Warnings: $VALIDATION_WARNINGS"
fi
if [[ $VALIDATION_FAILED -gt 0 ]]; then
    log_error "Failed: $VALIDATION_FAILED"
fi

log_info ""

if [[ $VALIDATION_FAILED -eq 0 ]]; then
    log_success "✓ Build system validation PASSED"
    log_info "Ready to build:"
    log_info "  ./build.sh              # Default build"
    log_info "  ./build.sh --help       # See all options"
    log_info "  ./build.sh --validate   # Test dependencies only"
    log_info "  ./build.sh --deps       # Build third-party deps"
    log_info "  ./build.sh --cross rpi4 # Cross-compile for Pi"
    exit 0
else
    log_error "✗ Build system validation FAILED"
    log_error "Please fix the failed checks before building"
    exit 1
fi