cmake_minimum_required(VERSION 3.16)
project(test_offline_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d stereo ximgproc)

# Include directories from main project
include_directories(
    ${CMAKE_SOURCE_DIR}  # For local headers (reference_sgm_census.hpp)
    ${CMAKE_SOURCE_DIR}/../include
    ${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/include
    ${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/include
    ${OpenCV_INCLUDE_DIRS}
)

# Link directories
link_directories(
    ${CMAKE_SOURCE_DIR}/../build/src
    ${CMAKE_SOURCE_DIR}/../build/src/calibration
    ${CMAKE_SOURCE_DIR}/../build/src/stereo
    ${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera
    ${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base
)

# DISABLED: test_offline_scan.cpp removed/archived
# add_executable(test_offline_scan
#     test_offline_scan.cpp
#     opencv_census_matcher.cpp
#     hybrid_census_matcher.cpp
# )
#
# target_link_libraries(test_offline_scan
#     unlook
#     unlook_calibration
#     unlook_stereo
#     ${OpenCV_LIBS}
#     pthread
#     stdc++fs
# )
#
# set_target_properties(test_offline_scan PROPERTIES
#     BUILD_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
#     INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
# )

# Create test executable for Census variants comparison
add_executable(test_census_variants
    test_census_variants.cpp
    opencv_census_matcher.cpp
    hybrid_census_matcher.cpp
)

# Link libraries for census variants test (needs calibration + stereo for rectification)
target_link_libraries(test_census_variants
    unlook_calibration          # Calibration loading
    unlook_stereo              # Rectification engine
    unlook_core                # Logger
    ${OpenCV_LIBS}
    pthread
    stdc++fs
)

# Set RPATH for census variants test
set_target_properties(test_census_variants PROPERTIES
    BUILD_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../build/src/calibration:${CMAKE_SOURCE_DIR}/../build/src/stereo:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../build/src/calibration:${CMAKE_SOURCE_DIR}/../build/src/stereo:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
)

# ============================================================================
# REFERENCE SGM-CENSUS IMPLEMENTATION
# Purpose: Ground truth stereo matching for calibration validation
# Based on verified epiception/SGM-Census repository
# ============================================================================
add_executable(test_reference_sgm
    test_reference_sgm.cpp
    reference_sgm_census.cpp
)

# Reference SGM test is STANDALONE - no Unlook dependencies
# Loads calibration directly from YAML and rectifies with pure OpenCV
target_link_libraries(test_reference_sgm
    ${OpenCV_LIBS}
    pthread
)

set_target_properties(test_reference_sgm PROPERTIES
    BUILD_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../build/src/calibration:${CMAKE_SOURCE_DIR}/../build/src/stereo:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../build/src/calibration:${CMAKE_SOURCE_DIR}/../build/src/stereo:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
)

message(STATUS "Test offline scan configured")
message(STATUS "  - OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  - Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  - Executables: test_census_variants, test_reference_sgm")
