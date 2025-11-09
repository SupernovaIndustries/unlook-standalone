cmake_minimum_required(VERSION 3.16)
project(test_offline_scan)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d)

# Include directories from main project
include_directories(
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

# Create executable
add_executable(test_offline_scan test_offline_scan.cpp)

# Link libraries (same as main project)
target_link_libraries(test_offline_scan
    unlook                      # Main library
    unlook_calibration          # Calibration
    unlook_stereo              # Stereo processing
    ${OpenCV_LIBS}
    pthread
    stdc++fs
)

# Set RPATH for runtime library search
set_target_properties(test_offline_scan PROPERTIES
    BUILD_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
    INSTALL_RPATH "${CMAKE_SOURCE_DIR}/../build/src:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera:${CMAKE_SOURCE_DIR}/../third-party/libcamera-sync-fix/build/src/libcamera/base"
)

message(STATUS "Test offline scan configured")
message(STATUS "  - OpenCV version: ${OpenCV_VERSION}")
message(STATUS "  - Build type: ${CMAKE_BUILD_TYPE}")
