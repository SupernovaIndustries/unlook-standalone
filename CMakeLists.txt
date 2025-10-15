cmake_minimum_required(VERSION 3.16)

project(UnlookScanner 
    VERSION 1.0.0
    DESCRIPTION "Unlook 3D Scanner Professional API Library"
    LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build configuration
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "System: ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION}")

# Build options
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(BUILD_EXAMPLES "Build example applications" ON)
option(BUILD_GUI "Build Qt-based GUI applications" ON)
option(BUILD_TESTS "Build test suite" OFF)
option(ENABLE_OPENMP "Enable OpenMP support" ON)
option(USE_SYSTEM_OPENCV "Use system OpenCV installation" ON)
option(ENABLE_BOOFCV "Enable BoofCV stereo algorithms via JNI" ON)

# Compiler flags
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -Wall -Wextra -pedantic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -Wall")

# ARM64 optimizations for Raspberry Pi CM4/CM5
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "Enabling ARM64 optimizations for Raspberry Pi")
    
    # CM5 specific optimizations (Cortex-A76)
    if(ENABLE_CM5_OPTIMIZATIONS OR TARGET_ARM_ARCH STREQUAL "cortex-a76")
        message(STATUS "Enabling CM5 Cortex-A76 specific optimizations")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a+crypto+simd -mtune=cortex-a76")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a76")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon-fp-armv8")
    else()
        # Default CM4 optimizations (Cortex-A72)
        message(STATUS "Enabling CM4 Cortex-A72 optimizations")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd -mtune=cortex-a72")
    endif()
    
    # Common ARM64 optimizations
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -funsafe-math-optimizations")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftree-vectorize -ffast-math")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto=auto")  # Link Time Optimization
endif()

# Threading support
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)

# OpenMP support
if(ENABLE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        message(STATUS "OpenMP found: ${OpenMP_CXX_VERSION}")
    else()
        message(WARNING "OpenMP not found - parallel processing will be limited")
    endif()
endif()

# Find dependencies
find_package(PkgConfig REQUIRED)

# OpenCV - essential for image processing
if(USE_SYSTEM_OPENCV)
    find_package(OpenCV 4.0 REQUIRED 
        COMPONENTS core imgproc imgcodecs calib3d features2d ximgproc photo highgui)
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV modules: core imgproc imgcodecs calib3d features2d ximgproc photo highgui")
else()
    # Add custom OpenCV build here if needed
    message(FATAL_ERROR "Custom OpenCV build not implemented - use system installation")
endif()

# libcamera-sync-fix - EXCLUSIVELY use third-party version for hardware synchronization
# This ensures consistent behavior and eliminates dependency on system installations
if(EXISTS "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/src/libcamera/libcamera.so.0.5.1")
    message(STATUS "Using third-party libcamera-sync-fix for hardware synchronization")
    set(LIBCAMERA_FOUND TRUE)
    set(LIBCAMERA_INCLUDE_DIRS 
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/include"
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/include")
    set(LIBCAMERA_LIBRARY_DIRS 
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/src/libcamera"
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/src/libcamera/base")
    set(LIBCAMERA_LIBRARIES 
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/src/libcamera/libcamera.so.0.5.1"
        "${CMAKE_SOURCE_DIR}/third-party/libcamera-sync-fix/build/src/libcamera/base/libcamera-base.so.0.5.1")
    # Set runtime library paths for self-contained deployment
    set(CMAKE_INSTALL_RPATH "${LIBCAMERA_LIBRARY_DIRS}")
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    message(STATUS "  Include dirs: ${LIBCAMERA_INCLUDE_DIRS}")
    message(STATUS "  Libraries: ${LIBCAMERA_LIBRARIES}")
    message(STATUS "  Runtime paths: ${CMAKE_INSTALL_RPATH}")
else()
    # REQUIRED: libcamera-sync-fix must be built first
    message(FATAL_ERROR "libcamera-sync-fix not found! This project requires the custom libcamera-sync-fix version.")
    message(FATAL_ERROR "Please build third-party/libcamera-sync-fix first:")
    message(FATAL_ERROR "  cd third-party/libcamera-sync-fix")
    message(FATAL_ERROR "  ./configure_sync_build.sh")  
    message(FATAL_ERROR "  make -j$(nproc)")
endif()

# Java/JNI - for BoofCV integration (optional)
if(ENABLE_BOOFCV)
    find_package(Java COMPONENTS Runtime Development)
    find_package(JNI)
    
    if(Java_FOUND AND JNI_FOUND)
        message(STATUS "Java found: ${Java_VERSION}")
        message(STATUS "JNI found: ${JNI_VERSION}")
        message(STATUS "Java runtime: ${Java_JAVA_EXECUTABLE}")
        message(STATUS "Java compiler: ${Java_JAVAC_EXECUTABLE}")
        message(STATUS "JNI include dirs: ${JNI_INCLUDE_DIRS}")
        set(BOOFCV_ENABLED TRUE)
        
        # Set BoofCV JAR path (can be overridden)
        if(NOT DEFINED BOOFCV_JAR_PATH)
            set(BOOFCV_JAR_PATH "/usr/share/java/boofcv/boofcv-all.jar" CACHE STRING "Path to BoofCV JAR file")
        endif()
        
        if(EXISTS "${BOOFCV_JAR_PATH}")
            message(STATUS "BoofCV JAR found: ${BOOFCV_JAR_PATH}")
        else()
            message(WARNING "BoofCV JAR not found at ${BOOFCV_JAR_PATH} - BoofCV integration may fail at runtime")
        endif()
        
    else()
        message(WARNING "Java/JNI not found - BoofCV stereo algorithms will be disabled")
        set(BOOFCV_ENABLED FALSE)
    endif()
else()
    message(STATUS "BoofCV integration disabled")
    set(BOOFCV_ENABLED FALSE)
endif()

# Qt5 - for GUI applications (optional)
if(BUILD_GUI)
    find_package(Qt5 COMPONENTS Core Widgets OpenGL)
    if(Qt5_FOUND)
        message(STATUS "Qt5 found: ${Qt5_VERSION}")
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_AUTORCC ON)
        set(CMAKE_AUTOUIC ON)
    else()
        message(WARNING "Qt5 not found - GUI applications will not be built")
        set(BUILD_GUI OFF)
    endif()
endif()

# Open3D - for point cloud processing and mesh generation
find_package(Open3D QUIET)
if(Open3D_FOUND)
    message(STATUS "Open3D found: ${Open3D_VERSION}")
    message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")
    message(STATUS "Open3D libraries: ${Open3D_LIBRARIES}")
    set(OPEN3D_ENABLED TRUE)
    add_definitions(-DOPEN3D_ENABLED=1)

    # ARM64 optimization for Open3D
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        message(STATUS "Enabling ARM64 optimizations for Open3D")
        add_definitions(-DOPEN3D_ARM64_OPTIMIZED=1)
    endif()

else()
    message(WARNING "Open3D not found - point cloud processing and mesh generation will use fallback implementations")
    message(WARNING "To install Open3D: sudo apt install libopen3d-dev")
    set(OPEN3D_ENABLED FALSE)

    # Fallback: Check for PCL (Point Cloud Library) as alternative
    find_package(PCL QUIET COMPONENTS common io filters surface)
    if(PCL_FOUND)
        message(STATUS "PCL found as Open3D fallback: ${PCL_VERSION}")
        set(PCL_ENABLED TRUE)
        add_definitions(-DUSE_PCL_FALLBACK=1)
    else()
        message(WARNING "Neither Open3D nor PCL found - advanced point cloud features will be limited")
        set(PCL_ENABLED FALSE)
    endif()
endif()

# Filesystem library for C++17
find_library(FILESYSTEM_LIB stdc++fs)
if(FILESYSTEM_LIB)
    message(STATUS "Found std::filesystem support")
else()
    message(STATUS "Using header-only std::filesystem")
    set(FILESYSTEM_LIB "")
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

if(LIBCAMERA_INCLUDE_DIRS)
    include_directories(${LIBCAMERA_INCLUDE_DIRS})
endif()

if(OPEN3D_ENABLED AND Open3D_INCLUDE_DIRS)
    include_directories(${Open3D_INCLUDE_DIRS})
endif()

if(PCL_ENABLED)
    include_directories(${PCL_INCLUDE_DIRS})
endif()

if(BOOFCV_ENABLED AND JNI_INCLUDE_DIRS)
    include_directories(${JNI_INCLUDE_DIRS})
endif()

# Add CMake module path for custom Find*.cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Add subdirectories
add_subdirectory(src)

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Install configuration
include(GNUInstallDirs)

# Install headers
install(DIRECTORY include/unlook
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Create pkg-config file
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/unlook.pc.in
    ${CMAKE_BINARY_DIR}/unlook.pc
    @ONLY)

install(FILES ${CMAKE_BINARY_DIR}/unlook.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Create CMake config files
include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/UnlookConfig.cmake.in
    ${CMAKE_BINARY_DIR}/UnlookConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Unlook)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/UnlookConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)

install(FILES
    ${CMAKE_BINARY_DIR}/UnlookConfig.cmake
    ${CMAKE_BINARY_DIR}/UnlookConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Unlook)

# Summary
message(STATUS "==================== Build Configuration ====================")
message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "Shared libraries:     ${BUILD_SHARED_LIBS}")
message(STATUS "Examples:             ${BUILD_EXAMPLES}")
message(STATUS "GUI applications:     ${BUILD_GUI}")
message(STATUS "Tests:                ${BUILD_TESTS}")
message(STATUS "OpenMP:               ${ENABLE_OPENMP}")
message(STATUS "OpenCV version:       ${OpenCV_VERSION}")
if(LIBCAMERA_FOUND)
    message(STATUS "libcamera:            ${LIBCAMERA_VERSION}")
else()
    message(STATUS "libcamera:            Not found")
endif()
if(Qt5_FOUND AND BUILD_GUI)
    message(STATUS "Qt5 version:          ${Qt5_VERSION}")
endif()
if(BOOFCV_ENABLED)
    message(STATUS "BoofCV integration:   Enabled (Java ${Java_VERSION})")
    message(STATUS "BoofCV JAR path:      ${BOOFCV_JAR_PATH}")
else()
    message(STATUS "BoofCV integration:   Disabled")
endif()
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "==========================================================")

# Documentation target
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
    if(EXISTS "${CMAKE_SOURCE_DIR}/docs/Doxyfile.in")
        configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.in
            ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
        
        add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM)
    else()
        message(STATUS "Doxygen found but Doxyfile.in missing - skipping documentation target")
    endif()
endif()

# Hardware sync test program
add_executable(test_hardware_sync_new
    test_hardware_sync_new.cpp
)

target_link_libraries(test_hardware_sync_new
    unlook_camera
    ${OpenCV_LIBS}
    ${LIBCAMERA_LIBRARIES}
)

target_include_directories(test_hardware_sync_new PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LIBCAMERA_INCLUDE_DIRS}
)

# ==========================================================
# PACKAGING CONFIGURATION (CPack)
# ==========================================================

# Set package information
set(CPACK_PACKAGE_NAME "unlook-scanner")
set(CPACK_PACKAGE_VENDOR "Supernova Industries")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Unlook 3D Scanner - Professional Open Source 3D Scanner")
set(CPACK_PACKAGE_DESCRIPTION 
    "Industrial-grade 3D scanner with 0.005mm precision for quality control, education, and research applications.")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_CONTACT "alessandro.cursoli@supernovaindustries.com")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/SupernovaIndustries/unlook-standalone")

# License and README
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")

# Installation directory
set(CPACK_PACKAGE_INSTALL_DIRECTORY "unlook-scanner")

# DEB specific configuration
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Alessandro Cursoli <alessandro.cursoli@supernovaindustries.com>")
set(CPACK_DEBIAN_PACKAGE_SECTION "graphics")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS 
    "libopencv-dev (>= 4.0), qtbase5-dev (>= 5.12), libqt5widgets5, build-essential, cmake")
set(CPACK_DEBIAN_PACKAGE_SUGGESTS "libopencv-contrib-dev")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
endif()

# RPM specific configuration  
set(CPACK_RPM_COMPONENT_INSTALL ON)
set(CPACK_RPM_PACKAGE_LICENSE "MIT")
set(CPACK_RPM_PACKAGE_GROUP "Applications/Graphics")
set(CPACK_RPM_PACKAGE_REQUIRES "opencv-devel >= 4.0, qt5-qtbase-devel >= 5.12")
set(CPACK_RPM_PACKAGE_ARCHITECTURE "aarch64")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_RPM_PACKAGE_ARCHITECTURE "x86_64")
endif()

# TGZ archive configuration
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

# Enable CPack
include(CPack)

# Component-based packaging
cpack_add_component(applications
    DISPLAY_NAME "Unlook Scanner Applications"
    DESCRIPTION "Main GUI application and command-line tools"
    REQUIRED)

cpack_add_component(libraries  
    DISPLAY_NAME "Unlook Scanner Libraries"
    DESCRIPTION "Shared libraries and API headers"
    REQUIRED)

cpack_add_component(development
    DISPLAY_NAME "Development Files"
    DESCRIPTION "Headers and development tools for API integration")

cpack_add_component(examples
    DISPLAY_NAME "Examples and Tests"
    DESCRIPTION "Example applications and validation tests")

cpack_add_component(documentation
    DISPLAY_NAME "Documentation"  
    DESCRIPTION "API documentation and user guides")

# Default generators
set(CPACK_GENERATOR "DEB;TGZ")
if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    list(APPEND CPACK_GENERATOR "RPM")
endif()

# Architecture-specific package naming
if(ENABLE_CM5_OPTIMIZATIONS OR TARGET_ARM_ARCH STREQUAL "cortex-a76")
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-cm5-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-cm4-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
else()
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
endif()

include(CPack)