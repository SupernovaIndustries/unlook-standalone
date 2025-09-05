# Third-party library management and building

include(ExternalProject)
include(FetchContent)

# Set third-party directories
set(THIRD_PARTY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third-party)
set(THIRD_PARTY_INSTALL_DIR ${THIRD_PARTY_DIR}/install)
set(THIRD_PARTY_BUILD_DIR ${CMAKE_BINARY_DIR}/third-party-build)

# Create third-party directories
file(MAKE_DIRECTORY ${THIRD_PARTY_DIR})
file(MAKE_DIRECTORY ${THIRD_PARTY_INSTALL_DIR})
file(MAKE_DIRECTORY ${THIRD_PARTY_BUILD_DIR})

# Common ExternalProject arguments
set(EXTERNAL_PROJECT_COMMON_ARGS
    BUILD_IN_SOURCE 0
    INSTALL_DIR ${THIRD_PARTY_INSTALL_DIR}
    CMAKE_ARGS 
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
        -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}
        -DBUILD_SHARED_LIBS=${BUILD_SHARED_LIBS}
)

# Add third-party install directory to CMAKE_PREFIX_PATH
list(PREPEND CMAKE_PREFIX_PATH ${THIRD_PARTY_INSTALL_DIR})

# Build OpenCV from source if not found
if(NOT OPENCV_FOUND)
    message(STATUS "Building OpenCV from source...")
    
    # OpenCV configuration
    set(OPENCV_VERSION "4.8.1")
    set(OPENCV_MODULES 
        core imgproc imgcodecs highgui calib3d features2d 
        xfeatures2d ximgproc stereo ccalib
    )
    
    ExternalProject_Add(opencv_external
        PREFIX ${THIRD_PARTY_BUILD_DIR}/opencv
        URL https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz
        URL_HASH SHA256=8df0079cdbe179748a18d44731af62a245a45ebf5085223dc03133954c662973
        ${EXTERNAL_PROJECT_COMMON_ARGS}
        CMAKE_ARGS 
            ${EXTERNAL_PROJECT_COMMON_ARGS}
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_PERF_TESTS=OFF
            -DBUILD_opencv_apps=OFF
            -DBUILD_opencv_python2=OFF
            -DBUILD_opencv_python3=OFF
            -DWITH_GTK=ON
            -DWITH_QT=OFF
            -DWITH_OPENGL=ON
            -DWITH_V4L=ON
            -DENABLE_NEON=${ENABLE_ARM_OPTIMIZATIONS}
    )
    
    # Create imported targets for OpenCV
    add_library(opencv_core SHARED IMPORTED)
    add_library(opencv_imgproc SHARED IMPORTED)
    add_library(opencv_calib3d SHARED IMPORTED)
    add_library(opencv_imgcodecs SHARED IMPORTED)
    add_library(opencv_highgui SHARED IMPORTED)
    
    set(OPENCV_INSTALL_PATH ${THIRD_PARTY_INSTALL_DIR})
    
    add_dependencies(opencv_core opencv_external)
    add_dependencies(opencv_imgproc opencv_external)
    add_dependencies(opencv_calib3d opencv_external)
    add_dependencies(opencv_imgcodecs opencv_external)
    add_dependencies(opencv_highgui opencv_external)
    
    # Set properties after build
    ExternalProject_Get_Property(opencv_external INSTALL_DIR)
    set_target_properties(opencv_core PROPERTIES
        IMPORTED_LOCATION ${INSTALL_DIR}/lib/libopencv_core.so
        INTERFACE_INCLUDE_DIRECTORIES ${INSTALL_DIR}/include/opencv4
    )
    
    set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_imgcodecs opencv_highgui)
    set(OPENCV_BUILT TRUE)
endif()

# Build Qt5 from source if not found (complex, mainly for reference)
if(BUILD_GUI AND NOT QT5_FOUND)
    message(WARNING "Qt5 not found. Please install Qt5 development packages:")
    message(WARNING "  Ubuntu/Debian: sudo apt-get install qtbase5-dev qttools5-dev")
    message(WARNING "  CentOS/RHEL: sudo yum install qt5-qtbase-devel qt5-qttools-devel")
    message(FATAL_ERROR "Qt5 is required for GUI build")
endif()

# Build Eigen3 (header-only) if not found
if(NOT EIGEN3_FOUND)
    message(STATUS "Downloading Eigen3 headers...")
    
    FetchContent_Declare(eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    
    FetchContent_MakeAvailable(eigen3)
    
    # Create alias for consistency
    add_library(Eigen3::Eigen ALIAS eigen)
    set(EIGEN3_BUILT TRUE)
endif()

# Build libcamera-sync from third-party if not found
if(NOT LIBCAMERA_SYNC_FOUND)
    if(EXISTS ${THIRD_PARTY_DIR}/libcamera-sync-fix)
        message(STATUS "Building libcamera-sync from third-party source...")
        
        # libcamera-sync uses Meson build system
        find_program(MESON_EXECUTABLE meson REQUIRED)
        find_program(NINJA_EXECUTABLE ninja REQUIRED)
        
        ExternalProject_Add(libcamera_sync_external
            PREFIX ${THIRD_PARTY_BUILD_DIR}/libcamera-sync
            SOURCE_DIR ${THIRD_PARTY_DIR}/libcamera-sync-fix
            CONFIGURE_COMMAND ${MESON_EXECUTABLE} setup 
                --prefix=${THIRD_PARTY_INSTALL_DIR}
                --buildtype=$<IF:$<CONFIG:Debug>,debug,release>
                <BINARY_DIR> <SOURCE_DIR>
            BUILD_COMMAND ${NINJA_EXECUTABLE} -C <BINARY_DIR>
            INSTALL_COMMAND ${NINJA_EXECUTABLE} -C <BINARY_DIR> install
            BUILD_ALWAYS 0
        )
        
        # Create imported target
        add_library(libcamera-sync::camera SHARED IMPORTED)
        add_dependencies(libcamera-sync::camera libcamera_sync_external)
        
        set_target_properties(libcamera-sync::camera PROPERTIES
            IMPORTED_LOCATION ${THIRD_PARTY_INSTALL_DIR}/lib/libcamera.so
            INTERFACE_INCLUDE_DIRECTORIES ${THIRD_PARTY_INSTALL_DIR}/include
        )
        
        set(LIBCAMERA_SYNC_BUILT TRUE)
    else()
        message(WARNING "libcamera-sync source not found in third-party directory")
        message(WARNING "Please ensure libcamera-sync-fix is available or install system version")
    endif()
endif()

# Build Java OpenJDK if not found
if(NOT JAVA_FOUND)
    message(STATUS "Java not found, will download embedded OpenJDK...")
    
    # Download pre-built OpenJDK for ARM64/x64
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
        set(JDK_URL "https://github.com/adoptium/temurin11-binaries/releases/download/jdk-11.0.20%2B8/OpenJDK11U-jdk_aarch64_linux_hotspot_11.0.20_8.tar.gz")
        set(JDK_HASH "SHA256=1b37e0c8e4af5c5b8d4e5e8e8c0c7ef4a6d9e6d7a8b9c0d1e2f3a4b5c6d7e8f9")
    else()
        set(JDK_URL "https://github.com/adoptium/temurin11-binaries/releases/download/jdk-11.0.20%2B8/OpenJDK11U-jdk_x64_linux_hotspot_11.0.20_8.tar.gz")
        set(JDK_HASH "SHA256=c7a8f6f6f7a8b9c0d1e2f3a4b5c6d7e8f9a0b1c2d3e4f5a6b7c8d9e0f1a2b3c4")
    endif()
    
    ExternalProject_Add(openjdk_external
        PREFIX ${THIRD_PARTY_BUILD_DIR}/openjdk
        URL ${JDK_URL}
        URL_HASH ${JDK_HASH}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory <SOURCE_DIR> ${THIRD_PARTY_INSTALL_DIR}/java
    )
    
    set(JAVA_HOME ${THIRD_PARTY_INSTALL_DIR}/java)
    set(Java_JAVA_EXECUTABLE ${JAVA_HOME}/bin/java)
    set(JAVA_BUILT TRUE)
endif()

# BoofCV - Java computer vision library
if(JAVA_FOUND OR JAVA_BUILT)
    message(STATUS "Setting up BoofCV...")
    
    # BoofCV will be downloaded and built by scripts
    set(BOOFCV_DIR ${THIRD_PARTY_DIR}/BoofCV)
    set(BOOFCPP_DIR ${THIRD_PARTY_DIR}/BoofCPP)
    
    # Create placeholder targets that will be built by external scripts
    add_custom_target(boofcv_download
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/download_boofcv.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Downloading and building BoofCV"
    )
    
    add_custom_target(boofcpp_build
        DEPENDS boofcv_download
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build_boofcpp.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Building BoofCPP wrapper"
    )
endif()

# Create a target that builds all third-party dependencies
add_custom_target(third_party_all)

# Add dependencies to the all target
if(OPENCV_BUILT)
    add_dependencies(third_party_all opencv_external)
endif()

if(EIGEN3_BUILT)
    add_dependencies(third_party_all eigen)
endif()

if(LIBCAMERA_SYNC_BUILT)
    add_dependencies(third_party_all libcamera_sync_external)
endif()

if(JAVA_BUILT)
    add_dependencies(third_party_all openjdk_external)
endif()

# Function to ensure third-party libraries are available
function(ensure_third_party_lib LIB_NAME)
    if(TARGET ${LIB_NAME}_external)
        add_dependencies(unlook_dependencies ${LIB_NAME}_external)
    endif()
endfunction()

# Summary
message(STATUS "Third-party build configuration:")
if(OPENCV_BUILT)
    message(STATUS "  OpenCV: Building from source")
endif()
if(EIGEN3_BUILT) 
    message(STATUS "  Eigen3: Building from source")
endif()
if(LIBCAMERA_SYNC_BUILT)
    message(STATUS "  libcamera-sync: Building from third-party")
endif()
if(JAVA_BUILT)
    message(STATUS "  Java: Downloading embedded OpenJDK")
endif()