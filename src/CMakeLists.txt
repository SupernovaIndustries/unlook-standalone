# Source directory CMakeLists.txt for Unlook 3D Scanner API

# Build component libraries in dependency order (calibration first, then stereo, then pointcloud, hardware, mesh, then validation)
add_subdirectory(calibration)
add_subdirectory(stereo)
# Temporarily disable pointcloud module due to mesh dependencies
add_subdirectory(pointcloud)
add_subdirectory(hardware)
# Temporarily disable mesh module due to compilation issues
# add_subdirectory(mesh)
add_subdirectory(validation)

# Core library sources (no Qt dependencies)
set(UNLOOK_CORE_SOURCES
    core/exception.cpp
    core/Logger.cpp
    core/config.cpp
)

# API implementation sources
set(UNLOOK_API_SOURCES
    api/unlook_scanner.cpp
    api/camera_system.cpp
    api/depth_processor.cpp
    api/calibration_manager.cpp
)

# Core component sources (minimal working set)
set(UNLOOK_COMPONENT_SOURCES
    # Utility components only (basic functionality)
    utils/FileUtils.cpp
    utils/MathUtils.cpp
    # Camera system implementation (required for GUI)
    camera/CameraSystem.cpp
    camera/LibcameraSyncDevice.cpp
    camera/CameraDevice.cpp
    camera/HardwareSyncManager.cpp
    camera/CameraSynchronizer.cpp
    camera/AutoExposure.cpp
    camera/CameraUtils.cpp
    camera/HardwareSyncCapture.cpp
    # Real-time processing components
    realtime/BayerNEON.cpp
    # realtime/RealtimePipeline.cpp  # Temporarily disabled for integration testing
)

# Combine all sources
set(UNLOOK_SOURCES
    ${UNLOOK_CORE_SOURCES}
    ${UNLOOK_API_SOURCES}
    ${UNLOOK_COMPONENT_SOURCES}
)

# Create the main Unlook library
if(BUILD_SHARED_LIBS)
    add_library(unlook SHARED ${UNLOOK_SOURCES})
    set_target_properties(unlook PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION ${PROJECT_VERSION_MAJOR}
        OUTPUT_NAME "unlook"
        CLEAN_DIRECT_OUTPUT 1
    )
    message(STATUS "Building shared library: libunlook.so")
else()
    add_library(unlook STATIC ${UNLOOK_SOURCES})
    set_target_properties(unlook PROPERTIES
        OUTPUT_NAME "unlook"
        CLEAN_DIRECT_OUTPUT 1
    )
    message(STATUS "Building static library: libunlook.a")
endif()

# Set target properties
set_target_properties(unlook PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    PUBLIC_HEADER "${CMAKE_SOURCE_DIR}/include/unlook/api/unlook_scanner.h"
)

# Target include directories
target_include_directories(unlook
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(unlook
    PUBLIC
        ${OpenCV_LIBS}
        Threads::Threads
        unlook_stereo       # Link stereo library publicly so GUI can access it
        unlook_calibration  # Link calibration library publicly so GUI can access it
        unlook_hardware     # Link hardware library publicly so GUI can access VCSEL functionality
        # unlook_pointcloud # Temporarily disabled due to mesh dependencies
        # unlook_mesh       # Temporarily disabled due to compilation issues
        unlook_validation   # Link validation library publicly so GUI can access it
    PRIVATE
        ${FILESYSTEM_LIB}
)

# Add libcamera if available
if(LIBCAMERA_FOUND OR LIBCAMERA_LIBRARIES)
    target_link_libraries(unlook PRIVATE ${LIBCAMERA_LIBRARIES})
    target_compile_definitions(unlook PRIVATE HAVE_LIBCAMERA=1)
endif()

# Add OpenMP support
if(OpenMP_CXX_FOUND)
    target_link_libraries(unlook PRIVATE OpenMP::OpenMP_CXX)
    target_compile_definitions(unlook PRIVATE HAVE_OPENMP=1)
endif()

# Compiler-specific flags
target_compile_options(unlook PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -pedantic>
    $<$<CXX_COMPILER_ID:GNU,Clang>:$<$<CONFIG:Debug>:-g -O0>>
    $<$<CXX_COMPILER_ID:GNU,Clang>:$<$<CONFIG:Release>:-O3 -DNDEBUG>>
)

# Define version macros
target_compile_definitions(unlook PRIVATE
    UNLOOK_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    UNLOOK_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    UNLOOK_VERSION_PATCH=${PROJECT_VERSION_PATCH}
    UNLOOK_VERSION_STRING="${PROJECT_VERSION}"
)

# Position Independent Code for shared library
if(BUILD_SHARED_LIBS)
    set_target_properties(unlook PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# Installation
install(TARGETS unlook
    EXPORT UnlookTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Runtime
        NAMELINK_COMPONENT Development
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT Runtime
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/api
        COMPONENT Development
)

# Export targets
install(EXPORT UnlookTargets
    FILE UnlookTargets.cmake
    NAMESPACE Unlook::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/Unlook
    COMPONENT Development
)

# Create additional API libraries for modular access
# These are convenience libraries that use the main unlook library

# Camera System convenience library
add_library(unlook_camera INTERFACE)
target_link_libraries(unlook_camera INTERFACE unlook)
target_include_directories(unlook_camera INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Depth Processing convenience library (when implemented)
add_library(unlook_depth INTERFACE)
target_link_libraries(unlook_depth INTERFACE unlook)
target_include_directories(unlook_depth INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Mesh Processing convenience library
# add_library(unlook_mesh_api INTERFACE)
# target_link_libraries(unlook_mesh_api INTERFACE unlook)
# target_include_directories(unlook_mesh_api INTERFACE
#     $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

# Install convenience libraries
install(TARGETS unlook_camera unlook_depth # unlook_mesh_api
    EXPORT UnlookTargets
    COMPONENT Development
)

# Add GUI application if Qt5 is available
if(BUILD_GUI AND Qt5_FOUND)
    add_subdirectory(gui)
endif()

# Debug information
message(STATUS "Library sources: ${UNLOOK_SOURCES}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
if(LIBCAMERA_LIBRARIES)
    message(STATUS "libcamera libraries: ${LIBCAMERA_LIBRARIES}")
endif()

# Summary for this directory
message(STATUS "--- Source Directory Configuration ---")
message(STATUS "Core sources: ${UNLOOK_CORE_SOURCES}")
message(STATUS "API sources: ${UNLOOK_API_SOURCES}")
message(STATUS "Library type: ${BUILD_SHARED_LIBS}")
message(STATUS "--------------------------------------")