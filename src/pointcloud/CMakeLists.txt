# Point cloud processing library with Open3D integration

# Point cloud source files
set(POINTCLOUD_SOURCES
    PointCloudProcessor.cpp
)

set(POINTCLOUD_HEADERS
    ${CMAKE_SOURCE_DIR}/include/unlook/pointcloud/PointCloudProcessor.hpp
)

# Create point cloud library
add_library(unlook_pointcloud ${POINTCLOUD_SOURCES} ${POINTCLOUD_HEADERS})

# Set library properties
set_target_properties(unlook_pointcloud PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    OUTPUT_NAME unlook_pointcloud
    EXPORT_NAME pointcloud
)

# Include directories
target_include_directories(unlook_pointcloud
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies
target_link_libraries(unlook_pointcloud
    PUBLIC
        unlook_stereo
        unlook_calibration
        ${OpenCV_LIBS}
    PRIVATE
        ${CMAKE_THREAD_LIBS_INIT}
        ${FILESYSTEM_LIB}
)

# Conditional Open3D linking
if(OPEN3D_ENABLED)
    target_link_libraries(unlook_pointcloud
        PUBLIC
            ${Open3D_LIBRARIES}
    )
    target_compile_definitions(unlook_pointcloud
        PUBLIC
            OPEN3D_ENABLED=1
    )

    # Open3D specific include directories
    if(Open3D_INCLUDE_DIRS)
        target_include_directories(unlook_pointcloud
            PUBLIC
                ${Open3D_INCLUDE_DIRS}
        )
    endif()

    message(STATUS "Point cloud library built with Open3D support")
else()
    # PCL fallback if available
    if(PCL_ENABLED)
        target_link_libraries(unlook_pointcloud
            PUBLIC
                ${PCL_LIBRARIES}
        )
        target_include_directories(unlook_pointcloud
            PUBLIC
                ${PCL_INCLUDE_DIRS}
        )
        target_compile_definitions(unlook_pointcloud
            PUBLIC
                USE_PCL_FALLBACK=1
        )
        message(STATUS "Point cloud library built with PCL fallback")
    else()
        message(STATUS "Point cloud library built with basic fallback implementations")
    endif()
endif()

# OpenMP support for parallel processing
if(OpenMP_CXX_FOUND)
    target_link_libraries(unlook_pointcloud
        PUBLIC
            OpenMP::OpenMP_CXX
    )
    target_compile_definitions(unlook_pointcloud
        PRIVATE
            USE_OPENMP=1
    )
    message(STATUS "Point cloud library built with OpenMP support")
endif()

# ARM64 specific optimizations
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_compile_definitions(unlook_pointcloud
        PRIVATE
            UNLOOK_ARM64_OPTIMIZED=1
    )
    message(STATUS "Point cloud library built with ARM64 optimizations")
endif()

# Compiler-specific optimizations
target_compile_features(unlook_pointcloud PUBLIC cxx_std_17)

# Set compiler flags for point cloud processing (allow unused parameter warnings for fallback implementations)
target_compile_options(unlook_pointcloud
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU>:-Wall -Wextra -O3 -ffast-math -march=native -Wno-unused-parameter>
        $<$<CXX_COMPILER_ID:Clang>:-Wall -Wextra -O3 -ffast-math -march=native -Wno-unused-parameter>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /O2>
)

# Position independent code for shared libraries
set_property(TARGET unlook_pointcloud PROPERTY POSITION_INDEPENDENT_CODE ON)

# Installation
install(TARGETS unlook_pointcloud
    EXPORT UnlookTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(FILES ${POINTCLOUD_HEADERS}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook/pointcloud
)

# Debug information
message(STATUS "Point cloud library configuration:")
message(STATUS "  Sources: ${POINTCLOUD_SOURCES}")
message(STATUS "  Headers: ${POINTCLOUD_HEADERS}")
message(STATUS "  Open3D enabled: ${OPEN3D_ENABLED}")
message(STATUS "  PCL fallback: ${PCL_ENABLED}")
message(STATUS "  ARM64 optimizations: ${CMAKE_SYSTEM_PROCESSOR}")