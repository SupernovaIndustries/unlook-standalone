# Base stereo library sources
set(STEREO_SOURCES
    DepthProcessor.cpp
    StereoMatcher.cpp
    SGBMStereoMatcher.cpp
    CensusStereoMatcher.cpp
    SGMCensus.cpp  # Production SGM-Census implementation
    ProgressiveStereoMatcher.cpp
    DisparityFilter.cpp
    ParameterManager.cpp
    RawStereoProcessor.cpp
    TemporalStereoProcessor.cpp
    VCSELStereoMatcher.cpp
    VulkanSGMAccelerator.cpp
    RectificationEngine.cpp
    DebugOutputManager.cpp
    PointCloudGenerator.cpp
    # New PHASE 2 and PHASE 3 implementations
    DisparityComputer.cpp
    DepthConverter.cpp
    MedianFilter.cpp
    SpatialFilter.cpp
    TemporalFilter.cpp
    HoleFiller.cpp
    # ARM NEON SIMD optimization
    SGM_NEON.cpp
)

# NEON optimized sources
set(NEON_SOURCES
    neon/census_neon.cpp
    neon/hamming_neon.cpp
    neon/ad_cost_neon.cpp
)

# Add NEON sources to main sources
list(APPEND STEREO_SOURCES ${NEON_SOURCES})

# Add BoofCV sources only if enabled
if(BOOFCV_ENABLED)
    list(APPEND STEREO_SOURCES BoofCVStereoMatcher.cpp)
endif()

# Stereo library
add_library(unlook_stereo STATIC ${STEREO_SOURCES})

# Set target properties
set_target_properties(unlook_stereo PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(unlook_stereo
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
        /usr/include/eigen3  # Add Eigen include path for Open3D
)

# Link dependencies
target_link_libraries(unlook_stereo
    PUBLIC
        ${OpenCV_LIBS}
        unlook_calibration
        unlook_camera_impl
        unlook_hardware
)

# Link Vulkan for GPU acceleration
find_package(Vulkan)
if(Vulkan_FOUND)
    target_link_libraries(unlook_stereo PUBLIC Vulkan::Vulkan)
    target_compile_definitions(unlook_stereo PUBLIC HAS_VULKAN)
    message(STATUS "Vulkan GPU acceleration enabled for stereo matching")

    # Copy SPIR-V shaders to build directory
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/vulkan/sgm_path_lr.spv
         DESTINATION ${CMAKE_BINARY_DIR}/src/stereo/vulkan/)
endif()

# Explicitly ensure OpenCV modules are linked (especially ximgproc for WLS filtering)
if(OpenCV_FOUND)
    target_compile_definitions(unlook_stereo PUBLIC
        HAVE_OPENCV_XIMGPROC=1
        HAVE_OPENCV_STEREO=1
    )
endif()

# Add OpenMP support
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(unlook_stereo PUBLIC OpenMP::OpenMP_CXX)
endif()

# Add ARM NEON compiler flags for ARM platforms
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
    target_compile_options(unlook_stereo PRIVATE -march=native -mtune=native)
    target_compile_definitions(unlook_stereo PRIVATE __ARM_NEON HAVE_NEON)

    # Apply aggressive optimization to NEON implementation files
    set_source_files_properties(
        neon/census_neon.cpp
        neon/hamming_neon.cpp
        neon/ad_cost_neon.cpp
        SGM_NEON.cpp
        PROPERTIES COMPILE_FLAGS "-march=armv8-a+simd -O3 -ftree-vectorize -ffast-math"
    )

    message(STATUS "NEON optimizations enabled for stereo matching")
    message(STATUS "  - census_neon.cpp: Census transform with ARM NEON")
    message(STATUS "  - hamming_neon.cpp: Hamming distance with ARM NEON")
    message(STATUS "  - ad_cost_neon.cpp: Absolute difference cost with ARM NEON")
endif()

# Link Open3D if available
if(Open3D_FOUND)
    target_link_libraries(unlook_stereo PUBLIC Open3D::Open3D)
    target_compile_definitions(unlook_stereo PUBLIC OPEN3D_ENABLED)
endif()

# Add BoofCV/JNI support if enabled
if(BOOFCV_ENABLED)
    target_link_libraries(unlook_stereo PRIVATE ${JNI_LIBRARIES})
    target_compile_definitions(unlook_stereo PRIVATE 
        HAVE_BOOFCV=1
        BOOFCV_JAR_PATH="${BOOFCV_JAR_PATH}"
    )
endif()

# Installation and export
install(TARGETS unlook_stereo
    EXPORT UnlookTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
)