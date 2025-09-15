# Base stereo library sources
set(STEREO_SOURCES
    DepthProcessor.cpp
    StereoMatcher.cpp
    SGBMStereoMatcher.cpp
    DisparityFilter.cpp
    ParameterManager.cpp
)

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
)

# Link dependencies
target_link_libraries(unlook_stereo
    PUBLIC
        ${OpenCV_LIBS}
        unlook_calibration
)

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