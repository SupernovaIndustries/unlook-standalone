# Gesture recognition module CMakeLists.txt

# Find ONNX Runtime
find_package(ONNXRuntime REQUIRED)

# Gesture library sources
set(GESTURE_SOURCES
    GestureRecognitionSystem.cpp
    HandDetector.cpp
    HandLandmarkExtractor.cpp
    HandTracker.cpp
    TemporalBuffer.cpp
    GeometricSwipeDetector.cpp
)

# Gesture library
add_library(unlook_gesture STATIC ${GESTURE_SOURCES})

# Set target properties
set_target_properties(unlook_gesture PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(unlook_gesture
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${OpenCV_INCLUDE_DIRS}
)

# Link dependencies
target_link_libraries(unlook_gesture
    PUBLIC
        ${OpenCV_LIBS}
        ONNXRuntime::ONNXRuntime
    PRIVATE
        unlook_camera_impl
        unlook_core
)

# Installation
install(TARGETS unlook_gesture
    EXPORT UnlookTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT Development
)

# Install headers
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/unlook/gesture
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/unlook
    FILES_MATCHING PATTERN "*.hpp"
)
