# Gesture recognition module CMakeLists.txt

# Find pybind11 and Python3 (for MediaPipe wrapper)
find_package(pybind11 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

message(STATUS "pybind11 found: ${pybind11_FOUND}")
message(STATUS "Python3 found: ${Python3_FOUND}")
message(STATUS "Python3 version: ${Python3_VERSION}")
message(STATUS "Python3 include dirs: ${Python3_INCLUDE_DIRS}")
message(STATUS "Python3 libraries: ${Python3_LIBRARIES}")

# Gesture library sources (MediaPipe only, no ONNX)
set(GESTURE_SOURCES
    GestureRecognitionSystem.cpp
    TemporalBuffer.cpp
    GeometricSwipeDetector.cpp
    MediaPipeWrapper.cpp
    MotionBlurCompensation.cpp
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
        ${Python3_INCLUDE_DIRS}
        ${pybind11_INCLUDE_DIRS}
)

# Link dependencies (MediaPipe only, no ONNX)
target_link_libraries(unlook_gesture
    PUBLIC
        ${OpenCV_LIBS}
    PRIVATE
        unlook_camera_impl
        unlook_core
        pybind11::embed
        Python3::Python
)

# Set Python path for runtime
target_compile_definitions(unlook_gesture PRIVATE
    PYTHON_SCRIPTS_PATH="${CMAKE_SOURCE_DIR}/python"
    MEDIAPIPE_MODEL_PATH="${CMAKE_SOURCE_DIR}/models/gesture_recognizer.task"
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
