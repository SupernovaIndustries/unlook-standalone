# CMakeLists.txt for Unlook Banking-Grade Facial Recognition Module
# Comprehensive facial recognition system with industrial-grade precision

cmake_minimum_required(VERSION 3.16)

# Face Recognition Module Configuration
set(FACE_MODULE_NAME "unlook_face")
set(FACE_MODULE_VERSION "1.0.0-banking")

# Module-specific options
option(BUILD_FACE_MODULE "Build facial recognition module" ON)
option(FACE_ENABLE_BANKING_MODE "Enable banking-grade features" ON)
option(FACE_ENABLE_GPU_SUPPORT "Enable GPU acceleration for face processing" OFF)
option(FACE_ENABLE_SUPERNOVA_ML "Enable Supernova ML integration" OFF)
option(FACE_BUILD_TESTS "Build facial recognition tests" ON)
option(FACE_BUILD_EXAMPLES "Build facial recognition examples" ON)

if(NOT BUILD_FACE_MODULE)
    message(STATUS "Facial recognition module disabled")
    return()
endif()

message(STATUS "Building Unlook Facial Recognition Module ${FACE_MODULE_VERSION}")

# Face module headers
set(FACE_HEADERS
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceTypes.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceDetector.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/LandmarkExtractor.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/Face3DReconstructor.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceEncoder.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceMatcher.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/LivenessDetector.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceEnroller.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/SupernovaMLInterface.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/face/FaceAPI.hpp
)

# Face module sources
set(FACE_SOURCES
    FaceTypes.cpp
    FaceDetector.cpp
    LandmarkExtractor.cpp
    Face3DReconstructor.cpp
    FaceEncoder.cpp
    FaceMatcher.cpp
    LivenessDetector.cpp
    FaceEnroller.cpp
    SupernovaMLInterface.cpp
    FaceAPI.cpp
)

# Required dependencies for facial recognition
find_package(OpenCV REQUIRED COMPONENTS
    core imgproc imgcodecs highgui objdetect calib3d features2d
    dnn video videoio photo ximgproc xphoto
)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# OpenSSL for encryption/security
find_package(OpenSSL REQUIRED)

# Optional GPU acceleration
if(FACE_ENABLE_GPU_SUPPORT)
    find_package(CUDA QUIET)
    if(CUDA_FOUND)
        enable_language(CUDA)
        message(STATUS "CUDA found: enabling GPU acceleration for face processing")
        add_definitions(-DFACE_ENABLE_CUDA)
    else()
        message(WARNING "CUDA not found: GPU acceleration disabled")
        set(FACE_ENABLE_GPU_SUPPORT OFF)
    endif()
endif()

# HTTP client library for Supernova ML integration
if(FACE_ENABLE_SUPERNOVA_ML)
    find_package(CURL REQUIRED)
    if(CURL_FOUND)
        message(STATUS "CURL found: enabling Supernova ML integration")
        add_definitions(-DFACE_ENABLE_SUPERNOVA_ML)
    else()
        message(WARNING "CURL not found: Supernova ML integration disabled")
        set(FACE_ENABLE_SUPERNOVA_ML OFF)
    endif()
endif()

# Create the face recognition library
add_library(${FACE_MODULE_NAME} SHARED ${FACE_SOURCES})

# Set library properties
set_target_properties(${FACE_MODULE_NAME} PROPERTIES
    VERSION ${FACE_MODULE_VERSION}
    SOVERSION 1
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Include directories
target_include_directories(${FACE_MODULE_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Link against required libraries
target_link_libraries(${FACE_MODULE_NAME} PUBLIC
    # Core Unlook libraries
    unlook_core
    unlook_stereo
    unlook_calibration

    # OpenCV libraries
    ${OpenCV_LIBS}

    # System libraries
    Threads::Threads
    OpenSSL::SSL
    OpenSSL::Crypto
)

# Optional GPU support
if(FACE_ENABLE_GPU_SUPPORT AND CUDA_FOUND)
    target_link_libraries(${FACE_MODULE_NAME} PRIVATE
        ${CUDA_LIBRARIES}
        ${CUDA_CUBLAS_LIBRARIES}
        ${CUDA_curand_LIBRARY}
    )
    target_include_directories(${FACE_MODULE_NAME} PRIVATE ${CUDA_INCLUDE_DIRS})
endif()

# Supernova ML integration
if(FACE_ENABLE_SUPERNOVA_ML)
    target_link_libraries(${FACE_MODULE_NAME} PRIVATE
        CURL::libcurl
    )
endif()

# Compiler definitions
target_compile_definitions(${FACE_MODULE_NAME} PRIVATE
    UNLOOK_FACE_MODULE_VERSION="${FACE_MODULE_VERSION}"
    UNLOOK_FACE_BUILD_TIMESTAMP="${CMAKE_BUILD_TIMESTAMP}"
)

if(FACE_ENABLE_BANKING_MODE)
    target_compile_definitions(${FACE_MODULE_NAME} PUBLIC
        FACE_BANKING_MODE_ENABLED
        FACE_BANKING_FAR_TARGET=0.0001f
        FACE_BANKING_FRR_TARGET=0.03f
    )
endif()

# ARM64/NEON optimizations for CM4/CM5
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    message(STATUS "ARM64 detected: enabling NEON optimizations for face processing")
    target_compile_options(${FACE_MODULE_NAME} PRIVATE
        -march=armv8-a
        -mtune=cortex-a76  # CM5 optimization
        -mfpu=neon-fp-armv8
        -ftree-vectorize
        -ffast-math
        -O3
    )
    target_compile_definitions(${FACE_MODULE_NAME} PRIVATE
        FACE_ARM64_NEON_ENABLED
        FACE_CM4_CM5_OPTIMIZED
    )
endif()

# Compiler-specific optimizations
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(${FACE_MODULE_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
        -fPIC
        -fopenmp  # Enable OpenMP for parallel processing
    )
    target_link_libraries(${FACE_MODULE_NAME} PRIVATE gomp)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(${FACE_MODULE_NAME} PRIVATE
        -Wall -Wextra -Wpedantic
        -Wno-unused-parameter
        -fPIC
    )
endif()

# Debug/Release specific settings
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${FACE_MODULE_NAME} PRIVATE
        FACE_DEBUG_MODE
        FACE_ENABLE_PROFILING
    )
    target_compile_options(${FACE_MODULE_NAME} PRIVATE -g -O0)
else()
    target_compile_definitions(${FACE_MODULE_NAME} PRIVATE
        FACE_RELEASE_MODE
        NDEBUG
    )
    target_compile_options(${FACE_MODULE_NAME} PRIVATE -O3 -DNDEBUG)
endif()

# Install face recognition library
install(TARGETS ${FACE_MODULE_NAME}
    EXPORT UnlookFaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install face recognition headers
install(FILES ${FACE_HEADERS}
    DESTINATION include/unlook/face
)

# Install CMake config files
install(EXPORT UnlookFaceTargets
    FILE UnlookFaceTargets.cmake
    NAMESPACE Unlook::
    DESTINATION lib/cmake/Unlook
)

# Create pkg-config file for face module
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/unlook-face.pc.in
    ${CMAKE_BINARY_DIR}/unlook-face.pc
    @ONLY
)
install(FILES ${CMAKE_BINARY_DIR}/unlook-face.pc
    DESTINATION lib/pkgconfig
)

# Face recognition examples
if(FACE_BUILD_EXAMPLES)
    message(STATUS "Building facial recognition examples")
    add_subdirectory(examples)
endif()

# Face recognition tests
if(FACE_BUILD_TESTS AND BUILD_TESTS)
    message(STATUS "Building facial recognition tests")
    add_subdirectory(tests)
endif()

# Face recognition benchmarks (optional)
if(BUILD_BENCHMARKS)
    message(STATUS "Building facial recognition benchmarks")
    add_subdirectory(benchmarks)
endif()

# Print configuration summary
message(STATUS "")
message(STATUS "=== Unlook Facial Recognition Module Configuration ===")
message(STATUS "Version: ${FACE_MODULE_VERSION}")
message(STATUS "Banking Mode: ${FACE_ENABLE_BANKING_MODE}")
message(STATUS "GPU Support: ${FACE_ENABLE_GPU_SUPPORT}")
message(STATUS "Supernova ML: ${FACE_ENABLE_SUPERNOVA_ML}")
message(STATUS "Build Tests: ${FACE_BUILD_TESTS}")
message(STATUS "Build Examples: ${FACE_BUILD_EXAMPLES}")
message(STATUS "Target Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}")
if(FACE_ENABLE_GPU_SUPPORT AND CUDA_FOUND)
    message(STATUS "CUDA Version: ${CUDA_VERSION}")
endif()
message(STATUS "==================================================")
message(STATUS "")

# Performance profiling support
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Add profiling targets for performance analysis
    add_custom_target(face_profile
        COMMAND echo "Face recognition profiling tools available"
        COMMENT "Use with: perf record --call-graph dwarf ./face_example"
    )
endif()

# Memory debugging support
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    # Add memory debugging options
    target_compile_options(${FACE_MODULE_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
        -fno-omit-frame-pointer
    )
    target_link_options(${FACE_MODULE_NAME} PRIVATE
        -fsanitize=address
        -fsanitize=leak
    )
endif()

# Thread safety validation
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${FACE_MODULE_NAME} PRIVATE
        -fsanitize=thread
    )
    target_link_options(${FACE_MODULE_NAME} PRIVATE
        -fsanitize=thread
    )
endif()

# Generate compile_commands.json for development
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)