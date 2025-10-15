# FindONNXRuntime.cmake
# Finds the ONNX Runtime library and headers for ARM64 deployment
#
# Variables set by this script:
#   ONNXRuntime_FOUND - System has ONNX Runtime
#   ONNXRuntime_INCLUDE_DIRS - The ONNX Runtime include directories
#   ONNXRuntime_LIBRARIES - The libraries needed to use ONNX Runtime
#   ONNXRuntime_VERSION - The version of ONNX Runtime found
#
# Targets created:
#   ONNXRuntime::ONNXRuntime - Imported target for ONNX Runtime

# Priority search paths
set(ONNXRUNTIME_SEARCH_PATHS
    ${CMAKE_SOURCE_DIR}/third-party/onnxruntime/onnxruntime-linux-aarch64-1.16.3
    ${CMAKE_SOURCE_DIR}/third-party/onnxruntime
    /usr/local
    /usr
)

# Find include directory
find_path(ONNXRuntime_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS ${ONNXRUNTIME_SEARCH_PATHS}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)

# Find library
find_library(ONNXRuntime_LIBRARY
    NAMES onnxruntime libonnxruntime.so libonnxruntime.so.1.16.3
    PATHS ${ONNXRUNTIME_SEARCH_PATHS}
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)

# Extract version if found
if(ONNXRuntime_INCLUDE_DIR)
    file(STRINGS "${ONNXRuntime_INCLUDE_DIR}/onnxruntime_c_api.h"
         VERSION_LINE REGEX "^#define ORT_API_VERSION")
    if(VERSION_LINE)
        string(REGEX REPLACE "^#define ORT_API_VERSION ([0-9]+)" "\\1"
               ONNXRuntime_VERSION "${VERSION_LINE}")
        # Version 1.16.3 corresponds to API version 16
        set(ONNXRuntime_VERSION "1.16.3")
    endif()
endif()

# Standard find_package_handle_standard_args
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ONNXRuntime
    REQUIRED_VARS
        ONNXRuntime_LIBRARY
        ONNXRuntime_INCLUDE_DIR
    VERSION_VAR
        ONNXRuntime_VERSION
)

# Set output variables
if(ONNXRuntime_FOUND)
    set(ONNXRuntime_INCLUDE_DIRS ${ONNXRuntime_INCLUDE_DIR})
    set(ONNXRuntime_LIBRARIES ${ONNXRuntime_LIBRARY})

    # Create imported target
    if(NOT TARGET ONNXRuntime::ONNXRuntime)
        add_library(ONNXRuntime::ONNXRuntime SHARED IMPORTED)
        set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
            IMPORTED_LOCATION "${ONNXRuntime_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}"
        )

        # Add RPATH for runtime library location
        get_filename_component(ONNXRuntime_LIB_DIR "${ONNXRuntime_LIBRARY}" DIRECTORY)
        set_target_properties(ONNXRuntime::ONNXRuntime PROPERTIES
            INTERFACE_LINK_DIRECTORIES "${ONNXRuntime_LIB_DIR}"
        )
    endif()

    # Print configuration info
    if(NOT ONNXRuntime_FIND_QUIETLY)
        message(STATUS "Found ONNX Runtime ${ONNXRuntime_VERSION}")
        message(STATUS "  Include: ${ONNXRuntime_INCLUDE_DIR}")
        message(STATUS "  Library: ${ONNXRuntime_LIBRARY}")
    endif()
endif()

# Mark variables as advanced
mark_as_advanced(
    ONNXRuntime_INCLUDE_DIR
    ONNXRuntime_LIBRARY
)
