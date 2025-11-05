# Example Applications for Unlook 3D Scanner

# Check which example files exist and only build those
set(EXAMPLE_TARGETS "")

# Basic API example (temporarily disabled - requires DepthProcessor/CalibrationManager)
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/basic_api_example.cpp")
#     add_executable(basic_api_example
#         basic_api_example.cpp
#     )
#     target_include_directories(basic_api_example PRIVATE
#         ${CMAKE_SOURCE_DIR}/src
#     )
#     target_link_libraries(basic_api_example PRIVATE
#         unlook
#         ${OpenCV_LIBS}
#     )
#     list(APPEND EXAMPLE_TARGETS basic_api_example)
# endif()

# Camera example
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/camera_example.cpp")
    add_executable(camera_example
        camera_example.cpp
    )
    target_include_directories(camera_example PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(camera_example PRIVATE
        unlook
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS camera_example)
endif()

# Stereo calibration example (temporarily disabled)
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stereo_calibration_example.cpp")
#     add_executable(stereo_calibration_example
#         stereo_calibration_example.cpp
#     )
#     target_include_directories(stereo_calibration_example PRIVATE
#         ${CMAKE_SOURCE_DIR}/src
#     )
#     target_link_libraries(stereo_calibration_example PRIVATE
#         unlook
#         ${OpenCV_LIBS}
#     )
#     list(APPEND EXAMPLE_TARGETS stereo_calibration_example)
# endif()

# VCSEL Stereo Matcher test
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_vcsel_stereo.cpp")
    add_executable(test_vcsel_stereo
        test_vcsel_stereo.cpp
    )
    target_include_directories(test_vcsel_stereo PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(test_vcsel_stereo PRIVATE
        unlook
        ${OpenCV_LIBS}
    )
    # Add OpenMP support if available
    if(OpenMP_CXX_FOUND)
        target_link_libraries(test_vcsel_stereo PRIVATE OpenMP::OpenMP_CXX)
    endif()
    list(APPEND EXAMPLE_TARGETS test_vcsel_stereo)
endif()

# Depth processing example (temporarily disabled)
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/depth_processing_example.cpp")
#     add_executable(depth_processing_example
#         depth_processing_example.cpp
#     )
#     target_include_directories(depth_processing_example PRIVATE
#         ${CMAKE_SOURCE_DIR}/src
#     )
#     target_link_libraries(depth_processing_example PRIVATE
#         unlook
#         ${OpenCV_LIBS}
#     )
#     list(APPEND EXAMPLE_TARGETS depth_processing_example)
# endif()

# Camera test
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/camera_test.cpp")
    add_executable(camera_test
        camera_test.cpp
    )
    target_include_directories(camera_test PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(camera_test PRIVATE
        unlook
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS camera_test)
endif()

# Calibration validation example
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/calibration_validation_example.cpp")
    add_executable(calibration_validation_example
        calibration_validation_example.cpp
    )
    target_include_directories(calibration_validation_example PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(calibration_validation_example PRIVATE
        unlook
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS calibration_validation_example)
endif()

# Dual VCSEL temporal matching test
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_dual_vcsel_temporal.cpp")
    add_executable(test_dual_vcsel_temporal
        test_dual_vcsel_temporal.cpp
    )
    target_include_directories(test_dual_vcsel_temporal PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(test_dual_vcsel_temporal PRIVATE
        unlook
        unlook_hardware
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS test_dual_vcsel_temporal)
endif()

# Enhanced temporal stereo test
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_temporal_stereo_enhanced.cpp")
    add_executable(test_temporal_stereo_enhanced
        test_temporal_stereo_enhanced.cpp
    )
    target_include_directories(test_temporal_stereo_enhanced PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(test_temporal_stereo_enhanced PRIVATE
        unlook
        unlook_hardware
        unlook_stereo
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS test_temporal_stereo_enhanced)
endif()

# BMI270 Stability Detector Example
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bmi270_stability_example.cpp")
    add_executable(bmi270_stability_example
        bmi270_stability_example.cpp
    )
    target_include_directories(bmi270_stability_example PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(bmi270_stability_example PRIVATE
        unlook_hardware
        unlook_core
        ${OpenCV_LIBS}
    )
    list(APPEND EXAMPLE_TARGETS bmi270_stability_example)
endif()

# Test stereo calibration
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test_stereo_calibration.cpp")
    add_executable(test_stereo_calibration
        test_stereo_calibration.cpp
    )
    target_include_directories(test_stereo_calibration PRIVATE
        ${CMAKE_SOURCE_DIR}/src
    )
    target_link_libraries(test_stereo_calibration PRIVATE
        unlook_calibration
        unlook_core
        ${OpenCV_LIBS}
        nlohmann_json::nlohmann_json
    )
    list(APPEND EXAMPLE_TARGETS test_stereo_calibration)
endif()

# C API example (temporarily disabled - requires C API implementation)
# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/c_api_example.c")
#     add_executable(c_api_example
#         c_api_example.c
#     )
#     target_include_directories(c_api_example PRIVATE
#         ${CMAKE_SOURCE_DIR}/src
#     )
#     target_link_libraries(c_api_example PRIVATE
#         unlook
#     )
#     list(APPEND EXAMPLE_TARGETS c_api_example)
# endif()

# Install examples (only if they were built)
if(EXAMPLE_TARGETS)
    install(TARGETS ${EXAMPLE_TARGETS}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    
    message(STATUS "Example applications configured:")
    foreach(target ${EXAMPLE_TARGETS})
        message(STATUS "  ${target}")
    endforeach()
else()
    message(STATUS "No example applications found to build")
endif()

# Install example source code for reference (all existing files)
file(GLOB EXAMPLE_SOURCES "*.cpp" "*.c")
if(EXAMPLE_SOURCES)
    install(FILES ${EXAMPLE_SOURCES} CMakeLists.txt
        DESTINATION ${CMAKE_INSTALL_DATADIR}/unlook/examples
        COMPONENT Development
    )
endif()