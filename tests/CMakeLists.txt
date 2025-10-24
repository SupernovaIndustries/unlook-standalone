# Test suite for Unlook Stereo Calibration System

# Add test executable
add_executable(test_calibration test_calibration.cpp)
target_link_libraries(test_calibration unlook_stereo_core)

add_executable(test_stereo_matching test_stereo_matching.cpp)
target_link_libraries(test_stereo_matching unlook_stereo_core)

# File logger test (thread-safety and performance validation)
add_executable(test_file_logger test_file_logger.cpp)
target_link_libraries(test_file_logger unlook_core)

# Artec-grade pipeline integration test (MISSION CRITICAL for investor demo)
if(OPEN3D_ENABLED)
    add_executable(test_complete_pipeline mesh/test_complete_pipeline.cpp)
    target_link_libraries(test_complete_pipeline
        unlook_pointcloud
        unlook_stereo_core
        unlook_core
        ${Open3D_LIBRARIES}
        ${OpenCV_LIBS}
    )
    target_include_directories(test_complete_pipeline PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${Open3D_INCLUDE_DIRS}
    )
    target_compile_definitions(test_complete_pipeline PRIVATE
        OPEN3D_ENABLED=1
    )
    message(STATUS "Building Artec-grade pipeline test (test_complete_pipeline)")
else()
    message(WARNING "Skipping test_complete_pipeline - requires Open3D")
endif()

# Register tests
add_test(NAME CalibrationTest COMMAND test_calibration)
add_test(NAME StereoMatchingTest COMMAND test_stereo_matching)
add_test(NAME FileLoggerTest COMMAND test_file_logger)

if(OPEN3D_ENABLED)
    add_test(NAME CompletePipelineTest COMMAND test_complete_pipeline)
endif()