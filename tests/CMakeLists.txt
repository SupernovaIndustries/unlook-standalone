# Test suite for Unlook Stereo Calibration System

# Add test executable
add_executable(test_calibration test_calibration.cpp)
target_link_libraries(test_calibration unlook_stereo unlook_calibration unlook_core ${OpenCV_LIBS})

add_executable(test_stereo_matching test_stereo_matching.cpp)
target_link_libraries(test_stereo_matching unlook_stereo unlook_core ${OpenCV_LIBS})

# File logger test (thread-safety and performance validation)
add_executable(test_file_logger test_file_logger.cpp)
target_link_libraries(test_file_logger unlook_core)

# Artec-grade pipeline integration test (MISSION CRITICAL for investor demo)
if(OPEN3D_ENABLED)
    add_executable(test_complete_pipeline mesh/test_complete_pipeline.cpp)
    target_link_libraries(test_complete_pipeline
        unlook_pointcloud
        unlook_stereo
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

# ============================================================================
# AD-Census Handheld Scanner Test Suite
# ============================================================================

# Check for Google Test
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    message(STATUS "Google Test not found, fetching from GitHub...")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Test 1: VCSELStereoMatcher Unit Tests
add_executable(test_vcsel_stereo_unit test_vcsel_stereo_unit.cpp)
target_link_libraries(test_vcsel_stereo_unit
    PRIVATE
    unlook_stereo
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(test_vcsel_stereo_unit PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Test 2: StabilityDetector Unit Tests
add_executable(test_stability_detector test_stability_detector.cpp)
target_link_libraries(test_stability_detector
    PRIVATE
    unlook_hardware
    unlook_core
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(test_stability_detector PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Test 3: HandheldScanPipeline Unit Tests
add_executable(test_handheld_pipeline test_handheld_pipeline.cpp)
target_link_libraries(test_handheld_pipeline
    PRIVATE
    unlook
    unlook_core
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(test_handheld_pipeline PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Test 4: Integration Tests
add_executable(test_handheld_integration test_handheld_integration.cpp)
target_link_libraries(test_handheld_integration
    PRIVATE
    unlook
    unlook_stereo
    unlook_hardware
    unlook_core
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)
target_include_directories(test_handheld_integration PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Benchmark: Performance Testing
add_executable(handheld_performance_benchmark benchmarks/handheld_performance_benchmark.cpp)
target_link_libraries(handheld_performance_benchmark
    PRIVATE
    unlook_stereo
    unlook_core
    ${OpenCV_LIBS}
)
target_include_directories(handheld_performance_benchmark PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Register tests
add_test(NAME CalibrationTest COMMAND test_calibration)
add_test(NAME StereoMatchingTest COMMAND test_stereo_matching)
add_test(NAME FileLoggerTest COMMAND test_file_logger)

# Register new AD-Census handheld scanner tests
add_test(NAME VCSELStereoUnitTest COMMAND test_vcsel_stereo_unit)
add_test(NAME StabilityDetectorTest COMMAND test_stability_detector)
add_test(NAME HandheldPipelineTest COMMAND test_handheld_pipeline)
add_test(NAME HandheldIntegrationTest COMMAND test_handheld_integration)

if(OPEN3D_ENABLED)
    add_test(NAME CompletePipelineTest COMMAND test_complete_pipeline)
endif()

# Print test suite summary
message(STATUS "==============================================")
message(STATUS "  AD-Census Handheld Scanner Test Suite")
message(STATUS "==============================================")
message(STATUS "Unit Tests:")
message(STATUS "  - test_vcsel_stereo_unit        (15 tests)")
message(STATUS "  - test_stability_detector       (17 tests)")
message(STATUS "  - test_handheld_pipeline        (20 tests)")
message(STATUS "Integration Tests:")
message(STATUS "  - test_handheld_integration     (10 tests)")
message(STATUS "Performance Benchmarks:")
message(STATUS "  - handheld_performance_benchmark (6 benchmarks)")
message(STATUS "==============================================")
message(STATUS "Total: 62+ tests covering AD-Census system")
message(STATUS "==============================================")