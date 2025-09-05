# Test suite for Unlook Stereo Calibration System

# Add test executable
add_executable(test_calibration test_calibration.cpp)
target_link_libraries(test_calibration unlook_stereo_core)

add_executable(test_stereo_matching test_stereo_matching.cpp)
target_link_libraries(test_stereo_matching unlook_stereo_core)

# Register tests
add_test(NAME CalibrationTest COMMAND test_calibration)
add_test(NAME StereoMatchingTest COMMAND test_stereo_matching)