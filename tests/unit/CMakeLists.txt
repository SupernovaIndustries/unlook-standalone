# Unit Tests - Individual component testing

# Core module tests
add_unlook_test(test_logger
    test_logger.cpp
)
set_tests_properties(test_logger PROPERTIES LABELS "unit;core")

add_unlook_test(test_config
    test_config.cpp
)
set_tests_properties(test_config PROPERTIES LABELS "unit;core")

add_unlook_test(test_timer
    test_timer.cpp
)
set_tests_properties(test_timer PROPERTIES LABELS "unit;core")

add_unlook_test(test_types
    test_types.cpp
)
set_tests_properties(test_types PROPERTIES LABELS "unit;core")

# Camera module tests
add_unlook_test(test_camera_manager
    test_camera_manager.cpp
)
set_tests_properties(test_camera_manager PROPERTIES LABELS "unit;camera")

add_unlook_test(test_stereo_camera
    test_stereo_camera.cpp
)
set_tests_properties(test_stereo_camera PROPERTIES LABELS "unit;camera")

# Calibration module tests
add_unlook_test(test_pattern_detector
    test_pattern_detector.cpp
)
set_tests_properties(test_pattern_detector PROPERTIES LABELS "unit;calibration")

add_unlook_test(test_stereo_calibration
    test_stereo_calibration.cpp
)
set_tests_properties(test_stereo_calibration PROPERTIES LABELS "unit;calibration")

add_unlook_test(test_calibration_validation
    test_calibration_validation.cpp
)
set_tests_properties(test_calibration_validation PROPERTIES LABELS "unit;calibration")

# Stereo processing tests
add_unlook_test(test_stereo_matcher
    test_stereo_matcher.cpp
)
set_tests_properties(test_stereo_matcher PROPERTIES LABELS "unit;stereo")

add_unlook_test(test_depth_processor
    test_depth_processor.cpp
)
set_tests_properties(test_depth_processor PROPERTIES LABELS "unit;stereo")

add_unlook_test(test_rectification
    test_rectification.cpp
)
set_tests_properties(test_rectification PROPERTIES LABELS "unit;stereo")

# Hardware module tests
add_unlook_test(test_i2c_controller
    test_i2c_controller.cpp
)
set_tests_properties(test_i2c_controller PROPERTIES LABELS "unit;hardware")

add_unlook_test(test_led_controller
    test_led_controller.cpp
)
set_tests_properties(test_led_controller PROPERTIES LABELS "unit;hardware")

# Math utilities tests
add_unlook_test(test_geometry
    test_geometry.cpp
)
set_tests_properties(test_geometry PROPERTIES LABELS "unit;math")

add_unlook_test(test_transforms
    test_transforms.cpp
)
set_tests_properties(test_transforms PROPERTIES LABELS "unit;math")

# I/O module tests
add_unlook_test(test_file_io
    test_file_io.cpp
)
set_tests_properties(test_file_io PROPERTIES LABELS "unit;io")

add_unlook_test(test_calibration_io
    test_calibration_io.cpp
)
set_tests_properties(test_calibration_io PROPERTIES LABELS "unit;io")

# Utility tests
add_unlook_test(test_image_utils
    test_image_utils.cpp
)
set_tests_properties(test_image_utils PROPERTIES LABELS "unit;utils")

# Create custom target for unit tests only
add_custom_target(test_unit
    DEPENDS
        test_logger
        test_config
        test_timer
        test_types
        test_camera_manager
        test_stereo_camera
        test_pattern_detector
        test_stereo_calibration
        test_calibration_validation
        test_stereo_matcher
        test_depth_processor
        test_rectification
        test_i2c_controller
        test_led_controller
        test_geometry
        test_transforms
        test_file_io
        test_calibration_io
        test_image_utils
)

message(STATUS "Unit tests configured: ${CMAKE_CURRENT_SOURCE_DIR}")