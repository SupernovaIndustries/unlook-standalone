# Hardware Tests - Real hardware validation
# These tests require actual hardware and should be run on target systems

# Camera hardware tests
add_unlook_test(test_camera_hardware
    test_camera_hardware.cpp
)
set_tests_properties(test_camera_hardware PROPERTIES 
    LABELS "hardware;camera"
    TIMEOUT 30
)

# Stereo synchronization hardware test
add_unlook_test(test_stereo_sync_hardware
    test_stereo_sync_hardware.cpp
)
set_tests_properties(test_stereo_sync_hardware PROPERTIES 
    LABELS "hardware;camera;sync"
    TIMEOUT 60
)

# I2C hardware tests (LED controller)
add_unlook_test(test_i2c_hardware
    test_i2c_hardware.cpp
)
set_tests_properties(test_i2c_hardware PROPERTIES 
    LABELS "hardware;i2c;led"
    TIMEOUT 10
)

# GPIO hardware tests
add_unlook_test(test_gpio_hardware
    test_gpio_hardware.cpp  
)
set_tests_properties(test_gpio_hardware PROPERTIES 
    LABELS "hardware;gpio"
    TIMEOUT 10
)

# LED system hardware test
add_unlook_test(test_led_hardware
    test_led_hardware.cpp
)
set_tests_properties(test_led_hardware PROPERTIES 
    LABELS "hardware;led;timing"
    TIMEOUT 30
)

# Camera-LED synchronization test
add_unlook_test(test_camera_led_sync
    test_camera_led_sync.cpp
)
set_tests_properties(test_camera_led_sync PROPERTIES 
    LABELS "hardware;camera;led;sync"
    TIMEOUT 60
)

# Precision timing tests
add_unlook_test(test_timing_precision
    test_timing_precision.cpp
)
set_tests_properties(test_timing_precision PROPERTIES 
    LABELS "hardware;timing;precision"
    TIMEOUT 30
)

# Temperature monitoring test
add_unlook_test(test_temperature_monitoring
    test_temperature_monitoring.cpp
)
set_tests_properties(test_temperature_monitoring PROPERTIES 
    LABELS "hardware;temperature;safety"
    TIMEOUT 20
)

# Full system integration hardware test
add_unlook_test(test_full_system_hardware
    test_full_system_hardware.cpp
)
set_tests_properties(test_full_system_hardware PROPERTIES 
    LABELS "hardware;integration;full_system"
    TIMEOUT 120
)

# Hardware validation script
add_unlook_test(test_hardware_validation
    test_hardware_validation.cpp
)
set_tests_properties(test_hardware_validation PROPERTIES 
    LABELS "hardware;validation;comprehensive"
    TIMEOUT 180
)

# Create custom target for hardware tests
add_custom_target(test_hardware
    DEPENDS
        test_camera_hardware
        test_stereo_sync_hardware
        test_i2c_hardware
        test_gpio_hardware
        test_led_hardware
        test_camera_led_sync
        test_timing_precision
        test_temperature_monitoring
        test_full_system_hardware
        test_hardware_validation
)

# Hardware-specific test runner script
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/run_hardware_tests.sh.in
    ${CMAKE_CURRENT_BINARY_DIR}/run_hardware_tests.sh
    @ONLY
)

install(PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/run_hardware_tests.sh
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    OPTIONAL
)

message(STATUS "Hardware tests configured - require real hardware")
message(STATUS "  Run with: make test_hardware")
message(STATUS "  Note: Tests may require root privileges for hardware access")