# Hardware subsystem CMakeLists.txt
# AS1170 LED Driver and VCSEL Projector Control System

cmake_minimum_required(VERSION 3.16)

# Find required dependencies
# I2C libraries are installed via libi2c-dev package
# No pkg-config needed, link directly to standard i2c library

# Create hardware library
set(HARDWARE_SOURCES
    AS1170Controller.cpp
    AS1170DualVCSELController.cpp
    LEDSyncManager.cpp
    LEDThermalManager.cpp
    VCSELProjector.cpp
    StructuredLightSystem.cpp
)

set(HARDWARE_HEADERS
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/AS1170Controller.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/AS1170DualVCSELController.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/LEDSyncManager.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/LEDThermalManager.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/VCSELProjector.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/hardware/StructuredLightSystem.hpp
)

# Create hardware static library
add_library(unlook_hardware STATIC ${HARDWARE_SOURCES} ${HARDWARE_HEADERS})

# Include directories
target_include_directories(unlook_hardware
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link libraries
target_link_libraries(unlook_hardware
    PUBLIC
        ${OpenCV_LIBS}
    PRIVATE
        i2c
        pthread
)

# Compiler flags
target_compile_features(unlook_hardware PUBLIC cxx_std_17)

target_compile_options(unlook_hardware
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
        $<$<CONFIG:Debug>:-g -O0>
        $<$<CONFIG:Release>:-O3 -DNDEBUG>
        ${I2C_CFLAGS_OTHER}
)

# Conditional compilation for hardware features
if(ENABLE_HARDWARE_GPIO)
    target_compile_definitions(unlook_hardware PRIVATE UNLOOK_ENABLE_GPIO)
endif()

if(ENABLE_HARDWARE_I2C)
    target_compile_definitions(unlook_hardware PRIVATE UNLOOK_ENABLE_I2C)
endif()

# Platform-specific libraries
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(unlook_hardware PRIVATE rt)
endif()

# Installation
install(TARGETS unlook_hardware
    EXPORT UnlookTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${HARDWARE_HEADERS}
    DESTINATION include/unlook/hardware
)

# Hardware examples and tests
# if(BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

# Create hardware examples
if(BUILD_EXAMPLES)
    # AS1170 Test Example
    # add_executable(as1170_test examples/as1170_test.cpp)
    # target_link_libraries(as1170_test unlook_hardware)

    # VCSEL Projector Example
    # add_executable(vcsel_example examples/vcsel_example.cpp)
    # target_link_libraries(vcsel_example unlook_hardware)

    # Structured Light System Example
    # add_executable(structured_light_example examples/structured_light_example.cpp)
    # target_link_libraries(structured_light_example unlook_hardware)

    # Install examples
    # install(TARGETS as1170_test vcsel_example structured_light_example
    #     RUNTIME DESTINATION bin/examples
    # )
endif()

# Hardware validation tests
if(BUILD_TESTS)
    # AS1170 Hardware Test
    add_executable(test_as1170_hardware tests/test_as1170_hardware.cpp)
    target_link_libraries(test_as1170_hardware unlook_hardware gtest gtest_main)

    # LED Sync Test
    add_executable(test_led_sync tests/test_led_sync.cpp)
    target_link_libraries(test_led_sync unlook_hardware gtest gtest_main)

    # Thermal Management Test
    add_executable(test_thermal_manager tests/test_thermal_manager.cpp)
    target_link_libraries(test_thermal_manager unlook_hardware gtest gtest_main)

    # VCSEL Integration Test
    add_executable(test_vcsel_integration tests/test_vcsel_integration.cpp)
    target_link_libraries(test_vcsel_integration unlook_hardware gtest gtest_main)

    # Register tests with CTest
    add_test(NAME AS1170Hardware COMMAND test_as1170_hardware)
    add_test(NAME LEDSync COMMAND test_led_sync)
    add_test(NAME ThermalManager COMMAND test_thermal_manager)
    add_test(NAME VCSELIntegration COMMAND test_vcsel_integration)

    # Install tests
    install(TARGETS test_as1170_hardware test_led_sync test_thermal_manager test_vcsel_integration
        RUNTIME DESTINATION bin/tests
    )
endif()

# Documentation
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        set(DOXYGEN_INPUT_DIR ${CMAKE_SOURCE_DIR}/include/unlook/hardware)
        set(DOXYGEN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/docs/hardware)
        configure_file(${CMAKE_SOURCE_DIR}/docs/Doxyfile.hardware.in
                       ${CMAKE_BINARY_DIR}/Doxyfile.hardware @ONLY)

        add_custom_target(docs_hardware
            COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile.hardware
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Generating hardware API documentation with Doxygen"
            VERBATIM
        )

        add_dependencies(docs docs_hardware)
    endif()
endif()

# Package configuration
set(HARDWARE_VERSION_MAJOR 1)
set(HARDWARE_VERSION_MINOR 0)
set(HARDWARE_VERSION_PATCH 0)
set(HARDWARE_VERSION "${HARDWARE_VERSION_MAJOR}.${HARDWARE_VERSION_MINOR}.${HARDWARE_VERSION_PATCH}")

# configure_file(
#     ${CMAKE_SOURCE_DIR}/src/hardware/hardware_version.h.in
#     ${CMAKE_BINARY_DIR}/include/unlook/hardware/hardware_version.h
#     @ONLY
# )

# Print configuration summary
message(STATUS "Hardware subsystem configuration:")
message(STATUS "  Version: ${HARDWARE_VERSION}")
message(STATUS "  GPIO Support: ${ENABLE_HARDWARE_GPIO}")
message(STATUS "  I2C Support: ${ENABLE_HARDWARE_I2C}")
message(STATUS "  Examples: ${BUILD_EXAMPLES}")
message(STATUS "  Tests: ${BUILD_TESTS}")
message(STATUS "  Documentation: ${BUILD_DOCS}")

# Compiler-specific optimizations
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(unlook_hardware PRIVATE
        $<$<CONFIG:Release>:-flto -ffast-math>
    )
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(unlook_hardware PRIVATE
        $<$<CONFIG:Release>:-flto -ffast-math>
    )
endif()

# ARM-specific optimizations for Raspberry Pi
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    target_compile_options(unlook_hardware PRIVATE
        -mcpu=cortex-a76  # CM5 optimization
        -mtune=cortex-a76
        -ftree-vectorize
    )
    message(STATUS "  ARM64 optimizations enabled for Raspberry Pi CM5")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
    target_compile_options(unlook_hardware PRIVATE
        -mcpu=cortex-a72  # CM4 optimization
        -mtune=cortex-a72
        -mfpu=neon-vfpv4
        -ftree-vectorize
    )
    message(STATUS "  ARM32 optimizations enabled for Raspberry Pi CM4")
endif()

# Hardware-specific definitions (FINAL AS1170 CONFIGURATION)
target_compile_definitions(unlook_hardware PRIVATE
    UNLOOK_HARDWARE_VERSION_MAJOR=${HARDWARE_VERSION_MAJOR}
    UNLOOK_HARDWARE_VERSION_MINOR=${HARDWARE_VERSION_MINOR}
    UNLOOK_HARDWARE_VERSION_PATCH=${HARDWARE_VERSION_PATCH}
    # Final AS1170 hardware configuration values:
    UNLOOK_AS1170_I2C_BUS=1          # I2C Bus 1 (confirmed working)
    UNLOOK_AS1170_I2C_ADDRESS=0x30   # I2C Address 0x30 (final production)
    UNLOOK_AS1170_STROBE_GPIO=19     # GPIO 19 (final strobe control pin)
)

# Safety and debugging features
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(unlook_hardware PRIVATE
        UNLOOK_ENABLE_HARDWARE_DEBUGGING
        UNLOOK_ENABLE_SAFETY_CHECKS
        UNLOOK_ENABLE_THERMAL_MONITORING
    )
endif()

# Thread safety
target_compile_definitions(unlook_hardware PRIVATE
    _REENTRANT
    _THREAD_SAFE
)

# Hardware abstraction layer defines
if(UNIX AND NOT APPLE)
    target_compile_definitions(unlook_hardware PRIVATE
        UNLOOK_PLATFORM_LINUX
        UNLOOK_GPIO_SYSFS_PATH="/sys/class/gpio"
        UNLOOK_I2C_DEV_PATH="/dev/i2c"
    )
endif()

# Create pkg-config file
# configure_file(
#     ${CMAKE_SOURCE_DIR}/src/hardware/unlook-hardware.pc.in
#     ${CMAKE_BINARY_DIR}/unlook-hardware.pc
#     @ONLY
# )

# install(FILES ${CMAKE_BINARY_DIR}/unlook-hardware.pc
#     DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
# )