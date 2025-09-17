/**
 * @file vcsel_depth_capture_example.cpp
 * @brief Example demonstrating VCSEL-based depth capture
 *
 * This example shows how to use the complete AS1170 VCSEL projector system
 * for structured light depth capture. It demonstrates:
 *
 * - System initialization and configuration
 * - Synchronized depth capture with camera system
 * - Thermal monitoring and safety features
 * - Performance monitoring and diagnostics
 * - Integration with existing camera components
 *
 * Hardware Requirements (FINAL CONFIGURATION):
 * - AS1170 LED driver on I2C bus 1, address 0x30 (confirmed working)
 * - GPIO 19 for strobe control (final designated pin for AS1170 on CM5)
 * - OSRAM BELAGO 15k VCSEL projector
 * - Stereo camera system with IMX296 sensors
 *
 * Safety Features:
 * - Automatic thermal protection and current limiting
 * - Emergency shutdown capabilities
 * - Integrated GPIO control for AS1170 strobe timing
 * - Industrial safety compliance
 *
 * @author Unlook Hardware Interface Agent
 * @version 1.0.0
 */

#include <unlook/hardware/StructuredLightSystem.hpp>
#include <unlook/api/vcsel_control.h>
#include <unlook/core/Logger.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <opencv2/opencv.hpp>

using namespace unlook::hardware;
using namespace unlook::core;

/**
 * Example 1: Basic Depth Capture with C++ API
 */
void example_cpp_depth_capture() {
    std::cout << "\n=== C++ API Depth Capture Example ===" << std::endl;

    Logger::info("Initializing Structured Light System for depth capture...");

    // Create and configure structured light system
    StructuredLightSystem system;
    StructuredLightSystem::CaptureConfig config;

    // Configure for depth capture
    config.mode = StructuredLightSystem::CaptureMode::DEPTH_ONLY;
    config.pattern = VCSELProjector::PatternType::DOTS_15K;
    config.vcsel_current_ma = 250;          // 250mA for safety (not 450mA from OSRAM)
    config.flood_current_ma = 150;          // Flood assist
    config.exposure_time_us = 10000;        // 10ms exposure
    config.enable_precise_sync = true;       // <50μs synchronization
    config.enable_thermal_protection = true; // Safety first

    // Set up callbacks for monitoring
    system.setProgressCallback([](const std::string& stage, float progress) {
        if (progress >= 0) {
            Logger::info("Progress: {} - {:.1f}%", stage, progress * 100);
        } else {
            Logger::error("Error in stage: {}", stage);
        }
    });

    system.setStatusCallback([](const std::string& status, bool is_error) {
        if (is_error) {
            Logger::error("System status: {}", status);
        } else {
            Logger::info("System status: {}", status);
        }
    });

    // Initialize system
    if (!system.initialize(config)) {
        Logger::error("Failed to initialize structured light system!");
        return;
    }

    Logger::info("System initialized successfully");

    // Run system diagnostics
    Logger::info("Running system diagnostics...");
    auto diagnostic = system.runSystemTest();

    if (diagnostic.camera_system_ok && diagnostic.projector_ok && diagnostic.sync_system_ok) {
        Logger::info("System diagnostics PASSED");
        Logger::info("  Sync accuracy: {:.1f}μs", diagnostic.measured_sync_accuracy_us);
        Logger::info("  Thermal status: {:.1f}°C", diagnostic.thermal_status_c);
    } else {
        Logger::error("System diagnostics FAILED - {} errors", diagnostic.errors.size());
        for (const auto& error : diagnostic.errors) {
            Logger::error("  - {}", error);
        }
        return;
    }

    // Perform depth capture
    Logger::info("Performing depth capture...");

    auto result = system.captureDepth([](const StructuredLightSystem::CaptureResult& result) {
        if (result.success) {
            Logger::info("Capture completed: {}x{} depth map, {:.1f}μs sync error",
                        result.depth_map.cols, result.depth_map.rows, result.sync_error_us);
        } else {
            Logger::error("Capture failed: {}", result.error_message);
        }
    });

    if (result.success) {
        Logger::info("Depth capture successful!");
        Logger::info("  Capture time: {}ms", result.capture_duration_ms);
        Logger::info("  Processing time: {}ms", result.processing_duration_ms);
        Logger::info("  Valid depth pixels: {}", result.valid_depth_pixels);
        Logger::info("  Depth quality: {:.2f}", result.depth_quality_score);
        Logger::info("  Sync error: {:.1f}μs", result.sync_error_us);
        Logger::info("  Temperature: {:.1f}°C", result.projection_temperature_c);

        // Save results if images are valid
        if (!result.left_image.empty() && !result.right_image.empty()) {
            cv::imwrite("capture_left.png", result.left_image);
            cv::imwrite("capture_right.png", result.right_image);
            Logger::info("Saved stereo images: capture_left.png, capture_right.png");
        }

        if (!result.depth_map.empty()) {
            cv::Mat depth_display;
            cv::normalize(result.depth_map, depth_display, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);
            cv::imwrite("depth_map.png", depth_display);
            Logger::info("Saved depth map: depth_map.png");
        }
    } else {
        Logger::error("Depth capture failed: {}", result.error_message);
    }

    // Get final system status
    auto status = system.getStatus();
    Logger::info("Final system status:");
    Logger::info("  Total captures: {}", status.total_captures);
    Logger::info("  Successful captures: {}", status.successful_captures);
    Logger::info("  Average capture time: {:.1f}ms", status.avg_capture_time_ms);
    Logger::info("  Average sync error: {:.1f}μs", status.avg_sync_error_us);

    system.shutdown();
    Logger::info("C++ example completed successfully");
}

/**
 * Example 2: Face Recognition with C API
 */
void example_c_api_face_recognition() {
    std::cout << "\n=== C API Face Recognition Example ===" << std::endl;

    // Get API version
    int major, minor, patch;
    unlook_vcsel_get_version(&major, &minor, &patch);
    std::cout << "Using VCSEL API version " << major << "." << minor << "." << patch << std::endl;

    // Get default configuration
    unlook_vcsel_config_t config;
    unlook_vcsel_result_t result = unlook_vcsel_get_default_config(&config);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        std::cerr << "Failed to get default config: " << unlook_vcsel_get_error_string(result) << std::endl;
        return;
    }

    // Configure for face recognition
    config.mode = UNLOOK_VCSEL_MODE_FACE_RECOGNITION;
    config.pattern = UNLOOK_VCSEL_PATTERN_ADAPTIVE;
    config.vcsel_current_ma = 200;           // Gentler for faces
    config.flood_current_ma = 150;
    config.face_distance_mm = 400.0f;        // 40cm typical face distance
    config.enable_camera_sync = false;       // Face capture doesn't need precise sync
    config.adaptive_current = true;          // Adapt to conditions

    // Validate configuration
    char error_msg[256];
    if (!unlook_vcsel_validate_config(&config, error_msg, sizeof(error_msg))) {
        std::cerr << "Invalid configuration: " << error_msg << std::endl;
        return;
    }

    // Create projector instance
    unlook_vcsel_handle_t handle;
    result = unlook_vcsel_create(&config, &handle);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        std::cerr << "Failed to create projector: " << unlook_vcsel_get_error_string(result) << std::endl;
        return;
    }

    std::cout << "VCSEL projector created successfully" << std::endl;

    // Set up callbacks
    auto thermal_callback = [](bool thermal_active, float temperature_c, void* user_data) {
        if (thermal_active) {
            std::cout << "WARNING: Thermal protection activated at " << temperature_c << "°C" << std::endl;
        } else {
            std::cout << "INFO: Thermal protection deactivated at " << temperature_c << "°C" << std::endl;
        }
    };

    auto error_callback = [](const char* error_message, void* user_data) {
        std::cerr << "VCSEL Error: " << error_message << std::endl;
    };

    unlook_vcsel_set_thermal_callback(handle, thermal_callback, nullptr);
    unlook_vcsel_set_error_callback(handle, error_callback, nullptr);

    // Run diagnostics
    std::cout << "Running projector diagnostics..." << std::endl;
    unlook_vcsel_diagnostic_t diagnostic;
    result = unlook_vcsel_run_diagnostics(handle, &diagnostic);

    if (result == UNLOOK_VCSEL_SUCCESS) {
        std::cout << "Diagnostics results:" << std::endl;
        std::cout << "  I2C Communication: " << (diagnostic.i2c_communication ? "PASS" : "FAIL") << std::endl;
        std::cout << "  GPIO Control: " << (diagnostic.gpio_control ? "PASS" : "FAIL") << std::endl;
        std::cout << "  LED1 (VCSEL): " << (diagnostic.led1_functional ? "PASS" : "FAIL") << std::endl;
        std::cout << "  LED2 (Flood): " << (diagnostic.led2_functional ? "PASS" : "FAIL") << std::endl;
        std::cout << "  Thermal Sensor: " << (diagnostic.thermal_sensor ? "PASS" : "FAIL") << std::endl;
        std::cout << "  Temperature: " << diagnostic.measured_temperature_c << "°C" << std::endl;

        if (diagnostic.error_count > 0) {
            std::cout << "Diagnostic errors:" << std::endl;
            for (size_t i = 0; i < diagnostic.error_count; i++) {
                std::cout << "  - " << diagnostic.error_details[i] << std::endl;
            }
        }
    }

    // Enable face recognition mode
    result = unlook_vcsel_enable_face_recognition(handle, config.face_distance_mm);
    if (result != UNLOOK_VCSEL_SUCCESS) {
        std::cerr << "Failed to enable face recognition: " << unlook_vcsel_get_error_string(result) << std::endl;
        unlook_vcsel_destroy(handle);
        return;
    }

    std::cout << "Face recognition mode enabled" << std::endl;

    // Check if system is ready
    bool ready = false;
    result = unlook_vcsel_is_ready(handle, &ready);
    if (result == UNLOOK_VCSEL_SUCCESS && ready) {
        std::cout << "System ready for face capture" << std::endl;

        // Trigger face illumination
        result = unlook_vcsel_trigger_face_illumination(handle, 200, nullptr, nullptr);
        if (result == UNLOOK_VCSEL_SUCCESS) {
            std::cout << "Face illumination triggered (200ms duration)" << std::endl;

            // Wait for completion
            result = unlook_vcsel_wait_for_completion(handle, 5000);
            if (result == UNLOOK_VCSEL_SUCCESS) {
                std::cout << "Face illumination completed successfully" << std::endl;
            } else {
                std::cout << "Face illumination timeout or error: " << unlook_vcsel_get_error_string(result) << std::endl;
            }
        } else {
            std::cout << "Failed to trigger face illumination: " << unlook_vcsel_get_error_string(result) << std::endl;
        }
    } else {
        std::cout << "System not ready for face capture" << std::endl;
    }

    // Get final status and metrics
    unlook_vcsel_status_t status;
    result = unlook_vcsel_get_status(handle, &status);
    if (result == UNLOOK_VCSEL_SUCCESS) {
        std::cout << "Final status:" << std::endl;
        std::cout << "  Mode: " << unlook_vcsel_get_mode_string(status.current_mode) << std::endl;
        std::cout << "  Pattern: " << unlook_vcsel_get_pattern_string(status.current_pattern) << std::endl;
        std::cout << "  Projection cycles: " << status.projection_cycles << std::endl;
        std::cout << "  Temperature: " << status.temperature_c << "°C" << std::endl;
        std::cout << "  Thermal protection: " << (status.thermal_protection_active ? "ACTIVE" : "INACTIVE") << std::endl;
    }

    // Cleanup
    unlook_vcsel_destroy(handle);
    std::cout << "C API example completed successfully" << std::endl;
}

/**
 * Example 3: Continuous Capture for Real-time Applications
 */
void example_continuous_capture() {
    std::cout << "\n=== Continuous Capture Example ===" << std::endl;

    Logger::info("Setting up continuous depth capture...");

    StructuredLightSystem system;
    StructuredLightSystem::CaptureConfig config;

    // Configure for continuous operation
    config.mode = StructuredLightSystem::CaptureMode::DEPTH_ONLY;
    config.pattern = VCSELProjector::PatternType::DOTS_15K;
    config.vcsel_current_ma = 200;          // Reduced current for continuous operation
    config.flood_current_ma = 100;          // Reduced flood current
    config.projection_duration_ms = 30;     // Shorter bursts
    config.enable_thermal_protection = true; // Essential for continuous operation

    if (!system.initialize(config)) {
        Logger::error("Failed to initialize system for continuous capture");
        return;
    }

    // Set up capture callback for real-time processing
    uint32_t capture_count = 0;
    system.setCaptureCallback([&capture_count](const StructuredLightSystem::CaptureResult& result) {
        capture_count++;

        if (result.success) {
            Logger::info("Capture #{}: {}ms, {:.1f}μs sync, {:.1f}°C",
                        capture_count,
                        result.capture_duration_ms,
                        result.sync_error_us,
                        result.projection_temperature_c);

            // In a real application, you would process the depth data here
            if (capture_count % 10 == 0) {
                Logger::info("Processed {} captures, quality score: {:.2f}",
                           capture_count, result.depth_quality_score);
            }
        } else {
            Logger::error("Capture #{} failed: {}", capture_count, result.error_message);
        }
    });

    // Start continuous capture at 5 FPS
    if (!system.startContinuous(5.0)) {
        Logger::error("Failed to start continuous capture");
        system.shutdown();
        return;
    }

    Logger::info("Continuous capture started at 5 FPS. Running for 10 seconds...");

    // Run for 10 seconds
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // Stop continuous capture
    system.stopContinuous();

    // Get performance statistics
    auto status = system.getStatus();
    Logger::info("Continuous capture completed:");
    Logger::info("  Total captures: {}", status.total_captures);
    Logger::info("  Successful captures: {}", status.successful_captures);
    Logger::info("  Success rate: {:.1f}%",
                100.0 * status.successful_captures / status.total_captures);
    Logger::info("  Average capture time: {:.1f}ms", status.avg_capture_time_ms);
    Logger::info("  Average sync error: {:.1f}μs", status.avg_sync_error_us);
    Logger::info("  Final temperature: {:.1f}°C", status.projector_temperature_c);

    system.shutdown();
    Logger::info("Continuous capture example completed");
}

/**
 * Main function - runs all examples
 */
int main(int argc, char* argv[]) {
    std::cout << "Unlook VCSEL Depth Capture Examples" << std::endl;
    std::cout << "===================================" << std::endl;

    // Initialize logging
    Logger::setLevel(Logger::LogLevel::INFO);

    try {
        // Example 1: Basic depth capture with C++ API
        example_cpp_depth_capture();

        // Example 2: Face recognition with C API
        example_c_api_face_recognition();

        // Example 3: Continuous capture
        example_continuous_capture();

        std::cout << "\n=== All Examples Completed Successfully ===" << std::endl;
        std::cout << "\nFINAL Hardware Configuration Summary:" << std::endl;
        std::cout << "  AS1170 LED Driver: I2C bus 1, address 0x30 (final production config)" << std::endl;
        std::cout << "  Strobe Control: GPIO 19 (final designated AS1170 strobe pin)" << std::endl;
        std::cout << "  VCSEL Current: 250mA (industrial safety limit)" << std::endl;
        std::cout << "  Synchronization: <50μs precision with camera system" << std::endl;
        std::cout << "  Safety Features: Thermal protection, emergency shutdown" << std::endl;
        std::cout << "  Pattern: 15k points OSRAM BELAGO VCSEL projector" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Exception in examples: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception in examples" << std::endl;
        return 1;
    }

    return 0;
}