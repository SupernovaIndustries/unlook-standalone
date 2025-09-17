/**
 * Test program for VCSEL integration with depth capture
 *
 * This validates:
 * - VCSEL initialization and configuration
 * - Synchronization with camera capture
 * - Thermal protection monitoring
 * - Safe shutdown on exit
 */

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <signal.h>

#include "unlook/hardware/VCSELProjector.hpp"
#include "unlook/camera/HardwareSyncCapture.hpp"

using namespace unlook;

// Global flag for clean shutdown
std::atomic<bool> should_exit(false);

void signal_handler(int sig) {
    std::cout << "\n[TEST] Shutdown signal received, cleaning up..." << std::endl;
    should_exit = true;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   UNLOOK VCSEL Integration Test" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Setup signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 1. Initialize VCSEL Projector
    std::cout << "[TEST] Initializing VCSEL projector..." << std::endl;
    auto vcsel = std::make_shared<hardware::VCSELProjector>();

    hardware::VCSELProjector::ProjectorConfig vcsel_config;
    vcsel_config.mode = hardware::VCSELProjector::ProjectionMode::DEPTH_CAPTURE;
    vcsel_config.pattern = hardware::VCSELProjector::PatternType::DOTS_15K;
    vcsel_config.vcsel_current_ma = 250;  // Safe operating current
    vcsel_config.flood_current_ma = 150;
    vcsel_config.enable_flood_assist = true;
    vcsel_config.projection_duration_ms = 50;
    vcsel_config.cool_down_delay_ms = 100;
    vcsel_config.max_duty_cycle = 0.3f;
    vcsel_config.enable_thermal_protection = true;
    vcsel_config.max_temperature_c = 70.0f;
    vcsel_config.thermal_throttle_temp_c = 65.0f;
    vcsel_config.enable_camera_sync = true;

    if (!vcsel->initialize(vcsel_config)) {
        std::cerr << "[ERROR] Failed to initialize VCSEL projector" << std::endl;
        std::cerr << "Make sure (FINAL HARDWARE CONFIG):" << std::endl;
        std::cerr << "  - I2C bus 1 is available (final confirmed)" << std::endl;
        std::cerr << "  - AS1170 is connected at address 0x30 (final production)" << std::endl;
        std::cerr << "  - GPIO 19 is available for strobe control (final pin)" << std::endl;
        return 1;
    }

    std::cout << "[TEST] VCSEL projector initialized successfully" << std::endl;

    // Set up thermal monitoring callback
    vcsel->setThermalCallback([](bool thermal_active, float temperature_c) {
        if (thermal_active) {
            std::cout << "[THERMAL] WARNING: Thermal protection active! Temperature: "
                     << temperature_c << "°C" << std::endl;
        } else {
            std::cout << "[THERMAL] Temperature: " << temperature_c << "°C (normal)" << std::endl;
        }
    });

    // Set up error callback
    vcsel->setErrorCallback([](const std::string& error) {
        std::cerr << "[VCSEL ERROR] " << error << std::endl;
    });

    // 2. Perform self-test
    std::cout << "\n[TEST] Running VCSEL self-test..." << std::endl;
    if (vcsel->performSelfTest()) {
        std::cout << "[TEST] Self-test PASSED" << std::endl;
    } else {
        std::cerr << "[TEST] Self-test FAILED" << std::endl;
        vcsel->shutdown();
        return 1;
    }

    // 3. Get initial status
    auto status = vcsel->getStatus();
    std::cout << "\n[TEST] Initial VCSEL Status:" << std::endl;
    std::cout << "  - Initialized: " << (status.initialized ? "YES" : "NO") << std::endl;
    std::cout << "  - Hardware OK: " << (status.hardware_ok ? "YES" : "NO") << std::endl;
    std::cout << "  - Thermal OK: " << (status.thermal_ok ? "YES" : "NO") << std::endl;
    std::cout << "  - Temperature: " << status.temperature_c << "°C" << std::endl;

    // 4. Enable depth capture mode
    std::cout << "\n[TEST] Enabling depth capture mode..." << std::endl;
    if (!vcsel->enableDepthCapture()) {
        std::cerr << "[ERROR] Failed to enable depth capture mode" << std::endl;
        vcsel->shutdown();
        return 1;
    }

    // 5. Test projection cycles
    std::cout << "\n[TEST] Testing VCSEL projection cycles..." << std::endl;
    std::cout << "Press Ctrl+C to stop the test" << std::endl;
    std::cout << std::endl;

    int cycle = 0;
    while (!should_exit && cycle < 10) {
        cycle++;
        std::cout << "[CYCLE " << cycle << "] Triggering VCSEL projection..." << std::endl;

        // Trigger structured light capture
        bool success = vcsel->triggerStructuredLightCapture(
            15000,  // 15ms exposure time
            [cycle](hardware::VCSELProjector::PatternType pattern, uint64_t timestamp_ns) {
                std::cout << "[CYCLE " << cycle << "] Projection complete at timestamp: "
                         << timestamp_ns << " ns" << std::endl;
            }
        );

        if (!success) {
            std::cerr << "[ERROR] Failed to trigger VCSEL projection" << std::endl;
            break;
        }

        // Wait for projection to complete plus cooldown
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Check status after each cycle
        status = vcsel->getStatus();
        std::cout << "[STATUS] Temp: " << status.temperature_c << "°C, "
                 << "Cycles: " << status.projection_cycles << ", "
                 << "Duty: " << (status.current_duty_cycle * 100) << "%" << std::endl;

        // Check for thermal issues
        if (status.thermal_protection_active) {
            std::cout << "[WARNING] Thermal protection active, waiting for cooldown..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }

        // Wait between cycles
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // 6. Get final performance metrics
    std::cout << "\n[TEST] Final Performance Metrics:" << std::endl;
    auto metrics = vcsel->getPerformanceMetrics();
    std::cout << "  - Total projection time: " << metrics.total_projection_time_ms << " ms" << std::endl;
    std::cout << "  - Successful syncs: " << metrics.successful_syncs << std::endl;
    std::cout << "  - Failed syncs: " << metrics.failed_syncs << std::endl;
    std::cout << "  - Average sync error: " << metrics.avg_sync_error_us << " µs" << std::endl;
    std::cout << "  - Max sync error: " << metrics.max_sync_error_us << " µs" << std::endl;

    // 7. Run diagnostics
    std::cout << "\n[TEST] Running final diagnostics..." << std::endl;
    auto diag = vcsel->runDiagnostics();
    std::cout << "Diagnostic Results:" << std::endl;
    std::cout << "  - I2C communication: " << (diag.i2c_communication ? "OK" : "FAIL") << std::endl;
    std::cout << "  - GPIO control: " << (diag.gpio_control ? "OK" : "FAIL") << std::endl;
    std::cout << "  - LED1 functional: " << (diag.led1_functional ? "OK" : "FAIL") << std::endl;
    std::cout << "  - LED2 functional: " << (diag.led2_functional ? "OK" : "FAIL") << std::endl;
    std::cout << "  - Thermal sensor: " << (diag.thermal_sensor ? "OK" : "FAIL") << std::endl;
    std::cout << "  - Sync timing: " << (diag.sync_timing ? "OK" : "FAIL") << std::endl;

    if (!diag.error_details.empty()) {
        std::cout << "Errors:" << std::endl;
        for (const auto& error : diag.error_details) {
            std::cout << "  - " << error << std::endl;
        }
    }

    // 8. Clean shutdown
    std::cout << "\n[TEST] Shutting down VCSEL projector..." << std::endl;
    vcsel->disableProjection();
    vcsel->shutdown();

    std::cout << "\n[TEST] Test completed successfully!" << std::endl;
    return 0;
}