/**
 * BMI270 IMU and Stability Detector Example
 *
 * Demonstrates real-time stability detection for handheld 3D scanning.
 * Shows how to:
 * - Initialize BMI270 IMU driver
 * - Read IMU data in real-time
 * - Detect scanner stability with StabilityDetector
 * - Display stability metrics and scores
 *
 * Usage:
 *   ./bmi270_stability_example [--mock]
 *
 * Options:
 *   --mock    Run in mock mode (simulated IMU data, no hardware required)
 */

#include <unlook/hardware/BMI270Driver.hpp>
#include <unlook/hardware/StabilityDetector.hpp>
#include <unlook/core/Logger.hpp>

#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <signal.h>

using namespace unlook::hardware;

// Global flag for clean shutdown
static volatile bool running = true;

void signalHandler(int signal) {
    (void)signal;
    running = false;
}

void printHeader() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║          BMI270 IMU Stability Detector - Real-Time Monitoring           ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════════════════════╝\n\n";
}

void printStatus(const BMI270Driver::IMUData& imu_data,
                 const StabilityDetector::StabilityStatus& stability_status) {
    // Clear screen (ANSI escape code)
    std::cout << "\033[2J\033[1;1H";

    printHeader();

    // IMU Data
    std::cout << "┌─ IMU Sensor Data ─────────────────────────────────────────────────────────┐\n";
    std::cout << "│ Gyroscope (deg/sec):                                                      │\n";
    std::cout << "│   X: " << std::setw(8) << std::fixed << std::setprecision(2) << imu_data.gyro_x
              << "   Y: " << std::setw(8) << imu_data.gyro_y
              << "   Z: " << std::setw(8) << imu_data.gyro_z << "                      │\n";
    std::cout << "│                                                                           │\n";
    std::cout << "│ Accelerometer (m/s²):                                                     │\n";
    std::cout << "│   X: " << std::setw(8) << imu_data.accel_x
              << "   Y: " << std::setw(8) << imu_data.accel_y
              << "   Z: " << std::setw(8) << imu_data.accel_z << "                      │\n";
    std::cout << "└───────────────────────────────────────────────────────────────────────────┘\n\n";

    // Stability Status
    std::cout << "┌─ Stability Analysis ──────────────────────────────────────────────────────┐\n";
    std::cout << "│ Status: ";
    if (stability_status.is_stable) {
        std::cout << "\033[1;32m✓ STABLE\033[0m      ";
    } else {
        std::cout << "\033[1;31m✗ UNSTABLE\033[0m    ";
    }
    std::cout << "  Stable Duration: " << std::setw(5) << stability_status.stable_duration_ms << " ms";
    std::cout << "                  │\n";

    // Stability score bar
    std::cout << "│                                                                           │\n";
    std::cout << "│ Stability Score: " << std::setw(5) << std::fixed << std::setprecision(1)
              << (stability_status.stability_score * 100.0f) << "%  ";

    // Draw progress bar
    int bar_width = 40;
    int filled = static_cast<int>(stability_status.stability_score * bar_width);
    std::cout << "[";
    for (int i = 0; i < bar_width; ++i) {
        if (i < filled) {
            std::cout << "█";
        } else {
            std::cout << "░";
        }
    }
    std::cout << "]  │\n";

    std::cout << "│                                                                           │\n";
    std::cout << "│ Gyro Magnitude:  " << std::setw(6) << std::setprecision(3)
              << stability_status.current_gyro_magnitude << " deg/s  ";
    std::cout << "  Accel Variance: " << std::setw(6) << stability_status.current_accel_variance
              << " m/s²       │\n";

    std::cout << "│ History Samples: " << std::setw(4) << stability_status.samples_in_history
              << "                                                       │\n";
    std::cout << "└───────────────────────────────────────────────────────────────────────────┘\n\n";

    // Instructions
    std::cout << "Press Ctrl+C to exit...\n";
}

int main(int argc, char** argv) {
    // Install signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Check for mock mode
    bool mock_mode = false;
    if (argc > 1 && std::string(argv[1]) == "--mock") {
        mock_mode = true;
        std::cout << "Running in MOCK MODE (simulated IMU data)\n";
    }

    // Initialize logger
    auto& logger = unlook::core::Logger::getInstance();
    logger.info("BMI270 Stability Detector Example Starting...");

    // Create BMI270 driver
    auto bmi270 = BMI270Driver::getInstance();

    // Configure BMI270
    BMI270Driver::BMI270Config imu_config;
    imu_config.i2c_bus = 1;
    imu_config.i2c_address = 0x69;
    imu_config.gyro_range_dps = 500;
    imu_config.accel_range_g = 2;
    imu_config.sample_rate_hz = 100;
    imu_config.enable_mock_mode = mock_mode;

    // Initialize IMU
    if (!bmi270->initialize(imu_config)) {
        std::cerr << "ERROR: Failed to initialize BMI270 driver\n";
        std::cerr << "Try running with --mock flag for simulated data\n";
        return 1;
    }

    logger.info("BMI270 initialized successfully");

    // Create stability detector
    auto stability_detector = std::make_shared<StabilityDetector>(bmi270);

    // Configure stability parameters
    StabilityDetector::StabilityParams stability_params;
    stability_params.gyro_threshold_dps = 0.5f;      // 0.5 deg/sec threshold
    stability_params.accel_variance_threshold = 0.1f; // 0.1 m/s² variance
    stability_params.stable_duration_ms = 500;        // 500ms stable duration
    stability_params.history_window_ms = 1000;        // 1 second history
    stability_params.gyro_weight = 0.7f;              // 70% gyro contribution
    stability_params.accel_weight = 0.3f;             // 30% accel contribution

    // Initialize stability detector
    if (!stability_detector->initialize(stability_params)) {
        std::cerr << "ERROR: Failed to initialize StabilityDetector\n";
        return 1;
    }

    logger.info("StabilityDetector initialized successfully");

    // Main loop - update at 30 Hz for smooth display
    auto last_update = std::chrono::steady_clock::now();
    const auto update_interval = std::chrono::milliseconds(33);  // ~30 Hz

    printHeader();
    std::cout << "Starting stability monitoring...\n\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));

    while (running) {
        auto now = std::chrono::steady_clock::now();

        // Update stability detector
        if (!stability_detector->update()) {
            std::cerr << "ERROR: Failed to update stability detector\n";
            break;
        }

        // Get latest IMU data
        BMI270Driver::IMUData imu_data;
        if (!bmi270->readIMUData(imu_data)) {
            std::cerr << "ERROR: Failed to read IMU data\n";
            break;
        }

        // Get stability status
        auto stability_status = stability_detector->getStatus();

        // Display status
        if (now - last_update >= update_interval) {
            printStatus(imu_data, stability_status);
            last_update = now;
        }

        // Sleep to maintain update rate
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100 Hz IMU sampling
    }

    // Cleanup
    std::cout << "\n\nShutting down...\n";
    stability_detector->shutdown();
    bmi270->shutdown();

    logger.info("BMI270 Stability Detector Example Complete");

    return 0;
}
