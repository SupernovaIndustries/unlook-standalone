/**
 * AS1170 Dual VCSEL Temporal Matching Test
 *
 * This example demonstrates triple frame capture with dual VCSEL projectors
 * for temporal stereo matching.
 *
 * Hardware Configuration:
 * - VCSEL1 (LED1): 2cm from LEFT camera, 15K dots, 200mA
 * - VCSEL2 (LED2): 2cm from RIGHT camera, 15K dots, 200mA
 * - AS1170: I2C bus 1, address 0x30, GPIO 19 strobe
 *
 * Temporal Sequence:
 * 1. Frame A: VCSEL1 ON (pattern from left)
 * 2. Frame B: VCSEL2 ON (pattern from right)
 * 3. Frame C: Both OFF (ambient for subtraction)
 *
 * Usage:
 *   ./test_dual_vcsel_temporal [num_captures]
 *
 * Example:
 *   ./test_dual_vcsel_temporal 10    # Capture 10 temporal sequences
 */

#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/Logger.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace unlook;

// Display statistics for a triple frame capture
void displayCaptureStats(const hardware::AS1170DualVCSELController::TripleFrameCapture& capture) {
    std::cout << "\n=== Temporal Capture Statistics ===" << std::endl;
    std::cout << "Valid: " << (capture.is_valid ? "YES" : "NO") << std::endl;
    std::cout << "Temperature: " << std::fixed << std::setprecision(1)
              << capture.temperature_c << " C" << std::endl;

    std::cout << "\nTiming:" << std::endl;
    std::cout << "  VCSEL1 activation: " << (capture.vcsel1_activation_us / 1000.0) << " ms" << std::endl;
    std::cout << "  VCSEL2 activation: " << (capture.vcsel2_activation_us / 1000.0) << " ms" << std::endl;
    std::cout << "  Total sequence: " << (capture.total_sequence_us / 1000.0) << " ms" << std::endl;

    std::cout << "\nSynchronization Errors:" << std::endl;
    std::cout << "  VCSEL1 sync: " << std::fixed << std::setprecision(3)
              << capture.vcsel1_sync_error_ms << " ms" << std::endl;
    std::cout << "  VCSEL2 sync: " << capture.vcsel2_sync_error_ms << " ms" << std::endl;
    std::cout << "  Ambient sync: " << capture.ambient_sync_error_ms << " ms" << std::endl;

    std::cout << "\nFrame Sizes:" << std::endl;
    std::cout << "  VCSEL1 Left: " << capture.frame_vcsel1_left.size() << std::endl;
    std::cout << "  VCSEL1 Right: " << capture.frame_vcsel1_right.size() << std::endl;
    std::cout << "  VCSEL2 Left: " << capture.frame_vcsel2_left.size() << std::endl;
    std::cout << "  VCSEL2 Right: " << capture.frame_vcsel2_right.size() << std::endl;
    std::cout << "  Ambient Left: " << capture.frame_ambient_left.size() << std::endl;
    std::cout << "  Ambient Right: " << capture.frame_ambient_right.size() << std::endl;
}

// Compute pattern visibility metric (simple std dev of pixel intensities)
double computePatternVisibility(const cv::Mat& image) {
    if (image.empty()) return 0.0;

    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);
    return stddev[0];
}

// Analyze pattern quality in captured frames
void analyzePatternQuality(const hardware::AS1170DualVCSELController::TripleFrameCapture& capture) {
    std::cout << "\n=== Pattern Quality Analysis ===" << std::endl;

    double vcsel1_left_vis = computePatternVisibility(capture.frame_vcsel1_left);
    double vcsel1_right_vis = computePatternVisibility(capture.frame_vcsel1_right);
    double vcsel2_left_vis = computePatternVisibility(capture.frame_vcsel2_left);
    double vcsel2_right_vis = computePatternVisibility(capture.frame_vcsel2_right);
    double ambient_left_vis = computePatternVisibility(capture.frame_ambient_left);
    double ambient_right_vis = computePatternVisibility(capture.frame_ambient_right);

    std::cout << "Pattern Visibility (std dev of intensity):" << std::endl;
    std::cout << "  VCSEL1 Left:  " << std::fixed << std::setprecision(2) << vcsel1_left_vis << std::endl;
    std::cout << "  VCSEL1 Right: " << vcsel1_right_vis << std::endl;
    std::cout << "  VCSEL2 Left:  " << vcsel2_left_vis << std::endl;
    std::cout << "  VCSEL2 Right: " << vcsel2_right_vis << std::endl;
    std::cout << "  Ambient Left: " << ambient_left_vis << std::endl;
    std::cout << "  Ambient Right: " << ambient_right_vis << std::endl;

    // Pattern enhancement (should be significantly higher with VCSEL ON)
    double vcsel1_enhancement = ((vcsel1_left_vis + vcsel1_right_vis) / 2.0) /
                                ((ambient_left_vis + ambient_right_vis) / 2.0 + 1e-6);
    double vcsel2_enhancement = ((vcsel2_left_vis + vcsel2_right_vis) / 2.0) /
                                ((ambient_left_vis + ambient_right_vis) / 2.0 + 1e-6);

    std::cout << "\nPattern Enhancement Factor:" << std::endl;
    std::cout << "  VCSEL1: " << std::setprecision(2) << vcsel1_enhancement << "x" << std::endl;
    std::cout << "  VCSEL2: " << vcsel2_enhancement << "x" << std::endl;

    if (vcsel1_enhancement < 1.2 || vcsel2_enhancement < 1.2) {
        std::cout << "\nWARNING: Pattern enhancement is low (<1.2x)" << std::endl;
        std::cout << "Check VCSEL alignment and current settings." << std::endl;
    } else {
        std::cout << "\nPattern quality: GOOD (enhancement >1.2x)" << std::endl;
    }
}

// Save captured frames to disk for analysis
void saveFrames(const hardware::AS1170DualVCSELController::TripleFrameCapture& capture,
                int capture_number, const std::string& output_dir = "temporal_captures") {

    // Create output directory
    std::string cmd = "mkdir -p " + output_dir;
    system(cmd.c_str());

    std::string prefix = output_dir + "/capture_" + std::to_string(capture_number);

    cv::imwrite(prefix + "_vcsel1_left.png", capture.frame_vcsel1_left);
    cv::imwrite(prefix + "_vcsel1_right.png", capture.frame_vcsel1_right);
    cv::imwrite(prefix + "_vcsel2_left.png", capture.frame_vcsel2_left);
    cv::imwrite(prefix + "_vcsel2_right.png", capture.frame_vcsel2_right);
    cv::imwrite(prefix + "_ambient_left.png", capture.frame_ambient_left);
    cv::imwrite(prefix + "_ambient_right.png", capture.frame_ambient_right);

    std::cout << "\nFrames saved to: " << prefix << "_*.png" << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    int num_captures = 1;
    if (argc > 1) {
        num_captures = std::atoi(argv[1]);
        if (num_captures < 1 || num_captures > 100) {
            std::cerr << "Invalid number of captures (valid: 1-100)" << std::endl;
            return 1;
        }
    }

    std::cout << "=== AS1170 Dual VCSEL Temporal Matching Test ===" << std::endl;
    std::cout << "Number of captures: " << num_captures << std::endl;

    // Initialize logger
    core::Logger::getInstance().setLevel(core::LogLevel::INFO);

    // Initialize camera system
    std::cout << "\nInitializing camera system..." << std::endl;
    auto camera_system = camera::CameraSystem::getInstance();

    camera::CameraConfig cam_config;
    cam_config.width = 1456;
    cam_config.height = 1088;
    cam_config.targetFps = 30.0;
    cam_config.enableSync = true;
    cam_config.autoExposure = true;

    if (!camera_system->initialize(cam_config)) {
        std::cerr << "ERROR: Failed to initialize camera system" << std::endl;
        return 1;
    }

    if (!camera_system->startCapture()) {
        std::cerr << "ERROR: Failed to start camera capture" << std::endl;
        return 1;
    }

    std::cout << "Camera system initialized successfully" << std::endl;

    // Initialize dual VCSEL controller
    std::cout << "\nInitializing dual VCSEL controller..." << std::endl;
    auto vcsel_controller = hardware::AS1170DualVCSELController::getInstance();

    hardware::AS1170DualVCSELController::VCSELConfig vcsel_config;
    vcsel_config.vcsel1_current_ma = 200;  // 200mA for VCSEL1
    vcsel_config.vcsel2_current_ma = 200;  // 200mA for VCSEL2
    vcsel_config.settle_time_ms = 50;      // 50ms settle time
    vcsel_config.capture_delay_ms = 10;    // 10ms capture delay
    vcsel_config.max_on_time_ms = 5000;    // 5 second max
    vcsel_config.enable_thermal_monitoring = true;
    vcsel_config.max_operating_temp_c = 70.0f;

    if (!vcsel_controller->initialize(camera_system, vcsel_config)) {
        std::cerr << "ERROR: Failed to initialize dual VCSEL controller" << std::endl;
        camera_system->stopCapture();
        camera_system->shutdown();
        return 1;
    }

    std::cout << "Dual VCSEL controller initialized successfully" << std::endl;

    // Display configuration
    auto config = vcsel_controller->getConfig();
    std::cout << "\n=== VCSEL Configuration ===" << std::endl;
    std::cout << "VCSEL1 current: " << config.vcsel1_current_ma << " mA" << std::endl;
    std::cout << "VCSEL2 current: " << config.vcsel2_current_ma << " mA" << std::endl;
    std::cout << "Settle time: " << config.settle_time_ms << " ms" << std::endl;
    std::cout << "Capture delay: " << config.capture_delay_ms << " ms" << std::endl;
    std::cout << "Max operating temp: " << config.max_operating_temp_c << " C" << std::endl;

    // Perform temporal captures
    std::cout << "\n=== Starting Temporal Captures ===" << std::endl;

    std::vector<double> capture_times;
    std::vector<double> temperatures;
    int successful_captures = 0;

    for (int i = 0; i < num_captures; i++) {
        std::cout << "\n--- Capture " << (i + 1) << " of " << num_captures << " ---" << std::endl;

        hardware::AS1170DualVCSELController::TripleFrameCapture capture;

        auto start_time = std::chrono::steady_clock::now();
        bool success = vcsel_controller->captureTemporalSequence(capture);
        auto end_time = std::chrono::steady_clock::now();

        if (!success) {
            std::cerr << "ERROR: Temporal capture " << (i + 1) << " failed" << std::endl;
            auto status = vcsel_controller->getStatus();
            std::cerr << "Error: " << status.error_message << std::endl;
            continue;
        }

        successful_captures++;

        double capture_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time).count();
        capture_times.push_back(capture_time_ms);
        temperatures.push_back(capture.temperature_c);

        // Display statistics
        displayCaptureStats(capture);
        analyzePatternQuality(capture);

        // Save frames (save only first capture by default)
        if (i == 0) {
            saveFrames(capture, i);
        }

        std::cout << "\nCapture wall time: " << capture_time_ms << " ms" << std::endl;

        // Wait between captures to allow thermal stabilization
        if (i < num_captures - 1) {
            std::cout << "\nWaiting 2 seconds for thermal stabilization..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // Display overall statistics
    if (successful_captures > 0) {
        std::cout << "\n=== Overall Statistics ===" << std::endl;
        std::cout << "Successful captures: " << successful_captures << " / " << num_captures << std::endl;

        double avg_time = std::accumulate(capture_times.begin(), capture_times.end(), 0.0) / capture_times.size();
        double avg_temp = std::accumulate(temperatures.begin(), temperatures.end(), 0.0) / temperatures.size();
        double max_temp = *std::max_element(temperatures.begin(), temperatures.end());

        std::cout << "Average capture time: " << std::fixed << std::setprecision(1)
                  << avg_time << " ms" << std::endl;
        std::cout << "Average temperature: " << std::setprecision(1) << avg_temp << " C" << std::endl;
        std::cout << "Maximum temperature: " << max_temp << " C" << std::endl;

        // Display VCSEL status
        auto status = vcsel_controller->getStatus();
        std::cout << "\n=== VCSEL Status ===" << std::endl;
        std::cout << "Total ON time: " << status.total_on_time_ms << " ms" << std::endl;
        std::cout << "Thermal throttling: " << (status.thermal_throttling ? "YES" : "NO") << std::endl;

        if (status.thermal_throttling) {
            std::cout << "\nWARNING: Thermal throttling detected!" << std::endl;
            std::cout << "Consider increasing settle time or reducing current." << std::endl;
        }
    }

    // Cleanup
    std::cout << "\n=== Shutting Down ===" << std::endl;
    vcsel_controller->shutdown();
    camera_system->stopCapture();
    camera_system->shutdown();

    std::cout << "\nTest complete!" << std::endl;

    return (successful_captures > 0) ? 0 : 1;
}
