/**
 * Example demonstrating temporal stereo matching with dual VCSEL projectors
 *
 * This example shows how to:
 * 1. Initialize the dual VCSEL controller
 * 2. Set up temporal stereo processing
 * 3. Capture and process triple frame sequences
 * 4. Validate depth quality metrics
 */

#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/stereo/TemporalStereoProcessor.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <unlook/camera/CameraSystem.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

using namespace unlook;

/**
 * Display processing results and statistics
 */
void displayResults(const stereo::TemporalStereoProcessor::TemporalStereoResult& result) {
    std::cout << "\n=== Temporal Stereo Processing Results ===" << std::endl;
    std::cout << std::fixed << std::setprecision(2);

    // Quality metrics
    std::cout << "\nQuality Metrics:" << std::endl;
    std::cout << "  Pattern Variance:    " << result.patternVariance
              << (result.patternVariance >= 200 ? " ✓" : " ✗ (target >= 200)") << std::endl;
    std::cout << "  Coverage Ratio:      " << (result.coverageRatio * 100) << "%"
              << (result.coverageRatio >= 0.6 && result.coverageRatio <= 0.8 ? " ✓" : " ✗ (target 60-80%)") << std::endl;
    std::cout << "  Average Confidence:  " << (result.avgConfidence * 100) << "%" << std::endl;
    std::cout << "  Valid Pixels:        " << result.validPixelCount << std::endl;

    // Timing information
    std::cout << "\nProcessing Times:" << std::endl;
    std::cout << "  Pattern Isolation:   " << result.isolationTimeMs << " ms" << std::endl;
    std::cout << "  Stereo Matching:     " << result.matchingTimeMs << " ms" << std::endl;
    std::cout << "  Total Processing:    " << result.totalTimeMs << " ms"
              << (result.totalTimeMs < 500 ? " ✓" : " ✗ (target < 500ms)") << std::endl;

    // Validation results
    std::cout << "\nValidation:" << std::endl;
    std::cout << "  Topologically Correct: " << (result.isTopologicallyCorrect ? "YES" : "NO") << std::endl;
    if (!result.validationMessage.empty()) {
        std::cout << "  Message: " << result.validationMessage << std::endl;
    }

    // Overall result
    std::cout << "\nOverall Result: " << (result.isValid() ? "VALID ✓" : "INVALID ✗") << std::endl;
}

/**
 * Visualize depth map with color coding
 */
void visualizeDepth(const cv::Mat& depthMap, const std::string& windowName = "Depth Map") {
    if (depthMap.empty()) {
        std::cerr << "Empty depth map, cannot visualize" << std::endl;
        return;
    }

    // Convert to 8-bit for visualization (scale to 0-3000mm range)
    cv::Mat depthVis;
    depthMap.convertTo(depthVis, CV_8U, 255.0 / 3000.0);

    // Apply color map
    cv::Mat coloredDepth;
    cv::applyColorMap(depthVis, coloredDepth, cv::COLORMAP_JET);

    // Add depth scale legend
    cv::putText(coloredDepth, "0mm", cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
    cv::putText(coloredDepth, "3000mm", cv::Point(10, coloredDepth.rows - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    cv::imshow(windowName, coloredDepth);
}

/**
 * Visualize isolated patterns
 */
void visualizePatterns(const stereo::TemporalStereoProcessor::TemporalStereoResult& result) {
    // Create mosaic of pattern images
    int h = result.isolatedPatternLeft.rows;
    int w = result.isolatedPatternLeft.cols;

    cv::Mat mosaic(h * 2, w * 3, CV_8UC1, cv::Scalar(0));

    // Top row: VCSEL1 patterns and combined left
    if (!result.vcsel1PatternLeft.empty())
        result.vcsel1PatternLeft.copyTo(mosaic(cv::Rect(0, 0, w, h)));
    if (!result.vcsel1PatternRight.empty())
        result.vcsel1PatternRight.copyTo(mosaic(cv::Rect(w, 0, w, h)));
    if (!result.isolatedPatternLeft.empty())
        result.isolatedPatternLeft.copyTo(mosaic(cv::Rect(2*w, 0, w, h)));

    // Bottom row: VCSEL2 patterns and combined right
    if (!result.vcsel2PatternLeft.empty())
        result.vcsel2PatternLeft.copyTo(mosaic(cv::Rect(0, h, w, h)));
    if (!result.vcsel2PatternRight.empty())
        result.vcsel2PatternRight.copyTo(mosaic(cv::Rect(w, h, w, h)));
    if (!result.isolatedPatternRight.empty())
        result.isolatedPatternRight.copyTo(mosaic(cv::Rect(2*w, h, w, h)));

    // Add labels
    cv::putText(mosaic, "VCSEL1 Left", cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(mosaic, "VCSEL1 Right", cv::Point(w + 10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(mosaic, "Combined Left", cv::Point(2*w + 10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(mosaic, "VCSEL2 Left", cv::Point(10, h + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(mosaic, "VCSEL2 Right", cv::Point(w + 10, h + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
    cv::putText(mosaic, "Combined Right", cv::Point(2*w + 10, h + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

    cv::imshow("Isolated Patterns", mosaic);
}

int main(int argc, char** argv) {
    // Initialize logging
    core::Logger::initialize(core::LogLevel::INFO);

    std::cout << "=== Temporal Stereo Processing Example ===" << std::endl;
    std::cout << "This example demonstrates VCSEL-based depth sensing with temporal matching" << std::endl;

    try {
        // Step 1: Initialize camera system
        std::cout << "\n1. Initializing camera system..." << std::endl;
        auto cameraSystem = std::make_shared<camera::CameraSystem>();
        if (!cameraSystem->initialize()) {
            std::cerr << "Failed to initialize camera system" << std::endl;
            return -1;
        }
        std::cout << "   Camera system initialized successfully" << std::endl;

        // Step 2: Load calibration
        std::cout << "\n2. Loading stereo calibration..." << std::endl;
        auto calibManager = std::make_shared<calibration::CalibrationManager>();

        // Try to load existing calibration
        std::string calibFile = "calibration/calib_boofcv_test3.yaml";
        if (!calibManager->loadCalibration(calibFile)) {
            std::cerr << "Failed to load calibration from: " << calibFile << std::endl;
            std::cerr << "Please run calibration first" << std::endl;
            return -1;
        }

        auto calibData = calibManager->getCalibrationData();
        std::cout << "   Calibration loaded: baseline = " << calibData.baseline
                  << "mm, RMS = " << calibData.rmsError << " pixels" << std::endl;

        // Step 3: Initialize dual VCSEL controller
        std::cout << "\n3. Initializing dual VCSEL controller..." << std::endl;
        auto dualVCSEL = hardware::AS1170DualVCSELController::getInstance();

        // Configure VCSELs for temporal matching
        hardware::AS1170DualVCSELController::VCSELConfig vcselConfig;
        vcselConfig.vcsel1_current_ma = 200;  // Safe operating current
        vcselConfig.vcsel2_current_ma = 200;
        vcselConfig.settle_time_ms = 50;      // Time between activations
        vcselConfig.capture_delay_ms = 10;    // Delay after activation
        vcselConfig.enable_thermal_monitoring = true;
        vcselConfig.max_operating_temp_c = 70.0f;

        if (!dualVCSEL->initialize(cameraSystem, vcselConfig)) {
            std::cerr << "Failed to initialize dual VCSEL controller" << std::endl;
            return -1;
        }
        std::cout << "   Dual VCSEL controller initialized successfully" << std::endl;

        // Step 4: Initialize temporal stereo processor
        std::cout << "\n4. Initializing temporal stereo processor..." << std::endl;
        auto temporalProcessor = std::make_unique<stereo::TemporalStereoProcessor>();

        // Use VCSEL-optimized configuration
        auto processorConfig = stereo::TemporalStereoProcessor::TemporalStereoConfig::getVCSELOptimized();

        // Optional: Enable debug output
        processorConfig.saveIntermediateImages = false;  // Set to true to save debug images
        processorConfig.debugOutputPath = "/tmp/temporal_stereo/";

        if (!temporalProcessor->initialize(dualVCSEL, calibManager, processorConfig)) {
            std::cerr << "Failed to initialize temporal stereo processor" << std::endl;
            return -1;
        }
        std::cout << "   Temporal stereo processor initialized with VCSEL-optimized parameters" << std::endl;

        // Step 5: Warm-up capture (allow hardware to stabilize)
        std::cout << "\n5. Performing warm-up capture..." << std::endl;
        stereo::TemporalStereoProcessor::TemporalStereoResult warmupResult;
        temporalProcessor->captureAndProcess(warmupResult);
        std::cout << "   Warm-up complete" << std::endl;

        // Step 6: Main processing loop
        std::cout << "\n6. Starting temporal stereo processing..." << std::endl;
        std::cout << "   Press 'q' to quit, 's' to save images, 'SPACE' for single capture" << std::endl;

        bool continuous = false;
        int frameCount = 0;

        while (true) {
            // Capture and process
            if (continuous || cv::waitKey(1) == ' ') {
                auto startTime = std::chrono::high_resolution_clock::now();

                stereo::TemporalStereoProcessor::TemporalStereoResult result;
                if (!temporalProcessor->captureAndProcess(result)) {
                    std::cerr << "Failed to capture and process" << std::endl;
                    continue;
                }

                auto endTime = std::chrono::high_resolution_clock::now();
                auto totalMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

                frameCount++;

                // Display results every 10th frame in continuous mode, or always in single mode
                if (!continuous || frameCount % 10 == 0) {
                    displayResults(result);
                    std::cout << "Total pipeline time: " << totalMs << " ms" << std::endl;
                }

                // Visualize
                visualizeDepth(result.depthMap);
                visualizePatterns(result);

                // Check VCSEL status
                auto vcselStatus = dualVCSEL->getStatus();
                if (vcselStatus.thermal_throttling) {
                    std::cout << "WARNING: Thermal throttling active (temp="
                              << vcselStatus.temperature_c << "°C)" << std::endl;
                }
            }

            // Handle keyboard input
            int key = cv::waitKey(continuous ? 1 : 30);
            if (key == 'q' || key == 27) {  // 'q' or ESC
                break;
            } else if (key == 's') {  // Save current images
                std::cout << "Saving images..." << std::endl;
                temporalProcessor->setDebugMode(true, "/tmp/temporal_stereo/");
                stereo::TemporalStereoProcessor::TemporalStereoResult saveResult;
                temporalProcessor->captureAndProcess(saveResult);
                temporalProcessor->setDebugMode(false);
                std::cout << "Images saved to /tmp/temporal_stereo/" << std::endl;
            } else if (key == 'c') {  // Toggle continuous mode
                continuous = !continuous;
                std::cout << "Continuous mode: " << (continuous ? "ON" : "OFF") << std::endl;
            }
        }

        // Step 7: Get final statistics
        std::cout << "\n7. Final Statistics:" << std::endl;
        auto stats = temporalProcessor->getStatistics();
        for (const auto& [key, value] : stats) {
            std::cout << "   " << key << ": " << value << std::endl;
        }

        // Cleanup
        std::cout << "\n8. Shutting down..." << std::endl;
        dualVCSEL->shutdown();
        cv::destroyAllWindows();

        std::cout << "Temporal stereo example completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}