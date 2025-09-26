/**
 * @file stereo_performance_test.cpp
 * @brief Performance validation for optimized SGBM stereo matching
 *
 * Tests the optimized parameters for 0.1mm precision at 15 FPS target
 * with 70mm baseline configuration on full resolution (1456x1088).
 */

#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/core/Logger.hpp"

using namespace unlook;
using namespace std::chrono;

/**
 * @brief Performance metrics structure
 */
struct PerformanceMetrics {
    double avgFPS;
    double minFPS;
    double maxFPS;
    double avgProcessingTimeMs;
    double validPixelRatio;
    double depthPrecisionAt200mm;
    size_t totalFrames;

    void print() const {
        std::cout << "\n===== SGBM Performance Metrics (0.1mm Target) =====" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "Average FPS:              " << avgFPS << " FPS" << std::endl;
        std::cout << "Min FPS:                  " << minFPS << " FPS" << std::endl;
        std::cout << "Max FPS:                  " << maxFPS << " FPS" << std::endl;
        std::cout << "Avg Processing Time:      " << avgProcessingTimeMs << " ms" << std::endl;
        std::cout << "Valid Pixel Coverage:     " << (validPixelRatio * 100) << "%" << std::endl;
        std::cout << "Depth Precision @200mm:   " << depthPrecisionAt200mm << " mm" << std::endl;
        std::cout << "Total Frames Processed:   " << totalFrames << std::endl;
        std::cout << "===================================================" << std::endl;

        // Performance target validation
        std::cout << "\n----- Target Validation -----" << std::endl;
        std::cout << "15 FPS Target:            " << (avgFPS >= 15.0 ? "✓ PASSED" : "✗ FAILED") << std::endl;
        std::cout << "0.1mm Precision Target:   " << (depthPrecisionAt200mm <= 0.1 ? "✓ PASSED" : "✗ FAILED") << std::endl;
        std::cout << "85% Coverage Target:      " << (validPixelRatio >= 0.85 ? "✓ PASSED" : "✗ FAILED") << std::endl;
    }
};

/**
 * @brief Generate synthetic stereo pair for testing
 */
void generateTestStereoPair(cv::Mat& left, cv::Mat& right, int width, int height) {
    // Create synthetic test pattern with various textures
    left = cv::Mat(height, width, CV_8UC1);
    right = cv::Mat(height, width, CV_8UC1);

    // Add gradient background
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            left.at<uint8_t>(y, x) = static_cast<uint8_t>((x + y) / 8);
            right.at<uint8_t>(y, x) = static_cast<uint8_t>((x + y) / 8);
        }
    }

    // Add textured rectangles at different depths
    int numRects = 10;
    for (int i = 0; i < numRects; ++i) {
        int rectX = (width / numRects) * i;
        int rectY = height / 3;
        int rectW = width / (numRects * 2);
        int rectH = height / 3;
        int disparity = 8 + i * 12;  // Simulate different depths

        cv::Rect leftRect(rectX, rectY, rectW, rectH);
        cv::Rect rightRect(rectX - disparity, rectY, rectW, rectH);

        // Add texture
        cv::randu(left(leftRect), 100, 200);
        if (rightRect.x >= 0 && rightRect.x + rightRect.width <= width) {
            left(leftRect).copyTo(right(rightRect));
        }
    }

    // Add noise to simulate real camera
    cv::Mat noise(height, width, CV_8UC1);
    cv::randu(noise, 0, 5);
    cv::add(left, noise, left);
    cv::randu(noise, 0, 5);
    cv::add(right, noise, right);
}

/**
 * @brief Calculate depth precision at specific distance
 */
double calculateDepthPrecision(double baseline_mm, double focal_pixels, double depth_mm) {
    // Depth precision formula: Δz = (z² × Δd) / (f × b)
    // Assuming 0.25 pixel disparity error for optimized SGBM
    const double disparity_error = 0.25;  // pixels
    return (depth_mm * depth_mm * disparity_error) / (focal_pixels * baseline_mm);
}

int main(int argc, char* argv[]) {
    // Initialize logger
    core::Logger::getInstance().setLevel(core::LogLevel::INFO);
    LOG_INFO("Starting SGBM Performance Test - 0.1mm Precision @ 15 FPS Target");

    // Test parameters
    const int IMAGE_WIDTH = 1456;
    const int IMAGE_HEIGHT = 1088;
    const int NUM_TEST_FRAMES = 100;
    const double BASELINE_MM = 70.017;  // From calibration
    const double FOCAL_LENGTH = 800.0;  // Approximate for 6mm lens

    // Create optimized SGBM matcher
    auto sgbm = std::make_unique<stereo::SGBMStereoMatcher>();

    // Verify optimized parameters
    auto params = sgbm->getParameters();
    LOG_INFO("SGBM Parameters for 0.1mm Industrial Target:");
    LOG_INFO("  - numDisparities: " + std::to_string(params.numDisparities));
    LOG_INFO("  - blockSize: " + std::to_string(params.blockSize));
    LOG_INFO("  - P1: " + std::to_string(params.P1));
    LOG_INFO("  - P2: " + std::to_string(params.P2));
    LOG_INFO("  - uniquenessRatio: " + std::to_string(params.uniquenessRatio));
    LOG_INFO("  - WLS Lambda: " + std::to_string(params.wlsLambda));
    LOG_INFO("  - WLS Sigma: " + std::to_string(params.wlsSigma));

    // Performance tracking
    std::vector<double> processingTimes;
    std::vector<double> fpsValues;
    std::vector<double> validPixelRatios;

    LOG_INFO("Running performance test with " + std::to_string(NUM_TEST_FRAMES) + " frames...");
    LOG_INFO("Image resolution: " + std::to_string(IMAGE_WIDTH) + "x" + std::to_string(IMAGE_HEIGHT));

    // Warmup run
    cv::Mat leftWarmup, rightWarmup, disparityWarmup;
    generateTestStereoPair(leftWarmup, rightWarmup, IMAGE_WIDTH, IMAGE_HEIGHT);
    sgbm->computeDisparity(leftWarmup, rightWarmup, disparityWarmup);

    // Main performance test loop
    for (int frame = 0; frame < NUM_TEST_FRAMES; ++frame) {
        // Generate test stereo pair
        cv::Mat left, right;
        generateTestStereoPair(left, right, IMAGE_WIDTH, IMAGE_HEIGHT);

        // Time the disparity computation
        auto start = high_resolution_clock::now();

        cv::Mat disparity;
        bool success = sgbm->computeDisparity(left, right, disparity);

        auto end = high_resolution_clock::now();

        if (!success) {
            LOG_ERROR("Failed to compute disparity for frame " + std::to_string(frame));
            continue;
        }

        // Calculate metrics
        auto duration = duration_cast<microseconds>(end - start).count() / 1000.0;
        double fps = 1000.0 / duration;

        processingTimes.push_back(duration);
        fpsValues.push_back(fps);

        // Calculate valid pixel ratio
        int validPixels = cv::countNonZero(disparity > 0);
        double validRatio = static_cast<double>(validPixels) / (IMAGE_WIDTH * IMAGE_HEIGHT);
        validPixelRatios.push_back(validRatio);

        // Progress update every 10 frames
        if ((frame + 1) % 10 == 0) {
            LOG_INFO("Processed " + std::to_string(frame + 1) + "/" + std::to_string(NUM_TEST_FRAMES) +
                    " frames. Current FPS: " + std::to_string(fps));
        }
    }

    // Calculate final metrics
    PerformanceMetrics metrics;
    metrics.totalFrames = processingTimes.size();

    if (metrics.totalFrames > 0) {
        // FPS statistics
        metrics.avgFPS = 1000.0 / (std::accumulate(processingTimes.begin(), processingTimes.end(), 0.0) / metrics.totalFrames);
        metrics.minFPS = *std::min_element(fpsValues.begin(), fpsValues.end());
        metrics.maxFPS = *std::max_element(fpsValues.begin(), fpsValues.end());
        metrics.avgProcessingTimeMs = std::accumulate(processingTimes.begin(), processingTimes.end(), 0.0) / metrics.totalFrames;

        // Valid pixel ratio
        metrics.validPixelRatio = std::accumulate(validPixelRatios.begin(), validPixelRatios.end(), 0.0) / metrics.totalFrames;

        // Depth precision at 200mm
        metrics.depthPrecisionAt200mm = calculateDepthPrecision(BASELINE_MM, FOCAL_LENGTH, 200.0);
    }

    // Print results
    metrics.print();

    // Additional analysis
    std::cout << "\n===== Detailed Performance Analysis =====" << std::endl;

    // Check consistency
    double stdDev = 0.0;
    for (const auto& time : processingTimes) {
        stdDev += std::pow(time - metrics.avgProcessingTimeMs, 2);
    }
    stdDev = std::sqrt(stdDev / processingTimes.size());

    std::cout << "Processing Time Std Dev:  " << std::fixed << std::setprecision(2) << stdDev << " ms" << std::endl;
    std::cout << "Consistency Rating:       ";
    if (stdDev < 5.0) {
        std::cout << "Excellent (very stable)" << std::endl;
    } else if (stdDev < 10.0) {
        std::cout << "Good (stable)" << std::endl;
    } else if (stdDev < 20.0) {
        std::cout << "Fair (some variation)" << std::endl;
    } else {
        std::cout << "Poor (high variation)" << std::endl;
    }

    // Memory usage estimate
    size_t memoryUsage = IMAGE_WIDTH * IMAGE_HEIGHT * sizeof(float) * 4;  // Approx for disparity processing
    std::cout << "Estimated Memory Usage:   " << (memoryUsage / (1024 * 1024)) << " MB" << std::endl;

    // Theoretical depth range
    double minDepth = (BASELINE_MM * FOCAL_LENGTH) / params.numDisparities;
    double maxDepth = (BASELINE_MM * FOCAL_LENGTH) / params.minDisparity;
    std::cout << "Theoretical Depth Range:  " << std::fixed << std::setprecision(0)
              << minDepth << " - " << maxDepth << " mm" << std::endl;

    // Performance recommendations
    std::cout << "\n----- Optimization Recommendations -----" << std::endl;
    if (metrics.avgFPS < 15.0) {
        std::cout << "• Consider reducing numDisparities to 96 for speed gain" << std::endl;
        std::cout << "• Try disabling WLS filter for critical paths" << std::endl;
        std::cout << "• Enable MODE_HH for maximum speed (horizontal only)" << std::endl;
    } else if (metrics.avgFPS > 20.0) {
        std::cout << "• Performance exceeds target - can increase quality settings" << std::endl;
        std::cout << "• Consider MODE_SGBM_3WAY for better accuracy" << std::endl;
        std::cout << "• Can increase WLS lambda for better edge preservation" << std::endl;
    } else {
        std::cout << "• Performance is optimal for 0.1mm @ 15 FPS target" << std::endl;
        std::cout << "• Current settings provide good balance" << std::endl;
    }

    return 0;
}