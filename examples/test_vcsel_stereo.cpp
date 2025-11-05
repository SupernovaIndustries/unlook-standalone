/**
 * @file test_vcsel_stereo.cpp
 * @brief Test program for VCSELStereoMatcher with AD-Census algorithm
 *
 * Tests the AD-Census stereo matcher with HD 1280x720 processing
 * and reports performance metrics.
 */

#include <unlook/stereo/VCSELStereoMatcher.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <iomanip>

using namespace unlook;
using namespace std::chrono;

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <left_image> <right_image> [calibration_file]\n";
    std::cout << "  left_image: Path to left rectified image\n";
    std::cout << "  right_image: Path to right rectified image\n";
    std::cout << "  calibration_file: Optional path to calibration YAML (default: calibration/calib_boofcv_test3.yaml)\n";
}

void visualizeDisparity(const cv::Mat& disparity, const std::string& windowName) {
    // Normalize disparity for visualization
    cv::Mat disparityVis;
    double minVal, maxVal;
    cv::minMaxLoc(disparity, &minVal, &maxVal);

    if (maxVal > minVal) {
        disparity.convertTo(disparityVis, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        cv::applyColorMap(disparityVis, disparityVis, cv::COLORMAP_JET);
    } else {
        disparityVis = cv::Mat::zeros(disparity.size(), CV_8UC3);
    }

    cv::imshow(windowName, disparityVis);
}

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    // Initialize logger
    core::Logger& logger = core::Logger::getInstance();
    logger.setLevel(core::LogLevel::INFO);
    logger.log(core::LogLevel::INFO, "=== VCSEL AD-Census Stereo Matcher Test ===");

    // Load images
    std::string leftPath = argv[1];
    std::string rightPath = argv[2];
    std::string calibPath = (argc > 3) ? argv[3] : "calibration/calib_boofcv_test3.yaml";

    cv::Mat leftImage = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat rightImage = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

    if (leftImage.empty() || rightImage.empty()) {
        logger.log(core::LogLevel::ERROR, "Failed to load images");
        return 1;
    }

    logger.log(core::LogLevel::INFO,
        "Loaded images: " + std::to_string(leftImage.cols) + "x" + std::to_string(leftImage.rows));

    // Load calibration (optional - for rectification if needed)
    calibration::CalibrationManager calibMgr;
    bool hasCalibration = false;

    if (calibMgr.loadCalibration(calibPath)) {
        logger.log(core::LogLevel::INFO, "Calibration loaded from: " + calibPath);
        hasCalibration = true;

        // Rectify images if calibration is available and images aren't already rectified
        cv::Mat rectifiedLeft, rectifiedRight;
        if (calibMgr.rectifyImages(leftImage, rightImage, rectifiedLeft, rectifiedRight)) {
            leftImage = rectifiedLeft;
            rightImage = rectifiedRight;
            logger.log(core::LogLevel::INFO, "Images rectified using calibration");
        }
    } else {
        logger.log(core::LogLevel::WARNING, "No calibration loaded, assuming images are pre-rectified");
    }

    // Create VCSEL stereo matcher
    auto matcher = std::make_unique<stereo::VCSELStereoMatcher>();

    // Configure parameters for VCSEL
    stereo::StereoMatchingParams params;
    params.minDisparity = 48;
    params.numDisparities = 256;  // Must be divisible by 16
    params.uniquenessRatio = 25;
    params.P1 = 4;   // Small penalty for VCSEL dots
    params.P2 = 24;  // Moderate smoothing

    matcher->setParameters(params);

    logger.log(core::LogLevel::INFO, "Algorithm: " + matcher->getAlgorithmName());
    logger.log(core::LogLevel::INFO, "Processing resolution: HD 1280x720");
    logger.log(core::LogLevel::INFO, "Disparity range: " +
        std::to_string(params.minDisparity) + " to " +
        std::to_string(params.minDisparity + params.numDisparities));

    // Warm-up run
    cv::Mat disparity;
    matcher->computeDisparity(leftImage, rightImage, disparity);

    // Performance test - run multiple times
    const int numRuns = 10;
    std::vector<double> timings;

    logger.log(core::LogLevel::INFO, "\nRunning " + std::to_string(numRuns) + " iterations for performance measurement...");

    for (int i = 0; i < numRuns; i++) {
        auto start = high_resolution_clock::now();

        bool success = matcher->computeDisparity(leftImage, rightImage, disparity);

        auto end = high_resolution_clock::now();
        double timeMs = duration_cast<microseconds>(end - start).count() / 1000.0;
        timings.push_back(timeMs);

        if (!success) {
            logger.log(core::LogLevel::ERROR, "Disparity computation failed on iteration " + std::to_string(i));
            continue;
        }

        // Get detailed stats for first run
        if (i == 0) {
            auto stats = matcher->getLastProcessingStats();
            logger.log(core::LogLevel::INFO, "\nDetailed timing breakdown:");
            logger.log(core::LogLevel::INFO, "  Downsample: " +
                std::to_string(stats.downsampleTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  Census Transform: " +
                std::to_string(stats.censusTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  Hamming Distance: " +
                std::to_string(stats.hammingTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  AD Cost: " +
                std::to_string(stats.adCostTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  Cost Fusion: " +
                std::to_string(stats.fusionTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  SGM Aggregation: " +
                std::to_string(stats.sgmTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  Post-processing: " +
                std::to_string(stats.postProcessingTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  TOTAL: " +
                std::to_string(stats.totalTimeMs) + " ms");
            logger.log(core::LogLevel::INFO, "  Valid pixels: " +
                std::to_string(100.0 * stats.validPixels / stats.totalPixels) + "%");
        }

        std::cout << "Run " << (i + 1) << "/" << numRuns << ": "
                  << std::fixed << std::setprecision(2) << timeMs << " ms ("
                  << (1000.0 / timeMs) << " FPS)" << std::endl;
    }

    // Calculate statistics
    double avgTime = 0;
    double minTime = timings[0];
    double maxTime = timings[0];

    for (double t : timings) {
        avgTime += t;
        minTime = std::min(minTime, t);
        maxTime = std::max(maxTime, t);
    }
    avgTime /= numRuns;

    logger.log(core::LogLevel::INFO, "\nPerformance Summary @ HD 1280x720:");
    logger.log(core::LogLevel::INFO, "  Average: " + std::to_string(avgTime) +
        " ms (" + std::to_string(1000.0 / avgTime) + " FPS)");
    logger.log(core::LogLevel::INFO, "  Min: " + std::to_string(minTime) +
        " ms (" + std::to_string(1000.0 / minTime) + " FPS)");
    logger.log(core::LogLevel::INFO, "  Max: " + std::to_string(maxTime) +
        " ms (" + std::to_string(1000.0 / maxTime) + " FPS)");

    // Target check
    bool meetsTarget = avgTime <= 100.0;  // Target: 100ms for 10 FPS
    logger.log(meetsTarget ? core::LogLevel::INFO : core::LogLevel::WARNING,
        meetsTarget ? "✓ Meets target of 10 FPS for handheld scanning" :
                     "✗ Below target of 10 FPS - optimization needed");

    // Compute quality metrics
    stereo::StereoQualityMetrics metrics;
    if (matcher->computeQualityMetrics(disparity, metrics)) {
        logger.log(core::LogLevel::INFO, "\nQuality Metrics:");
        logger.log(core::LogLevel::INFO, "  Average disparity: " +
            std::to_string(metrics.avgDisparity));
        logger.log(core::LogLevel::INFO, "  Std deviation: " +
            std::to_string(metrics.stdDisparity));
        logger.log(core::LogLevel::INFO, "  Valid pixel ratio: " +
            std::to_string(metrics.validPixelRatio * 100) + "%");
        logger.log(core::LogLevel::INFO, "  Texture score: " +
            std::to_string(metrics.textureScore));
    }

    // Visualize results
    cv::namedWindow("Left Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Right Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Disparity Map", cv::WINDOW_NORMAL);

    cv::imshow("Left Image", leftImage);
    cv::imshow("Right Image", rightImage);
    visualizeDisparity(disparity, "Disparity Map");

    // Save disparity map
    std::string outputPath = "vcsel_disparity_output.png";
    cv::Mat disparitySave;
    disparity.convertTo(disparitySave, CV_16U, 256.0);  // Scale to 16-bit for saving
    cv::imwrite(outputPath, disparitySave);
    logger.log(core::LogLevel::INFO, "\nDisparity map saved to: " + outputPath);

    // If we have calibration, compute depth map
    if (hasCalibration) {
        cv::Mat depth;
        cv::Mat Q = calibMgr.getCalibrationData().Q;

        if (!Q.empty() && stereo::StereoMatcher::disparityToDepth(disparity, Q, depth, true)) {
            // Visualize depth
            cv::Mat depthVis;
            depth.convertTo(depthVis, CV_8U, 255.0 / 800.0);  // Scale for 800mm max depth
            cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);

            cv::namedWindow("Depth Map", cv::WINDOW_NORMAL);
            cv::imshow("Depth Map", depthVis);

            logger.log(core::LogLevel::INFO, "Depth map computed from calibration");
        }
    }

    logger.log(core::LogLevel::INFO, "\nPress any key to exit...");
    cv::waitKey(0);

    return 0;
}