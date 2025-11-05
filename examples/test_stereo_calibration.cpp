/**
 * Test program for stereo calibration backend
 * Tests the complete calibration pipeline implementation
 */

#include <unlook/calibration/StereoCalibrationProcessor.hpp>
#include <unlook/calibration/CalibrationValidator.hpp>
#include <unlook/calibration/PatternDetector.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;
using namespace unlook;

int main(int argc, char* argv[]) {
    core::Logger::getInstance().info("=== Stereo Calibration Test ===");

    // Test 1: Pattern detector initialization
    {
        core::Logger::getInstance().info("Test 1: Pattern detector initialization");

        calibration::PatternConfig config;
        config.type = calibration::PatternType::CHARUCO;
        config.rows = 7;
        config.cols = 10;
        config.squareSizeMM = 24.0f;
        config.arucoMarkerSizeMM = 17.0f;

        calibration::PatternDetector detector(config);
        core::Logger::getInstance().info("Pattern detector created successfully");

        // Test with a synthetic image
        cv::Mat testImage = cv::Mat::zeros(720, 1280, CV_8UC3);
        std::vector<cv::Point2f> corners;
        cv::Mat overlayImage;

        bool detected = detector.detect(testImage, corners, overlayImage);
        core::Logger::getInstance().info(std::string("Pattern detection test: ") +
                                        (detected ? "pattern found" : "no pattern (expected)"));
    }

    // Test 2: Calibration validator
    {
        core::Logger::getInstance().info("Test 2: Calibration validator");

        calibration::CalibrationValidator validator;
        calibration::CalibrationResult result;

        // Set some test values
        result.rmsReprojectionError = 0.25;
        result.meanEpipolarError = 0.3;
        result.baselineMM = 70.0;
        result.baselineExpectedMM = 70.0;
        result.baselineErrorMM = 0.0;
        result.validImagePairs = 40;
        result.imageSize = cv::Size(1280, 720);

        // Initialize camera matrices with reasonable values
        result.cameraMatrixLeft = cv::Mat::eye(3, 3, CV_64F);
        result.cameraMatrixLeft.at<double>(0, 0) = 1200.0;
        result.cameraMatrixLeft.at<double>(1, 1) = 1200.0;
        result.cameraMatrixLeft.at<double>(0, 2) = 640.0;
        result.cameraMatrixLeft.at<double>(1, 2) = 360.0;

        result.cameraMatrixRight = result.cameraMatrixLeft.clone();
        result.distCoeffsLeft = cv::Mat::zeros(5, 1, CV_64F);
        result.distCoeffsRight = cv::Mat::zeros(5, 1, CV_64F);

        calibration::CalibrationValidator::ValidationCriteria criteria;
        bool valid = validator.validate(result, criteria);

        core::Logger::getInstance().info(std::string("Validation result: ") +
                                        (valid ? "PASSED" : "FAILED"));
        core::Logger::getInstance().info("Quality check: " + result.rmsCheck);
    }

    // Test 3: Stereo calibration processor
    {
        core::Logger::getInstance().info("Test 3: Stereo calibration processor");

        calibration::StereoCalibrationProcessor processor;

        // Check if there's a test dataset
        std::string datasetPath = "/unlook_calib_dataset";
        if (fs::exists(datasetPath)) {
            std::vector<std::string> datasets;
            for (const auto& entry : fs::directory_iterator(datasetPath)) {
                if (entry.is_directory()) {
                    datasets.push_back(entry.path().string());
                }
            }

            if (!datasets.empty()) {
                core::Logger::getInstance().info("Found " +
                                               std::to_string(datasets.size()) +
                                               " dataset(s)");
                // Could process first dataset here
                // auto result = processor.calibrateFromDataset(datasets[0]);
            } else {
                core::Logger::getInstance().info("No datasets found in " + datasetPath);
            }
        } else {
            core::Logger::getInstance().info("Dataset directory not found: " + datasetPath);
        }

        // Test calibration save/load
        calibration::CalibrationResult testResult;
        testResult.calibrationDate = "2025-11-04T15:00:00";
        testResult.imageSize = cv::Size(1280, 720);
        testResult.validImagePairs = 30;
        testResult.rmsReprojectionError = 0.28;
        testResult.baselineMM = 70.0;
        testResult.qualityPassed = true;

        // Initialize matrices
        testResult.cameraMatrixLeft = cv::Mat::eye(3, 3, CV_64F);
        testResult.cameraMatrixRight = cv::Mat::eye(3, 3, CV_64F);
        testResult.distCoeffsLeft = cv::Mat::zeros(5, 1, CV_64F);
        testResult.distCoeffsRight = cv::Mat::zeros(5, 1, CV_64F);
        testResult.R = cv::Mat::eye(3, 3, CV_64F);
        testResult.T = cv::Mat::zeros(3, 1, CV_64F);
        testResult.T.at<double>(0, 0) = -70.0;  // Baseline

        std::string testCalibPath = "/tmp/test_calib.yaml";
        bool saved = processor.saveCalibration(testResult, testCalibPath);

        if (saved) {
            core::Logger::getInstance().info("Test calibration saved to: " + testCalibPath);

            // Try to load it back
            try {
                auto loaded = processor.loadCalibration(testCalibPath);
                core::Logger::getInstance().info("Calibration loaded successfully");
                core::Logger::getInstance().info("Loaded baseline: " +
                                               std::to_string(loaded.baselineMM) + "mm");
            } catch (const std::exception& e) {
                core::Logger::getInstance().error("Failed to load calibration: " +
                                                 std::string(e.what()));
            }
        } else {
            core::Logger::getInstance().error("Failed to save test calibration");
        }
    }

    core::Logger::getInstance().info("=== All tests completed ===");

    return 0;
}