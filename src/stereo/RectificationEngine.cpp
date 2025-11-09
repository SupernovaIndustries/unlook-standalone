/**
 * @file RectificationEngine.cpp
 * @brief Implementation of stereo image rectification engine
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/RectificationEngine.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <filesystem>

namespace unlook {
namespace stereo {

// ========== CONSTRUCTOR / DESTRUCTOR ==========

RectificationEngine::RectificationEngine(
    calibration::CalibrationManager* calib,
    core::Logger* logger)
    : calibration_(calib)
    , logger_(logger)
    , mapsComputed_(false)
{
    if (!calibration_) {
        throw std::invalid_argument("CalibrationManager cannot be null");
    }
}

RectificationEngine::~RectificationEngine() = default;

// ========== CONFIGURATION ==========

void RectificationEngine::setConfig(const Config& config) {
    config_ = config;
}

cv::Size RectificationEngine::getExpectedInputSize() const {
    const auto& data = calibration_->getCalibrationData();
    return data.imageSize;
}

// ========== MAP COMPUTATION ==========

bool RectificationEngine::precomputeMaps() {
    return computeMaps();
}

bool RectificationEngine::computeMaps() {
    if (mapsComputed_) {
        return true; // Already computed
    }

    const auto& data = calibration_->getCalibrationData();

    if (logger_) {
        logger_->info("Computing rectification maps for size " +
                     std::to_string(data.imageSize.width) + "x" +
                     std::to_string(data.imageSize.height));
    }

    try {
        // Compute rectification maps for left camera
        cv::initUndistortRectifyMap(
            data.cameraMatrixLeft,
            data.distCoeffsLeft,
            data.R1,
            data.P1,
            data.imageSize,
            CV_32FC1,
            map1Left_,
            map2Left_);

        // Compute rectification maps for right camera
        cv::initUndistortRectifyMap(
            data.cameraMatrixRight,
            data.distCoeffsRight,
            data.R2,
            data.P2,
            data.imageSize,
            CV_32FC1,
            map1Right_,
            map2Right_);

        mapsComputed_ = true;

        if (logger_) {
            logger_->info("✓ Rectification maps computed successfully");
            logger_->info("  Map size: " +
                         std::to_string(map1Left_.cols) + "x" +
                         std::to_string(map1Left_.rows));
        }

        return true;

    } catch (const cv::Exception& e) {
        if (logger_) {
            logger_->error("Failed to compute rectification maps: " +
                          std::string(e.what()));
        }
        return false;
    }
}

// ========== INPUT VALIDATION ==========

bool RectificationEngine::validateInputSizes(
    const cv::Mat& leftInput,
    const cv::Mat& rightInput,
    std::string& errorMsg) const
{
    const auto& data = calibration_->getCalibrationData();
    const cv::Size expectedSize = data.imageSize;

    // Check left image size
    if (leftInput.size() != expectedSize) {
        std::ostringstream oss;
        oss << "Left image size " << leftInput.cols << "x" << leftInput.rows
            << " does NOT match calibration size "
            << expectedSize.width << "x" << expectedSize.height;
        errorMsg = oss.str();
        return false;
    }

    // Check right image size
    if (rightInput.size() != expectedSize) {
        std::ostringstream oss;
        oss << "Right image size " << rightInput.cols << "x" << rightInput.rows
            << " does NOT match calibration size "
            << expectedSize.width << "x" << expectedSize.height;
        errorMsg = oss.str();
        return false;
    }

    // Check images have same size
    if (leftInput.size() != rightInput.size()) {
        std::ostringstream oss;
        oss << "Left and right image sizes do not match: "
            << leftInput.cols << "x" << leftInput.rows << " vs "
            << rightInput.cols << "x" << rightInput.rows;
        errorMsg = oss.str();
        return false;
    }

    return true;
}

// ========== MAIN RECTIFICATION ==========

RectificationEngine::Result RectificationEngine::rectify(
    const cv::Mat& leftInput,
    const cv::Mat& rightInput)
{
    auto startTime = std::chrono::high_resolution_clock::now();

    Result result;
    result.success = false;

    // Validate input sizes
    if (config_.validateSize) {
        std::string validationError;
        if (!validateInputSizes(leftInput, rightInput, validationError)) {
            result.errorMessage = validationError;
            if (logger_) {
                logger_->error("╔════════════════════════════════════════╗");
                logger_->error("║  RECTIFICATION SIZE VALIDATION FAILED  ║");
                logger_->error("╚════════════════════════════════════════╝");
                logger_->error(validationError);
            }
            return result;
        }
    }

    // Compute maps if not already done
    if (!mapsComputed_) {
        if (!computeMaps()) {
            result.errorMessage = "Failed to compute rectification maps";
            return result;
        }
    }

    try {
        // Apply rectification using precomputed maps
        cv::remap(
            leftInput,
            result.leftRectified,
            map1Left_,
            map2Left_,
            config_.interpolation,
            cv::BORDER_REPLICATE);  // Changed from BORDER_CONSTANT to preserve brightness

        cv::remap(
            rightInput,
            result.rightRectified,
            map1Right_,
            map2Right_,
            config_.interpolation,
            cv::BORDER_REPLICATE);  // Changed from BORDER_CONSTANT to preserve brightness

        result.outputSize = result.leftRectified.size();
        result.success = true;

        // Calculate quality metrics
        result.validPixelPercent = calculateValidPixelPercent(result.leftRectified);

        // Debug visualization
        if (config_.enableDebug && !config_.debugDir.empty()) {
            saveDebugVisualization(result.leftRectified, result.rightRectified);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        result.rectificationTime =
            std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        if (logger_) {
            logger_->info("✓ Rectification completed in " +
                         std::to_string(result.rectificationTime.count()) + " ms");
            logger_->info("  Valid pixels: " +
                         std::to_string(static_cast<int>(result.validPixelPercent)) + "%");
        }

    } catch (const cv::Exception& e) {
        result.errorMessage = "OpenCV rectification error: " + std::string(e.what());
        if (logger_) {
            logger_->error(result.errorMessage);
        }
    }

    return result;
}

// ========== QUALITY METRICS ==========

double RectificationEngine::calculateValidPixelPercent(const cv::Mat& rectified) const {
    if (rectified.empty()) {
        return 0.0;
    }

    // Convert to grayscale if needed
    cv::Mat gray;
    if (rectified.channels() == 3) {
        cv::cvtColor(rectified, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = rectified;
    }

    // Count non-black pixels
    int validPixels = cv::countNonZero(gray);
    int totalPixels = gray.rows * gray.cols;

    return (100.0 * validPixels) / totalPixels;
}

// ========== DEBUG VISUALIZATION ==========

void RectificationEngine::saveDebugVisualization(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect) const
{
    std::filesystem::create_directories(config_.debugDir);

    // Save individual rectified images
    std::string leftPath = config_.debugDir + "/01_rectified_left.png";
    std::string rightPath = config_.debugDir + "/01_rectified_right.png";
    cv::imwrite(leftPath, leftRect);
    cv::imwrite(rightPath, rightRect);

    // Draw and save epipolar lines visualization
    cv::Mat epipolarViz = drawEpipolarLines(leftRect, rightRect);
    std::string epipolarPath = config_.debugDir + "/01_epipolar_check.png";
    cv::imwrite(epipolarPath, epipolarViz);

    if (logger_) {
        logger_->info("Debug visualizations saved to: " + config_.debugDir);
    }
}

cv::Mat RectificationEngine::drawEpipolarLines(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect) const
{
    // Convert to color if grayscale
    cv::Mat leftColor, rightColor;
    if (leftRect.channels() == 1) {
        cv::cvtColor(leftRect, leftColor, cv::COLOR_GRAY2BGR);
    } else {
        leftColor = leftRect.clone();
    }

    if (rightRect.channels() == 1) {
        cv::cvtColor(rightRect, rightColor, cv::COLOR_GRAY2BGR);
    } else {
        rightColor = rightRect.clone();
    }

    // Draw horizontal lines every 40 pixels
    const int lineSpacing = 40;
    const cv::Scalar lineColor(0, 255, 0); // Green
    const int thickness = 1;

    for (int y = 0; y < leftColor.rows; y += lineSpacing) {
        cv::line(leftColor,
                cv::Point(0, y),
                cv::Point(leftColor.cols - 1, y),
                lineColor,
                thickness);

        cv::line(rightColor,
                cv::Point(0, y),
                cv::Point(rightColor.cols - 1, y),
                lineColor,
                thickness);
    }

    // Combine left and right images horizontally
    cv::Mat combined;
    cv::hconcat(leftColor, rightColor, combined);

    // Add text label
    cv::putText(combined,
                "Epipolar Lines Check - Lines should be horizontal and aligned",
                cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(255, 255, 0),
                2);

    return combined;
}

} // namespace stereo
} // namespace unlook
