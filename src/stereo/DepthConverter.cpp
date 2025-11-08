/**
 * @file DepthConverter.cpp
 * @brief Implementation of disparity to depth conversion
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/DepthConverter.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace unlook {
namespace stereo {

// ========== CONSTRUCTOR / DESTRUCTOR ==========

DepthConverter::DepthConverter(
    calibration::CalibrationManager* calibration,
    core::Logger* logger)
    : calibration_(calibration)
    , logger_(logger ? logger : &core::Logger::getInstance())
{
    if (!calibration_) {
        throw std::invalid_argument("CalibrationManager cannot be null");
    }

    // Validate Q matrix exists
    cv::Mat Q = getQMatrix();
    if (Q.empty() || Q.rows != 4 || Q.cols != 4) {
        throw std::runtime_error("Invalid or missing Q matrix in calibration");
    }

    if (logger_) {
        logger_->info("DepthConverter initialized with Q matrix");
    }
}

DepthConverter::~DepthConverter() = default;

// ========== CONFIGURATION ==========

void DepthConverter::setConfig(const Config& config) {
    config_ = config;

    if (logger_) {
        logger_->info("DepthConverter configured:");
        logger_->info("  Depth range: " + std::to_string(config_.minDepthMM) +
                     " to " + std::to_string(config_.maxDepthMM) + " mm");
        logger_->info("  Disparity scale: " + std::to_string(config_.disparityScale));
        logger_->info("  Clamp to range: " + std::string(config_.clampToRange ? "yes" : "no"));
    }
}

// ========== Q MATRIX ACCESS ==========

cv::Mat DepthConverter::getQMatrix() const {
    if (!calibration_) {
        return cv::Mat();
    }

    const auto& data = calibration_->getCalibrationData();
    return data.Q;
}

bool DepthConverter::validateBaseline(float expectedBaseline, float tolerance) {
    cv::Mat Q = getQMatrix();
    if (Q.empty()) {
        if (logger_) logger_->error("Q matrix is empty, cannot validate baseline");
        return false;
    }

    // Extract baseline from Q matrix
    // Q[2,3] = -f * Tx where Tx is the baseline in the same units as f
    // Q[3,2] = -1/Tx
    double baseline_from_q = -1.0 / Q.at<double>(3, 2);

    float difference = std::abs(baseline_from_q - expectedBaseline);

    if (logger_) {
        logger_->info("Baseline validation:");
        logger_->info("  Expected: " + std::to_string(expectedBaseline) + " mm");
        logger_->info("  From Q matrix: " + std::to_string(baseline_from_q) + " mm");
        logger_->info("  Difference: " + std::to_string(difference) + " mm");
    }

    bool valid = difference <= tolerance;
    if (!valid && logger_) {
        logger_->warning("Baseline mismatch exceeds tolerance!");
    }

    return valid;
}

// ========== MAIN CONVERSION FUNCTION ==========

DepthConverter::Result DepthConverter::convert(const cv::Mat& disparity) {
    cv::Mat Q = getQMatrix();
    return convertWithQ(disparity, Q);
}

DepthConverter::Result DepthConverter::convertWithQ(const cv::Mat& disparity, const cv::Mat& Q) {
    Result result;

    // Validate inputs
    if (disparity.empty()) {
        result.success = false;
        result.errorMessage = "Empty disparity map";
        return result;
    }

    if (Q.empty() || Q.rows != 4 || Q.cols != 4) {
        result.success = false;
        result.errorMessage = "Invalid Q matrix";
        return result;
    }

    try {
        const int width = disparity.cols;
        const int height = disparity.rows;

        // Initialize output maps
        result.depthMap = cv::Mat(height, width, CV_32F);
        result.validMask = cv::Mat(height, width, CV_8U);

        // Extract Q matrix elements for direct calculation
        // Q matrix structure:
        // [1  0  0      -cx      ]
        // [0  1  0      -cy      ]
        // [0  0  0       f       ]
        // [0  0  -1/Tx  (cx-cx')/Tx]

        double q03 = Q.at<double>(0, 3); // -cx
        double q13 = Q.at<double>(1, 3); // -cy
        double q23 = Q.at<double>(2, 3); // f
        double q32 = Q.at<double>(3, 2); // -1/Tx (Tx = baseline)
        double q33 = Q.at<double>(3, 3); // (cx - cx')/Tx

        // Calculate baseline for validation
        double baseline = -1.0 / q32;

        if (logger_) {
            logger_->info("Converting disparity to depth:");
            logger_->info("  Focal length: " + std::to_string(q23));
            logger_->info("  Baseline: " + std::to_string(baseline) + " mm");
        }

        // Convert each pixel
        int validCount = 0;

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float d = 0;

                // Get disparity value based on type
                if (disparity.type() == CV_16S) {
                    // Fixed-point disparity with scale
                    d = static_cast<float>(disparity.at<int16_t>(y, x)) / config_.disparityScale;
                } else if (disparity.type() == CV_32F) {
                    // Floating-point disparity
                    d = disparity.at<float>(y, x);
                } else {
                    // Unsupported type
                    result.success = false;
                    result.errorMessage = "Unsupported disparity type (must be CV_16S or CV_32F)";
                    return result;
                }

                // Check for invalid disparity
                if (d <= 0 || std::isnan(d) || std::isinf(d)) {
                    if (config_.handleInvalidPixels) {
                        result.depthMap.at<float>(y, x) = std::numeric_limits<float>::quiet_NaN();
                        result.validMask.at<uint8_t>(y, x) = 0;
                    } else {
                        result.depthMap.at<float>(y, x) = 0;
                        result.validMask.at<uint8_t>(y, x) = 0;
                    }
                } else {
                    // Calculate 3D point using Q matrix
                    // Z = f * baseline / disparity
                    // More precisely: W = disparity + q33
                    //                 Z = q23 / W

                    double W = d + q33;

                    if (std::abs(W) < 0.001) {
                        // Near-zero denominator
                        result.depthMap.at<float>(y, x) = std::numeric_limits<float>::quiet_NaN();
                        result.validMask.at<uint8_t>(y, x) = 0;
                    } else {
                        // Calculate depth (Z coordinate)
                        float depth = static_cast<float>(q23 / W);

                        // Check depth range
                        if (depth > 0 && depth >= config_.minDepthMM && depth <= config_.maxDepthMM) {
                            result.depthMap.at<float>(y, x) = depth;
                            result.validMask.at<uint8_t>(y, x) = 255;
                            validCount++;
                        } else if (config_.clampToRange && depth > 0) {
                            // Clamp to valid range
                            depth = std::max(config_.minDepthMM, std::min(config_.maxDepthMM, depth));
                            result.depthMap.at<float>(y, x) = depth;
                            result.validMask.at<uint8_t>(y, x) = 128; // Mark as clamped
                            validCount++;
                        } else {
                            // Out of range
                            result.depthMap.at<float>(y, x) = std::numeric_limits<float>::quiet_NaN();
                            result.validMask.at<uint8_t>(y, x) = 0;
                        }
                    }
                }
            }
        }

        // Calculate statistics
        result.totalPixels = width * height;
        result.validPixels = validCount;
        calculateStatistics(result.depthMap, result.validMask, result);

        // TODO: Save debug output when DebugOutputManager singleton is available
        // if (DebugOutputManager::getInstance().isEnabled()) {
        //     // Normalize depth for visualization
        //     cv::Mat depthVis;
        //     cv::Mat validDepth;
        //     result.depthMap.copyTo(validDepth, result.validMask);

        //     double minVal, maxVal;
        //     cv::minMaxLoc(validDepth, &minVal, &maxVal, nullptr, nullptr, result.validMask);

        //     validDepth.convertTo(depthVis, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        //     cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_TURBO);

        //     DebugOutputManager::getInstance().saveDebugImage("depth_map", depthVis);
        //     DebugOutputManager::getInstance().saveDebugImage("depth_valid_mask", result.validMask);
        // }

        result.success = true;

        if (logger_) {
            logger_->info("Depth conversion completed:");
            logger_->info("  Valid pixels: " + std::to_string(result.validPercentage) + "%");
            logger_->info("  Depth range: " + std::to_string(result.minDepth) + " - " +
                         std::to_string(result.maxDepth) + " mm");
            logger_->info("  Mean depth: " + std::to_string(result.meanDepth) + " mm");
        }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "Exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== STATISTICS CALCULATION ==========

void DepthConverter::calculateStatistics(
    const cv::Mat& depthMap,
    const cv::Mat& validMask,
    Result& result)
{
    if (depthMap.empty() || validMask.empty()) {
        result.minDepth = 0;
        result.maxDepth = 0;
        result.meanDepth = 0;
        result.stdDevDepth = 0;
        result.validPercentage = 0;
        return;
    }

    // Calculate min, max, mean
    double minVal, maxVal;
    cv::Scalar meanVal, stdVal;

    cv::minMaxLoc(depthMap, &minVal, &maxVal, nullptr, nullptr, validMask);
    cv::meanStdDev(depthMap, meanVal, stdVal, validMask);

    result.minDepth = static_cast<float>(minVal);
    result.maxDepth = static_cast<float>(maxVal);
    result.meanDepth = static_cast<float>(meanVal[0]);
    result.stdDevDepth = static_cast<float>(stdVal[0]);
    result.validPercentage = (100.0f * result.validPixels) / result.totalPixels;
}

// ========== DEPTH CLAMPING ==========

void DepthConverter::applyDepthClamping(cv::Mat& depthMap, const cv::Mat& validMask) {
    if (!config_.clampToRange) {
        return;
    }

    for (int y = 0; y < depthMap.rows; ++y) {
        float* depthRow = depthMap.ptr<float>(y);
        const uint8_t* maskRow = validMask.ptr<uint8_t>(y);

        for (int x = 0; x < depthMap.cols; ++x) {
            if (maskRow[x] > 0) {
                float& depth = depthRow[x];
                if (!std::isnan(depth)) {
                    depth = std::max(config_.minDepthMM, std::min(config_.maxDepthMM, depth));
                }
            }
        }
    }
}

} // namespace stereo
} // namespace unlook