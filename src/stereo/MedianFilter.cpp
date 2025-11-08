/**
 * @file MedianFilter.cpp
 * @brief Implementation of median filter for speckle removal
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/MedianFilter.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <arm_neon.h>
#include <chrono>
#include <algorithm>
#include <vector>

namespace unlook {
namespace stereo {

using namespace std::chrono;

// ========== CONSTRUCTOR / DESTRUCTOR ==========

MedianFilter::MedianFilter(core::Logger* logger)
    : logger_(logger ? logger : &core::Logger::getInstance())
{
    if (logger_) {
        logger_->info("MedianFilter initialized");
    }
}

MedianFilter::~MedianFilter() = default;

// ========== CONFIGURATION ==========

void MedianFilter::setConfig(const Config& config) {
    config_ = config;

    if (logger_) {
        logger_->info("MedianFilter configured:");
        logger_->info("  Kernel size: " + std::to_string(config_.kernelSize) + "x" +
                     std::to_string(config_.kernelSize));
        logger_->info("  Edge preserving: " + std::string(config_.preserveEdges ? "yes" : "no"));
        if (config_.preserveEdges) {
            logger_->info("  Edge threshold: " + std::to_string(config_.edgeThreshold) + " mm");
        }
        logger_->info("  NEON optimization: " + std::string(config_.useNEON ? "enabled" : "disabled"));
    }
}

// ========== MAIN FILTER FUNCTIONS ==========

MedianFilter::Result MedianFilter::filterDepth(const cv::Mat& depthMap) {
    Result result;
    auto startTime = high_resolution_clock::now();

    if (depthMap.empty()) {
        result.success = false;
        result.errorMessage = "Empty depth map";
        return result;
    }

    if (depthMap.type() != CV_32F) {
        result.success = false;
        result.errorMessage = "Depth map must be CV_32F";
        return result;
    }

    try {
        // Apply median filter
        if (config_.preserveEdges) {
            applyEdgePreservingMedian(depthMap, result.filtered,
                                     config_.kernelSize, config_.edgeThreshold);
        } else if (config_.useNEON && config_.kernelSize == Config::KERNEL_5x5) {
            // Use NEON optimization for 5x5 kernel
            median5x5NEON(depthMap, result.filtered);
        } else {
            // Use standard OpenCV median filter
            applyMedianFilter(depthMap, result.filtered, config_.kernelSize);
        }

        // Detect speckles that were removed
        detectSpeckles(depthMap, result.filtered, result.speckleMask);

        // Calculate statistics
        result.specklesRemoved = cv::countNonZero(result.speckleMask);
        result.specklePercentage = (100.0f * result.specklesRemoved) /
                                  (depthMap.rows * depthMap.cols);

        result.processingTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime);
        result.success = true;

        if (logger_) {
            logger_->info("Median filter applied:");
            logger_->info("  Speckles removed: " + std::to_string(result.specklesRemoved) +
                         " (" + std::to_string(result.specklePercentage) + "%)");
            logger_->info("  Processing time: " + std::to_string(result.processingTime.count()) + " µs");
        }

        // TODO: Save debug output when DebugOutputManager singleton is available
        // if (DebugOutputManager::getInstance().isEnabled()) {
        //     DebugOutputManager::getInstance().saveDebugImage("median_speckle_mask", result.speckleMask);
        // }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

MedianFilter::Result MedianFilter::filterDisparity(const cv::Mat& disparity) {
    Result result;
    auto startTime = high_resolution_clock::now();

    if (disparity.empty()) {
        result.success = false;
        result.errorMessage = "Empty disparity map";
        return result;
    }

    try {
        cv::Mat floatDisp;

        // Convert to float for processing
        if (disparity.type() == CV_16S) {
            disparity.convertTo(floatDisp, CV_32F, 1.0/16.0);
        } else if (disparity.type() == CV_32F) {
            floatDisp = disparity.clone();
        } else {
            result.success = false;
            result.errorMessage = "Disparity must be CV_16S or CV_32F";
            return result;
        }

        // Apply median filter
        cv::Mat filteredFloat;
        if (config_.preserveEdges) {
            applyEdgePreservingMedian(floatDisp, filteredFloat,
                                     config_.kernelSize, config_.edgeThreshold / 10.0f); // Adjust threshold for disparity
        } else if (config_.useNEON && config_.kernelSize == Config::KERNEL_5x5) {
            median5x5NEON(floatDisp, filteredFloat);
        } else {
            applyMedianFilter(floatDisp, filteredFloat, config_.kernelSize);
        }

        // Convert back to original type
        if (disparity.type() == CV_16S) {
            filteredFloat.convertTo(result.filtered, CV_16S, 16.0);
        } else {
            result.filtered = filteredFloat;
        }

        // Detect speckles
        detectSpeckles(floatDisp, filteredFloat, result.speckleMask);

        // Calculate statistics
        result.specklesRemoved = cv::countNonZero(result.speckleMask);
        result.specklePercentage = (100.0f * result.specklesRemoved) /
                                  (disparity.rows * disparity.cols);

        result.processingTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime);
        result.success = true;

        if (logger_) {
            logger_->info("Median filter applied to disparity:");
            logger_->info("  Speckles removed: " + std::to_string(result.specklesRemoved));
            logger_->info("  Processing time: " + std::to_string(result.processingTime.count()) + " µs");
        }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== FILTER IMPLEMENTATIONS ==========

void MedianFilter::applyMedianFilter(const cv::Mat& input, cv::Mat& output, int kernelSize) {
    cv::medianBlur(input, output, kernelSize);
}

void MedianFilter::applyEdgePreservingMedian(const cv::Mat& input, cv::Mat& output,
                                             int kernelSize, float edgeThreshold) {
    output = cv::Mat(input.size(), input.type());
    int halfKernel = kernelSize / 2;

    for (int y = halfKernel; y < input.rows - halfKernel; ++y) {
        for (int x = halfKernel; x < input.cols - halfKernel; ++x) {
            float centerVal = input.at<float>(y, x);

            // Skip invalid pixels
            if (std::isnan(centerVal) || centerVal <= 0) {
                output.at<float>(y, x) = centerVal;
                continue;
            }

            // Collect neighborhood values that are within edge threshold
            std::vector<float> neighbors;
            neighbors.reserve(kernelSize * kernelSize);

            for (int dy = -halfKernel; dy <= halfKernel; ++dy) {
                for (int dx = -halfKernel; dx <= halfKernel; ++dx) {
                    float val = input.at<float>(y + dy, x + dx);

                    // Only include valid values within edge threshold
                    if (!std::isnan(val) && val > 0) {
                        if (std::abs(val - centerVal) <= edgeThreshold) {
                            neighbors.push_back(val);
                        }
                    }
                }
            }

            // Compute median of valid neighbors
            if (!neighbors.empty()) {
                size_t n = neighbors.size() / 2;
                std::nth_element(neighbors.begin(), neighbors.begin() + n, neighbors.end());
                output.at<float>(y, x) = neighbors[n];
            } else {
                // No valid neighbors within threshold, keep original
                output.at<float>(y, x) = centerVal;
            }
        }
    }

    // Copy borders
    for (int y = 0; y < halfKernel; ++y) {
        input.row(y).copyTo(output.row(y));
        input.row(input.rows - 1 - y).copyTo(output.row(output.rows - 1 - y));
    }
    for (int x = 0; x < halfKernel; ++x) {
        input.col(x).copyTo(output.col(x));
        input.col(input.cols - 1 - x).copyTo(output.col(output.cols - 1 - x));
    }
}

void MedianFilter::median5x5NEON(const cv::Mat& input, cv::Mat& output) {
#ifdef __ARM_NEON
    output = cv::Mat(input.size(), input.type());
    const int halfKernel = 2;

    // Process center region with NEON
    for (int y = halfKernel; y < input.rows - halfKernel; ++y) {
        for (int x = halfKernel; x < input.cols - halfKernel - 3; x += 4) {
            // Load 5x5 neighborhood for 4 pixels at once
            float vals[25][4];
            int idx = 0;

            for (int dy = -halfKernel; dy <= halfKernel; ++dy) {
                for (int dx = -halfKernel; dx <= halfKernel; ++dx) {
                    for (int i = 0; i < 4; ++i) {
                        vals[idx][i] = input.at<float>(y + dy, x + dx + i);
                    }
                    idx++;
                }
            }

            // Find median for each of the 4 pixels
            for (int i = 0; i < 4; ++i) {
                std::vector<float> pixelVals;
                for (int j = 0; j < 25; ++j) {
                    if (!std::isnan(vals[j][i]) && vals[j][i] > 0) {
                        pixelVals.push_back(vals[j][i]);
                    }
                }

                if (!pixelVals.empty()) {
                    size_t n = pixelVals.size() / 2;
                    std::nth_element(pixelVals.begin(), pixelVals.begin() + n, pixelVals.end());
                    output.at<float>(y, x + i) = pixelVals[n];
                } else {
                    output.at<float>(y, x + i) = input.at<float>(y, x + i);
                }
            }
        }

        // Handle remaining pixels
        for (int x = input.cols - halfKernel - (input.cols - 2 * halfKernel) % 4;
             x < input.cols - halfKernel; ++x) {
            std::vector<float> neighbors;
            for (int dy = -halfKernel; dy <= halfKernel; ++dy) {
                for (int dx = -halfKernel; dx <= halfKernel; ++dx) {
                    float val = input.at<float>(y + dy, x + dx);
                    if (!std::isnan(val) && val > 0) {
                        neighbors.push_back(val);
                    }
                }
            }

            if (!neighbors.empty()) {
                size_t n = neighbors.size() / 2;
                std::nth_element(neighbors.begin(), neighbors.begin() + n, neighbors.end());
                output.at<float>(y, x) = neighbors[n];
            } else {
                output.at<float>(y, x) = input.at<float>(y, x);
            }
        }
    }

    // Copy borders
    for (int y = 0; y < halfKernel; ++y) {
        input.row(y).copyTo(output.row(y));
        input.row(input.rows - 1 - y).copyTo(output.row(output.rows - 1 - y));
    }
    for (int x = 0; x < halfKernel; ++x) {
        input.col(x).copyTo(output.col(x));
        input.col(input.cols - 1 - x).copyTo(output.col(output.cols - 1 - x));
    }
#else
    // Fallback to standard median filter if NEON not available
    applyMedianFilter(input, output, 5);
#endif
}

// ========== SPECKLE DETECTION ==========

void MedianFilter::detectSpeckles(const cv::Mat& original, const cv::Mat& filtered, cv::Mat& speckleMask) {
    speckleMask = cv::Mat::zeros(original.size(), CV_8U);

    for (int y = 0; y < original.rows; ++y) {
        const float* origRow = original.ptr<float>(y);
        const float* filtRow = filtered.ptr<float>(y);
        uint8_t* maskRow = speckleMask.ptr<uint8_t>(y);

        for (int x = 0; x < original.cols; ++x) {
            float origVal = origRow[x];
            float filtVal = filtRow[x];

            // Mark as speckle if value changed significantly
            if (!std::isnan(origVal) && !std::isnan(filtVal) && origVal > 0) {
                if (std::abs(origVal - filtVal) > config_.edgeThreshold * 0.1f) {
                    maskRow[x] = 255;
                }
            }
        }
    }
}

} // namespace stereo
} // namespace unlook