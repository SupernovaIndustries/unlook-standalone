/**
 * @file SpatialFilter.cpp
 * @brief Implementation of edge-preserving spatial filter
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/SpatialFilter.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <chrono>
#include <cmath>

namespace unlook {
namespace stereo {

using namespace std::chrono;

// ========== CONSTRUCTOR / DESTRUCTOR ==========

SpatialFilter::SpatialFilter(core::Logger* logger)
    : logger_(logger ? logger : &core::Logger::getInstance())
{
    if (logger_) {
        logger_->info("SpatialFilter initialized");
    }
}

SpatialFilter::~SpatialFilter() = default;

// ========== CONFIGURATION ==========

void SpatialFilter::setConfig(const Config& config) {
    config_ = config;

    if (logger_) {
        logger_->info("SpatialFilter configured:");
        logger_->info("  Radius: " + std::to_string(config_.radius) + " pixels");
        logger_->info("  Spatial sigma: " + std::to_string(config_.spatialSigma));
        logger_->info("  Range sigma: " + std::to_string(config_.rangeSigma));
        logger_->info("  Iterations: " + std::to_string(config_.iterations));
        logger_->info("  Edge preserving: " + std::string(config_.preserveEdges ? "yes" : "no"));
    }
}

// ========== MAIN FILTER FUNCTIONS ==========

SpatialFilter::Result SpatialFilter::filter(const cv::Mat& depthMap, const cv::Mat& colorImage) {
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
        // Calculate initial smoothness
        float initialSmoothness = calculateSmoothness(depthMap);

        // Detect edges if edge preservation is enabled
        if (config_.preserveEdges) {
            detectEdges(depthMap, result.edgeMap, config_.edgeThreshold);
            result.edgePixels = cv::countNonZero(result.edgeMap);
        }

        // Apply filtering
        cv::Mat current = depthMap.clone();

        for (int iter = 0; iter < config_.iterations; ++iter) {
            cv::Mat filtered;

            if (config_.useColorGuide && !colorImage.empty()) {
                // Use guided filter with color image
                applyGuidedFilter(current, colorImage, filtered,
                                config_.radius, config_.rangeSigma * 255.0f);
            } else {
                // Use bilateral filter
                applyBilateralFilter(current, filtered, result.edgeMap,
                                   config_.radius, config_.spatialSigma,
                                   config_.rangeSigma);
            }

            current = filtered;
        }

        result.filtered = current;

        // Calculate smoothness improvement
        float finalSmoothness = calculateSmoothness(result.filtered);
        result.smoothnessImprovement = (1.0f - finalSmoothness / initialSmoothness) * 100.0f;

        result.processingTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime);
        result.success = true;

        if (logger_) {
            logger_->info("Spatial filter applied:");
            logger_->info("  Smoothness improvement: " + std::to_string(result.smoothnessImprovement) + "%");
            if (config_.preserveEdges) {
                logger_->info("  Edge pixels: " + std::to_string(result.edgePixels));
            }
            logger_->info("  Processing time: " + std::to_string(result.processingTime.count()) + " µs");
        }

        // TODO: Save debug output when DebugOutputManager singleton is available
        // if (DebugOutputManager::getInstance().isEnabled()) {
        //     if (!result.edgeMap.empty()) {
        //         DebugOutputManager::getInstance().saveDebugImage("spatial_edge_map", result.edgeMap);
        //     }
        //
        //     // Create difference map for visualization
        //     cv::Mat diff;
        //     cv::absdiff(depthMap, result.filtered, diff);
        //     cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
        //     diff.convertTo(diff, CV_8U);
        //     cv::applyColorMap(diff, diff, cv::COLORMAP_JET);
        //     DebugOutputManager::getInstance().saveDebugImage("spatial_diff_map", diff);
        // }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

SpatialFilter::Result SpatialFilter::filterDisparity(const cv::Mat& disparity) {
    Result result;

    // Convert to depth-like representation for filtering
    cv::Mat floatDisp;
    if (disparity.type() == CV_16S) {
        disparity.convertTo(floatDisp, CV_32F, 1.0/16.0);
    } else if (disparity.type() == CV_32F) {
        floatDisp = disparity.clone();
    } else {
        result.success = false;
        result.errorMessage = "Disparity must be CV_16S or CV_32F";
        return result;
    }

    // Apply filter
    Result depthResult = filter(floatDisp);

    if (depthResult.success) {
        // Convert back to original type
        if (disparity.type() == CV_16S) {
            depthResult.filtered.convertTo(result.filtered, CV_16S, 16.0);
        } else {
            result.filtered = depthResult.filtered;
        }

        result.edgeMap = depthResult.edgeMap;
        result.smoothnessImprovement = depthResult.smoothnessImprovement;
        result.edgePixels = depthResult.edgePixels;
        result.processingTime = depthResult.processingTime;
        result.success = true;
    } else {
        result = depthResult;
    }

    return result;
}

// ========== BILATERAL FILTER IMPLEMENTATION ==========

void SpatialFilter::applyBilateralFilter(const cv::Mat& input, cv::Mat& output,
                                         const cv::Mat& edgeMap, int radius,
                                         float spatialSigma, float rangeSigma) {
    output = cv::Mat(input.size(), input.type());

    const float spatialFactor = -0.5f / (spatialSigma * spatialSigma);
    const float rangeFactor = -0.5f / (rangeSigma * rangeSigma);

    for (int y = radius; y < input.rows - radius; ++y) {
        for (int x = radius; x < input.cols - radius; ++x) {
            float centerVal = input.at<float>(y, x);

            // Skip invalid pixels
            if (std::isnan(centerVal) || centerVal <= 0) {
                output.at<float>(y, x) = centerVal;
                continue;
            }

            // Check if this is an edge pixel
            bool isEdge = !edgeMap.empty() && edgeMap.at<uint8_t>(y, x) > 0;

            float sumWeights = 0;
            float sumValues = 0;

            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (dx == 0 && dy == 0) continue;

                    int ny = y + dy;
                    int nx = x + dx;

                    float neighborVal = input.at<float>(ny, nx);

                    // Skip invalid neighbors
                    if (std::isnan(neighborVal) || neighborVal <= 0) {
                        continue;
                    }

                    // Calculate spatial weight
                    float spatialDist = std::sqrt(dx * dx + dy * dy);
                    float spatialWeight = std::exp(spatialDist * spatialDist * spatialFactor);

                    // Calculate range weight
                    float rangeDist = std::abs(neighborVal - centerVal) / centerVal; // Relative difference
                    float rangeWeight = std::exp(rangeDist * rangeDist * rangeFactor);

                    // Reduce weight for edge pixels if preserving edges
                    if (config_.preserveEdges && isEdge) {
                        rangeWeight *= 0.1f; // Strongly reduce influence across edges
                    }

                    float weight = spatialWeight * rangeWeight;
                    sumWeights += weight;
                    sumValues += neighborVal * weight;
                }
            }

            // Include center pixel
            sumWeights += 1.0f;
            sumValues += centerVal;

            output.at<float>(y, x) = sumValues / sumWeights;
        }
    }

    // Copy borders
    for (int y = 0; y < radius; ++y) {
        input.row(y).copyTo(output.row(y));
        input.row(input.rows - 1 - y).copyTo(output.row(output.rows - 1 - y));
    }
    for (int x = 0; x < radius; ++x) {
        input.col(x).copyTo(output.col(x));
        input.col(input.cols - 1 - x).copyTo(output.col(output.cols - 1 - x));
    }
}

// ========== GUIDED FILTER IMPLEMENTATION ==========

void SpatialFilter::applyGuidedFilter(const cv::Mat& input, const cv::Mat& guide,
                                      cv::Mat& output, int radius, float epsilon) {
    // Use OpenCV's guided filter from ximgproc module
    cv::Mat guidedOutput;
    cv::ximgproc::guidedFilter(guide, input, guidedOutput, radius, epsilon);

    // Handle invalid pixels
    output = cv::Mat(input.size(), input.type());
    for (int y = 0; y < input.rows; ++y) {
        for (int x = 0; x < input.cols; ++x) {
            float val = input.at<float>(y, x);
            if (std::isnan(val) || val <= 0) {
                output.at<float>(y, x) = val; // Keep invalid as-is
            } else {
                output.at<float>(y, x) = guidedOutput.at<float>(y, x);
            }
        }
    }
}

// ========== EDGE DETECTION ==========

void SpatialFilter::detectEdges(const cv::Mat& depthMap, cv::Mat& edgeMap, float threshold) {
    edgeMap = cv::Mat::zeros(depthMap.size(), CV_8U);

    // Use Sobel operators for edge detection
    cv::Mat gradX, gradY;
    cv::Sobel(depthMap, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(depthMap, gradY, CV_32F, 0, 1, 3);

    for (int y = 1; y < depthMap.rows - 1; ++y) {
        for (int x = 1; x < depthMap.cols - 1; ++x) {
            float depth = depthMap.at<float>(y, x);

            if (std::isnan(depth) || depth <= 0) {
                continue;
            }

            float gx = gradX.at<float>(y, x);
            float gy = gradY.at<float>(y, x);

            // Calculate gradient magnitude relative to depth
            float gradMag = std::sqrt(gx * gx + gy * gy) / depth;

            if (gradMag > threshold) {
                edgeMap.at<uint8_t>(y, x) = 255;
            }
        }
    }

    // Apply morphological operations to clean up edge map
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(edgeMap, edgeMap, cv::MORPH_CLOSE, kernel);
}

// ========== SMOOTHNESS CALCULATION ==========

float SpatialFilter::calculateSmoothness(const cv::Mat& depthMap) {
    float totalVariation = 0;
    int validCount = 0;

    for (int y = 1; y < depthMap.rows - 1; ++y) {
        for (int x = 1; x < depthMap.cols - 1; ++x) {
            float center = depthMap.at<float>(y, x);

            if (std::isnan(center) || center <= 0) {
                continue;
            }

            // Calculate local variation
            float variation = 0;
            int neighborCount = 0;

            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;

                    float neighbor = depthMap.at<float>(y + dy, x + dx);
                    if (!std::isnan(neighbor) && neighbor > 0) {
                        variation += std::abs(neighbor - center) / center;
                        neighborCount++;
                    }
                }
            }

            if (neighborCount > 0) {
                totalVariation += variation / neighborCount;
                validCount++;
            }
        }
    }

    return (validCount > 0) ? (totalVariation / validCount) : 0;
}

} // namespace stereo
} // namespace unlook