/**
 * @file HoleFiller.cpp
 * @brief Implementation of hole filling for depth maps
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/HoleFiller.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <chrono>
#include <cmath>
#include <queue>

namespace unlook {
namespace stereo {

using namespace std::chrono;

// ========== CONSTRUCTOR / DESTRUCTOR ==========

HoleFiller::HoleFiller(core::Logger* logger)
    : logger_(logger ? logger : &core::Logger::getInstance())
{
    if (logger_) {
        logger_->info("HoleFiller initialized");
    }
}

HoleFiller::~HoleFiller() = default;

// ========== CONFIGURATION ==========

void HoleFiller::setConfig(const Config& config) {
    config_ = config;

    if (logger_) {
        logger_->info("HoleFiller configured:");
        logger_->info("  Method: " +
            (config_.method == Method::NEAREST_NEIGHBOR ? "NEAREST_NEIGHBOR" :
             config_.method == Method::WEIGHTED_AVERAGE ? "WEIGHTED_AVERAGE" :
             config_.method == Method::MORPHOLOGICAL ? "MORPHOLOGICAL" :
             config_.method == Method::INPAINTING ? "INPAINTING" :
             config_.method == Method::DIRECTIONAL ? "DIRECTIONAL" : "HYBRID"));
        logger_->info("  Max hole size: " + std::to_string(config_.maxHoleSize) + " pixels");
        logger_->info("  Max depth gap: " + std::to_string(config_.maxDepthGap) + " mm");
        logger_->info("  Preserve edges: " + std::string(config_.preserveEdges ? "yes" : "no"));
    }
}

// ========== MAIN FILL FUNCTIONS ==========

HoleFiller::Result HoleFiller::fill(const cv::Mat& depthMap, const cv::Mat& colorImage) {
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
        // Detect holes
        detectHoles(depthMap, result.holeMask, result.totalHoles);

        int initialHolePixels = cv::countNonZero(result.holeMask);
        result.fillPercentage = (100.0f * initialHolePixels) / (depthMap.rows * depthMap.cols);

        // Detect edges if needed
        cv::Mat edges;
        if (config_.preserveEdges) {
            detectEdges(depthMap, edges);
        }

        // Apply selected filling method
        switch (config_.method) {
            case Method::NEAREST_NEIGHBOR:
                fillNearestNeighbor(depthMap, result.holeMask, result.filled);
                break;

            case Method::WEIGHTED_AVERAGE:
                fillWeightedAverage(depthMap, result.holeMask, result.filled);
                break;

            case Method::MORPHOLOGICAL:
                fillMorphological(depthMap, result.holeMask, result.filled);
                break;

            case Method::INPAINTING:
                fillInpainting(depthMap, result.holeMask, result.filled);
                break;

            case Method::DIRECTIONAL:
                fillDirectional(depthMap, result.holeMask, edges, result.filled);
                break;

            case Method::HYBRID:
            default:
                fillHybrid(depthMap, result.holeMask, edges, result.filled);
                break;
        }

        // Calculate filled pixels
        cv::Mat remainingHoles;
        detectHoles(result.filled, remainingHoles, result.filledHoles);
        int finalHolePixels = cv::countNonZero(remainingHoles);
        result.filledPixels = initialHolePixels - finalHolePixels;
        result.filledHoles = result.totalHoles - result.filledHoles;

        // Update hole mask to show only filled regions
        cv::Mat filledMask;
        cv::bitwise_and(result.holeMask, cv::Scalar(255) - remainingHoles, filledMask);
        result.holeMask = filledMask;

        result.processingTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime);
        result.success = true;

        if (logger_) {
            logger_->info("Hole filling completed:");
            logger_->info("  Total holes: " + std::to_string(result.totalHoles));
            logger_->info("  Filled holes: " + std::to_string(result.filledHoles));
            logger_->info("  Filled pixels: " + std::to_string(result.filledPixels) +
                         " (" + std::to_string(result.fillPercentage) + "% of image)");
            logger_->info("  Processing time: " + std::to_string(result.processingTime.count()) + " µs");
        }

        // Save debug output
        if (DebugOutputManager::getInstance().isEnabled()) {
            DebugOutputManager::getInstance().saveDebugImage("hole_mask", result.holeMask);

            // Create visualization of filled regions
            cv::Mat filledViz;
            result.filled.convertTo(filledViz, CV_8U, 255.0 / 2000.0); // Assume max 2000mm
            cv::cvtColor(filledViz, filledViz, cv::COLOR_GRAY2BGR);

            // Highlight filled regions in green
            for (int y = 0; y < result.holeMask.rows; ++y) {
                for (int x = 0; x < result.holeMask.cols; ++x) {
                    if (result.holeMask.at<uint8_t>(y, x) > 0) {
                        filledViz.at<cv::Vec3b>(y, x)[1] = 255; // Green channel
                    }
                }
            }

            DebugOutputManager::getInstance().saveDebugImage("hole_filled_viz", filledViz);
        }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

HoleFiller::Result HoleFiller::fillDisparity(const cv::Mat& disparity) {
    Result result;

    // Convert to depth-like representation
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

    // Apply hole filling
    Result depthResult = fill(floatDisp);

    if (depthResult.success) {
        // Convert back to original type
        if (disparity.type() == CV_16S) {
            depthResult.filled.convertTo(result.filled, CV_16S, 16.0);
        } else {
            result.filled = depthResult.filled;
        }

        result.holeMask = depthResult.holeMask;
        result.totalHoles = depthResult.totalHoles;
        result.filledHoles = depthResult.filledHoles;
        result.filledPixels = depthResult.filledPixels;
        result.fillPercentage = depthResult.fillPercentage;
        result.processingTime = depthResult.processingTime;
        result.success = true;
    } else {
        result = depthResult;
    }

    return result;
}

// ========== HOLE DETECTION ==========

void HoleFiller::detectHoles(const cv::Mat& depthMap, cv::Mat& holeMask, int& numHoles) {
    holeMask = cv::Mat::zeros(depthMap.size(), CV_8U);

    for (int y = 0; y < depthMap.rows; ++y) {
        const float* depthRow = depthMap.ptr<float>(y);
        uint8_t* maskRow = holeMask.ptr<uint8_t>(y);

        for (int x = 0; x < depthMap.cols; ++x) {
            float val = depthRow[x];
            if (std::isnan(val) || val <= 0) {
                maskRow[x] = 255;
            }
        }
    }

    // Count connected components (individual holes)
    cv::Mat labels;
    numHoles = cv::connectedComponents(holeMask, labels, 8) - 1; // Subtract background
}

// ========== NEAREST NEIGHBOR FILLING ==========

void HoleFiller::fillNearestNeighbor(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output) {
    output = input.clone();

    // For each hole pixel, find nearest valid neighbor
    for (int y = 0; y < input.rows; ++y) {
        for (int x = 0; x < input.cols; ++x) {
            if (holeMask.at<uint8_t>(y, x) == 0) continue;

            // Search in expanding radius for valid value
            float fillValue = 0;
            bool found = false;

            for (int radius = 1; radius <= config_.maxHoleSize && !found; ++radius) {
                for (int dy = -radius; dy <= radius && !found; ++dy) {
                    for (int dx = -radius; dx <= radius && !found; ++dx) {
                        if (std::abs(dy) != radius && std::abs(dx) != radius) continue;

                        int ny = y + dy;
                        int nx = x + dx;

                        if (ny >= 0 && ny < input.rows && nx >= 0 && nx < input.cols) {
                            float val = input.at<float>(ny, nx);
                            if (!std::isnan(val) && val > 0) {
                                fillValue = val;
                                found = true;
                            }
                        }
                    }
                }
            }

            if (found) {
                output.at<float>(y, x) = fillValue;
            }
        }
    }
}

// ========== WEIGHTED AVERAGE FILLING ==========

void HoleFiller::fillWeightedAverage(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output) {
    output = input.clone();

    const int radius = 5;

    for (int y = radius; y < input.rows - radius; ++y) {
        for (int x = radius; x < input.cols - radius; ++x) {
            if (holeMask.at<uint8_t>(y, x) == 0) continue;

            float sumValues = 0;
            float sumWeights = 0;

            for (int dy = -radius; dy <= radius; ++dy) {
                for (int dx = -radius; dx <= radius; ++dx) {
                    if (dy == 0 && dx == 0) continue;

                    int ny = y + dy;
                    int nx = x + dx;

                    float val = input.at<float>(ny, nx);
                    if (!std::isnan(val) && val > 0) {
                        float dist = std::sqrt(dy * dy + dx * dx);
                        float weight = 1.0f / (1.0f + dist);

                        sumValues += val * weight;
                        sumWeights += weight;
                    }
                }
            }

            if (sumWeights > 0) {
                output.at<float>(y, x) = sumValues / sumWeights;
            }
        }
    }
}

// ========== MORPHOLOGICAL FILLING ==========

void HoleFiller::fillMorphological(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output) {
    // Create a version where holes are set to 0
    output = input.clone();
    for (int y = 0; y < input.rows; ++y) {
        for (int x = 0; x < input.cols; ++x) {
            if (holeMask.at<uint8_t>(y, x) > 0) {
                output.at<float>(y, x) = 0;
            }
        }
    }

    // Apply morphological closing
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(config_.dilationSize * 2 + 1,
                                                       config_.dilationSize * 2 + 1));

    cv::Mat dilated, eroded;
    cv::dilate(output, dilated, kernel);
    cv::erode(dilated, eroded, kernel);

    // Copy filled values back
    for (int y = 0; y < input.rows; ++y) {
        for (int x = 0; x < input.cols; ++x) {
            if (holeMask.at<uint8_t>(y, x) > 0 && eroded.at<float>(y, x) > 0) {
                output.at<float>(y, x) = eroded.at<float>(y, x);
            }
        }
    }
}

// ========== INPAINTING FILLING ==========

void HoleFiller::fillInpainting(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output) {
    // Convert to 8-bit for inpainting
    cv::Mat input8U, mask8U;
    double minVal, maxVal;
    cv::minMaxLoc(input, &minVal, &maxVal, nullptr, nullptr, cv::Scalar(255) - holeMask);

    input.convertTo(input8U, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    mask8U = holeMask;

    cv::Mat inpainted;
    cv::inpaint(input8U, mask8U, inpainted, 3, cv::INPAINT_TELEA);

    // Convert back to float
    inpainted.convertTo(output, CV_32F, (maxVal - minVal) / 255.0, minVal);

    // Restore original valid values
    for (int y = 0; y < input.rows; ++y) {
        for (int x = 0; x < input.cols; ++x) {
            if (holeMask.at<uint8_t>(y, x) == 0) {
                output.at<float>(y, x) = input.at<float>(y, x);
            }
        }
    }
}

// ========== DIRECTIONAL FILLING ==========

void HoleFiller::fillDirectional(const cv::Mat& input, const cv::Mat& holeMask,
                                 const cv::Mat& edges, cv::Mat& output) {
    output = input.clone();

    // Propagate values along edges
    for (int y = 1; y < input.rows - 1; ++y) {
        for (int x = 1; x < input.cols - 1; ++x) {
            if (holeMask.at<uint8_t>(y, x) == 0) continue;

            // Check if on an edge
            bool nearEdge = false;
            if (!edges.empty()) {
                for (int dy = -1; dy <= 1 && !nearEdge; ++dy) {
                    for (int dx = -1; dx <= 1 && !nearEdge; ++dx) {
                        if (edges.at<uint8_t>(y + dy, x + dx) > 0) {
                            nearEdge = true;
                        }
                    }
                }
            }

            if (nearEdge) {
                // Use nearest neighbor for edge pixels
                float minDist = FLT_MAX;
                float fillValue = 0;

                for (int dy = -3; dy <= 3; ++dy) {
                    for (int dx = -3; dx <= 3; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;

                        if (ny >= 0 && ny < input.rows && nx >= 0 && nx < input.cols) {
                            float val = input.at<float>(ny, nx);
                            if (!std::isnan(val) && val > 0) {
                                float dist = std::sqrt(dy * dy + dx * dx);
                                if (dist < minDist) {
                                    minDist = dist;
                                    fillValue = val;
                                }
                            }
                        }
                    }
                }

                if (fillValue > 0) {
                    output.at<float>(y, x) = fillValue;
                }
            } else {
                // Use weighted average for non-edge pixels
                float sumValues = 0;
                float sumWeights = 0;

                for (int dy = -2; dy <= 2; ++dy) {
                    for (int dx = -2; dx <= 2; ++dx) {
                        int ny = y + dy;
                        int nx = x + dx;

                        if (ny >= 0 && ny < input.rows && nx >= 0 && nx < input.cols) {
                            float val = input.at<float>(ny, nx);
                            if (!std::isnan(val) && val > 0) {
                                float weight = 1.0f / (1.0f + std::abs(dy) + std::abs(dx));
                                sumValues += val * weight;
                                sumWeights += weight;
                            }
                        }
                    }
                }

                if (sumWeights > 0) {
                    output.at<float>(y, x) = sumValues / sumWeights;
                }
            }
        }
    }
}

// ========== HYBRID FILLING ==========

void HoleFiller::fillHybrid(const cv::Mat& input, const cv::Mat& holeMask,
                            const cv::Mat& edges, cv::Mat& output) {
    // Analyze hole characteristics
    std::vector<cv::Rect> holeRegions;
    std::vector<int> holeSizes;
    analyzeHoles(holeMask, holeRegions, holeSizes);

    output = input.clone();

    // Apply different methods based on hole size
    for (size_t i = 0; i < holeRegions.size(); ++i) {
        cv::Rect region = holeRegions[i];
        int holeSize = holeSizes[i];

        cv::Mat regionInput = input(region);
        cv::Mat regionMask = holeMask(region);
        cv::Mat regionOutput = output(region);

        if (holeSize < 10) {
            // Small holes: use nearest neighbor
            fillNearestNeighbor(regionInput, regionMask, regionOutput);
        } else if (holeSize < 50) {
            // Medium holes: use weighted average
            fillWeightedAverage(regionInput, regionMask, regionOutput);
        } else if (holeSize < config_.maxHoleSize) {
            // Large holes: use morphological or inpainting
            if (config_.preserveEdges && !edges.empty()) {
                cv::Mat regionEdges = edges(region);
                fillDirectional(regionInput, regionMask, regionEdges, regionOutput);
            } else {
                fillMorphological(regionInput, regionMask, regionOutput);
            }
        }
        // Very large holes are left unfilled
    }
}

// ========== HOLE ANALYSIS ==========

void HoleFiller::analyzeHoles(const cv::Mat& holeMask, std::vector<cv::Rect>& holeRegions,
                              std::vector<int>& holeSizes) {
    cv::Mat labels, stats, centroids;
    int numComponents = cv::connectedComponentsWithStats(holeMask, labels, stats, centroids, 8);

    holeRegions.clear();
    holeSizes.clear();

    for (int i = 1; i < numComponents; ++i) { // Skip background (label 0)
        cv::Rect region(stats.at<int>(i, cv::CC_STAT_LEFT),
                       stats.at<int>(i, cv::CC_STAT_TOP),
                       stats.at<int>(i, cv::CC_STAT_WIDTH),
                       stats.at<int>(i, cv::CC_STAT_HEIGHT));
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        holeRegions.push_back(region);
        holeSizes.push_back(area);
    }
}

// ========== EDGE DETECTION ==========

void HoleFiller::detectEdges(const cv::Mat& depthMap, cv::Mat& edges) {
    edges = cv::Mat::zeros(depthMap.size(), CV_8U);

    // Calculate gradients
    cv::Mat gradX, gradY;
    cv::Sobel(depthMap, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(depthMap, gradY, CV_32F, 0, 1, 3);

    for (int y = 1; y < depthMap.rows - 1; ++y) {
        for (int x = 1; x < depthMap.cols - 1; ++x) {
            float depth = depthMap.at<float>(y, x);

            if (std::isnan(depth) || depth <= 0) continue;

            float gx = gradX.at<float>(y, x);
            float gy = gradY.at<float>(y, x);

            // Relative gradient magnitude
            float gradMag = std::sqrt(gx * gx + gy * gy) / depth;

            if (gradMag > config_.edgeThreshold) {
                edges.at<uint8_t>(y, x) = 255;
            }
        }
    }
}

} // namespace stereo
} // namespace unlook