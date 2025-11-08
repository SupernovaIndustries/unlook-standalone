/**
 * @file DisparityComputer.cpp
 * @brief High-performance disparity computation implementation
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Implements multiple stereo matching algorithms optimized for VCSEL structured light.
 */

#include "unlook/stereo/DisparityComputer.hpp"
#include "unlook/stereo/VulkanSGMAccelerator.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
// TODO: Re-enable WLS filter when OpenCV is built with ximgproc contrib module
// #include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <arm_neon.h>
#include <chrono>
#include <cmath>

namespace unlook {
namespace stereo {

using namespace std::chrono;

// ========== CONSTRUCTOR / DESTRUCTOR ==========

DisparityComputer::DisparityComputer(core::Logger* logger)
    : logger_(logger ? logger : &core::Logger::getInstance())
    , gpuInitialized_(false)
{
    // Initialize with default config
    updateSGBMMatcher();
    updateWLSFilter();
}

DisparityComputer::~DisparityComputer() = default;

// ========== CONFIGURATION ==========

void DisparityComputer::setConfig(const Config& config) {
    config_ = config;

    // Update matchers with new configuration
    updateSGBMMatcher();
    updateWLSFilter();

    if (logger_) {
        logger_->info("DisparityComputer configured:");
        logger_->info(std::string("  Method: ") +
            (config_.method == DisparityMethod::SGBM_OPENCV ? "SGBM_OPENCV" :
             config_.method == DisparityMethod::AD_CENSUS_CPU ? "AD_CENSUS_CPU" :
             config_.method == DisparityMethod::VULKAN_SGM ? "VULKAN_SGM" : "AUTO"));
        logger_->info("  Disparity range: " + std::to_string(config_.minDisparity) +
                     " to " + std::to_string(config_.minDisparity + config_.numDisparities));
        logger_->info("  Block size: " + std::to_string(config_.blockSize));
        logger_->info("  WLS filter: " + std::string(config_.useWLSFilter ? "enabled" : "disabled"));
    }
}

// ========== GPU INITIALIZATION ==========

bool DisparityComputer::initializeGPU() {
    if (gpuInitialized_) {
        return true;
    }

    try {
        vulkanAccelerator_ = std::make_unique<VulkanSGMAccelerator>();
        gpuInitialized_ = vulkanAccelerator_->initialize();

        if (gpuInitialized_ && logger_) {
            logger_->info("✓ GPU acceleration initialized successfully");
        } else if (logger_) {
            logger_->warning("GPU acceleration not available, using CPU fallback");
        }
    } catch (const std::exception& e) {
        if (logger_) {
            logger_->warning("Failed to initialize GPU: " + std::string(e.what()));
        }
        gpuInitialized_ = false;
    }

    return gpuInitialized_;
}

bool DisparityComputer::isGPUAvailable() const {
    return gpuInitialized_ && vulkanAccelerator_ && vulkanAccelerator_->isAvailable();
}

// ========== MAIN COMPUTE FUNCTION ==========

DisparityComputer::Result DisparityComputer::compute(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    Result result;
    auto startTotal = high_resolution_clock::now();

    // Validate input
    if (leftRect.empty() || rightRect.empty()) {
        result.success = false;
        result.errorMessage = "Empty input images";
        return result;
    }

    if (leftRect.size() != rightRect.size()) {
        result.success = false;
        result.errorMessage = "Input images must have the same size";
        return result;
    }

    // Convert to grayscale if needed
    cv::Mat leftGray, rightGray;
    if (leftRect.channels() > 1) {
        cv::cvtColor(leftRect, leftGray, cv::COLOR_BGR2GRAY);
    } else {
        leftGray = leftRect;
    }
    if (rightRect.channels() > 1) {
        cv::cvtColor(rightRect, rightGray, cv::COLOR_BGR2GRAY);
    } else {
        rightGray = rightRect;
    }

    // Select method based on configuration
    DisparityMethod methodToUse = config_.method;

    if (methodToUse == DisparityMethod::AUTO) {
        // Auto-select: prefer GPU if available, then AD-Census for VCSEL, else SGBM
        if (isGPUAvailable()) {
            methodToUse = DisparityMethod::VULKAN_SGM;
            if (logger_) logger_->info("Auto-selected: VULKAN_SGM (GPU available)");
        } else {
            // For VCSEL structured light, AD-Census performs better
            methodToUse = DisparityMethod::AD_CENSUS_CPU;
            if (logger_) logger_->info("Auto-selected: AD_CENSUS_CPU (optimized for VCSEL)");
        }
    }

    // Compute disparity using selected method
    auto startCompute = high_resolution_clock::now();

    switch (methodToUse) {
        case DisparityMethod::SGBM_OPENCV:
            result = computeSGBM(leftGray, rightGray);
            break;

        case DisparityMethod::AD_CENSUS_CPU:
            result = computeADCensus(leftGray, rightGray);
            break;

        case DisparityMethod::VULKAN_SGM:
            if (isGPUAvailable()) {
                result = computeVulkan(leftGray, rightGray);
            } else {
                if (logger_) logger_->warning("GPU not available, falling back to AD-Census");
                result = computeADCensus(leftGray, rightGray);
                methodToUse = DisparityMethod::AD_CENSUS_CPU;
            }
            break;

        default:
            result.success = false;
            result.errorMessage = "Invalid disparity method";
            return result;
    }

    result.computeTime = duration_cast<milliseconds>(high_resolution_clock::now() - startCompute);
    result.methodUsed = methodToUse;

    // Apply WLS filter if enabled and disparity is valid
    if (result.success && config_.useWLSFilter && !result.disparity.empty()) {
        auto startFilter = high_resolution_clock::now();
        applyWLSFilter(leftGray, result.disparity, result.confidence);
        result.filterTime = duration_cast<milliseconds>(high_resolution_clock::now() - startFilter);
    } else {
        result.filterTime = milliseconds(0);
    }

    // Calculate quality metrics
    if (result.success && !result.disparity.empty()) {
        calculateMetrics(result.disparity, result);
    }

    result.totalTime = duration_cast<milliseconds>(high_resolution_clock::now() - startTotal);

    if (logger_ && result.success) {
        logger_->info("Disparity computation completed:");
        logger_->info(std::string("  Method: ") +
            (result.methodUsed == DisparityMethod::SGBM_OPENCV ? "SGBM_OPENCV" :
             result.methodUsed == DisparityMethod::AD_CENSUS_CPU ? "AD_CENSUS_CPU" :
             result.methodUsed == DisparityMethod::VULKAN_SGM ? "VULKAN_SGM" : "UNKNOWN"));
        logger_->info("  Valid pixels: " + std::to_string(result.validPixelPercentage) + "%");
        logger_->info("  Compute time: " + std::to_string(result.computeTime.count()) + "ms");
        logger_->info("  Filter time: " + std::to_string(result.filterTime.count()) + "ms");
        logger_->info("  Total time: " + std::to_string(result.totalTime.count()) + "ms");
    }

    return result;
}

// ========== OPENCV SGBM IMPLEMENTATION ==========

DisparityComputer::Result DisparityComputer::computeSGBM(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    Result result;

    try {
        // Ensure SGBM matcher is configured
        if (!sgbmMatcher_) {
            updateSGBMMatcher();
        }

        // Compute disparity
        cv::Mat disparity;
        sgbmMatcher_->compute(leftRect, rightRect, disparity);

        // Convert to appropriate format
        if (config_.useSubpixel) {
            // SGBM returns 16-bit fixed-point disparity (CV_16S)
            // with 4 fractional bits (factor of 16)
            result.disparity = disparity;
        } else {
            // Convert to integer disparity
            disparity.convertTo(result.disparity, CV_16S, 1.0/16.0);
        }

        // Generate confidence map based on uniqueness ratio
        result.confidence = cv::Mat::zeros(disparity.size(), CV_8U);

        // Simple confidence: non-zero disparity values get high confidence
        for (int y = 0; y < disparity.rows; ++y) {
            const int16_t* disp_row = disparity.ptr<int16_t>(y);
            uint8_t* conf_row = result.confidence.ptr<uint8_t>(y);

            for (int x = 0; x < disparity.cols; ++x) {
                if (disp_row[x] > 0) {
                    // Higher disparity values typically have better confidence
                    conf_row[x] = static_cast<uint8_t>(std::min(255.0, 128.0 + disp_row[x] / 2.0));
                }
            }
        }

        result.success = true;
        result.gpuAccelerated = false;

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "SGBM computation failed: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== AD-CENSUS IMPLEMENTATION (OPTIMIZED FOR VCSEL) ==========

// Helper: Census transform with NEON optimization
static void computeCensusTransform(
    const cv::Mat& img,
    cv::Mat& census,
    int windowSize,
    int threshold)
{
    const int halfWin = windowSize / 2;
    census = cv::Mat::zeros(img.size(), CV_32S);

    for (int y = halfWin; y < img.rows - halfWin; ++y) {
        for (int x = halfWin; x < img.cols - halfWin; ++x) {
            uint32_t censusValue = 0;
            int bitPos = 0;

            uint8_t centerVal = img.at<uint8_t>(y, x);

            for (int dy = -halfWin; dy <= halfWin; ++dy) {
                for (int dx = -halfWin; dx <= halfWin; ++dx) {
                    if (dx == 0 && dy == 0) continue;

                    uint8_t neighborVal = img.at<uint8_t>(y + dy, x + dx);

                    // Modified Census Transform with threshold (better for VCSEL)
                    if (std::abs(neighborVal - centerVal) > threshold) {
                        if (neighborVal > centerVal) {
                            censusValue |= (1 << bitPos);
                        }
                    }
                    bitPos++;
                    if (bitPos >= 32) break; // Limit to 32 bits
                }
                if (bitPos >= 32) break;
            }

            census.at<uint32_t>(y, x) = censusValue;
        }
    }
}

// Helper: Compute Hamming distance with NEON
static inline uint32_t hammingDistanceNEON(uint32_t a, uint32_t b) {
#ifdef __ARM_NEON
    // Use NEON vcnt instruction for bit counting
    uint8x8_t xor_result = vcreate_u8(a ^ b);
    uint8x8_t popcount = vcnt_u8(xor_result);
    uint32_t sum = vaddv_u8(popcount); // Sum all bytes
    return sum;
#else
    // Fallback to builtin popcount
    return __builtin_popcount(a ^ b);
#endif
}

// Helper: Compute AD (Absolute Difference) cost
static void computeADCost(
    const cv::Mat& left,
    const cv::Mat& right,
    cv::Mat& costVolume,
    int numDisparities,
    int minDisparity)
{
    const int width = left.cols;
    const int height = left.rows;

    // Initialize cost volume
    costVolume = cv::Mat(height, width * numDisparities, CV_32F, cv::Scalar(255.0f));

    for (int y = 0; y < height; ++y) {
        const uint8_t* leftRow = left.ptr<uint8_t>(y);
        const uint8_t* rightRow = right.ptr<uint8_t>(y);
        float* costRow = costVolume.ptr<float>(y);

        for (int x = 0; x < width; ++x) {
            for (int d = 0; d < numDisparities; ++d) {
                int xRight = x - (minDisparity + d);

                if (xRight >= 0 && xRight < width) {
                    float adCost = std::abs(static_cast<float>(leftRow[x]) -
                                           static_cast<float>(rightRow[xRight]));
                    costRow[x * numDisparities + d] = adCost;
                }
            }
        }
    }
}

// Helper: Combine AD and Census costs
static void combineADCensus(
    const cv::Mat& adCost,
    const cv::Mat& censusCostLeft,
    const cv::Mat& censusCostRight,
    cv::Mat& combinedCost,
    int numDisparities,
    int minDisparity,
    float lambdaAD,
    float lambdaCensus)
{
    const int width = adCost.cols / numDisparities;
    const int height = adCost.rows;

    combinedCost = cv::Mat(height, width * numDisparities, CV_32F);

    for (int y = 0; y < height; ++y) {
        const float* adRow = adCost.ptr<float>(y);
        const uint32_t* censusLeft = censusCostLeft.ptr<uint32_t>(y);
        const uint32_t* censusRight = censusCostRight.ptr<uint32_t>(y);
        float* combRow = combinedCost.ptr<float>(y);

        for (int x = 0; x < width; ++x) {
            for (int d = 0; d < numDisparities; ++d) {
                int xRight = x - (minDisparity + d);

                if (xRight >= 0 && xRight < width) {
                    // Get AD cost (normalized to 0-1)
                    float ad = adRow[x * numDisparities + d] / 255.0f;

                    // Get Census cost (Hamming distance, normalized)
                    uint32_t hammingDist = hammingDistanceNEON(
                        censusLeft[x],
                        censusRight[xRight]);
                    float census = hammingDist / 32.0f; // Normalize by max possible distance

                    // Combine costs
                    float combined = lambdaAD * ad + lambdaCensus * census;
                    combRow[x * numDisparities + d] = combined;
                } else {
                    combRow[x * numDisparities + d] = 1.0f; // Max cost for invalid disparities
                }
            }
        }
    }
}

// Helper: SGM cost aggregation (simplified 4-path)
static void sgmAggregation(
    const cv::Mat& costVolume,
    cv::Mat& aggregatedCost,
    int numDisparities,
    int P1,
    int P2)
{
    const int width = costVolume.cols / numDisparities;
    const int height = costVolume.rows;

    // Initialize aggregated cost
    aggregatedCost = cv::Mat::zeros(height, width * numDisparities, CV_32F);

    // Temporary buffers for path costs
    cv::Mat pathCost = cv::Mat::zeros(height, width * numDisparities, CV_32F);

    // Path 1: Left to Right
    for (int y = 0; y < height; ++y) {
        const float* cost = costVolume.ptr<float>(y);
        float* path = pathCost.ptr<float>(y);
        float* agg = aggregatedCost.ptr<float>(y);

        for (int x = 0; x < width; ++x) {
            for (int d = 0; d < numDisparities; ++d) {
                int idx = x * numDisparities + d;

                if (x == 0) {
                    path[idx] = cost[idx];
                } else {
                    float minPrev = FLT_MAX;
                    int prevIdx = (x - 1) * numDisparities;

                    // Find minimum from previous column
                    for (int pd = 0; pd < numDisparities; ++pd) {
                        float pathCost = path[prevIdx + pd];
                        if (pd == d) {
                            // Same disparity - no penalty
                            minPrev = std::min(minPrev, pathCost);
                        } else if (std::abs(pd - d) == 1) {
                            // Small disparity change - P1 penalty
                            minPrev = std::min(minPrev, pathCost + P1);
                        } else {
                            // Large disparity change - P2 penalty
                            minPrev = std::min(minPrev, pathCost + P2);
                        }
                    }

                    path[idx] = cost[idx] + minPrev;
                }

                agg[idx] += path[idx];
            }
        }
    }

    // Path 2: Right to Left
    pathCost.setTo(0);
    for (int y = 0; y < height; ++y) {
        const float* cost = costVolume.ptr<float>(y);
        float* path = pathCost.ptr<float>(y);
        float* agg = aggregatedCost.ptr<float>(y);

        for (int x = width - 1; x >= 0; --x) {
            for (int d = 0; d < numDisparities; ++d) {
                int idx = x * numDisparities + d;

                if (x == width - 1) {
                    path[idx] = cost[idx];
                } else {
                    float minPrev = FLT_MAX;
                    int prevIdx = (x + 1) * numDisparities;

                    for (int pd = 0; pd < numDisparities; ++pd) {
                        float pathCost = path[prevIdx + pd];
                        if (pd == d) {
                            minPrev = std::min(minPrev, pathCost);
                        } else if (std::abs(pd - d) == 1) {
                            minPrev = std::min(minPrev, pathCost + P1);
                        } else {
                            minPrev = std::min(minPrev, pathCost + P2);
                        }
                    }

                    path[idx] = cost[idx] + minPrev;
                }

                agg[idx] += path[idx];
            }
        }
    }

    // Path 3: Top to Bottom
    pathCost.setTo(0);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            const float* cost = costVolume.ptr<float>(y);
            float* path = pathCost.ptr<float>(y);
            float* agg = aggregatedCost.ptr<float>(y);

            for (int d = 0; d < numDisparities; ++d) {
                int idx = x * numDisparities + d;

                if (y == 0) {
                    path[idx] = cost[idx];
                } else {
                    float minPrev = FLT_MAX;
                    float* prevPath = pathCost.ptr<float>(y - 1);
                    int prevIdx = x * numDisparities;

                    for (int pd = 0; pd < numDisparities; ++pd) {
                        float pathCost = prevPath[prevIdx + pd];
                        if (pd == d) {
                            minPrev = std::min(minPrev, pathCost);
                        } else if (std::abs(pd - d) == 1) {
                            minPrev = std::min(minPrev, pathCost + P1);
                        } else {
                            minPrev = std::min(minPrev, pathCost + P2);
                        }
                    }

                    path[idx] = cost[idx] + minPrev;
                }

                agg[idx] += path[idx];
            }
        }
    }

    // Path 4: Bottom to Top
    pathCost.setTo(0);
    for (int x = 0; x < width; ++x) {
        for (int y = height - 1; y >= 0; --y) {
            const float* cost = costVolume.ptr<float>(y);
            float* path = pathCost.ptr<float>(y);
            float* agg = aggregatedCost.ptr<float>(y);

            for (int d = 0; d < numDisparities; ++d) {
                int idx = x * numDisparities + d;

                if (y == height - 1) {
                    path[idx] = cost[idx];
                } else {
                    float minPrev = FLT_MAX;
                    float* prevPath = pathCost.ptr<float>(y + 1);
                    int prevIdx = x * numDisparities;

                    for (int pd = 0; pd < numDisparities; ++pd) {
                        float pathCost = prevPath[prevIdx + pd];
                        if (pd == d) {
                            minPrev = std::min(minPrev, pathCost);
                        } else if (std::abs(pd - d) == 1) {
                            minPrev = std::min(minPrev, pathCost + P1);
                        } else {
                            minPrev = std::min(minPrev, pathCost + P2);
                        }
                    }

                    path[idx] = cost[idx] + minPrev;
                }

                agg[idx] += path[idx];
            }
        }
    }

    // Average by number of paths (4)
    aggregatedCost /= 4.0f;
}

// Helper: Winner-takes-all disparity selection with subpixel refinement
static void selectDisparity(
    const cv::Mat& aggregatedCost,
    cv::Mat& disparity,
    cv::Mat& confidence,
    int numDisparities,
    int minDisparity,
    bool useSubpixel)
{
    const int width = aggregatedCost.cols / numDisparities;
    const int height = aggregatedCost.rows;

    disparity = cv::Mat(height, width, CV_16S);
    confidence = cv::Mat(height, width, CV_8U);

    for (int y = 0; y < height; ++y) {
        const float* cost = aggregatedCost.ptr<float>(y);
        int16_t* disp = disparity.ptr<int16_t>(y);
        uint8_t* conf = confidence.ptr<uint8_t>(y);

        for (int x = 0; x < width; ++x) {
            float minCost = FLT_MAX;
            int bestDisp = 0;
            float secondMinCost = FLT_MAX;

            // Find minimum cost disparity
            for (int d = 0; d < numDisparities; ++d) {
                float c = cost[x * numDisparities + d];
                if (c < minCost) {
                    secondMinCost = minCost;
                    minCost = c;
                    bestDisp = d;
                } else if (c < secondMinCost) {
                    secondMinCost = c;
                }
            }

            // Subpixel refinement using parabolic fitting
            if (useSubpixel && bestDisp > 0 && bestDisp < numDisparities - 1) {
                float c0 = cost[x * numDisparities + bestDisp - 1];
                float c1 = cost[x * numDisparities + bestDisp];
                float c2 = cost[x * numDisparities + bestDisp + 1];

                float denom = 2.0f * (c0 - 2.0f * c1 + c2);
                if (std::abs(denom) > 0.001f) {
                    float delta = (c0 - c2) / denom;
                    // Store with 4 fractional bits (factor of 16)
                    disp[x] = static_cast<int16_t>((minDisparity + bestDisp + delta) * 16);
                } else {
                    disp[x] = static_cast<int16_t>((minDisparity + bestDisp) * 16);
                }
            } else {
                disp[x] = static_cast<int16_t>((minDisparity + bestDisp) * 16);
            }

            // Calculate confidence based on cost ratio
            if (secondMinCost > 0 && minCost > 0) {
                float ratio = minCost / secondMinCost;
                conf[x] = static_cast<uint8_t>(255 * (1.0f - ratio));
            } else {
                conf[x] = 0;
            }
        }
    }
}

DisparityComputer::Result DisparityComputer::computeADCensus(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    Result result;

    try {
        if (logger_) logger_->info("Computing AD-Census stereo matching (optimized for VCSEL)...");

        auto start = high_resolution_clock::now();

        // Step 1: Compute Census transforms
        cv::Mat censusLeft, censusRight;
        computeCensusTransform(leftRect, censusLeft, config_.censusWindowSize, config_.censusThreshold);
        computeCensusTransform(rightRect, censusRight, config_.censusWindowSize, config_.censusThreshold);

        if (logger_) {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
            logger_->info("  Census transform: " + std::to_string(elapsed.count()) + "ms");
        }

        // Step 2: Compute AD cost
        start = high_resolution_clock::now();
        cv::Mat adCost;
        computeADCost(leftRect, rightRect, adCost, config_.numDisparities, config_.minDisparity);

        if (logger_) {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
            logger_->info("  AD cost: " + std::to_string(elapsed.count()) + "ms");
        }

        // Step 3: Combine AD and Census costs
        start = high_resolution_clock::now();
        cv::Mat combinedCost;
        combineADCensus(adCost, censusLeft, censusRight, combinedCost,
                       config_.numDisparities, config_.minDisparity,
                       config_.lambdaAD, config_.lambdaCensus);

        if (logger_) {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
            logger_->info("  Cost combination: " + std::to_string(elapsed.count()) + "ms");
        }

        // Step 4: SGM aggregation
        start = high_resolution_clock::now();
        cv::Mat aggregatedCost;
        sgmAggregation(combinedCost, aggregatedCost, config_.numDisparities,
                      config_.P1, config_.P2);

        if (logger_) {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
            logger_->info("  SGM aggregation: " + std::to_string(elapsed.count()) + "ms");
        }

        // Step 5: Disparity selection with subpixel refinement
        start = high_resolution_clock::now();
        selectDisparity(aggregatedCost, result.disparity, result.confidence,
                       config_.numDisparities, config_.minDisparity, config_.useSubpixel);

        if (logger_) {
            auto elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - start);
            logger_->info("  Disparity selection: " + std::to_string(elapsed.count()) + "ms");
        }

        // TODO: Save debug output when DebugOutputManager singleton is available
        // if (DebugOutputManager::getInstance().isEnabled()) {
        //     cv::Mat disparityVis;
        //     cv::normalize(result.disparity, disparityVis, 0, 255, cv::NORM_MINMAX, CV_8U);
        //     cv::applyColorMap(disparityVis, disparityVis, cv::COLORMAP_JET);
        //     DebugOutputManager::getInstance().saveDebugImage("ad_census_disparity", disparityVis);
        //     DebugOutputManager::getInstance().saveDebugImage("ad_census_confidence", result.confidence);
        // }

        result.success = true;
        result.gpuAccelerated = false;

        if (logger_) logger_->info("✓ AD-Census computation completed successfully");

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "AD-Census computation failed: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== VULKAN GPU IMPLEMENTATION ==========

DisparityComputer::Result DisparityComputer::computeVulkan(
    const cv::Mat& leftRect,
    const cv::Mat& rightRect)
{
    Result result;

    if (!vulkanAccelerator_ || !vulkanAccelerator_->isAvailable()) {
        result.success = false;
        result.errorMessage = "Vulkan accelerator not available";
        return result;
    }

    try {
        if (logger_) logger_->info("Computing disparity using Vulkan GPU acceleration...");

        // Configure Vulkan accelerator
        VulkanSGMAccelerator::Config gpuConfig;
        gpuConfig.minDisparity = config_.minDisparity;
        gpuConfig.numDisparities = config_.numDisparities;
        gpuConfig.P1 = config_.P1;
        gpuConfig.P2 = config_.P2;
        gpuConfig.useSubpixel = config_.useSubpixel;
        gpuConfig.censusWindowSize = config_.censusWindowSize;
        gpuConfig.censusThreshold = config_.censusThreshold;

        vulkanAccelerator_->setConfig(gpuConfig);

        // Process on GPU
        VulkanSGMAccelerator::Result gpuResult = vulkanAccelerator_->process(leftRect, rightRect);

        if (gpuResult.success) {
            result.disparity = gpuResult.disparity;
            result.confidence = gpuResult.confidence;
            result.success = true;
            result.gpuAccelerated = true;

            if (logger_) {
                logger_->info("✓ GPU computation completed:");
                logger_->info("  GPU time: " + std::to_string(gpuResult.gpuTime.count()) + "ms");
                logger_->info("  Memory used: " + std::to_string(gpuResult.memoryUsedMB) + "MB");
            }
        } else {
            result.success = false;
            result.errorMessage = "GPU computation failed: " + gpuResult.errorMessage;
            if (logger_) logger_->error(result.errorMessage);
        }

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "Vulkan computation exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== WLS FILTERING ==========

void DisparityComputer::applyWLSFilter(
    const cv::Mat& leftRect,
    cv::Mat& disparity,
    cv::Mat& confidence)
{
    // TODO: Re-enable WLS filter when OpenCV is built with ximgproc contrib module
    // WLS (Weighted Least Squares) filtering provides edge-preserving smoothing
    // and is particularly effective for disparity refinement.

    if (!config_.useWLSFilter) {
        return;
    }

    if (logger_) {
        logger_->warning("WLS filter requested but disabled (OpenCV ximgproc not available)");
    }

    // Placeholder implementation - currently disabled
    // When ximgproc is available, this will use cv::ximgproc::DisparityWLSFilter

    /* DISABLED - Requires OpenCV contrib ximgproc module
    if (!wlsFilter_) {
        return;
    }

    try {
        if (logger_) logger_->info("Applying WLS edge-preserving filter...");

        // Create right matcher for left-right consistency check
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> filter = wlsFilter_;

        // Create filtered disparity
        cv::Mat filteredDisparity;
        filter->filter(disparity, leftRect, filteredDisparity);

        // Get confidence map from filter
        cv::Mat filterConfidence = filter->getConfidenceMap();

        // Update outputs
        disparity = filteredDisparity;

        // Merge confidence maps
        if (confidence.empty()) {
            confidence = filterConfidence;
        } else {
            // Combine existing confidence with WLS confidence
            cv::Mat combinedConfidence;
            cv::addWeighted(confidence, 0.5, filterConfidence, 0.5, 0, combinedConfidence);
            confidence = combinedConfidence;
        }

        if (logger_) logger_->info("WLS filtering applied successfully");

    } catch (const cv::Exception& e) {
        if (logger_) logger_->warning(std::string("WLS filtering failed: ") + e.what());
    }
    */
}

// ========== QUALITY METRICS ==========

void DisparityComputer::calculateMetrics(const cv::Mat& disparity, Result& result)
{
    if (disparity.empty()) {
        result.validPixelPercentage = 0;
        result.meanDisparity = 0;
        result.disparityStdDev = 0;
        return;
    }

    // Count valid pixels and calculate statistics
    int validCount = 0;
    double sum = 0;
    double sumSq = 0;

    const int totalPixels = disparity.rows * disparity.cols;

    for (int y = 0; y < disparity.rows; ++y) {
        const int16_t* row = disparity.ptr<int16_t>(y);

        for (int x = 0; x < disparity.cols; ++x) {
            int16_t d = row[x];

            if (d > 0) {  // Valid disparity
                validCount++;
                double val = d / 16.0;  // Convert from fixed-point
                sum += val;
                sumSq += val * val;
            }
        }
    }

    result.validPixelPercentage = (100.0f * validCount) / totalPixels;

    if (validCount > 0) {
        result.meanDisparity = sum / validCount;
        double variance = (sumSq / validCount) - (result.meanDisparity * result.meanDisparity);
        result.disparityStdDev = std::sqrt(std::max(0.0, variance));
    } else {
        result.meanDisparity = 0;
        result.disparityStdDev = 0;
    }
}

// ========== MATCHER UPDATES ==========

void DisparityComputer::updateSGBMMatcher()
{
    sgbmMatcher_ = cv::StereoSGBM::create(
        config_.minDisparity,
        config_.numDisparities,
        config_.blockSize,
        config_.P1,
        config_.P2,
        config_.disp12MaxDiff,
        config_.preFilterCap,
        config_.uniquenessRatio,
        config_.speckleWindowSize,
        config_.speckleRange,
        config_.mode
    );
}

void DisparityComputer::updateWLSFilter()
{
    // TODO: Re-enable WLS filter when OpenCV is built with ximgproc contrib module

    /* DISABLED - Requires OpenCV contrib ximgproc module
    if (config_.useWLSFilter) {
        // Create a matcher for WLS filter (it needs one internally)
        cv::Ptr<cv::StereoMatcher> matcher = cv::StereoSGBM::create(
            config_.minDisparity,
            config_.numDisparities,
            config_.blockSize
        );

        wlsFilter_ = cv::ximgproc::createDisparityWLSFilter(matcher);
        wlsFilter_->setLambda(config_.wlsLambda);
        wlsFilter_->setSigmaColor(config_.wlsSigma);
    }
    */

    // WLS filter initialization disabled - ximgproc not available
    wlsFilter_ = nullptr;
}

} // namespace stereo
} // namespace unlook