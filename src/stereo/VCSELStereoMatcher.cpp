/**
 * @file VCSELStereoMatcher.cpp
 * @brief VCSEL-optimized stereo matcher using AD-Census algorithm
 *
 * Implements complete AD-Census fusion algorithm with ARM NEON optimization
 * for real-time HD 1280x720 processing at ~10 FPS on Raspberry Pi CM4/CM5.
 */

#include "unlook/stereo/VCSELStereoMatcher.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/photo.hpp>
#include <chrono>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <limits>
#include <omp.h>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

// Include NEON implementations
namespace unlook {
namespace stereo {
namespace neon {
    // Forward declarations of NEON functions
    void censusTransform9x9NEON(const cv::Mat& image, cv::Mat& census, int threshold);
    void censusTransform9x9CPU(const cv::Mat& image, cv::Mat& census, int threshold);
    void computeHammingCostVectorizedNEON(const cv::Mat& censusLeft, const cv::Mat& censusRight,
                                          cv::Mat& costVolume, int minDisparity, int numDisparities);
    void computeHammingCostCPU(const cv::Mat& censusLeft, const cv::Mat& censusRight,
                               cv::Mat& costVolume, int minDisparity, int numDisparities);
    void computeADCostVectorizedNEON(const cv::Mat& left, const cv::Mat& right,
                                     cv::Mat& costVolume, int minDisparity, int numDisparities);
    void computeADCostCPU(const cv::Mat& left, const cv::Mat& right,
                         cv::Mat& costVolume, int minDisparity, int numDisparities);
}
}
}

namespace unlook {
namespace stereo {

using namespace std::chrono;

VCSELStereoMatcher::VCSELStereoMatcher() {
    // Check NEON support
    useNEON_ = checkNEONSupport();

    core::Logger::getInstance().log(
        core::LogLevel::INFO,
        "VCSELStereoMatcher initialized with " +
        std::string(useNEON_ ? "NEON optimization" : "CPU fallback")
    );

    // Set default parameters optimized for VCSEL
    params_.minDisparity = adCensusParams_.minDisparity;
    params_.numDisparities = adCensusParams_.numDisparities;
    params_.uniquenessRatio = adCensusParams_.uniquenessRatio;

    // Try to initialize Vulkan (experimental, non-blocking)
    vulkanAvailable_ = tryInitVulkan();
    if (vulkanAvailable_) {
        core::Logger::getInstance().log(
            core::LogLevel::INFO,
            "Vulkan compute available for SGM acceleration"
        );
    }

    // Set OpenMP threads for parallel processing
    omp_set_num_threads(4);  // Optimize for Raspberry Pi quad-core
}

VCSELStereoMatcher::~VCSELStereoMatcher() {
    // Vulkan accelerator cleans up automatically via unique_ptr
}

bool VCSELStereoMatcher::computeDisparity(const cv::Mat& leftRectified,
                                          const cv::Mat& rightRectified,
                                          cv::Mat& disparity) {
    auto startTotal = high_resolution_clock::now();

    // Reset stats
    ProcessingStats stats;
    stats.totalPixels = adCensusParams_.processingSize.area();

    try {
        // Validate input
        if (leftRectified.empty() || rightRectified.empty()) {
            core::Logger::getInstance().log(
                core::LogLevel::ERROR,
                "Empty input images"
            );
            return false;
        }

        if (leftRectified.size() != rightRectified.size()) {
            core::Logger::getInstance().log(
                core::LogLevel::ERROR,
                "Input images must have the same size"
            );
            return false;
        }

        // Convert to grayscale if needed
        cv::Mat leftGray, rightGray;
        if (leftRectified.channels() == 3) {
            cv::cvtColor(leftRectified, leftGray, cv::COLOR_BGR2GRAY);
        } else {
            leftGray = leftRectified;
        }
        if (rightRectified.channels() == 3) {
            cv::cvtColor(rightRectified, rightGray, cv::COLOR_BGR2GRAY);
        } else {
            rightGray = rightRectified;
        }

        // Step 1: Downsample to HD resolution if needed
        auto startDownsample = high_resolution_clock::now();

        if (leftGray.size() != adCensusParams_.processingSize) {
            downsampleImage(leftGray, leftDownsampled_);
            downsampleImage(rightGray, rightDownsampled_);
        } else {
            leftDownsampled_ = leftGray;
            rightDownsampled_ = rightGray;
        }

        auto endDownsample = high_resolution_clock::now();
        stats.downsampleTimeMs = duration_cast<microseconds>(endDownsample - startDownsample).count() / 1000.0;

        // Step 2: Census Transform
        auto startCensus = high_resolution_clock::now();

        censusTransform(leftDownsampled_, censusLeft_);
        censusTransform(rightDownsampled_, censusRight_);

        auto endCensus = high_resolution_clock::now();
        stats.censusTimeMs = duration_cast<microseconds>(endCensus - startCensus).count() / 1000.0;

        // Step 3: Hamming Distance Cost
        auto startHamming = high_resolution_clock::now();

        computeHammingCost(censusLeft_, censusRight_, censusCostVolume_);

        auto endHamming = high_resolution_clock::now();
        stats.hammingTimeMs = duration_cast<microseconds>(endHamming - startHamming).count() / 1000.0;

        // Step 4: Absolute Difference Cost
        auto startAD = high_resolution_clock::now();

        computeADCost(leftDownsampled_, rightDownsampled_, adCostVolume_);

        auto endAD = high_resolution_clock::now();
        stats.adCostTimeMs = duration_cast<microseconds>(endAD - startAD).count() / 1000.0;

        // Step 5: Cost Fusion
        auto startFusion = high_resolution_clock::now();

        fuseCosts(adCostVolume_, censusCostVolume_, fusedCostVolume_);

        auto endFusion = high_resolution_clock::now();
        stats.fusionTimeMs = duration_cast<microseconds>(endFusion - startFusion).count() / 1000.0;

        // Step 6: SGM Aggregation
        auto startSGM = high_resolution_clock::now();

        sgmAggregation(fusedCostVolume_, aggregatedCostVolume_);

        auto endSGM = high_resolution_clock::now();
        stats.sgmTimeMs = duration_cast<microseconds>(endSGM - startSGM).count() / 1000.0;

        // Step 7: Winner-Take-All Disparity Selection
        winnerTakeAll(aggregatedCostVolume_, disparity);

        // Step 8: Subpixel Refinement
        if (adCensusParams_.useSubpixel) {
            subpixelRefinement(aggregatedCostVolume_, disparity);
        }

        // Step 9: Post-processing
        auto startPost = high_resolution_clock::now();

        postProcessDisparity(disparity);

        auto endPost = high_resolution_clock::now();
        stats.postProcessingTimeMs = duration_cast<microseconds>(endPost - startPost).count() / 1000.0;

        // Calculate total time and valid pixels
        auto endTotal = high_resolution_clock::now();
        stats.totalTimeMs = duration_cast<microseconds>(endTotal - startTotal).count() / 1000.0;

        // Count valid pixels
        stats.validPixels = cv::countNonZero(disparity > 0);

        // Update stats
        {
            std::lock_guard<std::mutex> lock(statsMutex_);
            lastStats_ = stats;
        }

        // Log performance
        core::Logger::getInstance().log(
            core::LogLevel::DEBUG,
            "VCSELStereoMatcher processing complete: " + stats.toString()
        );

        return true;

    } catch (const std::exception& e) {
        core::Logger::getInstance().log(
            core::LogLevel::ERROR,
            "Exception in computeDisparity: " + std::string(e.what())
        );
        return false;
    }
}

bool VCSELStereoMatcher::setParameters(const StereoMatchingParams& params) {
    params_ = params;

    // Update AD-Census specific parameters
    adCensusParams_.minDisparity = params.minDisparity;
    adCensusParams_.numDisparities = params.numDisparities;
    adCensusParams_.uniquenessRatio = params.uniquenessRatio;
    adCensusParams_.P1 = params.P1;
    adCensusParams_.P2 = params.P2;

    return true;
}

StereoMatchingParams VCSELStereoMatcher::getParameters() const {
    return params_;
}

bool VCSELStereoMatcher::patternIsolation(const cv::Mat& vcselImage,
                                          const cv::Mat& ambientImage,
                                          cv::Mat& isolated) {
    if (vcselImage.empty()) {
        return false;
    }

    if (ambientImage.empty()) {
        // No ambient image, use high-pass filter to extract VCSEL dots
        cv::Mat blurred;
        cv::GaussianBlur(vcselImage, blurred, cv::Size(5, 5), 1.0);
        cv::subtract(vcselImage, blurred, isolated);

        // Threshold to extract bright dots
        cv::threshold(isolated, isolated, 10, 255, cv::THRESH_BINARY);
    } else {
        // Subtract ambient illumination
        cv::subtract(vcselImage, ambientImage, isolated);

        // Enhance dots
        cv::threshold(isolated, isolated, 5, 255, cv::THRESH_BINARY);
    }

    // Morphological operations to clean up noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(isolated, isolated, cv::MORPH_OPEN, kernel);

    return true;
}

void VCSELStereoMatcher::downsampleImage(const cv::Mat& input, cv::Mat& output) {
    // Use INTER_AREA for best quality downsampling
    cv::resize(input, output, adCensusParams_.processingSize, 0, 0, cv::INTER_AREA);
}

void VCSELStereoMatcher::censusTransform(const cv::Mat& image, cv::Mat& census) {
    if (useNEON_) {
        censusTransformNEON(image, census);
    } else {
        censusTransformCPU(image, census);
    }
}

void VCSELStereoMatcher::censusTransformCPU(const cv::Mat& image, cv::Mat& census) {
    neon::censusTransform9x9CPU(image, census, adCensusParams_.censusThreshold);
}

void VCSELStereoMatcher::censusTransformNEON(const cv::Mat& image, cv::Mat& census) {
#ifdef __ARM_NEON
    neon::censusTransform9x9NEON(image, census, adCensusParams_.censusThreshold);
#else
    censusTransformCPU(image, census);
#endif
}

void VCSELStereoMatcher::computeHammingCost(const cv::Mat& censusLeft,
                                            const cv::Mat& censusRight,
                                            cv::Mat& costVolume) {
    if (useNEON_) {
        computeHammingCostNEON(censusLeft, censusRight, costVolume);
    } else {
        neon::computeHammingCostCPU(censusLeft, censusRight, costVolume,
                                    adCensusParams_.minDisparity,
                                    adCensusParams_.numDisparities);
    }
}

void VCSELStereoMatcher::computeHammingCostNEON(const cv::Mat& censusLeft,
                                                const cv::Mat& censusRight,
                                                cv::Mat& costVolume) {
#ifdef __ARM_NEON
    neon::computeHammingCostVectorizedNEON(censusLeft, censusRight, costVolume,
                                           adCensusParams_.minDisparity,
                                           adCensusParams_.numDisparities);
#else
    neon::computeHammingCostCPU(censusLeft, censusRight, costVolume,
                                adCensusParams_.minDisparity,
                                adCensusParams_.numDisparities);
#endif
}

void VCSELStereoMatcher::computeADCost(const cv::Mat& left,
                                       const cv::Mat& right,
                                       cv::Mat& costVolume) {
    if (useNEON_) {
        computeADCostNEON(left, right, costVolume);
    } else {
        neon::computeADCostCPU(left, right, costVolume,
                              adCensusParams_.minDisparity,
                              adCensusParams_.numDisparities);
    }
}

void VCSELStereoMatcher::computeADCostNEON(const cv::Mat& left,
                                           const cv::Mat& right,
                                           cv::Mat& costVolume) {
#ifdef __ARM_NEON
    neon::computeADCostVectorizedNEON(left, right, costVolume,
                                      adCensusParams_.minDisparity,
                                      adCensusParams_.numDisparities);
#else
    neon::computeADCostCPU(left, right, costVolume,
                          adCensusParams_.minDisparity,
                          adCensusParams_.numDisparities);
#endif
}

void VCSELStereoMatcher::fuseCosts(const cv::Mat& adCost,
                                   const cv::Mat& censusCost,
                                   cv::Mat& fusedCost) {
    // Create fused cost volume
    fusedCost.create(adCost.dims, adCost.size, CV_32F);

    const float lambdaAD = adCensusParams_.lambdaAD;
    const float lambdaCensus = adCensusParams_.lambdaCensus;

    // Normalize costs and fuse
    const int height = adCost.size[0];
    const int width = adCost.size[1];
    const int disparities = adCost.size[2];

    #pragma omp parallel for collapse(2)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const float* adPtr = adCost.ptr<float>(y, x);
            const float* censusPtr = censusCost.ptr<float>(y, x);
            float* fusedPtr = fusedCost.ptr<float>(y, x);

            for (int d = 0; d < disparities; d++) {
                // Normalize AD cost (0-255 -> 0-1)
                float adNorm = adPtr[d] / 255.0f;

                // Normalize Census cost (0-24 -> 0-1) [5x5 Census = 24-bit]
                float censusNorm = censusPtr[d] / 24.0f;

                // Fused cost
                fusedPtr[d] = lambdaAD * adNorm + lambdaCensus * censusNorm;
            }
        }
    }
}

void VCSELStereoMatcher::sgmAggregation(const cv::Mat& costVolume,
                                        cv::Mat& aggregatedCost) {
    const int height = costVolume.size[0];
    const int width = costVolume.size[1];
    const int disparities = costVolume.size[2];

    // Try GPU acceleration first
    if (vulkanAvailable_ && vulkanAccelerator_) {
        core::Logger::getInstance().log(
            core::LogLevel::DEBUG,
            "Attempting GPU-accelerated SGM aggregation..."
        );

        if (vulkanAccelerator_->aggregateSGM(costVolume, aggregatedCost,
                                             adCensusParams_.numDisparities,
                                             static_cast<float>(adCensusParams_.P1),
                                             static_cast<float>(adCensusParams_.P2))) {
            // GPU success - log performance
            auto gpuStats = vulkanAccelerator_->getLastStats();
            core::Logger::getInstance().log(
                core::LogLevel::INFO,
                "GPU SGM: upload=" + std::to_string(gpuStats.uploadTimeMs) +
                "ms, compute=" + std::to_string(gpuStats.computeTimeMs) +
                "ms, download=" + std::to_string(gpuStats.downloadTimeMs) + "ms"
            );
            return;
        } else {
            core::Logger::getInstance().log(
                core::LogLevel::WARNING,
                "GPU SGM failed, falling back to CPU"
            );
        }
    }

    // CPU fallback (original implementation)
    core::Logger::getInstance().log(
        core::LogLevel::DEBUG,
        "Using CPU SGM aggregation"
    );

    // Create aggregated cost volume
    aggregatedCost.create(costVolume.dims, costVolume.size, CV_32F);
    aggregatedCost.setTo(0);

    const int P1 = adCensusParams_.P1;
    const int P2 = adCensusParams_.P2;

    // 4-path SGM aggregation (L-R, R-L, T-B, B-T)
    cv::Mat pathCost(disparities, 1, CV_32F);

    // Path 1: Left to Right
    #pragma omp parallel for
    for (int y = 0; y < height; y++) {
        std::vector<float> prevCost(disparities, 0);

        for (int x = 0; x < width; x++) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            float* aggPtr = aggregatedCost.ptr<float>(y, x);

            float minPrevCost = *std::min_element(prevCost.begin(), prevCost.end());

            for (int d = 0; d < disparities; d++) {
                float cost = costPtr[d];

                // SGM smoothness term
                float smoothCost = prevCost[d];
                if (d > 0) {
                    smoothCost = std::min(smoothCost, prevCost[d-1] + P1);
                }
                if (d < disparities - 1) {
                    smoothCost = std::min(smoothCost, prevCost[d+1] + P1);
                }
                smoothCost = std::min(smoothCost, minPrevCost + P2);

                prevCost[d] = cost + smoothCost - minPrevCost;
                aggPtr[d] += prevCost[d];
            }
        }
    }

    // Path 2: Right to Left
    #pragma omp parallel for
    for (int y = 0; y < height; y++) {
        std::vector<float> prevCost(disparities, 0);

        for (int x = width - 1; x >= 0; x--) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            float* aggPtr = aggregatedCost.ptr<float>(y, x);

            float minPrevCost = *std::min_element(prevCost.begin(), prevCost.end());

            for (int d = 0; d < disparities; d++) {
                float cost = costPtr[d];

                float smoothCost = prevCost[d];
                if (d > 0) {
                    smoothCost = std::min(smoothCost, prevCost[d-1] + P1);
                }
                if (d < disparities - 1) {
                    smoothCost = std::min(smoothCost, prevCost[d+1] + P1);
                }
                smoothCost = std::min(smoothCost, minPrevCost + P2);

                prevCost[d] = cost + smoothCost - minPrevCost;
                aggPtr[d] += prevCost[d];
            }
        }
    }

    // Path 3: Top to Bottom
    #pragma omp parallel for
    for (int x = 0; x < width; x++) {
        std::vector<float> prevCost(disparities, 0);

        for (int y = 0; y < height; y++) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            float* aggPtr = aggregatedCost.ptr<float>(y, x);

            float minPrevCost = *std::min_element(prevCost.begin(), prevCost.end());

            for (int d = 0; d < disparities; d++) {
                float cost = costPtr[d];

                float smoothCost = prevCost[d];
                if (d > 0) {
                    smoothCost = std::min(smoothCost, prevCost[d-1] + P1);
                }
                if (d < disparities - 1) {
                    smoothCost = std::min(smoothCost, prevCost[d+1] + P1);
                }
                smoothCost = std::min(smoothCost, minPrevCost + P2);

                prevCost[d] = cost + smoothCost - minPrevCost;
                aggPtr[d] += prevCost[d];
            }
        }
    }

    // Path 4: Bottom to Top
    #pragma omp parallel for
    for (int x = 0; x < width; x++) {
        std::vector<float> prevCost(disparities, 0);

        for (int y = height - 1; y >= 0; y--) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            float* aggPtr = aggregatedCost.ptr<float>(y, x);

            float minPrevCost = *std::min_element(prevCost.begin(), prevCost.end());

            for (int d = 0; d < disparities; d++) {
                float cost = costPtr[d];

                float smoothCost = prevCost[d];
                if (d > 0) {
                    smoothCost = std::min(smoothCost, prevCost[d-1] + P1);
                }
                if (d < disparities - 1) {
                    smoothCost = std::min(smoothCost, prevCost[d+1] + P1);
                }
                smoothCost = std::min(smoothCost, minPrevCost + P2);

                prevCost[d] = cost + smoothCost - minPrevCost;
                aggPtr[d] += prevCost[d];
            }
        }
    }
}

void VCSELStereoMatcher::winnerTakeAll(const cv::Mat& aggregatedCost,
                                       cv::Mat& disparity) {
    const int height = aggregatedCost.size[0];
    const int width = aggregatedCost.size[1];
    const int disparities = aggregatedCost.size[2];

    disparity.create(height, width, CV_32F);

    #pragma omp parallel for collapse(2)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const float* costPtr = aggregatedCost.ptr<float>(y, x);

            // Find minimum cost disparity
            int bestDisp = 0;
            float minCost = costPtr[0];

            for (int d = 1; d < disparities; d++) {
                if (costPtr[d] < minCost) {
                    minCost = costPtr[d];
                    bestDisp = d;
                }
            }

            // Apply uniqueness check
            float secondMinCost = std::numeric_limits<float>::max();

            for (int d = 0; d < disparities; d++) {
                if (d != bestDisp && costPtr[d] < secondMinCost) {
                    secondMinCost = costPtr[d];
                }
            }

            float uniqueness = (secondMinCost - minCost) / minCost * 100.0f;

            if (uniqueness >= adCensusParams_.uniquenessRatio) {
                disparity.at<float>(y, x) = static_cast<float>(bestDisp + adCensusParams_.minDisparity);
            } else {
                disparity.at<float>(y, x) = 0;  // Invalid
            }
        }
    }
}

void VCSELStereoMatcher::subpixelRefinement(const cv::Mat& aggregatedCost,
                                            cv::Mat& disparity) {
    const int height = aggregatedCost.size[0];
    const int width = aggregatedCost.size[1];
    const int disparities = aggregatedCost.size[2];

    #pragma omp parallel for collapse(2)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float disp = disparity.at<float>(y, x);

            if (disp > 0) {
                int d = static_cast<int>(disp - adCensusParams_.minDisparity);

                if (d > 0 && d < disparities - 1) {
                    const float* costPtr = aggregatedCost.ptr<float>(y, x);

                    // Parabolic fitting
                    float c0 = costPtr[d - 1];
                    float c1 = costPtr[d];
                    float c2 = costPtr[d + 1];

                    float denom = 2.0f * (c2 + c0 - 2.0f * c1);
                    if (std::abs(denom) > 1e-6) {
                        float delta = (c0 - c2) / denom;

                        // Apply subpixel offset (delta already in correct range)
                        disparity.at<float>(y, x) = disp + delta;
                    }
                }
            }
        }
    }
}

void VCSELStereoMatcher::postProcessDisparity(cv::Mat& disparity) {
    // FAST speckle filter using OpenCV's optimized filterSpeckles
    // (previous connectedComponents implementation was TOO SLOW - 68 seconds!)
    const int maxSpeckleSize = 50;   // Slightly larger for better noise removal
    const double maxDiff = 2.0;      // Disparity difference threshold

    // Convert to 16-bit for filterSpeckles (requires CV_16S or CV_32F)
    cv::Mat disp16;
    disparity.convertTo(disp16, CV_16S);

    // Fast speckle filtering (OpenCV optimized)
    cv::filterSpeckles(disp16, 0, maxSpeckleSize, maxDiff);

    // Convert back to float
    disp16.convertTo(disparity, CV_32F);

    // Light median filter for smoothing (only 3x3 - fast!)
    cv::Mat temp;
    cv::medianBlur(disparity, temp, 3);

    // Only update non-zero values to preserve edges
    cv::Mat mask = disparity > 0;
    temp.copyTo(disparity, mask);

    // NO inpaint - was too slow (removed to get from 68s to <1s)
}

bool VCSELStereoMatcher::checkNEONSupport() const {
#ifdef __ARM_NEON
    return true;
#else
    return false;
#endif
}

int VCSELStereoMatcher::hammingDistance80(const uint64_t* desc1, const uint64_t* desc2) const {
    // XOR and count bits
    uint64_t xor_low = desc1[0] ^ desc2[0];
    uint64_t xor_high = desc1[1] ^ desc2[1];

    // Use builtin popcount
    return __builtin_popcountll(xor_low) + __builtin_popcountll(xor_high & 0xFFFF);
}

bool VCSELStereoMatcher::tryInitVulkan() {
    // TEMPORARY: Vulkan GPU disabled due to crash in compute pipeline creation
    // Will be re-enabled after proper debugging
    core::Logger::getInstance().log(
        core::LogLevel::INFO,
        "Vulkan GPU temporarily disabled - using optimized CPU SGM with NEON"
    );
    return false;

    // Original Vulkan code commented out for now
    /*
#ifdef HAS_VULKAN
    try {
        core::Logger::getInstance().log(
            core::LogLevel::INFO,
            "Attempting to initialize Vulkan GPU acceleration..."
        );

        vulkanAccelerator_ = std::make_unique<VulkanSGMAccelerator>();

        if (vulkanAccelerator_->initialize()) {
            core::Logger::getInstance().log(
                core::LogLevel::INFO,
                "Vulkan GPU acceleration successfully initialized for SGM"
            );
            return true;
        } else {
            core::Logger::getInstance().log(
                core::LogLevel::WARNING,
                "Vulkan initialization failed, falling back to CPU"
            );
            vulkanAccelerator_.reset();
            return false;
        }
    } catch (const std::exception& e) {
        core::Logger::getInstance().log(
            core::LogLevel::WARNING,
            "Vulkan initialization exception: " + std::string(e.what())
        );
        vulkanAccelerator_.reset();
        return false;
    }
#else
    core::Logger::getInstance().log(
        core::LogLevel::INFO,
        "Vulkan not available - using CPU SGM"
    );
    return false;
#endif
    */
}

std::string VCSELStereoMatcher::ProcessingStats::toString() const {
    std::stringstream ss;
    ss << "Total: " << totalTimeMs << "ms ("
       << (1000.0 / totalTimeMs) << " FPS), "
       << "Valid: " << (100.0 * validPixels / totalPixels) << "% | "
       << "Downsample: " << downsampleTimeMs << "ms, "
       << "Census: " << censusTimeMs << "ms, "
       << "Hamming: " << hammingTimeMs << "ms, "
       << "AD: " << adCostTimeMs << "ms, "
       << "Fusion: " << fusionTimeMs << "ms, "
       << "SGM: " << sgmTimeMs << "ms, "
       << "Post: " << postProcessingTimeMs << "ms";
    return ss.str();
}

} // namespace stereo
} // namespace unlook