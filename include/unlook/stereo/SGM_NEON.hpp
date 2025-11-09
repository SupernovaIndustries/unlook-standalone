/**
 * @file SGM_NEON.hpp
 * @brief ARM NEON-optimized Semi-Global Matching
 *
 * Target: 46+ FPS on VGA (1280x720) on Raspberry Pi CM5
 * Based on ReS2tAC approach with SIMD vectorization
 */

#pragma once

#include <opencv2/core.hpp>
#include <functional>
#include <string>

namespace unlook {
namespace stereo {

/**
 * @brief Compute Census transform with NEON optimization
 * @param image Input grayscale image
 * @param census Output 64-bit census codes per pixel
 */
void censusTransformNEON(const cv::Mat& image, cv::Mat& census);

/**
 * @brief Compute Census matching cost with NEON Hamming distance
 * @param censusLeft Left census image
 * @param censusRight Right census image
 * @param costVolume Output cost volume (height × width*disparities)
 * @param numDisparities Number of disparity levels
 */
void censusCostNEON(const cv::Mat& censusLeft, const cv::Mat& censusRight,
                    cv::Mat& costVolume, int numDisparities);

/**
 * @brief Complete 4-path SGM aggregation with NEON acceleration
 * @param costVolume Input cost volume (height × width*disparities)
 * @param aggregatedCost Output aggregated cost
 * @param numDisparities Number of disparity levels
 * @param P1 Small disparity change penalty
 * @param P2 Large disparity change penalty
 * @param progress_callback Optional progress reporting callback
 */
void sgmAggregation4PathNEON(const cv::Mat& costVolume,
                             cv::Mat& aggregatedCost,
                             int numDisparities,
                             float P1,
                             float P2,
                             std::function<void(float, const std::string&)> progress_callback = nullptr);

} // namespace stereo
} // namespace unlook
