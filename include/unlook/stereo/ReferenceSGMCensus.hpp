/**
 * @file reference_sgm_census.hpp
 * @brief Reference SGM-Census implementation for Unlook 3D Scanner
 *
 * CRITICAL CALIBRATION COMPONENT
 * ================================
 * This is a REFERENCE IMPLEMENTATION of Semi-Global Matching with Census Transform,
 * based on verified open-source code (epiception/SGM-Census).
 *
 * Purpose:
 * - Provide ACCURATE disparity maps for stereo calibration validation
 * - Serve as GROUND TRUTH for comparing other stereo matching algorithms
 * - Enable high-quality 3D reconstruction for VCSEL-based structured light
 *
 * Implementation Notes:
 * - Uses 8-path Semi-Global Matching (Hirschmueller 2008)
 * - Census Transform for robustness to illumination changes (critical for VCSEL)
 * - Hamming distance for fast binary descriptor matching
 * - Proper SGM cost aggregation with P1/P2 penalties
 *
 * Integration Path (when validated):
 * 1. Move to src/stereo/ReferenceSGMCensus.{hpp,cpp}
 * 2. Add to CalibrationManager for calibration validation
 * 3. Use as fallback/validation for production stereo matching
 * 4. Optimize with ARM NEON for real-time performance
 *
 * @author Alessandro (Unlook Project)
 * @date 2025-11-12
 * @version 1.0 - Reference Implementation
 */

#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>

namespace unlook {
namespace stereo {
namespace reference {

/**
 * @brief Reference implementation of SGM-Census stereo matching
 *
 * This class provides a REFERENCE implementation that prioritizes correctness
 * over performance. It serves as ground truth for calibration validation.
 *
 * Algorithm Pipeline:
 * 1. Census Transform: Convert images to binary descriptors (robust to lighting)
 * 2. Matching Cost: Hamming distance between Census descriptors
 * 3. Cost Aggregation: 8-path Semi-Global Matching with P1/P2 penalties
 * 4. Disparity Selection: Winner-Takes-All (WTA) on aggregated costs
 *
 * Key Features:
 * - 8-directional path aggregation (L→R, R→L, T→B, B→T, 4 diagonals)
 * - Proper SGM formula: C + min(prev[d], prev[d±1]+P1, min_all+P2) - prevMin
 * - Census window size configurable (5x5 recommended for VCSEL dot patterns)
 * - Penalties P1, P2 control smoothness vs discontinuity preservation
 */
class ReferenceSGMCensus {
public:
    /**
     * @brief Configuration parameters for SGM-Census
     */
    struct Config {
        // === CENSUS TRANSFORM ===
        int censusWindowSize = 5;  ///< Census window (5x5 recommended for VCSEL)

        // === DISPARITY SEARCH RANGE ===
        int minDisparity = 0;      ///< Minimum disparity (usually 0)
        int numDisparities = 384;  ///< Number of disparities to search (was 256 - too small!)

        // === SGM PENALTIES ===
        int P1 = 20;    ///< Small penalty for disparity change of ±1 pixel (was 300 - too large!)
        int P2 = 40;    ///< Large penalty for disparity change > 1 pixel (was 1200 - WAY too large!)

        // === SGM PATHS ===
        bool use8Paths = true;  ///< Use all 8 paths (recommended) vs 4 paths

        // === OUTPUT ===
        bool verbose = true;    ///< Print timing and statistics
    };

    /**
     * @brief Result from SGM-Census computation
     */
    struct Result {
        cv::Mat disparity;           ///< Disparity map (CV_16SC1, scaled by 16)
        double censusTimeMs = 0.0;   ///< Census transform time
        double costTimeMs = 0.0;     ///< Matching cost computation time
        double sgmTimeMs = 0.0;      ///< SGM aggregation time
        double wtaTimeMs = 0.0;      ///< Winner-Takes-All selection time
        double totalTimeMs = 0.0;    ///< Total processing time
        int validPixels = 0;         ///< Number of valid disparity pixels
        double validPercent = 0.0;   ///< Percentage of valid pixels
        bool success = false;        ///< Whether computation succeeded
        std::string errorMessage;    ///< Error message if failed
    };

    /**
     * @brief Constructor
     * @param config Configuration parameters
     */
    explicit ReferenceSGMCensus(const Config& config);

    /**
     * @brief Compute disparity map using SGM-Census
     * @param leftGray Left rectified image (CV_8UC1)
     * @param rightGray Right rectified image (CV_8UC1)
     * @return Result structure with disparity map and timing info
     */
    Result compute(const cv::Mat& leftGray, const cv::Mat& rightGray);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Update configuration
     */
    void setConfig(const Config& config) { config_ = config; }

private:
    Config config_;

    // === INTERNAL PROCESSING STAGES ===

    /**
     * @brief Compute Census Transform descriptor for an image
     *
     * Census Transform creates a binary descriptor by comparing each pixel
     * in a window with the center pixel. This is robust to lighting changes.
     *
     * Formula: For each neighbor pixel, bit = 1 if (pixel < center), 0 otherwise
     *
     * @param image Input grayscale image
     * @param census Output census descriptors (CV_64SC1, stores 64-bit descriptors)
     */
    void computeCensusTransform(const cv::Mat& image, cv::Mat& census);

    /**
     * @brief Compute Hamming distance matching cost
     *
     * For each pixel and disparity, compute the Hamming distance between
     * left and right Census descriptors.
     *
     * @param censusLeft Left Census descriptors
     * @param censusRight Right Census descriptors
     * @param costVolume Output cost volume [height x width x D]
     */
    void computeMatchingCost(
        const cv::Mat& censusLeft,
        const cv::Mat& censusRight,
        std::vector<uint8_t>& costVolume);

    /**
     * @brief Semi-Global Matching cost aggregation
     *
     * Implements the full SGM algorithm with 8 directional paths.
     * Each path propagates cost information and applies smoothness penalties.
     *
     * SGM Formula (for each path):
     * L_r(p, d) = C(p, d) + min(
     *     L_r(p-r, d),              // Same disparity (no penalty)
     *     L_r(p-r, d-1) + P1,       // Small disparity change
     *     L_r(p-r, d+1) + P1,
     *     min_k(L_r(p-r, k)) + P2   // Large disparity change
     * ) - min_k(L_r(p-r, k))        // Normalization
     *
     * Final cost: S(p, d) = Σ_r L_r(p, d)
     *
     * @param costVolume Input matching cost
     * @param aggregatedCost Output aggregated cost [height x width x D]
     * @param width Image width
     * @param height Image height
     */
    void aggregateCostsSGM(
        const std::vector<uint8_t>& costVolume,
        std::vector<uint32_t>& aggregatedCost,
        int width, int height);

    /**
     * @brief Process a single SGM path
     *
     * @param costVolume Matching cost volume
     * @param pathCost Output cost for this path
     * @param dirX X direction (-1, 0, +1)
     * @param dirY Y direction (-1, 0, +1)
     * @param width Image width
     * @param height Image height
     */
    void processSGMPath(
        const std::vector<uint8_t>& costVolume,
        std::vector<uint16_t>& pathCost,
        int dirX, int dirY,
        int width, int height);

    /**
     * @brief Winner-Takes-All disparity selection
     *
     * For each pixel, select the disparity with minimum aggregated cost.
     *
     * @param aggregatedCost Aggregated costs from all paths
     * @param disparity Output disparity map (CV_16SC1, scaled by 16)
     * @param width Image width
     * @param height Image height
     */
    void selectDisparitiesWTA(
        const std::vector<uint32_t>& aggregatedCost,
        cv::Mat& disparity,
        int width, int height);

    // === UTILITY FUNCTIONS ===

    /**
     * @brief Count set bits in a 32-bit integer (Hamming weight)
     */
    inline int hammingWeight(uint32_t val) const {
        return __builtin_popcount(val);
    }

    /**
     * @brief Validate input images
     */
    bool validateInputs(const cv::Mat& left, const cv::Mat& right, std::string& error);
};

} // namespace reference
} // namespace stereo
} // namespace unlook
