#pragma once

#include "StereoMatcher.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <atomic>
#include <mutex>

namespace unlook {
namespace stereo {

/**
 * @brief VCSEL-optimized stereo matcher using AD-Census algorithm
 *
 * Implements AD-Census fusion algorithm optimized for VCSEL structured light:
 * - Census Transform with 9x9 window (80-bit descriptor)
 * - Absolute Difference (AD) cost computation
 * - Cost fusion: 0.3×AD + 0.7×Census (optimized for VCSEL patterns)
 * - 4-path SGM aggregation for real-time performance
 * - ARM NEON optimizations for Raspberry Pi CM4/CM5
 * - HD 1280x720 processing for ~10 FPS handheld scanning
 *
 * Target Performance @ HD 1280x720:
 * - Census transform: ~6ms
 * - Hamming distance: ~9ms
 * - AD computation: ~3ms
 * - Cost fusion: ~1.5ms
 * - SGM 4-path: ~75ms
 * - Post-processing: ~15ms
 * Total: ~110ms → 9-10 FPS
 */
class VCSELStereoMatcher : public StereoMatcher {
public:
    /**
     * @brief Constructor with NEON detection
     */
    VCSELStereoMatcher();

    /**
     * @brief Destructor
     */
    virtual ~VCSELStereoMatcher();

    /**
     * @brief Compute disparity map from rectified stereo pair
     * @param leftRectified Rectified left image (1456x1088 or pre-downsampled)
     * @param rightRectified Rectified right image (1456x1088 or pre-downsampled)
     * @param disparity Output disparity map (CV_32F, HD 1280x720)
     * @return true if computation successful
     */
    virtual bool computeDisparity(const cv::Mat& leftRectified,
                                 const cv::Mat& rightRectified,
                                 cv::Mat& disparity) override;

    /**
     * @brief Set stereo matching parameters
     * @param params Stereo matching parameters (AD-Census specific)
     * @return true if parameters set successfully
     */
    virtual bool setParameters(const StereoMatchingParams& params) override;

    /**
     * @brief Get current stereo matching parameters
     * @return Current AD-Census parameters
     */
    virtual StereoMatchingParams getParameters() const override;

    /**
     * @brief Get algorithm type
     * @return StereoAlgorithm::CUSTOM (AD-Census)
     */
    virtual StereoAlgorithm getAlgorithmType() const override {
        return StereoAlgorithm::CUSTOM;
    }

    /**
     * @brief Get algorithm name
     * @return "VCSEL AD-Census"
     */
    virtual std::string getAlgorithmName() const override {
        return "VCSEL AD-Census";
    }

    /**
     * @brief VCSEL pattern isolation
     * @param vcsellImage Image with VCSEL pattern
     * @param ambientImage Image without VCSEL (optional)
     * @param isolated Output isolated VCSEL pattern
     * @return true if isolation successful
     */
    bool patternIsolation(const cv::Mat& vcselImage,
                         const cv::Mat& ambientImage,
                         cv::Mat& isolated);

    /**
     * @brief Get processing statistics
     */
    struct ProcessingStats {
        double downsampleTimeMs = 0.0;
        double censusTimeMs = 0.0;
        double hammingTimeMs = 0.0;
        double adCostTimeMs = 0.0;
        double fusionTimeMs = 0.0;
        double sgmTimeMs = 0.0;
        double postProcessingTimeMs = 0.0;
        double totalTimeMs = 0.0;
        size_t validPixels = 0;
        size_t totalPixels = 0;

        std::string toString() const;
    };

    ProcessingStats getLastProcessingStats() const {
        std::lock_guard<std::mutex> lock(statsMutex_);
        return lastStats_;
    }

private:
    // AD-Census specific parameters
    struct ADCensusParams {
        // Census parameters
        int censusWindowRadius = 4;  // 9x9 window (2*radius+1)
        int censusThreshold = 4;     // Illumination tolerance

        // AD-Census fusion weights
        float lambdaAD = 0.3f;        // AD cost weight
        float lambdaCensus = 0.7f;    // Census cost weight

        // SGM parameters optimized for VCSEL
        int minDisparity = 48;        // Minimum disparity for 70mm baseline
        int numDisparities = 256;     // Range optimized for 100-800mm depth
        int P1 = 4;                   // Small penalty (preserve VCSEL dots)
        int P2 = 24;                  // Large penalty (moderate smoothing)
        int uniquenessRatio = 25;     // Strict matching for precision

        // Subpixel refinement
        bool useSubpixel = true;
        int subpixelScale = 16;       // 1/16 pixel accuracy

        // Processing resolution
        cv::Size processingSize{1280, 720};  // HD processing
    } adCensusParams_;

    // Processing methods

    /**
     * @brief Downsample image to HD resolution
     * @param input Input image (1456x1088)
     * @param output Output image (1280x720)
     */
    void downsampleImage(const cv::Mat& input, cv::Mat& output);

    /**
     * @brief Compute Census Transform with NEON optimization
     * @param image Input grayscale image
     * @param census Output census descriptors (CV_64FC2 for 80-bit)
     */
    void censusTransform(const cv::Mat& image, cv::Mat& census);

    /**
     * @brief Compute Census Transform (non-NEON fallback)
     */
    void censusTransformCPU(const cv::Mat& image, cv::Mat& census);

    /**
     * @brief Compute Census Transform (NEON optimized)
     */
    void censusTransformNEON(const cv::Mat& image, cv::Mat& census);

    /**
     * @brief Compute Hamming distance cost volume
     * @param censusLeft Left census descriptors
     * @param censusRight Right census descriptors
     * @param costVolume Output cost volume (H×W×D)
     */
    void computeHammingCost(const cv::Mat& censusLeft,
                           const cv::Mat& censusRight,
                           cv::Mat& costVolume);

    /**
     * @brief Compute Hamming distance (NEON optimized)
     */
    void computeHammingCostNEON(const cv::Mat& censusLeft,
                                const cv::Mat& censusRight,
                                cv::Mat& costVolume);

    /**
     * @brief Compute Absolute Difference cost volume
     * @param left Left grayscale image
     * @param right Right grayscale image
     * @param costVolume Output AD cost volume
     */
    void computeADCost(const cv::Mat& left,
                      const cv::Mat& right,
                      cv::Mat& costVolume);

    /**
     * @brief Compute AD cost (NEON optimized)
     */
    void computeADCostNEON(const cv::Mat& left,
                          const cv::Mat& right,
                          cv::Mat& costVolume);

    /**
     * @brief Fuse AD and Census costs
     * @param adCost AD cost volume
     * @param censusCost Census cost volume
     * @param fusedCost Output fused cost volume
     */
    void fuseCosts(const cv::Mat& adCost,
                  const cv::Mat& censusCost,
                  cv::Mat& fusedCost);

    /**
     * @brief 4-path SGM aggregation
     * @param costVolume Input cost volume
     * @param aggregatedCost Output aggregated cost
     */
    void sgmAggregation(const cv::Mat& costVolume,
                       cv::Mat& aggregatedCost);

    /**
     * @brief Winner-take-all disparity selection
     * @param aggregatedCost Aggregated cost volume
     * @param disparity Output disparity map
     */
    void winnerTakeAll(const cv::Mat& aggregatedCost,
                      cv::Mat& disparity);

    /**
     * @brief Subpixel refinement using parabolic fitting
     * @param aggregatedCost Aggregated cost volume
     * @param disparity Input/output disparity map
     */
    void subpixelRefinement(const cv::Mat& aggregatedCost,
                           cv::Mat& disparity);

    /**
     * @brief Post-processing (speckle filter, median filter)
     * @param disparity Input/output disparity map
     */
    void postProcessDisparity(cv::Mat& disparity);

    // Helper methods

    /**
     * @brief Compute Hamming distance between two 80-bit descriptors
     */
    inline int hammingDistance80(const uint64_t* desc1, const uint64_t* desc2) const;

    /**
     * @brief Check if NEON is available
     */
    bool checkNEONSupport() const;

    // Member variables
    bool useNEON_;                          // NEON availability flag
    mutable std::mutex statsMutex_;         // Stats protection
    mutable ProcessingStats lastStats_;     // Last processing statistics

    // Pre-allocated buffers for performance
    cv::Mat censusLeft_, censusRight_;      // Census descriptors
    cv::Mat adCostVolume_;                  // AD cost volume
    cv::Mat censusCostVolume_;              // Census cost volume
    cv::Mat fusedCostVolume_;               // Fused cost volume
    cv::Mat aggregatedCostVolume_;          // SGM aggregated cost
    cv::Mat leftDownsampled_, rightDownsampled_;  // HD images

    // Vulkan compute (experimental, optional)
    bool tryInitVulkan();
    bool vulkanAvailable_ = false;
    void* vulkanContext_ = nullptr;  // Opaque pointer to Vulkan context
};

} // namespace stereo
} // namespace unlook