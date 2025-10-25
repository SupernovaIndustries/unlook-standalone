#pragma once

#include "unlook/stereo/StereoMatcher.hpp"
#include <opencv2/stereo.hpp>
#include <opencv2/ximgproc.hpp>

namespace unlook {
namespace stereo {

/**
 * @brief Census Transform stereo matcher implementation
 *
 * Census Transform + Semi-Global Matching optimized for VCSEL dot patterns.
 * Based on research: "A Comparison and Evaluation of Stereo Matching on Active Stereo Images" (PMC)
 * https://pmc.ncbi.nlm.nih.gov/articles/PMC9100404/
 *
 * Key findings:
 * - "Census produced the lowest average error for active stereo"
 * - Non-parametric matching robust to illumination changes
 * - Excellent performance on sparse features (VCSEL dots)
 * - Superior to SAD/NCC for projected pattern matching
 *
 * Target: Improve point retention from 5.65% to 60-80% for BELAGO1.1 15K VCSEL dots
 *
 * Algorithm:
 * 1. Optional VCSEL dot enhancement (LoG + CLAHE)
 * 2. Census Transform computation (binary bit strings from local intensity comparisons)
 * 3. Hamming distance cost computation (XOR + popcount)
 * 4. Semi-global path aggregation (SGBM 8-path or 16-path)
 * 5. Winner-takes-all disparity selection
 * 6. Left-right consistency check
 * 7. WLS post-filtering
 */
class CensusStereoMatcher : public StereoMatcher {
public:
    CensusStereoMatcher();
    ~CensusStereoMatcher() override;

    /**
     * @brief Compute disparity map from rectified stereo pair
     */
    bool computeDisparity(const cv::Mat& leftRectified,
                         const cv::Mat& rightRectified,
                         cv::Mat& disparity) override;

    /**
     * @brief Compute disparity with confidence map
     */
    bool computeDisparityWithConfidence(const cv::Mat& leftRectified,
                                       const cv::Mat& rightRectified,
                                       cv::Mat& disparity,
                                       cv::Mat& confidence) override;

    /**
     * @brief Set stereo matching parameters
     */
    bool setParameters(const StereoMatchingParams& params) override;

    /**
     * @brief Get current stereo matching parameters
     */
    StereoMatchingParams getParameters() const override;

    /**
     * @brief Get algorithm type
     */
    StereoAlgorithm getAlgorithmType() const override {
        return StereoAlgorithm::CENSUS;
    }

    /**
     * @brief Get algorithm name
     */
    std::string getAlgorithmName() const override {
        return "Census Transform + Semi-Global Matching (VCSEL-optimized)";
    }

    /**
     * @brief Apply WLS filter post-processing
     */
    bool applyPostProcessing(cv::Mat& disparity,
                           const cv::Mat& leftImage) override;

    /**
     * @brief Get disparity computation statistics
     */
    std::map<std::string, double> getStatistics() const;

private:
    // OpenCV SGBM matcher for path aggregation
    cv::Ptr<cv::StereoSGBM> sgbm_;
    cv::Ptr<cv::StereoMatcher> rightMatcher_;  // For left-right check
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;

    // Statistics tracking
    mutable std::map<std::string, double> statistics_;

    /**
     * @brief Compute Census Transform for an image
     * @param image Input grayscale image
     * @param census Output census transform (CV_32SC1 or CV_64SC1)
     *
     * For each pixel, creates a bit string by comparing all neighbors to the center pixel.
     * Window size determines number of bits (5x5 = 24 bits, 7x7 = 48 bits).
     */
    void computeCensusTransform(const cv::Mat& image, cv::Mat& census) const;

    /**
     * @brief Compute Hamming distance between two census values
     * @param a First census value (32-bit)
     * @param b Second census value (32-bit)
     * @return Hamming distance (number of differing bits)
     *
     * Uses GCC/Clang __builtin_popcount for hardware-optimized bit counting.
     * This is typically a single CPU instruction on modern processors.
     */
    inline int hammingDistance(uint32_t a, uint32_t b) const {
        return __builtin_popcount(a ^ b);  // GCC/Clang hardware-optimized popcount
    }

    /**
     * @brief Compute cost volume using Census Transform and Hamming distance
     * @param censusLeft Census transform of left image
     * @param censusRight Census transform of right image
     * @param costVolume Output cost volume (height x width x numDisparities)
     *
     * For each pixel and disparity, computes Hamming distance between
     * corresponding census values.
     */
    void computeCostVolume(const cv::Mat& censusLeft,
                          const cv::Mat& censusRight,
                          std::vector<cv::Mat>& costVolume) const;

    /**
     * @brief Aggregate costs using Semi-Global Matching paths
     * @param costVolume Input/output cost volume with aggregated costs
     *
     * Applies SGBM 8-path or 16-path aggregation with P1/P2 smoothness penalties.
     */
    void aggregateCosts(std::vector<cv::Mat>& costVolume) const;

    /**
     * @brief Extract disparity from aggregated cost volume
     * @param costVolume Aggregated cost volume
     * @param disparity Output disparity map (CV_16S for sub-pixel, CV_32F for final)
     *
     * Winner-takes-all selection: for each pixel, select disparity with minimum cost.
     */
    void extractDisparity(const std::vector<cv::Mat>& costVolume, cv::Mat& disparity) const;

    /**
     * @brief Apply VCSEL dot enhancement preprocessing
     * @param input Input grayscale image
     * @param output Enhanced output image
     *
     * Same as SGBMStereoMatcher: LoG + CLAHE + optional bilateral filtering
     * for BELAGO1.1 15K VCSEL dot pattern enhancement.
     */
    void applyVCSELDotEnhancement(const cv::Mat& input, cv::Mat& output) const;

    /**
     * @brief Update SGBM parameters (for path aggregation)
     */
    void updateSGBMParameters();

    /**
     * @brief Create WLS filter for post-processing
     */
    void createWLSFilter();

    /**
     * @brief Compute confidence map from left-right consistency
     */
    void computeConfidenceMap(const cv::Mat& disparity,
                             const cv::Mat& rightDisparity,
                             cv::Mat& confidence) const;

    /**
     * @brief Apply speckle filtering
     */
    void applySpeckleFilter(cv::Mat& disparity) const;

    /**
     * @brief Fill holes in disparity map
     */
    void fillDisparityHoles(cv::Mat& disparity) const;
};

} // namespace stereo
} // namespace unlook
