#pragma once

#include "unlook/stereo/StereoMatcher.hpp"
#include <opencv2/stereo.hpp>
#include <opencv2/ximgproc.hpp>

namespace unlook {
namespace stereo {

/**
 * @brief Semi-Global Block Matching stereo matcher implementation
 * 
 * High-precision SGBM implementation optimized for industrial applications
 * with target precision of 0.005mm repeatability.
 */
class SGBMStereoMatcher : public StereoMatcher {
public:
    SGBMStereoMatcher();
    ~SGBMStereoMatcher() override;
    
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
        return StereoAlgorithm::SGBM;
    }
    
    /**
     * @brief Get algorithm name
     */
    std::string getAlgorithmName() const override {
        return "Semi-Global Block Matching (OpenCV)";
    }
    
    /**
     * @brief Apply WLS filter post-processing
     */
    bool applyPostProcessing(cv::Mat& disparity,
                           const cv::Mat& leftImage) override;
    
    /**
     * @brief Set precision mode for high-accuracy applications
     * @param highPrecision If true, use settings optimized for precision
     */
    void setPrecisionMode(bool highPrecision);
    
    /**
     * @brief Enable/disable GPU acceleration (if available)
     * @param useGPU If true, attempt to use GPU acceleration
     */
    void setGPUAcceleration(bool useGPU);
    
    /**
     * @brief Get disparity computation statistics
     * @return Map of statistic name to value
     */
    std::map<std::string, double> getStatistics() const;

private:
    cv::Ptr<cv::StereoSGBM> sgbm_;
    cv::Ptr<cv::StereoMatcher> rightMatcher_;  // For left-right check
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;
    
    bool highPrecisionMode_ = true;
    bool useGPU_ = false;
    
    // Statistics tracking
    mutable std::map<std::string, double> statistics_;
    
    /**
     * @brief Update SGBM algorithm parameters
     */
    void updateSGBMParameters();
    
    /**
     * @brief Create WLS filter for post-processing
     */
    void createWLSFilter();
    
    /**
     * @brief Compute confidence map from disparity
     */
    void computeConfidenceMap(const cv::Mat& disparity,
                             const cv::Mat& leftDisparity,
                             cv::Mat& confidence) const;
    
    /**
     * @brief Apply speckle filtering
     */
    void applySpeckleFilter(cv::Mat& disparity) const;
    
    /**
     * @brief Fill holes in disparity map
     */
    void fillDisparityHoles(cv::Mat& disparity) const;

    /**
     * @brief Apply VCSEL dot enhancement preprocessing
     * @param input Input grayscale image
     * @param output Enhanced output image
     *
     * Applies Laplacian of Gaussian (LoG) for dot detection,
     * CLAHE for local contrast enhancement, and optional bilateral filtering.
     * Optimized for BELAGO1.1 15K VCSEL dot pattern (1-2 pixel dots).
     */
    void applyVCSELDotEnhancement(const cv::Mat& input, cv::Mat& output) const;
};

} // namespace stereo
} // namespace unlook