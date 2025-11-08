/**
 * @file SpatialFilter.hpp
 * @brief Edge-preserving spatial filter for depth maps
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Inspired by Intel RealSense spatial filter implementation.
 */

#ifndef UNLOOK_STEREO_SPATIAL_FILTER_HPP
#define UNLOOK_STEREO_SPATIAL_FILTER_HPP

#include <opencv2/core.hpp>
#include <memory>
#include <chrono>

namespace unlook {

namespace core {
    class Logger;
}

namespace stereo {

/**
 * @brief Edge-preserving bilateral spatial filter
 *
 * Smooths depth maps while preserving edges using bilateral filtering.
 * Based on RealSense approach with adaptive weights.
 */
class SpatialFilter {
public:
    /**
     * @brief Filter configuration
     */
    struct Config {
        int radius = 2;                          ///< Filter radius in pixels (1-5)
        float spatialSigma = 9.0f;               ///< Spatial weight sigma
        float rangeSigma = 0.5f;                 ///< Range (depth) weight sigma (relative)
        int iterations = 1;                      ///< Number of iterations
        bool preserveEdges = true;               ///< Use edge-preserving weights
        float edgeThreshold = 0.1f;              ///< Edge threshold (relative depth change)
        bool useColorGuide = false;              ///< Use color image for guidance
    };

    /**
     * @brief Filter result
     */
    struct Result {
        cv::Mat filtered;                        ///< Filtered depth map
        cv::Mat edgeMap;                         ///< Detected edges (CV_8U)

        bool success;
        std::string errorMessage;

        // Statistics
        float smoothnessImprovement;             ///< Reduction in noise (%)
        int edgePixels;                          ///< Number of edge pixels preserved
        std::chrono::microseconds processingTime;
    };

    /**
     * @brief Construct spatial filter
     *
     * @param logger Logger for output messages
     */
    explicit SpatialFilter(core::Logger* logger = nullptr);

    ~SpatialFilter();

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Apply spatial filter to depth map
     *
     * @param depthMap Input depth map (CV_32F in mm)
     * @param colorImage Optional color image for guidance (CV_8UC3)
     * @return Filtered result
     */
    Result filter(const cv::Mat& depthMap, const cv::Mat& colorImage = cv::Mat());

    /**
     * @brief Apply to disparity map
     *
     * @param disparity Input disparity map (CV_16S or CV_32F)
     * @return Filtered result
     */
    Result filterDisparity(const cv::Mat& disparity);

private:
    core::Logger* logger_;
    Config config_;

    /**
     * @brief Apply bilateral filter with adaptive weights
     */
    void applyBilateralFilter(const cv::Mat& input, cv::Mat& output,
                              const cv::Mat& guide, int radius,
                              float spatialSigma, float rangeSigma);

    /**
     * @brief Detect edges in depth map
     */
    void detectEdges(const cv::Mat& depthMap, cv::Mat& edgeMap, float threshold);

    /**
     * @brief Calculate smoothness metric
     */
    float calculateSmoothness(const cv::Mat& depthMap);

    /**
     * @brief Apply guided filter using color image
     */
    void applyGuidedFilter(const cv::Mat& input, const cv::Mat& guide, cv::Mat& output,
                           int radius, float epsilon);
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_SPATIAL_FILTER_HPP