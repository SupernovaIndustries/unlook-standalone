/**
 * @file MedianFilter.hpp
 * @brief Median filter for speckle removal in depth maps
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Inspired by Luxonis OAK-D implementation.
 */

#ifndef UNLOOK_STEREO_MEDIAN_FILTER_HPP
#define UNLOOK_STEREO_MEDIAN_FILTER_HPP

#include <opencv2/core.hpp>
#include <memory>

namespace unlook {

namespace core {
    class Logger;
}

namespace stereo {

/**
 * @brief Median filter for speckle noise removal
 *
 * Removes salt-and-pepper noise from disparity/depth maps using median filtering.
 * Optimized with NEON instructions for ARM64 when possible.
 */
class MedianFilter {
public:
    /**
     * @brief Filter configuration
     */
    struct Config {
        enum KernelSize {
            KERNEL_3x3 = 3,
            KERNEL_5x5 = 5,
            KERNEL_7x7 = 7,
            KERNEL_9x9 = 9
        };

        KernelSize kernelSize = KERNEL_5x5;      ///< Median filter kernel size
        bool preserveEdges = true;               ///< Use edge-aware median
        float edgeThreshold = 50.0f;             ///< Edge detection threshold (mm)
        bool useNEON = true;                     ///< Use NEON optimization if available
    };

    /**
     * @brief Filter result
     */
    struct Result {
        cv::Mat filtered;                        ///< Filtered depth/disparity map
        cv::Mat speckleMask;                     ///< Mask of removed speckles (CV_8U)

        bool success;
        std::string errorMessage;

        // Statistics
        int specklesRemoved;                     ///< Number of speckle pixels removed
        float specklePercentage;                 ///< Percentage of pixels identified as speckles
        std::chrono::microseconds processingTime;
    };

    /**
     * @brief Construct median filter
     *
     * @param logger Logger for output messages
     */
    explicit MedianFilter(core::Logger* logger = nullptr);

    ~MedianFilter();

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Apply median filter to depth map
     *
     * @param depthMap Input depth map (CV_32F in mm)
     * @return Filtered result
     */
    Result filterDepth(const cv::Mat& depthMap);

    /**
     * @brief Apply median filter to disparity map
     *
     * @param disparity Input disparity map (CV_16S or CV_32F)
     * @return Filtered result
     */
    Result filterDisparity(const cv::Mat& disparity);

private:
    core::Logger* logger_;
    Config config_;

    /**
     * @brief Apply standard median filter
     */
    void applyMedianFilter(const cv::Mat& input, cv::Mat& output, int kernelSize);

    /**
     * @brief Apply edge-preserving median filter
     */
    void applyEdgePreservingMedian(const cv::Mat& input, cv::Mat& output,
                                   int kernelSize, float edgeThreshold);

    /**
     * @brief NEON-optimized median for 5x5 kernel
     */
    void median5x5NEON(const cv::Mat& input, cv::Mat& output);

    /**
     * @brief Detect speckles in the image
     */
    void detectSpeckles(const cv::Mat& original, const cv::Mat& filtered, cv::Mat& speckleMask);
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_MEDIAN_FILTER_HPP