/**
 * @file HoleFiller.hpp
 * @brief Hole filling for missing depth values
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Inspired by Luxonis and RealSense hole filling approaches.
 */

#ifndef UNLOOK_STEREO_HOLE_FILLER_HPP
#define UNLOOK_STEREO_HOLE_FILLER_HPP

#include <opencv2/core.hpp>
#include <memory>
#include <chrono>

namespace unlook {

namespace core {
    class Logger;
}

namespace stereo {

/**
 * @brief Intelligent hole filling for depth maps
 *
 * Fills missing depth values using various inpainting strategies.
 * Optimized for real-time performance on ARM64.
 */
class HoleFiller {
public:
    /**
     * @brief Hole filling method
     */
    enum class Method {
        NEAREST_NEIGHBOR,     ///< Fill with nearest valid depth
        WEIGHTED_AVERAGE,     ///< Weighted average of neighbors
        MORPHOLOGICAL,        ///< Morphological closing
        INPAINTING,          ///< OpenCV inpainting (slow but high quality)
        DIRECTIONAL,         ///< Direction-aware filling (preserves edges)
        HYBRID               ///< Combination of methods based on hole size
    };

    /**
     * @brief Filter configuration
     */
    struct Config {
        Method method = Method::HYBRID;
        int maxHoleSize = 100;                   ///< Maximum hole size to fill (pixels)
        float maxDepthGap = 50.0f;               ///< Maximum depth difference for filling (mm)
        int dilationSize = 3;                    ///< Dilation kernel size for morphological
        bool preserveEdges = true;               ///< Preserve edges during filling
        float edgeThreshold = 0.1f;              ///< Edge detection threshold (relative)
    };

    /**
     * @brief Fill result
     */
    struct Result {
        cv::Mat filled;                          ///< Filled depth map
        cv::Mat holeMask;                        ///< Mask of filled holes (CV_8U)

        bool success;
        std::string errorMessage;

        // Statistics
        int totalHoles;                          ///< Number of hole regions detected
        int filledHoles;                         ///< Number of holes successfully filled
        int filledPixels;                        ///< Total pixels filled
        float fillPercentage;                    ///< Percentage of image that was holes
        std::chrono::microseconds processingTime;
    };

    /**
     * @brief Construct hole filler
     *
     * @param logger Logger for output messages
     */
    explicit HoleFiller(core::Logger* logger = nullptr);

    ~HoleFiller();

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Fill holes in depth map
     *
     * @param depthMap Input depth map (CV_32F in mm)
     * @param colorImage Optional color image for guided filling
     * @return Filled result
     */
    Result fill(const cv::Mat& depthMap, const cv::Mat& colorImage = cv::Mat());

    /**
     * @brief Fill holes in disparity map
     *
     * @param disparity Input disparity map (CV_16S or CV_32F)
     * @return Filled result
     */
    Result fillDisparity(const cv::Mat& disparity);

private:
    core::Logger* logger_;
    Config config_;

    /**
     * @brief Detect hole regions
     */
    void detectHoles(const cv::Mat& depthMap, cv::Mat& holeMask, int& numHoles);

    /**
     * @brief Fill using nearest neighbor
     */
    void fillNearestNeighbor(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output);

    /**
     * @brief Fill using weighted average
     */
    void fillWeightedAverage(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output);

    /**
     * @brief Fill using morphological operations
     */
    void fillMorphological(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output);

    /**
     * @brief Fill using OpenCV inpainting
     */
    void fillInpainting(const cv::Mat& input, const cv::Mat& holeMask, cv::Mat& output);

    /**
     * @brief Fill using directional propagation
     */
    void fillDirectional(const cv::Mat& input, const cv::Mat& holeMask,
                        const cv::Mat& edges, cv::Mat& output);

    /**
     * @brief Hybrid filling based on hole characteristics
     */
    void fillHybrid(const cv::Mat& input, const cv::Mat& holeMask,
                   const cv::Mat& edges, cv::Mat& output);

    /**
     * @brief Analyze hole size and characteristics
     */
    void analyzeHoles(const cv::Mat& holeMask, std::vector<cv::Rect>& holeRegions,
                     std::vector<int>& holeSizes);

    /**
     * @brief Detect edges for edge-aware filling
     */
    void detectEdges(const cv::Mat& depthMap, cv::Mat& edges);
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_HOLE_FILLER_HPP