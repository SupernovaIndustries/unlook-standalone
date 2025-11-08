/**
 * @file DisparityComputer.hpp
 * @brief High-performance disparity computation with multiple algorithms
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Supports AD-Census (CPU/NEON), SGBM, and Vulkan GPU acceleration.
 */

#ifndef UNLOOK_STEREO_DISPARITY_COMPUTER_HPP
#define UNLOOK_STEREO_DISPARITY_COMPUTER_HPP

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <string>
#include <memory>
#include <chrono>

namespace unlook {
namespace core {
    class Logger;
}
namespace stereo {
    class VulkanSGMAccelerator;

/**
 * @brief Disparity computation method
 */
enum class DisparityMethod {
    SGBM_OPENCV,          ///< OpenCV Semi-Global Block Matching
    AD_CENSUS_CPU,        ///< AD-Census transform (CPU/NEON)
    VULKAN_SGM,           ///< Vulkan GPU-accelerated SGM
    AUTO                  ///< Automatic selection (GPU if available, else CPU)
};

/**
 * @brief High-performance disparity computation engine
 *
 * Provides multiple stereo matching algorithms:
 * - OpenCV SGBM (baseline)
 * - AD-Census + SGBM (better for VCSEL structured light)
 * - Vulkan SGM (GPU-accelerated)
 */
class DisparityComputer {
public:
    /**
     * @brief Disparity computation configuration
     */
    struct Config {
        DisparityMethod method = DisparityMethod::AUTO;

        // Disparity range
        int minDisparity = 0;
        int numDisparities = 128;        // Must be divisible by 16

        // SGBM parameters
        int blockSize = 5;               // Odd number (3, 5, 7, 9, 11)
        int P1 = 4;                      // Small smoothness penalty
        int P2 = 24;                     // Large smoothness penalty
        int disp12MaxDiff = 1;           // Left-right check threshold
        int preFilterCap = 63;           // Texture filter limit
        int uniquenessRatio = 15;        // Uniqueness check (5-15%)
        int speckleWindowSize = 200;     // Speckle filter window
        int speckleRange = 2;            // Speckle filter range
        int mode = cv::StereoSGBM::MODE_SGBM; // or MODE_HH, MODE_SGBM_3WAY

        // AD-Census parameters (when using AD_CENSUS_CPU)
        float lambdaAD = 0.3f;           // AD weight (0.3 typical)
        float lambdaCensus = 0.7f;       // Census weight (0.7 typical)
        int censusWindowSize = 9;        // Census transform window (9x9)
        int censusThreshold = 4;         // Illumination tolerance

        // Subpixel refinement
        bool useSubpixel = true;
        int subpixelScale = 16;          // 1/16 pixel precision (fixed in OpenCV)

        // Post-filtering
        bool useWLSFilter = true;        // Weighted Least Squares filter
        double wlsLambda = 8000.0;       // WLS smoothness
        double wlsSigma = 1.5;           // WLS edge sensitivity

        // Performance
        int numThreads = 4;              // CPU threads (when not using GPU)
    };

    /**
     * @brief Disparity computation result
     */
    struct Result {
        cv::Mat disparity;               ///< Disparity map (CV_16S or CV_32F)
        cv::Mat confidence;              ///< Confidence map (CV_8U, 0-255)

        bool success;
        std::string errorMessage;

        // Quality metrics
        float validPixelPercentage;      ///< % of valid (non-zero) disparity pixels
        float meanDisparity;             ///< Mean disparity value
        float disparityStdDev;           ///< Disparity standard deviation

        // Performance metrics
        std::chrono::milliseconds computeTime;
        std::chrono::milliseconds filterTime;
        std::chrono::milliseconds totalTime;

        DisparityMethod methodUsed;      ///< Actual method used
        bool gpuAccelerated;             ///< True if GPU was used
    };

    /**
     * @brief Construct disparity computer
     *
     * @param logger Logger for output messages
     */
    explicit DisparityComputer(core::Logger* logger = nullptr);
    ~DisparityComputer();

    /**
     * @brief Configure disparity computation
     *
     * @param config Configuration parameters
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Initialize GPU accelerator (if available)
     *
     * @return true if GPU acceleration available
     */
    bool initializeGPU();

    /**
     * @brief Check if GPU acceleration is available
     */
    bool isGPUAvailable() const;

    /**
     * @brief Compute disparity from rectified stereo pair
     *
     * CRITICAL: Input images MUST be rectified!
     *
     * @param leftRect Rectified left image
     * @param rightRect Rectified right image
     * @return Disparity computation result
     */
    Result compute(const cv::Mat& leftRect, const cv::Mat& rightRect);

private:
    core::Logger* logger_;
    Config config_;

    // Algorithm implementations
    cv::Ptr<cv::StereoSGBM> sgbmMatcher_;
    std::unique_ptr<VulkanSGMAccelerator> vulkanAccelerator_;

    // WLS filter
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;

    bool gpuInitialized_ = false;

    /**
     * @brief Compute using OpenCV SGBM
     */
    Result computeSGBM(const cv::Mat& leftRect, const cv::Mat& rightRect);

    /**
     * @brief Compute using AD-Census CPU
     */
    Result computeADCensus(const cv::Mat& leftRect, const cv::Mat& rightRect);

    /**
     * @brief Compute using Vulkan GPU
     */
    Result computeVulkan(const cv::Mat& leftRect, const cv::Mat& rightRect);

    /**
     * @brief Apply WLS post-filtering
     */
    void applyWLSFilter(
        const cv::Mat& leftRect,
        cv::Mat& disparity,
        cv::Mat& confidence);

    /**
     * @brief Calculate quality metrics
     */
    void calculateMetrics(const cv::Mat& disparity, Result& result);

    /**
     * @brief Update SGBM matcher with current config
     */
    void updateSGBMMatcher();

    /**
     * @brief Update WLS filter with current config
     */
    void updateWLSFilter();
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_DISPARITY_COMPUTER_HPP
