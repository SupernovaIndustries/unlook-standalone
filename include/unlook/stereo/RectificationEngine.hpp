/**
 * @file RectificationEngine.hpp
 * @brief High-performance stereo image rectification with validation
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#ifndef UNLOOK_STEREO_RECTIFICATION_ENGINE_HPP
#define UNLOOK_STEREO_RECTIFICATION_ENGINE_HPP

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <memory>

namespace unlook {
namespace core {
    class Logger;
}
namespace calibration {
    class CalibrationManager;
}
namespace stereo {

/**
 * @brief Stereo image rectification engine with industrial-grade validation
 *
 * Performs epipolar rectification of stereo image pairs using calibration data.
 * CRITICAL: Validates image size matches calibration BEFORE rectification.
 */
class RectificationEngine {
public:
    /**
     * @brief Rectification configuration
     */
    struct Config {
        /// Interpolation method (INTER_LINEAR, INTER_CUBIC, INTER_LANCZOS4)
        int interpolation = cv::INTER_LINEAR;

        /// Fill value for border pixels (default: black)
        cv::Scalar borderValue = cv::Scalar(0, 0, 0);

        /// Enable size validation (CRITICAL: should always be true in production)
        bool validateSize = true;

        /// Enable debug output (epipolar line visualization)
        bool enableDebug = false;
        std::string debugDir = "";
    };

    /**
     * @brief Rectification result
     */
    struct Result {
        cv::Mat leftRectified;       ///< Rectified left image
        cv::Mat rightRectified;      ///< Rectified right image

        bool success;                ///< Overall success flag
        std::string errorMessage;    ///< Error message if failed

        // Quality metrics
        double validPixelPercent;    ///< Percentage of valid (non-border) pixels
        cv::Size outputSize;         ///< Size of rectified images

        // Performance metrics
        std::chrono::milliseconds rectificationTime;
    };

    /**
     * @brief Construct rectification engine
     *
     * @param calib Calibration manager with loaded stereo calibration
     * @param logger Logger for output messages
     */
    explicit RectificationEngine(
        calibration::CalibrationManager* calib,
        core::Logger* logger = nullptr);

    ~RectificationEngine();

    /**
     * @brief Configure rectification parameters
     *
     * @param config Rectification configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     *
     * @return Current rectification configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Rectify stereo image pair
     *
     * CRITICAL: Input images MUST match calibration size!
     * If validateSize is enabled, this will abort on size mismatch.
     *
     * @param leftInput Left camera input image
     * @param rightInput Right camera input image
     * @return Rectification result with rectified images and metrics
     */
    Result rectify(const cv::Mat& leftInput, const cv::Mat& rightInput);

    /**
     * @brief Pre-compute rectification maps (optional optimization)
     *
     * Rectification maps are computed on first rectify() call.
     * This method allows pre-computing them to avoid first-frame latency.
     *
     * @return true if maps computed successfully
     */
    bool precomputeMaps();

    /**
     * @brief Check if rectification maps are ready
     *
     * @return true if maps are precomputed
     */
    bool areMapsReady() const { return mapsComputed_; }

    /**
     * @brief Get expected input image size from calibration
     *
     * @return Expected image size
     */
    cv::Size getExpectedInputSize() const;

private:
    calibration::CalibrationManager* calibration_;
    core::Logger* logger_;
    Config config_;

    // Rectification maps (cached)
    cv::Mat map1Left_, map2Left_;
    cv::Mat map1Right_, map2Right_;
    bool mapsComputed_ = false;

    /**
     * @brief Compute rectification maps from calibration
     *
     * @return true if successful
     */
    bool computeMaps();

    /**
     * @brief Validate input image sizes
     *
     * @param leftInput Left input image
     * @param rightInput Right input image
     * @param errorMsg Output error message
     * @return true if validation passed
     */
    bool validateInputSizes(
        const cv::Mat& leftInput,
        const cv::Mat& rightInput,
        std::string& errorMsg) const;

    /**
     * @brief Calculate percentage of valid (non-border) pixels
     *
     * @param rectified Rectified image
     * @return Percentage of non-black pixels
     */
    double calculateValidPixelPercent(const cv::Mat& rectified) const;

    /**
     * @brief Save debug visualization (epipolar lines)
     *
     * @param leftRect Rectified left image
     * @param rightRect Rectified right image
     */
    void saveDebugVisualization(
        const cv::Mat& leftRect,
        const cv::Mat& rightRect) const;

    /**
     * @brief Draw horizontal epipolar lines on rectified image pair
     *
     * @param leftRect Rectified left image
     * @param rightRect Rectified right image
     * @return Combined visualization image
     */
    cv::Mat drawEpipolarLines(
        const cv::Mat& leftRect,
        const cv::Mat& rightRect) const;
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_RECTIFICATION_ENGINE_HPP
