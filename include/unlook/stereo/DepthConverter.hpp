/**
 * @file DepthConverter.hpp
 * @brief Convert disparity maps to depth using calibration Q matrix
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Precision target: 0.005mm at close range (100-800mm).
 */

#ifndef UNLOOK_STEREO_DEPTH_CONVERTER_HPP
#define UNLOOK_STEREO_DEPTH_CONVERTER_HPP

#include <opencv2/core.hpp>
#include <memory>

namespace unlook {

namespace calibration {
    class CalibrationManager;
}

namespace core {
    class Logger;
}

namespace stereo {

/**
 * @brief High-precision disparity to depth conversion
 *
 * Converts disparity maps to metric depth using the Q matrix from stereo calibration.
 * Optimized for 70mm baseline with target precision of 0.005mm at close range.
 */
class DepthConverter {
public:
    /**
     * @brief Conversion configuration
     */
    struct Config {
        bool handleInvalidPixels = true;     ///< Replace invalid disparities with NaN
        float minDepthMM = 100.0f;           ///< Minimum valid depth (mm)
        float maxDepthMM = 2000.0f;          ///< Maximum valid depth (mm)
        float disparityScale = 16.0f;        ///< Disparity scale factor (for fixed-point)
        bool clampToRange = false;           ///< Clamp depth to min/max range
    };

    /**
     * @brief Conversion result
     */
    struct Result {
        cv::Mat depthMap;                    ///< Depth map in mm (CV_32F)
        cv::Mat validMask;                   ///< Valid pixel mask (CV_8U)

        bool success;
        std::string errorMessage;

        // Statistics
        int totalPixels;
        int validPixels;
        float validPercentage;
        float minDepth;                      ///< Minimum depth value (mm)
        float maxDepth;                      ///< Maximum depth value (mm)
        float meanDepth;                     ///< Mean depth value (mm)
        float stdDevDepth;                   ///< Depth standard deviation (mm)
    };

    /**
     * @brief Construct depth converter
     *
     * @param calibration Calibration manager with Q matrix
     * @param logger Logger for output messages
     */
    DepthConverter(calibration::CalibrationManager* calibration,
                   core::Logger* logger = nullptr);

    ~DepthConverter();

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Convert disparity to depth
     *
     * Uses the Q matrix from calibration to convert disparity values to metric depth.
     *
     * @param disparity Input disparity map (CV_16S with scale or CV_32F)
     * @return Conversion result with depth map and statistics
     */
    Result convert(const cv::Mat& disparity);

    /**
     * @brief Convert disparity to depth with custom Q matrix
     *
     * @param disparity Input disparity map
     * @param Q Custom 4x4 reprojection matrix
     * @return Conversion result
     */
    Result convertWithQ(const cv::Mat& disparity, const cv::Mat& Q);

    /**
     * @brief Get the Q matrix being used
     */
    cv::Mat getQMatrix() const;

    /**
     * @brief Validate Q matrix for expected baseline
     *
     * @param expectedBaseline Expected baseline in mm (e.g., 70mm)
     * @param tolerance Tolerance in mm
     * @return true if Q matrix baseline matches expected value
     */
    bool validateBaseline(float expectedBaseline, float tolerance = 5.0f);

private:
    calibration::CalibrationManager* calibration_;
    core::Logger* logger_;
    Config config_;

    /**
     * @brief Calculate depth statistics
     */
    void calculateStatistics(const cv::Mat& depthMap, const cv::Mat& validMask, Result& result);

    /**
     * @brief Apply depth range clamping
     */
    void applyDepthClamping(cv::Mat& depthMap, const cv::Mat& validMask);
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_DEPTH_CONVERTER_HPP