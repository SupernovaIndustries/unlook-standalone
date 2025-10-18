/**
 * @file MotionBlurCompensation.hpp
 * @brief Motion blur compensation for fast hand movements
 *
 * Lightweight preprocessing to reduce motion blur effects during fast swipes.
 * Uses fast bilateral filtering to preserve edges while reducing noise.
 *
 * Performance: ~2-3ms overhead on typical gesture recognition frames (640x480).
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_MOTION_BLUR_COMPENSATION_HPP
#define UNLOOK_GESTURE_MOTION_BLUR_COMPENSATION_HPP

#include <opencv2/core.hpp>

namespace unlook {
namespace gesture {

/**
 * @brief Motion blur compensation configuration
 */
struct MotionBlurConfig {
    int bilateral_diameter = 5;      ///< Bilateral filter diameter (5 = fast, 9 = quality)
    double sigma_color = 50.0;       ///< Color space sigma (50 = moderate smoothing)
    double sigma_space = 50.0;       ///< Coordinate space sigma (50 = moderate spatial)
    bool enabled = true;             ///< Enable/disable preprocessing

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return bilateral_diameter > 0 && bilateral_diameter % 2 == 1 &&
               sigma_color > 0.0 && sigma_space > 0.0;
    }
};

/**
 * @brief Motion blur compensation for fast hand tracking
 *
 * Applies lightweight preprocessing to improve tracking during fast motion:
 * - Bilateral filtering preserves hand edges while reducing motion blur noise
 * - Low computational overhead (~2-3ms) suitable for real-time processing
 * - Configurable parameters for performance/quality tradeoff
 *
 * Usage:
 * @code
 * MotionBlurCompensation compensator;
 * cv::Mat processed = compensator.process(input_frame);
 * // Use processed frame for MediaPipe detection
 * @endcode
 *
 * Thread-safety: Not thread-safe. Use separate instances per thread.
 */
class MotionBlurCompensation {
public:
    /**
     * @brief Constructor with default configuration
     */
    MotionBlurCompensation();

    /**
     * @brief Constructor with custom configuration
     *
     * @param config Motion blur compensation configuration
     */
    explicit MotionBlurCompensation(const MotionBlurConfig& config);

    /**
     * @brief Process frame to compensate for motion blur
     *
     * Applies bilateral filtering if enabled. If disabled, returns clone.
     *
     * @param frame Input frame (BGR or grayscale)
     * @return Processed frame (same type as input)
     *
     * @throws std::invalid_argument if frame is empty
     */
    cv::Mat process(const cv::Mat& frame);

    /**
     * @brief Set configuration
     *
     * @param config New configuration
     * @return true if configuration is valid and applied
     */
    bool set_config(const MotionBlurConfig& config);

    /**
     * @brief Get current configuration
     */
    MotionBlurConfig get_config() const;

    /**
     * @brief Enable/disable preprocessing
     *
     * @param enabled true to enable, false to disable
     */
    void set_enabled(bool enabled);

    /**
     * @brief Check if preprocessing is enabled
     */
    bool is_enabled() const;

private:
    MotionBlurConfig config_;
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_MOTION_BLUR_COMPENSATION_HPP
