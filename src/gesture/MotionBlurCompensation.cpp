/**
 * @file MotionBlurCompensation.cpp
 * @brief Implementation of motion blur compensation
 */

#include <unlook/gesture/MotionBlurCompensation.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

namespace unlook {
namespace gesture {

MotionBlurCompensation::MotionBlurCompensation()
    : config_() {
    // Default configuration already set in struct
}

MotionBlurCompensation::MotionBlurCompensation(const MotionBlurConfig& config)
    : config_(config) {
    if (!config_.is_valid()) {
        throw std::invalid_argument("Invalid MotionBlurConfig");
    }
}

cv::Mat MotionBlurCompensation::process(const cv::Mat& frame) {
    if (frame.empty()) {
        throw std::invalid_argument("MotionBlurCompensation::process - empty input frame");
    }

    // If disabled, return clone
    if (!config_.enabled) {
        return frame.clone();
    }

    cv::Mat output;

    // Apply bilateral filter
    // Bilateral filter:
    // - Preserves edges (important for hand contours)
    // - Reduces noise and motion blur artifacts
    // - Non-linear filter (contrast-preserving smoothing)
    cv::bilateralFilter(
        frame,
        output,
        config_.bilateral_diameter,
        config_.sigma_color,
        config_.sigma_space
    );

    return output;
}

bool MotionBlurCompensation::set_config(const MotionBlurConfig& config) {
    if (!config.is_valid()) {
        return false;
    }
    config_ = config;
    return true;
}

MotionBlurConfig MotionBlurCompensation::get_config() const {
    return config_;
}

void MotionBlurCompensation::set_enabled(bool enabled) {
    config_.enabled = enabled;
}

bool MotionBlurCompensation::is_enabled() const {
    return config_.enabled;
}

} // namespace gesture
} // namespace unlook
