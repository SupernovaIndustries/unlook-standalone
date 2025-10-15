/**
 * @file GeometricSwipeDetector.hpp
 * @brief Geometric swipe gesture detection from temporal motion data
 *
 * Detects 6 swipe gestures (LEFT, RIGHT, UP, DOWN, FORWARD, BACKWARD) using
 * pure geometric analysis of hand motion trajectories. No ML models required.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_GEOMETRIC_SWIPE_DETECTOR_HPP
#define UNLOOK_GESTURE_GEOMETRIC_SWIPE_DETECTOR_HPP

#include <memory>
#include <opencv2/core.hpp>
#include "GestureTypes.hpp"
#include "TemporalBuffer.hpp"

namespace unlook {
namespace gesture {

/**
 * @brief Swipe detection configuration
 */
struct SwipeConfig {
    // Velocity thresholds (pixels per frame)
    float min_velocity = 50.0f;              ///< Minimum average velocity for swipe
    float min_instantaneous_velocity = 30.0f;///< Minimum velocity for any single frame

    // Displacement thresholds (pixels)
    float min_displacement = 100.0f;         ///< Minimum total displacement
    float min_displacement_horizontal = 80.0f;  ///< Minimum horizontal displacement (X swipes)
    float min_displacement_vertical = 80.0f;    ///< Minimum vertical displacement (Y swipes)

    // Direction thresholds
    float direction_threshold = 0.7f;        ///< cos(angle) for direction dominance (0.7 ≈ 45°)
    float direction_purity = 0.6f;           ///< Minimum ratio of dominant axis to total motion

    // Scale change thresholds (for FORWARD/BACKWARD)
    float scale_change_threshold = 0.25f;    ///< Minimum size change ratio (25%)
    float min_scale_velocity = 0.01f;        ///< Minimum scale change per frame

    // Temporal constraints
    int min_frames = 10;                     ///< Minimum frames required for valid gesture
    int max_frames = 45;                     ///< Maximum frames (prevent slow drifts)
    double min_duration_ms = 200.0;          ///< Minimum gesture duration (milliseconds)
    double max_duration_ms = 1500.0;         ///< Maximum gesture duration (milliseconds)

    // Confidence parameters
    float confidence_boost_per_threshold = 0.1f; ///< Confidence increase per exceeded threshold

    // Debounce parameters
    int debounce_frames = 15;                ///< Frames to wait before detecting same gesture again
    double debounce_duration_ms = 500.0;     ///< Time to wait before detecting same gesture again

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return min_velocity > 0.0f &&
               min_displacement > 0.0f &&
               direction_threshold >= 0.0f && direction_threshold <= 1.0f &&
               scale_change_threshold > 0.0f &&
               min_frames > 0 && min_frames <= max_frames &&
               min_duration_ms > 0.0 && min_duration_ms <= max_duration_ms;
    }
};

/**
 * @brief Geometric swipe gesture detector
 *
 * Analyzes temporal hand motion to detect 6 swipe gesture types:
 * - SWIPE_LEFT: Dominant X movement, negative direction
 * - SWIPE_RIGHT: Dominant X movement, positive direction
 * - SWIPE_UP: Dominant Y movement, negative direction (Y axis points down)
 * - SWIPE_DOWN: Dominant Y movement, positive direction
 * - SWIPE_FORWARD: Hand size increasing (moving toward camera)
 * - SWIPE_BACKWARD: Hand size decreasing (moving away from camera)
 *
 * Detection Algorithm:
 * 1. Validate buffer has sufficient frames and duration
 * 2. Compute motion statistics (displacement, velocity, scale change)
 * 3. Check horizontal swipes (X-dominant motion)
 * 4. Check vertical swipes (Y-dominant motion)
 * 5. Check depth swipes (size change dominant)
 * 6. Calculate confidence based on threshold exceedance
 * 7. Apply debouncing to prevent rapid re-triggering
 *
 * Performance: <0.5ms per detection on Raspberry Pi CM5.
 */
class GeometricSwipeDetector {
public:
    /**
     * @brief Constructor with default configuration
     */
    GeometricSwipeDetector();

    /**
     * @brief Constructor with custom configuration
     */
    explicit GeometricSwipeDetector(const SwipeConfig& config);

    /**
     * @brief Destructor
     */
    ~GeometricSwipeDetector();

    // Disable copy and move
    GeometricSwipeDetector(const GeometricSwipeDetector&) = delete;
    GeometricSwipeDetector& operator=(const GeometricSwipeDetector&) = delete;
    GeometricSwipeDetector(GeometricSwipeDetector&&) = delete;
    GeometricSwipeDetector& operator=(GeometricSwipeDetector&&) = delete;

    /**
     * @brief Configure detector parameters
     *
     * @param config New detector configuration
     */
    void configure(const SwipeConfig& config);

    /**
     * @brief Detect swipe gesture from temporal buffer
     *
     * Analyzes motion data in buffer and detects dominant swipe gesture.
     * Returns UNKNOWN if no valid gesture detected or if debouncing active.
     *
     * @param buffer Temporal buffer containing hand tracking history
     * @return Detected gesture type (or UNKNOWN)
     */
    GestureType detect(const TemporalBuffer& buffer);

    /**
     * @brief Get confidence of last detection
     *
     * @return Confidence value [0, 1] for most recent detection
     */
    float get_confidence() const;

    /**
     * @brief Get detailed motion statistics from last detection
     *
     * @param displacement Output: total displacement vector
     * @param velocity Output: average velocity vector
     * @param scale_change Output: scale change ratio
     */
    void get_motion_stats(cv::Point2f& displacement,
                          cv::Point2f& velocity,
                          float& scale_change) const;

    /**
     * @brief Reset detector state (clears debouncing)
     */
    void reset();

    /**
     * @brief Check if detector is currently in debounce period
     *
     * @return true if debouncing (won't detect new gestures)
     */
    bool is_debouncing() const;

    /**
     * @brief Get current configuration
     */
    SwipeConfig get_config() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  ///< PIMPL idiom for implementation hiding
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_GEOMETRIC_SWIPE_DETECTOR_HPP
