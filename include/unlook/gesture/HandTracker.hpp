/**
 * @file HandTracker.hpp
 * @brief Hand tracking with Kalman filtering and occlusion handling
 *
 * Tracks detected hands across frames using Kalman filter for smooth trajectories.
 * Handles temporary occlusions and maintains track IDs for gesture recognition.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_HAND_TRACKER_HPP
#define UNLOOK_GESTURE_HAND_TRACKER_HPP

#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include "GestureTypes.hpp"

namespace unlook {
namespace gesture {

/**
 * @brief Tracked hand state
 *
 * Represents a hand being tracked across multiple frames with smoothed
 * position, velocity, and landmark information.
 */
struct TrackedHand {
    int track_id = -1;                              ///< Unique tracking ID
    cv::Point2f position;                           ///< Smoothed centroid position
    cv::Point2f velocity;                           ///< Velocity vector (pixels/frame)
    cv::Size2f size;                                ///< Smoothed bounding box size
    HandLandmarks landmarks;                        ///< Latest detected landmarks
    float confidence = 0.0f;                        ///< Current tracking confidence
    int frames_since_detection = 0;                 ///< Frames without detection (occlusion)
    bool is_active = false;                         ///< Track is currently active
    std::chrono::steady_clock::time_point timestamp;///< Last update timestamp

    /**
     * @brief Check if track is valid and active
     */
    bool is_valid() const {
        return is_active && confidence > 0.3f && track_id >= 0;
    }

    /**
     * @brief Get bounding box from position and size
     */
    cv::Rect get_bounding_box() const {
        return cv::Rect(
            static_cast<int>(position.x - size.width / 2.0f),
            static_cast<int>(position.y - size.height / 2.0f),
            static_cast<int>(size.width),
            static_cast<int>(size.height)
        );
    }
};

/**
 * @brief Hand tracker configuration
 */
struct HandTrackerConfig {
    int max_missed_frames = 5;                ///< Max frames to keep track without detection
    float position_noise = 0.1f;              ///< Process noise for position (higher = more responsive)
    float velocity_noise = 0.5f;              ///< Process noise for velocity
    float measurement_noise = 1.0f;           ///< Measurement noise (higher = more smoothing)
    float max_association_distance = 50.0f;   ///< Max distance (pixels) for detection-track matching
    float min_confidence_threshold = 0.3f;    ///< Minimum confidence to maintain track

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return max_missed_frames > 0 &&
               position_noise > 0.0f &&
               velocity_noise > 0.0f &&
               measurement_noise > 0.0f &&
               max_association_distance > 0.0f;
    }
};

/**
 * @brief Hand tracker using Kalman filtering
 *
 * Tracks hands across frames using Kalman filter for position and velocity prediction.
 * Maintains track IDs and handles temporary occlusions gracefully.
 *
 * Features:
 * - Kalman filtering for smooth trajectories
 * - Occlusion handling (continues tracking for N frames)
 * - Detection-to-track association (nearest neighbor)
 * - Velocity estimation for swipe detection
 * - Automatic track lifecycle management
 *
 * Performance: <1ms per frame for single hand tracking.
 */
class HandTracker {
public:
    /**
     * @brief Constructor with default configuration
     */
    HandTracker();

    /**
     * @brief Constructor with custom configuration
     */
    explicit HandTracker(const HandTrackerConfig& config);

    /**
     * @brief Destructor
     */
    ~HandTracker();

    // Disable copy and move
    HandTracker(const HandTracker&) = delete;
    HandTracker& operator=(const HandTracker&) = delete;
    HandTracker(HandTracker&&) = delete;
    HandTracker& operator=(HandTracker&&) = delete;

    /**
     * @brief Configure tracker parameters
     *
     * @param config New tracker configuration
     */
    void configure(const HandTrackerConfig& config);

    /**
     * @brief Update tracker with new detections
     *
     * Associates detections with existing tracks, updates Kalman filters,
     * creates new tracks, and removes stale tracks.
     *
     * @param detections Hand detection bounding boxes
     * @param landmarks Corresponding hand landmarks (same size as detections)
     */
    void update(const std::vector<cv::Rect>& detections,
                const std::vector<HandLandmarks>& landmarks);

    /**
     * @brief Get currently active tracks
     *
     * @return Vector of active tracked hands
     */
    std::vector<TrackedHand> get_active_tracks() const;

    /**
     * @brief Get single primary hand (highest confidence)
     *
     * @return Primary tracked hand, or empty TrackedHand if none active
     */
    TrackedHand get_primary_hand() const;

    /**
     * @brief Clear all tracks
     */
    void clear();

    /**
     * @brief Get number of active tracks
     */
    size_t get_track_count() const;

    /**
     * @brief Get current configuration
     */
    HandTrackerConfig get_config() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  ///< PIMPL idiom for implementation hiding
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_HAND_TRACKER_HPP
