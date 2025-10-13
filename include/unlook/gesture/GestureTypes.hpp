/**
 * @file GestureTypes.hpp
 * @brief Core data types for gesture recognition system
 *
 * Defines fundamental types, enumerations, and structures used throughout
 * the gesture recognition module.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_TYPES_HPP
#define UNLOOK_GESTURE_TYPES_HPP

#include <array>
#include <vector>
#include <string>
#include <chrono>
#include <opencv2/core.hpp>

namespace unlook {
namespace gesture {

/**
 * @brief Supported gesture types for window/door control
 */
enum class GestureType {
    UNKNOWN = 0,        ///< Unknown or no gesture detected
    OPEN_PALM,          ///< Open palm (window open command)
    CLOSED_FIST,        ///< Closed fist (window close command)
    SWIPE_LEFT,         ///< Swipe left (close action)
    SWIPE_RIGHT,        ///< Swipe right (open action)
    POINT_UP,           ///< Point up (stop/pause command)
    POINT_DOWN,         ///< Point down (resume command)
    WAVE,               ///< Wave (cancel/dismiss)
    PINCH,              ///< Pinch gesture (fine control)
    THUMBS_UP,          ///< Thumbs up (confirm)
    THUMBS_DOWN         ///< Thumbs down (reject)
};

/**
 * @brief Hand landmark structure (21 points from MediaPipe)
 *
 * Represents the 3D positions of 21 hand landmarks detected by MediaPipe.
 * Coordinates are normalized to [0, 1] range relative to image dimensions.
 */
struct HandLandmarks {
    /// 21 landmark points (x, y, z coordinates)
    std::array<cv::Point3f, 21> points;

    /// Detection confidence [0, 1]
    float confidence = 0.0f;

    /// Handedness: true = right hand, false = left hand
    bool is_right_hand = true;

    /// Image dimensions for denormalization
    cv::Size image_size;

    /**
     * @brief Check if landmarks are valid
     * @return true if confidence is above threshold and points are within bounds
     */
    bool is_valid() const {
        return confidence > 0.5f && image_size.width > 0 && image_size.height > 0;
    }

    /**
     * @brief Get denormalized landmark position
     * @param index Landmark index [0-20]
     * @return Pixel coordinates in image space
     */
    cv::Point2f get_pixel_position(int index) const {
        if (index < 0 || index >= 21) {
            return cv::Point2f(-1, -1);
        }
        return cv::Point2f(
            points[index].x * image_size.width,
            points[index].y * image_size.height
        );
    }
};

/**
 * @brief Gesture recognition result
 */
struct GestureResult {
    /// Detected gesture type
    GestureType type = GestureType::UNKNOWN;

    /// Classification confidence [0, 1]
    float confidence = 0.0f;

    /// Hand landmarks used for classification
    HandLandmarks landmarks;

    /// Center position of the gesture in image coordinates
    cv::Point2f center_position;

    /// Gesture bounding box
    cv::Rect bounding_box;

    /// Timestamp when gesture was detected
    std::chrono::steady_clock::time_point timestamp;

    /// Processing time in milliseconds
    double processing_time_ms = 0.0;

    /**
     * @brief Check if result is valid
     * @return true if gesture was detected with sufficient confidence
     */
    bool is_valid() const {
        return type != GestureType::UNKNOWN &&
               confidence > 0.5f &&
               landmarks.is_valid();
    }

    /**
     * @brief Get human-readable gesture name
     * @return String representation of gesture type
     */
    std::string get_gesture_name() const;
};

/**
 * @brief Gesture recognition configuration
 */
struct GestureConfig {
    /// Minimum detection confidence threshold
    float min_detection_confidence = 0.7f;

    /// Minimum tracking confidence threshold
    float min_tracking_confidence = 0.5f;

    /// Minimum gesture classification confidence
    float min_gesture_confidence = 0.6f;

    /// Maximum number of hands to detect (1 or 2)
    int max_num_hands = 1;

    /// Enable temporal smoothing
    bool enable_temporal_smoothing = true;

    /// Temporal smoothing window size (frames)
    int smoothing_window_size = 5;

    /// Enable debug visualization
    bool enable_debug_viz = false;

    /// Process every Nth frame (for performance)
    int process_every_n_frames = 1;

    /**
     * @brief Validate configuration
     * @return true if configuration is valid
     */
    bool is_valid() const {
        return min_detection_confidence > 0.0f &&
               min_detection_confidence <= 1.0f &&
               min_tracking_confidence > 0.0f &&
               min_tracking_confidence <= 1.0f &&
               max_num_hands >= 1 && max_num_hands <= 2 &&
               smoothing_window_size > 0 &&
               process_every_n_frames > 0;
    }
};

/**
 * @brief Gesture event callback function type
 *
 * Signature for gesture detection callbacks. Called when a gesture is detected.
 *
 * @param result The detected gesture result
 * @param user_data Optional user data pointer passed to callback
 */
using GestureCallback = std::function<void(const GestureResult& result, void* user_data)>;

/**
 * @brief Convert GestureType enum to string
 * @param type Gesture type enum value
 * @return String representation
 */
inline std::string gesture_type_to_string(GestureType type) {
    switch (type) {
        case GestureType::UNKNOWN: return "Unknown";
        case GestureType::OPEN_PALM: return "OpenPalm";
        case GestureType::CLOSED_FIST: return "ClosedFist";
        case GestureType::SWIPE_LEFT: return "SwipeLeft";
        case GestureType::SWIPE_RIGHT: return "SwipeRight";
        case GestureType::POINT_UP: return "PointUp";
        case GestureType::POINT_DOWN: return "PointDown";
        case GestureType::WAVE: return "Wave";
        case GestureType::PINCH: return "Pinch";
        case GestureType::THUMBS_UP: return "ThumbsUp";
        case GestureType::THUMBS_DOWN: return "ThumbsDown";
        default: return "Invalid";
    }
}

inline std::string GestureResult::get_gesture_name() const {
    return gesture_type_to_string(type);
}

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_TYPES_HPP
