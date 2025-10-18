/**
 * @file TemporalBuffer.hpp
 * @brief Circular buffer for temporal hand tracking data
 *
 * Stores recent hand tracking frames for temporal gesture analysis.
 * Optimized for efficient swipe detection using motion vectors and size changes.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_TEMPORAL_BUFFER_HPP
#define UNLOOK_GESTURE_TEMPORAL_BUFFER_HPP

#include <memory>
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "HandTracker.hpp"

namespace unlook {
namespace gesture {

/**
 * @brief Single frame of hand tracking data
 *
 * Snapshot of tracked hand state at a specific moment in time.
 */
struct HandFrame {
    cv::Point2f position;                           ///< Hand centroid position
    cv::Point2f velocity;                           ///< Instantaneous velocity
    cv::Size2f size;                                ///< Hand bounding box size
    float confidence = 0.0f;                        ///< Tracking confidence
    std::chrono::steady_clock::time_point timestamp;///< Frame timestamp
    int track_id = -1;                              ///< Associated track ID

    /**
     * @brief Default constructor
     */
    HandFrame() = default;

    /**
     * @brief Construct from tracked hand
     */
    explicit HandFrame(const TrackedHand& hand)
        : position(hand.position)
        , velocity(hand.velocity)
        , size(hand.size)
        , confidence(hand.confidence)
        , timestamp(hand.timestamp)
        , track_id(hand.track_id)
    {}
};

/**
 * @brief Circular buffer for temporal hand tracking
 *
 * Fixed-size circular buffer storing recent hand tracking frames.
 * Optimized for gesture recognition requiring temporal motion analysis.
 *
 * Typical usage:
 * - 30 frames capacity at 30 FPS = 1 second of history
 * - Swipe detection analyzes motion over ~0.5-1 second
 * - Buffer fills before gesture analysis begins
 *
 * Frame Loss Tolerance:
 * - Allows 1-2 gap frames (hand temporarily lost) without clearing buffer
 * - Only resets if hand lost for >max_gap_frames consecutive frames
 * - Essential for fast swipe detection where MediaPipe may lose tracking briefly
 *
 * Thread-safety: Not thread-safe. External synchronization required.
 *
 * Performance: O(1) push, O(N) query operations.
 */
class TemporalBuffer {
public:
    /**
     * @brief Constructor with capacity
     *
     * @param capacity Maximum number of frames to store (default: 30)
     * @param max_gap_frames Maximum consecutive frames without detection before clearing (default: 2)
     */
    explicit TemporalBuffer(size_t capacity = 30, size_t max_gap_frames = 2);

    /**
     * @brief Destructor
     */
    ~TemporalBuffer();

    // Disable copy, allow move
    TemporalBuffer(const TemporalBuffer&) = delete;
    TemporalBuffer& operator=(const TemporalBuffer&) = delete;
    TemporalBuffer(TemporalBuffer&&) noexcept;
    TemporalBuffer& operator=(TemporalBuffer&&) noexcept;

    /**
     * @brief Push new hand frame to buffer
     *
     * If buffer is full, oldest frame is removed (FIFO).
     * If hand.is_active is false, increments gap counter but keeps buffer.
     * Only clears buffer if gaps exceed max_gap_frames.
     *
     * @param hand Tracked hand to add
     */
    void push(const TrackedHand& hand);

    /**
     * @brief Push hand frame directly
     *
     * @param frame Hand frame to add
     */
    void push(const HandFrame& frame);

    /**
     * @brief Get current buffer size
     *
     * @return Number of frames currently stored
     */
    size_t size() const;

    /**
     * @brief Check if buffer is full
     *
     * @return true if buffer contains capacity frames
     */
    bool is_full() const;

    /**
     * @brief Check if buffer is empty
     */
    bool is_empty() const;

    /**
     * @brief Clear all frames
     */
    void clear();

    /**
     * @brief Get buffer capacity
     */
    size_t capacity() const;

    /**
     * @brief Get recent frames (newest first)
     *
     * @param count Number of frames to retrieve (0 = all)
     * @return Vector of hand frames in reverse chronological order
     */
    std::vector<HandFrame> get_recent_frames(size_t count = 0) const;

    /**
     * @brief Get oldest frame in buffer
     *
     * @return Oldest hand frame (throws if empty)
     */
    HandFrame get_oldest_frame() const;

    /**
     * @brief Get newest frame in buffer
     *
     * @return Most recent hand frame (throws if empty)
     */
    HandFrame get_newest_frame() const;

    /**
     * @brief Get frame at specific index
     *
     * @param index Frame index (0 = oldest, size()-1 = newest)
     * @return Hand frame at index (throws if out of range)
     */
    HandFrame get_frame_at(size_t index) const;

    // ===== Motion Analysis Helpers =====

    /**
     * @brief Compute average velocity over all frames
     *
     * @return Average velocity vector (pixels/frame)
     */
    cv::Point2f compute_average_velocity() const;

    /**
     * @brief Compute total displacement from oldest to newest frame
     *
     * @return Total displacement magnitude (pixels)
     */
    float compute_total_displacement() const;

    /**
     * @brief Compute displacement vector from oldest to newest frame
     *
     * @return Displacement vector (newest - oldest position)
     */
    cv::Point2f compute_displacement_vector() const;

    /**
     * @brief Compute scale change ratio (size change)
     *
     * @return Scale change factor: (newest_size - oldest_size) / oldest_size
     *         Positive = hand growing (moving toward camera)
     *         Negative = hand shrinking (moving away from camera)
     */
    float compute_scale_change() const;

    /**
     * @brief Compute average hand size over all frames
     *
     * @return Average size (width and height)
     */
    cv::Size2f compute_average_size() const;

    /**
     * @brief Compute time duration covered by buffer
     *
     * @return Duration in milliseconds from oldest to newest frame
     */
    double compute_duration_ms() const;

    /**
     * @brief Check if buffer has minimum number of frames
     *
     * @param min_frames Minimum required frames
     * @return true if size() >= min_frames
     */
    bool has_minimum_frames(size_t min_frames) const;

    /**
     * @brief Get current consecutive gap frames count
     *
     * @return Number of consecutive frames without hand detection
     */
    size_t get_gap_frames() const;

    /**
     * @brief Get maximum allowed gap frames
     *
     * @return Maximum consecutive gap frames before buffer reset
     */
    size_t get_max_gap_frames() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  ///< PIMPL idiom for implementation hiding
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_TEMPORAL_BUFFER_HPP
