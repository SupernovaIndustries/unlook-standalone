/**
 * @file GestureRecognitionSystem.hpp
 * @brief Main gesture recognition system integrating MediaPipe and ONNX Runtime
 *
 * This class provides the primary interface for hand gesture recognition,
 * combining MediaPipe for hand detection and ONNX Runtime for gesture classification.
 * Designed for real-time operation on Raspberry Pi CM5.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_RECOGNITION_SYSTEM_HPP
#define UNLOOK_GESTURE_RECOGNITION_SYSTEM_HPP

#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include <unlook/gesture/GestureTypes.hpp>

// Forward declarations
namespace unlook {
namespace camera {
    class SynchronizedCameraSystem;
}
}

namespace unlook {
namespace gesture {

// Forward declarations
class MediaPipeWrapper;
class ONNXClassifier;
class IPCBridge;

/**
 * @brief Main gesture recognition system
 *
 * Provides complete pipeline for real-time hand gesture recognition:
 * 1. Acquires frames from Unlook camera system
 * 2. Detects hands using MediaPipe (via Python wrapper)
 * 3. Classifies gestures using ONNX Runtime (C++)
 * 4. Triggers gesture callbacks for application integration
 *
 * Example usage:
 * @code
 * GestureRecognitionSystem gesture_system;
 * GestureConfig config;
 * config.min_gesture_confidence = 0.75f;
 *
 * gesture_system.initialize(camera_system, config);
 * gesture_system.set_gesture_callback([](const GestureResult& result, void* data) {
 *     if (result.type == GestureType::OPEN_PALM) {
 *         std::cout << "Opening window..." << std::endl;
 *     }
 * }, nullptr);
 *
 * gesture_system.start();
 * // ... process frames ...
 * gesture_system.stop();
 * @endcode
 */
class GestureRecognitionSystem {
public:
    /**
     * @brief Constructor
     */
    GestureRecognitionSystem();

    /**
     * @brief Destructor
     */
    ~GestureRecognitionSystem();

    // Disable copy and move
    GestureRecognitionSystem(const GestureRecognitionSystem&) = delete;
    GestureRecognitionSystem& operator=(const GestureRecognitionSystem&) = delete;
    GestureRecognitionSystem(GestureRecognitionSystem&&) = delete;
    GestureRecognitionSystem& operator=(GestureRecognitionSystem&&) = delete;

    /**
     * @brief Initialize gesture recognition system
     *
     * @param camera_system Pointer to existing Unlook camera system
     * @param config Gesture recognition configuration
     * @return true if initialization successful, false otherwise
     */
    bool initialize(std::shared_ptr<camera::SynchronizedCameraSystem> camera_system,
                   const GestureConfig& config = GestureConfig());

    /**
     * @brief Check if system is initialized
     * @return true if initialized
     */
    bool is_initialized() const;

    /**
     * @brief Start gesture recognition processing
     *
     * Starts continuous gesture detection in background thread.
     *
     * @return true if started successfully
     */
    bool start();

    /**
     * @brief Stop gesture recognition processing
     */
    void stop();

    /**
     * @brief Check if system is running
     * @return true if processing is active
     */
    bool is_running() const;

    /**
     * @brief Process single frame for gesture detection
     *
     * Synchronous processing of a single frame. Use this for manual
     * frame-by-frame processing instead of start()/stop() background mode.
     *
     * @param frame Input image (BGR or grayscale)
     * @param result Output gesture result
     * @return true if processing successful
     */
    bool process_frame(const cv::Mat& frame, GestureResult& result);

    /**
     * @brief Set gesture detection callback
     *
     * Register a callback function to be called when gestures are detected.
     * Only called in background processing mode (after start()).
     *
     * @param callback Function to call on gesture detection
     * @param user_data Optional user data pointer passed to callback
     */
    void set_gesture_callback(GestureCallback callback, void* user_data = nullptr);

    /**
     * @brief Get current configuration
     * @return Current gesture configuration
     */
    GestureConfig get_config() const;

    /**
     * @brief Update configuration
     *
     * Updates configuration. System must be stopped before updating.
     *
     * @param config New configuration
     * @return true if configuration updated successfully
     */
    bool set_config(const GestureConfig& config);

    /**
     * @brief Get performance statistics
     *
     * @param avg_detection_time_ms Average hand detection time
     * @param avg_classification_time_ms Average gesture classification time
     * @param avg_total_time_ms Average total processing time
     * @param fps Current processing frame rate
     */
    void get_performance_stats(double& avg_detection_time_ms,
                               double& avg_classification_time_ms,
                               double& avg_total_time_ms,
                               double& fps) const;

    /**
     * @brief Reset performance statistics
     */
    void reset_performance_stats();

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string get_last_error() const;

    /**
     * @brief Enable/disable debug visualization
     *
     * When enabled, draws hand landmarks and gesture labels on frames.
     *
     * @param enable true to enable, false to disable
     */
    void set_debug_visualization(bool enable);

    /**
     * @brief Get debug visualization frame
     *
     * Returns the last processed frame with debug overlays if visualization is enabled.
     *
     * @return Debug visualization frame (empty if visualization disabled)
     */
    cv::Mat get_debug_frame() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  ///< PIMPL idiom for implementation hiding
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_RECOGNITION_SYSTEM_HPP
