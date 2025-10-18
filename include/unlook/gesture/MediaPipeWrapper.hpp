#pragma once

/**
 * @file MediaPipeWrapper.hpp
 * @brief C++ wrapper for MediaPipe Python gesture detector using pybind11
 *
 * This wrapper provides a C++ interface to the MediaPipe Python gesture
 * recognition backend, enabling seamless integration with the Unlook
 * gesture recognition system while maintaining the performance and
 * features of the official MediaPipe library.
 *
 * Architecture:
 * - C++ interface (this class) → pybind11 → Python MediaPipe → TFLite
 * - Zero-copy data transfer where possible (numpy array view of cv::Mat)
 * - Automatic Python interpreter lifecycle management
 * - Thread-safe singleton pattern for Python interpreter
 *
 * Performance:
 * - pybind11 overhead: <1-2ms (negligible)
 * - MediaPipe inference: ~65-70ms on Raspberry Pi 5
 * - Total FPS: 14-15 FPS baseline (meets requirements)
 *
 * @author Unlook Team
 * @date 2025-10-17
 */

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>

namespace unlook {
namespace gesture {

/**
 * @brief C++ wrapper for MediaPipe Python gesture detector
 *
 * Provides a C++ interface to MediaPipe gesture recognition via pybind11.
 * Handles Python interpreter lifecycle, data conversion, and error handling.
 *
 * Usage example:
 * @code
 * MediaPipeWrapper detector("/path/to/gesture_recognizer.task");
 *
 * cv::Mat frame = ...; // BGR frame from camera
 * std::string gesture;
 * std::vector<std::vector<float>> landmarks;
 * float confidence;
 *
 * if (detector.detect(frame, gesture, landmarks, confidence)) {
 *     if (!gesture.empty()) {
 *         std::cout << "Detected: " << gesture
 *                   << " (confidence: " << confidence << ")" << std::endl;
 *     }
 * }
 * @endcode
 */
class MediaPipeWrapper {
public:
    /**
     * @brief Initialize MediaPipe wrapper
     *
     * Creates Python interpreter (if not already created), imports MediaPipe
     * module, and initializes gesture recognizer.
     *
     * @param model_path Path to gesture_recognizer.task model file
     * @param num_hands Maximum number of hands to detect (default: 1)
     * @param min_detection_confidence Minimum palm detection confidence [0.0, 1.0] (default: 0.5)
     * @param min_tracking_confidence Minimum hand tracking confidence [0.0, 1.0] (default: 0.5)
     *
     * @throws std::runtime_error If model file not found or initialization fails
     */
    explicit MediaPipeWrapper(const std::string& model_path,
                              int num_hands = 1,
                              float min_detection_confidence = 0.5f,
                              float min_tracking_confidence = 0.5f);

    /**
     * @brief Destructor - cleanup Python resources
     */
    ~MediaPipeWrapper();

    /**
     * @brief Detect gesture from OpenCV frame
     *
     * Processes a single BGR frame from camera and detects hand gesture
     * and 21 hand landmarks using MediaPipe.
     *
     * @param frame BGR image from camera (cv::Mat, type CV_8UC3)
     * @param gesture_name Output gesture name (e.g., "Thumb_Up", "Victory")
     *                     Empty string if no gesture detected
     * @param landmarks Output 21 hand landmarks [[x,y,z], ...]
     *                  Normalized coordinates [0, 1] for x,y
     *                  z is depth (negative = closer to camera)
     *                  Empty vector if no hand detected
     * @param confidence Output detection confidence [0.0, 1.0]
     *                   0.0 if no gesture detected
     *
     * @return true if processing successful (even if no hand detected), false on error
     *
     * @note This function is thread-safe if Python GIL is properly handled
     * @note Frame should be 640x480 or similar resolution for best performance
     */
    bool detect(const cv::Mat& frame,
                std::string& gesture_name,
                std::vector<std::vector<float>>& landmarks,
                float& confidence);

    /**
     * @brief Get average inference time in milliseconds
     *
     * @return Average time spent in MediaPipe inference per frame
     */
    float getAvgInferenceTimeMs() const;

    /**
     * @brief Check if wrapper is initialized
     *
     * @return true if MediaPipe detector is ready to process frames
     */
    bool isInitialized() const;

    /**
     * @brief Get last error message
     *
     * @return Last error message string, empty if no error
     */
    std::string getLastError() const;

private:
    /**
     * @brief PIMPL implementation class
     *
     * Hides Python/pybind11 details from header to avoid polluting
     * compilation units that include this header with Python dependencies.
     */
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Non-copyable, non-movable (Python interpreter is singleton)
    MediaPipeWrapper(const MediaPipeWrapper&) = delete;
    MediaPipeWrapper& operator=(const MediaPipeWrapper&) = delete;
    MediaPipeWrapper(MediaPipeWrapper&&) = delete;
    MediaPipeWrapper& operator=(MediaPipeWrapper&&) = delete;
};

/**
 * @brief Helper function to convert MediaPipe gesture name to Unlook GestureType
 *
 * Maps MediaPipe pre-trained gesture names to Unlook's gesture enumeration.
 *
 * Mapping:
 * - "Open_Palm" → GestureType::PALM_OPEN
 * - "Closed_Fist" → GestureType::FIST
 * - "Thumb_Up" → GestureType::THUMB_UP
 * - "Thumb_Down" → GestureType::THUMB_DOWN
 * - "Victory" → GestureType::VICTORY
 * - "Pointing_Up" → GestureType::POINTING_UP
 * - Other gestures → GestureType::UNKNOWN
 *
 * @param mediapipe_gesture_name Gesture name from MediaPipe
 * @return Corresponding GestureType enumeration value
 */
//GestureType mapMediaPipeGesture(const std::string& mediapipe_gesture_name);

} // namespace gesture
} // namespace unlook
