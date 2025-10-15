/**
 * @file HandDetector.hpp
 * @brief Hand detection using ONNX Runtime with palm detection model
 *
 * Detects hands in images using a lightweight palm detection model (192x192).
 * Based on MediaPipe palm detection architecture converted to ONNX format.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_HAND_DETECTOR_HPP
#define UNLOOK_GESTURE_HAND_DETECTOR_HPP

#include <memory>
#include <vector>
#include <string>
#include <opencv2/core.hpp>

namespace unlook {
namespace gesture {

/**
 * @brief Detected hand bounding box with score
 */
struct HandDetection {
    cv::Rect bounding_box;       ///< Hand bounding box in image coordinates
    float score = 0.0f;          ///< Detection confidence score [0, 1]
    cv::Point2f center;          ///< Hand center point
    float rotation = 0.0f;       ///< Hand rotation angle in radians
    int track_id = -1;           ///< Tracking ID (assigned by tracker)

    /**
     * @brief Check if detection is valid
     * @return true if detection meets minimum criteria
     */
    bool is_valid() const {
        return score > 0.0f &&
               bounding_box.width > 0 &&
               bounding_box.height > 0;
    }

    /**
     * @brief Get expanded ROI for landmark extraction
     * @param expansion_factor Factor to expand bounding box (default 1.5x)
     * @param image_size Image dimensions to clamp ROI
     * @return Expanded ROI rectangle
     */
    cv::Rect get_landmark_roi(float expansion_factor, const cv::Size& image_size) const;
};

/**
 * @brief Hand detector configuration
 */
struct HandDetectorConfig {
    std::string model_path = "model/palm_detection/palm_detection_full_inf_post_192x192.onnx";
    float score_threshold = 0.6f;         ///< Minimum detection confidence
    int max_num_hands = 1;                ///< Maximum hands to detect (1-2)
    int input_width = 192;                ///< Model input width
    int input_height = 192;               ///< Model input height
    bool use_gpu = false;                 ///< Enable GPU acceleration (CUDA/TensorRT)

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return score_threshold > 0.0f && score_threshold <= 1.0f &&
               max_num_hands >= 1 && max_num_hands <= 2 &&
               input_width > 0 && input_height > 0;
    }
};

/**
 * @brief Hand detector using ONNX Runtime
 *
 * Lightweight hand detection using palm detection model from PINTO0309.
 * Model input: 192x192 RGB, Output: hand bounding boxes with scores.
 *
 * Performance target: 5-10ms per frame on Raspberry Pi CM5.
 */
class HandDetector {
public:
    /**
     * @brief Constructor
     */
    HandDetector();

    /**
     * @brief Destructor
     */
    ~HandDetector();

    // Disable copy and move
    HandDetector(const HandDetector&) = delete;
    HandDetector& operator=(const HandDetector&) = delete;
    HandDetector(HandDetector&&) = delete;
    HandDetector& operator=(HandDetector&&) = delete;

    /**
     * @brief Initialize hand detector
     *
     * @param config Detector configuration
     * @return true if initialization successful
     */
    bool initialize(const HandDetectorConfig& config);

    /**
     * @brief Check if detector is initialized
     */
    bool is_initialized() const;

    /**
     * @brief Detect hands in image
     *
     * @param image Input image (BGR or RGB format)
     * @param detections Output vector of hand detections
     * @return true if detection successful
     */
    bool detect(const cv::Mat& image, std::vector<HandDetection>& detections);

    /**
     * @brief Get last detection time in milliseconds
     */
    double get_last_detection_time_ms() const;

    /**
     * @brief Get current configuration
     */
    HandDetectorConfig get_config() const;

    /**
     * @brief Get last error message
     */
    std::string get_last_error() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;  ///< PIMPL idiom for implementation hiding
};

} // namespace gesture
} // namespace unlook

#endif // UNLOOK_GESTURE_HAND_DETECTOR_HPP
