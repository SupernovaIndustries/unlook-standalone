/**
 * @file HandLandmarkExtractor.hpp
 * @brief Hand landmark extraction using ONNX Runtime with MediaPipe model
 *
 * Extracts 21 hand landmarks from detected hand ROI using sparse hand landmark
 * detection model (224x224 input). Based on MediaPipe hand landmark architecture.
 *
 * @copyright 2025 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_GESTURE_HAND_LANDMARK_EXTRACTOR_HPP
#define UNLOOK_GESTURE_HAND_LANDMARK_EXTRACTOR_HPP

#include <memory>
#include <string>
#include <opencv2/core.hpp>
#include "GestureTypes.hpp"

namespace unlook {
namespace gesture {

/**
 * @brief Hand landmark extractor configuration
 */
struct HandLandmarkConfig {
    std::string model_path = "model/hand_landmark/hand_landmark_sparse_Nx3x224x224.onnx";
    int input_width = 224;                ///< Model input width
    int input_height = 224;               ///< Model input height
    float confidence_threshold = 0.5f;    ///< Minimum landmark confidence
    float roi_expansion_factor = 1.5f;    ///< Factor to expand detection ROI
    bool use_gpu = false;                 ///< Enable GPU acceleration

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return input_width > 0 && input_height > 0 &&
               confidence_threshold > 0.0f && confidence_threshold <= 1.0f &&
               roi_expansion_factor > 0.0f;
    }
};

/**
 * @brief Hand landmark extractor using ONNX Runtime
 *
 * Extracts 21 hand landmarks from a hand detection ROI using sparse landmark
 * model from PINTO0309. Model input: 224x224 RGB, Output: 21x3 normalized coordinates.
 *
 * Landmark indices follow MediaPipe convention:
 * 0: Wrist
 * 1-4: Thumb (CMC, MCP, IP, TIP)
 * 5-8: Index finger (MCP, PIP, DIP, TIP)
 * 9-12: Middle finger (MCP, PIP, DIP, TIP)
 * 13-16: Ring finger (MCP, PIP, DIP, TIP)
 * 17-20: Pinky (MCP, PIP, DIP, TIP)
 *
 * Performance target: 3-5ms per hand on Raspberry Pi CM5.
 */
class HandLandmarkExtractor {
public:
    /**
     * @brief Constructor
     */
    HandLandmarkExtractor();

    /**
     * @brief Destructor
     */
    ~HandLandmarkExtractor();

    // Disable copy and move
    HandLandmarkExtractor(const HandLandmarkExtractor&) = delete;
    HandLandmarkExtractor& operator=(const HandLandmarkExtractor&) = delete;
    HandLandmarkExtractor(HandLandmarkExtractor&&) = delete;
    HandLandmarkExtractor& operator=(HandLandmarkExtractor&&) = delete;

    /**
     * @brief Initialize landmark extractor
     *
     * @param config Extractor configuration
     * @return true if initialization successful
     */
    bool initialize(const HandLandmarkConfig& config);

    /**
     * @brief Check if extractor is initialized
     */
    bool is_initialized() const;

    /**
     * @brief Extract hand landmarks from image ROI
     *
     * @param image Input image (BGR or RGB format)
     * @param hand_roi Hand detection ROI (will be expanded internally)
     * @param landmarks Output hand landmarks structure
     * @return true if extraction successful
     */
    bool extract(const cv::Mat& image, const cv::Rect& hand_roi, HandLandmarks& landmarks);

    /**
     * @brief Get last extraction time in milliseconds
     */
    double get_last_extraction_time_ms() const;

    /**
     * @brief Get current configuration
     */
    HandLandmarkConfig get_config() const;

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

#endif // UNLOOK_GESTURE_HAND_LANDMARK_EXTRACTOR_HPP
