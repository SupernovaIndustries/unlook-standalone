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

    // Lowered from 0.5 (50%) to 0.1 (10%) for PINTO0309 model compatibility
    // PINTO0309 landmarks may have coordinates outside [0,1] range (outside ROI bounds)
    // Old 50% threshold caused ALL landmark extractions to fail
    // 10% allows extraction if at least 3/21 landmarks are valid
    // Can be tuned upward after confirming landmark extraction works
    float confidence_threshold = 0.1f;    ///< Minimum landmark confidence

    float roi_expansion_factor = 1.5f;    ///< Factor to expand detection ROI
    bool use_gpu = false;                 ///< Enable GPU acceleration

    // CLAHE preprocessing configuration
    // DISABLED: CLAHE adds 15-20ms overhead per frame (RGB->HSV->CLAHE->HSV->RGB)
    // Total system overhead with dual CLAHE (detector + landmark): 30-40ms
    // This was causing system lag (95-100ms total, only 10 FPS possible)
    // Can be re-enabled later with optimizations if needed (e.g., grayscale CLAHE)
    bool use_clahe = false;               ///< Enable CLAHE contrast enhancement
    double clahe_clip_limit = 2.0;        ///< CLAHE contrast limiting (1.0-4.0 typical)
    int clahe_tile_grid_width = 8;        ///< CLAHE tile grid width
    int clahe_tile_grid_height = 8;       ///< CLAHE tile grid height

    /**
     * @brief Validate configuration
     */
    bool is_valid() const {
        return input_width > 0 && input_height > 0 &&
               confidence_threshold > 0.0f && confidence_threshold <= 1.0f &&
               roi_expansion_factor > 0.0f &&
               clahe_clip_limit > 0.0 &&
               clahe_tile_grid_width > 0 && clahe_tile_grid_height > 0;
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
