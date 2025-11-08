/**
 * @file TemporalFilter.hpp
 * @brief Temporal filter for multi-frame depth smoothing
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Inspired by Intel RealSense temporal filtering approach.
 */

#ifndef UNLOOK_STEREO_TEMPORAL_FILTER_HPP
#define UNLOOK_STEREO_TEMPORAL_FILTER_HPP

#include <opencv2/core.hpp>
#include <memory>
#include <deque>
#include <chrono>

namespace unlook {

namespace core {
    class Logger;
}

namespace stereo {

/**
 * @brief IIR temporal filter for multi-frame depth smoothing
 *
 * Smooths depth over time using an Infinite Impulse Response filter.
 * Detects and handles motion jumps to avoid ghosting artifacts.
 */
class TemporalFilter {
public:
    /**
     * @brief Filter configuration
     */
    struct Config {
        float alpha = 0.4f;                      ///< IIR filter weight (0-1, higher = more smoothing)
        float deltaThreshold = 20.0f;            ///< Jump detection threshold (mm)
        int persistenceFrames = 8;               ///< Frames to persist valid values
        bool motionCompensation = false;         ///< Enable motion compensation
        int historySize = 5;                     ///< Number of frames to keep in history
        bool adaptiveAlpha = true;               ///< Adapt alpha based on confidence
    };

    /**
     * @brief Filter result
     */
    struct Result {
        cv::Mat filtered;                        ///< Temporally filtered depth map
        cv::Mat motionMask;                      ///< Mask of detected motion (CV_8U)

        bool success;
        std::string errorMessage;

        // Statistics
        int motionPixels;                        ///< Number of pixels with detected motion
        float temporalConsistency;               ///< Temporal consistency score (0-1)
        int validPixels;                         ///< Number of valid depth pixels
        std::chrono::microseconds processingTime;
    };

    /**
     * @brief Frame data for temporal processing
     */
    struct FrameData {
        cv::Mat depth;                           ///< Depth map (CV_32F)
        cv::Mat confidence;                      ///< Confidence map (CV_8U)
        std::chrono::steady_clock::time_point timestamp;
        int frameNumber;
    };

    /**
     * @brief Construct temporal filter
     *
     * @param logger Logger for output messages
     */
    explicit TemporalFilter(core::Logger* logger = nullptr);

    ~TemporalFilter();

    /**
     * @brief Set configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Reset filter state (clear history)
     */
    void reset();

    /**
     * @brief Process new depth frame
     *
     * @param depthMap Current depth map (CV_32F in mm)
     * @param confidence Optional confidence map (CV_8U, 0-255)
     * @return Filtered result
     */
    Result filter(const cv::Mat& depthMap, const cv::Mat& confidence = cv::Mat());

    /**
     * @brief Get number of frames in history
     */
    size_t getHistorySize() const { return frameHistory_.size(); }

    /**
     * @brief Get temporal consistency metric
     */
    float getTemporalConsistency() const { return lastConsistency_; }

private:
    core::Logger* logger_;
    Config config_;

    // Frame history for temporal filtering
    std::deque<FrameData> frameHistory_;
    cv::Mat lastFiltered_;
    cv::Mat persistenceMap_;                    ///< Tracks how long pixels have been valid
    int frameCounter_ = 0;
    float lastConsistency_ = 0;

    /**
     * @brief Apply IIR filter
     */
    void applyIIRFilter(const cv::Mat& current, const cv::Mat& previous,
                       cv::Mat& output, float alpha);

    /**
     * @brief Detect motion/jumps between frames
     */
    void detectMotion(const cv::Mat& current, const cv::Mat& previous,
                     cv::Mat& motionMask, float threshold);

    /**
     * @brief Update persistence map
     */
    void updatePersistence(const cv::Mat& depthMap, const cv::Mat& validMask);

    /**
     * @brief Calculate adaptive alpha based on confidence
     */
    float calculateAdaptiveAlpha(float baseAlpha, uint8_t confidence);

    /**
     * @brief Calculate temporal consistency
     */
    float calculateConsistency(const cv::Mat& current, const cv::Mat& previous);
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_TEMPORAL_FILTER_HPP