#pragma once

#include "unlook/core/types.hpp"
#include <memory>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace unlook {
namespace stereo {

/**
 * @brief High-performance stereo depth processing system
 * 
 * This class handles stereo matching and depth map generation using
 * multiple algorithms (OpenCV SGBM, BoofCV integration).
 * Thread-safe for real-time processing.
 */
class DepthProcessor {
public:
    /**
     * @brief Constructor
     */
    DepthProcessor();
    
    /**
     * @brief Destructor
     */
    ~DepthProcessor();

    // Non-copyable, non-movable for thread safety
    DepthProcessor(const DepthProcessor&) = delete;
    DepthProcessor& operator=(const DepthProcessor&) = delete;
    DepthProcessor(DepthProcessor&&) = delete;
    DepthProcessor& operator=(DepthProcessor&&) = delete;

    /**
     * @brief Initialize the depth processor
     * @param calibration_file Path to camera calibration YAML file
     * @return true if successful, false otherwise
     */
    bool initialize(const std::string& calibration_file);
    
    /**
     * @brief Load camera calibration from file
     */
    bool loadCalibration(const std::string& calibration_file);
    
    /**
     * @brief Configure stereo processing parameters
     */
    void configureStereo(const core::StereoConfig& config);
    
    /**
     * @brief Get current stereo configuration
     */
    core::StereoConfig getStereoConfig() const;
    
    /**
     * @brief Process stereo frame pair synchronously
     * @param stereo_pair Input stereo frame pair
     * @return Depth processing result
     */
    core::DepthResult processSync(const core::StereoFramePair& stereo_pair);
    
    /**
     * @brief Start asynchronous processing
     * @param result_callback Callback for depth results
     */
    bool startAsync(core::DepthResultCallback result_callback);
    
    /**
     * @brief Stop asynchronous processing
     */
    void stopAsync();
    
    /**
     * @brief Queue stereo frame pair for asynchronous processing
     * @param stereo_pair Input stereo frame pair
     * @return true if queued successfully, false if queue is full
     */
    bool queueFramePair(const core::StereoFramePair& stereo_pair);
    
    /**
     * @brief Get processing queue size
     */
    size_t getQueueSize() const;
    
    /**
     * @brief Get processing performance statistics
     */
    struct ProcessingStats {
        double average_processing_time_ms;
        double max_processing_time_ms;
        double min_processing_time_ms;
        size_t frames_processed;
        size_t frames_dropped;
        double fps;
    };
    
    ProcessingStats getProcessingStats() const;
    
    /**
     * @brief Reset processing statistics
     */
    void resetStats();
    
    /**
     * @brief Check if calibration is loaded
     */
    bool isCalibrationLoaded() const;
    
    /**
     * @brief Get rectification maps for preview
     */
    bool getRectificationMaps(cv::Mat& left_map1, cv::Mat& left_map2,
                             cv::Mat& right_map1, cv::Mat& right_map2) const;
    
    /**
     * @brief Apply rectification to images (for preview)
     */
    void rectifyImages(const cv::Mat& left_raw, const cv::Mat& right_raw,
                      cv::Mat& left_rect, cv::Mat& right_rect) const;
    
    /**
     * @brief Get depth map color visualization
     */
    cv::Mat visualizeDepthMap(const cv::Mat& depth_map, 
                             double min_depth = 0.0, double max_depth = 1000.0) const;
    
    /**
     * @brief Export depth map to file
     */
    bool exportDepthMap(const core::DepthResult& result, 
                       const std::string& filename, 
                       const std::string& format = "PLY") const;
    
    /**
     * @brief Create algorithm-specific parameter presets
     */
    static core::StereoConfig createPreset(core::DepthQuality quality, 
                                          core::StereoAlgorithm algorithm);

private:
    /**
     * @brief Asynchronous processing thread
     */
    void processingThread();
    
    /**
     * @brief Internal depth processing implementation
     */
    core::DepthResult processInternal(const core::StereoFramePair& stereo_pair);
    
    /**
     * @brief Process using OpenCV SGBM
     */
    core::DepthResult processOpenCVSGBM(const cv::Mat& left, const cv::Mat& right);
    
    /**
     * @brief Process using BoofCV (future implementation)
     */
    core::DepthResult processBoofCV(const cv::Mat& left, const cv::Mat& right);
    
    /**
     * @brief Update processing statistics
     */
    void updateStats(double processing_time_ms);
    
    // Configuration
    core::StereoConfig stereo_config_;
    mutable std::mutex config_mutex_;
    
    // Calibration data
    cv::Mat camera_matrix_left_, camera_matrix_right_;
    cv::Mat dist_coeffs_left_, dist_coeffs_right_;
    cv::Mat rotation_matrix_, translation_vector_;
    cv::Mat rectification_left_, rectification_right_;
    cv::Mat projection_left_, projection_right_;
    cv::Mat disparity_to_depth_map_;
    cv::Mat rectification_map_left_1_, rectification_map_left_2_;
    cv::Mat rectification_map_right_1_, rectification_map_right_2_;
    bool calibration_loaded_;
    mutable std::mutex calibration_mutex_;
    
    // OpenCV stereo matcher
    cv::Ptr<cv::StereoSGBM> sgbm_matcher_;
    
    // Async processing
    std::unique_ptr<std::thread> processing_thread_;
    std::atomic<bool> processing_running_;
    std::queue<core::StereoFramePair> processing_queue_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_condition_;
    static const size_t MAX_QUEUE_SIZE = 10;
    
    // Callback
    core::DepthResultCallback result_callback_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    ProcessingStats stats_;
    std::chrono::high_resolution_clock::time_point stats_start_time_;
};

} // namespace stereo
} // namespace unlook