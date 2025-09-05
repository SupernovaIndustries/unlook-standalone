#pragma once

#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <functional>
#include <string>

/**
 * @file depth_processor.h
 * @brief Depth processing API for stereo matching and 3D reconstruction
 */

namespace unlook {
namespace api {

/**
 * @brief Depth processing callback function type
 * 
 * Called when depth map processing is complete.
 * 
 * @param depth_map 16-bit depth map (CV_16UC1, millimeter values)
 * @param disparity_map 8-bit disparity map (CV_8UC1, scaled disparities)
 * @param processing_time_ms Processing time in milliseconds
 * @param user_data User-provided data pointer
 */
using DepthCallback = std::function<void(const cv::Mat& depth_map,
                                       const cv::Mat& disparity_map,
                                       double processing_time_ms,
                                       void* user_data)>;

/**
 * @brief Stereo matching algorithms
 */
enum class StereoAlgorithm {
    OPENCV_SGBM,     ///< OpenCV Semi-Global Block Matching
    OPENCV_BM,       ///< OpenCV Block Matching  
    BOOFCV_SGM,      ///< BoofCV Semi-Global Matching (high precision)
    BOOFCV_BLOCK     ///< BoofCV Block Matching (fast)
};

/**
 * @brief Point cloud filtering options
 */
struct FilterOptions {
    bool remove_outliers = true;        ///< Remove statistical outliers
    bool fill_holes = false;            ///< Fill small holes in depth map
    float max_depth_mm = 1000.0f;       ///< Maximum valid depth in mm
    float min_depth_mm = 50.0f;         ///< Minimum valid depth in mm
    int median_filter_size = 0;         ///< Median filter kernel size (0=disabled)
    float outlier_std_dev = 2.0f;       ///< Standard deviations for outlier removal
};

/**
 * @brief Point cloud export options
 */
struct ExportOptions {
    core::ExportFormat format = core::ExportFormat::PLY_BINARY;
    bool include_colors = false;        ///< Include RGB colors from left image
    bool include_normals = false;       ///< Calculate and include surface normals
    float point_density = 1.0f;        ///< Point density (1.0=full, 0.5=half, etc.)
    FilterOptions filters;              ///< Filtering options to apply
};

/**
 * @brief Depth processing API for stereo reconstruction
 * 
 * Provides high-precision stereo matching, depth map generation,
 * and point cloud export capabilities. Supports both OpenCV and
 * BoofCV algorithms for different speed/precision tradeoffs.
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
    
    // Non-copyable, non-movable
    DepthProcessor(const DepthProcessor&) = delete;
    DepthProcessor& operator=(const DepthProcessor&) = delete;
    DepthProcessor(DepthProcessor&&) = delete;
    DepthProcessor& operator=(DepthProcessor&&) = delete;
    
    /**
     * @brief Initialize depth processor
     * 
     * Loads calibration data and initializes stereo algorithms.
     * 
     * @param calibration_path Path to stereo calibration file
     * @return ResultCode indicating success or failure
     */
    core::ResultCode initialize(const std::string& calibration_path = "");
    
    /**
     * @brief Shutdown depth processor
     * 
     * Releases algorithm resources and clears calibration data.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode shutdown();
    
    /**
     * @brief Check if depth processor is initialized
     * @return True if initialized with valid calibration
     */
    bool isInitialized() const;
    
    /**
     * @brief Create stereo parameter preset
     * 
     * Creates optimized stereo matching parameters for different
     * quality/speed tradeoffs.
     * 
     * @param quality Target quality level
     * @param algorithm Stereo algorithm to use
     * @return Configured stereo parameters
     */
    static core::StereoConfig createPreset(core::DepthQuality quality, 
                                          core::StereoAlgorithm algorithm);
    
    /**
     * @brief Get current stereo configuration
     * @return Current stereo matching parameters
     */
    core::StereoConfig getStereoConfig() const;
    
    /**
     * @brief Configure stereo matching parameters
     * 
     * Updates the stereo algorithm parameters for depth processing.
     * 
     * @param config New stereo configuration
     */
    void configureStereo(const core::StereoConfig& config);
    
    /**
     * @brief Process stereo frame pair synchronously
     * 
     * Processes stereo pair and returns complete depth result
     * including timing and quality metrics.
     * 
     * @param frame_pair Stereo frame pair to process
     * @return Complete depth processing result
     */
    core::DepthResult processSync(const core::StereoFramePair& frame_pair);
    
    /**
     * @brief Visualize depth map as colored image
     * 
     * Converts depth map to false-color visualization for display.
     * 
     * @param depth_map Input depth map (CV_16UC1, mm values)
     * @param min_depth Minimum depth for color mapping
     * @param max_depth Maximum depth for color mapping
     * @return Color visualization image (CV_8UC3)
     */
    cv::Mat visualizeDepthMap(const cv::Mat& depth_map, double min_depth, double max_depth) const;
    
    /**
     * @brief Export depth map to file
     * 
     * Saves depth result data to various file formats.
     * 
     * @param result Depth processing result to export
     * @param filename Output file path
     * @param format Export format ("PNG", "PLY", "OBJ", etc.)
     * @return True if export successful
     */
    bool exportDepthMap(const core::DepthResult& result, const std::string& filename, const std::string& format) const;
    
    /**
     * @brief Load stereo calibration from file
     * 
     * Loads camera matrices, distortion coefficients, and
     * rectification maps from calibration file.
     * 
     * @param calibration_path Path to calibration file
     * @return ResultCode indicating success or failure
     */
    core::ResultCode loadCalibration(const std::string& calibration_path);
    
    /**
     * @brief Check if valid calibration is loaded
     * @return True if calibration is loaded and valid
     */
    bool hasValidCalibration() const;
    
    /**
     * @brief Get calibration quality metrics
     * @return Calibration quality assessment
     */
    core::CalibrationQuality getCalibrationQuality() const;
    
    /**
     * @brief Process stereo frame pair to depth map
     * 
     * Rectifies input frames and computes depth map using
     * configured stereo algorithm.
     * 
     * @param left_frame Left camera input frame
     * @param right_frame Right camera input frame
     * @param depth_map Output depth map (CV_16UC1, mm values)
     * @param disparity_map Optional output disparity map (CV_8UC1)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode processFrames(const cv::Mat& left_frame,
                                  const cv::Mat& right_frame,
                                  cv::Mat& depth_map,
                                  cv::Mat* disparity_map = nullptr);
    
    /**
     * @brief Process frames asynchronously
     * 
     * Starts background processing of stereo frames.
     * Result delivered via depth callback when complete.
     * 
     * @param left_frame Left camera input frame
     * @param right_frame Right camera input frame
     * @param callback Completion callback function
     * @param user_data Optional user data for callback
     * @return ResultCode indicating success or failure to start
     */
    core::ResultCode processFramesAsync(const cv::Mat& left_frame,
                                       const cv::Mat& right_frame,
                                       DepthCallback callback,
                                       void* user_data = nullptr);
    
    /**
     * @brief Get rectified frames
     * 
     * Returns rectified versions of input frames using
     * loaded calibration data.
     * 
     * @param left_frame Input left frame
     * @param right_frame Input right frame
     * @param left_rectified Output rectified left frame
     * @param right_rectified Output rectified right frame
     * @return ResultCode indicating success or failure
     */
    core::ResultCode getRectifiedFrames(const cv::Mat& left_frame,
                                       const cv::Mat& right_frame,
                                       cv::Mat& left_rectified,
                                       cv::Mat& right_rectified);
    
    /**
     * @brief Convert depth map to point cloud
     * 
     * Reprojects depth map to 3D point cloud using
     * calibration parameters.
     * 
     * @param depth_map Input depth map (CV_16UC1, mm values)
     * @param points Output 3D points (CV_32FC3, XYZ in mm)
     * @param colors Optional color image for point coloring
     * @return ResultCode indicating success or failure
     */
    core::ResultCode depthToPointCloud(const cv::Mat& depth_map,
                                      cv::Mat& points,
                                      const cv::Mat* colors = nullptr);
    
    /**
     * @brief Export point cloud to file
     * 
     * Saves 3D point cloud in specified format with
     * optional filtering and post-processing.
     * 
     * @param depth_map Input depth map
     * @param output_path Output file path
     * @param options Export options and filters
     * @param colors Optional color image for point coloring
     * @return ResultCode indicating success or failure
     */
    core::ResultCode exportPointCloud(const cv::Mat& depth_map,
                                     const std::string& output_path,
                                     const ExportOptions& options = {},
                                     const cv::Mat* colors = nullptr);
    
    /**
     * @brief Set stereo algorithm
     * 
     * Selects stereo matching algorithm for depth processing.
     * 
     * @param algorithm Stereo matching algorithm
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setStereoAlgorithm(StereoAlgorithm algorithm);
    
    /**
     * @brief Get current stereo algorithm
     * @return Currently selected stereo algorithm
     */
    StereoAlgorithm getStereoAlgorithm() const;
    
    /**
     * @brief Set stereo matching parameters
     * 
     * Updates parameters for stereo matching algorithm.
     * 
     * @param params Stereo matching parameters
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setStereoParams(const core::StereoConfig& params);
    
    /**
     * @brief Get current stereo parameters
     * @return Current stereo matching parameters
     */
    core::StereoConfig getStereoParams() const;
    
    /**
     * @brief Get processing performance metrics
     * 
     * Returns timing and performance statistics for monitoring.
     * 
     * @param avg_processing_time_ms Average processing time per frame
     * @param frames_processed Total frames processed
     * @param peak_memory_mb Peak memory usage in megabytes
     * @return ResultCode indicating success or failure
     */
    core::ResultCode getPerformanceMetrics(double& avg_processing_time_ms,
                                          uint64_t& frames_processed,
                                          double& peak_memory_mb) const;
    
    /**
     * @brief Reset performance metrics
     */
    void resetPerformanceMetrics();
    
    /**
     * @brief Validate depth map quality
     * 
     * Analyzes depth map for common issues and provides
     * quality assessment metrics.
     * 
     * @param depth_map Input depth map to analyze
     * @param valid_pixels_percent Percentage of valid depth pixels
     * @param avg_depth_mm Average depth in millimeters
     * @param depth_std_dev_mm Standard deviation of depth values
     * @return ResultCode indicating success or failure
     */
    core::ResultCode validateDepthQuality(const cv::Mat& depth_map,
                                         double& valid_pixels_percent,
                                         double& avg_depth_mm,
                                         double& depth_std_dev_mm) const;
    
    /**
     * @brief Get last error message
     * @return Detailed error information
     */
    std::string getLastError() const;

private:
    // Implementation details hidden in private section
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace api
} // namespace unlook