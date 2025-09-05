#pragma once

#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <string>

/**
 * @file calibration_manager.h
 * @brief Calibration management API for stereo camera system
 */

namespace unlook {
namespace api {

/**
 * @brief Calibration target types
 */
enum class CalibrationTarget {
    CHECKERBOARD,    ///< Standard checkerboard pattern
    CHARUCO_BOARD,   ///< ChArUco board (checkerboard + ArUco markers)
    CIRCLES_GRID,    ///< Symmetric circles grid
    ASYMMETRIC_CIRCLES ///< Asymmetric circles grid
};

/**
 * @brief Calibration pattern specification
 */
struct CalibrationPattern {
    CalibrationTarget type = CalibrationTarget::CHARUCO_BOARD;
    cv::Size board_size = cv::Size(7, 10);     ///< Pattern size (corners or squares)
    float square_size_mm = 24.0f;              ///< Square size in millimeters
    float marker_size_mm = 17.0f;              ///< ArUco marker size (ChArUco only)
    int dictionary_id = 0;                     ///< ArUco dictionary ID (ChArUco only)
};

/**
 * @brief Calibration capture session parameters
 */
struct CaptureSession {
    uint32_t target_images = 50;               ///< Target number of calibration images
    uint32_t min_images = 20;                  ///< Minimum images for valid calibration
    float min_corner_distance = 50.0f;         ///< Minimum pixel distance between corners
    float max_reprojection_error = 1.0f;       ///< Maximum RMS error for acceptance
    bool auto_capture = false;                 ///< Automatic capture when pattern detected
    uint32_t capture_delay_ms = 2000;          ///< Delay between automatic captures
};

/**
 * @brief Individual calibration image data
 */
struct CalibrationImage {
    cv::Mat left_image;                        ///< Left camera image
    cv::Mat right_image;                       ///< Right camera image
    std::vector<cv::Point2f> left_corners;     ///< Detected corners in left image
    std::vector<cv::Point2f> right_corners;    ///< Detected corners in right image
    bool pattern_found_left = false;           ///< Pattern detection success (left)
    bool pattern_found_right = false;          ///< Pattern detection success (right)
    double detection_quality = 0.0;            ///< Corner detection quality score
    uint64_t timestamp_us = 0;                 ///< Capture timestamp
};

/**
 * @brief Calibration results and metrics
 */
struct CalibrationResults {
    bool success = false;                      ///< Overall calibration success
    double rms_error = 0.0;                    ///< RMS reprojection error (pixels)
    double baseline_mm = 0.0;                  ///< Stereo baseline distance (mm)
    double precision_mm = 0.0;                 ///< Theoretical precision at 100mm
    uint32_t images_used = 0;                  ///< Number of images used
    cv::Mat camera_matrix_left;                ///< Left camera intrinsic matrix
    cv::Mat camera_matrix_right;               ///< Right camera intrinsic matrix  
    cv::Mat dist_coeffs_left;                  ///< Left camera distortion coefficients
    cv::Mat dist_coeffs_right;                 ///< Right camera distortion coefficients
    cv::Mat R, T, E, F;                        ///< Stereo geometry matrices
    cv::Mat R1, R2, P1, P2, Q;                ///< Rectification matrices
};

/**
 * @brief Calibration validation metrics
 */
struct ValidationMetrics {
    double epipolar_error_avg = 0.0;           ///< Average epipolar line error (pixels)
    double epipolar_error_max = 0.0;           ///< Maximum epipolar line error (pixels)
    double rectification_quality = 0.0;        ///< Rectification quality score (0-1)
    double disparity_range = 0.0;             ///< Valid disparity range
    bool meets_precision_target = false;       ///< Meets 0.005mm precision target
};

/**
 * @brief Calibration management API
 * 
 * Provides comprehensive calibration functionality including pattern
 * detection, stereo calibration computation, validation, and export.
 * Supports multiple calibration target types and quality assessment.
 */
class CalibrationManager {
public:
    /**
     * @brief Constructor
     */
    CalibrationManager();
    
    /**
     * @brief Destructor
     */
    ~CalibrationManager();
    
    // Non-copyable, non-movable
    CalibrationManager(const CalibrationManager&) = delete;
    CalibrationManager& operator=(const CalibrationManager&) = delete;
    CalibrationManager(CalibrationManager&&) = delete;
    CalibrationManager& operator=(CalibrationManager&&) = delete;
    
    /**
     * @brief Initialize calibration manager
     * 
     * Sets up calibration algorithms and prepares for pattern detection.
     * 
     * @param pattern Calibration pattern specification
     * @return ResultCode indicating success or failure
     */
    core::ResultCode initialize(const CalibrationPattern& pattern = {});
    
    /**
     * @brief Shutdown calibration manager
     * 
     * Releases resources and clears calibration data.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode shutdown();
    
    /**
     * @brief Check if calibration manager is initialized
     * @return True if initialized and ready
     */
    bool isInitialized() const;
    
    /**
     * @brief Start new calibration session
     * 
     * Begins collecting calibration images with specified parameters.
     * 
     * @param session Capture session parameters
     * @return ResultCode indicating success or failure
     */
    core::ResultCode startCalibrationSession(const CaptureSession& session = {});
    
    /**
     * @brief Stop current calibration session
     * 
     * Ends calibration image collection and prepares for processing.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode stopCalibrationSession();
    
    /**
     * @brief Check if calibration session is active
     * @return True if session is active
     */
    bool isSessionActive() const;
    
    /**
     * @brief Add calibration image pair
     * 
     * Processes stereo image pair for calibration pattern detection
     * and adds to current session if successful.
     * 
     * @param left_image Left camera image
     * @param right_image Right camera image
     * @param force_add Add even if pattern detection fails
     * @return ResultCode indicating success or failure
     */
    core::ResultCode addCalibrationImage(const cv::Mat& left_image,
                                        const cv::Mat& right_image,
                                        bool force_add = false);
    
    /**
     * @brief Get current session progress
     * 
     * Returns progress information for active calibration session.
     * 
     * @param images_collected Number of images collected
     * @param target_images Target number of images
     * @param avg_detection_quality Average pattern detection quality
     * @return ResultCode indicating success or failure
     */
    core::ResultCode getSessionProgress(uint32_t& images_collected,
                                       uint32_t& target_images,
                                       double& avg_detection_quality) const;
    
    /**
     * @brief Compute stereo calibration
     * 
     * Processes collected calibration images to compute stereo
     * camera parameters and rectification matrices.
     * 
     * @param results Output calibration results and metrics
     * @return ResultCode indicating success or failure
     */
    core::ResultCode computeCalibration(CalibrationResults& results);
    
    /**
     * @brief Load calibration from file
     * 
     * Loads previously computed calibration parameters from file.
     * 
     * @param calibration_path Path to calibration file
     * @param results Output loaded calibration data
     * @return ResultCode indicating success or failure
     */
    core::ResultCode loadCalibration(const std::string& calibration_path,
                                    CalibrationResults& results);
    
    /**
     * @brief Save calibration to file
     * 
     * Exports calibration results to YAML file for persistence.
     * 
     * @param calibration_path Output file path
     * @param results Calibration data to save
     * @param format File format (YAML, JSON, etc.)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode saveCalibration(const std::string& calibration_path,
                                    const CalibrationResults& results,
                                    const std::string& format = "yaml");
    
    /**
     * @brief Validate calibration quality
     * 
     * Performs comprehensive validation of calibration results
     * including epipolar geometry and rectification quality.
     * 
     * @param results Calibration results to validate
     * @param metrics Output validation metrics
     * @param test_images Optional test images for validation
     * @return ResultCode indicating success or failure
     */
    core::ResultCode validateCalibration(const CalibrationResults& results,
                                        ValidationMetrics& metrics,
                                        const std::vector<CalibrationImage>* test_images = nullptr);
    
    /**
     * @brief Generate calibration report
     * 
     * Creates detailed calibration report with quality metrics,
     * validation results, and recommendations.
     * 
     * @param results Calibration results
     * @param metrics Validation metrics
     * @param output_path Report output path (HTML/PDF)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode generateCalibrationReport(const CalibrationResults& results,
                                              const ValidationMetrics& metrics,
                                              const std::string& output_path);
    
    /**
     * @brief Get calibration images
     * 
     * Returns all collected calibration images from current session.
     * 
     * @return Vector of calibration images
     */
    std::vector<CalibrationImage> getCalibrationImages() const;
    
    /**
     * @brief Remove calibration image
     * 
     * Removes specific image from current calibration session.
     * 
     * @param image_index Index of image to remove
     * @return ResultCode indicating success or failure
     */
    core::ResultCode removeCalibrationImage(uint32_t image_index);
    
    /**
     * @brief Clear all calibration images
     * 
     * Removes all images from current session.
     */
    void clearCalibrationImages();
    
    /**
     * @brief Detect calibration pattern
     * 
     * Detects calibration pattern in image pair without adding
     * to session. Useful for live preview.
     * 
     * @param left_image Left camera image
     * @param right_image Right camera image
     * @param left_corners Output detected corners (left)
     * @param right_corners Output detected corners (right)
     * @param detection_quality Output detection quality score
     * @return ResultCode indicating detection success
     */
    core::ResultCode detectPattern(const cv::Mat& left_image,
                                  const cv::Mat& right_image,
                                  std::vector<cv::Point2f>& left_corners,
                                  std::vector<cv::Point2f>& right_corners,
                                  double& detection_quality) const;
    
    /**
     * @brief Draw detected pattern
     * 
     * Overlays detected calibration pattern on images for visualization.
     * 
     * @param image Input/output image
     * @param corners Detected corner points
     * @param pattern_found Pattern detection success
     * @param color Drawing color
     * @return ResultCode indicating success or failure
     */
    core::ResultCode drawPattern(cv::Mat& image,
                                const std::vector<cv::Point2f>& corners,
                                bool pattern_found,
                                const cv::Scalar& color = cv::Scalar(0, 255, 0)) const;
    
    /**
     * @brief Get current calibration pattern
     * @return Current pattern specification
     */
    CalibrationPattern getCurrentPattern() const;
    
    /**
     * @brief Set calibration pattern
     * 
     * Updates calibration pattern specification.
     * 
     * @param pattern New pattern specification
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setCalibrationPattern(const CalibrationPattern& pattern);
    
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