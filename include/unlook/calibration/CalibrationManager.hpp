#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace unlook {
namespace calibration {

/**
 * @brief Stereo calibration data structure containing all calibration parameters
 * 
 * This structure holds the complete calibration data for industrial-grade
 * stereo vision with target precision of 0.005mm repeatability.
 */
struct StereoCalibrationData {
    // Image dimensions
    cv::Size imageSize;
    
    // Camera intrinsics
    cv::Mat cameraMatrixLeft;   // 3x3 camera matrix for left camera
    cv::Mat cameraMatrixRight;  // 3x3 camera matrix for right camera
    cv::Mat distCoeffsLeft;     // Distortion coefficients for left camera
    cv::Mat distCoeffsRight;    // Distortion coefficients for right camera
    
    // Stereo extrinsics
    cv::Mat R;  // 3x3 rotation matrix between cameras
    cv::Mat T;  // 3x1 translation vector between cameras
    cv::Mat E;  // 3x3 essential matrix
    cv::Mat F;  // 3x3 fundamental matrix
    
    // Rectification parameters
    cv::Mat R1, R2;    // 3x3 rectification rotation matrices
    cv::Mat P1, P2;    // 3x4 projection matrices in rectified coordinate systems
    cv::Mat Q;         // 4x4 disparity-to-depth mapping matrix
    
    // Rectification maps (computed on demand)
    cv::Mat map1Left, map2Left;
    cv::Mat map1Right, map2Right;
    
    // Quality metrics
    double rmsError;        // RMS reprojection error in pixels
    double baselineMm;      // Calibrated baseline in millimeters
    double precisionMm;     // Expected precision in millimeters
    
    // Metadata
    std::string calibrationType;
    std::string timestamp;
    
    // Validation flags
    bool isValid = false;
    bool rectificationMapsComputed = false;
    
    // Methods
    void clear();
    bool validate() const;
    double computeBaseline() const;
};

/**
 * @brief High-precision stereo calibration manager
 * 
 * Manages loading, validation, and access to stereo calibration data
 * with industrial-grade precision requirements (< 0.2 pixel RMS error).
 */
class CalibrationManager {
public:
    CalibrationManager();
    ~CalibrationManager();
    
    /**
     * @brief Load calibration from YAML file
     * @param filePath Path to calibration YAML file
     * @return true if successfully loaded and validated
     */
    bool loadCalibration(const std::string& filePath);
    
    /**
     * @brief Save calibration to YAML file
     * @param filePath Path to output YAML file
     * @return true if successfully saved
     */
    bool saveCalibration(const std::string& filePath) const;
    
    /**
     * @brief Get calibration data (thread-safe)
     * @return Const reference to calibration data
     */
    const StereoCalibrationData& getCalibrationData() const;
    
    /**
     * @brief Check if calibration is loaded and valid
     * @return true if calibration is valid
     */
    bool isCalibrationValid() const;
    
    /**
     * @brief Compute rectification maps for stereo rectification
     * @param alpha Free scaling parameter (0-1), 0=no black pixels, 1=all pixels retained
     * @return true if rectification maps computed successfully
     */
    bool computeRectificationMaps(double alpha = 0.0);
    
    /**
     * @brief Rectify stereo image pair
     * @param leftImage Input left image
     * @param rightImage Input right image
     * @param leftRectified Output rectified left image
     * @param rightRectified Output rectified right image
     * @return true if rectification successful
     */
    bool rectifyImages(const cv::Mat& leftImage, const cv::Mat& rightImage,
                      cv::Mat& leftRectified, cv::Mat& rightRectified);
    
    /**
     * @brief Get RMS reprojection error
     * @return RMS error in pixels
     */
    double getRmsError() const;
    
    /**
     * @brief Get calibrated baseline
     * @return Baseline in millimeters
     */
    double getBaselineMm() const;
    
    /**
     * @brief Get expected precision
     * @return Precision in millimeters
     */
    double getPrecisionMm() const;
    
    /**
     * @brief Validate calibration quality
     * @param maxRmsError Maximum acceptable RMS error (default 0.2 pixels)
     * @param maxBaselineDeviation Maximum baseline deviation from nominal (default 0.5mm)
     * @return true if calibration meets quality requirements
     */
    bool validateCalibrationQuality(double maxRmsError = 0.2, 
                                   double maxBaselineDeviation = 0.5) const;
    
    /**
     * @brief Get detailed validation report
     * @return String containing validation details
     */
    std::string getValidationReport() const;
    
    /**
     * @brief Convert 2D point to 3D using disparity
     * @param point 2D point in rectified image
     * @param disparity Disparity value at the point
     * @return 3D point in camera coordinate system
     */
    cv::Point3f convertTo3D(const cv::Point2f& point, float disparity) const;
    
    /**
     * @brief Compute field of view
     * @param fovX Output horizontal FOV in degrees
     * @param fovY Output vertical FOV in degrees
     */
    void computeFieldOfView(double& fovX, double& fovY) const;
    
    /**
     * @brief Get epipolar lines for validation
     * @param points Points in left image
     * @param epilines Output epipolar lines in right image
     * @param useLeft If true, compute lines for left image points
     */
    void computeEpipolarLines(const std::vector<cv::Point2f>& points,
                              std::vector<cv::Vec3f>& epilines,
                              bool useLeft = true) const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    
    // Disable copy
    CalibrationManager(const CalibrationManager&) = delete;
    CalibrationManager& operator=(const CalibrationManager&) = delete;
};

} // namespace calibration
} // namespace unlook