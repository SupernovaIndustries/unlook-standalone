#pragma once

#include "unlook/calibration/CalibrationManager.hpp"
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <sstream>

namespace unlook {
namespace validation {

/**
 * @brief Calibration validation results
 */
struct ValidationResult {
    bool passed;                    // Overall validation status
    double overallScore;            // Overall quality score (0-100)
    
    // Individual metrics
    double rmsError;                // RMS reprojection error
    double maxError;                // Maximum reprojection error
    double meanError;               // Mean reprojection error
    
    // Stereo metrics
    double epipolarError;           // Average epipolar line error
    double rectificationError;      // Rectification quality error
    double baselineError;           // Baseline measurement error
    
    // Parameter consistency
    double focalLengthConsistency;  // Focal length similarity between cameras
    double principalPointDeviation; // Principal point deviation from center
    double distortionSymmetry;      // Distortion coefficient symmetry
    
    // Coverage metrics
    double imageCoverage;           // Percentage of image area covered
    double depthCoverage;           // Depth range coverage
    
    // Individual test results
    std::map<std::string, bool> testResults;
    std::vector<std::string> warnings;
    std::vector<std::string> errors;
    
    std::string generateReport() const;
};

/**
 * @brief Epipolar geometry validation data
 */
struct EpipolarValidation {
    std::vector<cv::Point2f> leftPoints;
    std::vector<cv::Point2f> rightPoints;
    std::vector<cv::Vec3f> leftEpilines;
    std::vector<cv::Vec3f> rightEpilines;
    std::vector<float> distances;
    float meanDistance;
    float maxDistance;
    float stdDistance;
};

/**
 * @brief High-precision calibration validator
 * 
 * Validates stereo calibration quality for industrial applications
 * requiring 0.005mm precision repeatability.
 */
class CalibrationValidator {
public:
    CalibrationValidator();
    ~CalibrationValidator();
    
    /**
     * @brief Set calibration to validate
     * @param calibration Calibration manager with loaded calibration
     * @return true if calibration set successfully
     */
    bool setCalibration(std::shared_ptr<calibration::CalibrationManager> calibration);
    
    /**
     * @brief Perform comprehensive validation
     * @param result Output validation results
     * @return true if validation completed (regardless of pass/fail)
     */
    bool validateComprehensive(ValidationResult& result);
    
    /**
     * @brief Validate reprojection error
     * @param imagePoints Vector of image point pairs for validation
     * @param maxAcceptableError Maximum acceptable RMS error in pixels
     * @return true if reprojection error is acceptable
     */
    bool validateReprojectionError(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                                  double maxAcceptableError = 0.2);
    
    /**
     * @brief Validate epipolar geometry
     * @param leftPoints Points in left image
     * @param rightPoints Corresponding points in right image
     * @param validation Output validation data
     * @return true if epipolar constraints are satisfied
     */
    bool validateEpipolarGeometry(const std::vector<cv::Point2f>& leftPoints,
                                 const std::vector<cv::Point2f>& rightPoints,
                                 EpipolarValidation& validation);
    
    /**
     * @brief Validate rectification quality
     * @param leftRectified Rectified left image
     * @param rightRectified Rectified right image
     * @param maxYDisparity Maximum acceptable Y disparity in pixels
     * @return Rectification quality score (0-1)
     */
    double validateRectification(const cv::Mat& leftRectified,
                                const cv::Mat& rightRectified,
                                double maxYDisparity = 1.0);
    
    /**
     * @brief Validate baseline measurement
     * @param nominalBaselineMm Expected baseline in millimeters
     * @param toleranceMm Acceptable tolerance in millimeters
     * @return true if baseline is within tolerance
     */
    bool validateBaseline(double nominalBaselineMm,
                         double toleranceMm = 0.5);
    
    /**
     * @brief Validate camera parameters consistency
     * @return Consistency score (0-1)
     */
    double validateParameterConsistency();
    
    /**
     * @brief Validate using checkerboard pattern
     * @param leftImage Left image with checkerboard
     * @param rightImage Right image with checkerboard
     * @param patternSize Checkerboard pattern size
     * @param squareSize Square size in millimeters
     * @return Validation score (0-1)
     */
    double validateWithCheckerboard(const cv::Mat& leftImage,
                                   const cv::Mat& rightImage,
                                   const cv::Size& patternSize,
                                   float squareSize);
    
    /**
     * @brief Validate depth accuracy with known target
     * @param depthMap Computed depth map
     * @param targetDepthMm Known target depth in millimeters
     * @param roi Region of interest for target
     * @param toleranceMm Acceptable tolerance in millimeters
     * @return true if depth accuracy is within tolerance
     */
    bool validateDepthAccuracy(const cv::Mat& depthMap,
                              double targetDepthMm,
                              const cv::Rect& roi,
                              double toleranceMm = 0.5);
    
    /**
     * @brief Generate validation test images
     * @param leftImage Output left test image
     * @param rightImage Output right test image
     * @param type Test pattern type ("checkerboard", "circles", "random")
     */
    void generateTestImages(cv::Mat& leftImage,
                          cv::Mat& rightImage,
                          const std::string& type = "checkerboard");
    
    /**
     * @brief Visualize epipolar lines
     * @param leftImage Left image
     * @param rightImage Right image
     * @param points Sample points for epipolar lines
     * @param visualization Output visualization image
     */
    void visualizeEpipolarLines(const cv::Mat& leftImage,
                               const cv::Mat& rightImage,
                               const std::vector<cv::Point2f>& points,
                               cv::Mat& visualization);
    
    /**
     * @brief Visualize rectification quality
     * @param leftRectified Rectified left image
     * @param rightRectified Rectified right image
     * @param visualization Output visualization image
     */
    void visualizeRectification(const cv::Mat& leftRectified,
                               const cv::Mat& rightRectified,
                               cv::Mat& visualization);
    
    /**
     * @brief Set validation parameters
     * @param strictMode Enable strict validation for industrial applications
     * @param verboseOutput Enable detailed output
     */
    void setValidationParameters(bool strictMode = true,
                                bool verboseOutput = false);
    
    /**
     * @brief Get detailed validation report
     * @return Multi-line string with validation details
     */
    std::string getDetailedReport() const;
    
    /**
     * @brief Export validation results to file
     * @param filename Output filename
     * @param format Format ("txt", "json", "yaml")
     * @return true if export successful
     */
    bool exportResults(const std::string& filename,
                      const std::string& format = "txt") const;
    
    /**
     * @brief Compute expected precision at given depth
     * @param depthMm Depth in millimeters
     * @return Expected precision in millimeters
     */
    double computeExpectedPrecision(double depthMm) const;
    
    /**
     * @brief Validate against industrial precision requirements
     * @return true if calibration meets industrial standards
     */
    bool validateIndustrialPrecision();

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    
    // Disable copy
    CalibrationValidator(const CalibrationValidator&) = delete;
    CalibrationValidator& operator=(const CalibrationValidator&) = delete;
};

/**
 * @brief Automated calibration quality assessment
 */
class AutoCalibrationQA {
public:
    /**
     * @brief Run automated quality assessment
     * @param calibration Calibration to assess
     * @param testImages Optional test images for validation
     * @return Quality score (0-100)
     */
    static double runQualityAssessment(
        std::shared_ptr<calibration::CalibrationManager> calibration,
        const std::vector<std::pair<cv::Mat, cv::Mat>>& testImages = {});
    
    /**
     * @brief Generate calibration quality certificate
     * @param calibration Calibration to certify
     * @param certificatePath Output certificate file path
     * @return true if certificate generated successfully
     */
    static bool generateQualityCertificate(
        std::shared_ptr<calibration::CalibrationManager> calibration,
        const std::string& certificatePath);
    
    /**
     * @brief Check if calibration needs recalibration
     * @param calibration Current calibration
     * @param daysOld Age of calibration in days
     * @return true if recalibration recommended
     */
    static bool needsRecalibration(
        std::shared_ptr<calibration::CalibrationManager> calibration,
        int daysOld);
};

} // namespace validation
} // namespace unlook