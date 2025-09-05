#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace unlook {
namespace calibration {

/**
 * @brief BoofCV calibration configuration
 */
struct BoofCVConfig {
    // Calibration pattern type
    enum PatternType {
        CHECKERBOARD,
        CIRCLES_GRID,
        CHARUCO,
        CUSTOM
    };
    
    PatternType patternType = CHECKERBOARD;
    cv::Size patternSize;          // Pattern dimensions
    float squareSize;              // Square/circle size in mm
    
    // Calibration parameters
    bool assumeZeroTangentialDistortion = false;
    bool fixPrincipalPoint = false;
    bool fixAspectRatio = false;
    int radialDistortionOrder = 2;  // 2, 3, or 4 coefficients
    
    // Optimization parameters
    int maxIterations = 100;
    double convergenceTolerance = 1e-8;
    bool useRobustCost = true;      // Use robust cost function
    
    // Advanced options
    bool enableBundleAdjustment = true;
    bool refineWithSubpixel = true;
    float subpixelWindowSize = 11;
    
    // Validation
    double maxReprojectionError = 0.2;  // Maximum acceptable error in pixels
    bool removeOutliers = true;
    double outlierThreshold = 3.0;      // Standard deviations for outlier
};

/**
 * @brief BoofCV calibration results
 */
struct BoofCVCalibrationResult {
    // Camera parameters
    cv::Mat cameraMatrixLeft;
    cv::Mat cameraMatrixRight;
    cv::Mat distCoeffsLeft;
    cv::Mat distCoeffsRight;
    
    // Stereo parameters
    cv::Mat R;  // Rotation matrix
    cv::Mat T;  // Translation vector
    cv::Mat E;  // Essential matrix
    cv::Mat F;  // Fundamental matrix
    
    // Quality metrics
    double rmsError;
    double maxError;
    std::vector<double> perViewErrors;
    int inlierCount;
    int outlierCount;
    
    // Statistics
    double computationTimeMs;
    int iterationsUsed;
    bool converged;
    
    // Validation
    bool isValid() const;
    std::string getReport() const;
};

/**
 * @brief BoofCV stereo matching configuration
 */
struct BoofCVStereoConfig {
    // Algorithm selection
    enum Algorithm {
        BLOCK_MATCHING,
        SEMI_GLOBAL,
        GRAPH_CUT,
        BELIEF_PROPAGATION
    };
    
    Algorithm algorithm = SEMI_GLOBAL;
    
    // Common parameters
    int minDisparity = 0;
    int maxDisparity = 128;
    int blockRadius = 4;
    
    // Semi-global parameters
    int sgmPenaltySmall = 100;
    int sgmPenaltyLarge = 1000;
    int sgmPaths = 8;  // 4, 8, or 16 paths
    
    // Block matching parameters
    int bmErrorType = 0;  // 0=SAD, 1=Census
    int bmTextureThreshold = 10;
    
    // Post-processing
    bool enableSubpixel = true;
    bool enableLeftRightCheck = true;
    int maxDisparityDiff = 1;
    
    // Performance
    bool useParallel = true;
    int numThreads = 4;
};

/**
 * @brief BoofCV wrapper for high-precision calibration and stereo matching
 * 
 * Integrates BoofCV's industrial-grade computer vision algorithms
 * for achieving sub-millimeter precision in 3D scanning.
 */
class BoofCVWrapper {
public:
    BoofCVWrapper();
    ~BoofCVWrapper();
    
    /**
     * @brief Initialize BoofCV with Java runtime
     * @param javaPath Path to Java runtime (empty for system Java)
     * @return true if initialization successful
     */
    bool initialize(const std::string& javaPath = "");
    
    /**
     * @brief Check if BoofCV is available and initialized
     * @return true if BoofCV is ready to use
     */
    bool isAvailable() const;
    
    /**
     * @brief Perform stereo calibration using BoofCV
     * @param leftImages Vector of left calibration images
     * @param rightImages Vector of right calibration images
     * @param config Calibration configuration
     * @param result Output calibration result
     * @return true if calibration successful
     */
    bool calibrateStereo(const std::vector<cv::Mat>& leftImages,
                        const std::vector<cv::Mat>& rightImages,
                        const BoofCVConfig& config,
                        BoofCVCalibrationResult& result);
    
    /**
     * @brief Detect calibration pattern in image
     * @param image Input image
     * @param config Pattern configuration
     * @param corners Output detected corners
     * @return true if pattern detected successfully
     */
    bool detectPattern(const cv::Mat& image,
                      const BoofCVConfig& config,
                      std::vector<cv::Point2f>& corners);
    
    /**
     * @brief Compute stereo disparity using BoofCV
     * @param leftImage Rectified left image
     * @param rightImage Rectified right image
     * @param config Stereo matching configuration
     * @param disparity Output disparity map
     * @return true if disparity computed successfully
     */
    bool computeStereoDisparity(const cv::Mat& leftImage,
                               const cv::Mat& rightImage,
                               const BoofCVStereoConfig& config,
                               cv::Mat& disparity);
    
    /**
     * @brief Refine existing calibration using BoofCV
     * @param initialCalib Initial calibration parameters
     * @param leftImages Calibration images
     * @param rightImages Calibration images
     * @param config Refinement configuration
     * @param refined Output refined calibration
     * @return true if refinement successful
     */
    bool refineCalibration(const BoofCVCalibrationResult& initialCalib,
                          const std::vector<cv::Mat>& leftImages,
                          const std::vector<cv::Mat>& rightImages,
                          const BoofCVConfig& config,
                          BoofCVCalibrationResult& refined);
    
    /**
     * @brief Perform bundle adjustment on stereo calibration
     * @param calibration Input/output calibration to adjust
     * @param imagePoints Detected image points
     * @param objectPoints 3D object points
     * @return RMS error after adjustment
     */
    double bundleAdjustment(BoofCVCalibrationResult& calibration,
                          const std::vector<std::vector<cv::Point2f>>& imagePoints,
                          const std::vector<std::vector<cv::Point3f>>& objectPoints);
    
    /**
     * @brief Get BoofCV version information
     * @return Version string
     */
    std::string getVersion() const;
    
    /**
     * @brief Get supported algorithms
     * @return List of supported algorithm names
     */
    std::vector<std::string> getSupportedAlgorithms() const;
    
    /**
     * @brief Set logging level
     * @param level Logging level (0=none, 1=error, 2=warning, 3=info, 4=debug)
     */
    void setLoggingLevel(int level);
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;
    
    /**
     * @brief Benchmark BoofCV vs OpenCV
     * @param leftImage Test image
     * @param rightImage Test image
     * @return Performance comparison report
     */
    std::string benchmarkPerformance(const cv::Mat& leftImage,
                                    const cv::Mat& rightImage);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    
    // Disable copy
    BoofCVWrapper(const BoofCVWrapper&) = delete;
    BoofCVWrapper& operator=(const BoofCVWrapper&) = delete;
};

/**
 * @brief Factory for creating BoofCV-based components
 */
class BoofCVFactory {
public:
    /**
     * @brief Create BoofCV wrapper instance
     * @return Unique pointer to BoofCV wrapper
     */
    static std::unique_ptr<BoofCVWrapper> createWrapper();
    
    /**
     * @brief Check if BoofCV is available on system
     * @return true if BoofCV can be used
     */
    static bool isBoofCVAvailable();
    
    /**
     * @brief Download and setup BoofCV
     * @param installPath Path to install BoofCV
     * @return true if setup successful
     */
    static bool setupBoofCV(const std::string& installPath);
};

} // namespace calibration
} // namespace unlook