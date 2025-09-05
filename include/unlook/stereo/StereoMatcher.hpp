#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace unlook {
namespace stereo {

/**
 * @brief Stereo matching algorithm types
 */
enum class StereoAlgorithm {
    SGBM,           // Semi-Global Block Matching (OpenCV)
    BM,             // Block Matching (OpenCV)
    SGBM_3WAY,      // SGBM with 3-way optimization
    BOOFCV_SGBM,    // BoofCV Semi-Global Matching
    BOOFCV_BM,      // BoofCV Block Matching
    CUSTOM          // Custom implementation
};

/**
 * @brief Stereo matching parameters structure
 * 
 * Contains all adjustable parameters for stereo matching algorithms
 * with defaults optimized for 0.005mm precision target.
 */
struct StereoMatchingParams {
    // Common parameters
    int minDisparity = 0;           // Minimum possible disparity value
    int numDisparities = 128;       // Maximum disparity minus minimum disparity
    int blockSize = 11;             // Matched block size (odd number >= 1)
    
    // SGBM specific parameters
    int P1 = 8 * 3 * blockSize * blockSize;   // Penalty for disparity change of 1
    int P2 = 32 * 3 * blockSize * blockSize;  // Penalty for disparity change > 1
    int disp12MaxDiff = 1;          // Maximum allowed difference in left-right check
    int preFilterCap = 63;          // Truncation value for prefiltered pixels
    int uniquenessRatio = 10;       // Margin in percentage by which best cost should win
    int speckleWindowSize = 100;    // Maximum size of smooth disparity regions
    int speckleRange = 32;          // Maximum disparity variation within each connected component
    int mode = 0;                   // Algorithm mode (0=SGBM, 1=HH, 2=SGBM_3WAY, 3=HH4)
    
    // Post-processing parameters
    bool useWLSFilter = true;       // Use Weighted Least Squares filter
    double wlsLambda = 8000.0;      // WLS filter lambda parameter
    double wlsSigma = 1.5;          // WLS filter sigma parameter
    
    // Validation parameters
    bool leftRightCheck = true;     // Enable left-right consistency check
    int textureThreshold = 10;      // Minimum texture for valid disparity
    
    // Performance parameters
    bool useParallel = true;        // Enable parallel processing
    int numThreads = 4;             // Number of threads for parallel processing
    
    // Quality parameters
    float confidenceThreshold = 0.95f;  // Minimum confidence for valid disparity
    
    /**
     * @brief Validate parameters
     * @return true if parameters are valid
     */
    bool validate() const;
    
    /**
     * @brief Get parameter as string for debugging
     * @return String representation of parameters
     */
    std::string toString() const;
    
    /**
     * @brief Load parameters from map
     * @param params Map of parameter name to value
     */
    void fromMap(const std::map<std::string, double>& params);
    
    /**
     * @brief Save parameters to map
     * @return Map of parameter name to value
     */
    std::map<std::string, double> toMap() const;
};

/**
 * @brief Stereo matching quality metrics
 */
struct StereoQualityMetrics {
    double avgDisparity;        // Average disparity value
    double stdDisparity;        // Standard deviation of disparity
    double validPixelRatio;     // Ratio of valid pixels
    double textureScore;        // Average texture score
    double confidenceScore;     // Average confidence score
    double processingTimeMs;    // Processing time in milliseconds
    int invalidPixelCount;      // Number of invalid pixels
    int occludedPixelCount;     // Number of occluded pixels
    
    std::string toString() const;
};

/**
 * @brief Abstract base class for stereo matching algorithms
 * 
 * Provides interface for various stereo matching implementations
 * targeting industrial-grade precision (0.005mm repeatability).
 */
class StereoMatcher {
public:
    StereoMatcher() = default;
    virtual ~StereoMatcher() = default;
    
    /**
     * @brief Compute disparity map from rectified stereo pair
     * @param leftRectified Rectified left image
     * @param rightRectified Rectified right image
     * @param disparity Output disparity map (CV_16S or CV_32F)
     * @return true if computation successful
     */
    virtual bool computeDisparity(const cv::Mat& leftRectified,
                                 const cv::Mat& rightRectified,
                                 cv::Mat& disparity) = 0;
    
    /**
     * @brief Compute disparity with confidence map
     * @param leftRectified Rectified left image
     * @param rightRectified Rectified right image
     * @param disparity Output disparity map
     * @param confidence Output confidence map (0-1 range)
     * @return true if computation successful
     */
    virtual bool computeDisparityWithConfidence(const cv::Mat& leftRectified,
                                               const cv::Mat& rightRectified,
                                               cv::Mat& disparity,
                                               cv::Mat& confidence);
    
    /**
     * @brief Set stereo matching parameters
     * @param params Stereo matching parameters
     * @return true if parameters set successfully
     */
    virtual bool setParameters(const StereoMatchingParams& params) = 0;
    
    /**
     * @brief Get current stereo matching parameters
     * @return Current parameters
     */
    virtual StereoMatchingParams getParameters() const = 0;
    
    /**
     * @brief Update single parameter
     * @param name Parameter name
     * @param value Parameter value
     * @return true if parameter updated successfully
     */
    virtual bool updateParameter(const std::string& name, double value);
    
    /**
     * @brief Get algorithm type
     * @return Algorithm type enum
     */
    virtual StereoAlgorithm getAlgorithmType() const = 0;
    
    /**
     * @brief Get algorithm name
     * @return Human-readable algorithm name
     */
    virtual std::string getAlgorithmName() const = 0;
    
    /**
     * @brief Compute quality metrics for disparity map
     * @param disparity Disparity map to evaluate
     * @param metrics Output quality metrics
     * @return true if metrics computed successfully
     */
    virtual bool computeQualityMetrics(const cv::Mat& disparity,
                                      StereoQualityMetrics& metrics) const;
    
    /**
     * @brief Apply post-processing to disparity map
     * @param disparity Input/output disparity map
     * @param leftImage Original left image for guided filtering
     * @return true if post-processing successful
     */
    virtual bool applyPostProcessing(cv::Mat& disparity,
                                    const cv::Mat& leftImage);
    
    /**
     * @brief Validate disparity map
     * @param disparity Disparity map to validate
     * @param validMask Output mask of valid pixels
     * @return Ratio of valid pixels (0-1)
     */
    virtual double validateDisparity(const cv::Mat& disparity,
                                    cv::Mat& validMask) const;
    
    /**
     * @brief Convert disparity to depth map
     * @param disparity Input disparity map
     * @param Q 4x4 reprojection matrix from calibration
     * @param depth Output depth map in millimeters
     * @param handleMissingValues If true, interpolate missing values
     * @return true if conversion successful
     */
    static bool disparityToDepth(const cv::Mat& disparity,
                                const cv::Mat& Q,
                                cv::Mat& depth,
                                bool handleMissingValues = true);
    
    /**
     * @brief Create stereo matcher instance
     * @param algorithm Algorithm type to create
     * @return Unique pointer to stereo matcher instance
     */
    static std::unique_ptr<StereoMatcher> create(StereoAlgorithm algorithm);
    
    /**
     * @brief Get list of available algorithms
     * @return Vector of available algorithm types
     */
    static std::vector<StereoAlgorithm> getAvailableAlgorithms();
    
protected:
    StereoMatchingParams params_;
    
    /**
     * @brief Normalize disparity map for visualization
     * @param disparity Input disparity map
     * @param normalized Output normalized map (0-255)
     */
    void normalizeDisparity(const cv::Mat& disparity, cv::Mat& normalized) const;
    
    /**
     * @brief Check if stereo pair is valid
     * @param left Left image
     * @param right Right image
     * @return true if pair is valid for matching
     */
    bool validateStereoPair(const cv::Mat& left, const cv::Mat& right) const;
};

/**
 * @brief Factory for creating stereo matcher instances
 */
class StereoMatcherFactory {
public:
    /**
     * @brief Register a stereo matcher creator
     * @param algorithm Algorithm type
     * @param creator Function to create matcher instance
     */
    static void registerMatcher(StereoAlgorithm algorithm,
                               std::function<std::unique_ptr<StereoMatcher>()> creator);
    
    /**
     * @brief Create stereo matcher instance
     * @param algorithm Algorithm type to create
     * @return Unique pointer to stereo matcher instance
     */
    static std::unique_ptr<StereoMatcher> create(StereoAlgorithm algorithm);
    
private:
    static std::map<StereoAlgorithm, std::function<std::unique_ptr<StereoMatcher>()>> creators_;
};

} // namespace stereo
} // namespace unlook