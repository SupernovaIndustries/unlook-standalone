/**
 * @file BoofCVStereoMatcher.hpp
 * @brief BoofCV-based stereo matching with sub-pixel precision for industrial 3D scanning
 * 
 * High-precision stereo matching using BoofCV algorithms via JNI.
 * Target precision: ≤ 0.005mm for Unlook 3D Scanner industrial applications.
 * 
 * @author Unlook 3D Scanner Team
 * @date 2025-01-15
 */

#pragma once

// Only compile BoofCV support if JNI is available
#ifdef HAVE_BOOFCV

#include <opencv2/opencv.hpp>
#include <jni.h>
#include <memory>
#include <string>
#include "StereoMatcher.hpp"

namespace unlook {
namespace stereo {

/**
 * @brief BoofCV stereo matching algorithm types
 */
enum class BoofCVAlgorithm {
    DENSE_DISPARITY_BM,      ///< Block matching with dense disparity
    DENSE_DISPARITY_SGM,     ///< Semi-global matching dense disparity  
    SUBPIXEL_BM,             ///< Block matching with sub-pixel accuracy
    SUBPIXEL_SGM             ///< Semi-global matching with sub-pixel accuracy
};

/**
 * @brief BoofCV error metric types for stereo matching
 */
enum class BoofCVErrorType {
    CENSUS,                  ///< Census transform (best for textureless regions)
    NORMALIZED_CROSS_CORRELATION, ///< NCC (good for lighting variations)
    SAD,                     ///< Sum of Absolute Differences
    SSD                      ///< Sum of Squared Differences
};

/**
 * @brief Configuration for BoofCV stereo matching
 */
struct BoofCVStereoConfig {
    BoofCVAlgorithm algorithm = BoofCVAlgorithm::SUBPIXEL_SGM;
    BoofCVErrorType error_type = BoofCVErrorType::CENSUS;
    
    // Core parameters
    int min_disparity = 0;          ///< Minimum disparity value
    int max_disparity = 128;        ///< Maximum disparity value
    int region_radius = 3;          ///< Matching region radius (3-7 optimal)
    
    // Quality parameters
    bool subpixel_enabled = true;   ///< Enable sub-pixel precision
    bool left_right_validation = true; ///< Enable left-right consistency check
    double max_error = 0.15;        ///< Maximum per-pixel error threshold
    double texture_threshold = 0.1; ///< Texture threshold for valid matches
    
    // Performance parameters
    int num_threads = 4;            ///< Number of processing threads
    bool use_gpu_acceleration = false; ///< GPU acceleration (if available)
    
    // Industrial precision settings
    double baseline_mm = 70.017;    ///< Camera baseline in mm (from calibration)
    double focal_length_px = 1200.0; ///< Focal length in pixels
    bool enforce_precision_target = true; ///< Enforce 0.005mm precision target
};

/**
 * @brief Quality metrics for BoofCV stereo matching results
 */
struct BoofCVQualityMetrics {
    double valid_pixel_percentage = 0.0;  ///< Percentage of valid disparity pixels
    double mean_error = 0.0;              ///< Mean matching error
    double rms_error = 0.0;               ///< RMS error
    double precision_mm = 0.0;            ///< Estimated precision in mm
    double processing_time_ms = 0.0;      ///< Processing time in milliseconds
    bool meets_precision_target = false;  ///< True if meets 0.005mm target
};

/**
 * @brief High-precision stereo matcher using BoofCV algorithms
 * 
 * This class provides a C++ interface to BoofCV's advanced stereo vision
 * algorithms through JNI. Designed for industrial applications requiring
 * sub-millimeter precision.
 * 
 * Features:
 * - Sub-pixel precision stereo matching
 * - CENSUS transform for robust matching
 * - Left-right consistency validation
 * - Industrial-grade quality metrics
 * - Target precision: ≤ 0.005mm
 * 
 * @threadsafe This class is not thread-safe. Use separate instances for concurrent processing.
 */
class BoofCVStereoMatcher : public StereoMatcher {
public:
    /**
     * @brief Constructor with configuration
     * @param config BoofCV stereo matching configuration
     * @throws std::runtime_error if JVM initialization fails
     */
    explicit BoofCVStereoMatcher(const BoofCVStereoConfig& config = BoofCVStereoConfig{});
    
    /**
     * @brief Destructor - cleans up JVM resources
     */
    ~BoofCVStereoMatcher();
    
    // Non-copyable but movable
    BoofCVStereoMatcher(const BoofCVStereoMatcher&) = delete;
    BoofCVStereoMatcher& operator=(const BoofCVStereoMatcher&) = delete;
    BoofCVStereoMatcher(BoofCVStereoMatcher&&) noexcept;
    BoofCVStereoMatcher& operator=(BoofCVStereoMatcher&&) noexcept;
    
    /**
     * @brief Compute disparity map from rectified stereo images
     * @param left_image Left rectified image (CV_8UC1 or CV_8UC3)
     * @param right_image Right rectified image (CV_8UC1 or CV_8UC3)
     * @param disparity_map Output disparity map (CV_32F)
     * @return Quality metrics for the computed disparity
     * @throws std::invalid_argument if images have different sizes or invalid format
     * @throws std::runtime_error if BoofCV processing fails
     */
    BoofCVQualityMetrics computeDisparity(
        const cv::Mat& left_image,
        const cv::Mat& right_image,
        cv::Mat& disparity_map
    );
    
    /**
     * @brief Compute high-precision disparity with sub-pixel accuracy
     * @param left_image Left rectified image
     * @param right_image Right rectified image
     * @param disparity_map Output disparity map with sub-pixel precision (CV_32F)
     * @return Quality metrics including precision estimation
     */
    BoofCVQualityMetrics computeSubpixelDisparity(
        const cv::Mat& left_image,
        const cv::Mat& right_image,
        cv::Mat& disparity_map
    );
    
    /**
     * @brief Update stereo matching configuration
     * @param config New configuration
     * @throws std::runtime_error if configuration is invalid
     */
    void updateConfig(const BoofCVStereoConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current stereo matching configuration
     */
    const BoofCVStereoConfig& getConfig() const { return config_; }
    
    /**
     * @brief Check if BoofCV JNI is available and working
     * @return true if BoofCV is available
     */
    static bool isAvailable();
    
    /**
     * @brief Get BoofCV version string
     * @return BoofCV version
     */
    static std::string getBoofCVVersion();
    
    /**
     * @brief Validate stereo images for processing
     * @param left_image Left image
     * @param right_image Right image
     * @throws std::invalid_argument if images are invalid
     */
    static void validateStereoImages(const cv::Mat& left_image, const cv::Mat& right_image);

    // StereoMatcher interface implementation
    
    /**
     * @brief Compute disparity map (StereoMatcher interface)
     * @param leftRectified Left rectified image
     * @param rightRectified Right rectified image
     * @param disparity Output disparity map
     * @return true if successful
     */
    bool computeDisparity(const cv::Mat& leftRectified,
                         const cv::Mat& rightRectified,
                         cv::Mat& disparity) override;
    
    /**
     * @brief Set stereo matching parameters (StereoMatcher interface)
     * @param params Stereo matching parameters
     * @return true if successful
     */
    bool setParameters(const StereoMatchingParams& params) override;
    
    /**
     * @brief Get current stereo matching parameters
     * @return Current parameters
     */
    StereoMatchingParams getParameters() const override;
    
    /**
     * @brief Get algorithm type
     * @return BoofCV algorithm type
     */
    StereoAlgorithm getAlgorithmType() const override;
    
    /**
     * @brief Get algorithm name as string
     * @return Algorithm name
     */
    std::string getAlgorithmName() const override;

private:
    BoofCVStereoConfig config_;
    
    // JNI components
    JavaVM* jvm_;
    JNIEnv* env_;
    jobject boofcv_matcher_;  // BoofCV stereo matcher Java object
    
    // Java method IDs (cached for performance)
    jmethodID compute_disparity_method_;
    jmethodID compute_subpixel_method_;
    jmethodID update_config_method_;
    
    // Internal methods
    void initializeJVM();
    void cleanupJVM();
    void createBoofCVMatcher();
    void cacheMethodIDs();
    
    // Helper methods for JNI conversion
    jobject matToBoofCVImage(const cv::Mat& mat);
    cv::Mat boofCVImageToMat(jobject boofcv_image);
    jobject configToJavaObject(const BoofCVStereoConfig& config);
    BoofCVQualityMetrics javaObjectToMetrics(jobject metrics_obj);
    
    // Precision validation
    void validatePrecisionTarget(const BoofCVQualityMetrics& metrics);
};

} // namespace stereo
} // namespace unlook

#endif // HAVE_BOOFCV