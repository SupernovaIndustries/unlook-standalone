#pragma once

#include <unlook/camera/RawCameraInterface.hpp>
#include <unlook/stereo/StereoMatcher.hpp>
#include <unlook/stereo/SGBMStereoMatcher.hpp>
#include <opencv2/core.hpp>
#include <memory>
#include <atomic>

namespace unlook {
namespace stereo {

/**
 * Raw Stereo Processor for VCSEL pattern preservation
 *
 * This class integrates the RAW camera pipeline with stereo matching,
 * specifically designed to preserve VCSEL dot patterns for accurate depth
 * calculation. It bypasses all preprocessing that could smooth or degrade
 * the IR pattern detail.
 *
 * Key features:
 * - Direct SBGGR10 to blue channel extraction
 * - No debayering or smoothing operations
 * - Variance-based pattern quality monitoring
 * - Adaptive parameter tuning for VCSEL patterns
 */
class RawStereoProcessor {
public:
    /**
     * Processing configuration
     */
    struct Config {
        // Camera options
        bool useRawPipeline;      // Use RAW pipeline vs standard
        bool extractBlueOnly;     // Extract blue channel for IR
        bool use16bitProcessing; // Use 16-bit for higher precision

        // SGBM parameters optimized for VCSEL
        int minDisparity;
        int numDisparities;        // Must be divisible by 16
        int blockSize;               // Small for dot patterns
        int p1;                     // Penalty for 1-pixel disparity change
        int p2;                    // Penalty for >1-pixel change
        int disp12MaxDiff;          // L-R consistency check
        int preFilterCap;
        int uniquenessRatio;
        int speckleWindowSize;
        int speckleRange;

        // Quality monitoring
        double minVarianceThreshold;  // Minimum variance for valid pattern
        double targetVariance;        // Target variance for VCSEL dots
        bool autoAdjustExposure;      // Auto-adjust for variance

        // Debug options
        bool saveRawImages;
        bool saveVarianceMaps;
        bool computeStatistics;
        std::string debugPath;

        // Constructor with defaults
        Config()
            : useRawPipeline(true),
              extractBlueOnly(true),
              use16bitProcessing(false),
              minDisparity(0),
              numDisparities(128),
              blockSize(5),
              p1(50),
              p2(200),
              disp12MaxDiff(1),
              preFilterCap(31),
              uniquenessRatio(10),
              speckleWindowSize(100),
              speckleRange(32),
              minVarianceThreshold(100.0),
              targetVariance(150.0),
              autoAdjustExposure(false),
              saveRawImages(false),
              saveVarianceMaps(false),
              computeStatistics(true),
              debugPath("/tmp/raw_stereo/") {}
    };

    /**
     * Processing results
     */
    struct ProcessingResult {
        cv::Mat disparityMap;        // Disparity output
        cv::Mat depthMap;            // Depth in mm
        cv::Mat leftBlue;            // Left blue channel used
        cv::Mat rightBlue;           // Right blue channel used
        cv::Mat confidenceMap;       // Confidence/quality map

        // Quality metrics
        double leftVariance;
        double rightVariance;
        double varianceRatio;        // raw/processed ratio
        int validPixels;
        int totalPixels;
        double coverage;             // % of valid depth pixels

        // Timing
        double captureTimeMs;
        double processingTimeMs;
        double totalTimeMs;

        // Sync status
        double syncErrorMs;
        bool synchronized;
    };

    /**
     * Constructor
     */
    explicit RawStereoProcessor(const Config& config = Config());

    /**
     * Destructor
     */
    ~RawStereoProcessor();

    /**
     * Initialize the processor
     */
    bool initialize();

    /**
     * Process single stereo pair with RAW pipeline
     * @param result Output processing result
     * @return Success status
     */
    bool processStereo(ProcessingResult& result);

    /**
     * Process pre-captured RAW frames
     * @param leftFrame Left RAW frame
     * @param rightFrame Right RAW frame
     * @param result Output processing result
     * @return Success status
     */
    bool processRawFrames(const camera::RawCameraInterface::RawFrame& leftFrame,
                         const camera::RawCameraInterface::RawFrame& rightFrame,
                         ProcessingResult& result);

    /**
     * Compare RAW vs standard pipeline
     * @param rawResult RAW pipeline result
     * @param standardResult Standard pipeline result
     * @return Success status
     */
    struct PipelineComparison {
        double rawVarianceLeft;
        double rawVarianceRight;
        double standardVarianceLeft;
        double standardVarianceRight;
        double varianceImprovementLeft;   // % improvement
        double varianceImprovementRight;  // % improvement

        double rawCoverage;
        double standardCoverage;
        double coverageImprovement;       // % improvement

        int rawValidPixels;
        int standardValidPixels;

        cv::Mat varianceDifferenceMap;    // Local variance difference
    };

    bool comparePipelines(PipelineComparison& comparison);

    /**
     * Auto-tune SGBM parameters for VCSEL patterns
     * @param targetVariance Target variance for optimization
     * @return Optimized configuration
     */
    Config autoTuneParameters(double targetVariance = 150.0);

    /**
     * Set configuration
     */
    void setConfig(const Config& config) { config_ = config; updateSGBMParameters(); }

    /**
     * Get current configuration
     */
    Config getConfig() const { return config_; }

    /**
     * Get last error
     */
    std::string getLastError() const { return lastError_; }

    /**
     * Enable verbose debug output
     */
    void setDebugMode(bool enable) { debugMode_ = enable; }

private:
    /**
     * Update SGBM parameters from config
     */
    void updateSGBMParameters();

    /**
     * Compute disparity from blue channel images
     */
    bool computeDisparityFromBlue(const cv::Mat& leftBlue,
                                  const cv::Mat& rightBlue,
                                  cv::Mat& disparity);

    /**
     * Compute confidence map from disparity
     */
    cv::Mat computeConfidenceMap(const cv::Mat& disparity);

    /**
     * Convert disparity to depth
     */
    void disparityToDepth(const cv::Mat& disparity,
                         cv::Mat& depth,
                         double baseline = 70.017,
                         double focalLength = 600.0);

    /**
     * Validate pattern quality
     */
    bool validatePatternQuality(const cv::Mat& image, double& variance);

    /**
     * Save debug images
     */
    void saveDebugImages(const ProcessingResult& result);

    /**
     * Compute coverage statistics
     */
    void computeCoverageStats(const cv::Mat& disparity, ProcessingResult& result);

    // Member variables
    Config config_;
    std::unique_ptr<camera::StereoRawCameraSystem> cameraSystem_;
    std::unique_ptr<SGBMStereoMatcher> sgbmMatcher_;

    // State
    std::atomic<bool> initialized_{false};
    bool debugMode_{false};

    // Error handling
    mutable std::string lastError_;

    // Statistics tracking
    struct Statistics {
        double avgLeftVariance = 0;
        double avgRightVariance = 0;
        double avgCoverage = 0;
        double avgSyncError = 0;
        int frameCount = 0;
    };
    Statistics stats_;
    mutable std::mutex statsMutex_;
};

/**
 * Variance-based exposure controller for VCSEL patterns
 */
class VarianceExposureController {
public:
    /**
     * Constructor
     * @param targetVariance Target variance for VCSEL patterns
     * @param tolerance Acceptable variance tolerance
     */
    VarianceExposureController(double targetVariance = 150.0, double tolerance = 10.0);

    /**
     * Compute optimal exposure for target variance
     * @param currentVariance Current image variance
     * @param currentExposure Current exposure in microseconds
     * @return Suggested exposure adjustment
     */
    double computeExposureAdjustment(double currentVariance, double currentExposure);

    /**
     * Update controller with feedback
     */
    void updateFeedback(double variance, double exposure);

    /**
     * Reset controller state
     */
    void reset();

private:
    double targetVariance_;
    double tolerance_;
    double kp_ = 0.5;  // Proportional gain
    double ki_ = 0.1;  // Integral gain
    double integralError_ = 0;
    double lastError_ = 0;
    std::deque<std::pair<double, double>> history_;  // variance, exposure pairs
    static constexpr size_t maxHistorySize_ = 10;
};

} // namespace stereo
} // namespace unlook