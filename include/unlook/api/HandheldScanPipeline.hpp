#pragma once

#include <unlook/core/types.hpp>
#include <unlook/core/Logger.hpp>
#include <unlook/stereo/StereoMatcher.hpp>
#include <unlook/stereo/TemporalStereoProcessor.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <memory>
#include <vector>
#include <functional>
#include <atomic>
#include <chrono>
#include <future>

namespace unlook {

// Forward declarations
namespace camera {
    class CameraSystem;
}

namespace api {

/**
 * @brief Handheld scanning pipeline with IMU-based stability detection
 *
 * Implements a complete handheld scanning pipeline with:
 * - IMU-gated capture (only when stable)
 * - Multi-frame capture and fusion for noise reduction
 * - Outlier rejection with configurable sigma threshold
 * - WLS filtering for edge-preserving smoothing
 * - Real-time performance monitoring
 *
 * Target precision:
 * - 500mm distance: 0.1mm precision (achievable: 0.04mm)
 * - 1000mm distance: 0.5mm precision (achievable: 0.16mm)
 *
 * Performance targets:
 * - 10 frames @ 10 FPS = 1.0 second scan time
 * - Total pipeline: <1.5 seconds including stability wait
 */
class HandheldScanPipeline {
public:
    /**
     * @brief Scan parameters for handheld operation
     */
    struct ScanParams {
        int numFrames = 10;                    ///< Number of frames to capture and fuse
        float targetPrecisionMM = 0.1f;        ///< Target precision in millimeters
        float maxDistanceMM = 1000.0f;         ///< Maximum scanning distance
        float outlierSigma = 2.5f;             ///< Outlier rejection threshold (std devs)

        // WLS filter parameters
        bool useWLSFilter = true;              ///< Enable WLS filtering
        double wlsLambda = 8000.0;             ///< WLS lambda parameter
        double wlsSigma = 1.5;                 ///< WLS sigma parameter

        // Stability parameters
        float stabilityThreshold = 0.95f;      ///< Minimum stability score (0-1)
        int stabilityTimeoutMs = 10000;        ///< Maximum time to wait for stability

        // Performance parameters
        bool useParallel = true;               ///< Enable parallel processing
        int numThreads = 4;                    ///< Number of threads for parallel ops

        // VCSEL parameters
        bool useVCSEL = true;                  ///< Use VCSEL projection
        int vcselPowerPercent = 100;           ///< VCSEL power level (0-100)
    };

    /**
     * @brief Scan result containing depth data and metrics
     */
    struct ScanResult {
        cv::Mat depthMap;                      ///< Fused depth map (CV_32F) in mm
        cv::Mat confidenceMap;                 ///< Confidence map (CV_32F, 0-1)
        cv::Mat pointCloud;                    ///< 3D point cloud (CV_32FC3)

        // Quality metrics
        float achievedPrecisionMM = 0.0f;      ///< Estimated achieved precision
        int validPixelPercentage = 0;          ///< Percentage of valid depth pixels
        float avgConfidence = 0.0f;            ///< Average confidence score

        // Timing information
        std::chrono::milliseconds scanDuration;     ///< Total scan time
        std::chrono::milliseconds stabilityWaitTime;///< Time spent waiting for stability
        std::chrono::milliseconds captureTime;      ///< Multi-frame capture time
        std::chrono::milliseconds processingTime;   ///< Processing and fusion time

        // Frame-level statistics
        int framesCaptures = 0;                ///< Number of frames captured
        int framesUsed = 0;                    ///< Number of frames after outlier rejection

        // Error information
        bool success = false;                  ///< Whether scan completed successfully
        std::string errorMessage;              ///< Error message if failed
    };

    /**
     * @brief Single stereo frame data
     */
    struct StereoFrame {
        cv::Mat leftImage;                     ///< Left camera image
        cv::Mat rightImage;                    ///< Right camera image
        cv::Mat leftVCSEL;                     ///< Left camera with VCSEL projection
        cv::Mat rightVCSEL;                    ///< Right camera with VCSEL projection
        uint64_t timestampUs;                  ///< Frame timestamp in microseconds
        float stabilityScore;                  ///< IMU stability score at capture
    };

    /**
     * @brief Progress callback for GUI updates
     * @param progress Progress value (0.0 to 1.0)
     * @param message Status message
     */
    using ProgressCallback = std::function<void(float progress, const std::string& message)>;

    /**
     * @brief Stability callback for real-time stability updates
     * @param stabilityScore Current stability score (0.0 to 1.0)
     */
    using StabilityCallback = std::function<void(float stabilityScore)>;

    /**
     * @brief Constructor
     * @param cameraSystem Shared pointer to camera system
     */
    explicit HandheldScanPipeline(std::shared_ptr<camera::CameraSystem> cameraSystem);

    /**
     * @brief Destructor
     */
    ~HandheldScanPipeline();

    /**
     * @brief Initialize the pipeline
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Shutdown the pipeline
     */
    void shutdown();

    /**
     * @brief Perform a handheld scan with stability detection
     *
     * Complete scanning pipeline:
     * 1. Wait for IMU stability
     * 2. Capture multiple frames
     * 3. Process each frame for depth
     * 4. Fuse depth maps with outlier rejection
     * 5. Apply WLS filtering
     * 6. Generate point cloud
     *
     * @param params Scan parameters
     * @param progressCallback Optional progress callback for GUI
     * @return Scan result with depth map and metrics
     */
    ScanResult scanWithStability(const ScanParams& params,
                                 ProgressCallback progressCallback = nullptr);

    /**
     * @brief Wait for IMU stability before scanning
     * @param callback Stability update callback
     * @param timeoutMs Maximum time to wait in milliseconds
     * @return true if stable, false if timeout
     */
    bool waitForStability(StabilityCallback callback, int timeoutMs = 10000);

    /**
     * @brief Capture multiple synchronized frames
     * @param numFrames Number of frames to capture
     * @param progressCallback Optional progress callback
     * @return Vector of captured stereo frames
     */
    std::vector<StereoFrame> captureMultiFrame(int numFrames,
                                               ProgressCallback progressCallback = nullptr);

    /**
     * @brief Process frames to generate depth maps
     * @param frames Vector of stereo frames
     * @param params Stereo matching parameters
     * @return Vector of depth maps
     */
    std::vector<cv::Mat> processFrames(const std::vector<StereoFrame>& frames,
                                       const stereo::StereoMatchingParams& params);

    /**
     * @brief Fuse multiple depth maps with outlier rejection
     *
     * Algorithm:
     * - For each pixel, collect depth values from all frames
     * - Calculate mean and standard deviation
     * - Reject outliers beyond sigma threshold
     * - Compute weighted median of inliers
     *
     * @param depthMaps Vector of depth maps to fuse
     * @param outlierSigma Number of standard deviations for outlier rejection
     * @return Fused depth map
     */
    cv::Mat fuseDepthMaps(const std::vector<cv::Mat>& depthMaps,
                         float outlierSigma = 2.5f);

    /**
     * @brief Apply WLS filter for edge-preserving smoothing
     * @param depthMap Input depth map
     * @param guideImage Guide image (typically left camera image)
     * @param lambda WLS lambda parameter
     * @param sigma WLS sigma parameter
     * @return Filtered depth map
     */
    cv::Mat wlsFilter(const cv::Mat& depthMap,
                     const cv::Mat& guideImage,
                     double lambda = 8000.0,
                     double sigma = 1.5);

    /**
     * @brief Generate 3D point cloud from depth map
     * @param depthMap Depth map in millimeters
     * @param colorImage Optional color image for point colors
     * @return 3D point cloud (CV_32FC3 or CV_32FC6 with colors)
     */
    cv::Mat generatePointCloud(const cv::Mat& depthMap,
                               const cv::Mat& colorImage = cv::Mat());

    /**
     * @brief Calculate precision estimate from depth variance
     * @param depthMaps Vector of depth maps
     * @return Estimated precision in millimeters
     */
    float calculatePrecision(const std::vector<cv::Mat>& depthMaps);

    /**
     * @brief Set stereo matching algorithm
     * @param algorithm Algorithm type
     */
    void setStereoAlgorithm(stereo::StereoAlgorithm algorithm);

    /**
     * @brief Get current stereo matching parameters
     * @return Current parameters
     */
    stereo::StereoMatchingParams getStereoParams() const;

    /**
     * @brief Set stereo matching parameters
     * @param params New parameters
     */
    void setStereoParams(const stereo::StereoMatchingParams& params);

    /**
     * @brief Enable/disable VCSEL projection
     * @param enable Enable state
     */
    void enableVCSEL(bool enable);

    /**
     * @brief Get pipeline statistics
     * @return Map of statistic name to value
     */
    std::map<std::string, double> getStatistics() const;

    /**
     * @brief Save debug output (images, depth maps, point cloud)
     * @param timestamp Timestamp string for file naming
     * @param frames Captured stereo frames
     * @param depthMaps Generated depth maps
     * @param fusedDepth Fused depth map
     * @param pointCloud Final point cloud
     * @return true if all debug output saved successfully
     */
    bool saveDebugOutput(const std::string& timestamp,
                        const std::vector<StereoFrame>& frames,
                        const std::vector<cv::Mat>& depthMaps,
                        const cv::Mat& fusedDepth,
                        const cv::Mat& pointCloud);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Disable copy and move
    HandheldScanPipeline(const HandheldScanPipeline&) = delete;
    HandheldScanPipeline& operator=(const HandheldScanPipeline&) = delete;
    HandheldScanPipeline(HandheldScanPipeline&&) = delete;
    HandheldScanPipeline& operator=(HandheldScanPipeline&&) = delete;
};

} // namespace api
} // namespace unlook