#pragma once

#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/stereo/StereoMatcher.hpp>
#include <unlook/stereo/DepthProcessor.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <chrono>
#include <atomic>
#include <mutex>

namespace unlook {
namespace stereo {

/**
 * Temporal Stereo Processor for VCSEL-Based Depth Sensing
 *
 * This class implements temporal stereo matching using triple frame capture
 * from dual VCSEL projectors. It isolates projected patterns by subtracting
 * ambient illumination and combines patterns from both VCSELs for enhanced
 * coverage and precision.
 *
 * Algorithm Overview:
 * 1. Capture triple frame sequence (VCSEL1, VCSEL2, Ambient)
 * 2. Isolate patterns by subtracting ambient from VCSEL frames
 * 3. Combine patterns from both VCSELs for maximum coverage
 * 4. Perform stereo matching on isolated patterns only
 * 5. Apply temporal fusion for stable depth estimation
 *
 * Performance Characteristics:
 * - Pattern isolation variance > 200 (high distinctivity)
 * - Coverage 60-80% on any surface
 * - Processing time < 500ms total
 * - Depth accuracy < 0.005mm target (hardware limited)
 *
 * Thread Safety: All public methods are thread-safe
 */
class TemporalStereoProcessor {
public:
    /**
     * Pattern isolation parameters
     */
    struct PatternIsolationParams {
        // Noise threshold for pattern isolation
        int noiseThreshold = 10;           // Minimum intensity difference for valid pattern

        // Pattern combination weights
        float vcsel1Weight = 0.5f;         // Weight for VCSEL1 pattern
        float vcsel2Weight = 0.5f;         // Weight for VCSEL2 pattern

        // Enhancement parameters
        bool enhanceContrast = true;       // Apply contrast enhancement to isolated patterns
        float contrastAlpha = 1.5f;        // Contrast scaling factor
        float contrastBeta = -20.0f;       // Contrast offset

        // Morphological operations
        bool applyMorphology = true;       // Apply morphological operations
        int morphKernelSize = 3;           // Kernel size for morphological operations

        // Temporal filtering
        bool enableTemporalFilter = false; // Enable temporal averaging
        int temporalWindowSize = 3;        // Number of frames for temporal averaging
        float temporalDecay = 0.7f;        // Decay factor for temporal averaging

        // Quality validation
        float minPatternVariance = 200.0f; // Minimum variance for valid pattern
        float minCoverage = 0.6f;          // Minimum coverage ratio (60%)
        float maxCoverage = 0.8f;          // Maximum coverage ratio (80%)
    };

    /**
     * Temporal stereo processing configuration
     */
    struct TemporalStereoConfig {
        PatternIsolationParams isolationParams;

        // SGBM parameters optimized for VCSEL patterns
        StereoMatchingParams sgbmParams;

        // Processing options
        bool useWLSFilter = true;          // Apply WLS filtering to disparity
        bool computeConfidence = true;     // Compute confidence map
        bool validateDepth = true;         // Validate depth topologically

        // Performance options
        bool useParallel = true;           // Enable parallel processing
        int numThreads = 4;                // Number of threads for processing

        // Debug options
        bool saveIntermediateImages = false; // Save debug images
        std::string debugOutputPath = "/tmp/temporal_stereo/";

        /**
         * Get default configuration optimized for VCSEL
         */
        static TemporalStereoConfig getVCSELOptimized() {
            TemporalStereoConfig config;

            // VCSEL-optimized SGBM parameters
            config.sgbmParams.blockSize = 5;           // Small block for dot patterns
            config.sgbmParams.numDisparities = 160;    // Extended range for 70mm baseline
            config.sgbmParams.minDisparity = 0;
            config.sgbmParams.P1 = 8 * 3 * 5 * 5;     // Adjusted for blockSize=5
            config.sgbmParams.P2 = 32 * 3 * 5 * 5;
            config.sgbmParams.uniquenessRatio = 15;    // Stricter for dot patterns
            config.sgbmParams.speckleWindowSize = 50;  // Smaller for dot preservation
            config.sgbmParams.speckleRange = 16;
            config.sgbmParams.disp12MaxDiff = 1;
            config.sgbmParams.preFilterCap = 31;
            config.sgbmParams.mode = 3;                // MODE_HH4 for best quality

            return config;
        }
    };

    /**
     * Temporal stereo processing results
     */
    struct TemporalStereoResult {
        cv::Mat depthMap;                  // Final depth map in millimeters
        cv::Mat confidenceMap;             // Confidence map (0-1)
        cv::Mat disparityMap;              // Raw disparity map

        // Isolated patterns for debugging
        cv::Mat isolatedPatternLeft;      // Combined isolated pattern (left)
        cv::Mat isolatedPatternRight;     // Combined isolated pattern (right)

        // Individual VCSEL patterns
        cv::Mat vcsel1PatternLeft;        // VCSEL1 isolated pattern (left)
        cv::Mat vcsel1PatternRight;       // VCSEL1 isolated pattern (right)
        cv::Mat vcsel2PatternLeft;        // VCSEL2 isolated pattern (left)
        cv::Mat vcsel2PatternRight;       // VCSEL2 isolated pattern (right)

        // Quality metrics
        float patternVariance = 0.0f;     // Pattern distinctiveness metric
        float coverageRatio = 0.0f;       // Pattern coverage ratio
        float avgConfidence = 0.0f;       // Average confidence score
        int validPixelCount = 0;          // Number of valid depth pixels

        // Timing information
        double isolationTimeMs = 0.0;     // Pattern isolation time
        double matchingTimeMs = 0.0;      // Stereo matching time
        double totalTimeMs = 0.0;         // Total processing time

        // Validation results
        bool isTopologicallyCorrect = false; // Z-order validation
        std::string validationMessage;       // Validation details

        bool isValid() const {
            return !depthMap.empty() &&
                   patternVariance >= 200.0f &&
                   coverageRatio >= 0.6f && coverageRatio <= 0.8f;
        }
    };

    /**
     * Constructor
     */
    TemporalStereoProcessor();

    /**
     * Destructor
     */
    ~TemporalStereoProcessor();

    /**
     * Initialize the temporal stereo processor
     * @param dualVCSEL Dual VCSEL controller instance
     * @param calibManager Calibration manager with stereo calibration
     * @param config Processing configuration
     * @return true if initialization successful
     */
    bool initialize(
        std::shared_ptr<hardware::AS1170DualVCSELController> dualVCSEL,
        std::shared_ptr<calibration::CalibrationManager> calibManager,
        const TemporalStereoConfig& config = TemporalStereoConfig::getVCSELOptimized()
    );

    /**
     * Process temporal stereo from triple frame capture
     * @param frames Triple frame capture from dual VCSEL controller
     * @param result Output processing results
     * @return true if processing successful
     */
    bool processTemporalStereo(
        const hardware::AS1170DualVCSELController::TripleFrameCapture& frames,
        TemporalStereoResult& result
    );

    /**
     * Capture and process in one step
     * @param result Output processing results
     * @return true if capture and processing successful
     */
    bool captureAndProcess(TemporalStereoResult& result);

    /**
     * Isolate VCSEL pattern by subtracting ambient
     * @param vcselFrame Frame with VCSEL illumination
     * @param ambientFrame Frame with ambient illumination only
     * @param isolatedPattern Output isolated pattern
     * @return Pattern variance metric (higher = better distinctiveness)
     */
    float isolateVCSELPattern(
        const cv::Mat& vcselFrame,
        const cv::Mat& ambientFrame,
        cv::Mat& isolatedPattern
    );

    /**
     * Combine patterns from dual VCSELs
     * @param pattern1 Pattern from VCSEL1
     * @param pattern2 Pattern from VCSEL2
     * @param combined Output combined pattern
     * @return Coverage ratio (0-1)
     */
    float combinePatterns(
        const cv::Mat& pattern1,
        const cv::Mat& pattern2,
        cv::Mat& combined
    );

    /**
     * Validate depth topologically
     * @param depthMap Depth map to validate
     * @param message Output validation message
     * @return true if topologically correct
     */
    bool validateDepthTopology(
        const cv::Mat& depthMap,
        std::string& message
    );

    /**
     * Update configuration
     * @param config New configuration
     * @return true if update successful
     */
    bool updateConfig(const TemporalStereoConfig& config);

    /**
     * Get current configuration
     */
    TemporalStereoConfig getConfig() const;

    /**
     * Get stereo matcher instance (for parameter tuning)
     */
    StereoMatcher* getStereoMatcher() const;

    /**
     * Enable/disable debug mode
     * @param enable Enable debug output
     * @param outputPath Path for debug images
     */
    void setDebugMode(bool enable, const std::string& outputPath = "/tmp/temporal_stereo/");

    /**
     * Get last processing statistics
     */
    std::map<std::string, double> getStatistics() const;

    /**
     * Check if processor is initialized
     */
    bool isInitialized() const { return initialized_.load(); }

private:
    // Core components
    std::shared_ptr<hardware::AS1170DualVCSELController> dualVCSEL_;
    std::shared_ptr<calibration::CalibrationManager> calibManager_;
    std::unique_ptr<StereoMatcher> stereoMatcher_;

    // Configuration
    TemporalStereoConfig config_;

    // State management
    std::atomic<bool> initialized_{false};
    mutable std::mutex configMutex_;
    mutable std::mutex statsMutex_;

    // Statistics
    std::map<std::string, double> statistics_;

    // Debug settings
    bool debugMode_ = false;
    std::string debugOutputPath_;

    // Temporal filtering state
    std::deque<cv::Mat> temporalBuffer_;
    std::mutex temporalMutex_;

    // Internal processing methods

    /**
     * Apply contrast enhancement to isolated pattern
     */
    void enhancePatternContrast(cv::Mat& pattern);

    /**
     * Apply morphological operations to clean pattern
     */
    void applyMorphologicalCleaning(cv::Mat& pattern);

    /**
     * Compute pattern quality metrics
     */
    void computePatternMetrics(
        const cv::Mat& pattern,
        float& variance,
        float& coverage
    );

    /**
     * Convert disparity to depth using calibration
     */
    bool disparityToDepth(
        const cv::Mat& disparity,
        cv::Mat& depth
    );

    /**
     * Generate dual depth maps from individual VCSEL patterns
     */
    bool generateDualDepthMaps(
        const cv::Mat& pattern1Left, const cv::Mat& pattern1Right,
        const cv::Mat& pattern2Left, const cv::Mat& pattern2Right,
        cv::Mat& depth1, cv::Mat& depth2,
        cv::Mat& confidence1, cv::Mat& confidence2
    );

    /**
     * Intelligently fuse dual depth maps using confidence and geometry
     */
    bool fuseDualDepthMaps(
        const cv::Mat& depth1, const cv::Mat& depth2,
        const cv::Mat& confidence1, const cv::Mat& confidence2,
        cv::Mat& fusedDepth, cv::Mat& fusedConfidence
    );

    /**
     * Apply temporal filtering to depth map
     */
    void applyTemporalFilter(cv::Mat& depth);

    /**
     * Save debug images if enabled
     */
    void saveDebugImages(const TemporalStereoResult& result);

    /**
     * Update processing statistics
     */
    void updateStatistics(const TemporalStereoResult& result);

    /**
     * Validate configuration parameters
     */
    bool validateConfig(const TemporalStereoConfig& config) const;
};

} // namespace stereo
} // namespace unlook