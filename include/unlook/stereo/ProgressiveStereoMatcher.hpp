#pragma once

#include "unlook/stereo/StereoMatcher.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include <opencv2/stereo.hpp>
#include <opencv2/ximgproc.hpp>
#include <chrono>
#include <atomic>
#include <mutex>

namespace unlook {
namespace stereo {

/**
 * @brief Depth layer definition for progressive processing
 *
 * Defines a specific depth range for focused processing
 * with associated disparity bounds and processing parameters
 */
struct DepthLayer {
    std::string name;           // Layer identifier (e.g., "NEAR", "MID", "FAR")
    float minDepthMm;          // Minimum depth in millimeters
    float maxDepthMm;          // Maximum depth in millimeters
    int minDisparity;          // Minimum disparity for this layer
    int maxDisparity;          // Maximum disparity for this layer
    int numDisparities;        // Number of disparity levels to search
    float confidenceThreshold;  // Minimum confidence to accept matches
    bool enabled;              // Whether to process this layer

    // Processing parameters optimized per layer
    int blockSize;             // Block size for this depth range
    int uniquenessRatio;       // Uniqueness ratio for validation
    int speckleWindowSize;     // Speckle filter size

    // Performance statistics
    double processingTimeMs = 0.0;
    int validPixelCount = 0;
    float averageConfidence = 0.0f;
};

/**
 * @brief Progressive processing configuration
 *
 * Controls the behavior of progressive depth layer processing
 * optimized for 70mm baseline and close-range scanning
 */
struct ProgressiveConfig {
    // Baseline and calibration parameters
    float baselineMm = 70.017f;        // Calibrated baseline in mm
    float focalLength = 1456.0f;       // Focal length in pixels (approximate)

    // Progressive processing control
    bool enableProgressive = true;      // Use progressive layers
    bool earlyTermination = true;       // Stop when confidence is high
    float earlyTerminationThreshold = 0.95f; // Confidence for early stop

    // Background filtering
    bool filterBackground = true;       // Remove far background
    float backgroundThresholdMm = 1500.0f; // Beyond this is background

    // Layer propagation
    bool propagateConfidence = true;    // Use previous layer confidence
    bool adaptiveParameters = true;     // Adjust params based on texture

    // Performance optimization
    bool useParallel = true;           // Parallel layer processing
    int numThreads = 4;                // Thread count for parallel ops
    bool useNEON = true;               // ARM64 NEON optimizations

    // Memory optimization
    bool reuseBuffers = true;          // Reuse memory between layers
    size_t maxMemoryMB = 500;          // Maximum memory usage (CM4 constraint)
};

/**
 * @brief Statistics for progressive processing
 */
struct ProgressiveStats {
    double totalProcessingTimeMs = 0.0;
    double perLayerTimeMs[3] = {0.0, 0.0, 0.0};
    int totalValidPixels = 0;
    int layerValidPixels[3] = {0, 0, 0};
    float averageConfidence = 0.0f;
    int earlyTerminationCount = 0;
    int backgroundFilteredPixels = 0;
    double memoryUsageMB = 0.0;

    std::string toString() const;
};

/**
 * @brief Progressive stereo matcher with depth layer processing
 *
 * Implements a multi-layer progressive stereo matching approach
 * optimized for the Unlook 3D Scanner with 70mm baseline.
 *
 * Key features:
 * - Progressive depth layers (near to far)
 * - Disparity ROI focusing (skip unnecessary ranges)
 * - Background filtering beyond useful range
 * - Confidence-guided propagation between layers
 * - Early termination for high-confidence regions
 * - ARM64/NEON optimizations for Raspberry Pi CM4
 *
 * Mathematical basis:
 * Given baseline b = 70.017mm and focal length f ≈ 1456 pixels,
 * the disparity-to-depth relationship is:
 *   depth = (b * f) / disparity
 *
 * For our target range 100-800mm:
 *   disparity_max = (70.017 * 1456) / 100 ≈ 1019 pixels
 *   disparity_min = (70.017 * 1456) / 800 ≈ 127 pixels
 *
 * However, practical limitations and noise considerations:
 *   - Maximum useful disparity: 320 (depth ≈ 318mm)
 *   - Minimum useful disparity: 50 (depth ≈ 2039mm)
 *   - Focus range disparity: 50-320 (covers 318-2039mm)
 */
class ProgressiveStereoMatcher : public StereoMatcher {
public:
    ProgressiveStereoMatcher();
    ~ProgressiveStereoMatcher() override;

    /**
     * @brief Compute disparity using progressive layer processing
     *
     * Processes depth in layers from near to far, with early
     * termination when confidence is high enough
     */
    bool computeDisparity(const cv::Mat& leftRectified,
                         const cv::Mat& rightRectified,
                         cv::Mat& disparity) override;

    /**
     * @brief Compute disparity with per-pixel confidence map
     */
    bool computeDisparityWithConfidence(const cv::Mat& leftRectified,
                                       const cv::Mat& rightRectified,
                                       cv::Mat& disparity,
                                       cv::Mat& confidence) override;

    /**
     * @brief Set stereo matching parameters
     */
    bool setParameters(const StereoMatchingParams& params) override;

    /**
     * @brief Get current stereo matching parameters
     */
    StereoMatchingParams getParameters() const override;

    /**
     * @brief Get algorithm type
     */
    StereoAlgorithm getAlgorithmType() const override {
        return StereoAlgorithm::CUSTOM;
    }

    /**
     * @brief Get algorithm name
     */
    std::string getAlgorithmName() const override {
        return "Progressive Layer Stereo Matcher (70mm Optimized)";
    }

    /**
     * @brief Apply post-processing with WLS filter
     */
    bool applyPostProcessing(cv::Mat& disparity,
                           const cv::Mat& leftImage) override;

    // Progressive-specific methods

    /**
     * @brief Configure progressive processing parameters
     * @param config Progressive processing configuration
     */
    void setProgressiveConfig(const ProgressiveConfig& config);

    /**
     * @brief Get current progressive configuration
     */
    ProgressiveConfig getProgressiveConfig() const;

    /**
     * @brief Setup depth layers for processing
     *
     * Default configuration for 70mm baseline:
     * - Layer 1 (NEAR): 300-600mm, disparity 170-340
     * - Layer 2 (MID): 500-1000mm, disparity 100-200
     * - Layer 3 (FAR): 900-1500mm, disparity 68-113
     */
    void setupDefaultLayers();

    /**
     * @brief Setup custom depth layers
     * @param layers Vector of depth layer definitions
     */
    void setDepthLayers(const std::vector<DepthLayer>& layers);

    /**
     * @brief Get current depth layers
     */
    std::vector<DepthLayer> getDepthLayers() const;

    /**
     * @brief Set disparity ROI for focused processing
     *
     * Limits the disparity search range to useful values,
     * skipping computation for background beyond useful range
     *
     * @param minDisparity Minimum disparity (far depth limit)
     * @param maxDisparity Maximum disparity (near depth limit)
     */
    void setDisparityROI(int minDisparity, int maxDisparity);

    /**
     * @brief Get processing statistics
     */
    ProgressiveStats getStatistics() const;

    /**
     * @brief Reset statistics counters
     */
    void resetStatistics();

    /**
     * @brief Enable/disable specific depth layer
     * @param layerIndex Index of the layer (0=NEAR, 1=MID, 2=FAR)
     * @param enabled Whether to process this layer
     */
    void setLayerEnabled(int layerIndex, bool enabled);

    /**
     * @brief Set callback for processing progress
     * @param callback Function called with progress percentage (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

private:
    // Progressive processing implementation

    /**
     * @brief Process a single depth layer
     * @param left Left rectified image
     * @param right Right rectified image
     * @param layer Depth layer definition
     * @param layerDisparity Output disparity for this layer
     * @param layerConfidence Output confidence for this layer
     * @return true if processing successful
     */
    bool processDepthLayer(const cv::Mat& left,
                          const cv::Mat& right,
                          const DepthLayer& layer,
                          cv::Mat& layerDisparity,
                          cv::Mat& layerConfidence);

    /**
     * @brief Merge layer results into final disparity map
     * @param layers Vector of layer disparities
     * @param confidences Vector of layer confidence maps
     * @param finalDisparity Output merged disparity
     * @param finalConfidence Output merged confidence
     */
    void mergeLayers(const std::vector<cv::Mat>& layers,
                    const std::vector<cv::Mat>& confidences,
                    cv::Mat& finalDisparity,
                    cv::Mat& finalConfidence);

    /**
     * @brief Filter background pixels beyond useful range
     * @param disparity Input/output disparity map
     * @param confidence Confidence map for masking
     */
    void filterBackground(cv::Mat& disparity, const cv::Mat& confidence);

    /**
     * @brief Check if early termination criteria met
     * @param confidence Current confidence map
     * @return true if can terminate early
     */
    bool checkEarlyTermination(const cv::Mat& confidence) const;

    /**
     * @brief Propagate confidence from previous layer
     * @param prevConfidence Previous layer confidence
     * @param currConfidence Current layer confidence (updated)
     */
    void propagateConfidence(const cv::Mat& prevConfidence,
                           cv::Mat& currConfidence);

    /**
     * @brief Adapt parameters based on image texture
     * @param image Input image
     * @param layer Layer parameters to adapt
     */
    void adaptLayerParameters(const cv::Mat& image, DepthLayer& layer);

    /**
     * @brief Calculate disparity from depth using calibration
     * @param depthMm Depth in millimeters
     * @return Disparity in pixels
     */
    float depthToDisparity(float depthMm) const;

    /**
     * @brief Calculate depth from disparity using calibration
     * @param disparity Disparity in pixels
     * @return Depth in millimeters
     */
    float disparityToDepth(float disparity) const;

    // Member variables
    ProgressiveConfig config_;
    std::vector<DepthLayer> layers_;

    // Base stereo matcher for actual computation
    std::unique_ptr<SGBMStereoMatcher> baseMatcher_;

    // WLS filter for post-processing
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter_;
    cv::Ptr<cv::StereoMatcher> rightMatcher_;

    // Processing state
    mutable ProgressiveStats stats_;
    std::function<void(int)> progressCallback_;
    std::atomic<bool> cancelRequested_{false};
    mutable std::mutex statsMutex_;

    // Memory optimization
    cv::Mat tempLeft_, tempRight_;
    std::vector<cv::Mat> layerBuffers_;
    std::vector<cv::Mat> confidenceBuffers_;

    // Disparity ROI
    int roiMinDisparity_ = 50;
    int roiMaxDisparity_ = 320;
};

/**
 * @brief Background filter for efficient far-range removal
 *
 * Specialized filter for removing pixels beyond the useful
 * scanning range, reducing noise and processing time
 */
class BackgroundFilter {
public:
    /**
     * @brief Configure background filter
     * @param maxDepthMm Maximum useful depth in millimeters
     * @param baselineMm Stereo baseline in millimeters
     * @param focalLength Focal length in pixels
     */
    BackgroundFilter(float maxDepthMm = 1500.0f,
                    float baselineMm = 70.017f,
                    float focalLength = 1456.0f);

    /**
     * @brief Apply background filtering to disparity map
     * @param disparity Input/output disparity map
     * @param confidence Optional confidence map for guided filtering
     * @return Number of pixels filtered
     */
    int filterDisparity(cv::Mat& disparity,
                       const cv::Mat& confidence = cv::Mat());

    /**
     * @brief Create background mask
     * @param disparity Input disparity map
     * @param mask Output binary mask (255=foreground, 0=background)
     */
    void createMask(const cv::Mat& disparity, cv::Mat& mask) const;

    /**
     * @brief Set maximum depth threshold
     * @param maxDepthMm Maximum depth in millimeters
     */
    void setMaxDepth(float maxDepthMm);

    /**
     * @brief Get filtering statistics
     */
    struct FilterStats {
        int totalPixels = 0;
        int filteredPixels = 0;
        float filterRatio = 0.0f;
        double processingTimeMs = 0.0;
    };

    FilterStats getStatistics() const { return stats_; }
    void resetStatistics() { stats_ = FilterStats(); }

private:
    float maxDepthMm_;
    float baselineMm_;
    float focalLength_;
    float minDisparity_;  // Calculated from maxDepthMm

    mutable FilterStats stats_;
};

} // namespace stereo
} // namespace unlook