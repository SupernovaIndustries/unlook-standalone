/**
 * @file DebugOutputManager.hpp
 * @brief Centralized debug image and data output management
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 * Saves ALL intermediate processing stages for forensic analysis.
 */

#ifndef UNLOOK_STEREO_DEBUG_OUTPUT_MANAGER_HPP
#define UNLOOK_STEREO_DEBUG_OUTPUT_MANAGER_HPP

#include <opencv2/core.hpp>
#include <string>
#include <map>
#include <chrono>
#include <memory>

namespace unlook {
namespace core {
    class Logger;
}
namespace stereo {

/**
 * @brief Manages debug output for stereo processing pipeline
 *
 * Provides systematic saving of all intermediate images with:
 * - Numbered prefixes for ordering (00_, 01_, 02_, etc.)
 * - Descriptive names
 * - Multiple output formats (visualization, raw data)
 * - Processing reports with metrics
 */
class DebugOutputManager {
public:
    /**
     * @brief Debug configuration
     */
    struct Config {
        bool enabled = false;           ///< Master debug enable flag
        std::string outputDir = "";     ///< Base output directory

        bool saveInputImages = true;       ///< Save 00_input_*.png
        bool saveRectified = true;         ///< Save 01_rectified_*.png
        bool saveRawDisparity = true;      ///< Save 02_disparity_raw.*
        bool saveFilteredDisparity = true; ///< Save 03_disparity_filtered.*
        bool saveConfidenceMap = true;     ///< Save 04_confidence_map.png
        bool saveDepthMap = true;          ///< Save 05_depth_map.*
        bool saveDepthHistogram = true;    ///< Save 06_depth_histogram.png
        bool savePointCloud = true;        ///< Save 07_pointcloud.ply
        bool saveProcessingReport = true;  ///< Save processing_report.txt

        bool saveRawTiff = true;           ///< Save raw data as TIFF (16-bit)
        bool saveColorMapped = true;       ///< Save colormapped visualizations
        bool saveEpipolarCheck = true;     ///< Save epipolar line check

        std::string sessionName = "";      ///< Session name prefix (e.g., "scan00001")
    };

    /**
     * @brief Processing metrics for report
     */
    struct ProcessingMetrics {
        // Input
        cv::Size inputSize;
        cv::Size calibSize;

        // Quality
        float validPixelPercentage = 0.0f;
        float meanDepthMM = 0.0f;
        float depthStdDevMM = 0.0f;
        float minDepthMM = 0.0f;
        float maxDepthMM = 0.0f;

        // Performance
        std::chrono::milliseconds rectificationTime{0};
        std::chrono::milliseconds disparityTime{0};
        std::chrono::milliseconds filteringTime{0};
        std::chrono::milliseconds pointCloudTime{0};
        std::chrono::milliseconds totalTime{0};

        // GPU
        bool gpuUsed = false;
        float gpuUtilization = 0.0f;
        size_t gpuMemoryMB = 0;
    };

    /**
     * @brief Construct debug output manager
     *
     * @param logger Logger for messages
     */
    explicit DebugOutputManager(core::Logger* logger = nullptr);
    ~DebugOutputManager();

    /**
     * @brief Configure debug output
     *
     * @param config Debug configuration
     */
    void setConfig(const Config& config);

    /**
     * @brief Get current configuration
     */
    const Config& getConfig() const { return config_; }

    /**
     * @brief Check if debug output is enabled
     */
    bool isEnabled() const { return config_.enabled && !config_.outputDir.empty(); }

    /**
     * @brief Create new debug session directory
     *
     * Creates timestamped directory: outputDir/sessionName_YYYYMMDD_HHMMSS/
     * or outputDir/scanXXXXX/ if sessionName provided
     *
     * @param sessionName Optional session name (default: auto-timestamped)
     * @return Path to created session directory
     */
    std::string createSession(const std::string& sessionName = "");

    /**
     * @brief Get current session directory
     *
     * @return Current session directory path (empty if no session)
     */
    std::string getCurrentSessionDir() const { return currentSessionDir_; }

    // ========== IMAGE SAVING METHODS ==========

    /**
     * @brief Save input images (00_input_*.png)
     */
    void saveInputImages(const cv::Mat& leftInput, const cv::Mat& rightInput);

    /**
     * @brief Save rectified images with epipolar check (01_rectified_*.png)
     */
    void saveRectifiedImages(const cv::Mat& leftRect, const cv::Mat& rightRect);

    /**
     * @brief Save raw disparity (02_disparity_raw.*)
     *
     * Saves both TIFF (16-bit raw) and colormap visualization
     */
    void saveRawDisparity(const cv::Mat& rawDisparity);

    /**
     * @brief Save filtered disparity (03_disparity_filtered.*)
     */
    void saveFilteredDisparity(const cv::Mat& filteredDisparity);

    /**
     * @brief Save confidence map (04_confidence_map.png)
     */
    void saveConfidenceMap(const cv::Mat& confidenceMap);

    /**
     * @brief Save depth map (05_depth_map.*)
     *
     * Saves both raw TIFF (32-bit float) and colormap visualization
     */
    void saveDepthMap(const cv::Mat& depthMap);

    /**
     * @brief Save depth histogram (06_depth_histogram.png)
     */
    void saveDepthHistogram(const cv::Mat& depthMap);

    /**
     * @brief Save point cloud (07_pointcloud.ply)
     */
    void savePointCloud(const std::string& plyPath);

    /**
     * @brief Save complete processing report
     */
    void saveProcessingReport(const ProcessingMetrics& metrics);

    // ========== UTILITY METHODS ==========

    /**
     * @brief Draw epipolar lines on rectified image pair
     */
    static cv::Mat drawEpipolarLines(
        const cv::Mat& leftRect,
        const cv::Mat& rightRect,
        int lineSpacing = 40);

    /**
     * @brief Create depth histogram visualization
     */
    static cv::Mat createDepthHistogram(
        const cv::Mat& depthMap,
        int histHeight = 400,
        int histWidth = 800);

    /**
     * @brief Apply colormap to disparity/depth (JET or TURBO)
     */
    static cv::Mat applyColorMap(
        const cv::Mat& input,
        int colormapType = 2); // cv::COLORMAP_JET

private:
    core::Logger* logger_;
    Config config_;
    std::string currentSessionDir_;

    /**
     * @brief Ensure current session directory exists
     */
    bool ensureSessionDir();

    /**
     * @brief Generate timestamped session name
     */
    static std::string generateSessionName();
};

} // namespace stereo
} // namespace unlook

#endif // UNLOOK_STEREO_DEBUG_OUTPUT_MANAGER_HPP
