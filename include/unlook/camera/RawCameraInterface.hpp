#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <libcamera/libcamera.h>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <deque>

namespace unlook {
namespace camera {

/**
 * Raw Camera Interface for preserving VCSEL pattern detail
 *
 * This class provides direct access to raw Bayer data without preprocessing,
 * specifically optimized for VCSEL dot pattern preservation. It extracts the
 * Blue channel directly from SBGGR10 format for maximum IR sensitivity.
 *
 * Key features:
 * - Direct SBGGR10 raw data access
 * - Blue channel extraction for VCSEL IR detection
 * - No lossy compression or smoothing
 * - Variance computation for pattern validation
 * - 16-bit output support for full 10-bit data preservation
 */
class RawCameraInterface {
public:
    /**
     * Raw frame data structure
     */
    struct RawFrame {
        cv::Mat rawBayer;      // Full raw Bayer data (CV_16UC1)
        cv::Mat blueChannel;   // Extracted blue channel (CV_16UC1)
        cv::Mat blueChannel8;  // 8-bit version for SGBM (CV_8UC1)
        uint64_t timestampNs;
        uint64_t frameNumber;
        double variance;       // Computed variance for pattern validation
        double blueVariance;  // Blue channel variance

        // Metadata
        double exposureTime;   // microseconds
        double analogGain;
        double digitalGain;
    };

    /**
     * Processing options
     */
    struct ProcessingOptions {
        bool extractBlueOnly;     // Extract only blue channel
        bool compute16bit;        // Keep 16-bit precision
        bool computeVariance;     // Compute variance metrics
        bool skipDebayering;      // Skip all debayering
        bool saveBinaryDump;      // Save raw binary data
        std::string dumpPath;     // Path for binary dumps

        // Constructor with defaults
        ProcessingOptions()
            : extractBlueOnly(true),
              compute16bit(true),
              computeVariance(true),
              skipDebayering(true),
              saveBinaryDump(false),
              dumpPath("/tmp/") {}
    };

    /**
     * Constructor
     * @param cameraId Camera index (0 or 1)
     */
    explicit RawCameraInterface(int cameraId);

    /**
     * Destructor
     */
    ~RawCameraInterface();

    /**
     * Initialize camera for raw capture
     */
    bool initialize();

    /**
     * Start raw capture stream
     */
    bool start();

    /**
     * Stop capture stream
     */
    void stop();

    /**
     * Capture raw frame without preprocessing
     * @param frame Output raw frame data
     * @param options Processing options
     * @param timeoutMs Capture timeout
     */
    bool captureRawFrame(RawFrame& frame,
                        const ProcessingOptions& options = ProcessingOptions(),
                        int timeoutMs = 1000);

    /**
     * Extract blue channel from SBGGR10 raw data
     *
     * SBGGR10 Bayer pattern:
     *   B G B G ... (even rows)
     *   G R G R ... (odd rows)
     *
     * Blue pixels are at (row, col) where:
     * - row is even, col is even
     *
     * @param rawData Raw SBGGR10 packed data
     * @param width Image width
     * @param height Image height
     * @param output16 16-bit blue channel output
     * @param output8 8-bit blue channel output (optional)
     */
    static bool extractBlueChannel(const uint8_t* rawData,
                                   int width,
                                   int height,
                                   cv::Mat& output16,
                                   cv::Mat* output8 = nullptr);

    /**
     * Unpack SBGGR10 to 16-bit array
     * @param packedData SBGGR10 packed (4 pixels in 5 bytes)
     * @param width Image width
     * @param height Image height
     * @param unpacked Output 16-bit array
     */
    static void unpackSBGGR10(const uint8_t* packedData,
                              int width,
                              int height,
                              std::vector<uint16_t>& unpacked);

    /**
     * Compute image variance for pattern validation
     * @param image Input image
     * @return Variance value
     */
    static double computeVariance(const cv::Mat& image);

    /**
     * Compute local variance map to visualize VCSEL patterns
     * @param image Input image
     * @param windowSize Size of local window
     * @return Variance map
     */
    static cv::Mat computeLocalVariance(const cv::Mat& image, int windowSize = 7);

    /**
     * Save raw binary dump for analysis
     * @param data Raw data pointer
     * @param size Data size in bytes
     * @param filename Output filename
     */
    static bool saveRawBinaryDump(const void* data, size_t size, const std::string& filename);

    /**
     * Convert 16-bit to 8-bit with optimal scaling
     * Preserves dynamic range for VCSEL patterns
     */
    static void convert16to8bit(const cv::Mat& src16, cv::Mat& dst8, bool autoScale = true);

    /**
     * Get processing statistics
     */
    struct Statistics {
        double avgVariance;
        double minVariance;
        double maxVariance;
        double avgBlueIntensity;
        int saturatedPixels;
        int underexposedPixels;
        double captureTimeMs;
        double processingTimeMs;
    };

    Statistics getStatistics() const { return stats_; }

    /**
     * Enable debug mode with verbose logging
     */
    void setDebugMode(bool enable) { debugMode_ = enable; }

    /**
     * Get last error message
     */
    std::string getLastError() const { return lastError_; }

private:
    // Request completed handler
    void requestComplete(libcamera::Request* request);

    // Direct libcamera buffer access without conversion
    bool accessRawBuffer(libcamera::FrameBuffer* buffer, RawFrame& frame);

    // Process raw frame with options
    void processRawFrame(const uint8_t* rawData, size_t dataSize,
                         RawFrame& frame, const ProcessingOptions& options);

    // Member variables
    int cameraId_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> streaming_{false};
    bool debugMode_{false};

    // libcamera objects
    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    libcamera::Stream* stream_{nullptr};

    // Thread safety
    mutable std::mutex captureMutex_;
    std::condition_variable frameReady_;
    libcamera::Request* completedRequest_{nullptr};

    // Statistics tracking
    Statistics stats_{};
    mutable std::mutex statsMutex_;

    // Error handling
    mutable std::string lastError_;

    // Frame counter
    std::atomic<uint64_t> frameCounter_{0};
};

/**
 * Stereo Raw Camera System for synchronized VCSEL capture
 */
class StereoRawCameraSystem {
public:
    /**
     * Synchronized stereo raw frame
     */
    struct StereoRawFrame {
        RawCameraInterface::RawFrame left;
        RawCameraInterface::RawFrame right;
        double syncErrorMs;
        bool synchronized;
    };

    /**
     * Constructor
     */
    StereoRawCameraSystem();

    /**
     * Initialize both cameras
     */
    bool initialize();

    /**
     * Start synchronized capture
     */
    bool start();

    /**
     * Stop capture
     */
    void stop();

    /**
     * Capture synchronized raw frames
     */
    bool captureStereoRaw(StereoRawFrame& stereoFrame,
                         const RawCameraInterface::ProcessingOptions& options =
                             RawCameraInterface::ProcessingOptions(),
                         int timeoutMs = 1000);

    /**
     * Get variance comparison between processed and raw
     */
    struct VarianceComparison {
        double leftRawVariance;
        double leftProcessedVariance;
        double rightRawVariance;
        double rightProcessedVariance;
        double varianceRatioLeft;   // raw/processed
        double varianceRatioRight;  // raw/processed
    };

    VarianceComparison getVarianceComparison() const { return varianceComp_; }

private:
    std::unique_ptr<RawCameraInterface> leftCamera_;
    std::unique_ptr<RawCameraInterface> rightCamera_;
    VarianceComparison varianceComp_{};
    mutable std::mutex compMutex_;
};

} // namespace camera
} // namespace unlook