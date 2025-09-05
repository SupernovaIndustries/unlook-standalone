/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Hardware Synchronized Camera Capture
 * Implementation following libcamera-sync-fix API patterns for stereo capture
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <functional>
#include <chrono>

#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

namespace unlook {
namespace camera {

/**
 * Hardware Synchronized Stereo Capture System
 * 
 * Implements proper libcamera-sync-fix request lifecycle pattern:
 * - Camera 1 = LEFT/MASTER starts first  
 * - Camera 0 = RIGHT/SLAVE follows master
 * - Request pool with proper reuse pattern
 * - Asynchronous callback handling
 * - Hardware sync via XVS/XHS pins (physical sync handled by hardware)
 */
class HardwareSyncCapture {
public:
    struct StereoFrame {
        cv::Mat left_image;
        cv::Mat right_image;
        uint64_t left_timestamp_ns;
        uint64_t right_timestamp_ns;
        double sync_error_ms;
    };

    using FrameCallback = std::function<void(const StereoFrame&)>;

    /**
     * Camera roles matching hardware configuration
     * Camera 1 (/base/soc/i2c0mux/i2c@1/imx296@1a) = LEFT/MASTER
     * Camera 0 (/base/soc/i2c0mux/i2c@0/imx296@1a) = RIGHT/SLAVE
     */
    enum class CameraRole {
        MASTER,  // LEFT camera - generates sync signals
        SLAVE    // RIGHT camera - follows sync signals
    };

    struct CameraConfig {
        int width;
        int height; 
        libcamera::PixelFormat format;
        unsigned int buffer_count;
        
        CameraConfig() 
            : width(1456)
            , height(1088)
            , format(libcamera::formats::SBGGR10)
            , buffer_count(4) {}
    };

    HardwareSyncCapture();
    ~HardwareSyncCapture();

    /**
     * Initialize synchronized camera system
     * Must be called before start()
     */
    bool initialize(const CameraConfig& config = CameraConfig{});

    /**
     * Start synchronized capture following hardware sync pattern:
     * 1. Start MASTER camera first
     * 2. Wait 50ms for master stabilization  
     * 3. Start SLAVE camera
     * 4. Begin request pool cycling
     */
    bool start();

    /**
     * Stop synchronized capture
     */
    void stop();

    /**
     * Set callback for captured stereo frames
     * Called asynchronously when synchronized frame pair is ready
     */
    void setFrameCallback(FrameCallback callback);

    /**
     * Capture single synchronized frame pair (blocking)
     * Returns false if capture fails or times out
     */
    bool captureSingle(StereoFrame& frame, int timeout_ms = 1000);

    /**
     * Get current synchronization statistics
     */
    struct SyncStats {
        uint32_t frames_captured = 0;
        uint32_t sync_errors = 0;
        double max_sync_error_ms = 0.0;
        double avg_sync_error_ms = 0.0;
        double measured_fps = 0.0;
    };
    
    SyncStats getStats() const;

    bool isRunning() const { return running_; }

private:
    // libcamera camera instances
    std::shared_ptr<libcamera::Camera> master_camera_;  // Camera 1 = LEFT
    std::shared_ptr<libcamera::Camera> slave_camera_;   // Camera 0 = RIGHT
    
    // libcamera infrastructure
    std::unique_ptr<libcamera::CameraManager> camera_manager_;
    std::unique_ptr<libcamera::FrameBufferAllocator> master_allocator_;
    std::unique_ptr<libcamera::FrameBufferAllocator> slave_allocator_;
    
    // Request pools (following cam app pattern)
    std::vector<std::unique_ptr<libcamera::Request>> master_requests_;
    std::vector<std::unique_ptr<libcamera::Request>> slave_requests_;
    
    // Stream configurations
    std::unique_ptr<libcamera::CameraConfiguration> master_config_;
    std::unique_ptr<libcamera::CameraConfiguration> slave_config_;
    
    // Synchronization state
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
    
    // Frame callback and synchronization
    FrameCallback frame_callback_;
    std::mutex frame_mutex_;
    std::condition_variable frame_condition_;
    
    // Pending frame data for synchronization
    struct PendingFrame {
        cv::Mat image;
        uint64_t timestamp_ns;
        bool ready = false;
    };
    
    PendingFrame pending_master_;
    PendingFrame pending_slave_;
    
    // Statistics
    mutable std::mutex stats_mutex_;
    SyncStats stats_;
    uint64_t last_timestamp_ns_ = 0;
    
    // Auto-exposure control
    struct ExposureControl {
        int32_t exposure_time_us = 10000;  // 10ms default
        float analogue_gain = 1.0f;        // 1.0x default
        float target_brightness = 80.0f;    // REDUCED: Target avg brightness for indoor (0-255)
        float brightness_tolerance = 10.0f; // Acceptable brightness range
        
        // Exposure limits for IMX296
        int32_t min_exposure_us = 100;     // 100us minimum
        int32_t max_exposure_us = 33000;   // 33ms maximum (for 30fps)
        float min_gain = 1.0f;             // 1x minimum gain
        float max_gain = 16.0f;            // 16x maximum gain (sensor limit)
        
        // Adaptive parameters
        float kp = 0.3f;  // Proportional gain for exposure adjustment
        float ki = 0.05f; // Integral gain for steady-state error
        float integral_error = 0.0f;
        
        // Noise reduction thresholds
        float high_gain_threshold = 4.0f;  // Above this gain, apply noise reduction
        bool apply_noise_reduction = false;
    };
    
    ExposureControl master_exposure_;
    ExposureControl slave_exposure_;
    std::mutex exposure_mutex_;
    
    // Private implementation methods
    bool initializeCameras(const CameraConfig& config);
    bool configureAutoExposure();
    bool createRequestPools();
    bool startCameraSequence();
    
    // libcamera callback handlers (following cam app async pattern)
    void onMasterRequestComplete(libcamera::Request* request);
    void onSlaveRequestComplete(libcamera::Request* request);
    
    // Request processing (deferred to avoid blocking camera threads)
    void processMasterRequest(libcamera::Request* request);
    void processSlaveRequest(libcamera::Request* request);
    
    // Frame synchronization logic
    void tryFrameSync();
    void updateStats(double sync_error_ms, double fps);
    
    // Utility methods
    cv::Mat convertBufferToMat(libcamera::FrameBuffer* buffer, 
                               const libcamera::StreamConfiguration& config);
    double calculateSyncError(uint64_t master_ts, uint64_t slave_ts);
    
    // Image conversion methods
    cv::Mat convertBufferToMatGreenOnly(libcamera::FrameBuffer* buffer, 
                                        const libcamera::StreamConfiguration& config);
    
    // Auto-exposure methods
    void updateAutoExposure(const cv::Mat& image, ExposureControl& exposure, 
                           const std::string& camera_name);
    void applyExposureControls(libcamera::Request* request, 
                              const ExposureControl& exposure);
    float calculateImageBrightness(const cv::Mat& image);
    cv::Mat applyNoiseReduction(const cv::Mat& image, float gain_level);
    
    // NEON-optimized image processing methods
    void unpackSBGGR10_NEON(const uint8_t* src, uint16_t* dst, int width, int height);
    cv::Mat fastDemosaicNEON(const cv::Mat& bayer, int pattern);
};

} // namespace camera
} // namespace unlook