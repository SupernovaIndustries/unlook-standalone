#pragma once

#include <libcamera/libcamera.h>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <map>
#include <opencv2/opencv.hpp>

namespace unlook {
namespace camera {

/**
 * LibcameraSyncDevice - Hardware synchronized camera device using libcamera-sync-fix
 * 
 * This class provides a proper implementation for IMX296 cameras with hardware
 * synchronization support through the third-party libcamera-sync-fix library.
 * 
 * Key features:
 * - Proper request lifecycle management (no "Request is not valid" errors)
 * - Hardware synchronization through XVS/XHS pins
 * - Master/slave camera coordination
 * - Thread-safe frame capture
 * - <1ms synchronization precision
 */
class LibcameraSyncDevice {
public:
    enum class Role {
        MASTER,  // Camera 1: /base/soc/i2c0mux/i2c@1/imx296@1a
        SLAVE    // Camera 0: /base/soc/i2c0mux/i2c@0/imx296@1a
    };
    
    struct CameraConfig {
        uint32_t width = 1456;
        uint32_t height = 1088;
        libcamera::PixelFormat format = libcamera::formats::SBGGR10;
        uint32_t buffer_count = 4;
        double exposure_time_us = 10000.0;
        double analog_gain = 1.0;
        double target_fps = 30.0;
        bool enable_sync = true;
    };
    
    struct FrameMetadata {
        uint64_t timestamp_ns = 0;
        uint64_t frame_number = 0;
        double exposure_time_us = 0.0;
        double analog_gain = 0.0;
        bool synchronized = false;
    };
    
    LibcameraSyncDevice(const std::string& camera_path, Role role);
    ~LibcameraSyncDevice();
    
    // Initialize camera with configuration
    bool initialize(const CameraConfig& config);
    
    // Start/stop streaming
    bool start();
    void stop();
    
    // Capture single frame
    bool captureFrame(cv::Mat& frame, FrameMetadata& metadata, uint32_t timeout_ms = 1000);
    
    // Queue a request (for continuous streaming)
    bool queueRequest();
    
    // Wait for next frame (for continuous streaming)
    bool waitForFrame(cv::Mat& frame, FrameMetadata& metadata, uint32_t timeout_ms = 100);
    
    // Camera control
    bool setExposure(double exposure_us);
    bool setGain(double gain);
    bool setAnalogGain(double gain) { return setGain(gain); }  // Alias for compatibility
    bool setFrameRate(double fps);
    
    // Get camera information
    std::string getCameraId() const { return camera_path_; }
    std::string getModel() const { return camera_model_; }
    std::string getCameraModel() const { return camera_model_; }  // Alias for compatibility
    Role getRole() const { return role_; }
    bool isInitialized() const { return initialized_; }
    bool isStreaming() const { return streaming_; }
    
    // Synchronization control
    bool enableHardwareSync(bool enable);
    bool isSynchronized() const { return sync_enabled_; }
    
    // Statistics
    uint64_t getFrameCount() const { return frame_count_; }
    uint64_t getDroppedFrames() const { return dropped_frames_; }
    double getCurrentFPS() const;
    
    // Error handling
    std::string getLastError() const { return last_error_; }
    
private:
    // Camera identification
    std::string camera_path_;
    std::string camera_model_;
    Role role_;
    
    // libcamera objects  
    std::shared_ptr<libcamera::CameraManager> camera_manager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    libcamera::Stream* stream_ = nullptr;
    
    // Request management
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    std::queue<libcamera::Request*> available_requests_;
    std::queue<libcamera::Request*> queued_requests_;
    std::map<libcamera::Request*, libcamera::FrameBuffer*> request_buffers_;
    
    // Frame buffers
    std::map<libcamera::FrameBuffer*, std::vector<uint8_t>> frame_buffers_;
    
    // State management
    std::atomic<bool> initialized_{false};
    std::atomic<bool> streaming_{false};
    std::atomic<bool> sync_enabled_{false};
    
    // Configuration
    CameraConfig config_params_;
    
    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> dropped_frames_{0};
    std::chrono::steady_clock::time_point start_time_;
    
    // Thread safety
    mutable std::mutex mutex_;
    std::condition_variable frame_ready_cv_;
    bool frame_ready_ = false;
    cv::Mat latest_frame_;
    FrameMetadata latest_metadata_;
    
    // Error handling
    mutable std::string last_error_;
    
    // Private methods
    bool allocateBuffers();
    bool createRequests();
    bool configureCamera();
    bool applyControls(libcamera::Request* request);
    void processCompletedRequest(libcamera::Request* request);
    void handleRequestComplete(libcamera::Request* request);
    bool mapFrameBuffer(libcamera::FrameBuffer* buffer);
    cv::Mat convertFrameToMat(const uint8_t* data, size_t size);
    
    // Request lifecycle management
    void recycleRequest(libcamera::Request* request);
    libcamera::Request* getAvailableRequest();
    
    // Camera manager singleton (shared across all devices)
    static std::shared_ptr<libcamera::CameraManager> getSharedCameraManager();
    
    /**
     * @brief Cleanup shared CameraManager resources
     * 
     * Called during application shutdown to ensure proper GLib object cleanup
     * and prevent GLib-GObject-CRITICAL errors during Qt application teardown.
     * This is critical for industrial-grade reliability.
     */
    static void cleanupSharedCameraManager();
    
    static std::shared_ptr<libcamera::CameraManager> shared_manager_;
    static std::mutex shared_manager_mutex_;
};

} // namespace camera
} // namespace unlook