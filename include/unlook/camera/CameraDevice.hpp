#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <libcamera/libcamera.h>
#include <opencv2/core.hpp>

namespace unlook {
namespace camera {

/**
 * Individual camera device wrapper for libcamera
 * 
 * This class provides a thread-safe interface to a single camera,
 * handling configuration, capture, and control operations.
 */
class CameraDevice {
public:
    /**
     * Camera role in stereo system
     */
    enum class Role {
        MASTER,  // Left camera with sync output
        SLAVE    // Right camera with sync input
    };
    
    /**
     * Camera parameters
     */
    struct Parameters {
        double exposureTime = 10000.0;  // microseconds
        double analogGain = 1.0;
        double digitalGain = 1.0;
        bool autoExposure = false;
        bool autoGain = false;
        int frameRate = 30;
    };
    
    /**
     * Frame metadata
     */
    struct FrameMetadata {
        uint64_t timestampNs;
        uint64_t frameNumber;
        double exposureTime;
        double analogGain;
        double digitalGain;
        uint64_t sensorTimestamp;
    };
    
    /**
     * Constructor
     * @param cameraId Camera index (0 or 1)
     * @param role Master or Slave role
     */
    CameraDevice(int cameraId, Role role);
    
    /**
     * Destructor
     */
    ~CameraDevice();
    
    /**
     * Initialize camera
     */
    bool initialize();
    
    /**
     * Start streaming
     */
    bool start();
    
    /**
     * Stop streaming
     */
    void stop();
    
    /**
     * Capture single frame
     * @param frame Output OpenCV Mat (will be grayscale)
     * @param metadata Frame metadata
     * @param timeoutMs Timeout in milliseconds
     */
    bool captureFrame(cv::Mat& frame, FrameMetadata& metadata, int timeoutMs = 1000);
    
    /**
     * Queue capture request (async)
     */
    bool queueRequest();
    
    /**
     * Wait for completed request
     */
    bool waitForFrame(cv::Mat& frame, FrameMetadata& metadata, int timeoutMs = 1000);
    
    // Parameter control
    
    /**
     * Set exposure time in microseconds
     */
    bool setExposure(double exposureUs);
    
    /**
     * Set analog gain
     */
    bool setAnalogGain(double gain);
    
    /**
     * Set digital gain
     */
    bool setDigitalGain(double gain);
    
    /**
     * Enable/disable auto exposure
     */
    bool setAutoExposure(bool enable);
    
    /**
     * Enable/disable auto gain
     */
    bool setAutoGain(bool enable);
    
    /**
     * Set frame rate
     */
    bool setFrameRate(int fps);
    
    /**
     * Get current parameters
     */
    Parameters getParameters() const;
    
    /**
     * Apply parameters
     */
    bool applyParameters(const Parameters& params);
    
    // Status
    
    /**
     * Check if initialized
     */
    bool isInitialized() const { return initialized_; }
    
    /**
     * Check if streaming
     */
    bool isStreaming() const { return streaming_; }
    
    /**
     * Get camera ID
     */
    int getCameraId() const { return cameraId_; }
    
    /**
     * Get camera role
     */
    Role getRole() const { return role_; }
    
    /**
     * Get camera model
     */
    std::string getModel() const { return cameraModel_; }
    
    /**
     * Get last error message
     */
    std::string getLastError() const { return lastError_; }
    
    // Hardware sync configuration
    
    /**
     * Configure hardware sync pins
     */
    bool configureHardwareSync();
    
    /**
     * Enable sync mode
     */
    bool enableSync(bool enable);
    
private:
    // libcamera event handler
    void requestComplete(libcamera::Request *request);
    
    // Convert libcamera buffer to OpenCV Mat
    bool bufferToMat(libcamera::FrameBuffer *buffer, cv::Mat& mat);
    
    // Extract metadata from completed request
    void extractMetadata(libcamera::Request *request, FrameMetadata& metadata);
    
    // Configure camera controls
    bool configureControls();
    
    // Allocate frame buffers
    bool allocateBuffers();
    
    // Release resources
    void releaseResources();
    
    // Member variables
    int cameraId_;
    Role role_;
    std::string cameraModel_;
    
    // libcamera objects
    std::unique_ptr<libcamera::CameraManager> cameraManager_;
    std::shared_ptr<libcamera::Camera> camera_;
    std::unique_ptr<libcamera::CameraConfiguration> config_;
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator_;
    std::vector<std::unique_ptr<libcamera::Request>> requests_;
    libcamera::Stream *stream_ = nullptr;
    
    // Control settings
    Parameters parameters_;
    
    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> streaming_{false};
    
    // Synchronization
    mutable std::mutex paramMutex_;
    std::mutex captureMutex_;
    std::condition_variable frameReady_;
    
    // Frame handling
    libcamera::Request *completedRequest_ = nullptr;
    cv::Mat latestFrame_;
    FrameMetadata latestMetadata_;
    
    // Error handling
    mutable std::string lastError_;
    
    // Statistics
    std::atomic<uint64_t> framesCaptu_ = {0};
    std::atomic<uint64_t> framesDropped_{0};
};

} // namespace camera
} // namespace unlook