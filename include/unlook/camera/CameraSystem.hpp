#pragma once

#include "unlook/core/types.hpp"
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <opencv2/core.hpp>

namespace unlook {

// Forward declarations for real-time pipeline
namespace realtime {
    class RealtimePipeline;
}

namespace camera {

// Forward declarations
class LibcameraSyncDevice;
class CameraSynchronizer;
class AutoExposure;
class HardwareSyncCapture;

/**
 * Camera identifiers for the stereo system
 * CRITICAL: These mappings are FIXED per hardware configuration
 */
enum class CameraId {
    LEFT_MASTER = 1,   // Camera 1: /base/soc/i2c0mux/i2c@1/imx296@1a
    RIGHT_SLAVE = 0    // Camera 0: /base/soc/i2c0mux/i2c@0/imx296@1a
};

/**
 * Camera configuration parameters
 */
struct CameraConfig {
    // Resolution (fixed for IMX296)
    int width = 1456;
    int height = 1088;
    
    // Frame rate control
    double targetFps = 30.0;
    
    // Exposure control
    bool autoExposure = true;
    double exposureTime = 10000.0;  // microseconds
    double analogGain = 1.0;
    
    // Hardware sync
    bool enableSync = true;
    double syncToleranceMs = 1.0;  // Max sync error tolerance
    
    // Baseline (from calibration)
    double baseline = 70.017;  // mm
};

/**
 * Stereo frame data with metadata
 */
struct StereoFrame {
    cv::Mat leftImage;
    cv::Mat rightImage;
    
    // Timestamps
    uint64_t leftTimestampNs;
    uint64_t rightTimestampNs;
    
    // Sync metadata
    double syncErrorMs;
    bool isSynchronized;
    
    // Frame numbers
    uint64_t leftFrameNumber;
    uint64_t rightFrameNumber;
    
    // Exposure metadata
    double leftExposure;
    double rightExposure;
    double leftGain;
    double rightGain;
};

/**
 * Camera system status
 */
struct CameraStatus {
    bool isInitialized = false;
    bool leftCameraReady = false;
    bool rightCameraReady = false;
    bool isSynchronized = false;
    
    double currentFps = 0.0;
    uint64_t totalFramesCaptured = 0;
    uint64_t syncErrors = 0;
    
    double avgSyncErrorMs = 0.0;
    double maxSyncErrorMs = 0.0;
    
    std::string errorMessage;
};

/**
 * Main camera system class - Singleton pattern for shared access
 * 
 * This class manages the stereo camera pair with hardware synchronization,
 * providing thread-safe access to camera controls and frame capture.
 */
class CameraSystem {
public:
    using FrameCallback = std::function<void(const StereoFrame&)>;
    // Use core::ErrorCallback instead of local definition
    
    /**
     * Get singleton instance
     */
    static std::shared_ptr<CameraSystem> getInstance();
    
    /**
     * Initialize camera system with configuration
     */
    bool initialize();
    bool initialize(const CameraConfig& config);
    
    /**
     * Shutdown camera system
     */
    void shutdown();
    
    /**
     * Start synchronized capture
     */
    bool startCapture();
    
    /**
     * Stop capture
     */
    void stopCapture();
    
    /**
     * Capture single synchronized frame pair
     */
    bool captureStereoFrame(StereoFrame& frame, int timeoutMs = 1000);
    
    /**
     * Set continuous frame callback
     */
    void setFrameCallback(FrameCallback callback);
    
    /**
     * Set error callback
     */
    void setErrorCallback(core::ErrorCallback callback);
    
    // Camera controls
    
    /**
     * Set exposure time (microseconds)
     */
    bool setExposure(double exposureUs);
    
    /**
     * Set analog gain
     */
    bool setGain(double gain);
    
    /**
     * Enable/disable auto exposure (both cameras)
     */
    void setAutoExposure(bool enable);
    
    /**
     * Set target FPS
     */
    bool setTargetFps(double fps);
    
    /**
     * Swap camera mapping (LEFT <-> RIGHT)
     */
    void swapCameras();
    
    // Status and monitoring
    
    /**
     * Get current system status
     */
    CameraStatus getStatus() const;
    
    /**
     * Get current configuration
     */
    CameraConfig getConfig() const;
    
    /**
     * Check if system is initialized
     */
    bool isInitialized() const;
    
    /**
     * Check if capturing
     */
    bool isCapturing() const;
    
    /**
     * Get synchronization precision statistics
     */
    void getSyncStats(double& avgErrorMs, double& maxErrorMs, uint64_t& errorCount) const;
    
    // Calibration helpers
    
    /**
     * Get camera baseline in mm
     */
    double getBaseline() const { return config_.baseline; }
    
    /**
     * Set camera baseline in mm
     */
    void setBaseline(double baselineMm) { config_.baseline = baselineMm; }
    
    // New methods required by GUI
    bool isReady() const;
    core::CameraState getCameraState(core::CameraId camera_id) const;
    core::CameraConfig getCameraConfig(core::CameraId camera_id) const;
    bool setAutoExposure(core::CameraId camera_id, bool enabled);
    bool setExposureTime(core::CameraId camera_id, double exposure_us);
    bool setGain(core::CameraId camera_id, double gain);
    bool setAutoGain(core::CameraId camera_id, bool enabled);
    bool setFrameRate(double fps);
    double getCurrentFrameRate() const;
    std::string getCameraInfo(core::CameraId camera_id) const;
    bool isHardwareSyncEnabled() const;
    double getAverageSyncError() const;
    core::StereoFramePair captureSingle();
    bool configureCameras(const core::CameraConfig& left_config, const core::CameraConfig& right_config);
    bool startCapture(core::StereoFrameCallback frame_callback);
    
private:
    CameraSystem();
    ~CameraSystem();
    
    // Delete copy/move constructors (singleton)
    CameraSystem(const CameraSystem&) = delete;
    CameraSystem& operator=(const CameraSystem&) = delete;
    CameraSystem(CameraSystem&&) = delete;
    CameraSystem& operator=(CameraSystem&&) = delete;
    
    // Internal methods
    bool initializeCameras();
    bool configureSynchronization();
    void captureThread();
    void updateSyncStats(double errorMs);
    
    // Member variables
    CameraConfig config_;
    CameraStatus status_;
    
    // Real-time processing pipeline for high performance
    // std::unique_ptr<realtime::RealtimePipeline> realtime_pipeline_;  // Temporarily disabled
    
    // Hardware-synchronized capture system
    std::unique_ptr<HardwareSyncCapture> hardware_sync_capture_;
    
    // Legacy components (deprecated)
    std::unique_ptr<LibcameraSyncDevice> leftCamera_;
    std::unique_ptr<LibcameraSyncDevice> rightCamera_;
    std::unique_ptr<CameraSynchronizer> synchronizer_;
    std::unique_ptr<AutoExposure> autoExposure_;
    
    // Thread management
    std::thread captureThread_;
    std::atomic<bool> captureRunning_;
    std::atomic<bool> shouldStop_;
    
    // Callbacks
    FrameCallback frameCallback_;
    core::ErrorCallback errorCallback_;
    
    // Synchronization
    mutable std::mutex statusMutex_;
    mutable std::mutex configMutex_;
    std::mutex callbackMutex_;
    
    // Statistics
    std::atomic<uint64_t> totalFrames_;
    std::atomic<uint64_t> syncErrors_;
    std::atomic<double> avgSyncError_;
    std::atomic<double> maxSyncError_;
    
    // Camera swap flag
    std::atomic<bool> camerasSwapped_;
    
    // Additional state tracking for GUI
    std::atomic<bool> system_ready_;
    std::atomic<core::CameraState> left_state_;
    std::atomic<core::CameraState> right_state_;
    core::CameraConfig left_config_;
    core::CameraConfig right_config_;
    core::StereoFrameCallback frame_callback_;
    
    // Static instance for singleton
    static std::shared_ptr<CameraSystem> instance_;
    static std::mutex instance_mutex_;
};

} // namespace camera
} // namespace unlook