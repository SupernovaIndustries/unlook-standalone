#pragma once

#include "unlook/core/types.hpp"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

// Forward declaration to avoid Qt/libcamera conflict
namespace unlook {
namespace camera {
class HardwareSyncCapture;
}
}

namespace unlook {
namespace camera {
namespace gui {

/**
 * @brief Thread-safe camera system manager for dual IMX296 cameras (GUI version)
 *
 * This class manages both cameras with hardware synchronization support.
 * It provides a single shared instance that can be safely used across
 * multiple GUI components.
 */
class CameraSystem {
public:
    /**
     * @brief Get the singleton instance of the camera system
     */
    static std::shared_ptr<CameraSystem> getInstance();
    
    /**
     * @brief Destructor - cleanup resources
     */
    ~CameraSystem();

    // Non-copyable, non-movable
    CameraSystem(const CameraSystem&) = delete;
    CameraSystem& operator=(const CameraSystem&) = delete;
    CameraSystem(CameraSystem&&) = delete;
    CameraSystem& operator=(CameraSystem&&) = delete;

    /**
     * @brief Initialize the camera system
     * @return true if successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Shutdown the camera system
     */
    void shutdown();
    
    /**
     * @brief Check if the system is initialized and ready
     */
    bool isReady() const;
    
    /**
     * @brief Get current camera states
     */
    core::CameraState getCameraState(core::CameraId camera_id) const;
    
    /**
     * @brief Configure camera settings
     */
    bool configureCameras(const core::CameraConfig& left_config, 
                         const core::CameraConfig& right_config);
    
    /**
     * @brief Start continuous capture
     * @param frame_callback Callback for each stereo frame pair
     */
    bool startCapture(core::StereoFrameCallback frame_callback);
    
    /**
     * @brief Stop continuous capture
     */
    void stopCapture();
    
    /**
     * @brief Capture a single stereo frame pair
     * @return Stereo frame pair, check synchronized flag for validity
     */
    core::StereoFramePair captureSingle();
    
    /**
     * @brief Get current camera configurations
     */
    core::CameraConfig getCameraConfig(core::CameraId camera_id) const;
    
    /**
     * @brief Enable/disable auto exposure
     */
    bool setAutoExposure(core::CameraId camera_id, bool enabled);
    
    /**
     * @brief Set manual exposure time in microseconds
     */
    bool setExposureTime(core::CameraId camera_id, double exposure_us);
    
    /**
     * @brief Enable/disable auto gain
     */
    bool setAutoGain(core::CameraId camera_id, bool enabled);
    
    /**
     * @brief Set manual gain value
     */
    bool setGain(core::CameraId camera_id, double gain);
    
    /**
     * @brief Set target frame rate
     */
    bool setFrameRate(double fps);
    
    /**
     * @brief Get current frame rate
     */
    double getCurrentFrameRate() const;
    
    /**
     * @brief Get camera information string
     */
    std::string getCameraInfo(core::CameraId camera_id) const;
    
    /**
     * @brief Check hardware synchronization status
     */
    bool isHardwareSyncEnabled() const;
    
    /**
     * @brief Get synchronization error statistics
     */
    double getAverageSyncError() const;
    
    /**
     * @brief Set error callback for system errors
     */
    void setErrorCallback(core::ErrorCallback callback);

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    CameraSystem();
    
    // Static instance for singleton
    static std::shared_ptr<CameraSystem> instance_;
    static std::mutex instance_mutex_;

    // State management
    std::atomic<bool> capture_running_;
    std::atomic<bool> system_ready_;
    
    // Callbacks
    core::StereoFrameCallback frame_callback_;
    core::ErrorCallback error_callback_;
    
    // Configuration
    core::CameraConfig left_config_;
    core::CameraConfig right_config_;
    
    // State tracking
    mutable std::mutex state_mutex_;
    std::atomic<core::CameraState> left_state_;
    std::atomic<core::CameraState> right_state_;
    
    // Synchronization statistics
    mutable std::mutex stats_mutex_;
    double total_sync_error_ms_;
    size_t sync_sample_count_;
    
    // Hardware synchronized capture system
    std::unique_ptr<unlook::camera::HardwareSyncCapture> hardware_sync_capture_;
};

} // namespace gui
} // namespace camera
} // namespace unlook