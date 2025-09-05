#pragma once

#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>

/**
 * @file camera_system.h
 * @brief Camera system API for synchronized stereo camera control
 */

namespace unlook {
namespace api {

/**
 * @brief Frame capture callback function type
 * 
 * Called when new synchronized frames are available.
 * Frames are provided as OpenCV Mat objects.
 * 
 * @param left_frame Left camera frame (Camera 1 = LEFT/MASTER)
 * @param right_frame Right camera frame (Camera 0 = RIGHT/SLAVE)
 * @param timestamp_us Frame timestamp in microseconds
 * @param user_data User-provided data pointer
 */
using FrameCallback = std::function<void(const cv::Mat& left_frame, 
                                       const cv::Mat& right_frame,
                                       uint64_t timestamp_us,
                                       void* user_data)>;

/**
 * @brief Camera exposure modes
 */
enum class ExposureMode {
    MANUAL,    ///< Manual exposure control
    AUTO,      ///< Automatic exposure control
    FIXED      ///< Fixed exposure optimized for 3D scanning
};

/**
 * @brief Camera gain modes
 */
enum class GainMode {
    MANUAL,    ///< Manual gain control
    AUTO,      ///< Automatic gain control
    FIXED      ///< Fixed gain optimized for 3D scanning
};

/**
 * @brief Camera system API for synchronized stereo capture
 * 
 * Provides hardware-synchronized stereo camera control with <1ms precision.
 * Manages IMX296 sensors with hardware XVS/XHS synchronization.
 * 
 * Camera mapping (from scanner perspective):
 * - Camera 1 = LEFT/MASTER (/base/soc/i2c0mux/i2c@1/imx296@1a)
 * - Camera 0 = RIGHT/SLAVE (/base/soc/i2c0mux/i2c@0/imx296@1a)
 */
class CameraSystem {
public:
    /**
     * @brief Constructor
     */
    CameraSystem();
    
    /**
     * @brief Destructor - releases camera resources
     */
    ~CameraSystem();
    
    // Non-copyable, non-movable (hardware resource)
    CameraSystem(const CameraSystem&) = delete;
    CameraSystem& operator=(const CameraSystem&) = delete;
    CameraSystem(CameraSystem&&) = delete;
    CameraSystem& operator=(CameraSystem&&) = delete;
    
    /**
     * @brief Initialize camera system
     * 
     * Detects cameras, establishes hardware synchronization,
     * and configures for stereo operation.
     * 
     * @param config Camera configuration parameters
     * @return ResultCode indicating success or failure
     */
    core::ResultCode initialize(const core::CameraConfig& config = {});
    
    /**
     * @brief Shutdown camera system
     * 
     * Releases camera resources and stops all capture operations.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode shutdown();
    
    /**
     * @brief Check if camera system is initialized
     * @return True if cameras are initialized and synchronized
     */
    bool isInitialized() const;
    
    /**
     * @brief Get hardware synchronization status
     * @return Current synchronization status
     */
    core::SyncStatus getSyncStatus() const;
    
    /**
     * @brief Start continuous frame capture
     * 
     * Begins synchronized frame capture at configured frame rate.
     * Frames are delivered via frame callback if set.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode startCapture();
    
    /**
     * @brief Stop continuous frame capture
     * 
     * Stops synchronized frame capture and releases frame buffers.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode stopCapture();
    
    /**
     * @brief Check if capture is active
     * @return True if actively capturing frames
     */
    bool isCapturing() const;
    
    /**
     * @brief Capture single synchronized frame pair
     * 
     * Captures one synchronized frame pair with precise timing.
     * Blocks until frames are available or timeout occurs.
     * 
     * @param left_frame Output left camera frame
     * @param right_frame Output right camera frame
     * @param timeout_ms Capture timeout in milliseconds
     * @return ResultCode indicating success or failure
     */
    core::ResultCode captureSingleFrame(cv::Mat& left_frame,
                                       cv::Mat& right_frame,
                                       uint32_t timeout_ms = 5000);
    
    /**
     * @brief Set frame capture callback
     * 
     * Callback is invoked for each synchronized frame pair during
     * continuous capture mode. Executed on capture thread.
     * 
     * @param callback Frame callback function
     * @param user_data Optional user data passed to callback
     */
    void setFrameCallback(FrameCallback callback, void* user_data = nullptr);
    
    /**
     * @brief Set camera exposure
     * 
     * Controls exposure time for both cameras simultaneously.
     * 
     * @param exposure_us Exposure time in microseconds (1-100000)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setExposure(uint32_t exposure_us);
    
    /**
     * @brief Get current exposure setting
     * @return Current exposure time in microseconds
     */
    uint32_t getExposure() const;
    
    /**
     * @brief Set camera gain
     * 
     * Controls analog gain for both cameras simultaneously.
     * 
     * @param gain Gain value (1.0-16.0)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setGain(float gain);
    
    /**
     * @brief Get current gain setting
     * @return Current gain value
     */
    float getGain() const;
    
    /**
     * @brief Set exposure mode
     * 
     * Controls automatic or manual exposure adjustment.
     * 
     * @param mode Exposure control mode
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setExposureMode(ExposureMode mode);
    
    /**
     * @brief Get current exposure mode
     * @return Current exposure control mode
     */
    ExposureMode getExposureMode() const;
    
    /**
     * @brief Set gain mode
     * 
     * Controls automatic or manual gain adjustment.
     * 
     * @param mode Gain control mode
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setGainMode(GainMode mode);
    
    /**
     * @brief Get current gain mode
     * @return Current gain control mode
     */
    GainMode getGainMode() const;
    
    /**
     * @brief Swap left/right camera assignment
     * 
     * Useful for different mounting orientations or
     * correcting camera mapping issues.
     * 
     * @param swapped True to swap L/R, false for normal mapping
     * @return ResultCode indicating success or failure
     */
    core::ResultCode setLRSwap(bool swapped);
    
    /**
     * @brief Check if L/R cameras are swapped
     * @return True if cameras are swapped
     */
    bool isLRSwapped() const;
    
    /**
     * @brief Get current camera configuration
     * @return Current camera configuration parameters
     */
    core::CameraConfig getCurrentConfig() const;
    
    /**
     * @brief Update camera configuration
     * 
     * Updates camera settings. Some changes may require
     * reinitialization to take effect.
     * 
     * @param config New camera configuration
     * @return ResultCode indicating success or failure
     */
    core::ResultCode updateConfig(const core::CameraConfig& config);
    
    /**
     * @brief Get camera temperature (if available)
     * 
     * Returns sensor temperature for thermal monitoring.
     * 
     * @param left_temp Output left camera temperature (°C)
     * @param right_temp Output right camera temperature (°C)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode getCameraTemperature(float& left_temp, float& right_temp) const;
    
    /**
     * @brief Get frame statistics
     * 
     * Returns capture statistics for monitoring performance.
     * 
     * @param frames_captured Total frames captured since start
     * @param frames_dropped Total frames dropped due to processing delays
     * @param avg_frame_rate Average frame rate (fps)
     * @return ResultCode indicating success or failure
     */
    core::ResultCode getFrameStats(uint64_t& frames_captured,
                                  uint64_t& frames_dropped,
                                  double& avg_frame_rate) const;
    
    /**
     * @brief Reset frame statistics
     */
    void resetFrameStats();
    
    /**
     * @brief Get last error message
     * @return Detailed error information
     */
    std::string getLastError() const;

private:
    // Implementation details hidden in private section
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace api
} // namespace unlook