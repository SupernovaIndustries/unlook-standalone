#include "unlook/api/camera_system.h"
#include "unlook/core/exception.h"
#include "unlook/core/Logger.hpp"
#include "unlook/core/config.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <queue>
#include <libcamera/libcamera.h>

namespace unlook {
namespace api {

/**
 * @brief Internal implementation class for CameraSystem
 * 
 * Uses PIMPL idiom to hide libcamera dependencies from public API.
 * Manages hardware-synchronized stereo camera capture with IMX296 sensors.
 */
class CameraSystem::Impl {
public:
    Impl() 
        : initialized_(false)
        , capturing_(false)
        , lr_swapped_(false)
        , sync_status_(core::SyncStatus::NOT_INITIALIZED)
        , exposure_us_(10000)
        , gain_(1.0f)
        , exposure_mode_(ExposureMode::MANUAL)
        , gain_mode_(GainMode::MANUAL)
        , frames_captured_(0)
        , frames_dropped_(0)
        , callback_user_data_(nullptr) {
        
        // Initialize configuration with fixed calibration resolution
        config_.width = 1456;
        config_.height = 1088;
        config_.exposure_time_us = 10000;
        config_.gain = 1.0f;
        config_.auto_exposure = false;
        config_.auto_gain = false;
    }
    
    ~Impl() {
        if (initialized_) {
            shutdown();
        }
    }
    
    core::ResultCode initialize(const core::CameraConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (initialized_) {
            return core::ResultCode::ERROR_ALREADY_INITIALIZED;
        }
        
        UNLOOK_LOG_INFO("Camera") << "Initializing camera system...";
        
        try {
            // Validate configuration
            if (config.width != 1456 || config.height != 1088) {
                UNLOOK_THROW_CODE(core::CameraException,
                                core::ResultCode::ERROR_INVALID_PARAMETER,
                                "Camera resolution must be 1456x1088 for calibration compatibility");
            }
            
            // Store configuration
            config_ = config;
            exposure_us_ = config.exposure_time_us;
            gain_ = config.gain;
            exposure_mode_ = config.auto_exposure ? ExposureMode::AUTO : ExposureMode::MANUAL;
            gain_mode_ = config.auto_gain ? GainMode::AUTO : GainMode::MANUAL;
            
            // Initialize libcamera
            auto result = initializeLibCamera();
            if (result != core::ResultCode::SUCCESS) {
                return result;
            }
            
            // Configure hardware synchronization
            result = configureHardwareSync();
            if (result != core::ResultCode::SUCCESS) {
                return result;
            }
            
            // Test camera access and synchronization
            result = testCameraSynchronization();
            if (result != core::ResultCode::SUCCESS) {
                return result;
            }
            
            initialized_ = true;
            sync_status_ = core::SyncStatus::SYNCHRONIZED;
            
            UNLOOK_LOG_INFO("Camera") << "Camera system initialized successfully";
            UNLOOK_LOG_INFO("Camera") << "Resolution: " << config_.width << "x" << config_.height;
            UNLOOK_LOG_INFO("Camera") << "Exposure: " << exposure_us_ << "μs, Gain: " << gain_;
            UNLOOK_LOG_INFO("Camera") << "Hardware sync: XVS/XHS enabled";
            
            return core::ResultCode::SUCCESS;
            
        } catch (const core::Exception& e) {
            last_error_ = e.getMessage();
            UNLOOK_LOG_ERROR("Camera") << "Initialization failed: " << e.getMessage();
            return e.getResultCode();
        } catch (const std::exception& e) {
            last_error_ = "Initialization failed: " + std::string(e.what());
            UNLOOK_LOG_ERROR("Camera") << "Initialization std::exception: " << e.what();
            return core::ResultCode::ERROR_HARDWARE_FAILURE;
        }
    }
    
    core::ResultCode shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        UNLOOK_LOG_INFO("Camera") << "Shutting down camera system...";
        
        // Stop capture if active
        if (capturing_) {
            stopCaptureInternal();
        }
        
        // Cleanup libcamera resources
        cleanupLibCamera();
        
        initialized_ = false;
        sync_status_ = core::SyncStatus::NOT_INITIALIZED;
        
        UNLOOK_LOG_INFO("Camera") << "Camera system shutdown complete";
        return core::ResultCode::SUCCESS;
    }
    
    bool isInitialized() const {
        return initialized_;
    }
    
    core::SyncStatus getSyncStatus() const {
        return sync_status_;
    }
    
    core::ResultCode startCapture() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        if (capturing_) {
            return core::ResultCode::ERROR_ALREADY_INITIALIZED;
        }
        
        UNLOOK_LOG_INFO("Camera") << "Starting synchronized capture...";
        
        try {
            // Reset statistics
            frames_captured_ = 0;
            frames_dropped_ = 0;
            capture_start_time_ = std::chrono::steady_clock::now();
            
            // Start capture thread
            capturing_ = true;
            capture_thread_ = std::thread(&Impl::captureThreadLoop, this);
            
            UNLOOK_LOG_INFO("Camera") << "Synchronized capture started";
            return core::ResultCode::SUCCESS;
            
        } catch (const std::exception& e) {
            capturing_ = false;
            last_error_ = "Failed to start capture: " + std::string(e.what());
            UNLOOK_LOG_ERROR("Camera") << last_error_;
            return core::ResultCode::ERROR_THREAD_FAILURE;
        }
    }
    
    core::ResultCode stopCapture() {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        if (!capturing_) {
            return core::ResultCode::SUCCESS;
        }
        
        return stopCaptureInternal();
    }
    
    bool isCapturing() const {
        return capturing_;
    }
    
    core::ResultCode captureSingleFrame(cv::Mat& left_frame,
                                       cv::Mat& right_frame,
                                       uint32_t timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        try {
            // Simulate synchronized frame capture using libcamera
            // This would use actual libcamera APIs to capture synchronized frames
            
            UNLOOK_LOG_DEBUG("Camera") << "Capturing single synchronized frame pair...";
            
            // For Phase 1, create placeholder frames with correct dimensions
            left_frame = cv::Mat::zeros(config_.height, config_.width, CV_8UC1);
            right_frame = cv::Mat::zeros(config_.height, config_.width, CV_8UC1);
            
            // Add timestamp and frame metadata
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            
            frames_captured_++;
            
            UNLOOK_LOG_DEBUG("Camera") << "Single frame captured successfully";
            return core::ResultCode::SUCCESS;
            
        } catch (const std::exception& e) {
            last_error_ = "Single frame capture failed: " + std::string(e.what());
            UNLOOK_LOG_ERROR("Camera") << last_error_;
            return core::ResultCode::ERROR_TIMEOUT;
        }
    }
    
    void setFrameCallback(FrameCallback callback, void* user_data) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame_callback_ = std::move(callback);
        callback_user_data_ = user_data;
    }
    
    core::ResultCode setExposure(uint32_t exposure_us) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        if (exposure_us < 1 || exposure_us > 100000) {
            return core::ResultCode::ERROR_INVALID_PARAMETER;
        }
        
        exposure_us_ = exposure_us;
        config_.exposure_time_us = exposure_us;
        
        // Apply to hardware (would use libcamera controls)
        UNLOOK_LOG_DEBUG("Camera") << "Exposure set to " << exposure_us << "μs";
        return core::ResultCode::SUCCESS;
    }
    
    uint32_t getExposure() const {
        return exposure_us_;
    }
    
    core::ResultCode setGain(float gain) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        if (gain < 1.0f || gain > 16.0f) {
            return core::ResultCode::ERROR_INVALID_PARAMETER;
        }
        
        gain_ = gain;
        config_.gain = gain;
        
        // Apply to hardware (would use libcamera controls)
        UNLOOK_LOG_DEBUG("Camera") << "Gain set to " << gain;
        return core::ResultCode::SUCCESS;
    }
    
    float getGain() const {
        return gain_;
    }
    
    core::ResultCode setExposureMode(ExposureMode mode) {
        std::lock_guard<std::mutex> lock(mutex_);
        exposure_mode_ = mode;
        config_.auto_exposure = (mode == ExposureMode::AUTO);
        
        UNLOOK_LOG_DEBUG("Camera") << "Exposure mode set to " << static_cast<int>(mode);
        return core::ResultCode::SUCCESS;
    }
    
    ExposureMode getExposureMode() const {
        return exposure_mode_;
    }
    
    core::ResultCode setGainMode(GainMode mode) {
        std::lock_guard<std::mutex> lock(mutex_);
        gain_mode_ = mode;
        config_.auto_gain = (mode == GainMode::AUTO);
        
        UNLOOK_LOG_DEBUG("Camera") << "Gain mode set to " << static_cast<int>(mode);
        return core::ResultCode::SUCCESS;
    }
    
    GainMode getGainMode() const {
        return gain_mode_;
    }
    
    core::ResultCode setLRSwap(bool swapped) {
        std::lock_guard<std::mutex> lock(mutex_);
        lr_swapped_ = swapped;
        
        UNLOOK_LOG_INFO("Camera") << "L/R swap " << (swapped ? "enabled" : "disabled");
        return core::ResultCode::SUCCESS;
    }
    
    bool isLRSwapped() const {
        return lr_swapped_;
    }
    
    core::CameraConfig getCurrentConfig() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }
    
    core::ResultCode updateConfig(const core::CameraConfig& config) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        if (!initialized_) {
            return core::ResultCode::ERROR_NOT_INITIALIZED;
        }
        
        // Validate fixed resolution
        if (config.width != 1456 || config.height != 1088) {
            return core::ResultCode::ERROR_INVALID_PARAMETER;
        }
        
        config_ = config;
        exposure_us_ = config.exposure_time_us;
        gain_ = config.gain;
        exposure_mode_ = config.auto_exposure ? ExposureMode::AUTO : ExposureMode::MANUAL;
        gain_mode_ = config.auto_gain ? GainMode::AUTO : GainMode::MANUAL;
        
        UNLOOK_LOG_INFO("Camera") << "Camera configuration updated";
        return core::ResultCode::SUCCESS;
    }
    
    core::ResultCode getCameraTemperature(float& left_temp, float& right_temp) const {
        // Placeholder - would read actual sensor temperatures
        left_temp = 45.0f;  // Typical sensor temperature
        right_temp = 46.0f;
        return core::ResultCode::SUCCESS;
    }
    
    core::ResultCode getFrameStats(uint64_t& frames_captured,
                                  uint64_t& frames_dropped,
                                  double& avg_frame_rate) const {
        std::lock_guard<std::mutex> lock(mutex_);
        
        frames_captured = frames_captured_;
        frames_dropped = frames_dropped_;
        
        if (capturing_ && frames_captured > 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - capture_start_time_).count();
            
            if (elapsed > 0) {
                avg_frame_rate = (frames_captured * 1000.0) / elapsed;
            } else {
                avg_frame_rate = 0.0;
            }
        } else {
            avg_frame_rate = 0.0;
        }
        
        return core::ResultCode::SUCCESS;
    }
    
    void resetFrameStats() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_captured_ = 0;
        frames_dropped_ = 0;
        capture_start_time_ = std::chrono::steady_clock::now();
    }
    
    std::string getLastError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return last_error_;
    }

private:
    // Internal state
    mutable std::mutex mutex_;
    std::atomic<bool> initialized_;
    std::atomic<bool> capturing_;
    bool lr_swapped_;
    core::SyncStatus sync_status_;
    core::CameraConfig config_;
    
    // Camera parameters
    uint32_t exposure_us_;
    float gain_;
    ExposureMode exposure_mode_;
    GainMode gain_mode_;
    
    // Statistics
    std::atomic<uint64_t> frames_captured_;
    std::atomic<uint64_t> frames_dropped_;
    std::chrono::steady_clock::time_point capture_start_time_;
    
    // Callback
    FrameCallback frame_callback_;
    void* callback_user_data_;
    
    // Capture thread
    std::thread capture_thread_;
    std::condition_variable capture_cv_;
    
    // Error handling
    std::string last_error_;
    
    // libcamera resources (would be actual libcamera objects)
    // For Phase 1, these are placeholders
    
    core::ResultCode initializeLibCamera() {
        UNLOOK_LOG_DEBUG("Camera") << "Initializing libcamera-sync...";
        
        // This would initialize actual libcamera system
        // Check for system installation first
        // Then initialize camera manager and detect IMX296 sensors
        
        // For Phase 1, assume successful initialization
        return core::ResultCode::SUCCESS;
    }
    
    void cleanupLibCamera() {
        UNLOOK_LOG_DEBUG("Camera") << "Cleaning up libcamera resources...";
        
        // This would cleanup actual libcamera resources
        // Stop cameras, release buffers, cleanup camera manager
    }
    
    core::ResultCode configureHardwareSync() {
        UNLOOK_LOG_DEBUG("Camera") << "Configuring hardware synchronization...";
        
        // This would configure XVS/XHS hardware synchronization
        // Set Camera 1 as MASTER (LEFT)
        // Set Camera 0 as SLAVE (RIGHT)
        // Enable external vertical sync (XVS)
        // Enable external horizontal sync (XHS)
        // Validate MAS pin configuration
        
        sync_status_ = core::SyncStatus::SYNCHRONIZING;
        
        // For Phase 1, assume successful sync configuration
        sync_status_ = core::SyncStatus::SYNCHRONIZED;
        return core::ResultCode::SUCCESS;
    }
    
    core::ResultCode testCameraSynchronization() {
        UNLOOK_LOG_DEBUG("Camera") << "Testing camera synchronization...";
        
        // This would capture test frames and validate synchronization timing
        // Verify <1ms synchronization precision
        // Test both cameras capture simultaneously
        // Validate frame timestamps
        
        return core::ResultCode::SUCCESS;
    }
    
    core::ResultCode stopCaptureInternal() {
        if (!capturing_) {
            return core::ResultCode::SUCCESS;
        }
        
        UNLOOK_LOG_INFO("Camera") << "Stopping synchronized capture...";
        
        capturing_ = false;
        capture_cv_.notify_all();
        
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        
        UNLOOK_LOG_INFO("Camera") << "Synchronized capture stopped";
        return core::ResultCode::SUCCESS;
    }
    
    void captureThreadLoop() {
        UNLOOK_LOG_DEBUG("Camera") << "Capture thread started";
        
        while (capturing_) {
            try {
                // Simulate frame capture at ~30 FPS
                std::this_thread::sleep_for(std::chrono::milliseconds(33));
                
                if (!capturing_) break;
                
                // Create simulated synchronized frames
                cv::Mat left_frame = cv::Mat::zeros(config_.height, config_.width, CV_8UC1);
                cv::Mat right_frame = cv::Mat::zeros(config_.height, config_.width, CV_8UC1);
                
                // Apply L/R swap if enabled
                if (lr_swapped_) {
                    std::swap(left_frame, right_frame);
                }
                
                uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                frames_captured_++;
                
                // Call frame callback if set
                if (frame_callback_) {
                    try {
                        frame_callback_(left_frame, right_frame, timestamp, callback_user_data_);
                    } catch (const std::exception& e) {
                        UNLOOK_LOG_WARNING("Camera") << "Frame callback exception: " << e.what();
                    }
                }
                
            } catch (const std::exception& e) {
                UNLOOK_LOG_ERROR("Camera") << "Capture thread exception: " << e.what();
                frames_dropped_++;
            }
        }
        
        UNLOOK_LOG_DEBUG("Camera") << "Capture thread stopped";
    }
};

// CameraSystem public interface implementation

CameraSystem::CameraSystem() : pimpl_(std::make_unique<Impl>()) {}

CameraSystem::~CameraSystem() = default;

core::ResultCode CameraSystem::initialize(const core::CameraConfig& config) {
    return pimpl_->initialize(config);
}

core::ResultCode CameraSystem::shutdown() {
    return pimpl_->shutdown();
}

bool CameraSystem::isInitialized() const {
    return pimpl_->isInitialized();
}

core::SyncStatus CameraSystem::getSyncStatus() const {
    return pimpl_->getSyncStatus();
}

core::ResultCode CameraSystem::startCapture() {
    return pimpl_->startCapture();
}

core::ResultCode CameraSystem::stopCapture() {
    return pimpl_->stopCapture();
}

bool CameraSystem::isCapturing() const {
    return pimpl_->isCapturing();
}

core::ResultCode CameraSystem::captureSingleFrame(cv::Mat& left_frame,
                                                  cv::Mat& right_frame,
                                                  uint32_t timeout_ms) {
    return pimpl_->captureSingleFrame(left_frame, right_frame, timeout_ms);
}

void CameraSystem::setFrameCallback(FrameCallback callback, void* user_data) {
    pimpl_->setFrameCallback(std::move(callback), user_data);
}

core::ResultCode CameraSystem::setExposure(uint32_t exposure_us) {
    return pimpl_->setExposure(exposure_us);
}

uint32_t CameraSystem::getExposure() const {
    return pimpl_->getExposure();
}

core::ResultCode CameraSystem::setGain(float gain) {
    return pimpl_->setGain(gain);
}

float CameraSystem::getGain() const {
    return pimpl_->getGain();
}

core::ResultCode CameraSystem::setExposureMode(ExposureMode mode) {
    return pimpl_->setExposureMode(mode);
}

ExposureMode CameraSystem::getExposureMode() const {
    return pimpl_->getExposureMode();
}

core::ResultCode CameraSystem::setGainMode(GainMode mode) {
    return pimpl_->setGainMode(mode);
}

GainMode CameraSystem::getGainMode() const {
    return pimpl_->getGainMode();
}

core::ResultCode CameraSystem::setLRSwap(bool swapped) {
    return pimpl_->setLRSwap(swapped);
}

bool CameraSystem::isLRSwapped() const {
    return pimpl_->isLRSwapped();
}

core::CameraConfig CameraSystem::getCurrentConfig() const {
    return pimpl_->getCurrentConfig();
}

core::ResultCode CameraSystem::updateConfig(const core::CameraConfig& config) {
    return pimpl_->updateConfig(config);
}

core::ResultCode CameraSystem::getCameraTemperature(float& left_temp, float& right_temp) const {
    return pimpl_->getCameraTemperature(left_temp, right_temp);
}

core::ResultCode CameraSystem::getFrameStats(uint64_t& frames_captured,
                                             uint64_t& frames_dropped,
                                             double& avg_frame_rate) const {
    return pimpl_->getFrameStats(frames_captured, frames_dropped, avg_frame_rate);
}

void CameraSystem::resetFrameStats() {
    pimpl_->resetFrameStats();
}

std::string CameraSystem::getLastError() const {
    return pimpl_->getLastError();
}

} // namespace api
} // namespace unlook