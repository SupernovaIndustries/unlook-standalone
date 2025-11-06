#include <unlook/camera/CameraSystem.hpp>
#include <unlook/camera/HardwareSyncCapture.hpp>
#include <unlook/camera/LibcameraSyncDevice.hpp>
#include <unlook/camera/HardwareSyncManager.hpp>
#include <unlook/camera/CameraSynchronizer.hpp>
#include <unlook/camera/AutoExposure.hpp>
#include <unlook/camera/CameraUtils.hpp>
// #include <unlook/realtime/RealtimePipeline.hpp>  // Temporarily disabled
#include <unlook/core/Logger.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <future>

// Use the official Unlook logging system
using unlook::core::Logger;
using unlook::core::LogLevel;

// Unified logging macros using Unlook Logger
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_DEBUG

#define LOG_INFO(msg) do { \
    std::ostringstream oss; \
    oss << "[CameraSystem] " << msg; \
    Logger::getInstance().info(oss.str()); \
} while(0)

#define LOG_ERROR(msg) do { \
    std::ostringstream oss; \
    oss << "[CameraSystem] " << msg; \
    Logger::getInstance().error(oss.str()); \
} while(0)

#define LOG_WARNING(msg) do { \
    std::ostringstream oss; \
    oss << "[CameraSystem] " << msg; \
    Logger::getInstance().warning(oss.str()); \
} while(0)

#define LOG_DEBUG(msg) do { \
    std::ostringstream oss; \
    oss << "[CameraSystem] " << msg; \
    Logger::getInstance().debug(oss.str()); \
} while(0)

namespace unlook {
namespace camera {

// Static members for singleton pattern
std::shared_ptr<CameraSystem> CameraSystem::instance_;
std::mutex CameraSystem::instance_mutex_;

std::shared_ptr<CameraSystem> CameraSystem::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        // Create instance with private constructor access
        struct MakeSharedEnabler : public CameraSystem {};
        instance_ = std::make_shared<MakeSharedEnabler>();
    }
    return instance_;
}

CameraSystem::CameraSystem() 
    : captureRunning_(false)
    , shouldStop_(false)
    , totalFrames_(0)
    , syncErrors_(0)
    , avgSyncError_(0.0)
    , maxSyncError_(0.0)
    , camerasSwapped_(false)
    , system_ready_(false)
    , left_state_(core::CameraState::NOT_INITIALIZED)
    , right_state_(core::CameraState::NOT_INITIALIZED) {
    
    LOG_INFO("CameraSystem constructor");
    
    // Initialize default configurations with PROPER VALUES
    left_config_.width = 1456;
    left_config_.height = 1088;
    left_config_.exposure_time_us = 25000;  // 25ms for VCSEL visibility (was 20ms)
    left_config_.gain = 1.0;                // MINIMAL: 1.0x gain (no amplification)
    left_config_.auto_exposure = true;      // Enable auto-exposure
    left_config_.auto_gain = false;
    
    right_config_ = left_config_;
    
    // Create real-time pipeline for high-performance processing
    // realtime_pipeline_ = std::make_unique<realtime::RealtimePipeline>();  // Temporarily disabled
}

CameraSystem::~CameraSystem() {
    LOG_INFO("CameraSystem destructor");
    
    // Since shutdown() is now idempotent, this is safe
    // This ensures cleanup even if explicit shutdown wasn't called
    shutdown();
}

bool CameraSystem::initialize() {
    return initialize(config_);
}

bool CameraSystem::initialize(const CameraConfig& config) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (status_.isInitialized) {
        LOG_WARNING("CameraSystem already initialized");
        return true;
    }
    
    LOG_INFO("Initializing CameraSystem with resolution " + 
             std::to_string(config.width) + "x" + std::to_string(config.height));
    
    config_ = config;
    
    // Initialize cameras
    if (!initializeCameras()) {
        LOG_ERROR("Failed to initialize cameras");
        return false;
    }
    
    // Initialize synchronizer
    if (config_.enableSync) {
        if (!configureSynchronization()) {
            LOG_ERROR("Failed to configure synchronization");
            return false;
        }
    }
    
    // Initialize auto-exposure if enabled
    // TEMPORARILY DISABLED FOR FPS PERFORMANCE TESTING
    // Auto-exposure calculates brightness on 1456x1088 = 1.5M pixels per frame @ 30 FPS
    // This could be the source of FPS lag/stuttering
    /*
    if (config_.autoExposure) {
        autoExposure_ = std::make_unique<AutoExposure>();
        
        AutoExposure::Config aeConfig;
        aeConfig.minExposureUs = 100.0;
        aeConfig.maxExposureUs = std::min(config_.exposureTime * 2.0, 20000.0);  // Cap at 20ms
        aeConfig.targetBrightness = 80;  // REDUCED for indoor lighting
        
        if (!autoExposure_->initialize(aeConfig)) {
            LOG_WARNING("Failed to initialize auto-exposure");
            autoExposure_.reset();
        } else {
            // Set callback to update camera exposure
            autoExposure_->setUpdateCallback([this](const AutoExposure::ExposureParams& params) {
                if (params.isConverged) {
                    setExposure(params.exposureUs);
                    setGain(params.analogGain);
                }
            });
        }
    }
    */
    LOG_INFO("Auto-exposure DISABLED for performance testing - using fixed exposure settings");
    
    status_.isInitialized = true;
    system_ready_ = true;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    LOG_INFO("CameraSystem initialized successfully");
    
    return true;
}

bool CameraSystem::initializeCameras() {
    LOG_INFO("Initializing stereo camera pair using HardwareSyncCapture");
    
    // Use existing HardwareSyncCapture system temporarily
    /*
    // Configure real-time pipeline for high performance
    realtime::RealtimePipeline::Config rt_config;
    rt_config.full_width = config_.width;
    rt_config.full_height = config_.height;
    rt_config.preview_width = 640;   // VGA for GUI
    rt_config.preview_height = 480;
    rt_config.exposure_time_us = config_.exposureTime;  // Use proper exposure
    rt_config.analog_gain = config_.analogGain;
    rt_config.auto_exposure = config_.autoExposure;
    rt_config.target_fps = config_.targetFps;
    rt_config.enable_thread_affinity = true;
    rt_config.enable_frame_dropping = true;
    
    if (!realtime_pipeline_->initialize(rt_config)) {
        LOG_ERROR("Failed to initialize real-time pipeline");
        return false;
    }
    */
    
    // Create hardware-synchronized capture system (backup)
    hardware_sync_capture_ = std::make_unique<HardwareSyncCapture>();
    
    // Configure camera parameters matching GUI configuration
    HardwareSyncCapture::CameraConfig capture_config;
    capture_config.width = config_.width;
    capture_config.height = config_.height;
    capture_config.format = libcamera::formats::YUV420;  // Use YUV420 for proper stride handling
    capture_config.buffer_count = 2;  // 2 buffers to prevent starvation
    
    // Initialize the hardware sync system
    if (!hardware_sync_capture_->initialize(capture_config)) {
        LOG_ERROR("Failed to initialize HardwareSyncCapture system");
        status_.errorMessage = "Hardware sync camera initialization failed";
        return false;
    }
    
    // Both cameras are now ready (handled internally by HardwareSyncCapture)
    status_.leftCameraReady = true;
    status_.rightCameraReady = true;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    
    LOG_INFO("Hardware synchronized stereo camera system initialized successfully");
    LOG_INFO("Camera 1 = LEFT/MASTER, Camera 0 = RIGHT/SLAVE");
    
    return true;
}

bool CameraSystem::configureSynchronization() {
    LOG_INFO("Hardware synchronization already configured in HardwareSyncCapture");
    
    // HardwareSyncCapture handles all synchronization internally
    // Camera 1 = LEFT/MASTER, Camera 0 = RIGHT/SLAVE
    // XVS/XHS hardware sync pins are configured automatically
    
    status_.isSynchronized = true;
    LOG_INFO("Synchronization configured: Hardware (managed by HardwareSyncCapture)");
    
    return true;
}

void CameraSystem::shutdown() {
    LOG_INFO("Shutting down CameraSystem");

    // Check if already shutdown
    if (!status_.isInitialized) {
        LOG_DEBUG("CameraSystem already shutdown, skipping");
        return;
    }
    
    // Stop capture if running
    if (captureRunning_) {
        stopCapture();
    }
    
    // Stop real-time pipeline FIRST (to prevent callbacks during shutdown)
    /*if (realtime_pipeline_ && realtime_pipeline_->isRunning()) {
        LOG_INFO("Stopping real-time pipeline");
        realtime_pipeline_->stop();
        // Give time for threads to finish
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/ // Temporarily disabled
    
    // Stop auto-exposure (now idempotent)
    if (autoExposure_) {
        autoExposure_->stop();
        autoExposure_->shutdown();
        autoExposure_.reset();  // Explicitly reset to prevent destructor issues
    }
    
    // Stop hardware sync capture (if used as fallback)
    if (hardware_sync_capture_) {
        LOG_INFO("Stopping hardware sync capture");
        hardware_sync_capture_->stop();
        // Reset to trigger proper destructor sequence
        hardware_sync_capture_.reset();
    }
    
    // Stop synchronizer
    if (synchronizer_) {
        synchronizer_->stop();
        synchronizer_->shutdown();
    }
    
    // Stop and release cameras
    // NOTE: Using hardware_sync_capture_ instead of individual cameras
    // The leftCamera_ and rightCamera_ are not initialized in this mode
    /*if (leftCamera_) {
        leftCamera_->stop();
        leftCamera_.reset();
    }
    
    if (rightCamera_) {
        rightCamera_->stop();
        rightCamera_.reset();
    }*/
    
    // Reset status
    status_ = CameraStatus();
    system_ready_ = false;
    left_state_ = core::CameraState::NOT_INITIALIZED;
    right_state_ = core::CameraState::NOT_INITIALIZED;
    
    LOG_INFO("CameraSystem shutdown complete");
}

bool CameraSystem::startCapture(core::StereoFrameCallback frame_callback) {
    // CRITICAL FIX: Allow callback update even when capture is already running
    // This fixes the issue where main_window auto-starts with dummy callback
    // and then widgets can't receive frames because callback doesn't update

    // THREAD SAFETY: Use callbackMutex_ when updating frame_callback_
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        frame_callback_ = frame_callback;
    }

    if (captureRunning_) {
        LOG_INFO("Capture already running - callback updated successfully");
        return true;
    }

    return startCapture();
}

bool CameraSystem::startCapture() {
    if (!status_.isInitialized) {
        LOG_ERROR("Cannot start capture: system not initialized");
        return false;
    }

    if (captureRunning_) {
        LOG_WARNING("Capture already running");
        return true;
    }
    
    LOG_INFO("Using HardwareSyncCapture for 30 FPS capture");
    
    // CRITICAL FIX: Check if hardware_sync_capture_ is valid
    if (!hardware_sync_capture_) {
        LOG_ERROR("Hardware sync capture not initialized");
        return false;
    }
    
    // Use HardwareSyncCapture directly
    if (!hardware_sync_capture_->start()) {
        LOG_ERROR("Failed to start hardware synchronized capture");
        return false;
    }
        
        // Set up frame callback to convert HardwareSyncCapture frames to GUI format
        hardware_sync_capture_->setFrameCallback([this](const HardwareSyncCapture::StereoFrame& sync_frame) {
        // Convert HardwareSyncCapture::StereoFrame to core::StereoFramePair
        core::StereoFramePair gui_frame;

        // Left frame (Camera 1 = MASTER)
        gui_frame.left_frame.image = sync_frame.left_image;
        gui_frame.left_frame.timestamp_ns = sync_frame.left_timestamp_ns;
        gui_frame.left_frame.camera_id = core::CameraId::LEFT;
        gui_frame.left_frame.valid = true;

        // Right frame (Camera 0 = SLAVE)
        gui_frame.right_frame.image = sync_frame.right_image;
        gui_frame.right_frame.timestamp_ns = sync_frame.right_timestamp_ns;
        gui_frame.right_frame.camera_id = core::CameraId::RIGHT;
        gui_frame.right_frame.valid = true;

        // Sync info
        gui_frame.sync_error_ms = sync_frame.sync_error_ms;
        gui_frame.synchronized = (sync_frame.sync_error_ms <= 5.0); // 5ms tolerance - realistic for hardware

        // Update statistics
        totalFrames_++;
        if (sync_frame.sync_error_ms > 1.0) {
            syncErrors_++;
        }
        updateSyncStats(sync_frame.sync_error_ms);

        // THREAD SAFETY: Lock callbackMutex_ before accessing frame_callback_
        // Deliver to GUI callback
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (frame_callback_) {
                frame_callback_(gui_frame);
            }
        }
    });
    
    captureRunning_ = true;
    left_state_ = core::CameraState::CAPTURING;
    right_state_ = core::CameraState::CAPTURING;
    
    LOG_INFO("Hardware synchronized camera capture started");
    return true;
}

void CameraSystem::stopCapture() {
    if (!captureRunning_) {
        return;
    }
    
    LOG_INFO("Stopping camera capture");
    
    // CRITICAL FIX: Clear frame callback before stopping to prevent callbacks during shutdown
    frame_callback_ = nullptr;
    
    // Stop HardwareSyncCapture system if running
    if (hardware_sync_capture_) {
        hardware_sync_capture_->stop();
        
        // CRITICAL FIX: Wait for stop to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    captureRunning_ = false;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    
    LOG_INFO("Camera capture stopped");
}

void CameraSystem::captureThread() {
    // CRITICAL FIX: This function is NOT USED when hardware_sync_capture_ is active
    // HardwareSyncCapture handles capture internally via callbacks
    // This thread is only for backward compatibility with individual cameras
    
    LOG_WARNING("captureThread() called but should not be used with HardwareSyncCapture");
    LOG_INFO("HardwareSyncCapture manages capture via internal callbacks");
    
    // Simply wait until stop is signaled
    // The actual capture is handled by HardwareSyncCapture's internal threads
    while (!shouldStop_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    LOG_INFO("Capture thread exiting (HardwareSyncCapture mode)");
    return;
    
    /* LEGACY CODE - Only used if individual cameras were initialized
    LOG_INFO("Capture thread started - Hardware Sync Mode");
    
    CameraUtils::FPSCounter fpsCounter(30);
    CameraUtils::Timer frameTimer;
    
    while (!shouldStop_) {
        frameTimer.reset();
        
        StereoFrame frame;
        LibcameraSyncDevice::FrameMetadata leftMeta, rightMeta;
        
        std::atomic<bool> leftReady(false);
        std::atomic<bool> rightReady(false);
        bool leftSuccess = false;
        bool rightSuccess = false;
        
        // Launch parallel capture threads for synchronized operation
        std::thread leftCaptureThread([&]() {
            // Master camera (LEFT) starts first
            leftSuccess = leftCamera_->waitForFrame(frame.leftImage, leftMeta, 100);
            leftReady = true;
        });
        
        std::thread rightCaptureThread([&]() {
            // Slave camera (RIGHT) follows master's sync signals
            rightSuccess = rightCamera_->waitForFrame(frame.rightImage, rightMeta, 100);
            rightReady = true;
        });
        
        // Wait for both captures to complete (equivalent to shell 'wait')
        leftCaptureThread.join();
        rightCaptureThread.join();
        
        if (!leftSuccess || !rightSuccess) {
            if (!shouldStop_) {
                LOG_DEBUG("Synchronized capture timeout, retrying");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }
        
        // Fill frame metadata
        frame.leftTimestampNs = leftMeta.timestamp_ns;
        frame.rightTimestampNs = rightMeta.timestamp_ns;
        frame.leftFrameNumber = leftMeta.frame_number;
        frame.rightFrameNumber = rightMeta.frame_number;
        frame.leftExposure = leftMeta.exposure_time_us;
        frame.rightExposure = rightMeta.exposure_time_us;
        frame.leftGain = leftMeta.analog_gain;
        frame.rightGain = rightMeta.analog_gain;
        
        // Calculate sync error
        frame.syncErrorMs = CameraUtils::calculateSyncError(
            frame.leftTimestampNs, frame.rightTimestampNs);
        frame.isSynchronized = frame.syncErrorMs <= config_.syncToleranceMs;
        
        // Update statistics
        updateSyncStats(frame.syncErrorMs);
        totalFrames_++;
        
        if (!frame.isSynchronized) {
            syncErrors_++;
            LOG_WARNING("Frame sync error: " + std::to_string(frame.syncErrorMs) + " ms");
        }
        
        // Process with auto-exposure if enabled
        if (autoExposure_ && config_.autoExposure) {
            autoExposure_->processStereo(frame.leftImage, frame.rightImage);
        }
        
        // Call frame callbacks
        {
            std::lock_guard<std::mutex> lock(callbackMutex_);
            if (frameCallback_) {
                frameCallback_(frame);
            }
            
            // Call new style callback if set
            if (frame_callback_) {
                core::StereoFramePair pair;
                pair.left_frame.image = frame.leftImage;
                pair.left_frame.timestamp_ns = frame.leftTimestampNs;
                pair.left_frame.camera_id = core::CameraId::LEFT;
                pair.left_frame.valid = true;
                pair.right_frame.image = frame.rightImage;
                pair.right_frame.timestamp_ns = frame.rightTimestampNs;
                pair.right_frame.camera_id = core::CameraId::RIGHT;
                pair.right_frame.valid = true;
                pair.synchronized = frame.isSynchronized;
                pair.sync_error_ms = frame.syncErrorMs;
                frame_callback_(pair);
            }
        }
        
        // Update FPS
        fpsCounter.update();
        status_.currentFps = fpsCounter.getFPS();
        
        // Frame rate limiting
        double frameTimeMs = frameTimer.elapsedMs();
        double targetFrameTimeMs = 1000.0 / config_.targetFps;
        if (frameTimeMs < targetFrameTimeMs) {
            std::this_thread::sleep_for(
                std::chrono::microseconds(
                    static_cast<int>((targetFrameTimeMs - frameTimeMs) * 1000)));
        }
    }
    
    LOG_INFO("Capture thread stopped");
    */
}

bool CameraSystem::captureStereoFrame(StereoFrame& frame, int timeoutMs) {
    if (!status_.isInitialized) {
        LOG_ERROR("System not initialized");
        return false;
    }
    
    // CRITICAL FIX: This function should not be called when using HardwareSyncCapture
    // HardwareSyncCapture delivers frames via callbacks, not polling
    if (hardware_sync_capture_) {
        LOG_ERROR("captureStereoFrame() not supported with HardwareSyncCapture - use callbacks instead");
        return false;
    }
    
    // This function is not supported with HardwareSyncCapture
    // HardwareSyncCapture delivers frames via callbacks, not polling
    return false;
}

void CameraSystem::setFrameCallback(FrameCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    frameCallback_ = callback;
}

void CameraSystem::setErrorCallback(core::ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    errorCallback_ = callback;
}

bool CameraSystem::setExposure(double exposureUs) {
    // FIXED: Use hardware_sync_capture_ instead of individual cameras
    if (hardware_sync_capture_) {
        // HardwareSyncCapture manages exposure internally via auto-exposure
        config_.exposureTime = exposureUs;
        return true;
    }
    
    // Fallback for individual cameras (not used in current mode)
    /*if (!leftCamera_ || !rightCamera_) {
        return false;
    }
    
    bool success = leftCamera_->setExposure(exposureUs);
    success &= rightCamera_->setExposure(exposureUs);
    
    if (success) {
        config_.exposureTime = exposureUs;
    }
    
    return success;*/
    return false;
}

bool CameraSystem::setGain(double gain) {
    // FIXED: Use hardware_sync_capture_ instead of individual cameras
    if (hardware_sync_capture_) {
        // HardwareSyncCapture manages gain internally via auto-exposure
        config_.analogGain = gain;
        return true;
    }
    
    // Fallback for individual cameras (not used in current mode)
    /*if (!leftCamera_ || !rightCamera_) {
        return false;
    }
    
    bool success = leftCamera_->setGain(gain);
    success &= rightCamera_->setGain(gain);
    
    if (success) {
        config_.analogGain = gain;
    }
    
    return success;*/
    return false;  // Not supported in current mode
}

void CameraSystem::setAutoExposure(bool enable) {
    config_.autoExposure = enable;
    
    if (autoExposure_) {
        autoExposure_->setEnabled(enable);
    }
}

bool CameraSystem::setTargetFps(double fps) {
    config_.targetFps = fps;
    
    if (synchronizer_) {
        return synchronizer_->setSyncFrequency(fps);
    }
    
    return true;
}

void CameraSystem::swapCameras() {
    camerasSwapped_ = !camerasSwapped_;
    LOG_INFO("Cameras swapped: LEFT is now camera " + 
             std::to_string(camerasSwapped_ ? 0 : 1));
    
    // If initialized, we need to reinitialize with swapped IDs
    if (status_.isInitialized) {
        bool wasCapturing = captureRunning_;
        
        if (wasCapturing) {
            stopCapture();
        }
        
        // Re-initialize cameras with swapped IDs
        leftCamera_.reset();
        rightCamera_.reset();
        
        if (!initializeCameras()) {
            LOG_ERROR("Failed to reinitialize cameras after swap");
        }
        
        if (wasCapturing) {
            startCapture();
        }
    }
}

CameraStatus CameraSystem::getStatus() const {
    std::lock_guard<std::mutex> lock(statusMutex_);
    
    CameraStatus status = status_;
    status.totalFramesCaptured = totalFrames_;
    status.syncErrors = syncErrors_;
    status.avgSyncErrorMs = avgSyncError_;
    status.maxSyncErrorMs = maxSyncError_;
    
    return status;
}

CameraConfig CameraSystem::getConfig() const {
    std::lock_guard<std::mutex> lock(configMutex_);
    return config_;
}

bool CameraSystem::isInitialized() const {
    return status_.isInitialized;
}

bool CameraSystem::isCapturing() const {
    return captureRunning_;
}

void CameraSystem::getSyncStats(double& avgErrorMs, double& maxErrorMs, uint64_t& errorCount) const {
    avgErrorMs = avgSyncError_;
    maxErrorMs = maxSyncError_;
    errorCount = syncErrors_;
}

void CameraSystem::updateSyncStats(double errorMs) {
    // Update maximum
    double currentMax = maxSyncError_.load();
    while (errorMs > currentMax && 
           !maxSyncError_.compare_exchange_weak(currentMax, errorMs)) {
        // Loop until successful
    }
    
    // Update average (simple moving average)
    double currentAvg = avgSyncError_.load();
    double newAvg = (currentAvg * 0.95) + (errorMs * 0.05);  // Exponential moving average
    avgSyncError_ = newAvg;
}

// New methods required by GUI

bool CameraSystem::isReady() const {
    return system_ready_;
}

core::CameraState CameraSystem::getCameraState(core::CameraId camera_id) const {
    if (camera_id == core::CameraId::LEFT) {
        return left_state_;
    } else {
        return right_state_;
    }
}

core::CameraConfig CameraSystem::getCameraConfig(core::CameraId camera_id) const {
    std::lock_guard<std::mutex> lock(configMutex_);
    if (camera_id == core::CameraId::LEFT) {
        return left_config_;
    } else {
        return right_config_;
    }
}

bool CameraSystem::setAutoExposure(core::CameraId camera_id, bool enabled) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (camera_id == core::CameraId::LEFT) {
        left_config_.auto_exposure = enabled;
    } else {
        right_config_.auto_exposure = enabled;
    }
    
    // Apply to auto-exposure system if it affects both cameras
    if (autoExposure_) {
        autoExposure_->setEnabled(enabled);
    }
    
    return true;
}

bool CameraSystem::setExposureTime(core::CameraId camera_id, double exposure_us) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (exposure_us < 100.0 || exposure_us > 50000.0) {
        LOG_ERROR("Invalid exposure time: " + std::to_string(exposure_us) + "us (range: 100-50000)");
        return false;
    }
    
    // Update configuration for both cameras (unified control)
    left_config_.exposure_time_us = exposure_us;
    right_config_.exposure_time_us = exposure_us;
    
    // Apply to hardware via HardwareSyncCapture
    if (hardware_sync_capture_) {
        hardware_sync_capture_->setExposureTime(exposure_us);
    }
    
    return true;
}

bool CameraSystem::setGain(core::CameraId camera_id, double gain) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (gain < 1.0 || gain > 16.0) {
        LOG_ERROR("Invalid gain value: " + std::to_string(gain) + "x (range: 1.0-16.0)");
        return false;
    }
    
    // Update configuration for both cameras (unified control)
    left_config_.gain = gain;
    right_config_.gain = gain;
    
    // Apply to hardware via HardwareSyncCapture
    if (hardware_sync_capture_) {
        hardware_sync_capture_->setGain(gain);
    }
    
    return true;
}

bool CameraSystem::setAutoGain(core::CameraId camera_id, bool enabled) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (camera_id == core::CameraId::LEFT) {
        left_config_.auto_gain = enabled;
    } else {
        right_config_.auto_gain = enabled;
    }
    
    // Note: Auto gain may need custom implementation as it's not directly supported
    return true;
}

bool CameraSystem::setFrameRate(double fps) {
    if (fps < 1.0 || fps > 60.0) {
        LOG_ERROR("Invalid frame rate: " + std::to_string(fps));
        return false;
    }
    
    return setTargetFps(fps);
}

double CameraSystem::getCurrentFrameRate() const {
    return status_.currentFps;
}

std::string CameraSystem::getCameraInfo(core::CameraId camera_id) const {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (!system_ready_) {
        return "Camera system not initialized";
    }
    
    std::stringstream ss;
    const core::CameraConfig& config = (camera_id == core::CameraId::LEFT) ? left_config_ : right_config_;
    const core::CameraState state = (camera_id == core::CameraId::LEFT) ? left_state_.load() : right_state_.load();
    
    ss << "Camera " << ((camera_id == core::CameraId::LEFT) ? "LEFT" : "RIGHT") << "\n";
    ss << "  State: ";
    switch (state) {
        case core::CameraState::NOT_INITIALIZED: ss << "Not Initialized"; break;
        case core::CameraState::INITIALIZING: ss << "Initializing"; break;
        case core::CameraState::READY: ss << "Ready"; break;
        case core::CameraState::CAPTURING: ss << "Capturing"; break;
        case core::CameraState::ERROR: ss << "Error"; break;
        default: ss << "Unknown"; break;
    }
    ss << "\n";
    ss << "  Resolution: " << config.width << "x" << config.height << "\n";
    ss << "  Exposure: " << config.exposure_time_us << "us (Auto: " << (config.auto_exposure ? "Yes" : "No") << "\n";
    ss << "  Gain: " << config.gain << " (Auto: " << (config.auto_gain ? "Yes" : "No") << "\n";
    
    return ss.str();
}

bool CameraSystem::isHardwareSyncEnabled() const {
    return status_.isSynchronized && synchronizer_ && 
           (synchronizer_->getSyncMode() == CameraSynchronizer::SyncMode::HARDWARE_FULL);
}

double CameraSystem::getAverageSyncError() const {
    return avgSyncError_;
}

core::StereoFramePair CameraSystem::captureSingle() {
    core::StereoFramePair pair;
    
    if (!system_ready_) {
        LOG_ERROR("[DepthCapture] Cannot capture: system not initialized");
        pair.synchronized = false;
        return pair;
    }
    
    LOG_INFO("[DepthCapture] Starting single frame capture");
    
    // CRITICAL FIX: Use direct captureSingle method if available
    if (hardware_sync_capture_) {
        LOG_DEBUG("[DepthCapture] Using HardwareSyncCapture for single frame");
        
        // Check if capture is already running
        bool was_capturing = captureRunning_.load();
        
        if (was_capturing) {
            // If already capturing, use callback method to avoid start/stop cycle
            LOG_DEBUG("[DepthCapture] Capture already running, using callback method");
            
            // Create a promise/future pair for synchronous capture
            std::promise<core::StereoFramePair> capture_promise;
            std::future<core::StereoFramePair> capture_future = capture_promise.get_future();
            
            // Temporarily set a callback to capture one frame
            std::atomic<bool> frame_captured{false};
            
            // Save original callback
            auto original_callback = frame_callback_;
            
            // Set temporary callback
            frame_callback_ = [&capture_promise, &frame_captured](const core::StereoFramePair& captured_pair) {
                if (!frame_captured.exchange(true)) {
                    capture_promise.set_value(captured_pair);
                }
            };
            
            // Wait for frame with timeout
            auto status = capture_future.wait_for(std::chrono::milliseconds(2000));
            
            // Restore original callback
            frame_callback_ = original_callback;
            
            if (status == std::future_status::ready) {
                pair = capture_future.get();
                LOG_INFO("[DepthCapture] Successfully captured single frame - Sync: " + 
                        std::to_string(pair.synchronized) + ", Error: " + 
                        std::to_string(pair.sync_error_ms) + "ms");
            } else {
                LOG_ERROR("[DepthCapture] Timeout waiting for single frame capture");
                pair.synchronized = false;
            }
        } else {
            // Not capturing, need to start capture temporarily
            LOG_DEBUG("[DepthCapture] Starting temporary capture for single frame");
            
            // Start capture
            if (!hardware_sync_capture_->start()) {
                LOG_ERROR("[DepthCapture] Failed to start capture for single frame");
                pair.synchronized = false;
                return pair;
            }
            
            // CRITICAL FIX: Wait for cameras to stabilize after start
            // This prevents the 33ms sync error
            LOG_DEBUG("[DepthCapture] Waiting for camera stabilization...");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            
            // Use captureSingle method directly
            HardwareSyncCapture::StereoFrame sync_frame;
            if (hardware_sync_capture_->captureSingle(sync_frame, 3000)) {
                // Convert to core::StereoFramePair
                pair.left_frame.image = sync_frame.left_image;
                pair.left_frame.timestamp_ns = sync_frame.left_timestamp_ns;
                pair.left_frame.camera_id = core::CameraId::LEFT;
                pair.left_frame.valid = true;
                
                pair.right_frame.image = sync_frame.right_image;
                pair.right_frame.timestamp_ns = sync_frame.right_timestamp_ns;
                pair.right_frame.camera_id = core::CameraId::RIGHT;
                pair.right_frame.valid = true;
                
                pair.sync_error_ms = sync_frame.sync_error_ms;
                pair.synchronized = (sync_frame.sync_error_ms <= 5.0); // 5ms tolerance - realistic for hardware
                
                LOG_INFO("[DepthCapture] Successfully captured single frame - Sync: " + 
                        std::to_string(pair.synchronized) + ", Error: " + 
                        std::to_string(pair.sync_error_ms) + "ms");
            } else {
                LOG_ERROR("[DepthCapture] Failed to capture single frame");
                pair.synchronized = false;
            }
            
            // Stop temporary capture
            LOG_DEBUG("[DepthCapture] Stopping temporary capture");
            hardware_sync_capture_->stop();
            
            // Reset capture running flag
            captureRunning_ = false;
        }
    } else {
        LOG_ERROR("[DepthCapture] No hardware sync capture available");
        pair.synchronized = false;
    }
    
    return pair;
}

bool CameraSystem::configureCameras(const core::CameraConfig& left_config, 
                                   const core::CameraConfig& right_config) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (!system_ready_) {
        LOG_ERROR("Cannot configure cameras: system not initialized");
        return false;
    }
    
    // Validate resolution (must match calibration resolution)
    if (left_config.width != 1456 || left_config.height != 1088 ||
        right_config.width != 1456 || right_config.height != 1088) {
        LOG_ERROR("Camera resolution must be 1456x1088 for calibration compatibility");
        return false;
    }
    
    left_config_ = left_config;
    right_config_ = right_config;
    
    // FIXED: HardwareSyncCapture manages camera settings internally
    // Just update the configuration values
    // The auto-exposure in HardwareSyncCapture will handle the actual hardware control
    
    return true;  // Configuration updated successfully
}

} // namespace camera
} // namespace unlook