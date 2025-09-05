#include <unlook/camera/CameraSystem.hpp>
#include <unlook/camera/HardwareSyncCapture.hpp>
#include <unlook/camera/LibcameraSyncDevice.hpp>
#include <unlook/camera/HardwareSyncManager.hpp>
#include <unlook/camera/CameraSynchronizer.hpp>
#include <unlook/camera/AutoExposure.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/logger.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>

// Use the official Unlook logging system
using unlook::core::LogStream;
using unlook::core::LogLevel;

// Simple logging to avoid macro conflicts
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_DEBUG
#define LOG_INFO(msg) std::cout << "[INFO] CameraSystem: " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] CameraSystem: " << msg << std::endl  
#define LOG_WARNING(msg) std::cout << "[WARN] CameraSystem: " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] CameraSystem: " << msg << std::endl

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
    
    // Initialize default configurations
    left_config_.width = 1456;
    left_config_.height = 1088;
    left_config_.exposure_time_us = 10000;
    left_config_.gain = 1.0;
    left_config_.auto_exposure = false;
    left_config_.auto_gain = false;
    
    right_config_ = left_config_;
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
    
    status_.isInitialized = true;
    system_ready_ = true;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    LOG_INFO("CameraSystem initialized successfully");
    
    return true;
}

bool CameraSystem::initializeCameras() {
    LOG_INFO("Initializing stereo camera pair using HardwareSyncCapture");
    
    // Create hardware-synchronized capture system
    hardware_sync_capture_ = std::make_unique<HardwareSyncCapture>();
    
    // Configure camera parameters matching GUI configuration
    HardwareSyncCapture::CameraConfig capture_config;
    capture_config.width = config_.width;
    capture_config.height = config_.height;
    capture_config.format = libcamera::formats::SBGGR10;
    capture_config.buffer_count = 4;
    
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
    // Make shutdown idempotent - use atomic flag for thread safety
    static std::atomic<bool> shutdownCompleted{false};
    
    bool expected = false;
    if (!shutdownCompleted.compare_exchange_strong(expected, true)) {
        // Shutdown already completed
        LOG_DEBUG("CameraSystem shutdown already completed, skipping");
        return;
    }
    
    LOG_INFO("Shutting down CameraSystem");
    
    // Stop capture if running
    if (captureRunning_) {
        stopCapture();
    }
    
    // Stop auto-exposure (now idempotent)
    if (autoExposure_) {
        autoExposure_->stop();
        autoExposure_->shutdown();
        autoExposure_.reset();  // Explicitly reset to prevent destructor issues
    }
    
    // Stop synchronizer
    if (synchronizer_) {
        synchronizer_->stop();
        synchronizer_->shutdown();
    }
    
    // Stop and release cameras
    if (leftCamera_) {
        leftCamera_->stop();
        leftCamera_.reset();
    }
    
    if (rightCamera_) {
        rightCamera_->stop();
        rightCamera_.reset();
    }
    
    // Reset status
    status_ = CameraStatus();
    system_ready_ = false;
    left_state_ = core::CameraState::NOT_INITIALIZED;
    right_state_ = core::CameraState::NOT_INITIALIZED;
    
    LOG_INFO("CameraSystem shutdown complete");
}

bool CameraSystem::startCapture(core::StereoFrameCallback frame_callback) {
    frame_callback_ = frame_callback;
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
    
    LOG_INFO("Starting hardware synchronized camera capture with HardwareSyncCapture");
    
    // Start hardware-synchronized capture
    if (!hardware_sync_capture_->start()) {
        LOG_ERROR("Failed to start hardware synchronized capture");
        return false;
    }
    
    // Set up frame callback to convert HardwareSyncCapture frames to GUI format
    hardware_sync_capture_->setFrameCallback([this](const HardwareSyncCapture::StereoFrame& sync_frame) {
        if (!frame_callback_) return;
        
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
        gui_frame.synchronized = (sync_frame.sync_error_ms <= 1.0); // 1ms tolerance
        
        // Update statistics
        totalFrames_++;
        if (sync_frame.sync_error_ms > 1.0) {
            syncErrors_++;
        }
        updateSyncStats(sync_frame.sync_error_ms);
        
        // Deliver to GUI callback
        frame_callback_(gui_frame);
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
    
    LOG_INFO("Stopping hardware synchronized camera capture");
    
    // Stop HardwareSyncCapture system
    if (hardware_sync_capture_) {
        hardware_sync_capture_->stop();
    }
    
    captureRunning_ = false;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;
    
    LOG_INFO("Hardware synchronized camera capture stopped");
}

void CameraSystem::captureThread() {
    LOG_INFO("Capture thread started - Hardware Sync Mode");
    
    CameraUtils::FPSCounter fpsCounter(30);
    CameraUtils::Timer frameTimer;
    
    // CRITICAL: With hardware sync, we must capture from both cameras SIMULTANEOUSLY
    // The master generates sync signals, slave follows them
    // Both cameras must have their waitForFrame calls active at the same time
    
    while (!shouldStop_) {
        frameTimer.reset();
        
        StereoFrame frame;
        LibcameraSyncDevice::FrameMetadata leftMeta, rightMeta;
        
        // HARDWARE SYNC PATTERN: Start both captures in parallel
        // This replicates the working cam command pattern:
        // ./src/apps/cam/cam -c 1 --capture=1 &  # Master in background
        // ./src/apps/cam/cam -c 2 --capture=1 && wait  # Slave + wait
        
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
}

bool CameraSystem::captureStereoFrame(StereoFrame& frame, int timeoutMs) {
    if (!status_.isInitialized) {
        LOG_ERROR("System not initialized");
        return false;
    }
    
    // LibcameraSyncDevice handles its own streaming state
    LibcameraSyncDevice::FrameMetadata leftMeta, rightMeta;
    
    // HARDWARE SYNC: Capture synchronized frames using parallel pattern
    std::atomic<bool> leftReady(false);
    std::atomic<bool> rightReady(false);
    bool leftSuccess = false;
    bool rightSuccess = false;
    
    // Launch parallel captures for hardware synchronization
    std::thread leftThread([&]() {
        leftSuccess = leftCamera_->captureFrame(frame.leftImage, leftMeta, timeoutMs);
        leftReady = true;
    });
    
    std::thread rightThread([&]() {
        rightSuccess = rightCamera_->captureFrame(frame.rightImage, rightMeta, timeoutMs);
        rightReady = true;
    });
    
    // Wait for both to complete (synchronized)
    leftThread.join();
    rightThread.join();
    
    if (!leftSuccess || !rightSuccess) {
        LOG_ERROR("Failed to capture synchronized stereo frame");
        return false;
    }
    
    // Fill metadata
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
    
    return true;
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
    if (!leftCamera_ || !rightCamera_) {
        return false;
    }
    
    bool success = leftCamera_->setExposure(exposureUs);
    success &= rightCamera_->setExposure(exposureUs);
    
    if (success) {
        config_.exposureTime = exposureUs;
    }
    
    return success;
}

bool CameraSystem::setGain(double gain) {
    if (!leftCamera_ || !rightCamera_) {
        return false;
    }
    
    bool success = leftCamera_->setGain(gain);
    success &= rightCamera_->setGain(gain);
    
    if (success) {
        config_.analogGain = gain;
    }
    
    return success;
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
    
    if (exposure_us < 1.0 || exposure_us > 100000.0) {
        LOG_ERROR("Invalid exposure time: " + std::to_string(exposure_us) + "us");
        return false;
    }
    
    bool success = true;
    
    if (camera_id == core::CameraId::LEFT) {
        left_config_.exposure_time_us = exposure_us;
        if (leftCamera_) {
            success = leftCamera_->setExposure(exposure_us);
        }
    } else {
        right_config_.exposure_time_us = exposure_us;
        if (rightCamera_) {
            success = rightCamera_->setExposure(exposure_us);
        }
    }
    
    return success;
}

bool CameraSystem::setGain(core::CameraId camera_id, double gain) {
    std::lock_guard<std::mutex> lock(configMutex_);
    
    if (gain < 1.0 || gain > 16.0) {
        LOG_ERROR("Invalid gain value: " + std::to_string(gain));
        return false;
    }
    
    bool success = true;
    
    if (camera_id == core::CameraId::LEFT) {
        left_config_.gain = gain;
        if (leftCamera_) {
            success = leftCamera_->setAnalogGain(gain);
        }
    } else {
        right_config_.gain = gain;
        if (rightCamera_) {
            success = rightCamera_->setAnalogGain(gain);
        }
    }
    
    return success;
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
        LOG_ERROR("Cannot capture: system not initialized");
        pair.synchronized = false;
        return pair;
    }
    
    StereoFrame frame;
    if (captureStereoFrame(frame, 5000)) {
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
    } else {
        pair.synchronized = false;
        pair.left_frame.valid = false;
        pair.right_frame.valid = false;
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
    
    // Apply to hardware
    bool success = true;
    if (leftCamera_) {
        success &= leftCamera_->setExposure(left_config.exposure_time_us);
        success &= leftCamera_->setGain(left_config.gain);
    }
    if (rightCamera_) {
        success &= rightCamera_->setExposure(right_config.exposure_time_us);
        success &= rightCamera_->setGain(right_config.gain);
    }
    
    return success;
}

} // namespace camera
} // namespace unlook