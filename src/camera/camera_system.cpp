#include "unlook/camera/CameraSystemGUI.hpp"
#include "unlook/camera/HardwareSyncCapture.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/exception.h"

#include <thread>
#include <chrono>

namespace unlook {
namespace camera {
namespace gui {

// Static members
std::shared_ptr<CameraSystem> CameraSystem::instance_;
std::mutex CameraSystem::instance_mutex_;

// Hardware configuration constants for IMX296 cameras
constexpr int IMX296_WIDTH = 1280;
constexpr int IMX296_HEIGHT = 720;
constexpr double TARGET_FPS = 30.0;

CameraSystem::CameraSystem()
    : capture_running_(false)
    , system_ready_(false)
    , left_state_(core::CameraState::DISCONNECTED)
    , right_state_(core::CameraState::DISCONNECTED)
    , frame_callback_(nullptr)
    , error_callback_(nullptr)
{
    UNLOOK_LOG_INFO("Camera") << "CameraSystem constructor - libcamera-sync integration";

    // Create HardwareSyncCapture instance
    hardware_sync_capture_ = std::make_unique<HardwareSyncCapture>();
}

CameraSystem::~CameraSystem() {
    shutdown();
}

std::shared_ptr<CameraSystem> CameraSystem::getInstance() {
    std::lock_guard<std::mutex> lock(instance_mutex_);
    if (!instance_) {
        instance_ = std::shared_ptr<CameraSystem>(new CameraSystem());
    }
    return instance_;
}

bool CameraSystem::initialize() {
    if (system_ready_) {
        UNLOOK_LOG_WARNING("Camera") << "System already initialized";
        return true;
    }

    UNLOOK_LOG_INFO("Camera") << "Initializing camera system with IMX296 sensors...";

    // Configure camera parameters
    HardwareSyncCapture::CameraConfig capture_config;
    capture_config.width = IMX296_WIDTH;
    capture_config.height = IMX296_HEIGHT;
    capture_config.format = libcamera::formats::YUV420;
    capture_config.buffer_count = 4;  // 4 buffers for smooth capture

    // Initialize the hardware sync system
    if (!hardware_sync_capture_->initialize(capture_config)) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to initialize HardwareSyncCapture system";
        left_state_ = core::CameraState::ERROR;
        right_state_ = core::CameraState::ERROR;
        if (error_callback_) {
            error_callback_("Hardware sync camera initialization failed");
        }
        return false;
    }

    // Both cameras are now ready (handled internally by HardwareSyncCapture)
    system_ready_ = true;
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;

    UNLOOK_LOG_INFO("Camera") << "Camera system initialized successfully";
    UNLOOK_LOG_INFO("Camera") << "Hardware sync: XVS/XHS enabled, <1ms precision target";
    UNLOOK_LOG_INFO("Camera") << "Resolution: " << IMX296_WIDTH << "x" << IMX296_HEIGHT << " YUV420 (calibration resolution)";
    UNLOOK_LOG_INFO("Camera") << "Baseline: 70.017mm (from calibration)";

    return true;
}

void CameraSystem::shutdown() {
    stopCapture();

    std::lock_guard<std::mutex> lock(state_mutex_);

    if (hardware_sync_capture_) {
        hardware_sync_capture_.reset();
    }

    system_ready_ = false;
    left_state_ = core::CameraState::DISCONNECTED;
    right_state_ = core::CameraState::DISCONNECTED;

    UNLOOK_LOG_INFO("Camera") << "Camera system shutdown complete";
}

bool CameraSystem::startCapture(core::StereoFrameCallback frame_callback) {
    if (!system_ready_) {
        UNLOOK_LOG_ERROR("Camera") << "System not initialized";
        return false;
    }

    // CRITICAL FIX: Allow callback update even when capture is already running
    // This enables multiple widgets to receive frames from the same capture session
    if (capture_running_.load()) {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            frame_callback_ = frame_callback;
        }

        // Update HardwareSyncCapture callback
        hardware_sync_capture_->setFrameCallback([this](const HardwareSyncCapture::StereoFrame& sync_frame) {
            // Convert HardwareSyncCapture::StereoFrame to core::StereoFramePair
            core::StereoFramePair gui_frame;

            // Left frame (Camera 1 = MASTER)
            gui_frame.left_frame.image = sync_frame.left_image.clone();
            gui_frame.left_frame.timestamp_ns = sync_frame.left_timestamp_ns;
            gui_frame.left_frame.camera_id = core::CameraId::LEFT;
            gui_frame.left_frame.valid = !sync_frame.left_image.empty();

            // Right frame (Camera 0 = SLAVE)
            gui_frame.right_frame.image = sync_frame.right_image.clone();
            gui_frame.right_frame.timestamp_ns = sync_frame.right_timestamp_ns;
            gui_frame.right_frame.camera_id = core::CameraId::RIGHT;
            gui_frame.right_frame.valid = !sync_frame.right_image.empty();

            // Sync info
            gui_frame.sync_error_ms = sync_frame.sync_error_ms;
            gui_frame.synchronized = (sync_frame.sync_error_ms <= 1.0); // 1ms tolerance

            // Deliver frame pair to callback
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                if (frame_callback_ && gui_frame.left_frame.valid && gui_frame.right_frame.valid) {
                    frame_callback_(gui_frame);
                }
            }
        });

        UNLOOK_LOG_INFO("Camera") << "Capture already running - callback updated successfully";
        return true;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        frame_callback_ = frame_callback;
        capture_running_ = true;
        left_state_ = core::CameraState::CAPTURING;
        right_state_ = core::CameraState::CAPTURING;
    }

    // Start hardware synchronized capture
    if (!hardware_sync_capture_->start()) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to start hardware synchronized capture";
        capture_running_ = false;
        left_state_ = core::CameraState::ERROR;
        right_state_ = core::CameraState::ERROR;
        return false;
    }

    // Set up frame callback
    hardware_sync_capture_->setFrameCallback([this](const HardwareSyncCapture::StereoFrame& sync_frame) {
        // Convert HardwareSyncCapture::StereoFrame to core::StereoFramePair
        core::StereoFramePair gui_frame;

        // Left frame (Camera 1 = MASTER)
        gui_frame.left_frame.image = sync_frame.left_image.clone();
        gui_frame.left_frame.timestamp_ns = sync_frame.left_timestamp_ns;
        gui_frame.left_frame.camera_id = core::CameraId::LEFT;
        gui_frame.left_frame.valid = !sync_frame.left_image.empty();

        // Right frame (Camera 0 = SLAVE)
        gui_frame.right_frame.image = sync_frame.right_image.clone();
        gui_frame.right_frame.timestamp_ns = sync_frame.right_timestamp_ns;
        gui_frame.right_frame.camera_id = core::CameraId::RIGHT;
        gui_frame.right_frame.valid = !sync_frame.right_image.empty();

        // Sync info
        gui_frame.sync_error_ms = sync_frame.sync_error_ms;
        gui_frame.synchronized = (sync_frame.sync_error_ms <= 1.0); // 1ms tolerance

        // Deliver frame pair to callback
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (frame_callback_ && gui_frame.left_frame.valid && gui_frame.right_frame.valid) {
                frame_callback_(gui_frame);
            }
        }
    });

    UNLOOK_LOG_INFO("Camera") << "Hardware-synchronized stereo capture started";
    return true;
}

void CameraSystem::stopCapture() {
    if (!capture_running_.load()) {
        return;
    }

    capture_running_ = false;

    if (hardware_sync_capture_) {
        hardware_sync_capture_->stop();
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    left_state_ = core::CameraState::READY;
    right_state_ = core::CameraState::READY;

    UNLOOK_LOG_INFO("Camera") << "Capture stopped";
}

core::StereoFramePair CameraSystem::captureSingle() {
    core::StereoFramePair frame_pair;

    if (!system_ready_) {
        UNLOOK_LOG_ERROR("Camera") << "System not initialized";
        return frame_pair;
    }

    // Use HardwareSyncCapture for single frame
    HardwareSyncCapture::StereoFrame sync_frame;
    if (!hardware_sync_capture_->captureSingle(sync_frame, 3000)) {
        UNLOOK_LOG_ERROR("Camera") << "Failed to capture single frame";
        return frame_pair;
    }

    // Convert to GUI format
    frame_pair.left_frame.image = sync_frame.left_image.clone();
    frame_pair.left_frame.timestamp_ns = sync_frame.left_timestamp_ns;
    frame_pair.left_frame.camera_id = core::CameraId::LEFT;
    frame_pair.left_frame.valid = !sync_frame.left_image.empty();

    frame_pair.right_frame.image = sync_frame.right_image.clone();
    frame_pair.right_frame.timestamp_ns = sync_frame.right_timestamp_ns;
    frame_pair.right_frame.camera_id = core::CameraId::RIGHT;
    frame_pair.right_frame.valid = !sync_frame.right_image.empty();

    frame_pair.sync_error_ms = sync_frame.sync_error_ms;
    frame_pair.synchronized = (sync_frame.sync_error_ms <= 1.0);

    return frame_pair;
}

bool CameraSystem::isReady() const {
    return system_ready_;
}

core::CameraState CameraSystem::getCameraState(core::CameraId camera_id) const {
    return (camera_id == core::CameraId::LEFT) ? left_state_ : right_state_;
}

core::CameraConfig CameraSystem::getCameraConfig(core::CameraId camera_id) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return (camera_id == core::CameraId::LEFT) ? left_config_ : right_config_;
}

bool CameraSystem::configureCameras(const core::CameraConfig& left_config, const core::CameraConfig& right_config) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    left_config_ = left_config;
    right_config_ = right_config;
    // HardwareSyncCapture manages camera settings internally
    return true;
}

bool CameraSystem::setAutoExposure(core::CameraId camera_id, bool enabled) {
    // HardwareSyncCapture manages exposure internally
    return true;
}

bool CameraSystem::setExposureTime(core::CameraId camera_id, double exposure_us) {
    // HardwareSyncCapture manages exposure internally
    if (hardware_sync_capture_) {
        hardware_sync_capture_->setExposureTime(static_cast<int>(exposure_us));
        return true;
    }
    return false;
}

bool CameraSystem::setAutoGain(core::CameraId camera_id, bool enabled) {
    // HardwareSyncCapture manages gain internally
    return true;
}

bool CameraSystem::setGain(core::CameraId camera_id, double gain) {
    // HardwareSyncCapture manages gain internally
    if (hardware_sync_capture_) {
        hardware_sync_capture_->setGain(gain);
        return true;
    }
    return false;
}

bool CameraSystem::setFrameRate(double fps) {
    // Frame rate is fixed at 30 FPS in HardwareSyncCapture
    return true;
}

double CameraSystem::getCurrentFrameRate() const {
    return TARGET_FPS;
}

std::string CameraSystem::getCameraInfo(core::CameraId camera_id) const {
    if (camera_id == core::CameraId::LEFT) {
        return "LEFT/MASTER IMX296 1280x720 @ 30 FPS";
    } else {
        return "RIGHT/SLAVE IMX296 1280x720 @ 30 FPS";
    }
}

bool CameraSystem::isHardwareSyncEnabled() const {
    return true; // Always enabled with HardwareSyncCapture
}

double CameraSystem::getAverageSyncError() const {
    // TODO: Expose sync statistics from HardwareSyncCapture
    return 0.0;
}

void CameraSystem::setErrorCallback(core::ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    error_callback_ = callback;
}

} // namespace gui
} // namespace camera
} // namespace unlook
