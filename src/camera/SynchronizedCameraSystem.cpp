#include <unlook/camera/CameraSystem.hpp>
#include <unlook/camera/LibcameraSyncDevice.hpp>
#include <unlook/camera/HardwareSyncManager.hpp>
#include <unlook/camera/AutoExposure.hpp>
#include <unlook/camera/CameraUtils.hpp>
#include <unlook/core/Logger.hpp>
#include <iostream>
#include <chrono>
#include <thread>

// Compatibility macros
#define LOG_INFO(msg) UNLOOK_LOG_INFO("SynchronizedCameraSystem") << msg
#define LOG_ERROR(msg) UNLOOK_LOG_ERROR("SynchronizedCameraSystem") << msg
#define LOG_WARNING(msg) UNLOOK_LOG_WARNING("SynchronizedCameraSystem") << msg
#define LOG_DEBUG(msg) UNLOOK_LOG_DEBUG("SynchronizedCameraSystem") << msg

namespace unlook {
namespace camera {

/**
 * Synchronized Camera System Implementation
 * 
 * This implementation uses the third-party libcamera-sync-fix library
 * for proper hardware synchronization with XVS/XHS pins.
 * 
 * Camera Mapping (FIXED - DO NOT CHANGE):
 * - Camera 1: /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
 * - Camera 0: /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
 * 
 * Startup Sequence:
 * 1. Initialize hardware sync manager
 * 2. Start MASTER camera (Camera 1 = LEFT)
 * 3. Wait for master stabilization
 * 4. Start SLAVE camera (Camera 0 = RIGHT)
 * 5. Begin synchronized capture
 */
class SynchronizedCameraSystem {
public:
    static std::shared_ptr<SynchronizedCameraSystem> getInstance() {
        static std::shared_ptr<SynchronizedCameraSystem> instance;
        static std::mutex mutex;
        
        std::lock_guard<std::mutex> lock(mutex);
        if (!instance) {
            instance = std::make_shared<SynchronizedCameraSystem>();
        }
        return instance;
    }
    
    SynchronizedCameraSystem() {
        LOG_INFO("Creating SynchronizedCameraSystem with libcamera-sync-fix");
    }
    
    ~SynchronizedCameraSystem() {
        LOG_INFO("Destroying SynchronizedCameraSystem");
        shutdown();
    }
    
    bool initialize() {
        if (initialized_) {
            LOG_WARNING("System already initialized");
            return true;
        }
        
        LOG_INFO("Initializing synchronized camera system");
        
        // Initialize hardware sync manager
        sync_manager_ = std::make_unique<HardwareSyncManager>();
        HardwareSyncManager::SyncConfig sync_config;
        sync_config.mode = HardwareSyncManager::SyncMode::XVS_XHS;
        sync_config.xvs_gpio = 17;  // XVS pin
        sync_config.xhs_gpio = 27;  // XHS pin
        sync_config.mas_gpio = 22;  // MAS pin
        sync_config.target_fps = 30.0;
        sync_config.sync_tolerance_ms = 1.0;
        sync_config.enable_diagnostics = true;
        
        if (!sync_manager_->initialize(sync_config)) {
            LOG_ERROR("Failed to initialize hardware sync manager");
            return false;
        }
        
        LOG_INFO("Hardware sync manager initialized");
        
        // CRITICAL: Initialize MASTER camera first (Camera 1 = LEFT)
        LOG_INFO("Initializing MASTER camera (Camera 1 = LEFT)");
        master_camera_ = std::make_unique<LibcameraSyncDevice>(
            "/base/soc/i2c0mux/i2c@1/imx296@1a", 
            LibcameraSyncDevice::Role::MASTER
        );
        
        LibcameraSyncDevice::CameraConfig config;
        config.width = 1456;
        config.height = 1088;
        config.format = libcamera::formats::SBGGR10;
        config.buffer_count = 4;
        config.exposure_time_us = 10000.0;
        config.analog_gain = 1.0;
        config.target_fps = 30.0;
        config.enable_sync = true;
        
        if (!master_camera_->initialize(config)) {
            LOG_ERROR("Failed to initialize master camera");
            return false;
        }
        
        // Configure master role in sync manager
        sync_manager_->configureCameraRole(1, HardwareSyncManager::CameraRole::MASTER);
        
        // Start master camera
        if (!master_camera_->start()) {
            LOG_ERROR("Failed to start master camera");
            return false;
        }
        
        LOG_INFO("Master camera started successfully");
        
        // Wait for master camera to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Initialize SLAVE camera (Camera 0 = RIGHT)
        LOG_INFO("Initializing SLAVE camera (Camera 0 = RIGHT)");
        slave_camera_ = std::make_unique<LibcameraSyncDevice>(
            "/base/soc/i2c0mux/i2c@0/imx296@1a",
            LibcameraSyncDevice::Role::SLAVE
        );
        
        if (!slave_camera_->initialize(config)) {
            LOG_ERROR("Failed to initialize slave camera");
            master_camera_->stop();
            return false;
        }
        
        // Configure slave role in sync manager
        sync_manager_->configureCameraRole(0, HardwareSyncManager::CameraRole::SLAVE);
        
        // Start slave camera
        if (!slave_camera_->start()) {
            LOG_ERROR("Failed to start slave camera");
            master_camera_->stop();
            return false;
        }
        
        LOG_INFO("Slave camera started successfully");
        
        // Enable hardware sync on both cameras
        master_camera_->enableHardwareSync(true);
        slave_camera_->enableHardwareSync(true);
        
        // Start hardware synchronization
        if (!sync_manager_->startSync()) {
            LOG_ERROR("Failed to start hardware synchronization");
            return false;
        }
        
        LOG_INFO("Hardware synchronization started");
        
        initialized_ = true;
        LOG_INFO("Synchronized camera system initialized successfully");
        
        return true;
    }
    
    bool captureSynchronizedPair(cv::Mat& left_frame, cv::Mat& right_frame, double& sync_error_ms) {
        if (!initialized_) {
            LOG_ERROR("System not initialized");
            return false;
        }
        
        LibcameraSyncDevice::FrameMetadata left_meta, right_meta;
        
        // Capture from both cameras
        bool left_success = master_camera_->captureFrame(left_frame, left_meta, 1000);
        bool right_success = slave_camera_->captureFrame(right_frame, right_meta, 1000);
        
        if (!left_success || !right_success) {
            LOG_ERROR("Failed to capture synchronized pair");
            return false;
        }
        
        // Calculate sync error
        sync_error_ms = sync_manager_->measureSyncError(left_meta.timestamp_ns, right_meta.timestamp_ns);
        
        if (sync_error_ms > 1.0) {
            LOG_WARNING("Sync error exceeds 1ms: " + std::to_string(sync_error_ms) + "ms");
        }
        
        return true;
    }
    
    void shutdown() {
        if (!initialized_) {
            return;
        }
        
        LOG_INFO("Shutting down synchronized camera system");
        
        // Stop sync
        if (sync_manager_) {
            sync_manager_->stopSync();
        }
        
        // Stop cameras (reverse order - slave first, then master)
        if (slave_camera_) {
            slave_camera_->stop();
            slave_camera_.reset();
        }
        
        if (master_camera_) {
            master_camera_->stop();
            master_camera_.reset();
        }
        
        sync_manager_.reset();
        
        initialized_ = false;
        LOG_INFO("Synchronized camera system shut down");
    }
    
    HardwareSyncManager::SyncStatus getSyncStatus() const {
        if (sync_manager_) {
            return sync_manager_->getStatus();
        }
        return HardwareSyncManager::SyncStatus();
    }
    
    bool isInitialized() const {
        return initialized_;
    }
    
private:
    std::unique_ptr<LibcameraSyncDevice> master_camera_;  // Camera 1 = LEFT
    std::unique_ptr<LibcameraSyncDevice> slave_camera_;   // Camera 0 = RIGHT
    std::unique_ptr<HardwareSyncManager> sync_manager_;
    std::atomic<bool> initialized_{false};
};

} // namespace camera
} // namespace unlook

// Test application for synchronized capture
int main(int argc, char* argv[]) {
    using namespace unlook::camera;
    
    LOG_INFO("=== Unlook Synchronized Camera Test ===");
    LOG_INFO("Using third-party libcamera-sync-fix for hardware synchronization");
    
    // Set environment for third-party libcamera
    setenv("LD_LIBRARY_PATH", 
           "/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/build/src/libcamera:"
           "/home/alessandro/unlook-standalone/third-party/libcamera-sync-fix/build/src/libcamera/base",
           1);
    
    // Create synchronized camera system
    auto camera_system = SynchronizedCameraSystem::getInstance();
    
    // Initialize system
    LOG_INFO("Initializing camera system...");
    if (!camera_system->initialize()) {
        LOG_ERROR("Failed to initialize camera system");
        return 1;
    }
    
    LOG_INFO("Camera system initialized successfully");
    
    // Capture test frames
    LOG_INFO("Capturing synchronized frames...");
    
    for (int i = 0; i < 10; ++i) {
        cv::Mat left_frame, right_frame;
        double sync_error_ms;
        
        if (camera_system->captureSynchronizedPair(left_frame, right_frame, sync_error_ms)) {
            LOG_INFO("Frame " + std::to_string(i) + " captured - Sync error: " + 
                     std::to_string(sync_error_ms) + "ms");
            
            // Save frames for inspection
            if (i == 0) {
                cv::imwrite("left_frame.png", left_frame);
                cv::imwrite("right_frame.png", right_frame);
                LOG_INFO("First frame pair saved to left_frame.png and right_frame.png");
            }
        } else {
            LOG_ERROR("Failed to capture frame " + std::to_string(i));
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Get sync statistics
    auto sync_status = camera_system->getSyncStatus();
    LOG_INFO("Sync Statistics:");
    LOG_INFO("  Frames captured: " + std::to_string(sync_status.frame_count));
    LOG_INFO("  Sync errors: " + std::to_string(sync_status.sync_errors));
    LOG_INFO("  Max sync error: " + std::to_string(sync_status.max_sync_error_ms) + "ms");
    LOG_INFO("  Measured FPS: " + std::to_string(sync_status.measured_fps));
    
    // Shutdown
    LOG_INFO("Shutting down camera system...");
    camera_system->shutdown();
    
    LOG_INFO("Test completed successfully");
    
    return 0;
}