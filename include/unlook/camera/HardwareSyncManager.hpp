#pragma once

#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <string>

namespace unlook {
namespace camera {

/**
 * Hardware Synchronization Manager for IMX296 Global Shutter Cameras
 * 
 * Manages XVS (External Vertical Sync) and XHS (External Horizontal Sync) pins
 * for precise hardware synchronization between master and slave cameras.
 * 
 * Pin Configuration:
 * - XVS (GPIO 17): External Vertical Sync - frame synchronization
 * - XHS (GPIO 27): External Horizontal Sync - line synchronization  
 * - MAS (GPIO 22): Master/Slave selection (soldered on camera sink)
 * 
 * Synchronization precision target: <1ms per frame
 */
class HardwareSyncManager {
public:
    enum class SyncMode {
        DISABLED,           // No hardware sync
        XVS_ONLY,          // Vertical sync only (frame level)
        XVS_XHS,           // Full sync (frame + line level)
        EXTERNAL_TRIGGER   // External trigger mode
    };
    
    enum class CameraRole {
        MASTER,            // Camera 1: /base/soc/i2c0mux/i2c@1/imx296@1a
        SLAVE              // Camera 0: /base/soc/i2c0mux/i2c@0/imx296@1a
    };
    
    struct SyncConfig {
        SyncMode mode = SyncMode::XVS_XHS;
        uint32_t xvs_gpio = 17;      // XVS pin (External Vertical Sync)
        uint32_t xhs_gpio = 27;      // XHS pin (External Horizontal Sync)
        uint32_t mas_gpio = 22;      // MAS pin (Master/Slave select)
        double target_fps = 30.0;     // Target frame rate
        double sync_tolerance_ms = 1.0; // Max sync error tolerance
        bool enable_diagnostics = true;
    };
    
    struct SyncStatus {
        bool initialized = false;
        bool sync_active = false;
        SyncMode current_mode = SyncMode::DISABLED;
        double measured_fps = 0.0;
        double sync_error_ms = 0.0;
        double max_sync_error_ms = 0.0;
        uint64_t frame_count = 0;
        uint64_t sync_errors = 0;
        std::string error_message;
    };
    
    HardwareSyncManager();
    ~HardwareSyncManager();
    
    // Initialize hardware synchronization
    bool initialize(const SyncConfig& config);
    
    // Configure camera role (must be called before starting sync)
    bool configureCameraRole(int camera_id, CameraRole role);
    
    // Start/stop synchronization
    bool startSync();
    void stopSync();
    
    // Enable/disable specific sync pins
    bool enableXVS(bool enable);
    bool enableXHS(bool enable);
    
    // Generate sync pulse (for master camera)
    bool generateSyncPulse();
    
    // Wait for sync signal (for slave camera)
    bool waitForSync(uint32_t timeout_ms = 100);
    
    // Measure sync precision
    double measureSyncError(uint64_t master_timestamp_ns, uint64_t slave_timestamp_ns);
    
    // Update sync statistics
    void updateSyncStats(double error_ms);
    
    // Get current sync status
    SyncStatus getStatus() const;
    
    // Reset sync statistics
    void resetStats();
    
    // Check if hardware sync is supported
    static bool isHardwareSyncSupported();
    
    // Get pin status for diagnostics
    bool getXVSState() const;
    bool getXHSState() const;
    bool getMASState() const;
    
private:
    class Impl;
    std::unique_ptr<Impl> pImpl;
    
    mutable std::mutex mutex_;
    std::atomic<bool> sync_active_;
    SyncConfig config_;
    SyncStatus status_;
    
    // GPIO file descriptors
    int xvs_fd_ = -1;
    int xhs_fd_ = -1;
    int mas_fd_ = -1;
    
    // Timing tracking
    std::chrono::steady_clock::time_point last_sync_time_;
    std::chrono::steady_clock::time_point sync_start_time_;
    
    // Initialize GPIO pins
    bool initializeGPIO(uint32_t gpio, const std::string& direction, int& fd);
    void cleanupGPIO(int& fd);
    
    // Set GPIO state
    bool setGPIOState(int fd, bool state);
    bool getGPIOState(int fd) const;
    
    // Export/unexport GPIO
    bool exportGPIO(uint32_t gpio);
    void unexportGPIO(uint32_t gpio);
};

} // namespace camera
} // namespace unlook