#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace unlook {
namespace camera {

/**
 * Hardware synchronization manager for stereo cameras
 * 
 * Manages XVS (External Vertical Sync) and XHS (External Horizontal Sync)
 * signals to ensure frame capture synchronization with <1ms precision.
 */
class CameraSynchronizer {
public:
    /**
     * Synchronization mode
     */
    enum class SyncMode {
        NONE,           // No synchronization
        SOFTWARE,       // Software-based sync (fallback)
        HARDWARE_XVS,   // Hardware sync using XVS only
        HARDWARE_FULL   // Full hardware sync (XVS + XHS)
    };
    
    /**
     * Sync statistics
     */
    struct SyncStats {
        uint64_t totalSyncs;
        uint64_t failedSyncs;
        double avgSyncErrorMs;
        double maxSyncErrorMs;
        double minSyncErrorMs;
        std::chrono::steady_clock::time_point lastSyncTime;
    };
    
    /**
     * Constructor
     */
    CameraSynchronizer();
    
    /**
     * Destructor
     */
    ~CameraSynchronizer();
    
    /**
     * Initialize synchronizer with specified mode
     */
    bool initialize(SyncMode mode = SyncMode::HARDWARE_FULL);
    
    /**
     * Shutdown synchronizer
     */
    void shutdown();
    
    /**
     * Start synchronization
     */
    bool start();
    
    /**
     * Stop synchronization
     */
    void stop();
    
    /**
     * Configure GPIO pins for hardware sync
     * @param xvsPin GPIO pin for XVS signal
     * @param xhsPin GPIO pin for XHS signal (optional)
     * @param masPin GPIO pin for Master/Slave selection
     */
    bool configureGPIO(int xvsPin = -1, int xhsPin = -1, int masPin = -1);
    
    /**
     * Trigger sync pulse (for master camera)
     */
    bool triggerSync();
    
    /**
     * Wait for sync signal (for slave camera)
     * @param timeoutMs Maximum wait time in milliseconds
     */
    bool waitForSync(int timeoutMs = 100);
    
    /**
     * Measure sync error between two timestamps
     * @param timestamp1 First camera timestamp (ns)
     * @param timestamp2 Second camera timestamp (ns)
     * @return Sync error in milliseconds
     */
    double measureSyncError(uint64_t timestamp1, uint64_t timestamp2) const;
    
    /**
     * Validate synchronization precision
     * @param errorToleranceMs Maximum acceptable error in ms
     */
    bool validateSync(double errorToleranceMs = 1.0) const;
    
    /**
     * Get current sync mode
     */
    SyncMode getSyncMode() const { return syncMode_; }
    
    /**
     * Get sync statistics
     */
    SyncStats getStats() const;
    
    /**
     * Reset statistics
     */
    void resetStats();
    
    /**
     * Check if synchronized
     */
    bool isSynchronized() const { return synchronized_; }
    
    /**
     * Set sync frequency (Hz)
     */
    bool setSyncFrequency(double frequencyHz);
    
    /**
     * Get sync frequency (Hz)
     */
    double getSyncFrequency() const { return syncFrequency_; }
    
    // Hardware-specific configuration
    
    /**
     * Configure IMX296 sensor sync registers
     */
    bool configureIMX296Sync(bool master);
    
    /**
     * Enable XVS output (master) or input (slave)
     */
    bool enableXVS(bool master);
    
    /**
     * Enable XHS output (master) or input (slave)
     */
    bool enableXHS(bool master);
    
    /**
     * Set MAS pin state (high for master, low for slave)
     */
    bool setMASPin(bool master);
    
private:
    // GPIO control methods
    bool initializeGPIO();
    void cleanupGPIO();
    bool setGPIODirection(int pin, bool output);
    bool setGPIOValue(int pin, bool value);
    bool getGPIOValue(int pin);
    bool exportGPIO(int pin);
    bool unexportGPIO(int pin);
    
    // Sync pulse generation thread (for master)
    void syncPulseThread();
    
    // Update statistics
    void updateStats(double syncErrorMs);
    
    // Member variables
    SyncMode syncMode_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> synchronized_{false};
    
    // GPIO pins
    int xvsPin_ = -1;
    int xhsPin_ = -1;
    int masPin_ = -1;
    std::vector<int> exportedPins_;
    
    // Sync parameters
    double syncFrequency_ = 30.0;  // Hz
    std::chrono::microseconds syncPeriod_;
    
    // Threading
    std::thread syncThread_;
    std::atomic<bool> shouldStop_{false};
    
    // Statistics
    mutable std::mutex statsMutex_;
    SyncStats stats_;
    
    // Timing
    std::chrono::steady_clock::time_point lastSyncTime_;
    
    // Error tracking
    std::string lastError_;
};

} // namespace camera
} // namespace unlook