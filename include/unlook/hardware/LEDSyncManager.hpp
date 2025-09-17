#pragma once

#include <unlook/hardware/AS1170Controller.hpp>
#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <queue>

namespace unlook {
namespace hardware {

/**
 * LED Synchronization Manager for Camera-LED Coordination
 *
 * Provides precise timing synchronization between AS1170 LED controller
 * and camera capture systems. Ensures optimal illumination timing for
 * structured light applications and depth capture.
 *
 * Key Features:
 * - Microsecond precision LED-camera synchronization (<50μs accuracy)
 * - Pre-capture LED activation for optimal illumination
 * - Synchronized patterns for structured light projection
 * - Face recognition optimized illumination
 * - Integration with existing camera hardware sync system
 * - Thread-safe operation with real-time scheduling
 *
 * Integration Points (FINAL HARDWARE CONFIG):
 * - Camera XVS (GPIO 17) for frame sync detection
 * - AS1170 strobe (GPIO 19) for LED control (via AS1170Controller)
 * - AS1170 I2C (Bus 1, Address 0x30) for LED driver communication
 * - Hardware sync coordination with HardwareSyncManager
 */
class LEDSyncManager {
public:
    enum class SyncMode {
        DISABLED,           // No LED synchronization
        PRE_CAPTURE,        // LED on before capture, off after
        STROBE_SYNC,        // Synchronized strobe with exposure
        CONTINUOUS,         // Continuous illumination during capture
        PATTERN_SYNC        // Structured light pattern synchronization
    };

    enum class IlluminationPattern {
        FLOOD_ONLY,         // LED2 flood illumination only
        VCSEL_ONLY,         // LED1 VCSEL projection only
        ALTERNATING,        // Alternate between flood and VCSEL
        COMBINED,           // Both LEDs simultaneously
        ADAPTIVE            // Adaptive based on scene conditions
    };

    struct SyncConfig {
        SyncMode mode;
        IlluminationPattern pattern;

        // Timing parameters (all in microseconds)
        uint32_t pre_capture_delay_us;     // LED on delay before capture
        uint32_t strobe_duration_us;       // LED strobe duration
        uint32_t post_capture_delay_us;    // LED off delay after capture
        uint32_t sync_tolerance_us;        // Maximum sync error tolerance

        // Current settings
        uint16_t flood_current_ma;         // Flood LED current (LED2)
        uint16_t vcsel_current_ma;         // VCSEL LED current (LED1)

        // Pattern timing for structured light
        uint32_t pattern_cycle_us;         // Full pattern cycle duration
        uint32_t pattern_steps;            // Number of pattern steps

        // Safety and monitoring
        bool enable_thermal_monitoring;
        bool enable_sync_diagnostics;
        float max_duty_cycle;              // Maximum LED duty cycle (30%)

        // Default constructor
        SyncConfig() :
            mode(SyncMode::STROBE_SYNC),
            pattern(IlluminationPattern::FLOOD_ONLY),
            pre_capture_delay_us(5000),     // LED on delay before capture
            strobe_duration_us(1000),       // LED strobe duration
            post_capture_delay_us(1000),    // LED off delay after capture
            sync_tolerance_us(50),          // Maximum sync error tolerance
            flood_current_ma(150),          // Flood LED current (LED2)
            vcsel_current_ma(250),          // VCSEL LED current (LED1)
            pattern_cycle_us(10000),        // Full pattern cycle duration
            pattern_steps(8),               // Number of pattern steps
            enable_thermal_monitoring(true),
            enable_sync_diagnostics(true),
            max_duty_cycle(0.3f)            // Maximum LED duty cycle (30%)
        {}
    };

    struct SyncStatus {
        bool initialized = false;
        bool sync_active = false;
        SyncMode current_mode = SyncMode::DISABLED;
        IlluminationPattern current_pattern = IlluminationPattern::FLOOD_ONLY;

        // Performance metrics
        uint64_t sync_cycles = 0;
        uint64_t sync_errors = 0;
        double avg_sync_error_us = 0.0;
        double max_sync_error_us = 0.0;
        double measured_fps = 0.0;

        // Current state
        bool led1_active = false;
        bool led2_active = false;
        uint16_t current_led1_ma = 0;
        uint16_t current_led2_ma = 0;

        // Timing statistics
        std::chrono::steady_clock::time_point last_sync;
        double current_duty_cycle = 0.0;

        std::string error_message;
    };

    // Callback types for integration
    using SyncCallback = std::function<void(bool led_state, uint64_t timestamp_ns)>;
    using CaptureCallback = std::function<void(uint64_t timestamp_ns)>;
    using ErrorCallback = std::function<void(const std::string& error)>;

    LEDSyncManager();
    ~LEDSyncManager();

    /**
     * Initialize LED sync manager with AS1170 controller
     * @param as1170_controller Shared pointer to initialized AS1170 controller
     * @param config Synchronization configuration
     * @return true if initialization successful
     */
    bool initialize(std::shared_ptr<AS1170Controller> as1170_controller,
                   const SyncConfig& config = SyncConfig{});

    /**
     * Shutdown synchronization system
     */
    void shutdown();

    /**
     * Start LED synchronization with specified mode
     * @param mode Synchronization mode to activate
     * @return true if started successfully
     */
    bool startSync(SyncMode mode = SyncMode::STROBE_SYNC);

    /**
     * Stop LED synchronization
     */
    void stopSync();

    /**
     * Trigger synchronized capture with LED illumination
     * This is the main method called by camera capture system
     * @param exposure_time_us Camera exposure time for timing coordination
     * @param callback Optional callback when capture sync is complete
     * @return true if sync triggered successfully
     */
    bool triggerSyncCapture(uint32_t exposure_time_us, CaptureCallback callback = nullptr);

    /**
     * Pre-activate LEDs for upcoming capture
     * Useful for reducing capture latency
     * @param pattern Illumination pattern to use
     * @param duration_us How long to keep LEDs active
     * @return true if successful
     */
    bool preActivateIllumination(IlluminationPattern pattern, uint32_t duration_us = 10000);

    /**
     * Configure synchronization parameters
     * @param config New configuration to apply
     * @return true if configuration valid and applied
     */
    bool configureSynchronization(const SyncConfig& config);

    /**
     * Set illumination pattern for next capture
     * @param pattern Pattern type to use
     * @param flood_current_ma Current for flood LED (LED2)
     * @param vcsel_current_ma Current for VCSEL LED (LED1)
     * @return true if pattern configured successfully
     */
    bool setIlluminationPattern(IlluminationPattern pattern,
                               uint16_t flood_current_ma = 0,
                               uint16_t vcsel_current_ma = 0);

    /**
     * Synchronize with external frame sync signal (XVS)
     * Called by camera system when frame sync detected
     * @param timestamp_ns Frame sync timestamp
     * @return true if sync handled successfully
     */
    bool onFrameSync(uint64_t timestamp_ns);

    /**
     * Emergency stop - immediate LED shutdown
     * Thread-safe, can be called from any context
     */
    void emergencyStop();

    /**
     * Get current synchronization status
     */
    SyncStatus getStatus() const;

    /**
     * Get current configuration
     */
    SyncConfig getConfig() const;

    /**
     * Register callbacks for integration
     */
    void setSyncCallback(SyncCallback callback);
    void setCaptureCallback(CaptureCallback callback);
    void setErrorCallback(ErrorCallback callback);

    /**
     * Manual LED control (bypasses synchronization)
     * Use with caution - for testing and debugging only
     */
    bool setManualLEDState(AS1170Controller::LEDChannel channel,
                          bool enable, uint16_t current_ma = 0);

    /**
     * Calculate optimal timing for camera integration
     * @param exposure_time_us Camera exposure time
     * @param fps Target frame rate
     * @return Suggested pre-capture delay
     */
    uint32_t calculateOptimalTiming(uint32_t exposure_time_us, double fps) const;

    /**
     * Validate synchronization accuracy
     * @return Current sync error statistics
     */
    struct SyncAccuracy {
        double mean_error_us;
        double std_dev_us;
        double max_error_us;
        uint64_t samples;
    };
    SyncAccuracy measureSyncAccuracy() const;

    /**
     * Check if system is properly synchronized
     */
    bool isInSync() const;

private:
    std::shared_ptr<AS1170Controller> as1170_controller_;
    SyncConfig config_;
    mutable std::mutex mutex_;
    mutable std::mutex status_mutex_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> sync_active_{false};
    std::atomic<bool> emergency_stop_{false};

    SyncStatus status_;

    // Synchronization thread and control
    std::unique_ptr<std::thread> sync_thread_;
    std::condition_variable sync_condition_;
    std::mutex sync_thread_mutex_;

    // Timing tracking
    std::chrono::steady_clock::time_point sync_start_time_;
    std::chrono::steady_clock::time_point last_frame_sync_;
    std::queue<double> sync_error_history_;
    static constexpr size_t MAX_ERROR_HISTORY = 100;

    // Callbacks
    SyncCallback sync_callback_;
    CaptureCallback capture_callback_;
    ErrorCallback error_callback_;

    // Pending sync operations
    struct PendingSyncOp {
        std::chrono::steady_clock::time_point trigger_time;
        uint32_t duration_us;
        IlluminationPattern pattern;
        CaptureCallback callback;
    };
    std::queue<PendingSyncOp> pending_ops_;

    // Duty cycle monitoring
    std::chrono::steady_clock::time_point duty_cycle_start_;
    std::chrono::duration<double> led_on_time_{0};

    // Private implementation methods
    void syncThreadWorker();
    bool executeSync(const PendingSyncOp& op);
    bool activatePattern(IlluminationPattern pattern, uint32_t duration_us);
    bool deactivatePattern();

    // Timing and synchronization
    bool waitForNextFrameSync(uint32_t timeout_us = 100000);
    uint64_t getCurrentTimestamp() const;
    double calculateSyncError(uint64_t expected_ns, uint64_t actual_ns) const;
    void updateSyncStatistics(double error_us);

    // Pattern control methods
    bool activateFloodOnly(uint16_t current_ma);
    bool activateVCSELOnly(uint16_t current_ma);
    bool activateAlternating(uint16_t flood_ma, uint16_t vcsel_ma, uint32_t duration_us);
    bool activateCombined(uint16_t flood_ma, uint16_t vcsel_ma);
    bool activateAdaptive(uint32_t exposure_time_us);

    // Safety and monitoring
    bool checkDutyCycle() const;
    bool validateTiming(uint32_t duration_us) const;
    void updateDutyCycle(bool led_active);
    void handleSyncError(const std::string& error);

    // Utility methods
    bool isValidPattern(IlluminationPattern pattern) const;
    bool isValidConfig(const SyncConfig& config) const;
    void resetStatistics();
    void updateStatus();
};

} // namespace hardware
} // namespace unlook