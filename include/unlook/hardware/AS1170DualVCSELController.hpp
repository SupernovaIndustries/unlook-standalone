#pragma once

#include <unlook/hardware/AS1170Controller.hpp>
#include <unlook/camera/CameraSystem.hpp>
#include <memory>
#include <chrono>
#include <functional>
#include <opencv2/core.hpp>
#include <atomic>
#include <mutex>

namespace unlook {
namespace hardware {

/**
 * AS1170 Dual VCSEL Controller for Temporal Matching
 *
 * Implements temporal stereo matching with dual VCSEL projectors:
 * - VCSEL1 (LED1): 2cm from LEFT camera, 15K dots, 200mA
 * - VCSEL2 (LED2): 2cm from RIGHT camera, 15K dots, 200mA
 *
 * Temporal Matching Strategy:
 * 1. Frame A: VCSEL1 ON (projects from left camera side)
 * 2. Frame B: VCSEL2 ON (projects from right camera side)
 * 3. Frame C: Both OFF (ambient illumination for subtraction)
 *
 * This temporal approach eliminates pattern correspondence ambiguity
 * by projecting from different angles and subtracting ambient.
 *
 * Hardware Configuration:
 * - AS1170: I2C bus 1, address 0x30, GPIO 19 strobe
 * - VCSEL1: OSRAM BELAGO 15k points (LED1)
 * - VCSEL2: OSRAM BELAGO 15k points (LED2)
 * - Target Current: 200mA per VCSEL (safety margin from 446mA max)
 * - Settle Time: 50ms between activations (thermal stability)
 *
 * Safety Features:
 * - Temperature monitoring (70C max)
 * - Overcurrent protection (200mA limit)
 * - Emergency shutdown (<5ms response)
 * - Timeout protection (max ON time)
 * - Thermal throttling on overheating
 *
 * Thread Safety: All public methods are thread-safe
 */
class AS1170DualVCSELController {
public:
    /**
     * Triple frame capture result for temporal matching
     */
    struct TripleFrameCapture {
        // Frame A: VCSEL1 ON (pattern from left camera perspective)
        cv::Mat frame_vcsel1_left;
        cv::Mat frame_vcsel1_right;

        // Frame B: VCSEL2 ON (pattern from right camera perspective)
        cv::Mat frame_vcsel2_left;
        cv::Mat frame_vcsel2_right;

        // Frame C: Ambient (no pattern, for subtraction)
        cv::Mat frame_ambient_left;
        cv::Mat frame_ambient_right;

        // Metadata
        std::chrono::steady_clock::time_point capture_time;
        bool is_valid = false;

        // Timing information (microseconds)
        uint64_t vcsel1_activation_us = 0;
        uint64_t vcsel2_activation_us = 0;
        uint64_t total_sequence_us = 0;

        // Temperature during capture
        float temperature_c = 0.0f;

        // Frame synchronization errors
        double vcsel1_sync_error_ms = 0.0;
        double vcsel2_sync_error_ms = 0.0;
        double ambient_sync_error_ms = 0.0;
    };

    /**
     * VCSEL activation configuration
     */
    struct VCSELConfig {
        uint16_t vcsel1_current_ma = 200;      // VCSEL1 current (LED1)
        uint16_t vcsel2_current_ma = 200;      // VCSEL2 current (LED2)
        uint32_t settle_time_ms = 50;          // Time between activations
        uint32_t capture_delay_ms = 10;        // Delay after activation before capture
        uint32_t max_on_time_ms = 5000;        // Maximum continuous ON time (safety)
        bool enable_thermal_monitoring = true;  // Monitor temperature during operation
        float max_operating_temp_c = 70.0f;    // Maximum safe operating temperature

        // Strobe configuration (if using strobe mode instead of torch)
        bool use_strobe_mode = false;          // Use strobe instead of continuous
        uint32_t strobe_duration_us = 1000;    // Strobe pulse duration
    };

    /**
     * VCSEL status information
     */
    struct VCSELStatus {
        bool vcsel1_active = false;
        bool vcsel2_active = false;
        uint16_t vcsel1_current_ma = 0;
        uint16_t vcsel2_current_ma = 0;
        float temperature_c = 0.0f;
        bool thermal_throttling = false;
        uint64_t total_captures = 0;
        uint64_t total_on_time_ms = 0;
        std::chrono::steady_clock::time_point last_activation;
        std::string error_message;
    };

    /**
     * Constructor - initializes dual VCSEL controller
     */
    AS1170DualVCSELController();

    /**
     * Destructor - ensures safe shutdown
     */
    ~AS1170DualVCSELController();

    /**
     * Get singleton instance
     */
    static std::shared_ptr<AS1170DualVCSELController> getInstance();

    /**
     * Initialize dual VCSEL system
     * @param camera_system Camera system for synchronized capture
     * @param config VCSEL configuration parameters
     * @return true if initialization successful
     */
    bool initialize(std::shared_ptr<camera::CameraSystem> camera_system);
    bool initialize(
        std::shared_ptr<camera::CameraSystem> camera_system,
        const VCSELConfig& config
    );

    /**
     * Shutdown and cleanup
     */
    void shutdown();

    /**
     * Capture triple frame sequence for temporal matching
     *
     * Sequence:
     * 1. Activate VCSEL1, wait settle time, capture frame pair
     * 2. Activate VCSEL2, wait settle time, capture frame pair
     * 3. Deactivate both, wait settle time, capture ambient frame pair
     *
     * @param result Output triple frame capture structure
     * @return true if capture successful
     */
    bool captureTemporalSequence(TripleFrameCapture& result);

    /**
     * Activate VCSEL1 only (LED1)
     * @param current_ma Current in mA (0 = use config default)
     * @return true if successful
     */
    bool activateVCSEL1(uint16_t current_ma = 0);

    /**
     * Activate VCSEL2 only (LED2)
     * @param current_ma Current in mA (0 = use config default)
     * @return true if successful
     */
    bool activateVCSEL2(uint16_t current_ma = 0);

    /**
     * Deactivate VCSEL1
     * @return true if successful
     */
    bool deactivateVCSEL1();

    /**
     * Deactivate VCSEL2
     * @return true if successful
     */
    bool deactivateVCSEL2();

    /**
     * Deactivate both VCSELs immediately
     * @return true if successful
     */
    bool deactivateBoth();

    /**
     * Emergency shutdown - immediate deactivation of both VCSELs
     * Thread-safe, can be called from any context
     */
    void emergencyShutdown();

    /**
     * Read current temperature
     * @return Temperature in Celsius, -999.0f on error
     */
    float readTemperature();

    /**
     * Get current VCSEL status
     */
    VCSELStatus getStatus() const;

    /**
     * Get current configuration
     */
    VCSELConfig getConfig() const;

    /**
     * Check if system is initialized
     */
    bool isInitialized() const { return initialized_.load(); }

    /**
     * Check if any VCSEL is currently active
     */
    bool isAnyVCSELActive() const;

    /**
     * Update VCSEL configuration (safe to call during operation)
     * @param config New configuration parameters
     * @return true if update successful
     */
    bool updateConfig(const VCSELConfig& config);

    /**
     * Set thermal callback for overheat notifications
     */
    using ThermalCallback = std::function<void(bool thermal_active, float temperature_c)>;
    void setThermalCallback(ThermalCallback callback);

    /**
     * Validate that VCSEL currents are within safe limits
     * @param current_ma Current to validate
     * @return true if safe
     */
    bool validateCurrent(uint16_t current_ma) const;

    /**
     * Get total ON time for thermal management
     * @return Total ON time in milliseconds
     */
    uint64_t getTotalOnTime() const;

    /**
     * Reset statistics (capture count, ON time, etc.)
     */
    void resetStatistics();

private:
    // AS1170 hardware controller
    std::shared_ptr<AS1170Controller> as1170_;

    // Camera system for synchronized capture
    std::shared_ptr<camera::CameraSystem> camera_system_;

    // Configuration
    VCSELConfig config_;

    // Status tracking
    mutable std::mutex status_mutex_;
    VCSELStatus status_;

    // State management
    std::atomic<bool> initialized_{false};
    std::atomic<bool> emergency_shutdown_{false};
    std::atomic<bool> vcsel1_active_{false};
    std::atomic<bool> vcsel2_active_{false};

    // Timing tracking
    std::chrono::steady_clock::time_point vcsel1_activation_time_;
    std::chrono::steady_clock::time_point vcsel2_activation_time_;
    std::atomic<uint64_t> total_on_time_ms_{0};
    std::atomic<uint64_t> total_captures_{0};

    // Thermal callback
    ThermalCallback thermal_callback_;

    // Safety limits
    static constexpr uint16_t MAX_SAFE_CURRENT_MA = 200;     // Conservative limit
    static constexpr uint16_t ABSOLUTE_MAX_CURRENT_MA = 446; // AS1170 hardware maximum: 446mA (127 * 3.515625)
    static constexpr uint32_t MAX_ON_TIME_MS = 10000;        // 10 second max continuous
    static constexpr float MAX_SAFE_TEMP_C = 70.0f;          // Maximum safe temperature

    // Internal methods

    /**
     * Capture synchronized stereo frame pair
     */
    bool captureStereoFrame(cv::Mat& left, cv::Mat& right, double& sync_error_ms);

    /**
     * Check thermal status and enforce safety limits
     * @return true if safe to continue, false if thermal shutdown needed
     */
    bool checkThermalSafety();

    /**
     * Update status structure (requires status_mutex_ to be locked)
     */
    void updateStatus();

    /**
     * Validate configuration parameters
     */
    bool validateConfig(const VCSELConfig& config) const;

    /**
     * Check maximum ON time limits
     * @return true if within safe limits
     */
    bool checkOnTimeLimits() const;

    /**
     * Wait for settle time between operations
     */
    void waitSettleTime();

    /**
     * Wait for capture delay after activation
     */
    void waitCaptureDelay();

    /**
     * Log error and update status
     */
    void setError(const std::string& error);

    /**
     * Clear error state
     */
    void clearError();
};

} // namespace hardware
} // namespace unlook
