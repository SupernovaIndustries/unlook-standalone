#pragma once

#include <unlook/hardware/BMI270Driver.hpp>
#include <memory>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <deque>

namespace unlook {
namespace hardware {

/**
 * Stability Detector for Handheld 3D Scanner
 *
 * Analyzes IMU data from BMI270 to detect when the scanner is held stable
 * enough for high-quality multi-frame scanning. Provides real-time stability
 * scoring for GUI feedback.
 *
 * Stability Criteria:
 * - Gyro threshold: |gyro_x|, |gyro_y|, |gyro_z| < 0.5 deg/sec (all axes)
 * - Accel variance: < 0.1 m/s² (movement detection)
 * - Stable duration: ≥ 500ms continuous stability before scan trigger
 *
 * Features:
 * - Real-time stability detection (<10ms latency)
 * - Smooth stability scoring (0.0-1.0) for GUI feedback
 * - Configurable thresholds and history window
 * - Robust to temporary disturbances
 * - Thread-safe operation
 *
 * Performance:
 * - Target: <500ms to achieve stable state from movement
 * - Update rate: 30-100 Hz for smooth GUI feedback
 * - History window: 1 second @ 100 Hz = 100 samples
 */
class StabilityDetector {
public:
    /**
     * Stability detection parameters
     */
    struct StabilityParams {
        float gyro_threshold_dps;      // Gyro threshold in deg/sec (all axes)
        float accel_variance_threshold; // Accel variance threshold in m/s²
        int stable_duration_ms;         // Required stable duration in milliseconds
        int history_window_ms;          // History window for analysis in milliseconds
        float gyro_weight;              // Gyro contribution to stability score (0.0-1.0)
        float accel_weight;             // Accel contribution to stability score (0.0-1.0)

        StabilityParams() :
            gyro_threshold_dps(0.5f),
            accel_variance_threshold(0.1f),
            stable_duration_ms(500),
            history_window_ms(1000),
            gyro_weight(0.7f),
            accel_weight(0.3f) {}
    };

    /**
     * Stability status information
     */
    struct StabilityStatus {
        bool is_stable;                             // Is currently stable
        float stability_score;                      // Stability score (0.0-1.0)
        int stable_duration_ms;                     // Duration of current stable state
        float current_gyro_magnitude;               // Current gyro magnitude (deg/sec)
        float current_accel_variance;               // Current accel variance (m/s²)
        int samples_in_history;                     // Number of samples in history
        std::chrono::steady_clock::time_point timestamp;

        StabilityStatus() :
            is_stable(false),
            stability_score(0.0f),
            stable_duration_ms(0),
            current_gyro_magnitude(0.0f),
            current_accel_variance(0.0f),
            samples_in_history(0),
            timestamp(std::chrono::steady_clock::now()) {}
    };

    /**
     * Constructor
     * @param imu_driver Shared pointer to BMI270Driver instance
     */
    explicit StabilityDetector(std::shared_ptr<BMI270Driver> imu_driver);

    /**
     * Destructor
     */
    ~StabilityDetector();

    /**
     * Initialize stability detector with parameters
     * @param params Stability detection parameters
     * @return true if initialization successful
     */
    bool initialize(const StabilityParams& params = StabilityParams());

    /**
     * Shutdown stability detector
     */
    void shutdown();

    /**
     * Update stability analysis with new IMU data
     * Call this at regular intervals (30-100 Hz recommended)
     * @return true if update successful
     */
    bool update();

    /**
     * Check if scanner is currently stable
     * @return true if stable according to criteria
     */
    bool isStable() const;

    /**
     * Get current stability score (0.0 = very unstable, 1.0 = perfectly stable)
     * This value updates smoothly for GUI feedback
     * @return Stability score in range [0.0, 1.0]
     */
    float getStabilityScore() const;

    /**
     * Get duration of current stable state
     * @return Duration in milliseconds, 0 if not stable
     */
    int getStableDuration() const;

    /**
     * Get detailed stability status
     * @return StabilityStatus structure with all metrics
     */
    StabilityStatus getStatus() const;

    /**
     * Reset stability state (clears history)
     */
    void reset();

    /**
     * Set stability parameters
     * @param params New stability parameters
     */
    void setParameters(const StabilityParams& params);

    /**
     * Get current parameters
     * @return Current stability parameters
     */
    StabilityParams getParameters() const;

    /**
     * Check if detector is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_.load(); }

private:
    /**
     * Internal IMU data with timestamp for history
     */
    struct TimestampedIMUData {
        BMI270Driver::IMUData data;
        std::chrono::steady_clock::time_point timestamp;
    };

    // Analysis methods
    float calculateGyroMagnitude(const BMI270Driver::IMUData& data) const;
    float calculateAccelVariance() const;
    float calculateGyroScore() const;
    float calculateAccelScore() const;
    float calculateOverallScore() const;
    bool checkStabilityCriteria() const;
    void updateStableState();
    void pruneHistory();

    // Member variables
    std::shared_ptr<BMI270Driver> imu_driver_;
    StabilityParams params_;
    StabilityStatus status_;
    std::atomic<bool> initialized_;
    mutable std::mutex mutex_;

    // IMU data history (ring buffer)
    std::deque<TimestampedIMUData> history_;

    // Stability tracking
    std::chrono::steady_clock::time_point stable_since_;
    bool was_stable_;

    // Cached calculations
    float last_gyro_magnitude_;
    float last_accel_variance_;
    float last_stability_score_;
};

} // namespace hardware
} // namespace unlook
