#pragma once

#include <unlook/hardware/AS1170Controller.hpp>
#include <memory>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
#include <chrono>
#include <deque>

namespace unlook {
namespace hardware {

/**
 * LED Thermal Management System
 *
 * Provides comprehensive thermal protection and monitoring for AS1170 LED system.
 * Implements real-time temperature monitoring, thermal throttling, and emergency
 * protection to ensure safe operation of high-power VCSEL and flood LEDs.
 *
 * Key Features:
 * - Real-time temperature monitoring (<1°C accuracy)
 * - Predictive thermal throttling to prevent overheating
 * - Emergency thermal shutdown (<5ms response time)
 * - Thermal efficiency optimization
 * - Historical temperature logging and analysis
 * - Integration with AS1170 hardware thermal sensors
 *
 * Safety Guarantees:
 * - Prevents LED overheating under all operating conditions
 * - Automatic current limiting based on thermal state
 * - Failsafe emergency shutdown if thermal control fails
 * - Compliance with industrial safety standards
 */
class LEDThermalManager {
public:
    enum class ThermalState {
        COLD,              // Below operating temperature
        OPTIMAL,           // Optimal operating temperature
        WARM,              // Elevated but safe temperature
        HOT,               // High temperature - throttling active
        CRITICAL,          // Critical temperature - emergency action required
        EMERGENCY_SHUTDOWN // Emergency thermal shutdown active
    };

    enum class CoolingStrategy {
        PASSIVE,           // Natural cooling only
        CURRENT_LIMITING,  // Reduce LED current
        DUTY_CYCLE,        // Reduce duty cycle
        EMERGENCY_OFF,     // Turn off all LEDs
        ADAPTIVE          // Adaptive strategy based on conditions
    };

    struct ThermalConfig {
        // Temperature thresholds (°C)
        float optimal_temp_c;          // Optimal operating temperature
        float warm_threshold_c;        // Start monitoring more closely
        float hot_threshold_c;         // Begin thermal throttling
        float critical_threshold_c;    // Emergency action required
        float emergency_threshold_c;   // Immediate shutdown

        // Hysteresis for temperature transitions (°C)
        float temp_hysteresis_c;

        // Monitoring parameters
        uint32_t monitoring_interval_ms;  // Temperature check interval
        uint32_t thermal_history_size;    // 60 seconds at 100ms intervals

        // Throttling parameters
        float throttle_step_percent;    // Current reduction step (%)
        uint32_t throttle_delay_ms;      // Delay between throttle steps
        float min_current_percent;      // Minimum current (% of max)

        // Safety parameters
        uint32_t emergency_response_ms;     // Max emergency response time
        bool enable_predictive_throttling; // Predict thermal events
        float thermal_time_constant_s;  // Thermal time constant

        // Cooling optimization
        CoolingStrategy primary_strategy;
        CoolingStrategy emergency_strategy;
        bool enable_adaptive_cooling;

        // Default constructor
        ThermalConfig() :
            optimal_temp_c(25.0f),          // Optimal operating temperature
            warm_threshold_c(50.0f),        // Start monitoring more closely
            hot_threshold_c(65.0f),         // Begin thermal throttling
            critical_threshold_c(70.0f),    // Emergency action required
            emergency_threshold_c(75.0f),   // Immediate shutdown
            temp_hysteresis_c(2.0f),
            monitoring_interval_ms(100),    // Temperature check interval
            thermal_history_size(600),      // 60 seconds at 100ms intervals
            throttle_step_percent(10.0f),   // Current reduction step (%)
            throttle_delay_ms(1000),        // Delay between throttle steps
            min_current_percent(20.0f),     // Minimum current (% of max)
            emergency_response_ms(5),       // Max emergency response time
            enable_predictive_throttling(true), // Predict thermal events
            thermal_time_constant_s(10.0f), // Thermal time constant
            primary_strategy(CoolingStrategy::CURRENT_LIMITING),
            emergency_strategy(CoolingStrategy::EMERGENCY_OFF),
            enable_adaptive_cooling(true)
        {}
    };

    struct ThermalStatus {
        bool initialized = false;
        bool monitoring_active = false;
        ThermalState current_state = ThermalState::COLD;
        CoolingStrategy active_strategy = CoolingStrategy::PASSIVE;

        // Current measurements
        float current_temp_c = 0.0f;
        float max_temp_c = 0.0f;
        float avg_temp_c = 0.0f;
        std::chrono::steady_clock::time_point last_measurement;

        // Thermal protection status
        bool thermal_protection_active = false;
        uint16_t throttled_current_ma = 0;
        uint16_t original_current_ma = 0;
        float current_throttle_percent = 0.0f;

        // Predictive analysis
        float predicted_temp_c = 0.0f;
        std::chrono::seconds time_to_critical{0};
        bool thermal_event_predicted = false;

        // Statistics
        uint64_t measurement_cycles = 0;
        uint64_t throttling_events = 0;
        uint64_t emergency_events = 0;
        std::chrono::duration<double> total_throttling_time{0};

        std::string error_message;
    };

    struct ThermalMetrics {
        float thermal_efficiency = 0.0f;        // Thermal efficiency (0-1)
        float average_power_w = 0.0f;           // Average power consumption
        float peak_power_w = 0.0f;              // Peak power consumption
        float thermal_resistance_c_per_w = 0.0f; // Thermal resistance
        uint32_t cooling_time_constant_ms = 0;  // Measured cooling time
        uint32_t heating_time_constant_ms = 0;  // Measured heating time
    };

    // Callback types
    using ThermalCallback = std::function<void(ThermalState state, float temperature_c)>;
    using ThrottleCallback = std::function<void(uint16_t new_current_ma, float throttle_percent)>;
    using EmergencyCallback = std::function<void(const std::string& reason)>;

    LEDThermalManager();
    ~LEDThermalManager();

    /**
     * Initialize thermal management system
     * @param as1170_controller AS1170 controller for temperature monitoring
     * @param config Thermal management configuration
     * @return true if initialization successful
     */
    bool initialize(std::shared_ptr<AS1170Controller> as1170_controller,
                   const ThermalConfig& config = ThermalConfig{});

    /**
     * Shutdown thermal management system
     */
    void shutdown();

    /**
     * Start thermal monitoring
     * Begins continuous temperature monitoring and protection
     * @return true if monitoring started successfully
     */
    bool startMonitoring();

    /**
     * Stop thermal monitoring
     */
    void stopMonitoring();

    /**
     * Check thermal status and apply protection if needed
     * This method should be called before any LED activation
     * @param proposed_current_ma Proposed LED current
     * @param duration_ms Proposed activation duration
     * @return Approved current after thermal considerations
     */
    uint16_t checkThermalProtection(uint16_t proposed_current_ma, uint32_t duration_ms);

    /**
     * Notify thermal manager of LED activation
     * Call when LEDs are turned on to track power/thermal state
     * @param led1_current_ma LED1 current
     * @param led2_current_ma LED2 current
     */
    void notifyLEDActivation(uint16_t led1_current_ma, uint16_t led2_current_ma);

    /**
     * Notify thermal manager of LED deactivation
     * Call when LEDs are turned off
     */
    void notifyLEDDeactivation();

    /**
     * Force thermal protection activation (for testing)
     * @param activate Enable or disable forced protection
     * @param throttle_percent Throttling percentage (0-100)
     * @return true if successful
     */
    bool forceThermalProtection(bool activate, float throttle_percent = 50.0f);

    /**
     * Emergency thermal shutdown
     * Immediate thermal protection activation
     * Thread-safe, can be called from any context
     */
    void emergencyThermalShutdown();

    /**
     * Predict thermal event
     * Uses thermal modeling to predict overheating
     * @param led_current_ma Proposed LED current
     * @param duration_ms Proposed duration
     * @return Predicted maximum temperature
     */
    float predictThermalEvent(uint16_t led_current_ma, uint32_t duration_ms);

    /**
     * Get current thermal status
     */
    ThermalStatus getStatus() const;

    /**
     * Get thermal performance metrics
     */
    ThermalMetrics getMetrics() const;

    /**
     * Get temperature history for analysis
     * @param duration_seconds Duration of history to retrieve
     * @return Vector of temperature readings with timestamps
     */
    struct TempReading {
        std::chrono::steady_clock::time_point timestamp;
        float temperature_c;
        uint16_t led_power_ma;
    };
    std::vector<TempReading> getTemperatureHistory(uint32_t duration_seconds = 60) const;

    /**
     * Configure thermal management parameters
     * @param config New configuration to apply
     * @return true if configuration valid and applied
     */
    bool configure(const ThermalConfig& config);

    /**
     * Get current configuration
     */
    ThermalConfig getConfig() const;

    /**
     * Register callbacks for thermal events
     */
    void setThermalCallback(ThermalCallback callback);
    void setThrottleCallback(ThrottleCallback callback);
    void setEmergencyCallback(EmergencyCallback callback);

    /**
     * Check if thermal system is ready
     */
    bool isReady() const;

    /**
     * Check if thermal protection is currently active
     */
    bool isThermalProtectionActive() const;

    /**
     * Calibrate thermal system
     * Runs calibration sequence to determine thermal characteristics
     * @return true if calibration successful
     */
    bool calibrateThermalSystem();

    /**
     * Optimize thermal performance
     * Adjusts parameters for optimal thermal/performance balance
     * @return true if optimization successful
     */
    bool optimizeThermalPerformance();

private:
    std::shared_ptr<AS1170Controller> as1170_controller_;
    ThermalConfig config_;
    mutable std::mutex mutex_;
    mutable std::mutex status_mutex_;

    std::atomic<bool> initialized_{false};
    std::atomic<bool> monitoring_active_{false};
    std::atomic<bool> emergency_shutdown_{false};

    // Monitoring thread
    std::unique_ptr<std::thread> monitoring_thread_;
    std::atomic<bool> stop_monitoring_{false};

    // Status and metrics
    ThermalStatus status_;
    ThermalMetrics metrics_;

    // Temperature history
    mutable std::mutex history_mutex_;
    std::deque<TempReading> temperature_history_;

    // Power tracking
    uint16_t current_led1_ma_ = 0;
    uint16_t current_led2_ma_ = 0;
    std::chrono::steady_clock::time_point led_activation_time_;
    std::chrono::duration<double> cumulative_on_time_{0};

    // Thermal modeling
    struct ThermalModel {
        float thermal_mass = 1.0f;              // Thermal mass coefficient
        float thermal_resistance = 1.0f;        // Thermal resistance (°C/W)
        float ambient_temp_c = 25.0f;           // Ambient temperature
        float heating_coefficient = 1.0f;       // Heating rate coefficient
        float cooling_coefficient = 1.0f;       // Cooling rate coefficient
    };
    ThermalModel thermal_model_;

    // Callbacks
    ThermalCallback thermal_callback_;
    ThrottleCallback throttle_callback_;
    EmergencyCallback emergency_callback_;

    // Performance tracking
    std::chrono::steady_clock::time_point performance_window_start_;
    std::chrono::duration<double> power_on_time_{0};
    float cumulative_power_w_ = 0.0f;
    uint32_t power_samples_ = 0;

    // Private methods
    void monitoringThreadWorker();
    bool readTemperature(float& temperature_c);
    void updateThermalState(float temperature_c);
    void handleThermalStateChange(ThermalState new_state);

    // Thermal protection methods
    bool activateThermalProtection(ThermalState state);
    bool deactivateThermalProtection();
    uint16_t calculateThrottledCurrent(uint16_t original_current, ThermalState state);
    CoolingStrategy selectCoolingStrategy(ThermalState state);

    // Thermal modeling methods
    void updateThermalModel(float measured_temp, uint16_t power_ma);
    float predictTemperature(float current_temp, uint16_t power_ma, uint32_t time_ms);
    void calibrateThermalModel();

    // Statistics and metrics
    void updateMetrics();
    void updateTemperatureHistory(float temperature_c);
    float calculateThermalEfficiency() const;
    float calculateCurrentPower() const;

    // Safety and validation
    bool validateConfig(const ThermalConfig& config) const;
    void handleThermalError(const std::string& error);
    void updateStatus();

    // Utility methods
    ThermalState temperatureToState(float temperature_c) const;
    bool isStateTransitionValid(ThermalState from, ThermalState to) const;
    std::string thermalStateToString(ThermalState state) const;
};

} // namespace hardware
} // namespace unlook