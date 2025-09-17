#include <unlook/hardware/LEDThermalManager.hpp>
#include <unlook/core/Logger.hpp>

#include <algorithm>
#include <numeric>
#include <cmath>

namespace unlook {
namespace hardware {

LEDThermalManager::LEDThermalManager() {
    // Initialize thermal model with reasonable defaults
    thermal_model_.ambient_temp_c = 25.0f;
    thermal_model_.thermal_mass = 1.0f;
    thermal_model_.thermal_resistance = 30.0f;  // °C/W (typical for small LED packages)
    thermal_model_.heating_coefficient = 1.0f;
    thermal_model_.cooling_coefficient = 0.8f;

    performance_window_start_ = std::chrono::steady_clock::now();
}

LEDThermalManager::~LEDThermalManager() {
    //     shutdown();
}

bool LEDThermalManager::initialize(std::shared_ptr<AS1170Controller> as1170_controller,
                                  const ThermalConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("LEDThermalManager already initialized");
        return true;
    }

    if (!as1170_controller || !as1170_controller->isInitialized()) {
        handleThermalError("AS1170Controller not provided or not initialized");
        return false;
    }

    if (!validateConfig(config)) {
        handleThermalError("Invalid thermal configuration");
        return false;
    }

    as1170_controller_ = as1170_controller;
    config_ = config;

    // Initialize status
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = true;
        status_.current_state = ThermalState::COLD;
        status_.active_strategy = CoolingStrategy::PASSIVE;
        status_.last_measurement = std::chrono::steady_clock::now();
    //         status_.error_message.clear();
    }

    // Initialize thermal model based on ambient conditions
    float initial_temp = as1170_controller_->readTemperature();
    if (initial_temp > -999.0f) {
        thermal_model_.ambient_temp_c = initial_temp;
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_temp_c = initial_temp;
        status_.avg_temp_c = initial_temp;
        status_.max_temp_c = initial_temp;
    }

    //     initialized_.store(true);
    //     emergency_shutdown_.store(false);
    //     stop_monitoring_.store(false);

    core::Logger::getInstance().info("LEDThermalManager initialized successfully");
    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Initial temperature: {:.1f}°C, Ambient: {:.1f}°C",
    //                       initial_temp, thermal_model_.ambient_temp_c);
    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Thermal thresholds: Warm={:.1f}°C, Hot={:.1f}°C, Critical={:.1f}°C",
    //                       config_.warm_threshold_c, config_.hot_threshold_c, config_.critical_threshold_c);

    return true;
}

void LEDThermalManager::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
    //         return;
    }

    core::Logger::getInstance().info("Shutting down LEDThermalManager...");

    //     stopMonitoring();

    // Clear callbacks
    thermal_callback_ = nullptr;
    throttle_callback_ = nullptr;
    emergency_callback_ = nullptr;

    //     as1170_controller_.reset();
    //     initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.monitoring_active = false;
        status_.thermal_protection_active = false;
    }

    core::Logger::getInstance().info("LEDThermalManager shutdown complete");
}

bool LEDThermalManager::startMonitoring() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        handleThermalError("LEDThermalManager not initialized");
        return false;
    }

    if (monitoring_active_.load()) {
        core::Logger::getInstance().warning("Thermal monitoring already active");
        return true;
    }

    //     stop_monitoring_.store(false);
    monitoring_thread_ = std::make_unique<std::thread>(&LEDThermalManager::monitoringThreadWorker, this);

    //     monitoring_active_.store(true);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.monitoring_active = true;
        status_.measurement_cycles = 0;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Thermal monitoring started (interval: {}ms)", config_.monitoring_interval_ms);
    return true;
}

void LEDThermalManager::stopMonitoring() {
    if (!monitoring_active_.load()) {
    //         return;
    }

    //     stop_monitoring_.store(true);

    if (monitoring_thread_ && monitoring_thread_->joinable()) {
        monitoring_thread_->join();
    //         monitoring_thread_.reset();
    }

    //     monitoring_active_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.monitoring_active = false;
    }

    core::Logger::getInstance().info("Thermal monitoring stopped");
}

uint16_t LEDThermalManager::checkThermalProtection(uint16_t proposed_current_ma, uint32_t duration_ms) {
    if (!initialized_.load() || emergency_shutdown_.load()) {
        return 0; // Emergency state - no current allowed
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Read current temperature
    float current_temp = 0.0f;
    if (!readTemperature(current_temp)) {
        handleThermalError("Failed to read temperature for protection check");
        return proposed_current_ma / 2; // Conservative fallback
    }

    // Predict temperature with proposed current
    float predicted_temp = predictTemperature(current_temp, proposed_current_ma, duration_ms);

    // Check if thermal protection is needed
    ThermalState predicted_state = temperatureToState(predicted_temp);

    uint16_t approved_current = proposed_current_ma;

    if (predicted_state >= ThermalState::HOT) {
        // Calculate throttled current
        approved_current = calculateThrottledCurrent(proposed_current_ma, predicted_state);

        if (!status_.thermal_protection_active) {
    //             activateThermalProtection(predicted_state);
        }

    // TODO: Fix formatted logging -         core::Logger::getInstance().debug("Thermal protection: {}mA -> {}mA (predicted: {:.1f}°C)",
    //                            proposed_current_ma, approved_current, predicted_temp);
    }

    return approved_current;
}

void LEDThermalManager::notifyLEDActivation(uint16_t led1_current_ma, uint16_t led2_current_ma) {
    std::lock_guard<std::mutex> lock(mutex_);

    current_led1_ma_ = led1_current_ma;
    current_led2_ma_ = led2_current_ma;
    led_activation_time_ = std::chrono::steady_clock::now();

    // Update power tracking
    power_on_time_ = std::chrono::duration<double>(0);

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("LED activation: LED1={}mA, LED2={}mA", led1_current_ma, led2_current_ma);
}

void LEDThermalManager::notifyLEDDeactivation() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (current_led1_ma_ > 0 || current_led2_ma_ > 0) {
        auto deactivation_time = std::chrono::steady_clock::now();
        auto on_duration = deactivation_time - led_activation_time_;
        cumulative_on_time_ += on_duration;

        // Update power metrics
        float current_power = calculateCurrentPower();
        cumulative_power_w_ += current_power * std::chrono::duration<float>(on_duration).count();
        power_samples_++;
    }

    current_led1_ma_ = 0;
    current_led2_ma_ = 0;

    core::Logger::getInstance().debug("LED deactivation");
}

bool LEDThermalManager::forceThermalProtection(bool activate, float throttle_percent) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return false;
    }

    if (activate) {
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.thermal_protection_active = true;
            status_.current_throttle_percent = throttle_percent;
            status_.throttled_current_ma = static_cast<uint16_t>(
                status_.original_current_ma * (100.0f - throttle_percent) / 100.0f);
        }

        if (throttle_callback_) {
            throttle_callback_(status_.throttled_current_ma, throttle_percent);
        }

    // TODO: Fix formatted logging -         core::Logger::getInstance().warning("Forced thermal protection activated: {}% throttling", throttle_percent);
    } else {
    //         deactivateThermalProtection();
        core::Logger::getInstance().info("Forced thermal protection deactivated");
    }

    return true;
}

void LEDThermalManager::emergencyThermalShutdown() {
    //     emergency_shutdown_.store(true);

    if (as1170_controller_) {
        as1170_controller_->emergencyShutdown();
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_state = ThermalState::EMERGENCY_SHUTDOWN;
        status_.thermal_protection_active = true;
        status_.throttled_current_ma = 0;
        status_.emergency_events++;
    }

    if (emergency_callback_) {
        emergency_callback_("Emergency thermal shutdown activated");
    }

    core::Logger::getInstance().error("Emergency thermal shutdown activated!");
}

float LEDThermalManager::predictThermalEvent(uint16_t led_current_ma, uint32_t duration_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    float current_temp = status_.current_temp_c;
    if (current_temp <= 0) {
        // No valid temperature reading
        current_temp = thermal_model_.ambient_temp_c + 10.0f; // Conservative estimate
    }

    return predictTemperature(current_temp, led_current_ma, duration_ms);
}

LEDThermalManager::ThermalStatus LEDThermalManager::getStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto status = status_;
    status.last_measurement = std::chrono::steady_clock::now();
    return status;
}

LEDThermalManager::ThermalMetrics LEDThermalManager::getMetrics() const {
    std::lock_guard<std::mutex> lock(mutex_);

    ThermalMetrics metrics = metrics_;

    // Update real-time calculations
    if (power_samples_ > 0) {
        metrics.average_power_w = cumulative_power_w_ / power_samples_;
    }

    metrics.thermal_efficiency = calculateThermalEfficiency();

    return metrics;
}

std::vector<LEDThermalManager::TempReading> LEDThermalManager::getTemperatureHistory(uint32_t duration_seconds) const {
    std::lock_guard<std::mutex> history_lock(history_mutex_);

    std::vector<TempReading> history;
    auto cutoff_time = std::chrono::steady_clock::now() - std::chrono::seconds(duration_seconds);

    for (const auto& reading : temperature_history_) {
        if (reading.timestamp >= cutoff_time) {
    //             history.push_back(reading);
        }
    }

    return history;
}

bool LEDThermalManager::configure(const ThermalConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!validateConfig(config)) {
        handleThermalError("Invalid thermal configuration");
        return false;
    }

    bool was_monitoring = monitoring_active_.load();

    if (was_monitoring) {
    //         stopMonitoring();
    }

    config_ = config;

    if (was_monitoring) {
        return startMonitoring();
    }

    core::Logger::getInstance().info("Thermal configuration updated");
    return true;
}

LEDThermalManager::ThermalConfig LEDThermalManager::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void LEDThermalManager::setThermalCallback(ThermalCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    thermal_callback_ = callback;
}

void LEDThermalManager::setThrottleCallback(ThrottleCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    throttle_callback_ = callback;
}

void LEDThermalManager::setEmergencyCallback(EmergencyCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    emergency_callback_ = callback;
}

bool LEDThermalManager::isReady() const {
    return initialized_.load() && !emergency_shutdown_.load();
}

bool LEDThermalManager::isThermalProtectionActive() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return status_.thermal_protection_active;
}

bool LEDThermalManager::calibrateThermalSystem() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().info("Starting thermal system calibration...");

    // Run calibration sequence
    //     calibrateThermalModel();

    // Update metrics
    //     updateMetrics();

    core::Logger::getInstance().info("Thermal system calibration completed");
    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Thermal resistance: {:.2f} °C/W", thermal_model_.thermal_resistance);

    return true;
}

bool LEDThermalManager::optimizeThermalPerformance() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().info("Optimizing thermal performance...");

    // Analyze temperature history to optimize parameters
    auto history = getTemperatureHistory(300); // Last 5 minutes

    if (history.size() < 10) {
        core::Logger::getInstance().warning("Insufficient thermal history for optimization");
        return false;
    }

    // Calculate optimal throttling parameters based on history
    float max_observed_temp = 0.0f;
    float avg_power = 0.0f;

    for (const auto& reading : history) {
        max_observed_temp = std::max(max_observed_temp, reading.temperature_c);
        avg_power += reading.led_power_ma;
    }

    avg_power /= history.size();

    // Adjust thermal model based on observations
    if (max_observed_temp > config_.warm_threshold_c) {
        thermal_model_.thermal_resistance *= 1.1f; // More conservative
    }

    //     updateMetrics();

    core::Logger::getInstance().info("Thermal performance optimization completed");
    return true;
}

// Private implementation methods

void LEDThermalManager::monitoringThreadWorker() {
    core::Logger::getInstance().info("Thermal monitoring thread started");

    while (!stop_monitoring_.load() && initialized_.load()) {
        try {
            float temperature_c = 0.0f;

            if (readTemperature(temperature_c)) {
    //                 updateThermalState(temperature_c);
    //                 updateTemperatureHistory(temperature_c);
    //                 updateMetrics();

                {
                    std::lock_guard<std::mutex> status_lock(status_mutex_);
                    status_.measurement_cycles++;
                }
            } else {
                handleThermalError("Failed to read temperature during monitoring");
            }

    //             std::this_thread::sleep_for(std::chrono::milliseconds(config_.monitoring_interval_ms));

        } catch (const std::exception& e) {
            handleThermalError("Monitoring thread exception: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Back off on error
        }
    }

    core::Logger::getInstance().info("Thermal monitoring thread stopped");
}

bool LEDThermalManager::readTemperature(float& temperature_c) {
    if (!as1170_controller_) {
        return false;
    }

    temperature_c = as1170_controller_->readTemperature();
    return temperature_c > -999.0f;
}

void LEDThermalManager::updateThermalState(float temperature_c) {
    ThermalState new_state = temperatureToState(temperature_c);
    ThermalState current_state;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        current_state = status_.current_state;

        // Update temperature statistics
        status_.current_temp_c = temperature_c;
        status_.max_temp_c = std::max(status_.max_temp_c, temperature_c);

        // Update running average
        if (status_.measurement_cycles > 0) {
            status_.avg_temp_c = ((status_.avg_temp_c * status_.measurement_cycles) + temperature_c) /
                                (status_.measurement_cycles + 1);
        } else {
            status_.avg_temp_c = temperature_c;
        }
    }

    // Apply hysteresis to prevent oscillation
    if (new_state != current_state) {
        float hysteresis = config_.temp_hysteresis_c;

        // Only transition if temperature change is significant
        if (new_state > current_state) {
            // Heating - immediate transition
    //             handleThermalStateChange(new_state);
        } else if (new_state < current_state) {
            // Cooling - apply hysteresis
            float threshold = 0.0f;
            switch (current_state) {
                case ThermalState::HOT:
                    threshold = config_.hot_threshold_c - hysteresis;
    //                     break;
                case ThermalState::WARM:
                    threshold = config_.warm_threshold_c - hysteresis;
    //                     break;
                default:
                    threshold = 0.0f;
    //                     break;
            }

            if (temperature_c < threshold || threshold == 0.0f) {
    //                 handleThermalStateChange(new_state);
            }
        }
    }
}

void LEDThermalManager::handleThermalStateChange(ThermalState new_state) {
    ThermalState old_state;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        old_state = status_.current_state;
        status_.current_state = new_state;
    }

    if (new_state != old_state) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().info("Thermal state change: {} -> {} (temp: {:.1f}°C)",
    //                       thermalStateToString(old_state),
    //                       thermalStateToString(new_state),
    //                       status_.current_temp_c);

        // Handle state-specific actions
        if (new_state >= ThermalState::HOT && !status_.thermal_protection_active) {
    //             activateThermalProtection(new_state);
        } else if (new_state < ThermalState::HOT && status_.thermal_protection_active) {
    //             deactivateThermalProtection();
        }

        if (new_state >= ThermalState::CRITICAL) {
    //             emergencyThermalShutdown();
        }

        // Call thermal callback
        if (thermal_callback_) {
            thermal_callback_(new_state, status_.current_temp_c);
        }
    }
}

bool LEDThermalManager::activateThermalProtection(ThermalState state) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.thermal_protection_active = true;
        status_.active_strategy = selectCoolingStrategy(state);
        status_.throttling_events++;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().warning("Thermal protection activated: state={}, strategy={}",
    //                   thermalStateToString(state), static_cast<int>(status_.active_strategy));

    return true;
}

bool LEDThermalManager::deactivateThermalProtection() {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.thermal_protection_active = false;
        status_.active_strategy = CoolingStrategy::PASSIVE;
        status_.current_throttle_percent = 0.0f;
        status_.throttled_current_ma = status_.original_current_ma;
    }

    core::Logger::getInstance().info("Thermal protection deactivated");

    return true;
}

uint16_t LEDThermalManager::calculateThrottledCurrent(uint16_t original_current, ThermalState state) {
    float throttle_percent = 0.0f;

    switch (state) {
        case ThermalState::HOT:
            throttle_percent = config_.throttle_step_percent;
    //             break;
        case ThermalState::CRITICAL:
            throttle_percent = 50.0f; // 50% reduction
    //             break;
        case ThermalState::EMERGENCY_SHUTDOWN:
            return 0; // Complete shutdown
        default:
            return original_current;
    }

    float multiplier = (100.0f - throttle_percent) / 100.0f;
    uint16_t throttled = static_cast<uint16_t>(original_current * multiplier);

    // Apply minimum current limit
    uint16_t min_current = static_cast<uint16_t>(original_current * config_.min_current_percent / 100.0f);
    throttled = std::max(throttled, min_current);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.original_current_ma = original_current;
        status_.throttled_current_ma = throttled;
        status_.current_throttle_percent = throttle_percent;
    }

    return throttled;
}

LEDThermalManager::CoolingStrategy LEDThermalManager::selectCoolingStrategy(ThermalState state) {
    if (state >= ThermalState::CRITICAL) {
        return config_.emergency_strategy;
    }

    return config_.primary_strategy;
}

void LEDThermalManager::updateThermalModel(float measured_temp, uint16_t power_ma) {
    // Simple thermal model update based on measurements
    float power_w = calculateCurrentPower();
    float temp_rise = measured_temp - thermal_model_.ambient_temp_c;

    if (power_w > 0.1f && temp_rise > 1.0f) {
        // Update thermal resistance estimate
        float new_resistance = temp_rise / power_w;
        thermal_model_.thermal_resistance =
            0.9f * thermal_model_.thermal_resistance + 0.1f * new_resistance;
    }
}

float LEDThermalManager::predictTemperature(float current_temp, uint16_t power_ma, uint32_t time_ms) {
    float power_w = (power_ma * 3.3f) / 1000.0f; // Rough power estimate
    float steady_state_rise = power_w * thermal_model_.thermal_resistance;
    float time_constant_ms = config_.thermal_time_constant_s * 1000.0f;

    // Exponential approach to steady state
    float time_ratio = static_cast<float>(time_ms) / time_constant_ms;
    float temp_rise = steady_state_rise * (1.0f - std::exp(-time_ratio));

    return thermal_model_.ambient_temp_c + temp_rise;
}

void LEDThermalManager::calibrateThermalModel() {
    // Thermal model calibration would require controlled heating/cooling cycles
    // For now, use reasonable defaults based on typical LED packages
    core::Logger::getInstance().info("Using default thermal model parameters");
}

void LEDThermalManager::updateMetrics() {
    std::lock_guard<std::mutex> lock(mutex_);

    metrics_.thermal_efficiency = calculateThermalEfficiency();
    metrics_.thermal_resistance_c_per_w = thermal_model_.thermal_resistance;

    // Update power metrics
    if (power_samples_ > 0) {
        metrics_.average_power_w = cumulative_power_w_ / power_samples_;
    }
}

void LEDThermalManager::updateTemperatureHistory(float temperature_c) {
    std::lock_guard<std::mutex> history_lock(history_mutex_);

    TempReading reading;
    reading.timestamp = std::chrono::steady_clock::now();
    reading.temperature_c = temperature_c;
    reading.led_power_ma = current_led1_ma_ + current_led2_ma_;

    //     temperature_history_.push_back(reading);

    // Limit history size
    while (temperature_history_.size() > config_.thermal_history_size) {
    //         temperature_history_.pop_front();
    }
}

float LEDThermalManager::calculateThermalEfficiency() const {
    if (status_.max_temp_c <= thermal_model_.ambient_temp_c) {
        return 1.0f; // Perfect efficiency (no heating)
    }

    float temp_rise = status_.max_temp_c - thermal_model_.ambient_temp_c;
    float max_safe_rise = config_.hot_threshold_c - thermal_model_.ambient_temp_c;

    return std::max(0.0f, 1.0f - (temp_rise / max_safe_rise));
}

float LEDThermalManager::calculateCurrentPower() const {
    // Approximate power calculation: P = I * V (assuming ~3.3V forward voltage)
    float total_current_a = (current_led1_ma_ + current_led2_ma_) / 1000.0f;
    return total_current_a * 3.3f; // Watts
}

bool LEDThermalManager::validateConfig(const ThermalConfig& config) const {
    if (config.optimal_temp_c >= config.warm_threshold_c) return false;
    if (config.warm_threshold_c >= config.hot_threshold_c) return false;
    if (config.hot_threshold_c >= config.critical_threshold_c) return false;
    if (config.critical_threshold_c >= config.emergency_threshold_c) return false;

    if (config.temp_hysteresis_c <= 0 || config.temp_hysteresis_c > 10.0f) return false;
    if (config.monitoring_interval_ms < 10 || config.monitoring_interval_ms > 10000) return false;
    if (config.throttle_step_percent <= 0 || config.throttle_step_percent > 50.0f) return false;
    if (config.min_current_percent <= 0 || config.min_current_percent > 100.0f) return false;

    return true;
}

void LEDThermalManager::handleThermalError(const std::string& error) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.error_message = error;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().error("LEDThermalManager: {}", error);
}

void LEDThermalManager::updateStatus() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_measurement = std::chrono::steady_clock::now();
}

LEDThermalManager::ThermalState LEDThermalManager::temperatureToState(float temperature_c) const {
    if (temperature_c >= config_.emergency_threshold_c) {
        return ThermalState::EMERGENCY_SHUTDOWN;
    } else if (temperature_c >= config_.critical_threshold_c) {
        return ThermalState::CRITICAL;
    } else if (temperature_c >= config_.hot_threshold_c) {
        return ThermalState::HOT;
    } else if (temperature_c >= config_.warm_threshold_c) {
        return ThermalState::WARM;
    } else if (temperature_c >= config_.optimal_temp_c) {
        return ThermalState::OPTIMAL;
    } else {
        return ThermalState::COLD;
    }
}

bool LEDThermalManager::isStateTransitionValid(ThermalState from, ThermalState to) const {
    // All transitions are valid for safety
    return true;
}

std::string LEDThermalManager::thermalStateToString(ThermalState state) const {
    switch (state) {
        case ThermalState::COLD: return "COLD";
        case ThermalState::OPTIMAL: return "OPTIMAL";
        case ThermalState::WARM: return "WARM";
        case ThermalState::HOT: return "HOT";
        case ThermalState::CRITICAL: return "CRITICAL";
        case ThermalState::EMERGENCY_SHUTDOWN: return "EMERGENCY_SHUTDOWN";
        default: return "UNKNOWN";
    }
}

} // namespace hardware
} // namespace unlook