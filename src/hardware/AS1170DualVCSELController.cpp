#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/core/Logger.hpp>

#include <thread>
#include <sstream>
#include <iomanip>

namespace unlook {
namespace hardware {

AS1170DualVCSELController::AS1170DualVCSELController() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.last_activation = std::chrono::steady_clock::now();
}

AS1170DualVCSELController::~AS1170DualVCSELController() {
    if (initialized_.load()) {
        shutdown();
    }
}

std::shared_ptr<AS1170DualVCSELController> AS1170DualVCSELController::getInstance() {
    static std::shared_ptr<AS1170DualVCSELController> instance = nullptr;
    static std::mutex instance_mutex;

    std::lock_guard<std::mutex> lock(instance_mutex);
    if (!instance) {
        instance = std::shared_ptr<AS1170DualVCSELController>(new AS1170DualVCSELController());
    }
    return instance;
}

bool AS1170DualVCSELController::initialize(std::shared_ptr<camera::CameraSystem> camera_system) {
    VCSELConfig default_config;
    return initialize(camera_system, default_config);
}

bool AS1170DualVCSELController::initialize(
    std::shared_ptr<camera::CameraSystem> camera_system,
    const VCSELConfig& config
) {
    if (initialized_.load()) {
        core::Logger::getInstance().warning("AS1170DualVCSELController already initialized");
        return true;
    }

    if (!camera_system) {
        setError("Camera system is null");
        return false;
    }

    if (!validateConfig(config)) {
        setError("Invalid configuration");
        return false;
    }

    camera_system_ = camera_system;
    config_ = config;

    core::Logger::getInstance().info("Initializing AS1170 Dual VCSEL Controller");
    core::Logger::getInstance().info("VCSEL1 (LED1): " + std::to_string(config_.vcsel1_current_ma) + "mA");
    core::Logger::getInstance().info("VCSEL2 (LED2): " + std::to_string(config_.vcsel2_current_ma) + "mA");
    core::Logger::getInstance().info("Settle time: " + std::to_string(config_.settle_time_ms) + "ms");

    // Get AS1170 controller instance
    as1170_ = AS1170Controller::getInstance();
    if (!as1170_) {
        setError("Failed to get AS1170 controller instance");
        return false;
    }

    // Initialize AS1170 with custom configuration for dual VCSEL
    AS1170Controller::AS1170Config as1170_config;
    as1170_config.i2c_bus = 1;
    as1170_config.i2c_address = 0x30;
    as1170_config.strobe_gpio = 19;
    as1170_config.target_current_ma = std::max(config_.vcsel1_current_ma, config_.vcsel2_current_ma);
    as1170_config.flash_mode = config_.use_strobe_mode ?
        AS1170Controller::FlashMode::FLASH_MODE :
        AS1170Controller::FlashMode::TORCH_MODE;
    as1170_config.enable_thermal_protection = config_.enable_thermal_monitoring;
    as1170_config.max_temperature_c = config_.max_operating_temp_c;

    if (!as1170_->initialize(as1170_config)) {
        setError("Failed to initialize AS1170 controller");
        return false;
    }

    // Ensure both VCSELs are initially OFF
    if (!deactivateBoth()) {
        setError("Failed to deactivate VCSELs during initialization");
        return false;
    }

    // Reset statistics
    resetStatistics();

    initialized_.store(true);
    clearError();

    core::Logger::getInstance().info("AS1170 Dual VCSEL Controller initialized successfully");
    core::Logger::getInstance().info("Temporal matching mode: Triple frame capture");
    core::Logger::getInstance().info("Safety limits: " + std::to_string(MAX_SAFE_CURRENT_MA) +
        "mA, " + std::to_string(MAX_SAFE_TEMP_C) + "C");

    return true;
}

void AS1170DualVCSELController::shutdown() {
    if (!initialized_.load()) {
        return;
    }

    core::Logger::getInstance().info("Shutting down AS1170 Dual VCSEL Controller");

    // Emergency shutdown both VCSELs
    emergencyShutdown();

    initialized_.store(false);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel1_active = false;
        status_.vcsel2_active = false;
        status_.vcsel1_current_ma = 0;
        status_.vcsel2_current_ma = 0;
    }

    core::Logger::getInstance().info("AS1170 Dual VCSEL Controller shutdown complete");
}

bool AS1170DualVCSELController::captureTemporalSequence(TripleFrameCapture& result) {
    if (!initialized_.load() || emergency_shutdown_.load()) {
        setError("System not initialized or in emergency shutdown");
        return false;
    }

    if (!camera_system_->isInitialized()) {
        setError("Camera system not initialized");
        return false;
    }

    auto sequence_start = std::chrono::steady_clock::now();

    core::Logger::getInstance().info("Starting temporal capture sequence (3 frames)");

    // Ensure both VCSELs are OFF before starting
    if (!deactivateBoth()) {
        setError("Failed to deactivate VCSELs before sequence");
        return false;
    }

    waitSettleTime();

    // FRAME A: VCSEL1 ON (pattern from left camera side)
    core::Logger::getInstance().info("[Frame A] Activating VCSEL1 (LEFT camera side)");
    auto vcsel1_start = std::chrono::steady_clock::now();

    if (!activateVCSEL1()) {
        setError("Failed to activate VCSEL1");
        deactivateBoth();
        return false;
    }

    waitCaptureDelay();

    if (!captureStereoFrame(result.frame_vcsel1_left, result.frame_vcsel1_right,
                           result.vcsel1_sync_error_ms)) {
        setError("Failed to capture VCSEL1 frame");
        deactivateBoth();
        return false;
    }

    auto vcsel1_end = std::chrono::steady_clock::now();
    result.vcsel1_activation_us = std::chrono::duration_cast<std::chrono::microseconds>(
        vcsel1_end - vcsel1_start).count();

    core::Logger::getInstance().info("[Frame A] VCSEL1 capture complete, sync error: " +
        std::to_string(result.vcsel1_sync_error_ms) + "ms");

    if (!deactivateVCSEL1()) {
        setError("Failed to deactivate VCSEL1");
        deactivateBoth();
        return false;
    }

    waitSettleTime();

    // FRAME B: VCSEL2 ON (pattern from right camera side)
    core::Logger::getInstance().info("[Frame B] Activating VCSEL2 (RIGHT camera side)");
    auto vcsel2_start = std::chrono::steady_clock::now();

    if (!activateVCSEL2()) {
        setError("Failed to activate VCSEL2");
        deactivateBoth();
        return false;
    }

    waitCaptureDelay();

    if (!captureStereoFrame(result.frame_vcsel2_left, result.frame_vcsel2_right,
                           result.vcsel2_sync_error_ms)) {
        setError("Failed to capture VCSEL2 frame");
        deactivateBoth();
        return false;
    }

    auto vcsel2_end = std::chrono::steady_clock::now();
    result.vcsel2_activation_us = std::chrono::duration_cast<std::chrono::microseconds>(
        vcsel2_end - vcsel2_start).count();

    core::Logger::getInstance().info("[Frame B] VCSEL2 capture complete, sync error: " +
        std::to_string(result.vcsel2_sync_error_ms) + "ms");

    if (!deactivateVCSEL2()) {
        setError("Failed to deactivate VCSEL2");
        deactivateBoth();
        return false;
    }

    waitSettleTime();

    // FRAME C: Both OFF (ambient illumination)
    core::Logger::getInstance().info("[Frame C] Capturing ambient (no VCSEL)");

    if (!captureStereoFrame(result.frame_ambient_left, result.frame_ambient_right,
                           result.ambient_sync_error_ms)) {
        setError("Failed to capture ambient frame");
        return false;
    }

    core::Logger::getInstance().info("[Frame C] Ambient capture complete, sync error: " +
        std::to_string(result.ambient_sync_error_ms) + "ms");

    // Calculate sequence timing
    auto sequence_end = std::chrono::steady_clock::now();
    result.total_sequence_us = std::chrono::duration_cast<std::chrono::microseconds>(
        sequence_end - sequence_start).count();

    // Read temperature
    result.temperature_c = readTemperature();

    // Update metadata
    result.capture_time = sequence_start;
    result.is_valid = true;

    // Update statistics
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.total_captures++;
    }

    total_captures_++;

    core::Logger::getInstance().info("Temporal sequence complete: " +
        std::to_string(result.total_sequence_us / 1000.0) + "ms total, temp: " +
        std::to_string(result.temperature_c) + "C");

    // Check thermal safety after capture
    if (!checkThermalSafety()) {
        core::Logger::getInstance().warning("Thermal limit exceeded, consider longer settle time");
    }

    return true;
}

bool AS1170DualVCSELController::activateVCSEL1(uint16_t current_ma) {
    if (!initialized_.load() || emergency_shutdown_.load()) {
        return false;
    }

    if (current_ma == 0) {
        current_ma = config_.vcsel1_current_ma;
    }

    if (!validateCurrent(current_ma)) {
        setError("VCSEL1 current out of range: " + std::to_string(current_ma) + "mA");
        return false;
    }

    if (!checkThermalSafety()) {
        setError("Thermal limit exceeded, cannot activate VCSEL1");
        return false;
    }

    if (!checkOnTimeLimits()) {
        setError("Maximum ON time exceeded, system needs cooldown");
        return false;
    }

    core::Logger::getInstance().debug("Activating VCSEL1 at " + std::to_string(current_ma) + "mA");

    if (!as1170_->setLEDState(AS1170Controller::LEDChannel::LED1, true, current_ma)) {
        setError("Failed to activate VCSEL1 hardware");
        return false;
    }

    vcsel1_activation_time_ = std::chrono::steady_clock::now();
    vcsel1_active_.store(true);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel1_active = true;
        status_.vcsel1_current_ma = current_ma;
        status_.last_activation = vcsel1_activation_time_;
    }

    return true;
}

bool AS1170DualVCSELController::activateVCSEL2(uint16_t current_ma) {
    if (!initialized_.load() || emergency_shutdown_.load()) {
        return false;
    }

    if (current_ma == 0) {
        current_ma = config_.vcsel2_current_ma;
    }

    if (!validateCurrent(current_ma)) {
        setError("VCSEL2 current out of range: " + std::to_string(current_ma) + "mA");
        return false;
    }

    if (!checkThermalSafety()) {
        setError("Thermal limit exceeded, cannot activate VCSEL2");
        return false;
    }

    if (!checkOnTimeLimits()) {
        setError("Maximum ON time exceeded, system needs cooldown");
        return false;
    }

    core::Logger::getInstance().debug("Activating VCSEL2 at " + std::to_string(current_ma) + "mA");

    if (!as1170_->setLEDState(AS1170Controller::LEDChannel::LED2, true, current_ma)) {
        setError("Failed to activate VCSEL2 hardware");
        return false;
    }

    vcsel2_activation_time_ = std::chrono::steady_clock::now();
    vcsel2_active_.store(true);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel2_active = true;
        status_.vcsel2_current_ma = current_ma;
        status_.last_activation = vcsel2_activation_time_;
    }

    return true;
}

bool AS1170DualVCSELController::deactivateVCSEL1() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().debug("Deactivating VCSEL1");

    if (!as1170_->setLEDState(AS1170Controller::LEDChannel::LED1, false, 0)) {
        setError("Failed to deactivate VCSEL1 hardware");
        return false;
    }

    // Update ON time statistics
    if (vcsel1_active_.load()) {
        auto now = std::chrono::steady_clock::now();
        auto on_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - vcsel1_activation_time_).count();
        total_on_time_ms_ += on_time_ms;
    }

    vcsel1_active_.store(false);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel1_active = false;
        status_.vcsel1_current_ma = 0;
    }

    return true;
}

bool AS1170DualVCSELController::deactivateVCSEL2() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().debug("Deactivating VCSEL2");

    if (!as1170_->setLEDState(AS1170Controller::LEDChannel::LED2, false, 0)) {
        setError("Failed to deactivate VCSEL2 hardware");
        return false;
    }

    // Update ON time statistics
    if (vcsel2_active_.load()) {
        auto now = std::chrono::steady_clock::now();
        auto on_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - vcsel2_activation_time_).count();
        total_on_time_ms_ += on_time_ms;
    }

    vcsel2_active_.store(false);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel2_active = false;
        status_.vcsel2_current_ma = 0;
    }

    return true;
}

bool AS1170DualVCSELController::deactivateBoth() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().debug("Deactivating both VCSELs");

    bool success = true;

    // Use BOTH channel for atomic deactivation
    if (!as1170_->setLEDState(AS1170Controller::LEDChannel::BOTH, false, 0)) {
        setError("Failed to deactivate both VCSELs");
        success = false;
    }

    // Update ON time statistics for both
    auto now = std::chrono::steady_clock::now();

    if (vcsel1_active_.load()) {
        auto on_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - vcsel1_activation_time_).count();
        total_on_time_ms_ += on_time_ms;
    }

    if (vcsel2_active_.load()) {
        auto on_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - vcsel2_activation_time_).count();
        total_on_time_ms_ += on_time_ms;
    }

    vcsel1_active_.store(false);
    vcsel2_active_.store(false);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel1_active = false;
        status_.vcsel2_active = false;
        status_.vcsel1_current_ma = 0;
        status_.vcsel2_current_ma = 0;
    }

    return success;
}

void AS1170DualVCSELController::emergencyShutdown() {
    emergency_shutdown_.store(true);

    core::Logger::getInstance().warning("AS1170 Dual VCSEL emergency shutdown activated");

    // Immediate hardware shutdown
    if (as1170_) {
        as1170_->emergencyShutdown();
    }

    vcsel1_active_.store(false);
    vcsel2_active_.store(false);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel1_active = false;
        status_.vcsel2_active = false;
        status_.vcsel1_current_ma = 0;
        status_.vcsel2_current_ma = 0;
        status_.error_message = "Emergency shutdown activated";
    }
}

float AS1170DualVCSELController::readTemperature() {
    if (!initialized_.load() || !as1170_) {
        return -999.0f;
    }

    float temp = as1170_->readTemperature();

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.temperature_c = temp;
    }

    return temp;
}

AS1170DualVCSELController::VCSELStatus AS1170DualVCSELController::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    auto status = status_;
    status.total_on_time_ms = total_on_time_ms_.load();
    status.total_captures = total_captures_.load();
    return status;
}

AS1170DualVCSELController::VCSELConfig AS1170DualVCSELController::getConfig() const {
    return config_;
}

bool AS1170DualVCSELController::isAnyVCSELActive() const {
    return vcsel1_active_.load() || vcsel2_active_.load();
}

bool AS1170DualVCSELController::updateConfig(const VCSELConfig& config) {
    if (!validateConfig(config)) {
        setError("Invalid configuration update");
        return false;
    }

    config_ = config;

    core::Logger::getInstance().info("VCSEL configuration updated");
    core::Logger::getInstance().info("VCSEL1: " + std::to_string(config_.vcsel1_current_ma) + "mA");
    core::Logger::getInstance().info("VCSEL2: " + std::to_string(config_.vcsel2_current_ma) + "mA");

    return true;
}

void AS1170DualVCSELController::setThermalCallback(ThermalCallback callback) {
    thermal_callback_ = callback;
}

bool AS1170DualVCSELController::validateCurrent(uint16_t current_ma) const {
    if (current_ma == 0) {
        return false;
    }

    if (current_ma > MAX_SAFE_CURRENT_MA) {
        core::Logger::getInstance().warning("Current " + std::to_string(current_ma) +
            "mA exceeds safe limit " + std::to_string(MAX_SAFE_CURRENT_MA) + "mA");
    }

    if (current_ma > ABSOLUTE_MAX_CURRENT_MA) {
        core::Logger::getInstance().error("Current " + std::to_string(current_ma) +
            "mA exceeds absolute hardware limit " + std::to_string(ABSOLUTE_MAX_CURRENT_MA) + "mA");
        return false;
    }

    return true;
}

uint64_t AS1170DualVCSELController::getTotalOnTime() const {
    return total_on_time_ms_.load();
}

void AS1170DualVCSELController::resetStatistics() {
    total_on_time_ms_.store(0);
    total_captures_.store(0);

    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.total_on_time_ms = 0;
    status_.total_captures = 0;
}

// Private methods

bool AS1170DualVCSELController::captureStereoFrame(
    cv::Mat& left,
    cv::Mat& right,
    double& sync_error_ms
) {
    if (!camera_system_ || !camera_system_->isInitialized()) {
        setError("Camera system not available");
        return false;
    }

    camera::StereoFrame frame;
    if (!camera_system_->captureStereoFrame(frame, 2000)) {
        setError("Failed to capture stereo frame");
        return false;
    }

    left = frame.leftImage.clone();
    right = frame.rightImage.clone();
    sync_error_ms = frame.syncErrorMs;

    if (!frame.isSynchronized) {
        core::Logger::getInstance().warning("Frame not properly synchronized, error: " +
            std::to_string(sync_error_ms) + "ms");
    }

    return true;
}

bool AS1170DualVCSELController::checkThermalSafety() {
    if (!config_.enable_thermal_monitoring) {
        return true;
    }

    float temp = readTemperature();

    if (temp < -100.0f) {
        // Invalid temperature reading
        core::Logger::getInstance().warning("Invalid temperature reading");
        return true; // Don't fail on sensor error
    }

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.temperature_c = temp;

        if (temp >= config_.max_operating_temp_c) {
            status_.thermal_throttling = true;
            status_.error_message = "Temperature limit exceeded: " +
                std::to_string(temp) + "C >= " + std::to_string(config_.max_operating_temp_c) + "C";

            if (thermal_callback_) {
                thermal_callback_(true, temp);
            }

            core::Logger::getInstance().error(status_.error_message);
            return false;
        }

        if (status_.thermal_throttling && temp < (config_.max_operating_temp_c - 5.0f)) {
            // 5C hysteresis for thermal recovery
            status_.thermal_throttling = false;
            core::Logger::getInstance().info("Thermal condition cleared, temp: " +
                std::to_string(temp) + "C");

            if (thermal_callback_) {
                thermal_callback_(false, temp);
            }
        }
    }

    return true;
}

void AS1170DualVCSELController::updateStatus() {
    // This should be called with status_mutex_ locked
    status_.total_on_time_ms = total_on_time_ms_.load();
    status_.total_captures = total_captures_.load();
}

bool AS1170DualVCSELController::validateConfig(const VCSELConfig& config) const {
    if (!validateCurrent(config.vcsel1_current_ma)) {
        core::Logger::getInstance().error("Invalid VCSEL1 current: " +
            std::to_string(config.vcsel1_current_ma) + "mA");
        return false;
    }

    if (!validateCurrent(config.vcsel2_current_ma)) {
        core::Logger::getInstance().error("Invalid VCSEL2 current: " +
            std::to_string(config.vcsel2_current_ma) + "mA");
        return false;
    }

    if (config.settle_time_ms < 10 || config.settle_time_ms > 1000) {
        core::Logger::getInstance().error("Invalid settle time: " +
            std::to_string(config.settle_time_ms) + "ms (valid: 10-1000ms)");
        return false;
    }

    if (config.capture_delay_ms > 100) {
        core::Logger::getInstance().error("Invalid capture delay: " +
            std::to_string(config.capture_delay_ms) + "ms (max: 100ms)");
        return false;
    }

    if (config.max_on_time_ms < 100 || config.max_on_time_ms > MAX_ON_TIME_MS) {
        core::Logger::getInstance().error("Invalid max ON time: " +
            std::to_string(config.max_on_time_ms) + "ms (valid: 100-" +
            std::to_string(MAX_ON_TIME_MS) + "ms)");
        return false;
    }

    if (config.max_operating_temp_c < 30.0f || config.max_operating_temp_c > 80.0f) {
        core::Logger::getInstance().error("Invalid max temperature: " +
            std::to_string(config.max_operating_temp_c) + "C (valid: 30-80C)");
        return false;
    }

    return true;
}

bool AS1170DualVCSELController::checkOnTimeLimits() const {
    uint64_t total_on = total_on_time_ms_.load();

    if (total_on > config_.max_on_time_ms) {
        core::Logger::getInstance().warning("Total ON time " + std::to_string(total_on) +
            "ms exceeds limit " + std::to_string(config_.max_on_time_ms) + "ms");
        return false;
    }

    return true;
}

void AS1170DualVCSELController::waitSettleTime() {
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.settle_time_ms));
}

void AS1170DualVCSELController::waitCaptureDelay() {
    std::this_thread::sleep_for(std::chrono::milliseconds(config_.capture_delay_ms));
}

void AS1170DualVCSELController::setError(const std::string& error) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.error_message = error;
    core::Logger::getInstance().error("AS1170DualVCSELController: " + error);
}

void AS1170DualVCSELController::clearError() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.error_message.clear();
}

} // namespace hardware
} // namespace unlook
