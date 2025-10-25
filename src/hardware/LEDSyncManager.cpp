#include <unlook/hardware/LEDSyncManager.hpp>
#include <unlook/core/Logger.hpp>

#include <algorithm>
#include <numeric>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <fstream>

namespace unlook {
namespace hardware {

LEDSyncManager::LEDSyncManager() {
    // Initialize status with defaults
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_sync = std::chrono::steady_clock::now();
}

LEDSyncManager::~LEDSyncManager() {
    //     shutdown();
}

bool LEDSyncManager::initialize(std::shared_ptr<AS1170Controller> as1170_controller,
                               const SyncConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("LEDSyncManager already initialized");
        return true;
    }

    if (!as1170_controller || !as1170_controller->isInitialized()) {
        handleSyncError("AS1170Controller not provided or not initialized");
        return false;
    }

    if (!isValidConfig(config)) {
        handleSyncError("Invalid synchronization configuration");
        return false;
    }

    as1170_controller_ = as1170_controller;
    config_ = config;

    // Initialize timing tracking
    sync_start_time_ = std::chrono::steady_clock::now();
    duty_cycle_start_ = sync_start_time_;
    led_on_time_ = std::chrono::duration<double>(0);

    // Clear statistics
    //     resetStatistics();

    //     initialized_.store(true);
    //     emergency_stop_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = true;
        status_.current_mode = SyncMode::DISABLED;
        status_.current_pattern = config_.pattern;
    //         status_.error_message.clear();
    }

    core::Logger::getInstance().info("LEDSyncManager initialized successfully");
    core::Logger::getInstance().info("Using AS1170Controller with FINAL hardware config: I2C Bus 1, Address 0x30, GPIO 573 (physical GPIO4)");
    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Sync mode: {}, Pattern: {}, Pre-delay: {}μs, Strobe: {}μs",
//                       static_cast<int>(config_.mode),
//                       static_cast<int>(config_.pattern),
    //                       config_.pre_capture_delay_us,
    //                       config_.strobe_duration_us);

    return true;
}

void LEDSyncManager::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
    //         return;
    }

    core::Logger::getInstance().info("Shutting down LEDSyncManager...");

    //     stopSync();
    //     emergencyStop();

    // Wait for sync thread to finish
    if (sync_thread_ && sync_thread_->joinable()) {
    //         sync_condition_.notify_all();
        sync_thread_->join();
    //         sync_thread_.reset();
    }

    // Clear callbacks
    sync_callback_ = nullptr;
    capture_callback_ = nullptr;
    error_callback_ = nullptr;

    //     as1170_controller_.reset();
    //     initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.sync_active = false;
        status_.current_mode = SyncMode::DISABLED;
    }

    core::Logger::getInstance().info("LEDSyncManager shutdown complete");
}

bool LEDSyncManager::startSync(SyncMode mode) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_stop_.load()) {
        handleSyncError("LEDSyncManager not initialized or in emergency stop");
        return false;
    }

    if (sync_active_.load()) {
        core::Logger::getInstance().warning("Sync already active, stopping previous sync");
    //         stopSync();
    }

    config_.mode = mode;

    if (mode == SyncMode::DISABLED) {
        core::Logger::getInstance().info("LED synchronization disabled");
        return true;
    }

    // Start synchronization thread
    sync_thread_ = std::make_unique<std::thread>(&LEDSyncManager::syncThreadWorker, this);

    //     sync_active_.store(true);
    sync_start_time_ = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_active = true;
        status_.current_mode = mode;
        status_.sync_cycles = 0;
        status_.sync_errors = 0;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("LED synchronization started with mode: {}", static_cast<int>(mode));
    return true;
}

void LEDSyncManager::stopSync() {
    if (!sync_active_.load()) {
    //         return;
    }

    //     sync_active_.store(false);

    {
        std::lock_guard<std::mutex> sync_lock(sync_thread_mutex_);
    //         sync_condition_.notify_all();
    }

    if (sync_thread_ && sync_thread_->joinable()) {
        sync_thread_->join();
    //         sync_thread_.reset();
    }

    // Turn off all LEDs
    //     deactivatePattern();

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_active = false;
        status_.current_mode = SyncMode::DISABLED;
        status_.led1_active = false;
        status_.led2_active = false;
        status_.current_led1_ma = 0;
        status_.current_led2_ma = 0;
    }

    core::Logger::getInstance().info("LED synchronization stopped");
}

bool LEDSyncManager::triggerSyncCapture(uint32_t exposure_time_us, CaptureCallback callback) {
    if (!initialized_.load() || emergency_stop_.load()) {
        return false;
    }

    if (!sync_active_.load()) {
        handleSyncError("Sync not active - call startSync() first");
        return false;
    }

    // Validate timing parameters
    if (!validateTiming(exposure_time_us)) {
        handleSyncError("Invalid exposure timing parameters");
        return false;
    }

    PendingSyncOp op;
    op.trigger_time = std::chrono::steady_clock::now(); // + std::chrono::microseconds(config_.pre_capture_delay_us);
    op.duration_us = std::max(exposure_time_us, config_.strobe_duration_us);
    op.pattern = config_.pattern;
    op.callback = callback;

    {
        std::lock_guard<std::mutex> sync_lock(sync_thread_mutex_);
    //         pending_ops_.push(op);
    //         sync_condition_.notify_one();
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Sync capture triggered: exposure={}μs, pattern={}, delay={}μs",
    //                        exposure_time_us, static_cast<int>(config_.pattern),
    //                        config_.pre_capture_delay_us);

    return true;
}

bool LEDSyncManager::preActivateIllumination(IlluminationPattern pattern, uint32_t duration_us) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_stop_.load()) {
        return false;
    }

    if (!checkDutyCycle()) {
        handleSyncError("Duty cycle limit exceeded - cannot pre-activate");
        return false;
    }

    if (!activatePattern(pattern, duration_us)) {
        handleSyncError("Failed to activate illumination pattern");
        return false;
    }

    // Schedule deactivation
    std::thread deactivate_thread([this, duration_us]() {
    //         std::this_thread::sleep_for(std::chrono::microseconds(duration_us));
        this->deactivatePattern();
    });
    //     deactivate_thread.detach();

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Pre-activated illumination: pattern={}, duration={}μs",
//                       static_cast<int>(pattern), duration_us);

    return true;
}

bool LEDSyncManager::configureSynchronization(const SyncConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!isValidConfig(config)) {
        handleSyncError("Invalid synchronization configuration");
        return false;
    }

    bool was_active = sync_active_.load();
    SyncMode current_mode = config_.mode;

    if (was_active) {
    //         stopSync();
    }

    config_ = config;

    if (was_active) {
        return startSync(current_mode);
    }

    core::Logger::getInstance().info("Synchronization configuration updated");
    return true;
}

bool LEDSyncManager::setIlluminationPattern(IlluminationPattern pattern,
                                           uint16_t flood_current_ma,
                                           uint16_t vcsel_current_ma) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!isValidPattern(pattern)) {
        return false;
    }

    config_.pattern = pattern;

    if (flood_current_ma > 0) {
        config_.flood_current_ma = flood_current_ma;
    }
    if (vcsel_current_ma > 0) {
        config_.vcsel_current_ma = vcsel_current_ma;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_pattern = pattern;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Illumination pattern set: {}, flood={}mA, vcsel={}mA",
//                       static_cast<int>(pattern), config_.flood_current_ma, config_.vcsel_current_ma);

    return true;
}

bool LEDSyncManager::onFrameSync(uint64_t timestamp_ns) {
    if (!sync_active_.load()) {
        return true; // Sync not active, ignore frame sync
    }

    last_frame_sync_ = std::chrono::steady_clock::now();

    // Calculate frame rate for statistics
    static uint64_t last_timestamp = 0;
    if (last_timestamp > 0) {
        double frame_interval_ms = (timestamp_ns - last_timestamp) / 1e6;
        if (frame_interval_ms > 0) {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.measured_fps = 1000.0 / frame_interval_ms;
        }
    }
    last_timestamp = timestamp_ns;

    // Notify sync callback if registered
    if (sync_callback_) {
        sync_callback_(status_.led1_active || status_.led2_active, timestamp_ns);
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Frame sync received: timestamp={}ns, fps={:.1f}",
    //                        timestamp_ns, status_.measured_fps);

    return true;
}

void LEDSyncManager::emergencyStop() {
    //     emergency_stop_.store(true);

    // Immediately turn off all LEDs
    if (as1170_controller_) {
        as1170_controller_->emergencyShutdown();
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_active = false;
        status_.led2_active = false;
        status_.current_led1_ma = 0;
        status_.current_led2_ma = 0;
    }

    core::Logger::getInstance().warning("LEDSyncManager emergency stop activated");
}

LEDSyncManager::SyncStatus LEDSyncManager::getStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto status = status_;
    status.last_sync = std::chrono::steady_clock::now();
    return status;
}

LEDSyncManager::SyncConfig LEDSyncManager::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void LEDSyncManager::setSyncCallback(SyncCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    sync_callback_ = callback;
}

void LEDSyncManager::setCaptureCallback(CaptureCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    capture_callback_ = callback;
}

void LEDSyncManager::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_callback_ = callback;
}

bool LEDSyncManager::setManualLEDState(AS1170Controller::LEDChannel channel,
                                      bool enable, uint16_t current_ma) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!as1170_controller_) {
        return false;
    }

    core::Logger::getInstance().warning("Manual LED control - bypassing synchronization system");

    bool result = as1170_controller_->setLEDState(channel, enable, current_ma);

    if (result) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        if (channel == AS1170Controller::LEDChannel::LED1 ||
            channel == AS1170Controller::LEDChannel::BOTH) {
            status_.led1_active = enable;
            status_.current_led1_ma = enable ? current_ma : 0;
        }
        if (channel == AS1170Controller::LEDChannel::LED2 ||
            channel == AS1170Controller::LEDChannel::BOTH) {
            status_.led2_active = enable;
            status_.current_led2_ma = enable ? current_ma : 0;
        }
    }

    return result;
}

uint32_t LEDSyncManager::calculateOptimalTiming(uint32_t exposure_time_us, double fps) const {
    // Calculate optimal pre-capture delay based on exposure time and frame rate
    uint32_t frame_time_us = static_cast<uint32_t>(1e6 / fps);

    // Ensure LED activation happens early enough for proper illumination
    uint32_t optimal_delay = std::max(
    //         config_.pre_capture_delay_us,
        static_cast<uint32_t>(exposure_time_us * 0.1),  // 10% of exposure time
        1000u  // minimum delay in microseconds
    );

    // Don't exceed frame time
    optimal_delay = std::min(optimal_delay, frame_time_us / 4);

    return optimal_delay;
}

LEDSyncManager::SyncAccuracy LEDSyncManager::measureSyncAccuracy() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);

    SyncAccuracy accuracy{};

    if (sync_error_history_.empty()) {
        return accuracy;
    }

    // Calculate statistics from error history
    std::vector<double> errors;
    auto queue_copy = sync_error_history_;
    while (!queue_copy.empty()) {
    //         errors.push_back(queue_copy.front());
    //         queue_copy.pop();
    }

    accuracy.samples = errors.size();
    accuracy.mean_error_us = std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
    accuracy.max_error_us = *std::max_element(errors.begin(), errors.end());

    // Calculate standard deviation
    double variance = 0.0;
    for (double error : errors) {
        variance += (error - accuracy.mean_error_us) * (error - accuracy.mean_error_us);
    }
    accuracy.std_dev_us = std::sqrt(variance / errors.size());

    return accuracy;
}

bool LEDSyncManager::isInSync() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return status_.sync_active &&
           status_.avg_sync_error_us < config_.sync_tolerance_us &&
           status_.max_sync_error_us < (config_.sync_tolerance_us * 2);
}

// Private implementation methods

void LEDSyncManager::syncThreadWorker() {
    core::Logger::getInstance().info("LED sync thread started");

    while (sync_active_.load() && !emergency_stop_.load()) {
        try {
            std::unique_lock<std::mutex> lock(sync_thread_mutex_);

            // Wait for sync operations or timeout
            sync_condition_.wait_for(lock, std::chrono::milliseconds(10), [this] {
                return !pending_ops_.empty() || !sync_active_.load();
            });

            if (!sync_active_.load()) {
    //                 break;
            }

            // Process pending sync operations
            while (!pending_ops_.empty() && sync_active_.load()) {
                auto op = pending_ops_.front();
    //                 pending_ops_.pop();
    //                 lock.unlock();

                // Wait until trigger time
    //                 std::this_thread::sleep_until(op.trigger_time);

                if (sync_active_.load() && !emergency_stop_.load()) {
    //                     executeSync(op);
                }

    //                 lock.lock();
            }

        } catch (const std::exception& e) {
            handleSyncError("Sync thread exception: " + std::string(e.what()));
        }
    }

    core::Logger::getInstance().info("LED sync thread stopped");
}

bool LEDSyncManager::executeSync(const PendingSyncOp& op) {
    auto start_time = std::chrono::steady_clock::now();

    // Activate pattern
    if (!activatePattern(op.pattern, op.duration_us)) {
        handleSyncError("Failed to activate LED pattern during sync");
        return false;
    }

    // Wait for duration
    //     std::this_thread::sleep_for(std::chrono::microseconds(op.duration_us));

    // Deactivate pattern
    //     deactivatePattern();

    auto end_time = std::chrono::steady_clock::now();
    auto actual_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        end_time - start_time).count();

    // Calculate sync error
    double sync_error_us = std::abs(static_cast<double>(actual_duration) - op.duration_us);
    //     updateSyncStatistics(sync_error_us);

    // Call completion callback
    if (op.callback) {
        uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
        op.callback(timestamp_ns);
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_cycles++;

        if (sync_error_us > config_.sync_tolerance_us) {
            status_.sync_errors++;
        }
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Sync executed: duration={}μs, error={:.1f}μs",
    //                        actual_duration, sync_error_us);

    return true;
}

bool LEDSyncManager::activatePattern(IlluminationPattern pattern, uint32_t duration_us) {
    if (!as1170_controller_ || emergency_stop_.load()) {
        return false;
    }

    //     updateDutyCycle(true);

    bool success = false;

    switch (pattern) {
        case IlluminationPattern::FLOOD_ONLY:
            success = activateFloodOnly(config_.flood_current_ma);
            break;

        case IlluminationPattern::VCSEL_ONLY:
            success = activateVCSELOnly(config_.vcsel_current_ma);
            break;

        case IlluminationPattern::ALTERNATING:
            success = activateAlternating(config_.flood_current_ma,
                                                config_.vcsel_current_ma, duration_us);
            break;

        case IlluminationPattern::COMBINED:
            success = activateCombined(config_.flood_current_ma, config_.vcsel_current_ma);
            break;

        case IlluminationPattern::ADAPTIVE:
            success = activateAdaptive(duration_us);
            break;

        default:
    // TODO: Fix formatted logging -             core::Logger::getInstance().error("Invalid illumination pattern: {}", static_cast<int>(pattern));
            return false;
    }

    if (success) {
    //         updateStatus();
    }

    return success;
}

bool LEDSyncManager::deactivatePattern() {
    if (!as1170_controller_) {
        return false;
    }

    //     updateDutyCycle(false);

    bool success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::BOTH, false, 0);

    if (success) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_active = false;
        status_.led2_active = false;
        status_.current_led1_ma = 0;
        status_.current_led2_ma = 0;
    }

    return success;
}

bool LEDSyncManager::activateFloodOnly(uint16_t current_ma) {
    bool success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED2,
                                                       true, current_ma);
    if (success) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_active = false;
        status_.led2_active = true;
        status_.current_led1_ma = 0;
        status_.current_led2_ma = current_ma;
    }
    return success;
}

bool LEDSyncManager::activateVCSELOnly(uint16_t current_ma) {
    bool success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED1,
                                                       true, current_ma);
    if (success) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_active = true;
        status_.led2_active = false;
        status_.current_led1_ma = current_ma;
        status_.current_led2_ma = 0;
    }
    return success;
}

bool LEDSyncManager::activateAlternating(uint16_t flood_ma, uint16_t vcsel_ma, uint32_t duration_us) {
    // Simple alternating pattern - start with flood
    return activateFloodOnly(flood_ma);
}

bool LEDSyncManager::activateCombined(uint16_t flood_ma, uint16_t vcsel_ma) {
    bool success1 = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED1,
                                                        true, vcsel_ma);
    bool success2 = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED2,
                                                        true, flood_ma);

    if (success1 && success2) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_active = true;
        status_.led2_active = true;
        status_.current_led1_ma = vcsel_ma;
        status_.current_led2_ma = flood_ma;
    }

    return success1 && success2;
}

bool LEDSyncManager::activateAdaptive(uint32_t exposure_time_us) {
    // Adaptive pattern based on exposure time
    if (exposure_time_us < 5000) {  // Short exposure - use VCSEL
        return activateVCSELOnly(config_.vcsel_current_ma);
    } else {  // Longer exposure - use flood
        return activateFloodOnly(config_.flood_current_ma);
    }
}

bool LEDSyncManager::checkDutyCycle() const {
    auto current_time = std::chrono::steady_clock::now();
    auto total_time = current_time - duty_cycle_start_;

    if (total_time.count() == 0) {
        return true;  // No time elapsed
    }

    double current_duty_cycle = led_on_time_.count() /
                               std::chrono::duration<double>(total_time).count();

    return current_duty_cycle < config_.max_duty_cycle;
}

bool LEDSyncManager::validateTiming(uint32_t duration_us) const {
    // Check minimum duration
    if (duration_us < 100) {  // 100μs minimum
        return false;
    }

    // Check maximum duration based on duty cycle
    uint32_t max_duration_us = static_cast<uint32_t>(
        (1e6 / status_.measured_fps) * config_.max_duty_cycle);

    return duration_us <= max_duration_us;
}

void LEDSyncManager::updateDutyCycle(bool led_active) {
    static bool last_state = false;
    static auto last_time = std::chrono::steady_clock::now();

    auto current_time = std::chrono::steady_clock::now();

    if (last_state) {  // LEDs were on
        led_on_time_ += current_time - last_time;
    }

    last_state = led_active;
    last_time = current_time;

    // Update duty cycle in status
    auto total_time = current_time - duty_cycle_start_;
    if (total_time.count() > 0) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_duty_cycle = led_on_time_.count() /
                                   std::chrono::duration<double>(total_time).count();
    }
}

void LEDSyncManager::handleSyncError(const std::string& error) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.error_message = error;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().error("LEDSyncManager: {}", error);

    if (error_callback_) {
    //         error_callback_(error);
    }
}

bool LEDSyncManager::isValidPattern(IlluminationPattern pattern) const {
    return pattern >= IlluminationPattern::FLOOD_ONLY &&
           pattern <= IlluminationPattern::ADAPTIVE;
}

bool LEDSyncManager::isValidConfig(const SyncConfig& config) const {
    if (config.pre_capture_delay_us > 100000) {  // 100ms max
        return false;
    }

    if (config.strobe_duration_us < 100 || config.strobe_duration_us > 50000) {  // 100μs - 50ms
        return false;
    }

    if (config.sync_tolerance_us > 1000) {  // 1ms max tolerance
        return false;
    }

    if (config.max_duty_cycle <= 0.0f || config.max_duty_cycle > 1.0f) {
        return false;
    }

    return true;
}

void LEDSyncManager::resetStatistics() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.sync_cycles = 0;
    status_.sync_errors = 0;
    status_.avg_sync_error_us = 0.0;
    status_.max_sync_error_us = 0.0;

    // Clear error history
    while (!sync_error_history_.empty()) {
    //         sync_error_history_.pop();
    }
}

void LEDSyncManager::updateStatus() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_sync = std::chrono::steady_clock::now();
}

uint64_t LEDSyncManager::getCurrentTimestamp() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

double LEDSyncManager::calculateSyncError(uint64_t expected_ns, uint64_t actual_ns) const {
    return std::abs(static_cast<double>(actual_ns - expected_ns)) / 1000.0;  // Convert to μs
}

void LEDSyncManager::updateSyncStatistics(double error_us) {
    std::lock_guard<std::mutex> status_lock(status_mutex_);

    // Update running average
    if (status_.sync_cycles > 0) {
        status_.avg_sync_error_us = ((status_.avg_sync_error_us * status_.sync_cycles) + error_us) /
                                   (status_.sync_cycles + 1);
    } else {
        status_.avg_sync_error_us = error_us;
    }

    // Update maximum error
    status_.max_sync_error_us = std::max(status_.max_sync_error_us, error_us);

    // Add to history queue
    //     sync_error_history_.push(error_us);
    if (sync_error_history_.size() > MAX_ERROR_HISTORY) {
    //         sync_error_history_.pop();
    }
}

} // namespace hardware
} // namespace unlook