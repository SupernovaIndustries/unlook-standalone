#include <unlook/hardware/StabilityDetector.hpp>
#include <unlook/core/Logger.hpp>

#include <cmath>
#include <algorithm>
#include <numeric>

namespace unlook {
namespace hardware {

StabilityDetector::StabilityDetector(std::shared_ptr<BMI270Driver> imu_driver) :
    imu_driver_(imu_driver),
    initialized_(false),
    was_stable_(false),
    last_gyro_magnitude_(0.0f),
    last_accel_variance_(0.0f),
    last_stability_score_(0.0f) {

    stable_since_ = std::chrono::steady_clock::now();
}

StabilityDetector::~StabilityDetector() {
    shutdown();
}

bool StabilityDetector::initialize(const StabilityParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("StabilityDetector already initialized");
        return true;
    }

    if (!imu_driver_ || !imu_driver_->isInitialized()) {
        core::Logger::getInstance().error("BMI270Driver not initialized");
        return false;
    }

    params_ = params;

    // Validate parameters
    if (params_.gyro_weight + params_.accel_weight != 1.0f) {
        core::Logger::getInstance().warning("Gyro and accel weights don't sum to 1.0, normalizing...");
        float total = params_.gyro_weight + params_.accel_weight;
        params_.gyro_weight /= total;
        params_.accel_weight /= total;
    }

    // Reserve history capacity for efficiency
    int max_samples = (params_.history_window_ms * 200) / 1000;  // Assume max 200 Hz
    history_.clear();

    // Initialize status
    status_ = StabilityStatus();
    was_stable_ = false;
    stable_since_ = std::chrono::steady_clock::now();

    initialized_.store(true);

    core::Logger::getInstance().info("StabilityDetector initialized successfully");
    core::Logger::getInstance().info("Parameters: Gyro threshold " +
                      std::to_string(params_.gyro_threshold_dps) + " deg/s, " +
                      "Accel variance " + std::to_string(params_.accel_variance_threshold) + " m/s², " +
                      "Stable duration " + std::to_string(params_.stable_duration_ms) + " ms");

    return true;
}

void StabilityDetector::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return;
    }

    core::Logger::getInstance().info("Shutting down StabilityDetector...");

    history_.clear();
    initialized_.store(false);

    core::Logger::getInstance().info("StabilityDetector shutdown complete");
}

bool StabilityDetector::update() {
    if (!initialized_.load()) {
        return false;
    }

    // Read new IMU data
    BMI270Driver::IMUData imu_data;
    if (!imu_driver_->readIMUData(imu_data)) {
        return false;
    }

    if (!imu_data.valid) {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Add to history with timestamp
    TimestampedIMUData timestamped_data;
    timestamped_data.data = imu_data;
    timestamped_data.timestamp = std::chrono::steady_clock::now();
    history_.push_back(timestamped_data);

    // Prune old data outside history window
    pruneHistory();

    // Calculate current metrics
    last_gyro_magnitude_ = calculateGyroMagnitude(imu_data);
    last_accel_variance_ = calculateAccelVariance();
    last_stability_score_ = calculateOverallScore();

    // Update stability state
    updateStableState();

    // Update status
    status_.current_gyro_magnitude = last_gyro_magnitude_;
    status_.current_accel_variance = last_accel_variance_;
    status_.stability_score = last_stability_score_;
    status_.samples_in_history = history_.size();
    status_.timestamp = std::chrono::steady_clock::now();

    return true;
}

bool StabilityDetector::isStable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_.is_stable;
}

float StabilityDetector::getStabilityScore() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_.stability_score;
}

int StabilityDetector::getStableDuration() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_.stable_duration_ms;
}

StabilityDetector::StabilityStatus StabilityDetector::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

void StabilityDetector::reset() {
    std::lock_guard<std::mutex> lock(mutex_);

    history_.clear();
    status_ = StabilityStatus();
    was_stable_ = false;
    stable_since_ = std::chrono::steady_clock::now();
    last_gyro_magnitude_ = 0.0f;
    last_accel_variance_ = 0.0f;
    last_stability_score_ = 0.0f;

    core::Logger::getInstance().info("StabilityDetector reset");
}

void StabilityDetector::setParameters(const StabilityParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;

    // Normalize weights if needed
    if (params_.gyro_weight + params_.accel_weight != 1.0f) {
        float total = params_.gyro_weight + params_.accel_weight;
        params_.gyro_weight /= total;
        params_.accel_weight /= total;
    }

    core::Logger::getInstance().info("StabilityDetector parameters updated");
}

StabilityDetector::StabilityParams StabilityDetector::getParameters() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return params_;
}

// Private methods

float StabilityDetector::calculateGyroMagnitude(const BMI270Driver::IMUData& data) const {
    // Calculate Euclidean magnitude of gyro vector
    float magnitude = std::sqrt(
        data.gyro_x * data.gyro_x +
        data.gyro_y * data.gyro_y +
        data.gyro_z * data.gyro_z
    );
    return magnitude;
}

float StabilityDetector::calculateAccelVariance() const {
    if (history_.empty()) {
        return 0.0f;
    }

    // Calculate variance of acceleration magnitude
    std::vector<float> accel_magnitudes;
    accel_magnitudes.reserve(history_.size());

    for (const auto& entry : history_) {
        float magnitude = std::sqrt(
            entry.data.accel_x * entry.data.accel_x +
            entry.data.accel_y * entry.data.accel_y +
            entry.data.accel_z * entry.data.accel_z
        );
        accel_magnitudes.push_back(magnitude);
    }

    // Calculate mean
    float mean = std::accumulate(accel_magnitudes.begin(), accel_magnitudes.end(), 0.0f) /
                 accel_magnitudes.size();

    // Calculate variance
    float variance = 0.0f;
    for (float magnitude : accel_magnitudes) {
        float diff = magnitude - mean;
        variance += diff * diff;
    }
    variance /= accel_magnitudes.size();

    return std::sqrt(variance);  // Return standard deviation instead of variance
}

float StabilityDetector::calculateGyroScore() const {
    // Gyro score: 1.0 when below threshold, decreases linearly above threshold
    float score = 1.0f - (last_gyro_magnitude_ / params_.gyro_threshold_dps);
    return std::clamp(score, 0.0f, 1.0f);
}

float StabilityDetector::calculateAccelScore() const {
    // Accel score: 1.0 when variance below threshold, decreases linearly above
    float score = 1.0f - (last_accel_variance_ / params_.accel_variance_threshold);
    return std::clamp(score, 0.0f, 1.0f);
}

float StabilityDetector::calculateOverallScore() const {
    float gyro_score = calculateGyroScore();
    float accel_score = calculateAccelScore();

    // Weighted combination
    float overall = gyro_score * params_.gyro_weight + accel_score * params_.accel_weight;

    return std::clamp(overall, 0.0f, 1.0f);
}

bool StabilityDetector::checkStabilityCriteria() const {
    // Check gyro threshold (all axes must be below threshold)
    if (last_gyro_magnitude_ >= params_.gyro_threshold_dps) {
        return false;
    }

    // Check accel variance threshold
    if (last_accel_variance_ >= params_.accel_variance_threshold) {
        return false;
    }

    // Check individual gyro axes for more strict detection
    if (history_.empty()) {
        return false;
    }

    const auto& latest = history_.back().data;
    if (std::abs(latest.gyro_x) >= params_.gyro_threshold_dps ||
        std::abs(latest.gyro_y) >= params_.gyro_threshold_dps ||
        std::abs(latest.gyro_z) >= params_.gyro_threshold_dps) {
        return false;
    }

    return true;
}

void StabilityDetector::updateStableState() {
    bool currently_stable = checkStabilityCriteria();

    if (currently_stable) {
        if (!was_stable_) {
            // Just became stable - reset timer
            stable_since_ = std::chrono::steady_clock::now();
            was_stable_ = true;
        }

        // Calculate duration of stable state
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - stable_since_);
        status_.stable_duration_ms = static_cast<int>(duration.count());

        // Check if we've been stable long enough
        status_.is_stable = (status_.stable_duration_ms >= params_.stable_duration_ms);
    } else {
        // Not stable - reset state
        was_stable_ = false;
        status_.is_stable = false;
        status_.stable_duration_ms = 0;
    }
}

void StabilityDetector::pruneHistory() {
    if (history_.empty()) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto cutoff_time = now - std::chrono::milliseconds(params_.history_window_ms);

    // Remove old entries from front
    while (!history_.empty() && history_.front().timestamp < cutoff_time) {
        history_.pop_front();
    }
}

} // namespace hardware
} // namespace unlook
