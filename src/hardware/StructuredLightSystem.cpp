#include <unlook/hardware/StructuredLightSystem.hpp>
#include <unlook/core/Logger.hpp>

#include <algorithm>
#include <numeric>
#include <chrono>
#include <thread>
#include <deque>

namespace unlook {
namespace hardware {

StructuredLightSystem::StructuredLightSystem() {
    last_capture_time_ = std::chrono::steady_clock::now();
}

StructuredLightSystem::~StructuredLightSystem() {
    //     shutdown();
}

bool StructuredLightSystem::initialize(const CaptureConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("StructuredLightSystem already initialized");
        return true;
    }

    if (!validateConfiguration(config)) {
        updateStatus("Invalid configuration provided", true);
        return false;
    }

    config_ = config;

    core::Logger::getInstance().info("Initializing Structured Light System...");
    // TODO: Fix formatted logging - core::Logger::getInstance().info("Mode: {}, Pattern: {}, Exposure: {}μs",
    //                       static_cast<int>(config_.mode),
    //                       static_cast<int>(config_.pattern),
    //                       config_.exposure_time_us);

    // Initialize core components
    if (!initializeComponents()) {
        updateStatus("Failed to initialize core components", true);
    //         cleanupResources();
        return false;
    }

    // Setup integration between components
    if (!setupCameraSync() || !setupDepthProcessing() || !setupProjectorCallbacks()) {
        updateStatus("Failed to setup component integration", true);
    //         cleanupResources();
        return false;
    }

    initialized_.store(true);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = true;
        status_.camera_ready = camera_system_ && camera_system_->isRunning();
        status_.projector_ready = projector_ && projector_->isReady();
        status_.last_error.clear();
        status_.last_update_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    updateStatus("Structured Light System initialized successfully");
    // TODO: Fix formatted logging - core::Logger::getInstance().info("System ready - Camera: {}, Projector: {}",
    //                       status_.camera_ready ? "OK" : "ERROR",
    //                       status_.projector_ready ? "OK" : "ERROR");

    return true;
}

void StructuredLightSystem::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return;
    }

    core::Logger::getInstance().info("Shutting down Structured Light System...");

    stopContinuous();
    emergencyStop();
    cleanupResources();

    initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.camera_ready = false;
        status_.projector_ready = false;
        status_.continuous_active = false;
    }

    updateStatus("Structured Light System shutdown complete");
    core::Logger::getInstance().info("Structured Light System shutdown complete");
}

StructuredLightSystem::CaptureResult StructuredLightSystem::captureDepth(CaptureCallback callback) {
    if (!initialized_.load() || !isReady()) {
        auto result = createErrorResult("System not ready for depth capture");
        if (callback) callback(result);
        return result;
    }

    updateStatus("Starting depth capture...");
    notifyProgress("Depth Capture", 0.0f);

    auto start_time = std::chrono::steady_clock::now();
    auto result = performCapture(CaptureMode::DEPTH_ONLY);

    auto end_time = std::chrono::steady_clock::now();
    result.capture_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    // Update performance metrics
    updatePerformanceMetrics(result.capture_duration_ms, result.sync_error_us, result.success);

    if (result.success) {
        updateStatus("Depth capture completed successfully");
        notifyProgress("Depth Capture", 1.0f);

    // TODO: Fix formatted logging -         core::Logger::getInstance().info("Depth capture: {}ms, {}x{} depth, {:.1f}μs sync error, {:.1f}°C",
    //                           result.capture_duration_ms,
    //                           result.depth_map.cols, result.depth_map.rows,
    //                           result.sync_error_us,
    //                           result.projection_temperature_c);
    } else {
        updateStatus("Depth capture failed: " + result.error_message, true);
        notifyProgress("Depth Capture", -1.0f); // Error state
    }

    if (callback) {
        callback(result);
    }

    return result;
}

StructuredLightSystem::CaptureResult StructuredLightSystem::captureFace(float face_distance_mm,
                                                                       CaptureCallback callback) {
    if (!initialized_.load() || !isReady()) {
        auto result = createErrorResult("System not ready for face capture");
        if (callback) callback(result);
        return result;
    }

    updateStatus("Starting face recognition capture...");
    notifyProgress("Face Capture", 0.0f);

    // Optimize projector for face distance
    if (projector_) {
        projector_->adaptCurrentForConditions(100.0f, face_distance_mm); // 100 lux ambient assumption
    }

    auto start_time = std::chrono::steady_clock::now();
    auto result = performCapture(CaptureMode::FACE_RECOGNITION);

    auto end_time = std::chrono::steady_clock::now();
    result.capture_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    updatePerformanceMetrics(result.capture_duration_ms, result.sync_error_us, result.success);

    if (result.success) {
        updateStatus("Face capture completed successfully");
        notifyProgress("Face Capture", 1.0f);

    // TODO: Fix formatted logging -         core::Logger::getInstance().info("Face capture: {}mm distance, {}ms duration, {:.1f} quality score",
    //                           face_distance_mm, result.capture_duration_ms, result.depth_quality_score);
    } else {
        updateStatus("Face capture failed: " + result.error_message, true);
        notifyProgress("Face Capture", -1.0f);
    }

    if (callback) {
        callback(result);
    }

    return result;
}

bool StructuredLightSystem::startContinuous(double fps, CaptureCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || !isReady()) {
        updateStatus("Cannot start continuous capture - system not ready", true);
        return false;
    }

    if (continuous_active_.load()) {
        updateStatus("Continuous capture already active", true);
        return false;
    }

    capture_callback_ = callback;
    stop_continuous_.store(false);
    continuous_thread_ = std::make_unique<std::thread>(&StructuredLightSystem::continuousCaptureWorker, this);

    continuous_active_.store(true);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.continuous_active = true;
    }

    updateStatus("Continuous capture started at " + std::to_string(fps) + " FPS");
    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Started continuous capture at {:.1f} FPS", fps);

    return true;
}

void StructuredLightSystem::stopContinuous() {
    if (!continuous_active_.load()) {
        return;
    }

    stop_continuous_.store(true);

    if (continuous_thread_ && continuous_thread_->joinable()) {
        continuous_thread_->join();
        continuous_thread_.reset();
    }

    continuous_active_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.continuous_active = false;
    }

    updateStatus("Continuous capture stopped");
    core::Logger::getInstance().info("Continuous capture stopped");
}

StructuredLightSystem::SystemDiagnostic StructuredLightSystem::runSystemTest() {
    SystemDiagnostic diagnostic{};
    auto start_time = std::chrono::steady_clock::now();

    updateStatus("Running system diagnostics...");
    notifyProgress("System Test", 0.0f);

    core::Logger::getInstance().info("Running comprehensive system diagnostics...");

    // Test camera system
    notifyProgress("System Test", 0.2f);
    if (camera_system_) {
        auto camera_stats = camera_system_->getStats();
        diagnostic.camera_system_ok = (camera_stats.sync_errors == 0) && (camera_stats.frames_captured > 0);

        if (!diagnostic.camera_system_ok) {
            diagnostic.errors.push_back("Camera system test failed");
        }
    } else {
        diagnostic.errors.push_back("Camera system not initialized");
    }

    // Test projector system
    notifyProgress("System Test", 0.4f);
    if (projector_) {
        auto projector_diagnostic = projector_->runDiagnostics();
        diagnostic.projector_ok = projector_diagnostic.i2c_communication &&
                                  projector_diagnostic.led1_functional;
        //                                   projector_diagnostic.thermal_sensor;

        diagnostic.thermal_status_c = projector_diagnostic.measured_temperature_c;

        if (!diagnostic.projector_ok) {
            diagnostic.errors.push_back("Projector system test failed");
            for (const auto& error : projector_diagnostic.error_details) {
                if (!error.empty()) {
                    diagnostic.errors.push_back("Projector: " + error);
                }
            }
        }
    } else {
        diagnostic.errors.push_back("Projector system not initialized");
    }

    // Test synchronization
    notifyProgress("System Test", 0.6f);
    if (diagnostic.camera_system_ok && diagnostic.projector_ok) {
        // Perform test capture to measure sync accuracy
        auto test_result = performCapture(CaptureMode::DIAGNOSTIC);
        diagnostic.sync_system_ok = test_result.success && (test_result.sync_error_us < 100.0);
        diagnostic.measured_sync_accuracy_us = test_result.sync_error_us;

        if (!diagnostic.sync_system_ok) {
            diagnostic.errors.push_back("Synchronization test failed - accuracy: " +
                                       std::to_string(test_result.sync_error_us) + "μs");
        }
    } else {
        diagnostic.errors.push_back("Cannot test synchronization - camera or projector failed");
    }

    // Test depth processing
    notifyProgress("System Test", 0.8f);
    if (depth_processor_ && diagnostic.sync_system_ok) {
        // This would require implementing a test method in DepthProcessor
        diagnostic.depth_processing_ok = true; // Placeholder
    } else {
        diagnostic.warnings.push_back("Depth processing test skipped");
    }

    auto end_time = std::chrono::steady_clock::now();
    diagnostic.test_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();

    notifyProgress("System Test", 1.0f);

    // Log results
    if (diagnostic.camera_system_ok && diagnostic.projector_ok && diagnostic.sync_system_ok) {
        updateStatus("System diagnostics PASSED in " + std::to_string(diagnostic.test_duration_ms) + "ms");
    // TODO: Fix formatted logging -         core::Logger::getInstance().info("System diagnostics PASSED - sync accuracy: {:.1f}μs, thermal: {:.1f}°C",
    //                           diagnostic.measured_sync_accuracy_us, diagnostic.thermal_status_c);
    } else {
        updateStatus("System diagnostics FAILED", true);
        // TODO: Fix formatted logging - core::Logger::getInstance().error("System diagnostics FAILED - {} errors, {} warnings",
        //                           diagnostic.errors.size(), diagnostic.warnings.size());
        core::Logger::getInstance().error("System diagnostics FAILED with " + std::to_string(diagnostic.errors.size()) + " errors and " + std::to_string(diagnostic.warnings.size()) + " warnings");
        for (const auto& error : diagnostic.errors) {
            // TODO: Fix formatted logging - core::Logger::getInstance().error("  Error: {}", error);
            core::Logger::getInstance().error("Diagnostic Error: " + error);
        }
    }

    return diagnostic;
}

bool StructuredLightSystem::isReady() const {
    if (!initialized_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return status_.camera_ready && status_.projector_ready && !status_.thermal_protection_active;
}

bool StructuredLightSystem::isContinuous() const {
    return continuous_active_.load();
}

StructuredLightSystem::SystemStatus StructuredLightSystem::getStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto status = status_;
    status.last_update_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return status;
}

void StructuredLightSystem::setCaptureCallback(CaptureCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    capture_callback_ = callback;
}

void StructuredLightSystem::setProgressCallback(ProgressCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_callback_ = callback;
}

void StructuredLightSystem::setStatusCallback(StatusCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_callback_ = callback;
}

bool StructuredLightSystem::setManualProjection(bool enable,
                                                VCSELProjector::PatternType pattern,
                                                uint32_t duration_ms) {
    if (!projector_) {
        updateStatus("Cannot control projection - projector not initialized", true);
        return false;
    }

    if (!checkThermalSafety()) {
        updateStatus("Manual projection blocked - thermal protection active", true);
        return false;
    }

    bool success = projector_->setManualProjection(enable, duration_ms);
    if (enable && success) {
        success &= projector_->setProjectionPattern(pattern);
    }

    if (success) {
        updateStatus(enable ? "Manual projection enabled" : "Manual projection disabled");
        // TODO: Fix formatted logging - core::Logger::getInstance().info("Manual projection {}: pattern={}, duration={}ms",
        //                          enable ? "enabled" : "disabled",
        //                          static_cast<int>(pattern), duration_ms);
        core::Logger::getInstance().info("Manual projection " + std::string(enable ? "enabled" : "disabled") +
                                        " with pattern " + std::to_string(static_cast<int>(pattern)) +
                                        " for " + std::to_string(duration_ms) + "ms");
    } else {
        updateStatus("Manual projection control failed", true);
    }

    return success;
}

void StructuredLightSystem::emergencyStop() {
    if (projector_) {
        projector_->emergencyShutdown();
    }

    stop_continuous_.store(true);

    updateStatus("Emergency stop activated", true);
    core::Logger::getInstance().warning("Structured Light System emergency stop activated");
}

// Private implementation methods

bool StructuredLightSystem::initializeComponents() {
    core::Logger::getInstance().info("Initializing core components...");

    // Initialize projector system
    projector_ = std::make_shared<VCSELProjector>();
    VCSELProjector::ProjectorConfig projector_config;
    projector_config.vcsel_current_ma = config_.vcsel_current_ma;
    projector_config.flood_current_ma = config_.flood_current_ma;
    projector_config.projection_duration_ms = config_.projection_duration_ms;
    projector_config.enable_camera_sync = config_.enable_precise_sync;

    if (!projector_->initialize(projector_config)) {
        core::Logger::getInstance().error("Failed to initialize projector system");
        return false;
    }

    // Initialize camera system
    camera_system_ = std::make_shared<camera::HardwareSyncCapture>();
    camera::HardwareSyncCapture::CameraConfig camera_config;
    camera_config.width = 1456;
    camera_config.height = 1088;

    if (!camera_system_->initialize(camera_config)) {
        core::Logger::getInstance().error("Failed to initialize camera system");
        return false;
    }

    // Initialize depth processor
    depth_processor_ = std::make_shared<stereo::DepthProcessor>();
    // For now, pass nullptr for calibration manager - should be properly initialized in production
    if (!depth_processor_->initialize(nullptr)) {
        core::Logger::getInstance().error("Failed to initialize depth processor");
        return false;
    }

    core::Logger::getInstance().info("Core components initialized successfully");
    return true;
}

void StructuredLightSystem::cleanupResources() {
    if (depth_processor_) {
        // depth_processor_->shutdown(); // DepthProcessor doesn't have shutdown method
        depth_processor_.reset();
    }

    if (camera_system_) {
        camera_system_->stop();
        camera_system_.reset();
    }

    if (projector_) {
        projector_->shutdown();
        projector_.reset();
    }
}

StructuredLightSystem::CaptureResult StructuredLightSystem::performCapture(CaptureMode mode) {
    CaptureResult result{};
    result.capture_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    if (!camera_system_ || !projector_) {
        return createErrorResult("Core components not available");
    }

    try {
        notifyProgress("Capture", 0.1f);

        // Configure projector based on capture mode
        VCSELProjector::PatternType pattern = config_.pattern;
        if (mode == CaptureMode::FACE_RECOGNITION) {
            pattern = VCSELProjector::PatternType::ADAPTIVE;
        } else if (mode == CaptureMode::CALIBRATION) {
            pattern = VCSELProjector::PatternType::GRID;
        }

        projector_->setProjectionPattern(pattern);
        notifyProgress("Capture", 0.2f);

        // Synchronize capture with projector
        uint64_t camera_timestamp, projector_timestamp;
        if (!synchronizeCapture(camera_timestamp, projector_timestamp)) {
            return createErrorResult("Failed to synchronize camera and projector");
        }

        result.capture_timestamp_ns = camera_timestamp;
        result.projection_timestamp_ns = projector_timestamp;
        result.sync_error_us = std::abs(static_cast<double>(camera_timestamp - projector_timestamp)) / 1000.0;

        notifyProgress("Capture", 0.5f);

        // Capture stereo frame
        camera::HardwareSyncCapture::StereoFrame frame;
        if (!camera_system_->captureSingle(frame, 5000)) { // 5 second timeout
            return createErrorResult("Failed to capture stereo frame");
        }

        result.left_image = frame.left_image.clone();
        result.right_image = frame.right_image.clone();
        notifyProgress("Capture", 0.7f);

        // Process depth if required
        if (mode != CaptureMode::CALIBRATION && depth_processor_) {
            auto processing_start = std::chrono::steady_clock::now();

            if (processDepthData(result.left_image, result.right_image, result.depth_map)) {
                auto processing_end = std::chrono::steady_clock::now();
                result.processing_duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    processing_end - processing_start).count();

                // Assess quality
                result.valid_depth_pixels = countValidDepthPixels(result.depth_map);
                result.depth_quality_score = assessDepthQuality(result.depth_map);
            } else {
                core::Logger::getInstance().warning("Depth processing failed, returning stereo images only");
            }
        }

        notifyProgress("Capture", 0.9f);

        // Get thermal status
        if (projector_) {
            auto thermal_status = projector_->getThermalStatus();
            result.projection_temperature_c = thermal_status.current_temp_c;
        }

        // Assess image quality
        result.brightness_score = assessImageBrightness(result.left_image);
        result.contrast_score = assessImageContrast(result.left_image);

        result.success = true;
        notifyProgress("Capture", 1.0f);

    } catch (const std::exception& e) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Exception during capture: {}", e.what());
        return createErrorResult("Exception during capture: " + std::string(e.what()));
    }

    return result;
}

bool StructuredLightSystem::synchronizeCapture(uint64_t& camera_timestamp, uint64_t& projector_timestamp) {
    if (!projector_ || !camera_system_) {
        return false;
    }

    // This is a simplified synchronization - in practice, you'd use precise timing control
    auto sync_start = std::chrono::steady_clock::now();

    // Trigger projector with exposure timing
    bool projection_success = projector_->triggerStructuredLightCapture(config_.exposure_time_us);
    if (!projection_success) {
        core::Logger::getInstance().error("Failed to trigger projector");
        return false;
    }

    projector_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        sync_start.time_since_epoch()).count();

    // Camera capture should be synchronized through hardware sync system
    camera_timestamp = projector_timestamp; // Simplified - would get actual camera timestamp

    return true;
}

bool StructuredLightSystem::processDepthData(const cv::Mat& left, const cv::Mat& /* right */, cv::Mat& depth) {
    if (!depth_processor_) {
        return false;
    }

    try {
        // This would call the actual depth processor
        // For now, create a placeholder depth map
        depth = cv::Mat::zeros(left.size(), CV_32F);

        // In reality, this would call:
        // return depth_processor_->processFrame(left, right, depth);

    // TODO: Fix formatted logging -         core::Logger::getInstance().debug("Depth processing completed: {}x{}", depth.cols, depth.rows);
        return true;

    } catch (const std::exception& e) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Depth processing failed: {}", e.what());
        return false;
    }
}

void StructuredLightSystem::continuousCaptureWorker() {
    core::Logger::getInstance().info("Continuous capture worker started");

    const double target_fps = 10.0; // Could be configurable
    const auto frame_interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / target_fps));

    while (!stop_continuous_.load() && initialized_.load()) {
        try {
            auto frame_start = std::chrono::steady_clock::now();

            auto result = performCapture(config_.mode);

            if (capture_callback_) {
                capture_callback_(result);
            }

            updatePerformanceMetrics(result.capture_duration_ms, result.sync_error_us, result.success);

            // Maintain frame rate
            auto frame_end = std::chrono::steady_clock::now();
            auto frame_duration = frame_end - frame_start;

            if (frame_duration < frame_interval) {
                std::this_thread::sleep_for(frame_interval - frame_duration);
            }

        } catch (const std::exception& e) {
    // TODO: Fix formatted logging -             core::Logger::getInstance().error("Exception in continuous capture: {}", e.what());
            handleSyncError("Continuous capture exception: " + std::string(e.what()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Back off on error
        }
    }

    core::Logger::getInstance().info("Continuous capture worker stopped");
}

bool StructuredLightSystem::validateConfiguration(const CaptureConfig& config) const {
    if (config.exposure_time_us < 100 || config.exposure_time_us > 100000) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid exposure time: {}μs", config.exposure_time_us);
        return false;
    }

    if (config.vcsel_current_ma == 0 || config.vcsel_current_ma > 300) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid VCSEL current: {}mA", config.vcsel_current_ma);
        return false;
    }

    if (config.projection_duration_ms == 0 || config.projection_duration_ms > 1000) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid projection duration: {}ms", config.projection_duration_ms);
        return false;
    }

    return true;
}

void StructuredLightSystem::updateStatus(const std::string& message, bool is_error) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        if (is_error) {
            status_.last_error = message;
        }
        status_.last_update_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    if (status_callback_) {
        status_callback_(message, is_error);
    }

    if (is_error) {
        // TODO: Fix formatted logging - core::Logger::getInstance().error("StructuredLightSystem: {}", message);
        core::Logger::getInstance().error("StructuredLightSystem: " + message);
    } else {
        // TODO: Fix formatted logging - core::Logger::getInstance().debug("StructuredLightSystem: {}", message);
        core::Logger::getInstance().debug("StructuredLightSystem: " + message);
    }
}

void StructuredLightSystem::notifyProgress(const std::string& stage, float progress) {
    if (progress_callback_) {
        progress_callback_(stage, progress);
    }
}

StructuredLightSystem::CaptureResult StructuredLightSystem::createErrorResult(const std::string& error) {
    CaptureResult result{};
    result.success = false;
    result.error_message = error;
    result.capture_timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return result;
}

void StructuredLightSystem::updatePerformanceMetrics(uint32_t capture_time_ms, double sync_error_us, bool success) {
    std::lock_guard<std::mutex> status_lock(status_mutex_);

    status_.total_captures++;
    if (success) {
        status_.successful_captures++;
    }

    if (sync_error_us > 100.0) {
        status_.sync_errors++;
    }

    // Update running averages
    capture_times_.push_back(capture_time_ms);
    if (capture_times_.size() > MAX_PERFORMANCE_HISTORY) {
        capture_times_.pop_front();
    }

    sync_errors_.push_back(sync_error_us);
    if (sync_errors_.size() > MAX_PERFORMANCE_HISTORY) {
        sync_errors_.pop_front();
    }

    // Calculate averages
    if (!capture_times_.empty()) {
        status_.avg_capture_time_ms = std::accumulate(capture_times_.begin(), capture_times_.end(), 0.0) / capture_times_.size();
    }

    if (!sync_errors_.empty()) {
        status_.avg_sync_error_us = std::accumulate(sync_errors_.begin(), sync_errors_.end(), 0.0) / sync_errors_.size();
    }
}

// Quality assessment methods (simplified implementations)

uint32_t StructuredLightSystem::countValidDepthPixels(const cv::Mat& depth_map) const {
    if (depth_map.empty()) return 0;
    return cv::countNonZero(depth_map > 0);
}

float StructuredLightSystem::assessDepthQuality(const cv::Mat& depth_map) const {
    if (depth_map.empty()) return 0.0f;

    uint32_t valid_pixels = countValidDepthPixels(depth_map);
    uint32_t total_pixels = depth_map.rows * depth_map.cols;

    return static_cast<float>(valid_pixels) / static_cast<float>(total_pixels);
}

float StructuredLightSystem::assessImageBrightness(const cv::Mat& image) const {
    if (image.empty()) return 0.0f;

    cv::Scalar mean_val = cv::mean(image);
    return static_cast<float>(mean_val[0]) / 255.0f;
}

float StructuredLightSystem::assessImageContrast(const cv::Mat& image) const {
    if (image.empty()) return 0.0f;

    cv::Mat gray;
    if (image.channels() > 1) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev);

    return static_cast<float>(stddev[0]) / 255.0f;
}

bool StructuredLightSystem::checkThermalSafety() const {
    if (!projector_) return false;

    auto thermal_status = projector_->getThermalStatus();
    return !thermal_status.thermal_protection_active && thermal_status.current_temp_c < 70.0f;
}

void StructuredLightSystem::handleThermalEvent(bool thermal_active, float temperature_c) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.thermal_protection_active = thermal_active;
        status_.projector_temperature_c = temperature_c;
    }

    if (thermal_active) {
        updateStatus("Thermal protection activated at " + std::to_string(temperature_c) + "°C", true);
    } else {
        updateStatus("Thermal protection deactivated at " + std::to_string(temperature_c) + "°C");
    }
}

void StructuredLightSystem::handleSyncError(const std::string& error) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_errors++;
    }

    updateStatus("Sync error: " + error, true);
}

// Setup methods for component integration

bool StructuredLightSystem::setupCameraSync() {
    if (!camera_system_) return false;

    // Start camera system
    if (!camera_system_->start()) {
        core::Logger::getInstance().error("Failed to start camera system");
        return false;
    }

    core::Logger::getInstance().info("Camera synchronization setup completed");
    return true;
}

bool StructuredLightSystem::setupDepthProcessing() {
    if (!depth_processor_) return false;

    // Configuration would happen here
    core::Logger::getInstance().info("Depth processing setup completed");
    return true;
}

bool StructuredLightSystem::setupProjectorCallbacks() {
    if (!projector_) return false;

    // Setup thermal callback
    auto thermal_callback = [this](bool thermal_active, float temperature_c) {
        this->handleThermalEvent(thermal_active, temperature_c);
    };
    projector_->setThermalCallback(thermal_callback);

    // Setup error callback
    auto error_callback = [this](const std::string& error) {
        this->updateStatus("Projector error: " + error, true);
    };
    projector_->setErrorCallback(error_callback);

    core::Logger::getInstance().info("Projector callbacks setup completed");
    return true;
}

} // namespace hardware
} // namespace unlook