#include <unlook/hardware/VCSELProjector.hpp>
#include <unlook/core/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <thread>
#include <chrono>

namespace unlook {
namespace hardware {

VCSELProjector::VCSELProjector() {
    initialization_time_ = std::chrono::steady_clock::now();
    duty_cycle_window_start_ = initialization_time_;

    // Initialize pattern library with defaults
    loadPatternLibrary();

    // Initialize performance metrics
    resetPerformanceMetrics();
}

VCSELProjector::~VCSELProjector() {
    shutdown();
}

bool VCSELProjector::initialize(const ProjectorConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("VCSELProjector already initialized");
        return true;
    }

    if (!validateConfiguration(config)) {
        setErrorState("Invalid projector configuration");
        return false;
    }

    config_ = config;

    core::Logger::getInstance().info("Initializing VCSEL Projector System...");
    // TODO: Fix formatted logging - core::Logger::getInstance().info("Mode: {}, Pattern: {}, VCSEL: {}mA, Flood: {}mA",
    //                   static_cast<int>(config_.mode),
    //                   static_cast<int>(config_.pattern),
    //                   config_.vcsel_current_ma,
    //                   config_.flood_current_ma);

    // Initialize hardware components
    if (!initializeHardwareComponents()) {
        setErrorState("Failed to initialize hardware components");
        cleanupResources();
        return false;
    }

    // Perform initial self-test
    if (!performSelfTest()) {
        setErrorState("Self-test failed");
        cleanupResources();
        return false;
    }

    initialized_.store(true);
    clearErrorState();

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = true;
        status_.current_mode = ProjectionMode::DISABLED;
        status_.current_pattern = PatternType::NONE;
        status_.hardware_ok = true;
        status_.sync_ok = true;
        status_.thermal_ok = true;
        status_.last_update = std::chrono::steady_clock::now();
    }

    core::Logger::getInstance().info("VCSEL Projector System initialized successfully");
    return true;
}

void VCSELProjector::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return;
    }

    core::Logger::getInstance().info("Shutting down VCSEL Projector System...");

    disableProjection();
    emergencyShutdown();
    cleanupResources();

    initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.projection_active = false;
        status_.current_mode = ProjectionMode::DISABLED;
        status_.hardware_ok = false;
        status_.sync_ok = false;
        status_.thermal_ok = false;
    }

    core::Logger::getInstance().info("VCSEL Projector System shutdown complete");
}

bool VCSELProjector::enableDepthCapture(PatternType pattern) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        setErrorState("Projector not initialized");
        return false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Enabling depth capture mode with pattern: {}", static_cast<int>(pattern));

    config_.mode = ProjectionMode::DEPTH_CAPTURE;
    config_.pattern = pattern;

    if (!configureForDepthCapture()) {
        setErrorState("Failed to configure for depth capture");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_mode = ProjectionMode::DEPTH_CAPTURE;
        status_.current_pattern = pattern;
    }

    updateStatus();
    core::Logger::getInstance().info("Depth capture mode enabled successfully");
    return true;
}

bool VCSELProjector::enableFaceRecognition(float distance_mm) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        setErrorState("Projector not initialized");
        return false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Enabling face recognition mode at {:.0f}mm distance", distance_mm);

    config_.mode = ProjectionMode::FACE_RECOGNITION;
    config_.face_distance_mm = distance_mm;

    if (!configureForFaceRecognition()) {
        setErrorState("Failed to configure for face recognition");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_mode = ProjectionMode::FACE_RECOGNITION;
        status_.current_pattern = PatternType::ADAPTIVE;
    }

    updateStatus();
    core::Logger::getInstance().info("Face recognition mode enabled successfully");
    return true;
}

bool VCSELProjector::enableStructuredLight(PatternType pattern) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        setErrorState("Projector not initialized");
        return false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Enabling structured light mode with pattern: {}", static_cast<int>(pattern));

    config_.mode = ProjectionMode::STRUCTURED_LIGHT;
    config_.pattern = pattern;

    if (!configureForStructuredLight()) {
        setErrorState("Failed to configure for structured light");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_mode = ProjectionMode::STRUCTURED_LIGHT;
        status_.current_pattern = pattern;
    }

    updateStatus();
    core::Logger::getInstance().info("Structured light mode enabled successfully");
    return true;
}

bool VCSELProjector::disableProjection() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return true; // Already disabled
    }

    core::Logger::getInstance().info("Disabling projection");

    projection_active_.store(false);

    // Stop sync manager if active
    if (sync_manager_) {
        sync_manager_->stopSync();
    }

    // Turn off LEDs
    deactivateCurrentPattern();

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.projection_active = false;
        status_.current_mode = ProjectionMode::DISABLED;
        status_.current_pattern = PatternType::NONE;
        status_.vcsel_current_ma = 0;
        status_.flood_current_ma = 0;
    }

    updateDutyCycleTracking(false);
    updateStatus();

    core::Logger::getInstance().info("Projection disabled");
    return true;
}

bool VCSELProjector::triggerStructuredLightCapture(uint32_t exposure_time_us,
                                                  ProjectionCallback callback) {
    if (!initialized_.load() || !isReady()) {
        core::Logger::getInstance().error("Projector not ready for capture");
        return false;
    }

    if (!checkSafetyLimits()) {
        core::Logger::getInstance().error("Safety limits exceeded - capture blocked");
        return false;
    }

    // Check thermal protection
    if (thermal_manager_) {
        uint16_t approved_current = thermal_manager_->checkThermalProtection(
                     config_.vcsel_current_ma, config_.projection_duration_ms);

        if (approved_current < config_.vcsel_current_ma) {
    // TODO: Fix formatted logging -             core::Logger::getInstance().warning("Thermal protection active - current reduced to {}mA", approved_current);
        }
    }

    // Trigger synchronized capture via sync manager
    if (sync_manager_) {
        auto sync_callback = [this, callback](uint64_t timestamp_ns) {
            this->updatePerformanceMetrics(0.0, true); // Successful sync
            this->updateStatus();

            if (callback) {
                callback(this->status_.current_pattern, timestamp_ns);
            }

            if (this->sync_callback_) {
                this->sync_callback_(timestamp_ns, timestamp_ns);
            }
        };

        if (!sync_manager_->triggerSyncCapture(exposure_time_us, sync_callback)) {
            setErrorState("Failed to trigger synchronized capture");
            return false;
        }
    } else {
        // Fallback to manual projection
        return setManualProjection(true, config_.projection_duration_ms);
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_captures++;
        status_.last_projection = std::chrono::steady_clock::now();
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Structured light capture triggered: exposure={}μs", exposure_time_us);
    return true;
}

bool VCSELProjector::triggerFaceIllumination(uint32_t duration_ms,
                                            ProjectionCallback callback) {
    if (!initialized_.load() || !isReady()) {
        return false;
    }

    if (!checkSafetyLimits()) {
        core::Logger::getInstance().warning("Safety limits exceeded for face illumination");
        return false;
    }

    // Use flood illumination for face recognition
    auto start_time = std::chrono::steady_clock::now();

    if (!activatePattern(PatternType::NONE, duration_ms)) {
        setErrorState("Failed to activate face illumination");
        return false;
    }

    // Schedule deactivation
    std::thread deactivate_thread([this, duration_ms, callback, start_time]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
        this->deactivateCurrentPattern();

        if (callback) {
            uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::steady_clock::now().time_since_epoch()).count();
            callback(PatternType::NONE, timestamp_ns);
        }
    });
    deactivate_thread.detach();

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.projection_cycles++;
        status_.last_projection = start_time;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Face illumination triggered: duration={}ms", duration_ms);
    return true;
}

bool VCSELProjector::setProjectionPattern(PatternType pattern,
                                         uint16_t vcsel_current_ma,
                                         uint16_t flood_current_ma) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return false;
    }

    config_.pattern = pattern;

    if (vcsel_current_ma > 0) {
        config_.vcsel_current_ma = vcsel_current_ma;
    }
    if (flood_current_ma > 0) {
        config_.flood_current_ma = flood_current_ma;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_pattern = pattern;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Projection pattern set: {}, VCSEL={}mA, Flood={}mA",
    //                       static_cast<int>(pattern), config_.vcsel_current_ma, config_.flood_current_ma);
    return true;
}

bool VCSELProjector::setManualProjection(bool enable, uint32_t duration_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Manual projection: {}, duration={}ms", enable, duration_ms);

    if (enable) {
        if (!checkSafetyLimits()) {
            core::Logger::getInstance().warning("Safety limits exceeded - manual projection blocked");
            return false;
        }

        updateDutyCycleTracking(true);

        if (!activatePattern(config_.pattern, duration_ms)) {
            setErrorState("Failed to activate manual projection");
            return false;
        }

        projection_active_.store(true);

        if (duration_ms > 0) {
            // Schedule automatic deactivation
            std::thread deactivate_thread([this, duration_ms]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
                this->setManualProjection(false);
            });
            deactivate_thread.detach();
        }
    } else {
        deactivateCurrentPattern();
        projection_active_.store(false);
        updateDutyCycleTracking(false);
    }

    updateStatus();
    return true;
}

bool VCSELProjector::configureCameraSync(bool enable, uint32_t tolerance_us) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!sync_manager_) {
        setErrorState("Sync manager not available");
        return false;
    }

    config_.enable_camera_sync = enable;
    config_.sync_tolerance_us = tolerance_us;

    // Update sync manager configuration
    auto sync_config = sync_manager_->getConfig();
    sync_config.sync_tolerance_us = tolerance_us;

    if (!sync_manager_->configureSynchronization(sync_config)) {
        setErrorState("Failed to configure camera synchronization");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.sync_ok = true;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Camera sync configured: enabled={}, tolerance={}μs", enable, tolerance_us);
    return true;
}

bool VCSELProjector::adaptCurrentForConditions(float ambient_light_lux, float target_distance_mm) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!config_.adaptive_current) {
        return true; // Adaptive current disabled
    }

    // Calculate optimal current based on conditions
    uint16_t optimal_vcsel = calculateOptimalCurrent(config_.pattern, target_distance_mm, ambient_light_lux);
    uint16_t optimal_flood = static_cast<uint16_t>(optimal_vcsel * 0.6f); // Flood typically 60% of VCSEL

    // Apply limits
    optimal_vcsel = std::min(optimal_vcsel, config_.vcsel_current_ma);
    optimal_flood = std::min(optimal_flood, config_.flood_current_ma);

    config_.vcsel_current_ma = optimal_vcsel;
    config_.flood_current_ma = optimal_flood;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.vcsel_current_ma = optimal_vcsel;
        status_.flood_current_ma = optimal_flood;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Current adapted: ambient={:.1f}lux, distance={:.0f}mm, VCSEL={}mA, Flood={}mA",
    //                        ambient_light_lux, target_distance_mm, optimal_vcsel, optimal_flood);

    return true;
}

void VCSELProjector::emergencyShutdown() {
    projection_active_.store(false);

    // Emergency shutdown all hardware components
    if (as1170_controller_) {
        as1170_controller_->emergencyShutdown();
    }

    if (sync_manager_) {
        sync_manager_->emergencyStop();
    }

    if (thermal_manager_) {
        thermal_manager_->emergencyThermalShutdown();
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.projection_active = false;
        status_.current_mode = ProjectionMode::DISABLED;
        status_.vcsel_current_ma = 0;
        status_.flood_current_ma = 0;
        status_.hardware_ok = false;
    }

    core::Logger::getInstance().error("VCSEL Projector emergency shutdown activated!");
}

bool VCSELProjector::performSelfTest() {
    core::Logger::getInstance().info("Performing VCSEL projector self-test...");

    DiagnosticResult result = runDiagnostics();

    bool overall_pass = result.i2c_communication &&
                       result.gpio_control &&
                       result.led1_functional &&
                       result.led2_functional &&
                       result.thermal_sensor &&
                       result.sync_timing;

    if (overall_pass) {
        core::Logger::getInstance().info("Self-test PASSED");
    // TODO: Fix formatted logging -         core::Logger::getInstance().info("LED1: {:.1f}mA, LED2: {:.1f}mA, Temp: {:.1f}°C",
    //                           result.measured_current_led1_ma,
    //                           result.measured_current_led2_ma,
    //                           result.measured_temperature_c);
    } else {
        core::Logger::getInstance().error("Self-test FAILED");
        for (const auto& error : result.error_details) {
    // TODO: Fix formatted logging -             core::Logger::getInstance().error("  - {}", error);
        }
    }

    return overall_pass;
}

VCSELProjector::ProjectorStatus VCSELProjector::getStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto status = status_;
    status.last_update = std::chrono::steady_clock::now();
    return status;
}

VCSELProjector::ProjectorConfig VCSELProjector::getConfig() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

bool VCSELProjector::isReady() const {
    if (!initialized_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return status_.initialized &&
           status_.hardware_ok &&
           status_.sync_ok &&
           status_.thermal_ok &&
           !status_.thermal_protection_active;
}

bool VCSELProjector::isProjectionActive() const {
    return projection_active_.load();
}

AS1170Controller::ThermalStatus VCSELProjector::getThermalStatus() const {
    if (as1170_controller_) {
        return as1170_controller_->getThermalStatus();
    }
    return AS1170Controller::ThermalStatus{};
}

void VCSELProjector::setProjectionCallback(ProjectionCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    projection_callback_ = callback;
}

void VCSELProjector::setThermalCallback(ThermalCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    thermal_callback_ = callback;
}

void VCSELProjector::setErrorCallback(ErrorCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    error_callback_ = callback;
}

void VCSELProjector::setSyncCallback(SyncCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    sync_callback_ = callback;
}

bool VCSELProjector::setAdvancedPattern(const PatternParams& params) {
    std::lock_guard<std::mutex> lock(mutex_);

    current_pattern_params_ = params;

    // Update pattern library with custom parameters
    pattern_library_[PatternType::CUSTOM] = params;

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Advanced pattern configured: intensity={:.2f}, freq={}Hz",
    //                        params.intensity, params.frequency_hz);

    return true;
}

VCSELProjector::PerformanceMetrics VCSELProjector::getPerformanceMetrics() const {
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    return performance_metrics_;
}

bool VCSELProjector::calibrateProjectionIntensity(float target_distance_mm) {
    if (!initialized_.load()) {
        return false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Calibrating projection intensity for {:.0f}mm distance", target_distance_mm);

    // Measurement-based calibration would require light sensors
    // For now, use distance-based estimation
    uint16_t optimal_current = calculateOptimalCurrent(config_.pattern, target_distance_mm, 100.0f);

    config_.vcsel_current_ma = optimal_current;

    // TODO: Fix formatted logging -     core::Logger::getInstance().info("Projection intensity calibrated: {}mA for {:.0f}mm",
    //                       optimal_current, target_distance_mm);

    return true;
}

bool VCSELProjector::optimizeForDepthAccuracy() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().info("Optimizing for depth accuracy");

    // Optimize pattern and current for maximum depth precision
    config_.pattern = PatternType::DOTS_15K;
    config_.vcsel_current_ma = std::min(config_.vcsel_current_ma, static_cast<uint16_t>(250)); // Safety limit
    config_.projection_duration_ms = 50; // Optimal for depth capture

    return configureForDepthCapture();
}

bool VCSELProjector::optimizeForFaceRecognition() {
    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().info("Optimizing for face recognition");

    // Optimize for facial feature enhancement
    config_.pattern = PatternType::ADAPTIVE;
    config_.flood_current_ma = 150; // Gentle illumination
    config_.vcsel_current_ma = 200; // Moderate structured light

    return configureForFaceRecognition();
}

VCSELProjector::DiagnosticResult VCSELProjector::runDiagnostics() {
    DiagnosticResult result{};

    core::Logger::getInstance().info("Running comprehensive diagnostics...");

    // Test I2C communication
    result.i2c_communication = testI2CCommunication();
    if (!result.i2c_communication) {
        result.error_details.push_back("I2C communication test failed");
    }

    // Test GPIO control
    result.gpio_control = testGPIOControl();
    if (!result.gpio_control) {
        result.error_details.push_back("GPIO control test failed");
    }

    // Test LED functionality
    result.led1_functional = testLEDFunctionality(AS1170Controller::LEDChannel::LED1);
    if (!result.led1_functional) {
        result.error_details.push_back("LED1 (VCSEL) functionality test failed");
    }

    result.led2_functional = testLEDFunctionality(AS1170Controller::LEDChannel::LED2);
    if (!result.led2_functional) {
        result.error_details.push_back("LED2 (Flood) functionality test failed");
    }

    // Test thermal sensor
    result.thermal_sensor = testThermalSensor();
    if (!result.thermal_sensor) {
        result.error_details.push_back("Thermal sensor test failed");
    } else if (as1170_controller_) {
        result.measured_temperature_c = as1170_controller_->readTemperature();
    }

    // Test sync timing
    result.sync_timing = testSyncTiming();
    if (!result.sync_timing) {
        result.error_details.push_back("Sync timing test failed");
    }

    // Measure LED currents (if available)
    if (result.led1_functional && as1170_controller_) {
        auto status = as1170_controller_->getStatus();
        result.measured_current_led1_ma = status.led1_current_ma;
    }

    if (result.led2_functional && as1170_controller_) {
        auto status = as1170_controller_->getStatus();
        result.measured_current_led2_ma = status.led2_current_ma;
    }

    return result;
}

// Private implementation methods

bool VCSELProjector::initializeHardwareComponents() {
    core::Logger::getInstance().info("Initializing hardware components...");

    // Initialize AS1170 controller using singleton to prevent I2C conflicts
    as1170_controller_ = AS1170Controller::getInstance();
    AS1170Controller::AS1170Config as1170_config;
    as1170_config.target_current_ma = std::max(config_.vcsel_current_ma, config_.flood_current_ma);

    if (!as1170_controller_->initialize(as1170_config)) {
        core::Logger::getInstance().error("Failed to initialize AS1170 controller");
        return false;
    }

    // Initialize thermal manager
    thermal_manager_ = std::make_shared<LEDThermalManager>();
    LEDThermalManager::ThermalConfig thermal_config;
    thermal_config.enable_predictive_throttling = config_.enable_thermal_protection;
    thermal_config.hot_threshold_c = config_.thermal_throttle_temp_c;
    thermal_config.critical_threshold_c = config_.max_temperature_c;

    if (!thermal_manager_->initialize(as1170_controller_, thermal_config)) {
        core::Logger::getInstance().error("Failed to initialize thermal manager");
        return false;
    }

    // Start thermal monitoring
    if (!thermal_manager_->startMonitoring()) {
        core::Logger::getInstance().error("Failed to start thermal monitoring");
        return false;
    }

    // Set thermal callback
    auto thermal_cb = [this](LEDThermalManager::ThermalState state, float temperature_c) {
        bool thermal_active = (state >= LEDThermalManager::ThermalState::HOT);
        this->handleThermalEvent(thermal_active, temperature_c);
    };
    thermal_manager_->setThermalCallback(thermal_cb);

    // Initialize sync manager
    sync_manager_ = std::make_shared<LEDSyncManager>();
    LEDSyncManager::SyncConfig sync_config;
    sync_config.vcsel_current_ma = config_.vcsel_current_ma;
    sync_config.flood_current_ma = config_.flood_current_ma;
    sync_config.strobe_duration_us = config_.projection_duration_ms * 1000;

    if (!sync_manager_->initialize(as1170_controller_, sync_config)) {
        core::Logger::getInstance().error("Failed to initialize sync manager");
        return false;
    }

    core::Logger::getInstance().info("Hardware components initialized successfully");
    return true;
}

bool VCSELProjector::validateConfiguration(const ProjectorConfig& config) const {
    if (config.vcsel_current_ma == 0 || config.vcsel_current_ma > 300) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid VCSEL current: {}mA", config.vcsel_current_ma);
        return false;
    }

    if (config.flood_current_ma > 250) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid flood current: {}mA", config.flood_current_ma);
        return false;
    }

    if (config.projection_duration_ms == 0 || config.projection_duration_ms > 10000) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid projection duration: {}ms", config.projection_duration_ms);
        return false;
    }

    if (config.max_duty_cycle <= 0.0f || config.max_duty_cycle > 1.0f) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().error("Invalid duty cycle: {:.2f}", config.max_duty_cycle);
        return false;
    }

    return true;
}

void VCSELProjector::cleanupResources() {
    if (sync_manager_) {
        sync_manager_->shutdown();
        sync_manager_.reset();
    }

    if (thermal_manager_) {
        thermal_manager_->shutdown();
        thermal_manager_.reset();
    }

    if (as1170_controller_) {
        as1170_controller_->shutdown();
        as1170_controller_.reset();
    }
}

bool VCSELProjector::loadPatternLibrary() {
    // Initialize default pattern parameters
    PatternParams dots_15k;
    dots_15k.intensity = 1.0f;
    dots_15k.frequency_hz = 60;
    dots_15k.enable_dithering = false;
    pattern_library_[PatternType::DOTS_15K] = dots_15k;

    PatternParams flood;
    flood.intensity = 0.8f;
    flood.frequency_hz = 0; // Continuous
    flood.enable_dithering = false;
    pattern_library_[PatternType::NONE] = flood;

    // Add other patterns...
    pattern_library_[PatternType::GRID] = dots_15k;
    pattern_library_[PatternType::LINES_HORIZONTAL] = dots_15k;
    pattern_library_[PatternType::LINES_VERTICAL] = dots_15k;

    // TODO: Fix formatted logging -     core::Logger::getInstance().debug("Pattern library loaded with {} patterns", pattern_library_.size());
    return true;
}

bool VCSELProjector::activatePattern(PatternType pattern, uint32_t duration_ms) {
    if (!as1170_controller_) {
        return false;
    }

    // Notify thermal manager
    if (thermal_manager_) {
        uint16_t vcsel_current = (pattern == PatternType::NONE) ? 0 : config_.vcsel_current_ma;
        uint16_t flood_current = config_.flood_current_ma;

        thermal_manager_->notifyLEDActivation(vcsel_current, flood_current);

        // Check thermal protection
        vcsel_current = thermal_manager_->checkThermalProtection(vcsel_current, duration_ms);
        flood_current = thermal_manager_->checkThermalProtection(flood_current, duration_ms);
    }

    bool success = false;

    switch (pattern) {
        case PatternType::NONE:
            success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED2,
                                                      true, config_.flood_current_ma);
            break;

        case PatternType::DOTS_15K:
        case PatternType::GRID:
        case PatternType::LINES_HORIZONTAL:
        case PatternType::LINES_VERTICAL:
        case PatternType::RANDOM_SPECKLE:
            success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED1,
                                                      true, config_.vcsel_current_ma);
            if (success && config_.enable_flood_assist) {
                success &= as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED2,
                                                           true, config_.flood_current_ma);
            }
            break;

        case PatternType::ADAPTIVE:
            // Use both LEDs with adaptive current
            success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::BOTH,
                                                      true, config_.vcsel_current_ma);
            break;

        case PatternType::CUSTOM:
            // Custom pattern implementation
            success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::LED1,
                                                      true, config_.vcsel_current_ma);
            break;

        default:
    // TODO: Fix formatted logging -             core::Logger::getInstance().error("Unsupported pattern type: {}", static_cast<int>(pattern));
            return false;
    }

    if (success) {
        last_projection_start_ = std::chrono::steady_clock::now();
        projection_active_.store(true);
        updateDutyCycleTracking(true);

        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.projection_active = true;
            status_.current_pattern = pattern;
            status_.projection_cycles++;
            status_.last_projection = last_projection_start_;

            // Update current display based on pattern
            if (pattern == PatternType::NONE) {
                status_.vcsel_current_ma = 0;
                status_.flood_current_ma = config_.flood_current_ma;
            } else {
                status_.vcsel_current_ma = config_.vcsel_current_ma;
                status_.flood_current_ma = config_.enable_flood_assist ? config_.flood_current_ma : 0;
            }
        }

        // Call projection callback
        if (projection_callback_) {
            uint64_t timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                last_projection_start_.time_since_epoch()).count();
            projection_callback_(pattern, timestamp_ns);
        }
    }

    return success;
}

bool VCSELProjector::deactivateCurrentPattern() {
    if (!as1170_controller_) {
        return false;
    }

    bool success = as1170_controller_->setLEDState(AS1170Controller::LEDChannel::BOTH, false, 0);

    if (success) {
        auto end_time = std::chrono::steady_clock::now();
        auto projection_duration = end_time - last_projection_start_;
        total_projection_time_ += projection_duration;

        projection_active_.store(false);
        updateDutyCycleTracking(false);

        // Notify thermal manager
        if (thermal_manager_) {
            thermal_manager_->notifyLEDDeactivation();
        }

        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.projection_active = false;
            status_.vcsel_current_ma = 0;
            status_.flood_current_ma = 0;
        }
    }

    return success;
}

uint16_t VCSELProjector::calculateOptimalCurrent(PatternType pattern, float distance_mm, float ambient_lux) {
    // Base current calculation based on distance (inverse square law)
    float distance_factor = std::pow(300.0f / distance_mm, 2.0f); // 300mm reference distance

    // Ambient light compensation
    float ambient_factor = 1.0f + (ambient_lux / 500.0f); // 500 lux reference

    // Pattern-specific scaling
    float pattern_factor = 1.0f;
    switch (pattern) {
        case PatternType::DOTS_15K:
            pattern_factor = 1.0f;
            break;
        case PatternType::GRID:
            pattern_factor = 0.8f;
            break;
        case PatternType::NONE:
            pattern_factor = 0.6f;
            break;
        default:
            pattern_factor = 0.9f;
            break;
    }

    uint16_t base_current = 200; // Base current in mA
    uint16_t optimal = static_cast<uint16_t>(
        base_current * distance_factor * ambient_factor * pattern_factor
    );

    // Apply safety limits
    optimal = std::clamp(optimal, static_cast<uint16_t>(50), static_cast<uint16_t>(250));

    return optimal;
}

bool VCSELProjector::switchToMode(ProjectionMode mode) {
    switch (mode) {
        case ProjectionMode::DEPTH_CAPTURE:
            return configureForDepthCapture();
        case ProjectionMode::FACE_RECOGNITION:
            return configureForFaceRecognition();
        case ProjectionMode::STRUCTURED_LIGHT:
            return configureForStructuredLight();
        case ProjectionMode::DISABLED:
            return disableProjection();
        default:
            return false;
    }
}

bool VCSELProjector::configureForDepthCapture() {
    // Optimize for depth capture
    config_.pattern = PatternType::DOTS_15K;
    config_.enable_flood_assist = true;
    config_.projection_duration_ms = 50;
    config_.enable_camera_sync = true;

    if (sync_manager_) {
        return sync_manager_->startSync(LEDSyncManager::SyncMode::STROBE_SYNC);
    }

    return true;
}

bool VCSELProjector::configureForFaceRecognition() {
    // Optimize for face recognition
    config_.pattern = PatternType::ADAPTIVE;
    config_.enable_flood_assist = true;
    config_.projection_duration_ms = 100;
    config_.enable_camera_sync = false;

    // Use gentler current for faces
    config_.vcsel_current_ma = std::min(config_.vcsel_current_ma, static_cast<uint16_t>(200));
    config_.flood_current_ma = std::min(config_.flood_current_ma, static_cast<uint16_t>(150));

    return true;
}

bool VCSELProjector::configureForStructuredLight() {
    // Full structured light configuration
    config_.pattern = PatternType::DOTS_15K;
    config_.enable_flood_assist = false; // Pure structured light
    config_.projection_duration_ms = 30;  // Short bursts
    config_.enable_camera_sync = true;

    if (sync_manager_) {
        return sync_manager_->startSync(LEDSyncManager::SyncMode::PATTERN_SYNC);
    }

    return true;
}

bool VCSELProjector::checkSafetyLimits() const {
    // Check duty cycle
    if (!checkDutyCycle()) {
        return false;
    }

    // Check thermal status
    if (thermal_manager_ && thermal_manager_->isThermalProtectionActive()) {
        auto thermal_status = thermal_manager_->getStatus();
        if (thermal_status.current_state >= LEDThermalManager::ThermalState::CRITICAL) {
            return false;
        }
    }

    // Check continuous operation time
    if (projection_active_.load()) {
        auto now = std::chrono::steady_clock::now();
        auto continuous_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_projection_start_).count();

        if (continuous_time > config_.max_continuous_time_ms) {
            return false;
        }
    }

    return true;
}

bool VCSELProjector::checkDutyCycle() const {
    auto current_time = std::chrono::steady_clock::now();
    auto window_duration = current_time - duty_cycle_window_start_;

    if (window_duration >= DUTY_CYCLE_WINDOW) {
        // Reset window
        const_cast<VCSELProjector*>(this)->duty_cycle_window_start_ = current_time;
        const_cast<VCSELProjector*>(this)->active_time_in_window_ = std::chrono::duration<double>(0);
    }

    double current_duty_cycle = active_time_in_window_.count() /
                               std::chrono::duration<double>(window_duration).count();

    return current_duty_cycle < config_.max_duty_cycle;
}

void VCSELProjector::updateDutyCycleTracking(bool projection_active) {
    static bool last_state = false;
    static auto last_time = std::chrono::steady_clock::now();

    auto current_time = std::chrono::steady_clock::now();

    if (last_state) {
        active_time_in_window_ += current_time - last_time;
    }

    last_state = projection_active;
    last_time = current_time;

    // Update duty cycle in status
    auto window_duration = current_time - duty_cycle_window_start_;
    if (window_duration.count() > 0) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_duty_cycle = active_time_in_window_.count() /
                                   std::chrono::duration<double>(window_duration).count();
    }
}

void VCSELProjector::handleThermalEvent(bool thermal_active, float temperature_c) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.thermal_protection_active = thermal_active;
        status_.temperature_c = temperature_c;
        status_.thermal_events++;
        status_.thermal_ok = !thermal_active;
    }

    if (thermal_callback_) {
        thermal_callback_(thermal_active, temperature_c);
    }

    if (thermal_active) {
    // TODO: Fix formatted logging -         core::Logger::getInstance().warning("Thermal protection active at {:.1f}°C", temperature_c);
    } else {
    // TODO: Fix formatted logging -         core::Logger::getInstance().info("Thermal protection deactivated at {:.1f}°C", temperature_c);
    }
}

void VCSELProjector::updatePerformanceMetrics(double sync_error_us, bool sync_success) {
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);

    if (sync_success) {
        performance_metrics_.successful_syncs++;
    } else {
        performance_metrics_.failed_syncs++;
    }

    // Update sync error statistics
    sync_error_history_.push_back(sync_error_us);
    if (sync_error_history_.size() > MAX_SYNC_HISTORY) {
        sync_error_history_.erase(sync_error_history_.begin());
    }

    if (!sync_error_history_.empty()) {
        performance_metrics_.avg_sync_error_us = std::accumulate(
            sync_error_history_.begin(), sync_error_history_.end(), 0.0) / sync_error_history_.size();

        performance_metrics_.max_sync_error_us = *std::max_element(
            sync_error_history_.begin(), sync_error_history_.end());
    }
}

void VCSELProjector::resetPerformanceMetrics() {
    std::lock_guard<std::mutex> metrics_lock(metrics_mutex_);
    performance_metrics_ = PerformanceMetrics{};
    sync_error_history_.clear();
}

void VCSELProjector::updateStatus() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_update = std::chrono::steady_clock::now();

    // Update average projection time
    if (status_.projection_cycles > 0) {
        status_.avg_projection_time_ms = std::chrono::duration<double, std::milli>(
            total_projection_time_).count() / status_.projection_cycles;
    }
}

void VCSELProjector::setErrorState(const std::string& error) {
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.last_error = error;
        status_.hardware_ok = false;
    }

    // TODO: Fix formatted logging -     core::Logger::getInstance().error("VCSELProjector: {}", error);

    if (error_callback_) {
        error_callback_(error);
    }
}

void VCSELProjector::clearErrorState() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_error.clear();
    status_.hardware_ok = true;
}

// Diagnostic test methods

bool VCSELProjector::testI2CCommunication() {
    if (!as1170_controller_) {
        return false;
    }

    return as1170_controller_->testCommunication();
}

bool VCSELProjector::testGPIOControl() {
    if (!as1170_controller_) {
        return false;
    }

    // Test GPIO strobe control
    return as1170_controller_->generateStrobe(1000); // 1ms test strobe
}

bool VCSELProjector::testLEDFunctionality(AS1170Controller::LEDChannel channel) {
    if (!as1170_controller_) {
        return false;
    }

    // Test LED with minimal current
    bool result = as1170_controller_->setLEDState(channel, true, 50); // 50mA test
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    result &= as1170_controller_->setLEDState(channel, false, 0);

    return result;
}

bool VCSELProjector::testThermalSensor() {
    if (!as1170_controller_) {
        return false;
    }

    float temp = as1170_controller_->readTemperature();
    return temp > -999.0f && temp > -40.0f && temp < 150.0f; // Reasonable range
}

bool VCSELProjector::testSyncTiming() {
    if (!sync_manager_) {
        return false;
    }

    // Basic sync functionality test
    return sync_manager_->isInSync() || !sync_manager_->getStatus().sync_active;
}

} // namespace hardware
} // namespace unlook