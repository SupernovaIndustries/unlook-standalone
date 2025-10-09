#pragma once

#include <unlook/hardware/AS1170Controller.hpp>
#include <unlook/hardware/LEDSyncManager.hpp>
#include <unlook/hardware/LEDThermalManager.hpp>

#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <map>

namespace unlook {
namespace hardware {

/**
 * VCSEL Projector High-Level API
 *
 * Provides a comprehensive, user-friendly interface for controlling the OSRAM BELAGO
 * 15k points VCSEL projector system. Integrates AS1170 driver, synchronization, and
 * thermal management into a single, easy-to-use API.
 *
 * Key Features:
 * - High-level pattern projection management
 * - Automatic thermal protection and current limiting
 * - Camera synchronization for depth capture
 * - Face recognition optimized illumination
 * - Safety monitoring and emergency shutdown
 * - Performance metrics and diagnostics
 *
 * Usage Example:
 * ```cpp
 * auto projector = std::make_shared<VCSELProjector>();
 * projector->initialize();
 * projector->enableDepthCapture();
 * projector->triggerStructuredLightCapture();
 * ```
 *
 * Integration Points:
 * - Depth capture pipeline
 * - Face recognition system
 * - Camera synchronization
 * - GUI controls and monitoring
 */
class VCSELProjector {
public:
    enum class ProjectionMode {
        DISABLED,           // Projector disabled
        DEPTH_CAPTURE,      // Optimized for stereo depth capture
        FACE_RECOGNITION,   // Optimized for facial feature enhancement
        STRUCTURED_LIGHT,   // Full structured light pattern projection
        FLOOD_ILLUMINATION, // Simple flood illumination
        DIAGNOSTIC         // Diagnostic and testing mode
    };

    enum class PatternType {
        NONE,              // No pattern (flood only)
        DOTS_15K,          // 15k points dot pattern (BELAGO default)
        LINES_HORIZONTAL,  // Horizontal line patterns
        LINES_VERTICAL,    // Vertical line patterns
        GRID,              // Grid pattern for calibration
        RANDOM_SPECKLE,    // Random speckle pattern
        ADAPTIVE,          // Adaptive pattern based on scene
        CUSTOM             // User-defined custom pattern
    };

    struct ProjectorConfig {
        // Basic configuration
        ProjectionMode mode;
        PatternType pattern;

        // Power settings
        uint16_t vcsel_current_ma;       // VCSEL current (LED1)
        uint16_t flood_current_ma;       // Flood current (LED2)
        bool enable_flood_assist;       // Use flood for low-light assist

        // Timing configuration
        uint32_t projection_duration_ms;  // Pattern projection duration
        uint32_t cool_down_delay_ms;     // Cool down between projections
        float max_duty_cycle;           // Maximum duty cycle (30%)

        // Thermal protection
        bool enable_thermal_protection;
        float max_temperature_c;       // Maximum operating temperature
        float thermal_throttle_temp_c; // Temperature for current throttling

        // Synchronization
        bool enable_camera_sync;        // Sync with camera captures
        uint32_t sync_tolerance_us;       // Sync timing tolerance

        // Face recognition optimization
        float face_distance_mm;       // Typical face distance for optimization
        bool adaptive_current;          // Adapt current based on distance/ambient

        // Safety limits
        uint32_t max_continuous_time_ms;  // Maximum continuous projection time
        uint32_t mandatory_pause_ms;      // Mandatory pause after max time

        // Default constructor
        ProjectorConfig() :
            mode(ProjectionMode::DEPTH_CAPTURE),
            pattern(PatternType::DOTS_15K),
            vcsel_current_ma(500),       // INCREASED: 500mA for high VCSEL visibility
            flood_current_ma(500),       // INCREASED: 500mA for high flood visibility
            enable_flood_assist(true),   // Use flood for low-light assist
            projection_duration_ms(50),  // Pattern projection duration
            cool_down_delay_ms(100),     // Cool down between projections
            max_duty_cycle(0.3f),        // Maximum duty cycle (30%)
            enable_thermal_protection(true),
            max_temperature_c(70.0f),    // Maximum operating temperature
            thermal_throttle_temp_c(65.0f), // Temperature for current throttling
            enable_camera_sync(true),    // Sync with camera captures
            sync_tolerance_us(50),       // Sync timing tolerance
            face_distance_mm(300.0f),    // Typical face distance for optimization
            adaptive_current(true),      // Adapt current based on distance/ambient
            max_continuous_time_ms(5000), // Maximum continuous projection time
            mandatory_pause_ms(1000)     // Mandatory pause after max time
        {}
    };

    struct ProjectorStatus {
        bool initialized = false;
        bool projection_active = false;
        ProjectionMode current_mode = ProjectionMode::DISABLED;
        PatternType current_pattern = PatternType::NONE;

        // Current state
        uint16_t vcsel_current_ma = 0;
        uint16_t flood_current_ma = 0;
        float temperature_c = 0.0f;
        bool thermal_protection_active = false;

        // Performance metrics
        uint64_t projection_cycles = 0;
        uint64_t sync_captures = 0;
        uint64_t thermal_events = 0;
        double avg_projection_time_ms = 0.0;
        double current_duty_cycle = 0.0;

        // Diagnostics
        bool hardware_ok = true;
        bool sync_ok = true;
        bool thermal_ok = true;
        std::string last_error;

        // Timing
        std::chrono::steady_clock::time_point last_projection;
        std::chrono::steady_clock::time_point last_update;
    };

    // Callback types for integration
    using ProjectionCallback = std::function<void(PatternType pattern, uint64_t timestamp_ns)>;
    using ThermalCallback = std::function<void(bool thermal_active, float temperature_c)>;
    using ErrorCallback = std::function<void(const std::string& error)>;
    using SyncCallback = std::function<void(uint64_t camera_timestamp_ns, uint64_t led_timestamp_ns)>;

    VCSELProjector();
    ~VCSELProjector();

    /**
     * Initialize VCSEL projector system
     * Sets up all hardware components and performs self-test
     * @param config Projector configuration
     * @return true if initialization successful
     */
    bool initialize(const ProjectorConfig& config = ProjectorConfig{});

    /**
     * Shutdown projector system safely
     * Ensures all LEDs are disabled and resources cleaned up
     */
    void shutdown();

    /**
     * Enable depth capture mode
     * Optimizes projector for stereo depth capture applications
     * @param pattern Pattern type to use for depth capture
     * @return true if mode enabled successfully
     */
    bool enableDepthCapture(PatternType pattern = PatternType::DOTS_15K);

    /**
     * Enable face recognition mode
     * Optimizes illumination for facial feature enhancement
     * @param distance_mm Typical face distance for optimization
     * @return true if mode enabled successfully
     */
    bool enableFaceRecognition(float distance_mm = 300.0f);

    /**
     * Enable structured light mode
     * Full featured structured light pattern projection
     * @param pattern Structured light pattern type
     * @return true if mode enabled successfully
     */
    bool enableStructuredLight(PatternType pattern = PatternType::DOTS_15K);

    /**
     * Disable projection (safe shutdown)
     */
    bool disableProjection();

    /**
     * Trigger synchronized capture with camera
     * Main method for depth capture integration
     * @param exposure_time_us Camera exposure time for timing optimization
     * @param callback Optional callback when projection complete
     * @return true if capture triggered successfully
     */
    bool triggerStructuredLightCapture(uint32_t exposure_time_us = 10000,
                                      ProjectionCallback callback = nullptr);

    /**
     * Trigger face recognition illumination
     * Optimized illumination for face capture
     * @param duration_ms Illumination duration
     * @param callback Optional completion callback
     * @return true if illumination triggered successfully
     */
    bool triggerFaceIllumination(uint32_t duration_ms = 100,
                                ProjectionCallback callback = nullptr);

    /**
     * Set projection pattern
     * @param pattern Pattern type to project
     * @param vcsel_current_ma VCSEL current (0 = use config default)
     * @param flood_current_ma Flood current (0 = use config default)
     * @return true if pattern set successfully
     */
    bool setProjectionPattern(PatternType pattern,
                             uint16_t vcsel_current_ma = 0,
                             uint16_t flood_current_ma = 0);

    /**
     * Manual projection control (for testing/diagnostics)
     * @param enable Enable or disable projection
     * @param duration_ms Projection duration (0 = until manually disabled)
     * @return true if successful
     */
    bool setManualProjection(bool enable, uint32_t duration_ms = 0);

    /**
     * Configure camera synchronization
     * @param enable Enable sync with camera system
     * @param tolerance_us Timing tolerance in microseconds
     * @return true if configured successfully
     */
    bool configureCameraSync(bool enable, uint32_t tolerance_us = 50);

    /**
     * Adaptive current control based on ambient conditions
     * @param ambient_light_lux Ambient light level (lux)
     * @param target_distance_mm Distance to target object
     * @return true if current adapted successfully
     */
    bool adaptCurrentForConditions(float ambient_light_lux, float target_distance_mm);

    /**
     * Emergency shutdown - immediate LED disable
     * Thread-safe, can be called from any context
     */
    void emergencyShutdown();

    /**
     * Perform comprehensive self-test
     * @return true if all systems pass self-test
     */
    bool performSelfTest();

    /**
     * Get current projector status
     */
    ProjectorStatus getStatus() const;

    /**
     * Get current configuration
     */
    ProjectorConfig getConfig() const;

    /**
     * Check if projector is ready for operation
     */
    bool isReady() const;

    /**
     * Check if projection is currently active
     */
    bool isProjectionActive() const;

    /**
     * Get thermal status
     */
    AS1170Controller::ThermalStatus getThermalStatus() const;

    /**
     * Register callbacks for integration
     */
    void setProjectionCallback(ProjectionCallback callback);
    void setThermalCallback(ThermalCallback callback);
    void setErrorCallback(ErrorCallback callback);
    void setSyncCallback(SyncCallback callback);

    /**
     * Advanced pattern management
     */
    struct PatternParams {
        float intensity = 1.0f;        // Pattern intensity (0.0-1.0)
        uint32_t frequency_hz = 60;    // Pattern switching frequency
        bool enable_dithering = false; // Temporal dithering
        uint8_t custom_pattern[256];   // Custom pattern data
    };

    bool setAdvancedPattern(const PatternParams& params);

    /**
     * Performance monitoring
     */
    struct PerformanceMetrics {
        double avg_sync_error_us;
        double max_sync_error_us;
        double thermal_efficiency;
        double power_consumption_w;
        uint64_t total_projection_time_ms;
        uint64_t successful_syncs;
        uint64_t failed_syncs;
    };

    PerformanceMetrics getPerformanceMetrics() const;

    /**
     * Calibration and optimization
     */
    bool calibrateProjectionIntensity(float target_distance_mm);
    bool optimizeForDepthAccuracy();
    bool optimizeForFaceRecognition();

    /**
     * Hardware diagnostics
     */
    struct DiagnosticResult {
        bool i2c_communication = false;
        bool gpio_control = false;
        bool led1_functional = false;
        bool led2_functional = false;
        bool thermal_sensor = false;
        bool sync_timing = false;
        float measured_current_led1_ma = 0.0f;
        float measured_current_led2_ma = 0.0f;
        float measured_temperature_c = 0.0f;
        std::vector<std::string> error_details;
    };

    DiagnosticResult runDiagnostics();

private:
    // Hardware components
    std::shared_ptr<AS1170Controller> as1170_controller_;
    std::shared_ptr<LEDSyncManager> sync_manager_;
    std::shared_ptr<LEDThermalManager> thermal_manager_;

    // Configuration and state
    ProjectorConfig config_;
    mutable std::mutex mutex_;
    mutable std::mutex status_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> projection_active_{false};

    // Status tracking
    ProjectorStatus status_;

    // Callbacks
    ProjectionCallback projection_callback_;
    ThermalCallback thermal_callback_;
    ErrorCallback error_callback_;
    SyncCallback sync_callback_;

    // Timing and performance tracking
    std::chrono::steady_clock::time_point last_projection_start_;
    std::chrono::steady_clock::time_point initialization_time_;
    std::chrono::duration<double> total_projection_time_{0};

    // Duty cycle monitoring
    std::chrono::steady_clock::time_point duty_cycle_window_start_;
    std::chrono::duration<double> active_time_in_window_{0};
    static constexpr std::chrono::seconds DUTY_CYCLE_WINDOW{60}; // 60 second window

    // Pattern management
    std::map<PatternType, PatternParams> pattern_library_;
    PatternParams current_pattern_params_;

    // Performance metrics
    mutable std::mutex metrics_mutex_;
    PerformanceMetrics performance_metrics_;
    std::vector<double> sync_error_history_;
    static constexpr size_t MAX_SYNC_HISTORY = 1000;

    // Private implementation methods
    bool initializeHardwareComponents();
    bool validateConfiguration(const ProjectorConfig& config) const;
    void cleanupResources();

    // Pattern management methods
    bool loadPatternLibrary();
    bool activatePattern(PatternType pattern, uint32_t duration_ms);
    bool deactivateCurrentPattern();
    uint16_t calculateOptimalCurrent(PatternType pattern, float distance_mm, float ambient_lux);

    // Mode switching methods
    bool switchToMode(ProjectionMode mode);
    bool configureForDepthCapture();
    bool configureForFaceRecognition();
    bool configureForStructuredLight();

    // Safety and monitoring
    bool checkSafetyLimits() const;
    bool checkDutyCycle() const;
    void updateDutyCycleTracking(bool projection_active);
    void handleThermalEvent(bool thermal_active, float temperature_c);
    void handleSyncError(const std::string& error);

    // Performance tracking
    void updatePerformanceMetrics(double sync_error_us, bool sync_success);
    void resetPerformanceMetrics();

    // Status updates
    void updateStatus();
    void setErrorState(const std::string& error);
    void clearErrorState();

    // Hardware abstraction helpers
    bool setHardwareLEDState(bool led1_enable, uint16_t led1_current,
                            bool led2_enable, uint16_t led2_current);
    bool readHardwareStatus();

    // Synchronization helpers
    bool waitForCameraReady(uint32_t timeout_ms = 1000);
    uint64_t getCurrentTimestamp() const;

    // Calibration helpers
    bool measureProjectionIntensity(PatternType pattern, float& intensity_lux);
    bool adjustCurrentForTarget(float target_intensity, uint16_t& current_ma);

    // Diagnostic helpers
    bool testI2CCommunication();
    bool testGPIOControl();
    bool testLEDFunctionality(AS1170Controller::LEDChannel channel);
    bool testThermalSensor();
    bool testSyncTiming();
};

} // namespace hardware
} // namespace unlook