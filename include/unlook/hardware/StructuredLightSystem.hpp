#pragma once

#include <unlook/hardware/VCSELProjector.hpp>
#include <unlook/camera/HardwareSyncCapture.hpp>
#include <unlook/stereo/DepthProcessor.hpp>

#include <memory>
#include <functional>
#include <atomic>
#include <mutex>
#include <thread>
#include <deque>
#include <opencv2/opencv.hpp>

namespace unlook {
namespace hardware {

/**
 * Structured Light System Integration
 *
 * High-level integration class that combines the VCSEL projector system with
 * the existing camera and depth processing pipeline. Provides seamless
 * integration for depth capture, face recognition, and GUI control.
 *
 * This class serves as the primary integration point between:
 * - VCSEL projector hardware control
 * - Camera synchronization system
 * - Depth processing pipeline
 * - GUI components and user interfaces
 *
 * Key Features:
 * - One-shot depth capture with structured light
 * - Synchronized camera-projector operation
 * - Face recognition illumination
 * - GUI integration callbacks
 * - Performance monitoring and diagnostics
 * - Thread-safe operation
 *
 * Integration Points:
 * - camera::HardwareSyncCapture for stereo capture
 * - stereo::DepthProcessor for depth computation
 * - gui components for control and monitoring
 */
class StructuredLightSystem {
public:
    enum class CaptureMode {
        DEPTH_ONLY,          // Structured light depth capture only
        DEPTH_WITH_RGB,      // Depth + RGB stereo capture
        FACE_RECOGNITION,    // Optimized for face capture
        CALIBRATION,         // System calibration mode
        DIAGNOSTIC          // Diagnostic and testing mode
    };

    struct CaptureConfig {
        CaptureMode mode;
        VCSELProjector::PatternType pattern;

        // Camera settings
        uint32_t exposure_time_us;    // Exposure time in microseconds
        float gain;                   // Camera gain
        bool enable_auto_exposure;    // Use manual exposure for consistency

        // Projector settings
        uint16_t vcsel_current_ma;      // VCSEL current
        uint16_t flood_current_ma;      // Flood illumination current
        uint32_t projection_duration_ms; // Projection duration

        // Synchronization
        bool enable_precise_sync;         // Enable <50μs synchronization
        uint32_t sync_timeout_ms;         // Sync timeout
        uint32_t pre_projection_delay_us; // LED pre-activation delay

        // Quality settings
        bool enable_multi_shot;           // Multi-shot for better quality
        uint32_t shots_count;             // Number of shots (if multi-shot)
        uint32_t shot_interval_ms;        // Interval between shots

        // Face recognition optimization
        float face_distance_mm;           // Expected face distance
        bool adaptive_illumination;       // Adapt to face distance/ambient

        // Default constructor with industrial-grade defaults
        CaptureConfig() :
            mode(CaptureMode::DEPTH_ONLY),
            pattern(VCSELProjector::PatternType::DOTS_15K),
            exposure_time_us(10000),      // 10ms default exposure
            gain(1.0f),                   // Unity gain
            enable_auto_exposure(false),  // Manual exposure for consistency
            vcsel_current_ma(250),        // 250mA VCSEL current
            flood_current_ma(150),        // 150mA flood current
            projection_duration_ms(50),   // 50ms projection
            enable_precise_sync(true),    // Enable sync
            sync_timeout_ms(1000),        // 1s timeout
            pre_projection_delay_us(5000), // 5ms pre-delay
            enable_multi_shot(false),     // Single shot
            shots_count(1),               // One shot
            shot_interval_ms(100),        // 100ms interval
            face_distance_mm(300.0f),     // 300mm face distance
            adaptive_illumination(true)   // Enable adaptation
        {}
    };

    struct CaptureResult {
        bool success = false;
        std::string error_message;

        // Captured data
        cv::Mat left_image;                   // Left camera image
        cv::Mat right_image;                  // Right camera image
        cv::Mat depth_map;                    // Computed depth map
        cv::Mat point_cloud;                  // 3D point cloud (optional)

        // Metadata
        uint64_t capture_timestamp_ns = 0;    // Capture timestamp
        uint64_t projection_timestamp_ns = 0; // Projection timestamp
        double sync_error_us = 0.0;           // Synchronization error
        float projection_temperature_c = 0.0f;// Projector temperature during capture

        // Quality metrics
        uint32_t valid_depth_pixels = 0;      // Number of valid depth pixels
        float depth_quality_score = 0.0f;     // Depth quality (0-1)
        float brightness_score = 0.0f;        // Image brightness quality
        float contrast_score = 0.0f;          // Image contrast quality

        // Performance
        uint32_t capture_duration_ms = 0;     // Total capture time
        uint32_t processing_duration_ms = 0;  // Depth processing time
    };

    // Callback types for GUI integration
    using CaptureCallback = std::function<void(const CaptureResult& result)>;
    using ProgressCallback = std::function<void(const std::string& stage, float progress)>;
    using StatusCallback = std::function<void(const std::string& status, bool is_error)>;

    StructuredLightSystem();
    ~StructuredLightSystem();

    /**
     * Initialize structured light system
     * Sets up camera, projector, and depth processing components
     * @param config Capture configuration
     * @return true if initialization successful
     */
    bool initialize(const CaptureConfig& config = CaptureConfig{});

    /**
     * Shutdown system and cleanup resources
     */
    void shutdown();

    /**
     * Perform synchronized depth capture
     * Main method for depth capture with structured light
     * @param callback Optional callback for capture completion
     * @return Capture result with depth data and metadata
     */
    CaptureResult captureDepth(CaptureCallback callback = nullptr);

    /**
     * Perform face recognition capture
     * Optimized capture for face recognition applications
     * @param face_distance_mm Estimated face distance for optimization
     * @param callback Optional callback for capture completion
     * @return Capture result with face-optimized data
     */
    CaptureResult captureFace(float face_distance_mm = 300.0f,
                             CaptureCallback callback = nullptr);

    /**
     * Perform calibration capture
     * Special capture mode for system calibration
     * @param calibration_target Target type (checkerboard, etc.)
     * @param callback Optional callback for capture completion
     * @return Capture result with calibration data
     */
    CaptureResult captureCalibration(const std::string& calibration_target = "checkerboard",
                                    CaptureCallback callback = nullptr);

    /**
     * Start continuous capture mode
     * Begins continuous synchronized capture for real-time applications
     * @param fps Target frame rate
     * @param callback Callback for each captured frame
     * @return true if continuous mode started successfully
     */
    bool startContinuous(double fps = 10.0, CaptureCallback callback = nullptr);

    /**
     * Stop continuous capture mode
     */
    void stopContinuous();

    /**
     * Test system functionality
     * Runs comprehensive system test with diagnostics
     * @return Detailed diagnostic results
     */
    struct SystemDiagnostic {
        bool camera_system_ok = false;
        bool projector_ok = false;
        bool sync_system_ok = false;
        bool depth_processing_ok = false;

        float measured_sync_accuracy_us = 0.0f;
        float thermal_status_c = 0.0f;
        uint32_t test_duration_ms = 0;

        std::vector<std::string> warnings;
        std::vector<std::string> errors;
    };
    SystemDiagnostic runSystemTest();

    /**
     * Configure capture parameters
     * @param config New configuration to apply
     * @return true if configuration valid and applied
     */
    bool configure(const CaptureConfig& config);

    /**
     * Get current configuration
     */
    CaptureConfig getConfig() const;

    /**
     * Check if system is ready for capture
     */
    bool isReady() const;

    /**
     * Check if continuous capture is active
     */
    bool isContinuous() const;

    /**
     * Get current system status
     */
    struct SystemStatus {
        bool initialized = false;
        bool camera_ready = false;
        bool projector_ready = false;
        bool continuous_active = false;

        // Performance metrics
        uint64_t total_captures = 0;
        uint64_t successful_captures = 0;
        uint64_t sync_errors = 0;
        double avg_capture_time_ms = 0.0;
        double avg_sync_error_us = 0.0;

        // Current state
        float projector_temperature_c = 0.0f;
        bool thermal_protection_active = false;
        std::string last_error;
        uint64_t last_update_timestamp_ns = 0;
    };
    SystemStatus getStatus() const;

    /**
     * Set callbacks for GUI integration
     */
    void setCaptureCallback(CaptureCallback callback);
    void setProgressCallback(ProgressCallback callback);
    void setStatusCallback(StatusCallback callback);

    /**
     * Manual projector control for testing
     * @param enable Enable or disable projector
     * @param pattern Pattern to project
     * @param duration_ms Duration (0 = until disabled)
     */
    bool setManualProjection(bool enable,
                            VCSELProjector::PatternType pattern = VCSELProjector::PatternType::DOTS_15K,
                            uint32_t duration_ms = 0);

    /**
     * Optimize system for specific use case
     */
    bool optimizeForDepthAccuracy();
    bool optimizeForFaceRecognition();
    bool optimizeForSpeed();

    /**
     * Calibrate system with known target
     * @param target_distance_mm Known target distance
     * @param target_size_mm Known target size
     * @return true if calibration successful
     */
    bool calibrateSystem(float target_distance_mm, float target_size_mm);

    /**
     * Emergency stop all operations
     * Thread-safe emergency shutdown
     */
    void emergencyStop();

    // Advanced features for power users

    /**
     * Get access to underlying components for advanced control
     * WARNING: Direct component access bypasses safety checks
     */
    std::shared_ptr<VCSELProjector> getProjector() const { return projector_; }
    std::shared_ptr<camera::HardwareSyncCapture> getCamera() const { return camera_system_; }
    std::shared_ptr<stereo::DepthProcessor> getDepthProcessor() const { return depth_processor_; }

    /**
     * Custom capture with full parameter control
     * For advanced users who need complete control
     */
    CaptureResult customCapture(const std::map<std::string, std::string>& parameters);

private:
    // Core components
    std::shared_ptr<VCSELProjector> projector_;
    std::shared_ptr<camera::HardwareSyncCapture> camera_system_;
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // Configuration and state
    CaptureConfig config_;
    mutable std::mutex mutex_;
    mutable std::mutex status_mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> continuous_active_{false};

    // Status tracking
    SystemStatus status_;

    // Callbacks
    CaptureCallback capture_callback_;
    ProgressCallback progress_callback_;
    StatusCallback status_callback_;

    // Continuous capture thread
    std::unique_ptr<std::thread> continuous_thread_;
    std::atomic<bool> stop_continuous_{false};

    // Performance tracking
    std::chrono::steady_clock::time_point last_capture_time_;
    std::deque<double> capture_times_;
    std::deque<double> sync_errors_;
    static constexpr size_t MAX_PERFORMANCE_HISTORY = 100;

    // Private implementation methods
    bool initializeComponents();
    void cleanupResources();

    // Capture implementation methods
    CaptureResult performCapture(CaptureMode mode);
    bool synchronizeCapture(uint64_t& camera_timestamp, uint64_t& projector_timestamp);
    bool processDepthData(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth);
    void updatePerformanceMetrics(uint32_t capture_time_ms, double sync_error_us, bool success);

    // Continuous capture worker
    void continuousCaptureWorker();

    // Utility methods
    bool validateConfiguration(const CaptureConfig& config) const;
    void updateStatus(const std::string& message, bool is_error = false);
    void notifyProgress(const std::string& stage, float progress);
    CaptureResult createErrorResult(const std::string& error);

    // Quality assessment
    float assessDepthQuality(const cv::Mat& depth_map) const;
    float assessImageBrightness(const cv::Mat& image) const;
    float assessImageContrast(const cv::Mat& image) const;
    uint32_t countValidDepthPixels(const cv::Mat& depth_map) const;

    // Safety and thermal management integration
    bool checkThermalSafety() const;
    bool checkSyncAccuracy() const;
    void handleThermalEvent(bool thermal_active, float temperature_c);
    void handleSyncError(const std::string& error);

    // Integration helpers for existing components
    bool setupCameraSync();
    bool setupDepthProcessing();
    bool setupProjectorCallbacks();
};

} // namespace hardware
} // namespace unlook