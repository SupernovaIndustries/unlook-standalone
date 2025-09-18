#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QTextEdit>
#include <QTimer>
#include <QPixmap>
// Note: Using libcamera for camera access, not Qt multimedia
#include <memory>
#include <mutex>
#include <atomic>

// Unlook core systems
#include "unlook/camera/camera_system.hpp"
#include "unlook/core/types.hpp"

// Face recognition APIs
#include "unlook/face/FaceAPI.hpp"
#include "unlook/face/FaceEnroller.hpp"
#include "unlook/face/LivenessDetector.hpp"
#include "unlook/face/BankingMLValidator.hpp"

// VCSEL hardware integration (temporarily disabled)
// #include "unlook/hardware/VCSELProjector.hpp"
// #include "unlook/hardware/AS1170Controller.hpp"
// #include "unlook/hardware/LEDSyncManager.hpp"

// GUI components
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/status_display.hpp"

namespace unlook {
namespace gui {

/**
 * @brief VCSEL-Integrated Face Enrollment Widget
 *
 * Banking-grade facial recognition enrollment system with integrated VCSEL illumination.
 * Provides Intel RealSense style workflow with industrial safety and security features.
 *
 * Key Features:
 * - VCSEL-synchronized face capture for enhanced IR feature detection
 * - Banking-grade security (FAR < 0.001%, FRR < 3%)
 * - Multi-angle enrollment with adaptive VCSEL illumination
 * - Real-time liveness detection with depth verification
 * - Thermal safety monitoring for face-proximity operation
 * - Supernova-tech UI design with touch optimization
 *
 * Safety Implementation:
 * - VCSEL OFF by default, only activated during capture
 * - Reduced current (250mA) for face proximity safety
 * - Automatic thermal protection and emergency shutdown
 * - Eye safety compliance with short burst durations
 *
 * Workflow:
 * 1. Face Detection Phase - Ambient lighting detection
 * 2. Enrollment Setup - User identity and settings
 * 3. Multi-Angle Capture - 5-point face capture with VCSEL sync
 * 4. Quality Verification - Banking-grade template validation
 * 5. Template Storage - Secure biometric template creation
 */
class FaceEnrollmentWidget : public QWidget {
    Q_OBJECT

public:
    enum class EnrollmentPhase {
        INITIALIZATION,     // System initialization and safety checks
        USER_SETUP,         // User identity and settings configuration
        FACE_DETECTION,     // Initial face detection and positioning
        MULTI_ANGLE_CAPTURE, // 5-point face capture with VCSEL
        QUALITY_VERIFICATION, // Template quality assessment
        TEMPLATE_STORAGE,   // Secure template creation and storage
        COMPLETION,         // Enrollment success/failure
        ERROR_STATE        // Error recovery and troubleshooting
    };

    enum class CaptureAngle {
        CENTER,            // Center facing capture (primary)
        LEFT_PROFILE,      // 45° left profile
        RIGHT_PROFILE,     // 45° right profile
        SLIGHT_UP,         // 15° upward angle
        SLIGHT_DOWN        // 15° downward angle
    };

    struct EnrollmentConfig {
        // Banking-grade security settings
        face::BankingSecurityLevel security_level = face::BankingSecurityLevel::MAXIMUM;
        float false_accept_rate_threshold = 0.00001f;    // 0.001% FAR requirement
        float false_reject_rate_threshold = 0.03f;       // 3% FRR maximum

        // VCSEL illumination settings
        bool enable_vcsel_illumination = true;
        uint16_t vcsel_current_ma = 200;                 // Reduced for face safety
        uint32_t vcsel_burst_duration_ms = 100;          // Short bursts for eye safety
        float face_distance_optimal_mm = 300.0f;         // Optimal face distance

        // Enrollment quality requirements
        uint32_t required_capture_angles = 5;           // Multi-angle requirement
        float minimum_face_quality = 0.85f;             // Quality threshold
        float minimum_liveness_score = 0.90f;           // Anti-spoofing threshold
        bool require_depth_verification = true;         // 3D face verification

        // UI and user experience
        bool enable_real_time_feedback = true;
        bool enable_voice_guidance = false;             // Future feature
        uint32_t capture_timeout_seconds = 30;          // Per-angle timeout
        bool auto_capture_on_quality = true;            // Auto-capture when quality good
    };

    struct EnrollmentStatus {
        EnrollmentPhase current_phase = EnrollmentPhase::INITIALIZATION;
        CaptureAngle current_angle = CaptureAngle::CENTER;
        float overall_progress = 0.0f;

        // Capture status
        std::map<CaptureAngle, bool> angles_captured;
        std::map<CaptureAngle, float> angle_quality_scores;
        uint32_t successful_captures = 0;
        uint32_t failed_attempts = 0;

        // VCSEL status
        bool vcsel_ready = false;
        bool vcsel_active = false;
        float vcsel_temperature_c = 0.0f;
        bool thermal_protection_active = false;

        // Face detection status
        bool face_detected = false;
        face::FaceQuality face_quality = face::FaceQuality::POOR;
        float liveness_score = 0.0f;
        bool depth_verification_passed = false;

        // Security validation
        float biometric_template_quality = 0.0f;
        bool banking_validation_passed = false;

        // Error state
        std::string last_error_message;
        bool has_critical_error = false;
    };

    /**
     * @brief Constructor
     * @param camera_system Shared camera system for stereo capture
     * @param parent Parent widget
     */
    explicit FaceEnrollmentWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                                  QWidget* parent = nullptr);

    /**
     * @brief Destructor with safety shutdown
     */
    ~FaceEnrollmentWidget();

    /**
     * @brief Initialize face enrollment system with VCSEL integration
     * @param config Enrollment configuration
     * @return true if initialization successful
     */
    bool initialize(const EnrollmentConfig& config = EnrollmentConfig{});

    /**
     * @brief Start enrollment process for new user
     * @param user_id Unique user identifier
     * @param display_name User display name
     */
    void startEnrollment(const std::string& user_id, const std::string& display_name);

    /**
     * @brief Cancel current enrollment process
     */
    void cancelEnrollment();

    /**
     * @brief Get current enrollment status
     */
    EnrollmentStatus getEnrollmentStatus() const;

    /**
     * @brief Emergency shutdown - immediate VCSEL disable
     */
    void emergencyShutdown();

signals:
    /**
     * @brief Enrollment completed successfully
     * @param user_id User identifier
     * @param template_quality Final template quality score
     */
    void enrollmentCompleted(const std::string& user_id, float template_quality);

    /**
     * @brief Enrollment failed
     * @param user_id User identifier
     * @param error_message Detailed error description
     */
    void enrollmentFailed(const std::string& user_id, const std::string& error_message);

    /**
     * @brief Enrollment phase changed
     * @param phase New enrollment phase
     */
    void phaseChanged(EnrollmentPhase phase);

    /**
     * @brief Face quality updated
     * @param quality Current face quality assessment
     */
    void faceQualityChanged(face::FaceQuality quality);

    /**
     * @brief VCSEL status changed
     * @param active VCSEL activation state
     * @param temperature Current temperature
     */
    void vcselStatusChanged(bool active, float temperature);

private slots:
    /**
     * @brief Handle camera frame updates for face detection
     */
    void onCameraFrameUpdate();

    /**
     * @brief Handle VCSEL temperature monitoring
     */
    void onVCSELTemperatureUpdate();

    /**
     * @brief Process face enrollment step
     */
    void processEnrollmentStep();

    /**
     * @brief Handle capture button press
     */
    void onCaptureButtonPressed();

    /**
     * @brief Handle angle navigation buttons
     */
    void onNextAnglePressed();
    void onPreviousAnglePressed();

    /**
     * @brief Handle enrollment restart
     */
    void onRestartEnrollment();

    /**
     * @brief Handle emergency stop
     */
    void onEmergencyStop();

private:
    /**
     * @brief Setup UI layout with Supernova styling
     */
    void setupUI();

    /**
     * @brief Apply Supernova-tech styling
     */
    void applySupernovanStyling();

    /**
     * @brief Initialize hardware systems
     */
    bool initializeHardware();

    /**
     * @brief Initialize face recognition pipeline
     */
    bool initializeFaceRecognition();

    /**
     * @brief Setup VCSEL projector for face operations
     */
    bool initializeVCSEL();

    /**
     * @brief Update UI for current enrollment phase
     */
    void updateUIForPhase(EnrollmentPhase phase);

    /**
     * @brief Update progress indicators
     */
    void updateProgress();

    /**
     * @brief Update face detection visualization
     */
    void updateFaceDetectionUI();

    /**
     * @brief Update VCSEL status indicators
     */
    void updateVCSELStatus();

    /**
     * @brief Process face detection and quality assessment
     */
    void processFaceDetection(const core::StereoFramePair& frame_pair);

    /**
     * @brief Capture face data for current angle with VCSEL sync
     */
    bool captureFaceAngle(CaptureAngle angle);

    /**
     * @brief Validate captured face template quality
     */
    bool validateFaceTemplate(const face::FaceTemplate& face_template);

    /**
     * @brief Generate final biometric template
     */
    bool generateBiometricTemplate();

    /**
     * @brief Handle enrollment error
     */
    void handleEnrollmentError(const std::string& error_message);

    /**
     * @brief Safe VCSEL activation with monitoring
     */
    bool activateVCSELSafe();

    /**
     * @brief Safe VCSEL deactivation
     */
    void deactivateVCSELSafe();

    // UI Components
    QVBoxLayout* main_layout_;
    QHBoxLayout* top_bar_layout_;
    QGridLayout* content_layout_;
    QHBoxLayout* controls_layout_;

    // Status and monitoring displays
    widgets::StatusDisplay* enrollment_status_display_;
    widgets::StatusDisplay* vcsel_status_display_;
    widgets::StatusDisplay* face_quality_display_;
    QProgressBar* overall_progress_bar_;
    QProgressBar* angle_progress_bar_;

    // Face capture and preview
    QLabel* camera_preview_label_;
    QLabel* face_detection_overlay_;
    QLabel* instruction_label_;
    QTextEdit* feedback_text_;

    // Control buttons
    widgets::TouchButton* capture_button_;
    widgets::TouchButton* next_angle_button_;
    widgets::TouchButton* previous_angle_button_;
    widgets::TouchButton* restart_button_;
    widgets::TouchButton* emergency_stop_button_;

    // Angle indicator buttons
    std::map<CaptureAngle, widgets::TouchButton*> angle_indicator_buttons_;

    // Core system integrations
    std::shared_ptr<camera::CameraSystem> camera_system_;
    // std::unique_ptr<hardware::VCSELProjector> vcsel_projector_;  // Temporarily disabled
    std::shared_ptr<hardware::AS1170Controller> as1170_controller_;  // Singleton instance

    // Face recognition pipeline
    std::unique_ptr<face::FaceEnroller> face_enroller_;
    std::unique_ptr<face::LivenessDetector> liveness_detector_;
    std::unique_ptr<face::BankingMLValidator> banking_validator_;

    // Configuration and state
    EnrollmentConfig config_;
    mutable std::mutex status_mutex_;
    EnrollmentStatus status_;
    std::atomic<bool> enrollment_active_{false};
    std::atomic<bool> emergency_shutdown_active_{false};

    // Enrollment data
    std::string current_user_id_;
    std::string current_display_name_;
    std::map<CaptureAngle, face::FaceTemplate> captured_templates_;
    face::CompositeFaceTemplate final_template_;

    // Monitoring and safety
    QTimer* frame_update_timer_;
    QTimer* vcsel_monitoring_timer_;
    QTimer* enrollment_timeout_timer_;

    // Thread safety
    mutable std::mutex camera_mutex_;
    mutable std::mutex vcsel_mutex_;
    mutable std::mutex ui_update_mutex_;
};

} // namespace gui
} // namespace unlook