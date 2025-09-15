#pragma once

#include <QWidget>
#include <QLabel>
#include <QStackedWidget>
#include <QProgressBar>
#include <QTimer>
#include <QCheckBox>
#include <QLineEdit>
#include <QPushButton>
#include <QFrame>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QShowEvent>
#include <QHideEvent>
#include <memory>
#include <atomic>
#include <mutex>

#include "unlook/camera/camera_system.hpp"
#include "unlook/face/FaceEnroller.hpp"
#include "unlook/face/FaceTypes.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/status_display.hpp"

// Forward declarations
QT_BEGIN_NAMESPACE
namespace Ui { class FaceEnrollmentWidget; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

/**
 * @brief Intel RealSense style facial enrollment widget with banking-grade UX
 *
 * Professional face enrollment system that guides users through a comprehensive
 * enrollment process similar to Intel RealSense ID with banking-grade security
 * and user experience. Integrates seamlessly with the Unlook 3D Scanner's
 * face recognition API and camera system.
 *
 * Key Features:
 * - Intel RealSense style multi-step workflow
 * - Banking-grade consent and security indicators
 * - Real-time face preview with 3D overlay guidance
 * - Interactive positioning with quality feedback
 * - Multi-sample collection with pose variation requirements
 * - Quality gates and validation checkpoints
 * - Comprehensive error recovery with user-friendly suggestions
 * - Accessibility compliance (high contrast, screen reader support)
 * - Touch-optimized interface for Raspberry Pi touchscreen
 * - ARM64/CM4 performance optimized
 * - GDPR compliance with clear consent flows
 * - Audit trail for banking compliance
 *
 * Workflow Steps:
 * 1. User ID input and banking-grade consent flow
 * 2. Interactive face positioning guidance with real-time feedback
 * 3. Multi-sample collection with pose variation requirements
 * 4. Quality validation and sample acceptance gates
 * 5. 3D model generation and template creation
 * 6. Enrollment completion with success confirmation
 */
class FaceEnrollmentWidget : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Enrollment workflow steps
     */
    enum class EnrollmentStep {
        CONSENT_AND_SETUP,      ///< User ID and consent flow
        FACE_ENROLLMENT,        ///< Face positioning and sample collection
        COMPLETION_SUCCESS,     ///< Successful enrollment completion
        COMPLETION_FAILURE      ///< Failed enrollment with recovery options
    };

    /**
     * @brief Enrollment completion status
     */
    enum class CompletionStatus {
        SUCCESS,                ///< Enrollment completed successfully
        CANCELLED_BY_USER,      ///< User cancelled enrollment
        TECHNICAL_FAILURE,      ///< Technical failure occurred
        QUALITY_INSUFFICIENT,   ///< Sample quality insufficient
        TIMEOUT_EXCEEDED        ///< Enrollment timeout exceeded
    };

    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance
     * @param parent Parent widget
     */
    explicit FaceEnrollmentWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                                 QWidget* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~FaceEnrollmentWidget();

    /**
     * @brief Initialize enrollment widget
     * @return True if initialization successful
     */
    bool initialize();

    /**
     * @brief Check if widget is initialized and ready
     * @return True if ready for enrollment
     */
    bool isReady() const;

    /**
     * @brief Start new enrollment session
     * @param user_id Optional pre-filled user ID
     * @return True if enrollment started successfully
     */
    bool startEnrollment(const QString& user_id = QString());

    /**
     * @brief Cancel current enrollment session
     */
    void cancelEnrollment();

    /**
     * @brief Get current enrollment progress
     * @return Current progress percentage (0-100)
     */
    int getCurrentProgress() const;

    /**
     * @brief Check if enrollment is currently active
     * @return True if enrollment is in progress
     */
    bool isEnrollmentActive() const;

    /**
     * @brief Enable/disable accessibility features
     * @param enable Enable accessibility mode
     * @param high_contrast Enable high contrast mode
     * @param large_text Enable large text mode
     */
    void setAccessibilityMode(bool enable,
                             bool high_contrast = true,
                             bool large_text = false);

    /**
     * @brief Enable/disable voice guidance
     * @param enable Enable voice guidance for accessibility
     */
    void setVoiceGuidanceEnabled(bool enable);

    /**
     * @brief Set enrollment configuration
     * @param banking_grade Enable banking-grade enrollment
     * @param required_samples Number of samples required
     * @param timeout_seconds Enrollment timeout in seconds
     */
    void setEnrollmentConfiguration(bool banking_grade = true,
                                   int required_samples = 5,
                                   int timeout_seconds = 300);

protected:
    /**
     * @brief Handle show events
     */
    void showEvent(QShowEvent* event) override;

    /**
     * @brief Handle hide events
     */
    void hideEvent(QHideEvent* event) override;

    /**
     * @brief Handle key press events (accessibility support)
     */
    void keyPressEvent(QKeyEvent* event) override;

signals:
    /**
     * @brief Enrollment completed successfully
     * @param user_id Enrolled user ID
     * @param template_id Generated biometric template ID
     * @param quality_score Final enrollment quality score (0-100)
     */
    void enrollmentCompleted(const QString& user_id,
                           const QString& template_id,
                           float quality_score);

    /**
     * @brief Enrollment cancelled or failed
     * @param user_id User ID that was being enrolled
     * @param status Completion status
     * @param error_message Human-readable error message
     */
    void enrollmentFailed(const QString& user_id,
                         CompletionStatus status,
                         const QString& error_message);

    /**
     * @brief Enrollment progress updated
     * @param progress Progress percentage (0-100)
     * @param step Current enrollment step
     * @param message Current status message
     */
    void enrollmentProgressChanged(int progress,
                                  EnrollmentStep step,
                                  const QString& message);

    /**
     * @brief Request to navigate back to main menu
     */
    void backToMainMenuRequested();

    /**
     * @brief Request to test authentication with enrolled template
     * @param user_id User ID to test authentication for
     */
    void testAuthenticationRequested(const QString& user_id);

private slots:
    /**
     * @brief Handle consent form validation
     */
    void validateConsentForm();

    /**
     * @brief Handle user ID input changes
     */
    void onUserIdInputChanged();

    /**
     * @brief Handle consent checkbox changes
     */
    void onConsentCheckboxChanged();

    /**
     * @brief Start enrollment process after consent
     */
    void onStartEnrollmentClicked();

    /**
     * @brief Handle manual capture button
     */
    void onManualCaptureClicked();

    /**
     * @brief Handle skip sample button
     */
    void onSkipSampleClicked();

    /**
     * @brief Handle cancel enrollment button
     */
    void onCancelEnrollmentClicked();

    /**
     * @brief Handle finish button on completion page
     */
    void onFinishClicked();

    /**
     * @brief Handle test authentication button
     */
    void onTestAuthenticationClicked();

    /**
     * @brief Handle camera frame updates
     */
    void onCameraFrameReceived(const core::StereoFramePair& frame_pair);

    /**
     * @brief Handle enrollment progress updates from FaceEnroller
     */
    void onEnrollmentProgressUpdated(const face::EnrollmentProgress& progress);

    /**
     * @brief Handle face enroller frame processing
     */
    void onFaceEnrollerFrameProcessed(const cv::Mat& preview_frame,
                                     const face::EnrollmentProgress& progress);

    /**
     * @brief Update real-time preview and guidance
     */
    void updatePreviewAndGuidance();

    /**
     * @brief Update enrollment timeout
     */
    void onEnrollmentTimeout();

    /**
     * @brief Update quality metrics display
     */
    void updateQualityMetrics(const face::EnrollmentProgress& progress);

    /**
     * @brief Update pose instruction
     */
    void updatePoseInstruction(const face::EnrollmentProgress& progress);

    /**
     * @brief Update security status indicators
     */
    void updateSecurityStatus();

    /**
     * @brief Handle accessibility shortcuts
     */
    void handleAccessibilityShortcut(int key);

private:
    /**
     * @brief Initialize UI components
     */
    void initializeUI();

    /**
     * @brief Initialize face enroller
     */
    void initializeFaceEnroller();

    /**
     * @brief Connect signals and slots
     */
    void connectSignals();

    /**
     * @brief Apply Supernova-tech styling
     */
    void applySupernovanStyling();

    /**
     * @brief Navigate to specific enrollment step
     */
    void navigateToStep(EnrollmentStep step);

    /**
     * @brief Start camera preview for enrollment
     */
    void startCameraPreview();

    /**
     * @brief Stop camera preview
     */
    void stopCameraPreview();

    /**
     * @brief Update UI elements with enrollment progress
     */
    void updateProgressDisplay(const face::EnrollmentProgress& progress);

    /**
     * @brief Update instruction text based on enrollment state
     */
    void updateInstructionText(const face::EnrollmentProgress& progress);

    /**
     * @brief Generate user guidance message
     */
    QString generateGuidanceMessage(const face::EnrollmentProgress& progress);

    /**
     * @brief Update quality status indicators
     */
    void updateQualityStatusIndicators(const face::EnrollmentProgress& progress);

    /**
     * @brief Display face preview with overlay guidance
     */
    void displayPreviewWithOverlay(const cv::Mat& preview_frame);

    /**
     * @brief Handle enrollment completion
     */
    void handleEnrollmentCompletion(const face::EnrollmentProgress& progress);

    /**
     * @brief Handle enrollment failure
     */
    void handleEnrollmentFailure(face::FaceResultCode error_code,
                                const std::string& error_message);

    /**
     * @brief Generate completion statistics
     */
    void updateCompletionStatistics(const face::EnrollmentSession& session);

    /**
     * @brief Reset widget state for new enrollment
     */
    void resetEnrollmentState();

    /**
     * @brief Validate user input before starting enrollment
     */
    bool validateUserInput();

    /**
     * @brief Add audit trail entry
     */
    void addAuditTrailEntry(const QString& event, const QString& details);

    /**
     * @brief Update accessibility features
     */
    void updateAccessibilityFeatures();

    /**
     * @brief Speak text using voice guidance (if enabled)
     */
    void speakGuidanceText(const QString& text);

    /**
     * @brief Convert quality score to user-friendly text
     */
    QString qualityScoreToText(float score);

    /**
     * @brief Convert face result code to user-friendly message
     */
    QString faceResultCodeToMessage(face::FaceResultCode code);

    /**
     * @brief Generate recovery suggestions for errors
     */
    QString generateRecoverySuggestion(face::FaceResultCode error_code);

    /**
     * @brief Apply banking-grade security styling
     */
    void applyBankingSecurityStyling();

    /**
     * @brief Update consent form validation
     */
    void updateConsentFormValidation();

    /**
     * @brief Format time duration for display
     */
    QString formatDuration(int seconds);

    // UI Components
    Ui::FaceEnrollmentWidget *ui;

    // System integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    std::unique_ptr<face::FaceEnroller> face_enroller_;

    // State management
    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> enrollment_active_{false};
    std::atomic<bool> camera_preview_active_{false};
    EnrollmentStep current_step_;
    CompletionStatus completion_status_;

    // Enrollment session data
    mutable std::mutex enrollment_mutex_;
    std::unique_ptr<face::EnrollmentSession> current_session_;
    QString current_user_id_;
    std::chrono::system_clock::time_point enrollment_start_time_;

    // Configuration
    bool banking_grade_mode_;
    int required_samples_;
    int enrollment_timeout_seconds_;
    bool accessibility_mode_;
    bool high_contrast_mode_;
    bool voice_guidance_enabled_;

    // Real-time processing
    std::mutex preview_mutex_;
    cv::Mat current_preview_frame_;
    bool has_current_preview_{false};

    // Timers
    std::unique_ptr<QTimer> preview_update_timer_;
    std::unique_ptr<QTimer> enrollment_timeout_timer_;
    std::unique_ptr<QTimer> quality_update_timer_;

    // Status tracking
    face::EnrollmentProgress last_progress_;
    int samples_captured_;
    float overall_quality_score_;
    std::vector<QString> quality_issues_;

    // Audit trail (banking compliance)
    QStringList audit_trail_entries_;

    // Performance tracking
    std::chrono::system_clock::time_point last_frame_time_;
    std::atomic<double> average_fps_{0.0};

    // Voice guidance (accessibility)
    std::unique_ptr<QTimer> voice_guidance_timer_;
    QString last_spoken_text_;

    // Constants
    static constexpr int PREVIEW_UPDATE_INTERVAL_MS = 33;  // ~30 FPS
    static constexpr int QUALITY_UPDATE_INTERVAL_MS = 100;  // 10 Hz
    static constexpr int DEFAULT_ENROLLMENT_TIMEOUT_S = 300;  // 5 minutes
    static constexpr int DEFAULT_REQUIRED_SAMPLES = 5;
    static constexpr float MIN_BANKING_QUALITY_SCORE = 0.85f;
    static constexpr float MIN_INDUSTRIAL_QUALITY_SCORE = 0.75f;

    // Disable copy constructor and assignment
    FaceEnrollmentWidget(const FaceEnrollmentWidget&) = delete;
    FaceEnrollmentWidget& operator=(const FaceEnrollmentWidget&) = delete;
};

/**
 * @brief Face enrollment guidance and user experience utilities
 */
class EnrollmentGuidanceHelper {
public:
    /**
     * @brief Generate optimal guidance text for current enrollment state
     * @param progress Current enrollment progress
     * @param is_banking_mode Whether banking-grade mode is enabled
     * @return User-friendly guidance text
     */
    static QString generateGuidanceText(const face::EnrollmentProgress& progress,
                                       bool is_banking_mode = true);

    /**
     * @brief Generate quality feedback text
     * @param quality_score Current quality score (0-1)
     * @param issues List of quality issues
     * @return User-friendly quality feedback
     */
    static QString generateQualityFeedback(float quality_score,
                                          const std::vector<std::string>& issues);

    /**
     * @brief Generate pose instruction text
     * @param current_sample Current sample number
     * @param total_samples Total samples required
     * @param pose_coverage Current pose coverage (0-1)
     * @return Pose instruction text
     */
    static QString generatePoseInstruction(int current_sample,
                                          int total_samples,
                                          float pose_coverage);

    /**
     * @brief Convert enrollment state to user-friendly text
     * @param state Current enrollment state
     * @return User-friendly state description
     */
    static QString enrollmentStateToText(face::EnrollmentState state);

    /**
     * @brief Generate error recovery suggestions
     * @param error_code Error code that occurred
     * @param consecutive_failures Number of consecutive failures
     * @return Recovery suggestion text
     */
    static QString generateRecoverySuggestions(face::FaceResultCode error_code,
                                              int consecutive_failures);

    /**
     * @brief Format quality score for display
     * @param score Quality score (0-1)
     * @param show_percentage Show as percentage
     * @return Formatted quality score text
     */
    static QString formatQualityScore(float score, bool show_percentage = true);

    /**
     * @brief Get banking-grade compliance indicators
     * @param session Current enrollment session
     * @return List of compliance status indicators
     */
    static QStringList getBankingComplianceIndicators(const face::EnrollmentSession& session);
};

} // namespace gui
} // namespace unlook