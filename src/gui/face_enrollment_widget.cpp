#include "unlook/gui/face_enrollment_widget.hpp"
#include "ui_face_enrollment_widget.h"

#include <QApplication>
#include <QMessageBox>
#include <QPixmap>
#include <QDateTime>
#include <QRegularExpression>
#include <QAccessible>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSpacerItem>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

namespace unlook {
namespace gui {

FaceEnrollmentWidget::FaceEnrollmentWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                                         QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::FaceEnrollmentWidget)
    , camera_system_(camera_system)
    , current_step_(EnrollmentStep::CONSENT_AND_SETUP)
    , completion_status_(CompletionStatus::SUCCESS)
    , banking_grade_mode_(true)
    , required_samples_(DEFAULT_REQUIRED_SAMPLES)
    , enrollment_timeout_seconds_(DEFAULT_ENROLLMENT_TIMEOUT_S)
    , accessibility_mode_(false)
    , high_contrast_mode_(false)
    , voice_guidance_enabled_(false)
    , samples_captured_(0)
    , overall_quality_score_(0.0f)
{
    ui->setupUi(this);

    // Initialize timers
    preview_update_timer_ = std::make_unique<QTimer>(this);
    enrollment_timeout_timer_ = std::make_unique<QTimer>(this);
    quality_update_timer_ = std::make_unique<QTimer>(this);
    voice_guidance_timer_ = std::make_unique<QTimer>(this);

    // Configure timers
    preview_update_timer_->setInterval(PREVIEW_UPDATE_INTERVAL_MS);
    quality_update_timer_->setInterval(QUALITY_UPDATE_INTERVAL_MS);
    enrollment_timeout_timer_->setSingleShot(true);
    voice_guidance_timer_->setSingleShot(true);

    // Initialize UI
    initializeUI();
    connectSignals();
    applySupernovanStyling();

    // Set initial state
    navigateToStep(EnrollmentStep::CONSENT_AND_SETUP);

    // Add audit trail entry
    addAuditTrailEntry("Widget Created", "Face enrollment widget initialized");
}

FaceEnrollmentWidget::~FaceEnrollmentWidget() {
    // Stop all timers
    if (preview_update_timer_ && preview_update_timer_->isActive()) {
        preview_update_timer_->stop();
    }
    if (enrollment_timeout_timer_ && enrollment_timeout_timer_->isActive()) {
        enrollment_timeout_timer_->stop();
    }
    if (quality_update_timer_ && quality_update_timer_->isActive()) {
        quality_update_timer_->stop();
    }

    // Stop camera preview
    stopCameraPreview();

    // Cancel any active enrollment
    if (enrollment_active_.load()) {
        cancelEnrollment();
    }

    // Add final audit trail entry
    addAuditTrailEntry("Widget Destroyed", "Face enrollment widget cleaned up");
}

bool FaceEnrollmentWidget::initialize() {
    if (is_initialized_.load()) {
        return true;
    }

    try {
        // Initialize face enroller
        initializeFaceEnroller();

        // Validate camera system
        if (!camera_system_ || !camera_system_->isInitialized()) {
            throw std::runtime_error("Camera system not available or not initialized");
        }

        // Initialize enrollment configuration
        face::EnrollmentConfig config;
        config.banking_grade_mode = banking_grade_mode_;
        config.required_samples = required_samples_;
        config.timeout_seconds = enrollment_timeout_seconds_;
        config.enable_liveness_detection = true;
        config.min_quality_threshold = banking_grade_mode_ ?
            MIN_BANKING_QUALITY_SCORE : MIN_INDUSTRIAL_QUALITY_SCORE;

        auto result = face_enroller_->initialize(config);
        if (result != face::FaceResultCode::SUCCESS) {
            throw std::runtime_error("Face enroller initialization failed: " +
                faceResultCodeToMessage(result).toStdString());
        }

        is_initialized_.store(true);

        // Add audit trail entry
        addAuditTrailEntry("System Initialized",
            QString("Banking mode: %1, Required samples: %2")
            .arg(banking_grade_mode_ ? "Yes" : "No")
            .arg(required_samples_));

        return true;

    } catch (const std::exception& e) {
        addAuditTrailEntry("Initialization Failed", QString::fromStdString(e.what()));
        return false;
    }
}

bool FaceEnrollmentWidget::isReady() const {
    return is_initialized_.load() &&
           camera_system_ &&
           camera_system_->isInitialized() &&
           face_enroller_ &&
           face_enroller_->isInitialized();
}

bool FaceEnrollmentWidget::startEnrollment(const QString& user_id) {
    if (!isReady()) {
        handleEnrollmentFailure(face::FaceResultCode::ERROR_CAMERA_NOT_AVAILABLE,
                               "System not ready for enrollment");
        return false;
    }

    if (enrollment_active_.load()) {
        addAuditTrailEntry("Start Enrollment Failed", "Enrollment already active");
        return false;
    }

    try {
        // Pre-fill user ID if provided
        if (!user_id.isEmpty()) {
            ui->user_id_input->setText(user_id);
            current_user_id_ = user_id;
        }

        // Reset state
        resetEnrollmentState();

        // Navigate to consent page
        navigateToStep(EnrollmentStep::CONSENT_AND_SETUP);

        // Update form validation
        updateConsentFormValidation();

        addAuditTrailEntry("Enrollment Started",
            QString("User ID: %1").arg(user_id.isEmpty() ? "Not specified" : user_id));

        return true;

    } catch (const std::exception& e) {
        handleEnrollmentFailure(face::FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED,
                               QString::fromStdString(e.what()));
        return false;
    }
}

void FaceEnrollmentWidget::cancelEnrollment() {
    if (!enrollment_active_.load()) {
        return;
    }

    try {
        // Stop timers
        preview_update_timer_->stop();
        enrollment_timeout_timer_->stop();
        quality_update_timer_->stop();

        // Stop camera preview
        stopCameraPreview();

        // Cancel face enroller session
        if (face_enroller_) {
            face_enroller_->cancelEnrollment();
        }

        // Reset state
        enrollment_active_.store(false);
        current_session_.reset();

        // Add audit trail entry
        addAuditTrailEntry("Enrollment Cancelled",
            QString("User: %1, Samples collected: %2")
            .arg(current_user_id_).arg(samples_captured_));

        // Emit signal
        emit enrollmentFailed(current_user_id_, CompletionStatus::CANCELLED_BY_USER,
                             "Enrollment cancelled by user");

        // Navigate back to consent page
        navigateToStep(EnrollmentStep::CONSENT_AND_SETUP);

    } catch (const std::exception& e) {
        addAuditTrailEntry("Cancel Enrollment Error", QString::fromStdString(e.what()));
    }
}

int FaceEnrollmentWidget::getCurrentProgress() const {
    if (!enrollment_active_.load()) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(enrollment_mutex_);
    if (current_session_) {
        return static_cast<int>(current_session_->completion_percentage);
    }

    return 0;
}

bool FaceEnrollmentWidget::isEnrollmentActive() const {
    return enrollment_active_.load();
}

void FaceEnrollmentWidget::setAccessibilityMode(bool enable, bool high_contrast, bool large_text) {
    accessibility_mode_ = enable;
    high_contrast_mode_ = high_contrast;

    if (enable) {
        setProperty("accessibleName", high_contrast ? "high_contrast" : "normal");

        // Update styling for accessibility
        updateAccessibilityFeatures();

        // Enable keyboard navigation
        setFocusPolicy(Qt::StrongFocus);

        addAuditTrailEntry("Accessibility Mode",
            QString("Enabled - High contrast: %1, Large text: %2")
            .arg(high_contrast ? "Yes" : "No")
            .arg(large_text ? "Yes" : "No"));
    }
}

void FaceEnrollmentWidget::setVoiceGuidanceEnabled(bool enable) {
    voice_guidance_enabled_ = enable;

    if (enable) {
        addAuditTrailEntry("Voice Guidance", "Enabled");
    }
}

void FaceEnrollmentWidget::setEnrollmentConfiguration(bool banking_grade,
                                                     int required_samples,
                                                     int timeout_seconds) {
    banking_grade_mode_ = banking_grade;
    required_samples_ = required_samples;
    enrollment_timeout_seconds_ = timeout_seconds;

    // Update UI labels
    ui->samples_count_label->setText(QString("Samples Collected: 0 / %1").arg(required_samples_));
    ui->samples_progress_bar->setMaximum(required_samples_);

    addAuditTrailEntry("Configuration Updated",
        QString("Banking: %1, Samples: %2, Timeout: %3s")
        .arg(banking_grade ? "Yes" : "No")
        .arg(required_samples)
        .arg(timeout_seconds));
}

void FaceEnrollmentWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);

    // Initialize if not already done
    if (!is_initialized_.load()) {
        initialize();
    }

    // Update security status
    updateSecurityStatus();
}

void FaceEnrollmentWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);

    // Stop camera preview when hidden
    stopCameraPreview();
}

void FaceEnrollmentWidget::keyPressEvent(QKeyEvent* event) {
    if (accessibility_mode_) {
        handleAccessibilityShortcut(event->key());
    }

    QWidget::keyPressEvent(event);
}

void FaceEnrollmentWidget::validateConsentForm() {
    updateConsentFormValidation();
}

void FaceEnrollmentWidget::onUserIdInputChanged() {
    current_user_id_ = ui->user_id_input->text().trimmed();
    updateConsentFormValidation();
}

void FaceEnrollmentWidget::onConsentCheckboxChanged() {
    updateConsentFormValidation();
}

void FaceEnrollmentWidget::onStartEnrollmentClicked() {
    if (!validateUserInput()) {
        return;
    }

    try {
        // Start face enrollment session
        auto result = face_enroller_->startEnrollment(current_user_id_.toStdString());
        if (result != face::FaceResultCode::SUCCESS) {
            handleEnrollmentFailure(result, "Failed to start enrollment session");
            return;
        }

        // Set enrollment as active
        enrollment_active_.store(true);
        enrollment_start_time_ = std::chrono::system_clock::now();

        // Start enrollment timeout timer
        enrollment_timeout_timer_->start(enrollment_timeout_seconds_ * 1000);

        // Navigate to enrollment page
        navigateToStep(EnrollmentStep::FACE_ENROLLMENT);

        // Start camera preview
        startCameraPreview();

        // Add audit trail entry
        addAuditTrailEntry("Enrollment Session Started",
            QString("User: %1, Timeout: %2s")
            .arg(current_user_id_).arg(enrollment_timeout_seconds_));

        // Emit progress signal
        emit enrollmentProgressChanged(0, EnrollmentStep::FACE_ENROLLMENT,
                                     "Starting face enrollment...");

        // Speak guidance if voice enabled
        if (voice_guidance_enabled_) {
            speakGuidanceText("Face enrollment started. Please position your face within the guide.");
        }

    } catch (const std::exception& e) {
        handleEnrollmentFailure(face::FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED,
                               QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::onManualCaptureClicked() {
    if (!enrollment_active_.load() || !face_enroller_) {
        return;
    }

    try {
        auto result = face_enroller_->captureEnrollmentSample(false);
        if (result != face::FaceResultCode::SUCCESS) {
            addAuditTrailEntry("Manual Capture Failed", faceResultCodeToMessage(result));

            // Show user-friendly error
            QString suggestion = generateRecoverySuggestion(result);
            ui->instruction_label->setText(suggestion);

            if (voice_guidance_enabled_) {
                speakGuidanceText(suggestion);
            }
        } else {
            addAuditTrailEntry("Manual Capture Success", "Sample captured successfully");
        }
    } catch (const std::exception& e) {
        addAuditTrailEntry("Manual Capture Error", QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::onSkipSampleClicked() {
    if (!enrollment_active_.load()) {
        return;
    }

    addAuditTrailEntry("Sample Skipped", QString("Sample %1 skipped by user").arg(samples_captured_ + 1));

    // Continue with next pose instruction
    if (samples_captured_ < required_samples_ - 1) {
        QString next_instruction = EnrollmentGuidanceHelper::generatePoseInstruction(
            samples_captured_ + 1, required_samples_,
            current_session_ ? current_session_->pose_coverage : 0.0f);
        ui->current_pose_instruction->setText(next_instruction);

        if (voice_guidance_enabled_) {
            speakGuidanceText(next_instruction);
        }
    }
}

void FaceEnrollmentWidget::onCancelEnrollmentClicked() {
    // Show confirmation dialog
    QMessageBox::StandardButton reply = QMessageBox::question(
        this, "Cancel Enrollment",
        "Are you sure you want to cancel the enrollment process?\n\n"
        "All progress will be lost and you will need to start over.",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);

    if (reply == QMessageBox::Yes) {
        cancelEnrollment();
    }
}

void FaceEnrollmentWidget::onFinishClicked() {
    // Emit completion signal
    emit enrollmentCompleted(current_user_id_,
                           current_session_ ? QString::fromStdString(current_session_->session_id) : "",
                           overall_quality_score_);

    // Request navigation back to main menu
    emit backToMainMenuRequested();
}

void FaceEnrollmentWidget::onTestAuthenticationClicked() {
    emit testAuthenticationRequested(current_user_id_);
}

void FaceEnrollmentWidget::onCameraFrameReceived(const core::StereoFramePair& frame_pair) {
    if (!enrollment_active_.load() || !face_enroller_) {
        return;
    }

    try {
        // Process frame with face enroller
        face::EnrollmentProgress progress;
        auto result = face_enroller_->processEnrollmentFrame(
            frame_pair.left_frame.image,
            frame_pair.depth_map,
            progress);

        if (result == face::FaceResultCode::SUCCESS) {
            // Update progress
            onEnrollmentProgressUpdated(progress);

            // Generate preview with guidance overlay
            cv::Mat preview_frame;
            face_enroller_->generateEnrollmentPreview(
                frame_pair.left_frame.image,
                preview_frame,
                true,  // show landmarks
                true   // show quality metrics
            );

            onFaceEnrollerFrameProcessed(preview_frame, progress);
        }

        // Track performance
        auto current_time = std::chrono::system_clock::now();
        if (last_frame_time_.time_since_epoch().count() > 0) {
            auto frame_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_frame_time_).count();
            if (frame_duration > 0) {
                double fps = 1000.0 / frame_duration;
                average_fps_.store(average_fps_.load() * 0.9 + fps * 0.1); // Exponential moving average
            }
        }
        last_frame_time_ = current_time;

    } catch (const std::exception& e) {
        addAuditTrailEntry("Frame Processing Error", QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::onEnrollmentProgressUpdated(const face::EnrollmentProgress& progress) {
    last_progress_ = progress;

    // Update UI on main thread
    QMetaObject::invokeMethod(this, [this, progress]() {
        updateProgressDisplay(progress);
        updateInstructionText(progress);
        updateQualityMetrics(progress);
        updatePoseInstruction(progress);

        // Check for completion
        if (progress.is_complete) {
            handleEnrollmentCompletion(progress);
        } else if (progress.has_error) {
            handleEnrollmentFailure(progress.error_code, progress.error_message);
        }
    }, Qt::QueuedConnection);
}

void FaceEnrollmentWidget::onFaceEnrollerFrameProcessed(const cv::Mat& preview_frame,
                                                       const face::EnrollmentProgress& progress) {
    // Update preview frame thread-safely
    {
        std::lock_guard<std::mutex> lock(preview_mutex_);
        current_preview_frame_ = preview_frame.clone();
        has_current_preview_ = true;
    }

    // Update UI on main thread
    QMetaObject::invokeMethod(this, [this]() {
        updatePreviewAndGuidance();
    }, Qt::QueuedConnection);
}

void FaceEnrollmentWidget::updatePreviewAndGuidance() {
    cv::Mat preview_frame;
    bool has_preview = false;

    {
        std::lock_guard<std::mutex> lock(preview_mutex_);
        if (has_current_preview_) {
            preview_frame = current_preview_frame_.clone();
            has_preview = true;
        }
    }

    if (has_preview) {
        displayPreviewWithOverlay(preview_frame);
    }
}

void FaceEnrollmentWidget::onEnrollmentTimeout() {
    addAuditTrailEntry("Enrollment Timeout",
        QString("Timeout after %1 seconds").arg(enrollment_timeout_seconds_));

    handleEnrollmentFailure(face::FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED,
                           "Enrollment timeout - please try again");
}

void FaceEnrollmentWidget::updateQualityMetrics(const face::EnrollmentProgress& progress) {
    // Update face distance status
    QString distance_text = "👤 Distance: ";
    if (progress.face_distance_mm > 0) {
        distance_text += QString("%1mm").arg(static_cast<int>(progress.face_distance_mm));
        if (progress.face_distance_optimal) {
            distance_text += " ✓";
            ui->face_distance_status->setStyleSheet("QLabel { color: #00FF88; font-weight: 600; }");
        } else {
            distance_text += " ⚠";
            ui->face_distance_status->setStyleSheet("QLabel { color: #FFB800; font-weight: 600; }");
        }
    } else {
        distance_text += "Measuring...";
        ui->face_distance_status->setStyleSheet("QLabel { color: #CCCCCC; }");
    }
    ui->face_distance_status->setText(distance_text);

    // Update lighting status
    QString lighting_text = "💡 Lighting: ";
    if (progress.lighting_quality > 0.7f) {
        lighting_text += "Good ✓";
        ui->lighting_status->setStyleSheet("QLabel { color: #00FF88; font-weight: 600; }");
    } else if (progress.lighting_quality > 0.4f) {
        lighting_text += "Fair ⚠";
        ui->lighting_status->setStyleSheet("QLabel { color: #FFB800; font-weight: 600; }");
    } else {
        lighting_text += "Poor ✗";
        ui->lighting_status->setStyleSheet("QLabel { color: #FF4444; font-weight: 600; }");
    }
    ui->lighting_status->setText(lighting_text);

    // Update angle status
    QString angle_text = "📐 Angle: ";
    if (progress.face_angle_optimal) {
        angle_text += "Optimal ✓";
        ui->angle_status->setStyleSheet("QLabel { color: #00FF88; font-weight: 600; }");
    } else {
        angle_text += "Adjust ⚠";
        ui->angle_status->setStyleSheet("QLabel { color: #FFB800; font-weight: 600; }");
    }
    ui->angle_status->setText(angle_text);

    // Update liveness status
    QString liveness_text = "🔒 Liveness: ";
    if (progress.liveness_confirmed) {
        liveness_text += "Confirmed ✓";
        ui->liveness_status->setStyleSheet("QLabel { color: #00FF88; font-weight: 600; }");
    } else if (progress.liveness_checking) {
        liveness_text += "Checking...";
        ui->liveness_status->setStyleSheet("QLabel { color: #00E5CC; font-weight: 600; }");
    } else {
        liveness_text += "Pending";
        ui->liveness_status->setStyleSheet("QLabel { color: #CCCCCC; }");
    }
    ui->liveness_status->setText(liveness_text);

    // Update pose coverage
    int pose_percentage = static_cast<int>(progress.pose_coverage * 100);
    QString pose_text = QString("🎯 Pose Coverage: %1%").arg(pose_percentage);
    ui->pose_variation_status->setText(pose_text);

    if (pose_percentage >= 80) {
        ui->pose_variation_status->setStyleSheet("QLabel { color: #00FF88; font-weight: 600; }");
    } else if (pose_percentage >= 50) {
        ui->pose_variation_status->setStyleSheet("QLabel { color: #FFB800; font-weight: 600; }");
    } else {
        ui->pose_variation_status->setStyleSheet("QLabel { color: #CCCCCC; }");
    }

    // Update capture button state
    bool ready_to_capture = progress.quality_score >= (banking_grade_mode_ ?
        MIN_BANKING_QUALITY_SCORE : MIN_INDUSTRIAL_QUALITY_SCORE) &&
        progress.face_detected && progress.liveness_confirmed;

    ui->manual_capture_button->setEnabled(ready_to_capture);
}

void FaceEnrollmentWidget::updatePoseInstruction(const face::EnrollmentProgress& progress) {
    QString instruction = EnrollmentGuidanceHelper::generatePoseInstruction(
        progress.samples_captured, required_samples_, progress.pose_coverage);

    ui->current_pose_instruction->setText(instruction);
}

void FaceEnrollmentWidget::updateSecurityStatus() {
    // These should always be active for our system
    ui->encryption_status->setText("🔑 AES-256 Encryption: Active");
    ui->encryption_status->setStyleSheet("QLabel { color: #00FF88; font-size: 12px; }");

    ui->local_processing_status->setText("💻 Local Processing: Enabled");
    ui->local_processing_status->setStyleSheet("QLabel { color: #00FF88; font-size: 12px; }");

    ui->gdpr_status->setText("📋 GDPR Compliant: Yes");
    ui->gdpr_status->setStyleSheet("QLabel { color: #00FF88; font-size: 12px; }");
}

void FaceEnrollmentWidget::handleAccessibilityShortcut(int key) {
    switch (key) {
        case Qt::Key_F1: // Help
            if (voice_guidance_enabled_) {
                speakGuidanceText("Face enrollment help. Use Tab to navigate, Space to activate buttons, F2 to repeat instructions.");
            }
            break;
        case Qt::Key_F2: // Repeat instructions
            if (voice_guidance_enabled_ && !last_spoken_text_.isEmpty()) {
                speakGuidanceText(last_spoken_text_);
            }
            break;
        case Qt::Key_Space: // Manual capture
            if (current_step_ == EnrollmentStep::FACE_ENROLLMENT && ui->manual_capture_button->isEnabled()) {
                onManualCaptureClicked();
            }
            break;
        case Qt::Key_Escape: // Cancel
            if (current_step_ == EnrollmentStep::FACE_ENROLLMENT) {
                onCancelEnrollmentClicked();
            }
            break;
    }
}

void FaceEnrollmentWidget::initializeUI() {
    // Set initial values
    ui->enrollment_stack->setCurrentIndex(0); // Consent page
    ui->enrollment_progress_bar->setValue(0);
    ui->samples_progress_bar->setValue(0);
    ui->samples_progress_bar->setMaximum(required_samples_);

    // Set focus policy for accessibility
    setFocusPolicy(Qt::StrongFocus);
    ui->user_id_input->setFocus();

    // Initialize security status
    updateSecurityStatus();
}

void FaceEnrollmentWidget::initializeFaceEnroller() {
    try {
        face_enroller_ = std::make_unique<face::FaceEnroller>();

        // Set callbacks
        face_enroller_->setProgressCallback([this](const face::EnrollmentProgress& progress) {
            onEnrollmentProgressUpdated(progress);
        });

        face_enroller_->setFrameProcessingCallback([this](const cv::Mat& frame, const face::EnrollmentProgress& progress) {
            onFaceEnrollerFrameProcessed(frame, progress);
        });

    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to create face enroller: " + std::string(e.what()));
    }
}

void FaceEnrollmentWidget::connectSignals() {
    // Consent page signals
    connect(ui->user_id_input, &QLineEdit::textChanged,
            this, &FaceEnrollmentWidget::onUserIdInputChanged);

    connect(ui->data_consent_checkbox, &QCheckBox::toggled,
            this, &FaceEnrollmentWidget::onConsentCheckboxChanged);
    connect(ui->privacy_consent_checkbox, &QCheckBox::toggled,
            this, &FaceEnrollmentWidget::onConsentCheckboxChanged);
    connect(ui->storage_consent_checkbox, &QCheckBox::toggled,
            this, &FaceEnrollmentWidget::onConsentCheckboxChanged);

    connect(ui->start_enrollment_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onStartEnrollmentClicked);
    connect(ui->cancel_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onCancelEnrollmentClicked);

    // Enrollment page signals
    connect(ui->manual_capture_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onManualCaptureClicked);
    connect(ui->skip_sample_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onSkipSampleClicked);
    connect(ui->cancel_enrollment_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onCancelEnrollmentClicked);

    // Completion page signals
    connect(ui->finish_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onFinishClicked);
    connect(ui->test_authentication_button, &QPushButton::clicked,
            this, &FaceEnrollmentWidget::onTestAuthenticationClicked);

    // Timer signals
    connect(preview_update_timer_.get(), &QTimer::timeout,
            this, &FaceEnrollmentWidget::updatePreviewAndGuidance);
    connect(enrollment_timeout_timer_.get(), &QTimer::timeout,
            this, &FaceEnrollmentWidget::onEnrollmentTimeout);
    connect(quality_update_timer_.get(), &QTimer::timeout,
            this, [this]() { updateQualityMetrics(last_progress_); });

    // Camera system signals
    if (camera_system_) {
        connect(camera_system_.get(), &camera::CameraSystem::stereoFrameCaptured,
                this, &FaceEnrollmentWidget::onCameraFrameReceived);
    }
}

void FaceEnrollmentWidget::applySupernovanStyling() {
    // Apply button classes to match the UI styling
    ui->start_enrollment_button->setProperty("class", "primary_button");
    ui->cancel_button->setProperty("class", "secondary_button");
    ui->manual_capture_button->setProperty("class", "primary_button");
    ui->skip_sample_button->setProperty("class", "secondary_button");
    ui->cancel_enrollment_button->setProperty("class", "error_button");
    ui->finish_button->setProperty("class", "success_button");
    ui->test_authentication_button->setProperty("class", "secondary_button");

    // Apply banking-grade security styling
    applyBankingSecurityStyling();
}

void FaceEnrollmentWidget::navigateToStep(EnrollmentStep step) {
    current_step_ = step;

    switch (step) {
        case EnrollmentStep::CONSENT_AND_SETUP:
            ui->enrollment_stack->setCurrentWidget(ui->consent_page);
            ui->user_id_input->setFocus();
            break;

        case EnrollmentStep::FACE_ENROLLMENT:
            ui->enrollment_stack->setCurrentWidget(ui->enrollment_page);
            ui->manual_capture_button->setFocus();
            break;

        case EnrollmentStep::COMPLETION_SUCCESS:
            ui->enrollment_stack->setCurrentWidget(ui->completion_page);
            ui->finish_button->setFocus();
            break;

        case EnrollmentStep::COMPLETION_FAILURE:
            // Handle failure case - could add separate failure page
            ui->enrollment_stack->setCurrentWidget(ui->consent_page);
            break;
    }

    // Update accessibility announcement
    if (voice_guidance_enabled_) {
        QString announcement;
        switch (step) {
            case EnrollmentStep::CONSENT_AND_SETUP:
                announcement = "Face enrollment setup. Please enter your user ID and provide consent.";
                break;
            case EnrollmentStep::FACE_ENROLLMENT:
                announcement = "Face enrollment in progress. Position your face within the guide.";
                break;
            case EnrollmentStep::COMPLETION_SUCCESS:
                announcement = "Enrollment completed successfully.";
                break;
            case EnrollmentStep::COMPLETION_FAILURE:
                announcement = "Enrollment failed. Please try again.";
                break;
        }
        speakGuidanceText(announcement);
    }
}

void FaceEnrollmentWidget::startCameraPreview() {
    if (camera_preview_active_.load() || !camera_system_ || !camera_system_->isInitialized()) {
        return;
    }

    try {
        // Start camera streaming
        if (camera_system_->startStreaming()) {
            camera_preview_active_.store(true);
            preview_update_timer_->start();
            quality_update_timer_->start();

            addAuditTrailEntry("Camera Preview Started", "Real-time face preview activated");
        } else {
            throw std::runtime_error("Failed to start camera streaming");
        }

    } catch (const std::exception& e) {
        addAuditTrailEntry("Camera Preview Error", QString::fromStdString(e.what()));
        handleEnrollmentFailure(face::FaceResultCode::ERROR_CAMERA_NOT_AVAILABLE,
                               "Camera not available for enrollment");
    }
}

void FaceEnrollmentWidget::stopCameraPreview() {
    if (!camera_preview_active_.load()) {
        return;
    }

    try {
        preview_update_timer_->stop();
        quality_update_timer_->stop();

        if (camera_system_) {
            camera_system_->stopStreaming();
        }

        camera_preview_active_.store(false);

        // Clear preview display
        ui->face_preview->setText("Camera Stopped");

        addAuditTrailEntry("Camera Preview Stopped", "Real-time preview deactivated");

    } catch (const std::exception& e) {
        addAuditTrailEntry("Camera Stop Error", QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::updateProgressDisplay(const face::EnrollmentProgress& progress) {
    // Update main progress bar
    int main_progress = static_cast<int>(progress.completion_percentage);
    ui->enrollment_progress_bar->setValue(main_progress);

    // Update samples progress
    ui->samples_count_label->setText(
        QString("Samples Collected: %1 / %2").arg(progress.samples_captured).arg(required_samples_));
    ui->samples_progress_bar->setValue(progress.samples_captured);

    // Store current values
    samples_captured_ = progress.samples_captured;
    overall_quality_score_ = progress.overall_quality * 100.0f; // Convert to percentage

    // Emit progress signal
    emit enrollmentProgressChanged(main_progress, current_step_,
                                 QString::fromStdString(progress.current_instruction));
}

void FaceEnrollmentWidget::updateInstructionText(const face::EnrollmentProgress& progress) {
    QString guidance = EnrollmentGuidanceHelper::generateGuidanceText(progress, banking_grade_mode_);
    ui->instruction_label->setText(guidance);

    // Speak guidance if voice enabled and text changed
    if (voice_guidance_enabled_ && guidance != last_spoken_text_) {
        voice_guidance_timer_->start(1000); // Delay to avoid too frequent speech
        voice_guidance_timer_->disconnect();
        connect(voice_guidance_timer_.get(), &QTimer::timeout, [this, guidance]() {
            speakGuidanceText(guidance);
        });
    }
}

QString FaceEnrollmentWidget::generateGuidanceMessage(const face::EnrollmentProgress& progress) {
    return EnrollmentGuidanceHelper::generateGuidanceText(progress, banking_grade_mode_);
}

void FaceEnrollmentWidget::updateQualityStatusIndicators(const face::EnrollmentProgress& progress) {
    updateQualityMetrics(progress);
}

void FaceEnrollmentWidget::displayPreviewWithOverlay(const cv::Mat& preview_frame) {
    if (preview_frame.empty()) {
        ui->face_preview->setText("No Preview Available");
        return;
    }

    try {
        // Convert OpenCV Mat to QPixmap
        cv::Mat display_frame;
        if (preview_frame.channels() == 3) {
            cv::cvtColor(preview_frame, display_frame, cv::COLOR_BGR2RGB);
        } else {
            display_frame = preview_frame.clone();
        }

        QImage qimg(display_frame.data, display_frame.cols, display_frame.rows,
                    display_frame.step, QImage::Format_RGB888);

        // Scale to fit preview label while maintaining aspect ratio
        QSize preview_size = ui->face_preview->size();
        QPixmap pixmap = QPixmap::fromImage(qimg).scaled(
            preview_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);

        ui->face_preview->setPixmap(pixmap);

    } catch (const std::exception& e) {
        ui->face_preview->setText("Preview Error");
        addAuditTrailEntry("Preview Display Error", QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::handleEnrollmentCompletion(const face::EnrollmentProgress& progress) {
    try {
        // Stop timers and camera
        enrollment_timeout_timer_->stop();
        stopCameraPreview();

        // Complete enrollment with face enroller
        face::BiometricTemplate final_template;
        auto result = face_enroller_->completeEnrollment(final_template);

        if (result != face::FaceResultCode::SUCCESS) {
            handleEnrollmentFailure(result, "Failed to complete enrollment");
            return;
        }

        // Get final session data
        face::EnrollmentSession session;
        face_enroller_->getEnrollmentSession(session);

        // Update completion statistics
        updateCompletionStatistics(session);

        // Set completion status
        completion_status_ = CompletionStatus::SUCCESS;
        enrollment_active_.store(false);

        // Navigate to completion page
        navigateToStep(EnrollmentStep::COMPLETION_SUCCESS);

        // Add audit trail entry
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now() - enrollment_start_time_).count();

        addAuditTrailEntry("Enrollment Completed",
            QString("User: %1, Quality: %2%, Duration: %3s, Template ID: %4")
            .arg(current_user_id_)
            .arg(static_cast<int>(overall_quality_score_))
            .arg(duration)
            .arg(QString::fromStdString(final_template.template_id)));

        // Emit completion signal
        emit enrollmentCompleted(current_user_id_,
                               QString::fromStdString(final_template.template_id),
                               overall_quality_score_);

        // Speak completion message
        if (voice_guidance_enabled_) {
            speakGuidanceText(QString("Enrollment completed successfully with %1 percent quality.")
                             .arg(static_cast<int>(overall_quality_score_)));
        }

    } catch (const std::exception& e) {
        handleEnrollmentFailure(face::FaceResultCode::ERROR_TEMPLATE_GENERATION_FAILED,
                               QString::fromStdString(e.what()));
    }
}

void FaceEnrollmentWidget::handleEnrollmentFailure(face::FaceResultCode error_code,
                                                  const std::string& error_message) {
    handleEnrollmentFailure(error_code, QString::fromStdString(error_message));
}

void FaceEnrollmentWidget::handleEnrollmentFailure(face::FaceResultCode error_code,
                                                  const QString& error_message) {
    // Stop timers and camera
    enrollment_timeout_timer_->stop();
    stopCameraPreview();

    // Set failure status
    completion_status_ = CompletionStatus::TECHNICAL_FAILURE;
    enrollment_active_.store(false);

    // Generate user-friendly error message and recovery suggestions
    QString user_message = faceResultCodeToMessage(error_code);
    QString recovery_suggestion = generateRecoverySuggestion(error_code);

    // Add audit trail entry
    addAuditTrailEntry("Enrollment Failed",
        QString("User: %1, Error: %2, Technical: %3")
        .arg(current_user_id_)
        .arg(user_message)
        .arg(error_message));

    // Update UI with error information
    ui->instruction_label->setText(user_message + "\n\n" + recovery_suggestion);
    ui->instruction_label->setStyleSheet(
        "QLabel { color: #FF4444; background: rgba(255, 68, 68, 0.1); "
        "border: 1px solid rgba(255, 68, 68, 0.3); border-radius: 8px; padding: 16px; }");

    // Emit failure signal
    emit enrollmentFailed(current_user_id_, completion_status_, user_message);

    // Navigate back to consent page for retry
    QTimer::singleShot(3000, [this]() {
        navigateToStep(EnrollmentStep::CONSENT_AND_SETUP);
        ui->instruction_label->setStyleSheet(""); // Reset styling
    });

    // Speak error message if voice enabled
    if (voice_guidance_enabled_) {
        speakGuidanceText(user_message + ". " + recovery_suggestion);
    }
}

void FaceEnrollmentWidget::updateCompletionStatistics(const face::EnrollmentSession& session) {
    // Update final quality score
    ui->final_quality_score->setText(QString("%1%").arg(static_cast<int>(session.overall_quality * 100)));

    // Update samples captured
    ui->samples_captured_final->setText(QString("%1 Samples Captured").arg(session.samples_captured));

    // Show banking grade badge if applicable
    if (banking_grade_mode_ && session.overall_quality >= MIN_BANKING_QUALITY_SCORE) {
        ui->banking_grade_badge->setVisible(true);
    } else {
        ui->banking_grade_badge->setVisible(false);
    }
}

void FaceEnrollmentWidget::resetEnrollmentState() {
    // Reset state variables
    enrollment_active_.store(false);
    camera_preview_active_.store(false);
    samples_captured_ = 0;
    overall_quality_score_ = 0.0f;
    quality_issues_.clear();

    // Reset UI elements
    ui->enrollment_progress_bar->setValue(0);
    ui->samples_progress_bar->setValue(0);
    ui->samples_count_label->setText(QString("Samples Collected: 0 / %1").arg(required_samples_));
    ui->face_preview->setText("Initializing Camera...");
    ui->instruction_label->setText("Position your face within the guide and look directly at the camera");

    // Reset session
    current_session_.reset();

    // Reset styling
    ui->instruction_label->setStyleSheet(
        "QLabel { color: #00E5CC; font-size: 18px; font-weight: 500; "
        "background: rgba(0, 229, 204, 0.1); border: 1px solid rgba(0, 229, 204, 0.3); "
        "border-radius: 8px; padding: 16px; }");
}

bool FaceEnrollmentWidget::validateUserInput() {
    // Validate user ID
    if (current_user_id_.trimmed().isEmpty()) {
        QMessageBox::warning(this, "Validation Error",
            "Please enter a valid User ID before starting enrollment.");
        ui->user_id_input->setFocus();
        return false;
    }

    // Validate user ID format (alphanumeric and some special characters)
    QRegularExpression regex("^[a-zA-Z0-9._-]+$");
    if (!regex.match(current_user_id_).hasMatch()) {
        QMessageBox::warning(this, "Validation Error",
            "User ID can only contain letters, numbers, dots, underscores, and hyphens.");
        ui->user_id_input->setFocus();
        return false;
    }

    // Validate consent checkboxes
    if (!ui->data_consent_checkbox->isChecked() ||
        !ui->privacy_consent_checkbox->isChecked() ||
        !ui->storage_consent_checkbox->isChecked()) {
        QMessageBox::warning(this, "Consent Required",
            "All consent checkboxes must be checked before starting enrollment.");
        return false;
    }

    return true;
}

void FaceEnrollmentWidget::addAuditTrailEntry(const QString& event, const QString& details) {
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString entry = QString("[%1] %2: %3").arg(timestamp, event, details);
    audit_trail_entries_.append(entry);

    // Limit audit trail size to prevent memory issues
    if (audit_trail_entries_.size() > 100) {
        audit_trail_entries_.removeFirst();
    }

    // Log to system logger if available
    // unlook::core::Logger::info("FaceEnrollment", entry.toStdString());
}

void FaceEnrollmentWidget::updateAccessibilityFeatures() {
    if (accessibility_mode_) {
        // Update focus policy
        setFocusPolicy(Qt::StrongFocus);

        // Enable keyboard navigation for all buttons
        ui->start_enrollment_button->setFocusPolicy(Qt::StrongFocus);
        ui->cancel_button->setFocusPolicy(Qt::StrongFocus);
        ui->manual_capture_button->setFocusPolicy(Qt::StrongFocus);
        ui->skip_sample_button->setFocusPolicy(Qt::StrongFocus);
        ui->cancel_enrollment_button->setFocusPolicy(Qt::StrongFocus);
        ui->finish_button->setFocusPolicy(Qt::StrongFocus);
        ui->test_authentication_button->setFocusPolicy(Qt::StrongFocus);

        // Update high contrast styling if enabled
        if (high_contrast_mode_) {
            setStyleSheet(styleSheet() +
                "QWidget { border: 2px solid #FFFFFF; }"
                "QLabel { border: 1px solid #FFFFFF; }"
            );
        }
    }
}

void FaceEnrollmentWidget::speakGuidanceText(const QString& text) {
    if (!voice_guidance_enabled_ || text.isEmpty()) {
        return;
    }

    last_spoken_text_ = text;

    // Here you would integrate with a text-to-speech system
    // For now, we'll use a simple accessibility announcement
    QAccessible::updateAccessibility(new QAccessibleValueChangeEvent(this, text));
}

QString FaceEnrollmentWidget::qualityScoreToText(float score) {
    if (score >= 0.9f) return "Excellent";
    if (score >= 0.8f) return "Very Good";
    if (score >= 0.7f) return "Good";
    if (score >= 0.6f) return "Fair";
    return "Poor";
}

QString FaceEnrollmentWidget::faceResultCodeToMessage(face::FaceResultCode code) {
    switch (code) {
        case face::FaceResultCode::SUCCESS:
            return "Success";
        case face::FaceResultCode::ERROR_NO_FACE_DETECTED:
            return "No face detected. Please position yourself in front of the camera.";
        case face::FaceResultCode::ERROR_MULTIPLE_FACES_DETECTED:
            return "Multiple faces detected. Please ensure only one person is in view.";
        case face::FaceResultCode::ERROR_FACE_TOO_SMALL:
            return "Face too small. Please move closer to the camera.";
        case face::FaceResultCode::ERROR_FACE_TOO_LARGE:
            return "Face too large. Please move back from the camera.";
        case face::FaceResultCode::ERROR_POOR_FACE_QUALITY:
            return "Face image quality too low. Please improve lighting and focus.";
        case face::FaceResultCode::ERROR_LIVENESS_CHECK_FAILED:
            return "Liveness check failed. Please ensure you are a real person.";
        case face::FaceResultCode::ERROR_CAMERA_NOT_AVAILABLE:
            return "Camera not available. Please check camera connection.";
        default:
            return "An unexpected error occurred. Please try again.";
    }
}

QString FaceEnrollmentWidget::generateRecoverySuggestion(face::FaceResultCode error_code) {
    return EnrollmentGuidanceHelper::generateRecoverySuggestions(error_code, 0);
}

void FaceEnrollmentWidget::applyBankingSecurityStyling() {
    // Add visual security indicators
    setProperty("banking_grade", banking_grade_mode_);

    if (banking_grade_mode_) {
        // Add subtle security-themed styling
        ui->security_info_frame->setStyleSheet(
            ui->security_info_frame->styleSheet() +
            "QFrame { background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
            "stop: 0 rgba(26, 43, 42, 0.9), stop: 1 rgba(0, 139, 122, 0.1)); }"
        );
    }
}

void FaceEnrollmentWidget::updateConsentFormValidation() {
    bool all_consents_given = ui->data_consent_checkbox->isChecked() &&
                             ui->privacy_consent_checkbox->isChecked() &&
                             ui->storage_consent_checkbox->isChecked();

    bool user_id_valid = !ui->user_id_input->text().trimmed().isEmpty();

    bool can_start = all_consents_given && user_id_valid;

    ui->start_enrollment_button->setEnabled(can_start);

    // Update button text to guide user
    if (!user_id_valid) {
        ui->start_enrollment_button->setText("ENTER USER ID");
    } else if (!all_consents_given) {
        ui->start_enrollment_button->setText("PROVIDE CONSENT");
    } else {
        ui->start_enrollment_button->setText("START ENROLLMENT");
    }
}

QString FaceEnrollmentWidget::formatDuration(int seconds) {
    if (seconds < 60) {
        return QString("%1s").arg(seconds);
    } else if (seconds < 3600) {
        return QString("%1m %2s").arg(seconds / 60).arg(seconds % 60);
    } else {
        return QString("%1h %2m").arg(seconds / 3600).arg((seconds % 3600) / 60);
    }
}

// EnrollmentGuidanceHelper implementation
QString EnrollmentGuidanceHelper::generateGuidanceText(const face::EnrollmentProgress& progress,
                                                      bool is_banking_mode) {
    if (!progress.face_detected) {
        return "Please position your face within the camera view.";
    }

    if (progress.face_distance_mm > 0) {
        if (progress.face_distance_mm < 300) {
            return "Move back - you're too close to the camera.";
        } else if (progress.face_distance_mm > 800) {
            return "Move closer - you're too far from the camera.";
        }
    }

    if (progress.lighting_quality < 0.5f) {
        return "Improve lighting - turn on room lights or move to a brighter area.";
    }

    if (!progress.face_angle_optimal) {
        return "Look straight at the camera and keep your head level.";
    }

    if (is_banking_mode && !progress.liveness_confirmed && progress.liveness_checking) {
        return "Hold still while we verify you're a real person...";
    }

    if (progress.quality_score < (is_banking_mode ? 0.8f : 0.7f)) {
        return "Hold steady and look directly at the camera for better quality.";
    }

    if (progress.samples_captured == 0) {
        return "Perfect! Now look straight at the camera for the first sample.";
    }

    return generatePoseInstruction(progress.samples_captured, 5, progress.pose_coverage);
}

QString EnrollmentGuidanceHelper::generatePoseInstruction(int current_sample,
                                                         int total_samples,
                                                         float pose_coverage) {
    QStringList poses = {"straight ahead", "slightly left", "slightly right",
                        "chin slightly up", "chin slightly down"};

    if (current_sample < poses.size()) {
        return QString("Sample %1/%2: Please turn your head %3")
                .arg(current_sample + 1).arg(total_samples).arg(poses[current_sample]);
    }

    return QString("Sample %1/%2: Look at the camera naturally").arg(current_sample + 1).arg(total_samples);
}

QString EnrollmentGuidanceHelper::generateRecoverySuggestions(face::FaceResultCode error_code,
                                                             int consecutive_failures) {
    QString base_suggestion = "Please try the following: ";

    switch (error_code) {
        case face::FaceResultCode::ERROR_NO_FACE_DETECTED:
            base_suggestion += "Position yourself directly in front of the camera, remove any face coverings, and ensure good lighting.";
            break;
        case face::FaceResultCode::ERROR_POOR_FACE_QUALITY:
            base_suggestion += "Improve lighting by turning on room lights, clean the camera lens, and hold still during capture.";
            break;
        case face::FaceResultCode::ERROR_LIVENESS_CHECK_FAILED:
            base_suggestion += "Ensure you are a real person, remove sunglasses or masks, and look directly at the camera.";
            break;
        default:
            base_suggestion += "Check your camera connection, improve lighting, and try again.";
            break;
    }

    if (consecutive_failures >= 3) {
        base_suggestion += " If problems persist, please contact technical support.";
    }

    return base_suggestion;
}

QString EnrollmentGuidanceHelper::formatQualityScore(float score, bool show_percentage) {
    if (show_percentage) {
        return QString("%1%").arg(static_cast<int>(score * 100));
    } else {
        return QString::number(score, 'f', 2);
    }
}

} // namespace gui
} // namespace unlook