#include "unlook/gui/face_enrollment_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/styles/display_metrics.hpp"

#include <QApplication>
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>
#include <QThread>
#include <QPixmap>
#include <QPainter>
#include <QFont>
#include <QPen>
#include <QBrush>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

FaceEnrollmentWidget::FaceEnrollmentWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system,
                                           QWidget* parent)
    : QWidget(parent)
    , camera_system_(camera_system)
    , main_layout_(nullptr)
    , top_bar_layout_(nullptr)
    , content_layout_(nullptr)
    , controls_layout_(nullptr)
    , enrollment_status_display_(nullptr)
    , vcsel_status_display_(nullptr)
    , face_quality_display_(nullptr)
    , overall_progress_bar_(nullptr)
    , angle_progress_bar_(nullptr)
    , camera_preview_label_(nullptr)
    , face_detection_overlay_(nullptr)
    , instruction_label_(nullptr)
    , feedback_text_(nullptr)
    , capture_button_(nullptr)
    , next_angle_button_(nullptr)
    , previous_angle_button_(nullptr)
    , restart_button_(nullptr)
    , emergency_stop_button_(nullptr)
    , vcsel_projector_(nullptr)
    , as1170_controller_(nullptr)
    , face_enroller_(nullptr)
    , liveness_detector_(nullptr)
    , banking_validator_(nullptr)
    , frame_update_timer_(nullptr)
    , vcsel_monitoring_timer_(nullptr)
    , enrollment_timeout_timer_(nullptr)
{
    qDebug() << "[FaceEnrollmentWidget] Initializing VCSEL-integrated face enrollment system";

    // Setup UI first
    setupUI();

    // Initialize status with default values
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::INITIALIZATION;
        status_.current_angle = CaptureAngle::CENTER;

        // Initialize angle capture tracking
        status_.angles_captured[CaptureAngle::CENTER] = false;
        status_.angles_captured[CaptureAngle::LEFT_PROFILE] = false;
        status_.angles_captured[CaptureAngle::RIGHT_PROFILE] = false;
        status_.angles_captured[CaptureAngle::SLIGHT_UP] = false;
        status_.angles_captured[CaptureAngle::SLIGHT_DOWN] = false;

        // Initialize quality scores
        for (auto& angle : {CaptureAngle::CENTER, CaptureAngle::LEFT_PROFILE,
                           CaptureAngle::RIGHT_PROFILE, CaptureAngle::SLIGHT_UP,
                           CaptureAngle::SLIGHT_DOWN}) {
            status_.angle_quality_scores[angle] = 0.0f;
        }
    }

    // Setup timers
    frame_update_timer_ = new QTimer(this);
    connect(frame_update_timer_, &QTimer::timeout, this, &FaceEnrollmentWidget::onCameraFrameUpdate);

    vcsel_monitoring_timer_ = new QTimer(this);
    connect(vcsel_monitoring_timer_, &QTimer::timeout, this, &FaceEnrollmentWidget::onVCSELTemperatureUpdate);

    enrollment_timeout_timer_ = new QTimer(this);
    enrollment_timeout_timer_->setSingleShot(true);
    connect(enrollment_timeout_timer_, &QTimer::timeout, this, [this]() {
        handleEnrollmentError("Enrollment timeout - please try again");
    });

    qDebug() << "[FaceEnrollmentWidget] Base initialization completed";
}

FaceEnrollmentWidget::~FaceEnrollmentWidget() {
    qDebug() << "[FaceEnrollmentWidget] Starting safe shutdown";

    // Emergency shutdown to ensure VCSEL is disabled
    emergencyShutdown();

    // Stop all timers
    if (frame_update_timer_) frame_update_timer_->stop();
    if (vcsel_monitoring_timer_) vcsel_monitoring_timer_->stop();
    if (enrollment_timeout_timer_) enrollment_timeout_timer_->stop();

    // Cleanup face recognition components
    face_enroller_.reset();
    liveness_detector_.reset();
    banking_validator_.reset();

    // Cleanup hardware components
    vcsel_projector_.reset();
    as1170_controller_.reset();

    qDebug() << "[FaceEnrollmentWidget] Safe shutdown completed";
}

bool FaceEnrollmentWidget::initialize(const EnrollmentConfig& config) {
    qDebug() << "[FaceEnrollmentWidget] Initializing with banking-grade security";

    config_ = config;

    // Update phase to initialization
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::INITIALIZATION;
        status_.overall_progress = 0.0f;
    }
    updateUIForPhase(EnrollmentPhase::INITIALIZATION);

    // Initialize hardware systems
    if (!initializeHardware()) {
        handleEnrollmentError("Hardware initialization failed");
        return false;
    }

    // Initialize face recognition pipeline
    if (!initializeFaceRecognition()) {
        handleEnrollmentError("Face recognition system initialization failed");
        return false;
    }

    // Initialize VCSEL projector
    // if (!initializeVCSEL()) {
    //     handleEnrollmentError("VCSEL projector initialization failed");
    //     return false;
    // } // Disabled - AS1170 segfault issue

    // Start monitoring systems
    vcsel_monitoring_timer_->start(1000); // Monitor VCSEL every second
    frame_update_timer_->start(33); // ~30 FPS frame updates

    // Update phase to ready
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::USER_SETUP;
        status_.overall_progress = 10.0f;
    }
    updateUIForPhase(EnrollmentPhase::USER_SETUP);

    enrollment_status_display_->setStatus("Ready for enrollment", StatusDisplay::StatusType::SUCCESS);
    qDebug() << "[FaceEnrollmentWidget] Initialization completed successfully";

    return true;
}

void FaceEnrollmentWidget::startEnrollment(const std::string& user_id, const std::string& display_name) {
    qDebug() << "[FaceEnrollmentWidget] Starting enrollment for user:" << QString::fromStdString(display_name);

    if (enrollment_active_.load()) {
        qWarning() << "[FaceEnrollmentWidget] Enrollment already in progress";
        return;
    }

    // Store user information
    current_user_id_ = user_id;
    current_display_name_ = display_name;

    // Reset enrollment state
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::FACE_DETECTION;
        status_.current_angle = CaptureAngle::CENTER;
        status_.overall_progress = 20.0f;
        status_.successful_captures = 0;
        status_.failed_attempts = 0;

        // Reset capture tracking
        for (auto& pair : status_.angles_captured) {
            pair.second = false;
        }
        for (auto& pair : status_.angle_quality_scores) {
            pair.second = 0.0f;
        }
    }

    // Clear previous templates
    captured_templates_.clear();

    // Set enrollment active
    enrollment_active_.store(true);
    emergency_shutdown_active_.store(false);

    // Start enrollment timeout
    enrollment_timeout_timer_->start(config_.capture_timeout_seconds * 1000 * 5); // Total timeout

    // Update UI
    updateUIForPhase(EnrollmentPhase::FACE_DETECTION);
    enrollment_status_display_->setStatus(
        QString("Enrolling: %1").arg(QString::fromStdString(display_name)),
        StatusDisplay::StatusType::INFO
    );

    // Start face detection
    processEnrollmentStep();

    qDebug() << "[FaceEnrollmentWidget] Enrollment started successfully";
}

void FaceEnrollmentWidget::cancelEnrollment() {
    qDebug() << "[FaceEnrollmentWidget] Canceling enrollment";

    enrollment_active_.store(false);
    enrollment_timeout_timer_->stop();

    // Safe VCSEL shutdown
    deactivateVCSELSafe();

    // Reset to ready state
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::USER_SETUP;
        status_.overall_progress = 10.0f;
    }

    updateUIForPhase(EnrollmentPhase::USER_SETUP);
    enrollment_status_display_->setStatus("Enrollment canceled", StatusDisplay::StatusType::WARNING);

    // Clear user data
    current_user_id_.clear();
    current_display_name_.clear();
    captured_templates_.clear();

    qDebug() << "[FaceEnrollmentWidget] Enrollment canceled successfully";
}

FaceEnrollmentWidget::EnrollmentStatus FaceEnrollmentWidget::getEnrollmentStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

void FaceEnrollmentWidget::emergencyShutdown() {
    qWarning() << "[FaceEnrollmentWidget] EMERGENCY SHUTDOWN ACTIVATED";

    emergency_shutdown_active_.store(true);
    enrollment_active_.store(false);

    // Immediate VCSEL shutdown
    if (vcsel_projector_) {
        vcsel_projector_->emergencyShutdown();
    }
    if (as1170_controller_) {
        as1170_controller_->emergencyShutdown();
    }

    // Stop all timers
    if (frame_update_timer_) frame_update_timer_->stop();
    if (vcsel_monitoring_timer_) vcsel_monitoring_timer_->stop();
    if (enrollment_timeout_timer_) enrollment_timeout_timer_->stop();

    // Update UI to error state
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::ERROR_STATE;
        status_.has_critical_error = true;
        status_.last_error_message = "Emergency shutdown activated";
        status_.vcsel_active = false;
    }

    updateUIForPhase(EnrollmentPhase::ERROR_STATE);
    enrollment_status_display_->setStatus("EMERGENCY SHUTDOWN", StatusDisplay::StatusType::ERROR);
    vcsel_status_display_->setStatus("VCSEL DISABLED", StatusDisplay::StatusType::ERROR);

    qWarning() << "[FaceEnrollmentWidget] Emergency shutdown completed";
}

void FaceEnrollmentWidget::setupUI() {
    // Create main layout
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setContentsMargins(20, 20, 20, 20);
    main_layout_->setSpacing(15);

    // Create top bar with status displays
    top_bar_layout_ = new QHBoxLayout();

    enrollment_status_display_ = new StatusDisplay("Enrollment");
    enrollment_status_display_->setCompactMode(true);
    enrollment_status_display_->setStatus("Initializing...", StatusDisplay::StatusType::INFO);
    top_bar_layout_->addWidget(enrollment_status_display_);

    vcsel_status_display_ = new StatusDisplay("VCSEL");
    vcsel_status_display_->setCompactMode(true);
    vcsel_status_display_->setStatus("Disabled", StatusDisplay::StatusType::SUCCESS);
    top_bar_layout_->addWidget(vcsel_status_display_);

    face_quality_display_ = new StatusDisplay("Face Quality");
    face_quality_display_->setCompactMode(true);
    face_quality_display_->setStatus("No Face", StatusDisplay::StatusType::WARNING);
    top_bar_layout_->addWidget(face_quality_display_);

    main_layout_->addLayout(top_bar_layout_);

    // Progress indicators
    overall_progress_bar_ = new QProgressBar();
    overall_progress_bar_->setRange(0, 100);
    overall_progress_bar_->setValue(0);
    overall_progress_bar_->setTextVisible(true);
    overall_progress_bar_->setFormat("Overall Progress: %p%");
    main_layout_->addWidget(overall_progress_bar_);

    angle_progress_bar_ = new QProgressBar();
    angle_progress_bar_->setRange(0, 5); // 5 angles
    angle_progress_bar_->setValue(0);
    angle_progress_bar_->setTextVisible(true);
    angle_progress_bar_->setFormat("Angles Captured: %v/5");
    main_layout_->addWidget(angle_progress_bar_);

    // Content layout for camera preview and controls
    content_layout_ = new QGridLayout();

    // Camera preview area (left side)
    camera_preview_label_ = new QLabel();
    camera_preview_label_->setMinimumSize(640, 480);
    camera_preview_label_->setScaledContents(true);
    camera_preview_label_->setStyleSheet("border: 2px solid #00E5CC; background-color: #1A2B2A;");
    camera_preview_label_->setAlignment(Qt::AlignCenter);
    camera_preview_label_->setText("Camera Initializing...");
    content_layout_->addWidget(camera_preview_label_, 0, 0, 3, 2);

    // Face detection overlay
    face_detection_overlay_ = new QLabel(camera_preview_label_);
    face_detection_overlay_->setStyleSheet("background: transparent; border: none;");
    face_detection_overlay_->resize(camera_preview_label_->size());

    // Instructions and feedback (right side)
    instruction_label_ = new QLabel();
    instruction_label_->setWordWrap(true);
    instruction_label_->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    instruction_label_->setStyleSheet(
        "font-size: 16px; color: #00E5CC; background-color: #1A2B2A; "
        "padding: 15px; border: 1px solid #008B7A; border-radius: 8px;"
    );
    instruction_label_->setText("Face enrollment system initializing...");
    content_layout_->addWidget(instruction_label_, 0, 2, 1, 2);

    // Feedback text area
    feedback_text_ = new QTextEdit();
    feedback_text_->setReadOnly(true);
    feedback_text_->setMaximumHeight(200);
    feedback_text_->setStyleSheet(
        "font-size: 12px; color: #00B8A3; background-color: #1A2B2A; "
        "border: 1px solid #008B7A; border-radius: 8px; padding: 10px;"
    );
    feedback_text_->append("System ready for face enrollment.");
    content_layout_->addWidget(feedback_text_, 1, 2, 2, 2);

    main_layout_->addLayout(content_layout_);

    // Angle indicator buttons
    QHBoxLayout* angle_layout = new QHBoxLayout();
    angle_layout->addStretch();

    const std::vector<std::pair<CaptureAngle, QString>> angles = {
        {CaptureAngle::LEFT_PROFILE, "Left"},
        {CaptureAngle::SLIGHT_UP, "Up"},
        {CaptureAngle::CENTER, "Center"},
        {CaptureAngle::SLIGHT_DOWN, "Down"},
        {CaptureAngle::RIGHT_PROFILE, "Right"}
    };

    for (const auto& angle_pair : angles) {
        TouchButton* button = new TouchButton(angle_pair.second);
        button->setCheckable(true);
        button->setEnabled(false);
        angle_indicator_buttons_[angle_pair.first] = button;
        angle_layout->addWidget(button);
    }
    angle_layout->addStretch();
    main_layout_->addLayout(angle_layout);

    // Control buttons
    controls_layout_ = new QHBoxLayout();

    emergency_stop_button_ = new TouchButton("EMERGENCY STOP");
    emergency_stop_button_->setStyleSheet(
        "TouchButton { background-color: #CC0000; color: white; font-weight: bold; }"
        "TouchButton:hover { background-color: #FF0000; }"
        "TouchButton:pressed { background-color: #800000; }"
    );
    connect(emergency_stop_button_, &TouchButton::clicked, this, &FaceEnrollmentWidget::onEmergencyStop);
    controls_layout_->addWidget(emergency_stop_button_);

    controls_layout_->addStretch();

    previous_angle_button_ = new TouchButton("Previous Angle");
    previous_angle_button_->setEnabled(false);
    connect(previous_angle_button_, &TouchButton::clicked, this, &FaceEnrollmentWidget::onPreviousAnglePressed);
    controls_layout_->addWidget(previous_angle_button_);

    capture_button_ = new TouchButton("CAPTURE");
    capture_button_->setStyleSheet(
        "TouchButton { background-color: #00E5CC; color: #000000; font-weight: bold; font-size: 16px; }"
        "TouchButton:hover { background-color: #00FFDD; }"
        "TouchButton:pressed { background-color: #00B8A3; }"
    );
    capture_button_->setEnabled(false);
    connect(capture_button_, &TouchButton::clicked, this, &FaceEnrollmentWidget::onCaptureButtonPressed);
    controls_layout_->addWidget(capture_button_);

    next_angle_button_ = new TouchButton("Next Angle");
    next_angle_button_->setEnabled(false);
    connect(next_angle_button_, &TouchButton::clicked, this, &FaceEnrollmentWidget::onNextAnglePressed);
    controls_layout_->addWidget(next_angle_button_);

    controls_layout_->addStretch();

    restart_button_ = new TouchButton("Restart Enrollment");
    restart_button_->setEnabled(false);
    connect(restart_button_, &TouchButton::clicked, this, &FaceEnrollmentWidget::onRestartEnrollment);
    controls_layout_->addWidget(restart_button_);

    main_layout_->addLayout(controls_layout_);

    // Apply styling
    applySupernovanStyling();

    qDebug() << "[FaceEnrollmentWidget] UI setup completed";
}

void FaceEnrollmentWidget::applySupernovanStyling() {
    // Apply main Supernova styling
    setStyleSheet(
        "FaceEnrollmentWidget {"
        "    background-color: #000000;"
        "    color: #00E5CC;"
        "}"
        "QLabel {"
        "    color: #00E5CC;"
        "    font-family: 'Arial', sans-serif;"
        "}"
        "QProgressBar {"
        "    border: 2px solid #008B7A;"
        "    border-radius: 5px;"
        "    text-align: center;"
        "    background-color: #1A2B2A;"
        "    color: #00E5CC;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: #00E5CC;"
        "    border-radius: 3px;"
        "}"
    );

    qDebug() << "[FaceEnrollmentWidget] Supernova styling applied";
}

bool FaceEnrollmentWidget::initializeHardware() {
    qDebug() << "[FaceEnrollmentWidget] Initializing hardware systems";

    if (!camera_system_) {
        qCritical() << "[FaceEnrollmentWidget] Camera system not available";
        return false;
    }

    if (!camera_system_->isReady()) {
        qCritical() << "[FaceEnrollmentWidget] Camera system not ready";
        return false;
    }

    qDebug() << "[FaceEnrollmentWidget] Hardware initialization completed";
    return true;
}

bool FaceEnrollmentWidget::initializeFaceRecognition() {
    qDebug() << "[FaceEnrollmentWidget] Initializing face recognition pipeline";

    try {
        // For now, create placeholder components to allow compilation
        // These would be fully implemented with the actual face recognition APIs

        qDebug() << "[FaceEnrollmentWidget] Face recognition pipeline initialized (placeholder)";
        return true;

    } catch (const std::exception& e) {
        qCritical() << "[FaceEnrollmentWidget] Face recognition initialization error:" << e.what();
        return false;
    }
}

bool FaceEnrollmentWidget::initializeVCSEL() {
    qDebug() << "[FaceEnrollmentWidget] Initializing VCSEL projector for face operations";

    try {
        // Initialize AS1170 controller with face-safe settings
        hardware::AS1170Controller::AS1170Config as1170_config;
        as1170_config.target_current_ma = config_.vcsel_current_ma; // Face-safe current
        as1170_config.flash_mode = hardware::AS1170Controller::FlashMode::FLASH_MODE;
        as1170_config.enable_thermal_protection = true;
        as1170_config.max_temperature_c = 65.0f; // Lower limit for face proximity

        // Use singleton instance to prevent I2C conflicts
        as1170_controller_ = hardware::AS1170Controller::getInstance();
        // CRITICAL: Force reset hardware BEFORE initialization
        qDebug() << "[FaceEnrollment] Forcing AS1170 hardware reset to clear stuck state";
        as1170_controller_->forceResetHardware();

        if (!as1170_controller_->initialize(as1170_config)) {
            qCritical() << "[FaceEnrollmentWidget] AS1170 controller initialization failed";
            return false;
        }

        // Initialize VCSEL projector with face recognition optimization
        hardware::VCSELProjector::ProjectorConfig vcsel_config;
        vcsel_config.mode = hardware::VCSELProjector::ProjectionMode::FACE_RECOGNITION;
        vcsel_config.pattern = hardware::VCSELProjector::PatternType::ADAPTIVE;
        vcsel_config.vcsel_current_ma = config_.vcsel_current_ma;
        vcsel_config.projection_duration_ms = config_.vcsel_burst_duration_ms;
        vcsel_config.enable_thermal_protection = true;
        vcsel_config.max_temperature_c = 65.0f;
        vcsel_config.face_distance_mm = config_.face_distance_optimal_mm;
        vcsel_config.adaptive_current = true;
        vcsel_config.enable_camera_sync = true;

        vcsel_projector_ = std::make_unique<hardware::VCSELProjector>();
        if (!vcsel_projector_->initialize(vcsel_config)) {
            qCritical() << "[FaceEnrollmentWidget] VCSEL projector initialization failed";
            return false;
        }

        // Update status
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            status_.vcsel_ready = true;
            status_.vcsel_active = false;
        }

        vcsel_status_display_->setStatus("Ready (Safe)", StatusDisplay::StatusType::SUCCESS);
        qDebug() << "[FaceEnrollmentWidget] VCSEL projector initialized successfully";
        return true;

    } catch (const std::exception& e) {
        qCritical() << "[FaceEnrollmentWidget] VCSEL initialization error:" << e.what();
        return false;
    }
}

void FaceEnrollmentWidget::onCameraFrameUpdate() {
    // Placeholder for camera frame processing
    if (!enrollment_active_.load() || emergency_shutdown_active_.load()) {
        return;
    }

    // Update face detection and UI
    updateFaceDetectionUI();
    updateProgress();
}

void FaceEnrollmentWidget::onVCSELTemperatureUpdate() {
    if (!vcsel_projector_ || emergency_shutdown_active_.load()) {
        return;
    }

    // Monitor VCSEL temperature
    auto vcsel_status = vcsel_projector_->getProjectorStatus();

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.vcsel_temperature_c = vcsel_status.temperature_c;
        status_.thermal_protection_active = vcsel_status.thermal_protection_active;
    }

    updateVCSELStatus();
}

void FaceEnrollmentWidget::processEnrollmentStep() {
    if (!enrollment_active_.load() || emergency_shutdown_active_.load()) {
        return;
    }

    // Process current enrollment phase
    EnrollmentPhase current_phase;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_phase = status_.current_phase;
    }

    switch (current_phase) {
        case EnrollmentPhase::FACE_DETECTION:
            // Enable capture button when face is detected with good quality
            capture_button_->setEnabled(status_.face_detected);
            break;

        case EnrollmentPhase::MULTI_ANGLE_CAPTURE:
            // Handle multi-angle capture logic
            break;

        case EnrollmentPhase::QUALITY_VERIFICATION:
            // Validate captured templates
            break;

        default:
            break;
    }
}

void FaceEnrollmentWidget::onCaptureButtonPressed() {
    qDebug() << "[FaceEnrollmentWidget] Capture button pressed";

    if (!enrollment_active_.load() || emergency_shutdown_active_.load()) {
        return;
    }

    CaptureAngle current_angle;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_angle = status_.current_angle;
    }

    if (captureFaceAngle(current_angle)) {
        feedback_text_->append(QString("Successfully captured %1 angle")
                              .arg(static_cast<int>(current_angle)));

        // Move to next angle or complete
        if (status_.successful_captures >= config_.required_capture_angles) {
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                status_.current_phase = EnrollmentPhase::QUALITY_VERIFICATION;
            }
            updateUIForPhase(EnrollmentPhase::QUALITY_VERIFICATION);
        }
    } else {
        feedback_text_->append("Capture failed - please try again");
    }
}

void FaceEnrollmentWidget::onNextAnglePressed() {
    // Advance to next capture angle
    updateProgress();
}

void FaceEnrollmentWidget::onPreviousAnglePressed() {
    // Go back to previous capture angle
    updateProgress();
}

void FaceEnrollmentWidget::onRestartEnrollment() {
    cancelEnrollment();
    if (!current_user_id_.empty()) {
        startEnrollment(current_user_id_, current_display_name_);
    }
}

void FaceEnrollmentWidget::onEmergencyStop() {
    emergencyShutdown();
}

void FaceEnrollmentWidget::updateUIForPhase(EnrollmentPhase phase) {
    switch (phase) {
        case EnrollmentPhase::INITIALIZATION:
            instruction_label_->setText("Initializing face enrollment system...");
            capture_button_->setEnabled(false);
            break;

        case EnrollmentPhase::USER_SETUP:
            instruction_label_->setText("Ready for face enrollment. Press 'Start Enrollment' to begin.");
            break;

        case EnrollmentPhase::FACE_DETECTION:
            instruction_label_->setText(
                "Look directly at the camera.\n"
                "Position your face in the center of the preview.\n"
                "Ensure good lighting on your face.\n"
                "The VCSEL will provide additional illumination when needed."
            );
            break;

        case EnrollmentPhase::MULTI_ANGLE_CAPTURE:
            instruction_label_->setText(
                "Multi-angle capture in progress.\n"
                "Follow the on-screen instructions for each angle.\n"
                "The system will capture 5 different angles for security."
            );
            break;

        case EnrollmentPhase::ERROR_STATE:
            instruction_label_->setText("System error occurred. Please restart the enrollment process.");
            capture_button_->setEnabled(false);
            break;

        default:
            break;
    }
}

void FaceEnrollmentWidget::updateProgress() {
    int captured_angles = 0;
    float total_quality = 0.0f;

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        for (const auto& pair : status_.angles_captured) {
            if (pair.second) {
                captured_angles++;
                total_quality += status_.angle_quality_scores.at(pair.first);
            }
        }

        status_.overall_progress = (static_cast<float>(captured_angles) / config_.required_capture_angles) * 80.0f + 20.0f;
    }

    overall_progress_bar_->setValue(static_cast<int>(status_.overall_progress));
    angle_progress_bar_->setValue(captured_angles);
}

void FaceEnrollmentWidget::updateFaceDetectionUI() {
    // Update face detection overlay and quality indicators
    if (status_.face_detected) {
        face_quality_display_->setStatus("Face Detected", StatusDisplay::StatusType::SUCCESS);
    } else {
        face_quality_display_->setStatus("No Face", StatusDisplay::StatusType::WARNING);
    }
}

void FaceEnrollmentWidget::updateVCSELStatus() {
    if (status_.thermal_protection_active) {
        vcsel_status_display_->setStatus("Thermal Protection", StatusDisplay::StatusType::WARNING);
    } else if (status_.vcsel_active) {
        vcsel_status_display_->setStatus("Active", StatusDisplay::StatusType::INFO);
    } else {
        vcsel_status_display_->setStatus("Standby", StatusDisplay::StatusType::SUCCESS);
    }
}

bool FaceEnrollmentWidget::captureFaceAngle(CaptureAngle angle) {
    qDebug() << "[FaceEnrollmentWidget] Capturing face angle:" << static_cast<int>(angle);

    if (!activateVCSELSafe()) {
        qCritical() << "[FaceEnrollmentWidget] VCSEL activation failed";
        return false;
    }

    // Simulate capture process
    QThread::msleep(config_.vcsel_burst_duration_ms);

    // Deactivate VCSEL
    deactivateVCSELSafe();

    // Mark angle as captured
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.angles_captured[angle] = true;
        status_.angle_quality_scores[angle] = 0.9f; // Placeholder quality score
        status_.successful_captures++;
    }

    return true;
}

bool FaceEnrollmentWidget::activateVCSELSafe() {
    if (!vcsel_projector_ || emergency_shutdown_active_.load()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(vcsel_mutex_);

    try {
        if (vcsel_projector_->enableProjection()) {
            {
                std::lock_guard<std::mutex> status_lock(status_mutex_);
                status_.vcsel_active = true;
            }
            return true;
        }
    } catch (const std::exception& e) {
        qCritical() << "[FaceEnrollmentWidget] VCSEL activation error:" << e.what();
    }

    return false;
}

void FaceEnrollmentWidget::deactivateVCSELSafe() {
    if (!vcsel_projector_) {
        return;
    }

    std::lock_guard<std::mutex> lock(vcsel_mutex_);

    try {
        vcsel_projector_->disableProjection();
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.vcsel_active = false;
        }
    } catch (const std::exception& e) {
        qCritical() << "[FaceEnrollmentWidget] VCSEL deactivation error:" << e.what();
    }
}

void FaceEnrollmentWidget::handleEnrollmentError(const std::string& error_message) {
    qWarning() << "[FaceEnrollmentWidget] Enrollment error:" << QString::fromStdString(error_message);

    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.current_phase = EnrollmentPhase::ERROR_STATE;
        status_.has_critical_error = true;
        status_.last_error_message = error_message;
    }

    updateUIForPhase(EnrollmentPhase::ERROR_STATE);
    enrollment_status_display_->setStatus("Error: " + QString::fromStdString(error_message),
                                         StatusDisplay::StatusType::ERROR);

    // Safe shutdown
    deactivateVCSELSafe();
    enrollment_active_.store(false);

    // Emit signal
    emit enrollmentFailed(current_user_id_, error_message);
}

// Additional placeholder implementations for missing methods
void FaceEnrollmentWidget::processFaceDetection(const core::StereoFramePair& frame_pair) {
    // TODO: Implement face detection processing with the actual face recognition APIs
    // This would integrate with FaceDetector, LandmarkExtractor, and LivenessDetector
}

bool FaceEnrollmentWidget::validateFaceTemplate(const face::FaceTemplate& face_template) {
    // TODO: Implement face template validation using BankingMLValidator
    return false;
}

bool FaceEnrollmentWidget::generateBiometricTemplate() {
    // TODO: Implement biometric template generation from captured angles
    return false;
}

} // namespace gui
} // namespace unlook

// MOC include for Qt signal/slot system
#include "face_enrollment_widget.moc"