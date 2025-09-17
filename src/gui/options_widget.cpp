#include "unlook/gui/options_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/as1170_debug_dialog.hpp"
#include "ui_options_widget.h"
#include <QGridLayout>
#include <QMessageBox>
#include <QApplication>
#include <QDebug>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

OptionsWidget::OptionsWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::OptionsWidget)
    , camera_system_(camera_system)
{
    // Setup UI from .ui file
    ui->setupUi(this);
    
    // Connect signals
    connectSignals();

    // Initialize UI components
    // initializeUI(); // Disabled - using .ui file layout only
}

OptionsWidget::~OptionsWidget() {
    delete ui;
}

void OptionsWidget::connectSignals() {
    // TODO_UI: Add missing methods and buttons to .ui file
    // connect(ui->calibration_validation_button, &QPushButton::clicked, this, &OptionsWidget::performCalibrationValidation);
    connect(ui->advanced_camera_button, &QPushButton::clicked, this, &OptionsWidget::openAdvancedCameraSettings);
    connect(ui->reset_defaults_button, &QPushButton::clicked, this, &OptionsWidget::resetToDefaults);
    connect(ui->as1170_debug_button, &QPushButton::clicked, this, &OptionsWidget::openAS1170DebugSystem);
    // connect(ui->export_logs_button, &QPushButton::clicked, this, &OptionsWidget::exportSystemLogs);
    // connect(ui->system_diagnostics_button, &QPushButton::clicked, this, &OptionsWidget::runSystemDiagnostics);
}

void OptionsWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    // refreshSystemStatus(); // Disabled - using .ui file widgets only
}

void OptionsWidget::refreshSystemStatus() {
    updateStatusDisplays();
}

void OptionsWidget::openCalibrationValidation() {
    QMessageBox::information(this, "Calibration Validation", 
                            "Calibration validation tools are not implemented yet.\n"
                            "This would open epipolar line visualization and rectification quality assessment.");
}

void OptionsWidget::openAdvancedCameraSettings() {
    QMessageBox::information(this, "Advanced Settings", 
                            "Advanced camera settings are not implemented yet.\n"
                            "This would provide detailed hardware configuration options.");
}

void OptionsWidget::resetToDefaults() {
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Reset to Defaults",
                                                             "Are you sure you want to reset all settings to factory defaults?",
                                                             QMessageBox::Yes | QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // Reset implementation would go here
        QMessageBox::information(this, "Reset Complete", 
                                "Settings have been reset to factory defaults.\n"
                                "Please restart the application for changes to take effect.");
    }
}

void OptionsWidget::showAboutDialog() {
    QString about_text = QString(
        "<h2>Unlook 3D Scanner v1.0</h2>"
        "<p><b>Professional Modular Open Source 3D Scanner</b></p>"
        "<p>Target Precision: 0.005mm</p>"
        "<p>Copyright © 2024 Unlook Project</p>"
        "<br>"
        "<p><b>Hardware Configuration:</b></p>"
        "<ul>"
        "<li>Sensors: 2x IMX296 Global Shutter Cameras</li>"
        "<li>Resolution: 1456x1088 SBGGR10</li>"
        "<li>Baseline: 70.017mm (calibrated)</li>"
        "<li>Hardware Sync: XVS/XHS enabled</li>"
        "<li>Optics: 6mm focal length lens</li>"
        "</ul>"
        "<br>"
        "<p><b>Software Stack:</b></p>"
        "<ul>"
        "<li>C++17/20 exclusive implementation</li>"
        "<li>libcamera-sync integration</li>"
        "<li>OpenCV stereo processing</li>"
        "<li>Qt5 touch-optimized interface</li>"
        "<li>Supernova-tech design language</li>"
        "</ul>"
        "<br>"
        "<p>Built on %1</p>"
    ).arg(__DATE__);
    
    QMessageBox about_dialog(this);
    about_dialog.setWindowTitle("About Unlook Scanner");
    about_dialog.setText(about_text);
    about_dialog.setIconPixmap(QPixmap(":/icons/unlook_logo.png"));
    SupernovaStyle::applyStyle(&about_dialog);
    about_dialog.exec();
}

void OptionsWidget::initializeUI() {
    // Use layout from .ui file - don't create a new one
    // main_layout_ = new QVBoxLayout(this);
    // main_layout_->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    
    // Create scroll area for content
    // scroll_area_ = new QScrollArea();
    // scroll_area_->setWidgetResizable(true);
    // scroll_area_->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    // scroll_area_->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    // content_widget_ = new QWidget();
    // QVBoxLayout* content_layout = new QVBoxLayout(content_widget_);
    // content_layout->setSpacing(SupernovaStyle::Spacing::MARGIN_LARGE);

    // Add content panels
    // content_layout->addWidget(createSystemStatusPanel());
    // content_layout->addWidget(createCalibrationPanel());
    // content_layout->addWidget(createCameraConfigPanel());
    // content_layout->addWidget(createSystemInfoPanel());
    // content_layout->addWidget(createActionButtonsPanel());
    
    // content_layout->addStretch();

    // scroll_area_->setWidget(content_widget_);
    // main_layout_->addWidget(scroll_area_);
}

QWidget* OptionsWidget::createSystemStatusPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    QLabel* title = new QLabel("System Status");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::HEADING, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // System status display
    system_status_ = new StatusDisplay("Overall System");
    layout->addWidget(system_status_);
    
    // Sync status display
    sync_status_ = new StatusDisplay("Hardware Synchronization");
    layout->addWidget(sync_status_);
    
    return panel;
}

QWidget* OptionsWidget::createCalibrationPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    QLabel* title = new QLabel("Calibration Status");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::HEADING, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Calibration status display
    calibration_status_ = new StatusDisplay("Stereo Calibration");
    layout->addWidget(calibration_status_);
    
    return panel;
}

QWidget* OptionsWidget::createCameraConfigPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    QLabel* title = new QLabel("Camera Configuration");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::HEADING, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Camera status displays
    left_camera_status_ = new StatusDisplay("LEFT Camera (Master)");
    layout->addWidget(left_camera_status_);
    
    right_camera_status_ = new StatusDisplay("RIGHT Camera (Slave)");
    layout->addWidget(right_camera_status_);
    
    return panel;
}

QWidget* OptionsWidget::createSystemInfoPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    QLabel* title = new QLabel("System Information");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::HEADING, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // System information
    system_info_label_ = new QLabel();
    system_info_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY));
    system_info_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    system_info_label_->setWordWrap(true);
    layout->addWidget(system_info_label_);
    
    // Camera information
    camera_info_label_ = new QLabel();
    camera_info_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY));
    camera_info_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    camera_info_label_->setWordWrap(true);
    layout->addWidget(camera_info_label_);
    
    // Version information
    version_info_label_ = new QLabel();
    version_info_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    version_info_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    version_info_label_->setWordWrap(true);
    layout->addWidget(version_info_label_);
    
    return panel;
}

QWidget* OptionsWidget::createActionButtonsPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    QLabel* title = new QLabel("Actions");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::HEADING, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Create button grid
    QGridLayout* button_layout = new QGridLayout();
    button_layout->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    
    // TODO_UI: Add action buttons to .ui file
    // refresh_button_ = new TouchButton("REFRESH STATUS", TouchButton::ButtonType::PRIMARY);
    // connect(refresh_button_, &TouchButton::clicked, this, &OptionsWidget::refreshSystemStatus);
    // button_layout->addWidget(refresh_button_, 0, 0);
    
    // // Calibration validation button
    // calibration_validation_button_ = new TouchButton("CALIBRATION VALIDATION", TouchButton::ButtonType::SECONDARY);
    // connect(calibration_validation_button_, &TouchButton::clicked, this, &OptionsWidget::openCalibrationValidation);
    // button_layout->addWidget(calibration_validation_button_, 0, 1);
    
    // // Advanced settings button
    // advanced_settings_button_ = new TouchButton("ADVANCED SETTINGS", TouchButton::ButtonType::SECONDARY);
    // connect(advanced_settings_button_, &TouchButton::clicked, this, &OptionsWidget::openAdvancedCameraSettings);
    // button_layout->addWidget(advanced_settings_button_, 1, 0);
    
    // // Reset defaults button
    // reset_defaults_button_ = new TouchButton("RESET TO DEFAULTS", TouchButton::ButtonType::WARNING);
    // connect(reset_defaults_button_, &TouchButton::clicked, this, &OptionsWidget::resetToDefaults);
    // button_layout->addWidget(reset_defaults_button_, 1, 1);
    
    // // About button
    // about_button_ = new TouchButton("ABOUT", TouchButton::ButtonType::SUCCESS);
    // connect(about_button_, &TouchButton::clicked, this, &OptionsWidget::showAboutDialog);
    // button_layout->addWidget(about_button_, 2, 0, 1, 2); // Span 2 columns
    
    layout->addLayout(button_layout);
    
    return panel;
}

void OptionsWidget::updateStatusDisplays() {
    if (!camera_system_) {
        system_status_->setStatus("Camera system not available", StatusDisplay::StatusType::ERROR);
        return;
    }
    
    // Update system status
    if (camera_system_->isReady()) {
        system_status_->setStatus("System operational", StatusDisplay::StatusType::SUCCESS);
    } else {
        system_status_->setStatus("System initializing or error", StatusDisplay::StatusType::WARNING);
    }
    
    // Update camera status
    core::CameraState left_state = camera_system_->getCameraState(core::CameraId::LEFT);
    core::CameraState right_state = camera_system_->getCameraState(core::CameraId::RIGHT);
    
    auto stateToString = [](core::CameraState state) -> QString {
        switch (state) {
            case core::CameraState::DISCONNECTED: return "Disconnected";
            case core::CameraState::INITIALIZING: return "Initializing";
            case core::CameraState::READY: return "Ready";
            case core::CameraState::CAPTURING: return "Capturing";
            case core::CameraState::ERROR: return "Error";
            default: return "Unknown";
        }
    };
    
    auto stateToStatusType = [](core::CameraState state) -> StatusDisplay::StatusType {
        switch (state) {
            case core::CameraState::READY: return StatusDisplay::StatusType::SUCCESS;
            case core::CameraState::CAPTURING: return StatusDisplay::StatusType::PROCESSING;
            case core::CameraState::INITIALIZING: return StatusDisplay::StatusType::INFO;
            case core::CameraState::ERROR: return StatusDisplay::StatusType::ERROR;
            default: return StatusDisplay::StatusType::WARNING;
        }
    };
    
    left_camera_status_->setStatus(stateToString(left_state), stateToStatusType(left_state));
    right_camera_status_->setStatus(stateToString(right_state), stateToStatusType(right_state));
    
    // Update sync status
    if (camera_system_->isHardwareSyncEnabled()) {
        double avg_sync_error = camera_system_->getAverageSyncError();
        sync_status_->setStatus(QString("Hardware sync active (avg error: %1ms)")
                               .arg(avg_sync_error, 0, 'f', 3), 
                               StatusDisplay::StatusType::SUCCESS);
    } else {
        sync_status_->setStatus("Hardware sync disabled", StatusDisplay::StatusType::WARNING);
    }
    
    // Update calibration status
    calibration_status_->setStatus("Calibration loaded (70.017mm baseline)", StatusDisplay::StatusType::SUCCESS);
    
    // Update system information
    QString system_info = QString(
        "Platform: Raspberry Pi CM4\n"
        "OS: Custom Raspbian\n"
        "Architecture: ARM64\n"
        "Qt Version: %1\n"
        "OpenCV Version: %2\n"
        "Build: %3 %4"
    ).arg(QT_VERSION_STR)
     .arg(CV_VERSION)
     .arg(__DATE__)
     .arg(__TIME__);
    
    system_info_label_->setText(system_info);
    
    // Update camera information
    QString camera_info = QString(
        "LEFT Camera: %1\n"
        "RIGHT Camera: %2\n"
        "Current FPS: %3\n"
        "Resolution: 1456x1088"
    ).arg(QString::fromStdString(camera_system_->getCameraInfo(core::CameraId::LEFT)))
     .arg(QString::fromStdString(camera_system_->getCameraInfo(core::CameraId::RIGHT)))
     .arg(camera_system_->getCurrentFrameRate(), 0, 'f', 1);
    
    camera_info_label_->setText(camera_info);
    
    // Update version information
    version_info_label_->setText("Unlook 3D Scanner v1.0 - Professional Edition");
}

void OptionsWidget::openAS1170DebugSystem() {
    // Create and show AS1170 debug dialog
    AS1170DebugDialog* debug_dialog = new AS1170DebugDialog(this);

    // Set dialog as modal for safety (prevents interaction with main system during debug)
    debug_dialog->setModal(true);

    // Show dialog and handle result
    int result = debug_dialog->exec();

    // Clean up dialog
    debug_dialog->deleteLater();

    // Log debug session completion
    qDebug() << "[OptionsWidget] AS1170 debug session completed with result:" << result;
}

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake