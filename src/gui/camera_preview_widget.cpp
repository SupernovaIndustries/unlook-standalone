#include "unlook/gui/camera_preview_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/styles/display_metrics.hpp"
#include <QTimer>
#include <QPixmap>
#include <QDateTime>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

CameraPreviewWidget::CameraPreviewWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , camera_system_(camera_system)
    , cameras_swapped_(false)
    , capture_active_(false)
    , widget_visible_(false)
    , frame_count_(0)
    , current_fps_(0.0)
{
    initializeUI();
    
    // Setup FPS timer
    fps_timer_ = new QTimer(this);
    connect(fps_timer_, &QTimer::timeout, this, &CameraPreviewWidget::updateFPSDisplay);
    fps_timer_->start(1000); // Update every second
}

CameraPreviewWidget::~CameraPreviewWidget() {
    if (capture_active_) {
        stopCapture();
    }
}

void CameraPreviewWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    widget_visible_ = true;
    
    // Start capture automatically when widget is shown
    if (camera_system_ && camera_system_->isReady()) {
        startCapture();
    }
}

void CameraPreviewWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    widget_visible_ = false;
    
    // Stop capture when widget is hidden
    stopCapture();
}

void CameraPreviewWidget::onStereoFrameReceived(const core::StereoFramePair& frame_pair) {
    if (!widget_visible_) return;
    
    frame_count_++;
    
    // Update preview images
    updatePreviewImages(frame_pair.left_frame, frame_pair.right_frame);
    
    // Update sync status
    if (frame_pair.synchronized) {
        sync_status_->setStatus(QString("Sync OK (%1ms error)")
                               .arg(frame_pair.sync_error_ms, 0, 'f', 3),
                               StatusDisplay::StatusType::SUCCESS);
    } else {
        sync_status_->setStatus("Synchronization error", StatusDisplay::StatusType::ERROR);
    }
}

void CameraPreviewWidget::updateFPSDisplay() {
    static int last_frame_count = 0;
    current_fps_ = frame_count_ - last_frame_count;
    last_frame_count = frame_count_;
    
    fps_status_->setStatus(QString("%1 FPS").arg(current_fps_), StatusDisplay::StatusType::INFO);
}

void CameraPreviewWidget::onLeftExposureChanged(double value) {
    if (camera_system_) {
        camera_system_->setExposureTime(core::CameraId::LEFT, value);
    }
}

void CameraPreviewWidget::onRightExposureChanged(double value) {
    if (camera_system_) {
        camera_system_->setExposureTime(core::CameraId::RIGHT, value);
    }
}

void CameraPreviewWidget::onLeftGainChanged(double value) {
    if (camera_system_) {
        camera_system_->setGain(core::CameraId::LEFT, value);
    }
}

void CameraPreviewWidget::onRightGainChanged(double value) {
    if (camera_system_) {
        camera_system_->setGain(core::CameraId::RIGHT, value);
    }
}

void CameraPreviewWidget::onFPSChanged(double value) {
    if (camera_system_) {
        camera_system_->setFrameRate(value);
    }
}

void CameraPreviewWidget::toggleLeftAutoExposure() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::LEFT);
    bool new_auto = !config.auto_exposure;
    
    camera_system_->setAutoExposure(core::CameraId::LEFT, new_auto);
    
    left_auto_exposure_button_->setText(new_auto ? "Auto ON" : "Auto OFF");
    left_auto_exposure_button_->setButtonType(new_auto ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    left_exposure_slider_->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleRightAutoExposure() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::RIGHT);
    bool new_auto = !config.auto_exposure;
    
    camera_system_->setAutoExposure(core::CameraId::RIGHT, new_auto);
    
    right_auto_exposure_button_->setText(new_auto ? "Auto ON" : "Auto OFF");
    right_auto_exposure_button_->setButtonType(new_auto ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    right_exposure_slider_->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleLeftAutoGain() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::LEFT);
    bool new_auto = !config.auto_gain;
    
    camera_system_->setAutoGain(core::CameraId::LEFT, new_auto);
    
    left_auto_gain_button_->setText(new_auto ? "Auto ON" : "Auto OFF");
    left_auto_gain_button_->setButtonType(new_auto ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    left_gain_slider_->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleRightAutoGain() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::RIGHT);
    bool new_auto = !config.auto_gain;
    
    camera_system_->setAutoGain(core::CameraId::RIGHT, new_auto);
    
    right_auto_gain_button_->setText(new_auto ? "Auto ON" : "Auto OFF");
    right_auto_gain_button_->setButtonType(new_auto ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    right_gain_slider_->setEnabled(!new_auto);
}

void CameraPreviewWidget::swapCameraDisplays() {
    cameras_swapped_ = !cameras_swapped_;
    
    if (cameras_swapped_) {
        left_title_label_->setText("RIGHT Camera");
        right_title_label_->setText("LEFT Camera");
        swap_cameras_button_->setText("SWAP: R↔L");
    } else {
        left_title_label_->setText("LEFT Camera");
        right_title_label_->setText("RIGHT Camera");
        swap_cameras_button_->setText("SWAP: L↔R");
    }
}

void CameraPreviewWidget::startCapture() {
    if (!camera_system_ || capture_active_) return;
    
    // Set frame callback
    auto frame_callback = [this](const core::StereoFramePair& frame_pair) {
        // Thread-safe callback using Qt's signal-slot mechanism
        QTimer::singleShot(0, this, [this, frame_pair]() {
            onStereoFrameReceived(frame_pair);
        });
    };
    
    if (camera_system_->startCapture(frame_callback)) {
        capture_active_ = true;
        start_capture_button_->setEnabled(false);
        stop_capture_button_->setEnabled(true);
        
        left_camera_status_->setStatus("Capturing", StatusDisplay::StatusType::PROCESSING);
        right_camera_status_->setStatus("Capturing", StatusDisplay::StatusType::PROCESSING);
    } else {
        left_camera_status_->setStatus("Failed to start capture", StatusDisplay::StatusType::ERROR);
        right_camera_status_->setStatus("Failed to start capture", StatusDisplay::StatusType::ERROR);
    }
}

void CameraPreviewWidget::stopCapture() {
    if (!capture_active_) return;
    
    camera_system_->stopCapture();
    capture_active_ = false;
    
    start_capture_button_->setEnabled(true);
    stop_capture_button_->setEnabled(false);
    
    left_camera_status_->setStatus("Stopped", StatusDisplay::StatusType::INFO);
    right_camera_status_->setStatus("Stopped", StatusDisplay::StatusType::INFO);
}

void CameraPreviewWidget::initializeUI() {
    const auto& metrics = styles::DisplayMetrics::instance();
    bool compact_mode = metrics.shouldUseCompactMode();
    
    qDebug() << "CameraPreviewWidget: Compact mode:" << compact_mode << "Screen:" << metrics.screenSize();
    
    if (compact_mode) {
        // Vertical layout for small screens (840x480)
        main_layout_ = new QHBoxLayout(this);
        main_layout_->setSpacing(metrics.getSpacing(SupernovaStyle::Spacing::MARGIN_SMALL));
        
        // Create preview area (takes most space)
        preview_area_ = createPreviewArea();
        main_layout_->addWidget(preview_area_, 2); // More stretch for previews
        
        // Create combined controls+status panel (compact)
        QWidget* side_panel = createCompactSidePanel();
        side_panel->setFixedWidth(metrics.screenSize().width() * 0.25); // 25% of screen width
        main_layout_->addWidget(side_panel);
        
    } else {
        // Horizontal layout for larger screens
        main_layout_ = new QHBoxLayout(this);
        main_layout_->setSpacing(SupernovaStyle::Spacing::MARGIN_LARGE);
        
        // Create preview area
        preview_area_ = createPreviewArea();
        main_layout_->addWidget(preview_area_, 1);
        
        // Create controls panel
        controls_panel_ = createControlsPanel();
        controls_panel_->setFixedWidth(300);  // Standard width for larger screens
        main_layout_->addWidget(controls_panel_);
        
        // Create status panel  
        status_panel_ = createStatusPanel();
        status_panel_->setFixedWidth(250);
        main_layout_->addWidget(status_panel_);
    }
}

QWidget* CameraPreviewWidget::createPreviewArea() {
    const auto& metrics = styles::DisplayMetrics::instance();
    bool compact_mode = metrics.shouldUseCompactMode();
    
    QWidget* area = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(area);
    layout->setSpacing(compact_mode ? 2 : metrics.getSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM));
    
    // Title with responsive font - HIDDEN on SMALL screens to save space
    if (!compact_mode) {
        QLabel* title = new QLabel("Camera Preview");
        title->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::HEADING), QFont::Bold));
        title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
        title->setAlignment(Qt::AlignCenter);
        layout->addWidget(title);
    }
    
    // Calculate responsive preview dimensions
    QSize preview_size = getResponsivePreviewSize();
    int spacing = metrics.getSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    
    // Camera displays layout - vertical stacking for compact mode
    QBoxLayout* cameras_layout;
    if (compact_mode) {
        cameras_layout = new QVBoxLayout();
    } else {
        cameras_layout = new QHBoxLayout();
    }
    cameras_layout->setSpacing(spacing);
    
    // Left camera
    QVBoxLayout* left_layout = new QVBoxLayout();
    left_title_label_ = new QLabel(compact_mode ? "L" : "LEFT Camera");
    left_title_label_->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::SUBTITLE), QFont::Bold));
    left_title_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    left_title_label_->setAlignment(Qt::AlignCenter);
    left_layout->addWidget(left_title_label_);
    
    left_camera_label_ = new QLabel();
    left_camera_label_->setFixedSize(preview_size);
    left_camera_label_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
    left_camera_label_->setAlignment(Qt::AlignCenter);
    left_camera_label_->setText("No Video");
    left_layout->addWidget(left_camera_label_);
    
    cameras_layout->addLayout(left_layout);
    
    // Right camera
    QVBoxLayout* right_layout = new QVBoxLayout();
    right_title_label_ = new QLabel(compact_mode ? "R" : "RIGHT Camera");
    right_title_label_->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::SUBTITLE), QFont::Bold));
    right_title_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    right_title_label_->setAlignment(Qt::AlignCenter);
    right_layout->addWidget(right_title_label_);
    
    right_camera_label_ = new QLabel();
    right_camera_label_->setFixedSize(preview_size);
    right_camera_label_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
    right_camera_label_->setAlignment(Qt::AlignCenter);
    right_camera_label_->setText("No Video");
    right_layout->addWidget(right_camera_label_);
    
    cameras_layout->addLayout(right_layout);
    
    layout->addLayout(cameras_layout);
    
    // Swap button - OPTIMIZED: Compact size for better UI density
    swap_cameras_button_ = new TouchButton("SWAP: L↔R", TouchButton::ButtonType::SECONDARY);
    swap_cameras_button_->setCompactSize();
    connect(swap_cameras_button_, &TouchButton::clicked, this, &CameraPreviewWidget::swapCameraDisplays);
    layout->addWidget(swap_cameras_button_, 0, Qt::AlignCenter);
    
    return area;
}

QWidget* CameraPreviewWidget::createControlsPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    // Title
    QLabel* title = new QLabel("Camera Controls");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Capture controls - HORIZONTAL LAYOUT: Start and Stop buttons side by side
    QHBoxLayout* capture_buttons_layout = new QHBoxLayout();
    capture_buttons_layout->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM); // 16px spacing between buttons
    
    start_capture_button_ = new TouchButton("START Preview", TouchButton::ButtonType::SUCCESS);
    start_capture_button_->setCompactSize();
    connect(start_capture_button_, &TouchButton::clicked, this, &CameraPreviewWidget::startCapture);
    capture_buttons_layout->addWidget(start_capture_button_);
    
    stop_capture_button_ = new TouchButton("STOP Preview", TouchButton::ButtonType::ERROR);
    stop_capture_button_->setCompactSize();
    stop_capture_button_->setEnabled(false);
    connect(stop_capture_button_, &TouchButton::clicked, this, &CameraPreviewWidget::stopCapture);
    capture_buttons_layout->addWidget(stop_capture_button_);
    
    layout->addLayout(capture_buttons_layout);
    
    // LEFT camera controls
    QLabel* left_section = new QLabel("LEFT Camera");
    left_section->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY, SupernovaStyle::FontWeight::BOLD));
    left_section->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(left_section);
    
    left_exposure_slider_ = new ParameterSlider("Exposure (μs)", MIN_EXPOSURE_US, MAX_EXPOSURE_US, 10000);
    connect(left_exposure_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onLeftExposureChanged);
    layout->addWidget(left_exposure_slider_);
    
    left_auto_exposure_button_ = new TouchButton("Auto OFF", TouchButton::ButtonType::SECONDARY);
    left_auto_exposure_button_->setCompactSize();
    connect(left_auto_exposure_button_, &TouchButton::clicked, this, &CameraPreviewWidget::toggleLeftAutoExposure);
    layout->addWidget(left_auto_exposure_button_);
    
    left_gain_slider_ = new ParameterSlider("Gain", MIN_GAIN, MAX_GAIN, 1.0, ParameterSlider::ValueType::FLOATING_POINT);
    connect(left_gain_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onLeftGainChanged);
    layout->addWidget(left_gain_slider_);
    
    left_auto_gain_button_ = new TouchButton("Auto OFF", TouchButton::ButtonType::SECONDARY);
    left_auto_gain_button_->setCompactSize();
    connect(left_auto_gain_button_, &TouchButton::clicked, this, &CameraPreviewWidget::toggleLeftAutoGain);
    layout->addWidget(left_auto_gain_button_);
    
    // RIGHT camera controls
    QLabel* right_section = new QLabel("RIGHT Camera");
    right_section->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY, SupernovaStyle::FontWeight::BOLD));
    right_section->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(right_section);
    
    right_exposure_slider_ = new ParameterSlider("Exposure (μs)", MIN_EXPOSURE_US, MAX_EXPOSURE_US, 10000);
    connect(right_exposure_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onRightExposureChanged);
    layout->addWidget(right_exposure_slider_);
    
    right_auto_exposure_button_ = new TouchButton("Auto OFF", TouchButton::ButtonType::SECONDARY);
    right_auto_exposure_button_->setCompactSize();
    connect(right_auto_exposure_button_, &TouchButton::clicked, this, &CameraPreviewWidget::toggleRightAutoExposure);
    layout->addWidget(right_auto_exposure_button_);
    
    right_gain_slider_ = new ParameterSlider("Gain", MIN_GAIN, MAX_GAIN, 1.0, ParameterSlider::ValueType::FLOATING_POINT);
    connect(right_gain_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onRightGainChanged);
    layout->addWidget(right_gain_slider_);
    
    right_auto_gain_button_ = new TouchButton("Auto OFF", TouchButton::ButtonType::SECONDARY);
    right_auto_gain_button_->setCompactSize();
    connect(right_auto_gain_button_, &TouchButton::clicked, this, &CameraPreviewWidget::toggleRightAutoGain);
    layout->addWidget(right_auto_gain_button_);
    
    // Global controls
    QLabel* global_section = new QLabel("Global Settings");
    global_section->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY, SupernovaStyle::FontWeight::BOLD));
    global_section->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(global_section);
    
    fps_slider_ = new ParameterSlider("Target FPS", MIN_FPS, MAX_FPS, 30.0, ParameterSlider::ValueType::FLOATING_POINT);
    connect(fps_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onFPSChanged);
    layout->addWidget(fps_slider_);
    
    layout->addStretch();
    
    return panel;
}

QWidget* CameraPreviewWidget::createStatusPanel() {
    QWidget* panel = new QWidget();
    panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    layout->setContentsMargins(SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE,
                              SupernovaStyle::Spacing::PADDING_LARGE);
    
    // Title
    QLabel* title = new QLabel("Status");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Status displays
    left_camera_status_ = new StatusDisplay("LEFT Camera");
    left_camera_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(left_camera_status_);
    
    right_camera_status_ = new StatusDisplay("RIGHT Camera");  
    right_camera_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(right_camera_status_);
    
    fps_status_ = new StatusDisplay("Frame Rate");
    fps_status_->setStatus("0 FPS", StatusDisplay::StatusType::INFO);
    layout->addWidget(fps_status_);
    
    sync_status_ = new StatusDisplay("Synchronization");
    sync_status_->setStatus("Not capturing", StatusDisplay::StatusType::INFO);
    layout->addWidget(sync_status_);
    
    layout->addStretch();
    
    return panel;
}

void CameraPreviewWidget::updateCameraControls() {
    // Update UI controls based on current camera configuration
    if (!camera_system_) return;
    
    core::CameraConfig left_config = camera_system_->getCameraConfig(core::CameraId::LEFT);
    core::CameraConfig right_config = camera_system_->getCameraConfig(core::CameraId::RIGHT);
    
    // Update sliders
    left_exposure_slider_->setValue(left_config.exposure_time_us);
    right_exposure_slider_->setValue(right_config.exposure_time_us);
    left_gain_slider_->setValue(left_config.gain);
    right_gain_slider_->setValue(right_config.gain);
    fps_slider_->setValue(left_config.fps);
    
    // Update auto buttons
    left_auto_exposure_button_->setText(left_config.auto_exposure ? "Auto ON" : "Auto OFF");
    left_auto_exposure_button_->setButtonType(left_config.auto_exposure ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    
    right_auto_exposure_button_->setText(right_config.auto_exposure ? "Auto ON" : "Auto OFF");
    right_auto_exposure_button_->setButtonType(right_config.auto_exposure ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    
    left_auto_gain_button_->setText(left_config.auto_gain ? "Auto ON" : "Auto OFF");
    left_auto_gain_button_->setButtonType(left_config.auto_gain ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    
    right_auto_gain_button_->setText(right_config.auto_gain ? "Auto ON" : "Auto OFF");
    right_auto_gain_button_->setButtonType(right_config.auto_gain ? TouchButton::ButtonType::SUCCESS : TouchButton::ButtonType::SECONDARY);
    
    // Enable/disable sliders based on auto settings
    left_exposure_slider_->setEnabled(!left_config.auto_exposure);
    right_exposure_slider_->setEnabled(!right_config.auto_exposure);
    left_gain_slider_->setEnabled(!left_config.auto_gain);
    right_gain_slider_->setEnabled(!right_config.auto_gain);
}

QPixmap CameraPreviewWidget::matToQPixmap(const cv::Mat& mat) {
    if (mat.empty()) return QPixmap();
    
    QImage qimg;
    if (mat.type() == CV_8UC1) {
        // Grayscale image
        qimg = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
    } else if (mat.type() == CV_8UC3) {
        // Color image (BGR)
        qimg = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_BGR888);
        qimg = qimg.rgbSwapped();
    } else {
        return QPixmap();
    }
    
    return QPixmap::fromImage(qimg);
}

QPixmap CameraPreviewWidget::scalePixmapToFit(const QPixmap& pixmap, const QSize& size) {
    if (pixmap.isNull()) return QPixmap();
    
    return pixmap.scaled(size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

void CameraPreviewWidget::updatePreviewImages(const core::CameraFrame& left_frame, 
                                             const core::CameraFrame& right_frame) {
    // Convert frames to pixmaps
    QPixmap left_pixmap = matToQPixmap(left_frame.image);
    QPixmap right_pixmap = matToQPixmap(right_frame.image);
    
    // Scale to fit responsive displays
    QSize preview_size = getResponsivePreviewSize();
    left_pixmap = scalePixmapToFit(left_pixmap, preview_size);
    right_pixmap = scalePixmapToFit(right_pixmap, preview_size);
    
    // Apply to labels (considering swap state)
    if (cameras_swapped_) {
        left_camera_label_->setPixmap(right_pixmap);
        right_camera_label_->setPixmap(left_pixmap);
    } else {
        left_camera_label_->setPixmap(left_pixmap);
        right_camera_label_->setPixmap(right_pixmap);
    }
}

QSize CameraPreviewWidget::getResponsivePreviewSize() {
    const auto& metrics = styles::DisplayMetrics::instance();
    QSize screen_size = metrics.screenSize();
    bool compact_mode = metrics.shouldUseCompactMode();
    
    if (compact_mode) {
        // For 800x480, use DRASTICALLY smaller previews - MAX 200x150px as requested
        // Must fit both vertically stacked previews + controls in 420px available space
        int preview_width = 200;  // Max width as requested
        int preview_height = 150; // Max height as requested (200 * 0.75 = 150)
        
        // Ensure we don't exceed limits even on very small screens
        preview_width = std::min(preview_width, static_cast<int>(screen_size.width() * 0.4));
        preview_height = std::min(preview_height, static_cast<int>(screen_size.height() * 0.25));
        
        return QSize(preview_width, preview_height);
    } else {
        // Standard size for larger screens
        return QSize(400, 300);
    }
}

QWidget* CameraPreviewWidget::createCompactSidePanel() {
    const auto& metrics = styles::DisplayMetrics::instance();
    
    QWidget* side_panel = new QWidget();
    side_panel->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    
    QVBoxLayout* layout = new QVBoxLayout(side_panel);
    layout->setSpacing(2); // Ultra-compact spacing for SMALL screens
    
    int padding = 2; // Minimal padding for SMALL screens
    layout->setContentsMargins(padding, padding, padding, padding);
    
    // Title - shortened for SMALL screens
    QLabel* title = new QLabel("Controls");
    title->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::BODY), QFont::Bold)); // Smaller font
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Compact capture controls - vertical stack
    start_capture_button_ = new TouchButton("START", TouchButton::ButtonType::SUCCESS);
    start_capture_button_->setCompactSize();
    connect(start_capture_button_, &TouchButton::clicked, this, &CameraPreviewWidget::startCapture);
    layout->addWidget(start_capture_button_);
    
    stop_capture_button_ = new TouchButton("STOP", TouchButton::ButtonType::ERROR);
    stop_capture_button_->setCompactSize();
    stop_capture_button_->setEnabled(false);
    connect(stop_capture_button_, &TouchButton::clicked, this, &CameraPreviewWidget::stopCapture);
    layout->addWidget(stop_capture_button_);
    
    // Swap cameras button
    swap_cameras_button_ = new TouchButton("SWAP L↔R", TouchButton::ButtonType::SECONDARY);
    swap_cameras_button_->setCompactSize();
    connect(swap_cameras_button_, &TouchButton::clicked, this, &CameraPreviewWidget::swapCameraDisplays);
    layout->addWidget(swap_cameras_button_);
    
    // Essential controls section - REMOVED to save space on SMALL screens
    
    // Only essential sliders for compact mode
    fps_slider_ = new ParameterSlider("FPS", MIN_FPS, MAX_FPS, 30.0, ParameterSlider::ValueType::FLOATING_POINT);
    // fps_slider_->setCompactMode(true);  // TODO: Implement setCompactMode in ParameterSlider
    connect(fps_slider_, &ParameterSlider::valueChanged, this, &CameraPreviewWidget::onFPSChanged);
    layout->addWidget(fps_slider_);
    
    // Status section - REMOVED to save space on SMALL screens
    
    left_camera_status_ = new StatusDisplay("LEFT");
    // left_camera_status_->setCompactMode(true);  // TODO: Implement setCompactMode in StatusDisplay
    left_camera_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(left_camera_status_);
    
    right_camera_status_ = new StatusDisplay("RIGHT");
    // right_camera_status_->setCompactMode(true);  // TODO: Implement setCompactMode in StatusDisplay
    right_camera_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(right_camera_status_);
    
    fps_status_ = new StatusDisplay("FPS");
    // fps_status_->setCompactMode(true);  // TODO: Implement setCompactMode in StatusDisplay
    fps_status_->setStatus("0 FPS", StatusDisplay::StatusType::INFO);
    layout->addWidget(fps_status_);
    
    sync_status_ = new StatusDisplay("Sync");
    // sync_status_->setCompactMode(true);  // TODO: Implement setCompactMode in StatusDisplay
    sync_status_->setStatus("Not capturing", StatusDisplay::StatusType::INFO);
    layout->addWidget(sync_status_);
    
    layout->addStretch();
    
    return side_panel;
}

} // namespace gui
} // namespace unlook