#include "unlook/gui/camera_preview_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/styles/display_metrics.hpp"
#include <QTimer>
#include <QPixmap>
#include <QDateTime>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <opencv2/imgproc.hpp>

#include "ui_camera_preview_widget.h"

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

CameraPreviewWidget::CameraPreviewWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::CameraPreviewWidget)
    , camera_system_(camera_system)
    , cameras_swapped_(false)
    , capture_active_(false)
    , widget_visible_(false)
    , frame_count_(0)
    , current_fps_(0.0)
    , left_camera_label_(nullptr)
    , right_camera_label_(nullptr)
{
    // Setup UI from .ui file
    ui->setupUi(this);
    
    // Connect signals
    connectSignals();
    
    // Initialize additional components
    initializeAdditionalComponents();
    
    // Initialize UI compatibility components
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
    delete ui;
}

void CameraPreviewWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    widget_visible_ = true;
    
    // Do NOT auto-start capture in showEvent - causes immediate start/stop cycle
    // Let user manually control capture with START/STOP buttons
}

void CameraPreviewWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    widget_visible_ = false;
    
    // Do NOT auto-stop capture in hideEvent - causes immediate start/stop cycle
    // Let capture continue until user clicks STOP button or explicit navigation
}

void CameraPreviewWidget::onStereoFrameReceived(const core::StereoFramePair& frame_pair) {
    if (!widget_visible_) return;
    
    frame_count_++;
    
    // PERFORMANCE FIX: Frame skipping for GUI - process only every 3rd frame
    // This allows cameras to run at full speed while GUI runs at manageable rate
    static int frame_skip_counter = 0;
    if (++frame_skip_counter % 3 != 0) {
        // Skip this frame for GUI processing, but still update sync status
        if (frame_pair.synchronized) {
            if (sync_status_) {
// TODO_UI:                 sync_status_->setStatus(QString("Sync OK (%1ms error)")
//                                      .arg(frame_pair.sync_error_ms, 0, 'f', 3),
//                                      StatusDisplay::StatusType::SUCCESS);
            }
        } else {
            if (sync_status_) {
// TODO_UI:                 sync_status_->setStatus("Synchronization error", StatusDisplay::StatusType::ERROR);
            }
        }
        return;
    }
    
    // Update preview images (only every 3rd frame)
    updatePreviewImages(frame_pair.left_frame, frame_pair.right_frame);
    
    // Update sync status
    if (frame_pair.synchronized) {
        if (sync_status_) {
// TODO_UI:             sync_status_->setStatus(QString("Sync OK (%1ms error)")
//                                  .arg(frame_pair.sync_error_ms, 0, 'f', 3),
//                                  StatusDisplay::StatusType::SUCCESS);
        }
    } else {
        if (sync_status_) {
// TODO_UI:             sync_status_->setStatus("Synchronization error", StatusDisplay::StatusType::ERROR);
        }
    }
}

void CameraPreviewWidget::connectSignals() {
    // Connect UI signals to slots
    connect(ui->start_capture_button, &QPushButton::clicked, this, &CameraPreviewWidget::startCapture);
    connect(ui->stop_capture_button, &QPushButton::clicked, this, &CameraPreviewWidget::stopCapture);
    connect(ui->swap_cameras_button, &QPushButton::clicked, this, &CameraPreviewWidget::swapCameraDisplays);
    
    // Connect sliders
    connect(ui->left_exposure_slider, QOverload<int>::of(&QSlider::valueChanged), 
            this, [this](int value) { onLeftExposureChanged(static_cast<double>(value)); });
    connect(ui->right_exposure_slider, QOverload<int>::of(&QSlider::valueChanged), 
            this, [this](int value) { onRightExposureChanged(static_cast<double>(value)); });
    connect(ui->left_gain_slider, QOverload<int>::of(&QSlider::valueChanged), 
            this, [this](int value) { onLeftGainChanged(static_cast<double>(value) / 100.0); });
    connect(ui->right_gain_slider, QOverload<int>::of(&QSlider::valueChanged), 
            this, [this](int value) { onRightGainChanged(static_cast<double>(value) / 100.0); });
    // TODO_UI: Add fps_slider to .ui file
    // connect(ui->fps_slider, QOverload<int>::of(&QSlider::valueChanged), 
    //         this, [this](int value) { onFPSChanged(static_cast<double>(value)); });
    
    // TODO_UI: Add auto exposure/gain buttons to .ui file
    // Connect auto buttons
    // connect(ui->left_auto_exposure_button, &QPushButton::clicked, this, &CameraPreviewWidget::toggleLeftAutoExposure);
    // connect(ui->right_auto_exposure_button, &QPushButton::clicked, this, &CameraPreviewWidget::toggleRightAutoExposure);
    // connect(ui->left_auto_gain_button, &QPushButton::clicked, this, &CameraPreviewWidget::toggleLeftAutoGain);
    // connect(ui->right_auto_gain_button, &QPushButton::clicked, this, &CameraPreviewWidget::toggleRightAutoGain);
}

void CameraPreviewWidget::initializeAdditionalComponents() {
    // Initialize slider value labels with default values (unified for both cameras)
    // Exposure default: 10000µs (from .ui file)
    ui->left_exposure_value->setText("10000µs");
    
    // Gain default: 1.0x (from .ui file, slider value 100 = gain 1.0)
    ui->left_gain_value->setText("1.0x");
    
    // Hide/disable right sliders (will be removed from UI)
    if (ui->right_exposure_slider) {
        ui->right_exposure_slider->setVisible(false);
    }
    if (ui->right_gain_slider) {
        ui->right_gain_slider->setVisible(false);
    }
    if (ui->right_exposure_value) {
        ui->right_exposure_value->setVisible(false);
    }
    if (ui->right_gain_value) {
        ui->right_gain_value->setVisible(false);
    }
}

void CameraPreviewWidget::updateFPSDisplay() {
    static int last_frame_count = 0;
    current_fps_ = frame_count_ - last_frame_count;
    last_frame_count = frame_count_;
    
    ui->fps_status->setText(QString("FPS: %1").arg(current_fps_));
}

void CameraPreviewWidget::onLeftExposureChanged(double value) {
    if (camera_system_) {
        // Now controls BOTH cameras (unified control)
        camera_system_->setExposureTime(core::CameraId::LEFT, value);
    }
    // Update label to show current value (for both cameras)
    ui->left_exposure_value->setText(QString("%1µs").arg(static_cast<int>(value)));
}

void CameraPreviewWidget::onRightExposureChanged(double value) {
    // Deprecated - right slider will be removed
    onLeftExposureChanged(value);
}

void CameraPreviewWidget::onLeftGainChanged(double value) {
    if (camera_system_) {
        // Now controls BOTH cameras (unified control)
        camera_system_->setGain(core::CameraId::LEFT, value);
    }
    // Update label to show current value (for both cameras, gain is from 1.0 to 16.0)
    ui->left_gain_value->setText(QString("%1x").arg(value, 0, 'f', 1));
}

void CameraPreviewWidget::onRightGainChanged(double value) {
    // Deprecated - right slider will be removed
    onLeftGainChanged(value);
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
    
    // TODO_UI: Add auto exposure button to .ui file
    // ui->left_auto_exposure_button->setText(new_auto ? "Auto ON" : "Auto OFF");
    ui->left_exposure_slider->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleRightAutoExposure() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::RIGHT);
    bool new_auto = !config.auto_exposure;
    
    camera_system_->setAutoExposure(core::CameraId::RIGHT, new_auto);
    
    // TODO_UI: Add auto exposure button to .ui file
    // ui->right_auto_exposure_button->setText(new_auto ? "Auto ON" : "Auto OFF");
    ui->right_exposure_slider->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleLeftAutoGain() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::LEFT);
    bool new_auto = !config.auto_gain;
    
    camera_system_->setAutoGain(core::CameraId::LEFT, new_auto);
    
    // TODO_UI: Add auto gain button to .ui file
    // ui->left_auto_gain_button->setText(new_auto ? "Auto ON" : "Auto OFF");
    ui->left_gain_slider->setEnabled(!new_auto);
}

void CameraPreviewWidget::toggleRightAutoGain() {
    if (!camera_system_) return;
    
    core::CameraConfig config = camera_system_->getCameraConfig(core::CameraId::RIGHT);
    bool new_auto = !config.auto_gain;
    
    camera_system_->setAutoGain(core::CameraId::RIGHT, new_auto);
    
    // TODO_UI: Add auto gain button to .ui file
    // ui->right_auto_gain_button->setText(new_auto ? "Auto ON" : "Auto OFF");
    ui->right_gain_slider->setEnabled(!new_auto);
}

void CameraPreviewWidget::swapCameraDisplays() {
    cameras_swapped_ = !cameras_swapped_;
    
    if (cameras_swapped_) {
// TODO_UI:         left_title_label_->setText("RIGHT Camera");
// TODO_UI:         right_title_label_->setText("LEFT Camera");
// TODO_UI:         swap_cameras_button_->setText("SWAP: R↔L");
    } else {
// TODO_UI:         left_title_label_->setText("LEFT Camera");
// TODO_UI:         right_title_label_->setText("RIGHT Camera");
// TODO_UI:         swap_cameras_button_->setText("SWAP: L↔R");
    }
}

void CameraPreviewWidget::startCapture() {
    if (!camera_system_ || capture_active_) return;
    
    // UX IMPROVEMENT: If cameras are already running (from auto-start), just attach to them
    qDebug() << "[CameraPreview] Starting camera preview capture...";
    
    // Set frame callback for this widget
    auto frame_callback = [this](const core::StereoFramePair& frame_pair) {
        // PERFORMANCE FIX: Use direct Qt signal emission for real-time processing
        // QTimer::singleShot causes batching/accumulation - BAD for 30 FPS video!
        QMetaObject::invokeMethod(this, [this, frame_pair]() {
            onStereoFrameReceived(frame_pair);
        }, Qt::QueuedConnection);
    };
    
    if (camera_system_->startCapture(frame_callback)) {
        capture_active_ = true;
        ui->start_capture_button->setEnabled(false);
        ui->stop_capture_button->setEnabled(true);
        qDebug() << "[CameraPreview] Successfully attached to camera capture";
        
// TODO_UI:         left_camera_status_->setStatus("Capturing", StatusDisplay::StatusType::PROCESSING);
// TODO_UI:         right_camera_status_->setStatus("Capturing", StatusDisplay::StatusType::PROCESSING);
    } else {
        qDebug() << "[CameraPreview] Failed to start camera capture";
// TODO_UI:         left_camera_status_->setStatus("Failed to start capture", StatusDisplay::StatusType::ERROR);
// TODO_UI:         right_camera_status_->setStatus("Failed to start capture", StatusDisplay::StatusType::ERROR);
    }
}

void CameraPreviewWidget::stopCapture() {
    if (!capture_active_) return;
    
    // UX IMPROVEMENT: Don't stop cameras completely, just return to background operation
    // This keeps cameras running for other widgets and better user experience
    qDebug() << "[CameraPreview] Switching to background capture mode instead of stopping completely";
    
    // Create a minimal background callback to keep cameras running
    auto background_callback = [](const core::StereoFramePair& /*frame_pair*/) {
        // Frame received but not processed - just keeps cameras running in background
    };
    
    if (camera_system_->startCapture(background_callback)) {
        qDebug() << "[CameraPreview] Switched to background capture successfully";
    } else {
        qDebug() << "[CameraPreview] Failed to switch to background capture, stopping completely";
        camera_system_->stopCapture();
    }
    
    capture_active_ = false;
    ui->start_capture_button->setEnabled(true);
    ui->stop_capture_button->setEnabled(false);
    
// TODO_UI:     left_camera_status_->setStatus("Background Mode", StatusDisplay::StatusType::INFO);
// TODO_UI:     right_camera_status_->setStatus("Background Mode", StatusDisplay::StatusType::INFO);
}

void CameraPreviewWidget::initializeUI() {
    const auto& metrics = styles::DisplayMetrics::instance();
    bool compact_mode = metrics.shouldUseCompactMode();
    
    qDebug() << "CameraPreviewWidget: Compact mode:" << compact_mode << "Screen:" << metrics.screenSize();
    
    // DO NOT create layouts here - setupUi() already created them from .ui file
    // Instead, find and connect to existing widgets from .ui file
    
    // TODO: Find widgets by object name from .ui file and assign to pointers
    // For now, create them manually but without layout conflicts
    
    // Find camera labels from .ui file (using correct snake_case names)
    if (!left_camera_label_) {
        left_camera_label_ = findChild<QLabel*>("left_camera_label");
        if (!left_camera_label_) {
            qWarning() << "Could not find left_camera_label in .ui file";
            return; // Don't create duplicates
        }
    }
    
    if (!right_camera_label_) {
        right_camera_label_ = findChild<QLabel*>("right_camera_label");
        if (!right_camera_label_) {
            qWarning() << "Could not find right_camera_label in .ui file";
            return; // Don't create duplicates
        }
    }
    
    // Apply camera preview styling
    QSize preview_size = getResponsivePreviewSize();
    if (left_camera_label_) {
        left_camera_label_->setFixedSize(preview_size);
        left_camera_label_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
        left_camera_label_->setAlignment(Qt::AlignCenter);
    }
    
    if (right_camera_label_) {
        right_camera_label_->setFixedSize(preview_size);
        right_camera_label_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
        right_camera_label_->setAlignment(Qt::AlignCenter);
    }
}

void CameraPreviewWidget::updatePreviewImages(const core::CameraFrame& left_frame, 
                                               const core::CameraFrame& right_frame) {
    if (!left_camera_label_ || !right_camera_label_) return;
    
    // Convert frames to QPixmap
    cv::Mat left_rgb, right_rgb;
    cv::cvtColor(left_frame.image, left_rgb, cv::COLOR_BGRA2RGB);
    cv::cvtColor(right_frame.image, right_rgb, cv::COLOR_BGRA2RGB);
    
    // Create QImage from cv::Mat
    QImage left_qimg(left_rgb.data, left_rgb.cols, left_rgb.rows, left_rgb.step, QImage::Format_RGB888);
    QImage right_qimg(right_rgb.data, right_rgb.cols, right_rgb.rows, right_rgb.step, QImage::Format_RGB888);
    
    // Scale to preview size
    QSize preview_size = getResponsivePreviewSize();
    QPixmap left_pixmap = QPixmap::fromImage(left_qimg.scaled(preview_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    QPixmap right_pixmap = QPixmap::fromImage(right_qimg.scaled(preview_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    
    // Handle camera swapping
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
    
    if (metrics.shouldUseCompactMode()) {
        // Compact mode: smaller previews
        return QSize(320, 240);
    } else {
        // Desktop mode: larger previews
        return QSize(480, 360);
    }
}

} // namespace gui
} // namespace unlook
