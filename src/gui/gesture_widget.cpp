#include "unlook/gui/gesture_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/styles/display_metrics.hpp"
#include <QTimer>
#include <QPixmap>
#include <QDateTime>
#include <QDebug>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFile>
#include <QCoreApplication>
#include <opencv2/imgproc.hpp>

#include "ui_gesture_widget.h"

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

GestureWidget::GestureWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::GestureWidget)
    , gesture_system_(nullptr)
    , camera_system_(camera_system)
    , is_running_(false)
    , widget_visible_(false)
    , debug_visualization_enabled_(false)
    , fps_timer_(nullptr)
    , camera_feed_label_(nullptr)
    , frame_count_(0)
    , gesture_count_(0)
    , current_fps_(0.0)
    , avg_processing_time_(0.0)
{
    // Setup UI from .ui file
    ui->setupUi(this);

    // Connect signals
    connectSignals();

    // Initialize additional components
    initializeAdditionalComponents();

    // Initialize gesture system
    initializeGestureSystem();

    // Setup FPS timer (only for statistics display)
    fps_timer_ = new QTimer(this);
    connect(fps_timer_, &QTimer::timeout, this, &GestureWidget::updateFPSDisplay);
    fps_timer_->start(1000); // Update every second

    qDebug() << "[GestureWidget] Initialized";
}

GestureWidget::~GestureWidget() {
    if (is_running_) {
        stopGestureRecognition();
    }
    delete ui;
    qDebug() << "[GestureWidget] Destroyed";
}

void GestureWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    widget_visible_ = true;
    qDebug() << "[GestureWidget] Widget shown";

    // Do NOT auto-start - let user control with START button
}

void GestureWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    widget_visible_ = false;
    qDebug() << "[GestureWidget] Widget hidden";

    // Do NOT auto-stop - let user control with STOP button
}

void GestureWidget::connectSignals() {
    // Connect UI signals to slots
    connect(ui->start_button, &QPushButton::clicked, this, &GestureWidget::startGestureRecognition);
    connect(ui->stop_button, &QPushButton::clicked, this, &GestureWidget::stopGestureRecognition);
    connect(ui->clear_history_button, &QPushButton::clicked, this, &GestureWidget::onClearHistoryClicked);

    // Connect confidence slider
    connect(ui->confidence_spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &GestureWidget::onConfidenceChanged);

    // Connect debug visualization checkbox
    connect(ui->debug_checkbox, &QCheckBox::toggled, this, &GestureWidget::onDebugVisualizationToggled);
}

void GestureWidget::initializeAdditionalComponents() {
    // Find camera feed label from .ui file
    camera_feed_label_ = findChild<QLabel*>("camera_feed_label");
    if (!camera_feed_label_) {
        qWarning() << "[GestureWidget] Could not find camera_feed_label in .ui file";
        return;
    }

    // Apply styling
    QSize preview_size = getResponsivePreviewSize();
    camera_feed_label_->setFixedSize(preview_size);
    camera_feed_label_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
    camera_feed_label_->setAlignment(Qt::AlignCenter);
    camera_feed_label_->setText("Camera Preview\n(Press START)");

    // Set initial values
    ui->confidence_spinbox->setValue(DEFAULT_CONFIDENCE_THRESHOLD);
    ui->debug_checkbox->setChecked(false);

    // Initialize status displays (if needed)
    // TODO: Create status display widgets if not in .ui file
}

void GestureWidget::initializeGestureSystem() {
    if (!camera_system_) {
        qWarning() << "[GestureWidget] No camera system available";
        return;
    }

    // Create gesture recognition system
    gesture_system_ = std::make_unique<gesture::GestureRecognitionSystem>();

    // Configure gesture system
    gesture::GestureConfig config;
    config.min_gesture_confidence = DEFAULT_CONFIDENCE_THRESHOLD;
    config.enable_debug_viz = debug_visualization_enabled_;
    config.max_num_hands = 1;
    config.enable_temporal_smoothing = true;

    // Note: Gesture system will be initialized when START is clicked
    qDebug() << "[GestureWidget] Gesture system created (not yet initialized)";
}

void GestureWidget::startGestureRecognition() {
    if (is_running_ || !gesture_system_ || !camera_system_) {
        qWarning() << "[GestureWidget] Cannot start: already running or system not available";
        return;
    }

    qDebug() << "[GestureWidget] Starting gesture recognition...";

    // Initialize gesture system if needed
    if (!gesture_system_->is_initialized()) {
        gesture::GestureConfig config;
        config.min_gesture_confidence = ui->confidence_spinbox->value();
        config.enable_debug_viz = debug_visualization_enabled_;
        config.max_num_hands = 1;
        config.enable_temporal_smoothing = true;

        // Set ABSOLUTE paths to ONNX models
        // Calculate base path relative to executable location
        // Executable is at: build/src/gui/unlook_scanner
        // Models are at: third-party/hand-gesture-recognition-using-onnx/model/...
        // Need to go up 3 levels: gui -> src -> build -> project_root
        QString exe_dir = QCoreApplication::applicationDirPath();
        QString base_path = exe_dir + "/../../../third-party/hand-gesture-recognition-using-onnx/model";

        // Set model paths in config
        config.palm_detection_model_path = base_path.toStdString() + "/palm_detection/palm_detection_full_inf_post_192x192.onnx";
        config.hand_landmark_model_path = base_path.toStdString() + "/hand_landmark/hand_landmark_sparse_Nx3x224x224.onnx";

        qDebug() << "[GestureWidget] Model paths:";
        qDebug() << "  Palm detection:" << QString::fromStdString(config.palm_detection_model_path);
        qDebug() << "  Hand landmark:" << QString::fromStdString(config.hand_landmark_model_path);

        // Verify models exist BEFORE initialization
        if (!QFile::exists(QString::fromStdString(config.palm_detection_model_path))) {
            qCritical() << "[GestureWidget] Palm detection model NOT FOUND:"
                        << QString::fromStdString(config.palm_detection_model_path);
            emit errorOccurred("Palm detection model not found. Please check installation.");
            return;
        }

        if (!QFile::exists(QString::fromStdString(config.hand_landmark_model_path))) {
            qCritical() << "[GestureWidget] Hand landmark model NOT FOUND:"
                        << QString::fromStdString(config.hand_landmark_model_path);
            emit errorOccurred("Hand landmark model not found. Please check installation.");
            return;
        }

        qDebug() << "[GestureWidget] Model files verified, initializing gesture system...";

        // Initialize with nullptr camera (gesture system doesn't use it internally)
        if (!gesture_system_->initialize(nullptr, config)) {
            QString error = QString::fromStdString(gesture_system_->get_last_error());
            qCritical() << "[GestureWidget] Failed to initialize gesture system:" << error;
            emit errorOccurred("Gesture init failed: " + error);
            return;
        }
        qDebug() << "[GestureWidget] Gesture system initialized successfully";
    }

    // Start camera capture with frame callback (REAL-TIME)
    auto frame_callback = [this](const core::StereoFramePair& frame_pair) {
        static int callback_count = 0;
        callback_count++;
        if (callback_count % 30 == 0) {  // Log every 30 frames
            qDebug() << "[GestureWidget] Camera callback received, frame count:" << callback_count;
        }

        // Invoke in UI thread for thread safety
        QMetaObject::invokeMethod(this, [this, frame_pair]() {
            processFrame(frame_pair);  // Process EVERY frame in real-time
        }, Qt::QueuedConnection);
    };

    if (!camera_system_->startCapture(frame_callback)) {
        qWarning() << "[GestureWidget] Failed to start camera capture";
        emit errorOccurred("Failed to start camera");
        return;
    }

    // Update UI state
    is_running_ = true;
    ui->start_button->setEnabled(false);
    ui->stop_button->setEnabled(true);

    // Reset statistics
    frame_count_ = 0;
    gesture_count_ = 0;
    gesture_histogram_.clear();

    qDebug() << "[GestureWidget] Gesture recognition started (real-time mode)";
}

void GestureWidget::stopGestureRecognition() {
    if (!is_running_) {
        return;
    }

    qDebug() << "[GestureWidget] Stopping gesture recognition...";

    // Switch camera to background mode (don't stop completely - other widgets may be using it)
    auto background_callback = [](const core::StereoFramePair& /*frame_pair*/) {
        // Keep cameras running in background for other widgets
    };

    if (camera_system_) {
        camera_system_->startCapture(background_callback);
    }

    // Update UI state
    is_running_ = false;
    ui->start_button->setEnabled(true);
    ui->stop_button->setEnabled(false);

    // Clear camera preview
    if (camera_feed_label_) {
        camera_feed_label_->setText("Camera Preview\n(Press START)");
    }

    qDebug() << "[GestureWidget] Gesture recognition stopped";
}

void GestureWidget::processFrame(const core::StereoFramePair& frame_pair) {
    if (!is_running_ || !widget_visible_ || !gesture_system_) {
        return;
    }

    frame_count_++;

    // PERFORMANCE OPTIMIZATION: Process only every 2nd frame (15 FPS instead of 30 FPS)
    // Gesture recognition takes ~55ms, camera delivers frames every 33ms
    // Skip frames to prevent backlog and UI freeze
    if (frame_count_ % 2 != 0) {
        return;  // Skip odd frames, process only even frames
    }

    // Log first few frames and periodically
    if (frame_count_ <= 6 || frame_count_ % 30 == 0) {
        qDebug() << "[GestureWidget] processFrame called, frame_count:" << frame_count_
                 << "is_running:" << is_running_ << "widget_visible:" << widget_visible_;
    }

    // Use LEFT camera frame (MASTER camera)
    cv::Mat frame = frame_pair.left_frame.image.clone();

    // Skip if no frame available
    if (frame.empty()) {
        qWarning() << "[GestureWidget] Received empty frame from camera";
        return;
    }

    if (frame_count_ <= 3) {
        qDebug() << "[GestureWidget] Frame received: size=" << frame.cols << "x" << frame.rows
                 << "channels=" << frame.channels() << "type=" << frame.type();
    }

    // Convert from YUV420/Bayer/BGRA to BGR if needed
    cv::Mat bgr_frame;
    if (frame.channels() == 1) {
        // Bayer to BGR conversion (assume BGGR pattern for IMX296)
        cv::cvtColor(frame, bgr_frame, cv::COLOR_BayerBG2BGR);
    } else if (frame.channels() == 3) {
        // Already BGR
        bgr_frame = frame;
    } else if (frame.channels() == 4) {
        // BGRA to BGR
        cv::cvtColor(frame, bgr_frame, cv::COLOR_BGRA2BGR);
    } else {
        qWarning() << "[GestureWidget] Unexpected frame format: " << frame.channels() << " channels";
        return;
    }

    // Process frame for gesture recognition
    gesture::GestureResult result;
    auto start_time = std::chrono::steady_clock::now();

    if (gesture_system_->process_frame(bgr_frame, result)) {
        auto end_time = std::chrono::steady_clock::now();
        double processing_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        // Update processing time stats
        avg_processing_time_ = (avg_processing_time_ * (frame_count_ - 1) + processing_ms) / frame_count_;

        // Log processing success periodically
        if (frame_count_ % 30 == 0) {
            qDebug() << "[GestureWidget] Gesture processing successful, avg time:" << avg_processing_time_ << "ms";
        }

        // Check if gesture was detected
        if (result.type != gesture::GestureType::UNKNOWN) {
            gesture_count_++;

            QString gesture_name = QString::fromStdString(gesture::gesture_type_to_string(result.type));

            // Update gesture history
            updateGestureHistory(gesture_name, result.confidence);

            // Emit signal
            emit gestureDetected(gesture_name, result.confidence);

            qDebug() << "[GestureWidget] Gesture detected:" << gesture_name
                     << "confidence:" << result.confidence;
        }
        // Display camera frame with overlays
        if (debug_visualization_enabled_) {
            cv::Mat debug_frame = gesture_system_->get_debug_frame();
            if (!debug_frame.empty()) {
                displayCameraFrame(debug_frame);
            } else {
                displayCameraFrame(bgr_frame);
            }
        } else {
            displayCameraFrame(bgr_frame);
        }

        // Update performance stats
        updatePerformanceStats(current_fps_, avg_processing_time_);

        // Update gesture statistics
        updateGestureStatistics();
    } else {
        // Log processing errors
        QString error = QString::fromStdString(gesture_system_->get_last_error());
        if (frame_count_ % 30 == 0) {  // Don't spam logs
            qWarning() << "[GestureWidget] Gesture processing failed:" << error;
        }
    }
}

void GestureWidget::updateGestureHistory(const QString& gesture, float confidence) {
    if (!ui->history_text_edit) {
        return;
    }

    // Get current timestamp
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss.zzz");

    // Format history entry
    QString entry = QString("[%1] %2 (%.2f)")
                        .arg(timestamp)
                        .arg(gesture)
                        .arg(confidence);

    // Prepend to history (newest first)
    QString currentText = ui->history_text_edit->toPlainText();
    QStringList lines = currentText.split("\n");

    // Limit history size
    if (lines.size() > MAX_HISTORY_ENTRIES) {
        lines.removeLast();
    }

    lines.prepend(entry);
    ui->history_text_edit->setPlainText(lines.join("\n"));
}

void GestureWidget::updatePerformanceStats(double fps, double processingTime) {
    // Update FPS label
    if (ui->fps_label) {
        ui->fps_label->setText(QString("FPS: %1").arg(fps, 0, 'f', 1));
    }

    // Update processing time label
    if (ui->processing_time_label) {
        ui->processing_time_label->setText(QString("Time: %1ms").arg(processingTime, 0, 'f', 1));
    }
}

void GestureWidget::updateGestureStatistics() {
    // Update total count
    if (ui->total_gestures_label) {
        ui->total_gestures_label->setText(QString("Total: %1").arg(gesture_count_));
    }

    // Update individual gesture counts
    // TODO: Update UI labels for each gesture type
    // For now, just log
    if (frame_count_ % 30 == 0) {  // Log every 30 frames
        qDebug() << "[GestureWidget] Gesture statistics:";
        for (const auto& pair : gesture_histogram_) {
            QString gestureName = QString::fromStdString(gesture::gesture_type_to_string(pair.first));
            qDebug() << "  " << gestureName << ":" << pair.second;
        }
    }
}

void GestureWidget::displayCameraFrame(const cv::Mat& frame) {
    if (!camera_feed_label_ || frame.empty()) {
        return;
    }

    // Convert to QPixmap and display
    QPixmap pixmap = matToQPixmap(frame);
    QSize preview_size = getResponsivePreviewSize();
    QPixmap scaled_pixmap = scalePixmapToFit(pixmap, preview_size);

    camera_feed_label_->setPixmap(scaled_pixmap);
}

QPixmap GestureWidget::matToQPixmap(const cv::Mat& mat) {
    if (mat.empty()) {
        return QPixmap();
    }

    cv::Mat rgb;
    if (mat.channels() == 1) {
        // Grayscale
        cv::cvtColor(mat, rgb, cv::COLOR_GRAY2RGB);
    } else if (mat.channels() == 3) {
        // BGR to RGB
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
    } else if (mat.channels() == 4) {
        // BGRA to RGB
        cv::cvtColor(mat, rgb, cv::COLOR_BGRA2RGB);
    } else {
        return QPixmap();
    }

    // Create QImage with deep copy
    QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
    QImage qimg_copy = qimg.copy();

    return QPixmap::fromImage(qimg_copy);
}

QPixmap GestureWidget::scalePixmapToFit(const QPixmap& pixmap, const QSize& size) {
    if (pixmap.isNull()) {
        return QPixmap();
    }

    return pixmap.scaled(size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
}

QSize GestureWidget::getResponsivePreviewSize() {
    const auto& metrics = styles::DisplayMetrics::instance();

    if (metrics.shouldUseCompactMode()) {
        // Compact mode: smaller preview
        return QSize(480, 360);
    } else {
        // Desktop mode: larger preview
        return QSize(640, 480);
    }
}

void GestureWidget::onConfidenceChanged(double value) {
    qDebug() << "[GestureWidget] Confidence threshold changed to:" << value;

    // Update gesture system configuration
    if (gesture_system_) {
        gesture::GestureConfig config = gesture_system_->get_config();
        config.min_gesture_confidence = static_cast<float>(value);

        if (!is_running_) {
            gesture_system_->set_config(config);
        } else {
            qWarning() << "[GestureWidget] Cannot change config while running";
        }
    }
}

void GestureWidget::onDebugVisualizationToggled(bool enabled) {
    debug_visualization_enabled_ = enabled;
    qDebug() << "[GestureWidget] Debug visualization:" << (enabled ? "enabled" : "disabled");

    if (gesture_system_) {
        gesture_system_->set_debug_visualization(enabled);
    }
}

void GestureWidget::onClearHistoryClicked() {
    if (ui->history_text_edit) {
        ui->history_text_edit->clear();
    }

    // Reset statistics
    gesture_count_ = 0;
    gesture_histogram_.clear();

    updateGestureStatistics();

    qDebug() << "[GestureWidget] History and statistics cleared";
}

void GestureWidget::updateFPSDisplay() {
    static int last_frame_count = 0;
    current_fps_ = frame_count_ - last_frame_count;
    last_frame_count = frame_count_;

    // Update will be done in updatePerformanceStats()
}

} // namespace gui
} // namespace unlook
