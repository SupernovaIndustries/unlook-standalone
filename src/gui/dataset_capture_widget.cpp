#include "unlook/gui/dataset_capture_widget.hpp"
#include "unlook/gui/board_options_dialog.hpp"
#include "ui_dataset_capture_widget.h"
#include "unlook/camera/CameraSystem.hpp"
#include "unlook/hardware/AS1170Controller.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QMessageBox>
#include <QDir>
#include <QDateTime>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <fstream>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>

namespace unlook {
namespace gui {

DatasetCaptureWidget::DatasetCaptureWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::DatasetCaptureWidget)
    , cameraSystem_(camera_system)
    , isCapturing_(false)
    , previewActive_(false)
    , captureCount_(0)
    , targetCaptures_(50)
{
    // Get LED controller singleton instance (same as camera_preview_widget)
    ledController_ = hardware::AS1170Controller::getInstance();

    // Setup UI from .ui file
    setupUi();

    // Setup signal/slot connections
    setupConnections();

    // Initialize default pattern config (ChArUco 7x10, 24mm squares, 17mm ArUco)
    currentPatternConfig_.type = calibration::PatternType::CHARUCO;
    currentPatternConfig_.rows = 7;
    currentPatternConfig_.cols = 10;
    currentPatternConfig_.squareSizeMM = 24.0f;
    currentPatternConfig_.arucoMarkerSizeMM = 17.0f;
    currentPatternConfig_.arucoDict = cv::aruco::DICT_4X4_250;

    // Initialize pattern detector with default config
    patternDetector_ = std::make_unique<calibration::PatternDetector>(currentPatternConfig_);

    // Initialize capture timer (not started yet)
    captureTimer_ = new QTimer(this);
    captureTimer_->setSingleShot(true);
    captureTimer_->setInterval(5000);  // 5 seconds between captures
    connect(captureTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCaptureFrame);

    // Initialize countdown timer (1 second interval)
    countdownTimer_ = new QTimer(this);
    countdownTimer_->setSingleShot(false);
    countdownTimer_->setInterval(1000);  // 1 second ticks
    connect(countdownTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCountdownTick);
    countdownValue_ = 0;

    // Start camera preview using callback system (same as camera_preview_widget)
    startPreviewCapture();
}

DatasetCaptureWidget::~DatasetCaptureWidget() {
    if (isCapturing_) {
        captureTimer_->stop();
        countdownTimer_->stop();
        ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
    }
    delete ui;
}

void DatasetCaptureWidget::setupUi() {
    // Load UI from .ui file
    ui->setupUi(this);

    // Set pointers to widgets from UI
    leftPreview_ = ui->left_preview;
    rightPreview_ = ui->right_preview;
    patternOverlayLeft_ = ui->pattern_overlay_left;
    patternOverlayRight_ = ui->pattern_overlay_right;

    // Capture controls
    boardOptionsButton_ = ui->board_options_button;
    startCaptureButton_ = ui->start_capture_button;
    captureProgress_ = ui->capture_progress;
    detectionStatusLabel_ = ui->detection_status_label;
    countdownLabel_ = ui->countdown_label;
}

void DatasetCaptureWidget::setupConnections() {
    // Board options button opens dialog
    connect(boardOptionsButton_, &QPushButton::clicked, this, &DatasetCaptureWidget::onBoardOptionsClicked);

    // Start capture button
    connect(startCaptureButton_, &QPushButton::clicked, this, &DatasetCaptureWidget::onStartCapture);
}

void DatasetCaptureWidget::onBoardOptionsClicked() {
    BoardOptionsDialog dialog(this);

    // Set current configuration
    dialog.setPatternConfig(currentPatternConfig_);

    // Connect to configuration changed signal
    connect(&dialog, &BoardOptionsDialog::configurationChanged,
            this, &DatasetCaptureWidget::onBoardConfigChanged);

    // Show dialog
    if (dialog.exec() == QDialog::Accepted) {
        // Update pattern detector with new config
        currentPatternConfig_ = dialog.getPatternConfig();

        std::lock_guard<std::mutex> lock(patternDetectorMutex_);
        patternDetector_ = std::make_unique<calibration::PatternDetector>(currentPatternConfig_);

        qDebug() << "Board configuration updated";
    }
}

void DatasetCaptureWidget::onBoardConfigChanged() {
    // Live update when changing values in dialog
    qDebug() << "Board configuration changed (live update)";
}

void DatasetCaptureWidget::onCountdownTick() {
    countdownValue_--;

    if (countdownValue_ > 0) {
        // Update countdown display
        countdownLabel_->setText(QString("Next capture in: %1").arg(countdownValue_));
        countdownLabel_->setStyleSheet("font-size: 18pt; color: #f97316; font-weight: bold;");
    } else {
        // Countdown finished
        countdownTimer_->stop();
        countdownLabel_->setText("📸 CAPTURING...");
        countdownLabel_->setStyleSheet("font-size: 18pt; color: #10b981; font-weight: bold;");
    }
}

// Start camera preview using callback system (same as camera_preview_widget)
void DatasetCaptureWidget::startPreviewCapture() {
    if (!cameraSystem_ || previewActive_) return;

    qDebug() << "[DatasetCapture] Starting camera preview capture...";

    // Set frame callback for preview (same system as camera_preview_widget)
    auto frame_callback = [this](const core::StereoFramePair& frame_pair) {
        // Use QMetaObject::invokeMethod for thread-safe GUI updates
        QMetaObject::invokeMethod(this, [this, frame_pair]() {
            updatePreview(frame_pair);
        }, Qt::QueuedConnection);
    };

    if (cameraSystem_->startCapture(frame_callback)) {
        previewActive_ = true;
        qDebug() << "[DatasetCapture] Successfully started camera preview";
    } else {
        qDebug() << "[DatasetCapture] Failed to start camera preview";
    }
}

// Stop camera preview
void DatasetCaptureWidget::stopPreviewCapture() {
    if (!previewActive_) return;

    qDebug() << "[DatasetCapture] Switching to background capture mode";

    // Create background callback to keep cameras running
    auto background_callback = [](const core::StereoFramePair& /*frame_pair*/) {
        // Frame received but not processed
    };

    cameraSystem_->startCapture(background_callback);
    previewActive_ = false;
}

// Update preview from camera callback (same as camera_preview_widget)
void DatasetCaptureWidget::updatePreview(const core::StereoFramePair& frame_pair) {
    if (!previewActive_) return;

    // Save latest frame for capture (thread-safe)
    {
        std::lock_guard<std::mutex> lock(latestFrameMutex_);
        latestFramePair_ = frame_pair;
    }

    // Frame skipping for performance - CRITICAL: checkerboard detection is HEAVY
    static int frame_skip_counter = 0;
    int skip_factor = isCapturing_ ? 3 : 10;  // Preview: skip 10 frames (~3 FPS detection), Capture: skip 3
    if (++frame_skip_counter % skip_factor != 0) {
        return;
    }

    // Extract frames (already in BGRA format from core::CameraFrame)
    cv::Mat leftRGB, rightRGB;
    cv::cvtColor(frame_pair.left_frame.image, leftRGB, cv::COLOR_BGRA2RGB);
    cv::cvtColor(frame_pair.right_frame.image, rightRGB, cv::COLOR_BGRA2RGB);

    // CRITICAL OPTIMIZATION: Always detect on VGA (640x360) - 4x faster than HD!
    // Checkerboard detection is MUCH slower than ChArUco (no ArUco markers to guide search)
    cv::Mat leftVGA, rightVGA;
    cv::resize(leftRGB, leftVGA, cv::Size(640, 360), 0, 0, cv::INTER_AREA);
    cv::resize(rightRGB, rightVGA, cv::Size(640, 360), 0, 0, cv::INTER_AREA);

    // Detect pattern on VGA resolution (much faster)
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat overlayLeftVGA, overlayRightVGA;
    bool leftDetected = false;
    bool rightDetected = false;

    {
        std::lock_guard<std::mutex> lock(patternDetectorMutex_);
        if (patternDetector_) {
            leftDetected = patternDetector_->detect(leftVGA, cornersLeft, overlayLeftVGA);
            rightDetected = patternDetector_->detect(rightVGA, cornersRight, overlayRightVGA);
        }
    }

    // Scale overlay back to HD for display
    cv::Mat overlayLeft, overlayRight;
    if (!overlayLeftVGA.empty()) {
        cv::resize(overlayLeftVGA, overlayLeft, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    } else {
        cv::resize(leftVGA, overlayLeft, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    }

    if (!overlayRightVGA.empty()) {
        cv::resize(overlayRightVGA, overlayRight, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    } else {
        cv::resize(rightVGA, overlayRight, cv::Size(1280, 720), 0, 0, cv::INTER_LINEAR);
    }

    // Update detection status
    if (leftDetected && rightDetected) {
        detectionStatusLabel_->setText("✓ Pattern Detected");
        detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: #00FF00; font-weight: bold;");
        patternOverlayLeft_->setText(QString("✓ Detected: %1 corners").arg(cornersLeft.size()));
        patternOverlayLeft_->setStyleSheet("font-size: 11pt; color: #00FF00;");
        patternOverlayRight_->setText(QString("✓ Detected: %1 corners").arg(cornersRight.size()));
        patternOverlayRight_->setStyleSheet("font-size: 11pt; color: #00FF00;");
    } else {
        detectionStatusLabel_->setText("✗ Pattern Not Detected");
        detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: #FF0000; font-weight: bold;");
        patternOverlayLeft_->setText(leftDetected ? "✓ Detected" : "✗ Not detected");
        patternOverlayLeft_->setStyleSheet(leftDetected ? "font-size: 11pt; color: #00FF00;" : "font-size: 11pt; color: #FF0000;");
        patternOverlayRight_->setText(rightDetected ? "✓ Detected" : "✗ Not detected");
        patternOverlayRight_->setStyleSheet(rightDetected ? "font-size: 11pt; color: #00FF00;" : "font-size: 11pt; color: #FF0000;");
    }

    // Convert to QPixmap and display (with deep copy for safety)
    if (!overlayLeft.empty()) {
        QImage qimgLeft(overlayLeft.data, overlayLeft.cols, overlayLeft.rows,
                       overlayLeft.step, QImage::Format_RGB888);
        QImage qimgLeftCopy = qimgLeft.copy();  // Deep copy for safety
        leftPreview_->setPixmap(QPixmap::fromImage(qimgLeftCopy));
    }

    if (!overlayRight.empty()) {
        QImage qimgRight(overlayRight.data, overlayRight.cols, overlayRight.rows,
                        overlayRight.step, QImage::Format_RGB888);
        QImage qimgRightCopy = qimgRight.copy();  // Deep copy for safety
        rightPreview_->setPixmap(QPixmap::fromImage(qimgRightCopy));
    }
}

void DatasetCaptureWidget::onStartCapture() {
    if (isCapturing_) {
        // Stop capture
        captureTimer_->stop();
        countdownTimer_->stop();
        isCapturing_ = false;
        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        startCaptureButton_->setStyleSheet(
            "QPushButton { background-color: #059669; color: white; font-size: 14pt; "
            "padding: 12px; border-radius: 8px; font-weight: bold; }");

        // Clear countdown display
        countdownLabel_->setText("");

        // Disable both LEDs (ensure both are OFF)
        if (ledController_) {
            bool led1_off = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
            bool led2_off = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

            if (led1_off) {
                qDebug() << "[DatasetCapture] LED1 (VCSEL) disabled";
            }
            if (led2_off) {
                qDebug() << "[DatasetCapture] LED2 (Flood) disabled";
            }
        }
        return;
    }

    // Create dataset directory
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    currentDatasetPath_ = "/unlook_calib_dataset/dataset_" + timestamp;

    QDir().mkpath(currentDatasetPath_ + "/left");
    QDir().mkpath(currentDatasetPath_ + "/right");

    // Initialize dataset info JSON
    datasetInfo_["dataset_info"]["timestamp"] = timestamp.toStdString();
    datasetInfo_["dataset_info"]["creation_date"] =
        QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
    datasetInfo_["dataset_info"]["dataset_path"] = currentDatasetPath_.toStdString();

    // Pattern config (use currentPatternConfig_ instead of widget values)
    QString patternTypeName;
    switch (currentPatternConfig_.type) {
        case calibration::PatternType::CHECKERBOARD:
            patternTypeName = "Checkerboard";
            break;
        case calibration::PatternType::CHARUCO:
            patternTypeName = "ChArUco";
            break;
        case calibration::PatternType::CIRCLE_GRID:
            patternTypeName = "Circle Grid";
            break;
        default:
            patternTypeName = "Unknown";
    }

    datasetInfo_["pattern_config"]["type"] = patternTypeName.toStdString();
    datasetInfo_["pattern_config"]["rows"] = currentPatternConfig_.rows;
    datasetInfo_["pattern_config"]["cols"] = currentPatternConfig_.cols;
    datasetInfo_["pattern_config"]["square_size_mm"] = currentPatternConfig_.squareSizeMM;

    // Only save ArUco-specific fields for ChArUco pattern
    if (currentPatternConfig_.type == calibration::PatternType::CHARUCO) {
        datasetInfo_["pattern_config"]["aruco_marker_size_mm"] = currentPatternConfig_.arucoMarkerSizeMM;
        datasetInfo_["pattern_config"]["aruco_dict"] = currentPatternConfig_.arucoDict;
    }

    // Capture config
    datasetInfo_["capture_config"]["image_width"] = 1280;
    datasetInfo_["capture_config"]["image_height"] = 720;
    datasetInfo_["capture_config"]["capture_delay_seconds"] = 5;
    datasetInfo_["capture_config"]["target_image_pairs"] = 50;
    datasetInfo_["capture_config"]["led1_vcsel_enabled"] = false;
    datasetInfo_["capture_config"]["led2_flood_enabled"] = false;
    datasetInfo_["capture_config"]["lighting"] = "ambient_only";

    // Initialize and enable Flood LED (LED2 only for calibration)
    if (ledController_) {
        // Initialize if not already done
        if (!ledController_->isInitialized()) {
            qDebug() << "[DatasetCapture] Initializing AS1170 controller";

            // CRITICAL: Force reset hardware BEFORE initialization
            qDebug() << "[DatasetCapture] Forcing AS1170 hardware reset to clear stuck state";
            ledController_->forceResetHardware();

            if (!ledController_->initialize()) {
                qWarning() << "[DatasetCapture] Failed to initialize AS1170 controller - LEDs will not work";
            } else {
                qDebug() << "[DatasetCapture] AS1170 controller initialized successfully";
            }
        }

        // For calibration: DISABLE ALL LEDs - use ambient lighting only
        // LED1 (VCSEL) OFF - to avoid pattern interference
        // LED2 (Flood) OFF - use external lighting (neon + softbox)
        bool led1_success = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        bool led2_success = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

        if (led1_success) {
            qDebug() << "[DatasetCapture] LED1 (VCSEL) disabled for calibration";
        } else {
            qWarning() << "[DatasetCapture] Failed to disable LED1 (VCSEL)";
        }

        if (led2_success) {
            qDebug() << "[DatasetCapture] LED2 (Flood) disabled - using ambient lighting";
        } else {
            qWarning() << "[DatasetCapture] Failed to disable LED2";
        }
    }

    // Start capture
    captureCount_ = 0;
    targetCaptures_ = 50;
    isCapturing_ = true;

    startCaptureButton_->setText("Stop Capture");
    startCaptureButton_->setStyleSheet(
        "QPushButton { background-color: #dc2626; color: white; font-size: 14pt; "
        "padding: 12px; border-radius: 8px; font-weight: bold; }");

    // Save initial JSON file with configuration
    saveDatasetInfo();

    // Start countdown for first capture
    countdownValue_ = 5;
    countdownLabel_->setText("Next capture in: 5");
    countdownTimer_->start();
    captureTimer_->start();
}

void DatasetCaptureWidget::onCaptureFrame() {
    if (captureCount_ >= targetCaptures_) {
        // Capture complete
        captureTimer_->stop();
        countdownTimer_->stop();
        isCapturing_ = false;

        // Disable both LEDs (ensure both are OFF)
        if (ledController_) {
            bool led1_off = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
            bool led2_off = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

            if (led1_off) {
                qDebug() << "[DatasetCapture] LED1 (VCSEL) disabled after completion";
            }
            if (led2_off) {
                qDebug() << "[DatasetCapture] LED2 (Flood) disabled after completion";
            }
        }

        // Clear countdown display
        countdownLabel_->setText("");

        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        startCaptureButton_->setStyleSheet(
            "QPushButton { background-color: #059669; color: white; font-size: 14pt; "
            "padding: 12px; border-radius: 8px; font-weight: bold; }");

        QMessageBox::information(this, "Capture Complete",
            "Dataset saved to:\n" + currentDatasetPath_ +
            "\n\nProceed to Processing tab to calibrate.");

        emit datasetCaptureCompleted(currentDatasetPath_);
        return;
    }

    // Capture and save frame pair
    captureAndSaveFrame();
    captureCount_++;
    captureProgress_->setValue(captureCount_);

    // Save JSON after each capture (progressive save)
    saveDatasetInfo();

    // Start countdown for next capture
    countdownValue_ = 5;
    countdownLabel_->setText("Next capture in: 5");
    countdownTimer_->start();
    captureTimer_->start();  // Restart timer for next capture
}

void DatasetCaptureWidget::captureAndSaveFrame() {
    // Get latest frame from preview callback (thread-safe)
    core::StereoFramePair framePair;
    {
        std::lock_guard<std::mutex> lock(latestFrameMutex_);
        if (latestFramePair_.left_frame.image.empty() || latestFramePair_.right_frame.image.empty()) {
            qCritical() << "CRITICAL: No frame available for capture";
            return;
        }
        framePair = latestFramePair_;
    }

    // Extract frames (already in BGRA format)
    cv::Mat leftFrame = framePair.left_frame.image.clone();
    cv::Mat rightFrame = framePair.right_frame.image.clone();

    // Log sync quality (timestamp_ns is in nanoseconds, convert to milliseconds)
    double syncErrorMs = std::abs(static_cast<double>(framePair.left_frame.timestamp_ns) -
                                   static_cast<double>(framePair.right_frame.timestamp_ns)) / 1000000.0;
    if (syncErrorMs > 1.0) {
        qWarning() << "Frame" << captureCount_ << "sync error:" << syncErrorMs << "ms";
    }

    // Downsample to HD
    cv::Mat leftHD, rightHD;
    cv::resize(leftFrame, leftHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
    cv::resize(rightFrame, rightHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);

    // Save images
    QString leftPath = currentDatasetPath_ + "/left/frame_" +
                      QString::number(captureCount_).rightJustified(3, '0') + ".png";
    QString rightPath = currentDatasetPath_ + "/right/frame_" +
                       QString::number(captureCount_).rightJustified(3, '0') + ".png";

    cv::imwrite(leftPath.toStdString(), leftHD);
    cv::imwrite(rightPath.toStdString(), rightHD);

    // Detect pattern for quality info (thread-safe)
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat dummy;
    bool leftDetected, rightDetected;
    float qualityScore = 0.0f;
    {
        std::lock_guard<std::mutex> lock(patternDetectorMutex_);
        if (patternDetector_) {
            leftDetected = patternDetector_->detect(leftHD, cornersLeft, dummy);
            rightDetected = patternDetector_->detect(rightHD, cornersRight, dummy);
            qualityScore = (leftDetected && rightDetected) ?
                patternDetector_->getConfidenceScore() : 0.0f;
        } else {
            leftDetected = false;
            rightDetected = false;
            qualityScore = 0.0f;
        }
    }

    // Add to dataset info JSON with sync metadata
    nlohmann::json pairInfo;
    pairInfo["index"] = captureCount_;
    pairInfo["left_filename"] = ("left/frame_" +
        QString::number(captureCount_).rightJustified(3, '0') + ".png").toStdString();
    pairInfo["right_filename"] = ("right/frame_" +
        QString::number(captureCount_).rightJustified(3, '0') + ".png").toStdString();
    pairInfo["timestamp"] = QDateTime::currentDateTime().toString(Qt::ISODate).toStdString();
    pairInfo["corners_detected_left"] = cornersLeft.size();
    pairInfo["corners_detected_right"] = cornersRight.size();
    pairInfo["quality_score"] = qualityScore;

    // Add hardware sync metadata for validation
    pairInfo["sync_error_ms"] = syncErrorMs;
    pairInfo["is_synchronized"] = (syncErrorMs < 1.0);
    pairInfo["left_timestamp_ns"] = framePair.left_frame.timestamp_ns;
    pairInfo["right_timestamp_ns"] = framePair.right_frame.timestamp_ns;

    datasetInfo_["image_pairs"].push_back(pairInfo);
}

void DatasetCaptureWidget::saveDatasetInfo() {
    QString jsonPath = currentDatasetPath_ + "/dataset_info.json";
    std::ofstream file(jsonPath.toStdString());

    if (!file.is_open()) {
        qCritical() << "Failed to open dataset info file for writing:" << jsonPath;
        QMessageBox::critical(this, "Save Error",
            "Failed to save dataset info file:\n" + jsonPath +
            "\n\nCheck disk space and permissions.");
        return;
    }

    file << datasetInfo_.dump(2);  // Pretty print with 2-space indent

    if (!file.good()) {
        qCritical() << "Write error for dataset info file:" << jsonPath;
        QMessageBox::critical(this, "Save Error",
            "Write error for dataset info file:\n" + jsonPath +
            "\n\nDisk may be full or I/O error occurred.");
        file.close();
        return;
    }

    file.close();

    if (!file.good()) {
        qCritical() << "Error closing dataset info file:" << jsonPath;
    }
}

} // namespace gui
} // namespace unlook
