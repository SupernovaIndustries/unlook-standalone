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
#include <QShowEvent>
#include <QHideEvent>
#include <fstream>
#include <chrono>
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
    , targetCaptures_(100)
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
    captureTimer_->setInterval(3000);  // 3 seconds countdown after 5s continuous detection
    connect(captureTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCaptureFrame);

    // Initialize countdown timer (1 second interval)
    countdownTimer_ = new QTimer(this);
    countdownTimer_->setSingleShot(false);
    countdownTimer_->setInterval(1000);  // 1 second ticks
    connect(countdownTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCountdownTick);
    countdownValue_ = 0;

    // Initialize continuous detection tracking (countdown starts only after 5s continuous detection)
    isDetectedContinuously_ = false;
    continuousDetectionSeconds_ = 0.0;

    // Initialize coverage tracking for guided calibration
    initializeCoverageZones();

    // Start camera preview using callback system (same as camera_preview_widget)
    startPreviewCapture();
}

DatasetCaptureWidget::~DatasetCaptureWidget() {
    if (isCapturing_) {
        captureTimer_->stop();
        countdownTimer_->stop();
        ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
    }
    // Stop preview capture when destroying widget
    if (previewActive_ && cameraSystem_) {
        cameraSystem_->stopCapture();
        previewActive_ = false;
    }
    delete ui;
}

void DatasetCaptureWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);

    // Restart preview capture when widget becomes visible
    // (in case it was stopped by another widget like handheld scan)
    if (!previewActive_ && cameraSystem_) {
        qDebug() << "[DatasetCapture] Widget shown, restarting preview capture";
        startPreviewCapture();
    }
}

void DatasetCaptureWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);

    // Don't stop capture on hide - let other widgets take over if needed
    // Preview will restart automatically when we're shown again (showEvent)
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
        // Countdown finished - capture happens NOW via timer timeout
        countdownTimer_->stop();
        countdownLabel_->setText("📸");  // Just camera icon - capture imminent
        countdownLabel_->setStyleSheet("font-size: 24pt; color: #10b981; font-weight: bold;");
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
    // Adaptive skip factor to avoid lag:
    // - Preview mode (not capturing): skip 20 frames (~1.5 FPS detection)
    // - Capturing + countdown active: skip 5 frames (~6 FPS detection - tracking mode)
    // - Capturing + no countdown: skip 12 frames (~2.5 FPS detection - search mode)
    static int frame_skip_counter = 0;
    int skip_factor;
    if (!isCapturing_) {
        skip_factor = 20;  // Preview mode: very slow detection to avoid lag
    } else if (captureTimer_->isActive()) {
        skip_factor = 5;   // Countdown active: faster detection for tracking
    } else {
        skip_factor = 12;  // Capturing but searching: moderate detection to avoid lag
    }

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

    // CRITICAL: Continuous detection tracking for countdown trigger
    // Countdown starts ONLY after 5 seconds of continuous checkerboard detection
    if (isCapturing_) {
        if (leftDetected && rightDetected) {
            // Checkerboard detected on BOTH cameras
            if (!isDetectedContinuously_) {
                // First detection - start tracking time
                firstDetectionTime_ = std::chrono::steady_clock::now();
                isDetectedContinuously_ = true;
                continuousDetectionSeconds_ = 0.0;
                qDebug() << "[Countdown] Checkerboard detected - tracking continuous detection...";
            } else {
                // Already detecting - calculate elapsed time
                auto now = std::chrono::steady_clock::now();
                continuousDetectionSeconds_ = std::chrono::duration<double>(now - firstDetectionTime_).count();

                // Start countdown ONLY after 5 seconds continuous detection
                if (continuousDetectionSeconds_ >= 5.0 && !captureTimer_->isActive() && !countdownTimer_->isActive()) {
                    qDebug() << "[Countdown] 5 seconds continuous detection - STARTING 3 SECOND COUNTDOWN!";
                    countdownValue_ = 3;  // 3 second countdown
                    countdownLabel_->setText("Next capture in: 3");
                    countdownLabel_->setStyleSheet("font-size: 18pt; color: #f97316; font-weight: bold;");
                    countdownTimer_->start();
                    captureTimer_->start();
                }
            }
        } else {
            // Checkerboard NOT detected - reset continuous tracking
            if (isDetectedContinuously_) {
                isDetectedContinuously_ = false;
                continuousDetectionSeconds_ = 0.0;
                qDebug() << "[Countdown] Checkerboard lost - resetting continuous detection tracking";
            }
        }
    }

    // CRITICAL: Draw coverage tracking overlay on BOTH previews
    // Shows real-time guided coverage to ensure complete FOV calibration
    if (isCapturing_) {
        // Draw coverage zones and current checkerboard position
        if (leftDetected) {
            drawCoverageOverlay(overlayLeft, cornersLeft);
        } else {
            // Draw zones without checkerboard highlight
            drawCoverageOverlay(overlayLeft, std::vector<cv::Point2f>());
        }

        if (rightDetected) {
            drawCoverageOverlay(overlayRight, cornersRight);
        } else {
            // Draw zones without checkerboard highlight
            drawCoverageOverlay(overlayRight, std::vector<cv::Point2f>());
        }
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
        startCaptureButton_->setText("Start Dataset Capture (100 pairs)");
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
    datasetInfo_["capture_config"]["capture_delay_seconds"] = 3;  // 3 second countdown (after 5s continuous detection)
    datasetInfo_["capture_config"]["target_image_pairs"] = 100;
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
    targetCaptures_ = 100;
    isCapturing_ = true;

    // Reset coverage tracking for new dataset
    initializeCoverageZones();

    startCaptureButton_->setText("Stop Capture");
    startCaptureButton_->setStyleSheet(
        "QPushButton { background-color: #dc2626; color: white; font-size: 14pt; "
        "padding: 12px; border-radius: 8px; font-weight: bold; }");

    // Save initial JSON file with configuration
    saveDatasetInfo();

    // Countdown will start automatically after 5 seconds of continuous checkerboard detection
    // (see updatePreview() logic)
    countdownLabel_->setText("Hold checkerboard steady for 5 seconds to start capture...");
    countdownLabel_->setStyleSheet("font-size: 16pt; color: #3b82f6; font-weight: bold;");
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

        startCaptureButton_->setText("Start Dataset Capture (100 pairs)");
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

    // Show immediate feedback: CAPTURED!
    countdownLabel_->setText(QString("✓ CAPTURED! (%1/%2)").arg(captureCount_).arg(targetCaptures_));
    countdownLabel_->setStyleSheet("font-size: 18pt; color: #10b981; font-weight: bold;");

    // Save JSON after each capture (progressive save)
    saveDatasetInfo();

    // Brief pause to show "CAPTURED!" message (500ms)
    QTimer::singleShot(500, this, [this]() {
        // Reset continuous detection tracking for next capture
        // User must hold checkerboard steady for 5 seconds again
        isDetectedContinuously_ = false;
        continuousDetectionSeconds_ = 0.0;
        countdownLabel_->setText("Hold checkerboard steady for 5 seconds...");
        countdownLabel_->setStyleSheet("font-size: 16pt; color: #3b82f6; font-weight: bold;");
    });

    qDebug() << "[Countdown] Frame" << captureCount_ << "captured - reset detection tracking for next capture";
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

    // CAMERA MAPPING FIX: Swap left_frame ↔ right_frame to match stereo convention
    // Physical setup: Camera 0 (SLAVE) on right, Camera 1 (MASTER) on left (looking forward)
    // Software maps: Camera 1 → left_frame, Camera 0 → right_frame
    // For stereo convention (RIGHT camera at physical right):
    //   - Camera 0 (SLAVE, right_frame) → /left/ directory
    //   - Camera 1 (MASTER, left_frame) → /right/ directory
    cv::Mat leftFrame = framePair.right_frame.image.clone();   // Camera 0 SLAVE → LEFT
    cv::Mat rightFrame = framePair.left_frame.image.clone();   // Camera 1 MASTER → RIGHT

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

    // CRITICAL: Update coverage tracking when pattern detected on BOTH cameras
    // This ensures we track which zones of FOV have been covered by calibration
    if (leftDetected && rightDetected) {
        // Update using left camera corners (could use either, tracking is per-camera)
        // Corners are at HD resolution (1280x720), need to scale to VGA for tracking
        std::vector<cv::Point2f> cornersVGA;
        for (const auto& corner : cornersLeft) {
            cornersVGA.push_back(cv::Point2f(corner.x * 0.5f, corner.y * 0.5f));
        }
        updateCoverageTracking(cornersVGA);

        qDebug() << "[Coverage] Frame" << captureCount_
                 << "captured - Coverage:" << QString::fromStdString(getCoverageSummary());
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

// ============================================================================
// COVERAGE TRACKING IMPLEMENTATION
// ============================================================================
// CRITICAL: Guided calibration coverage tracking to ensure complete FOV coverage
// Research findings (CALIBRATION_RESEARCH_FINDINGS.md):
// - Coverage insufficiency causes extrapolation failures at image edges
// - Need < 30px margins on all edges (2-3% of image dimension)
// - Checkerboard at center = 0.26px epipolar error, but 58px at edges!
// - Solution: 9-zone coverage tracking with real-time visual guidance

void DatasetCaptureWidget::initializeCoverageZones() {
    // Initialize for 1280x720 image (HD resolution after downsampling)
    const int width = 1280;
    const int height = 720;

    // Divide into 3x3 grid for comprehensive coverage tracking
    // Each cell is approximately 426x240 pixels
    const int cellWidth = width / 3;
    const int cellHeight = height / 3;

    coverageZones_.clear();
    totalFramesCaptured_ = 0;

    // Define 9 zones: 4 corners + 4 edges + center
    // Zone naming follows grid layout (scanner POV looking forward)
    // Target distribution (total 100 frames):
    // - CENTER: 20 frames (most important for baseline accuracy)
    // - 4 EDGES: 10 frames each = 40 frames (critical for edge calibration)
    // - 4 CORNERS: 10 frames each = 40 frames (complete FOV coverage)

    // Row 1: TOP zones
    coverageZones_.push_back({
        "TOP-LEFT",
        cv::Rect(0, 0, cellWidth, cellHeight),
        0, 10, false  // CORNER: 10 frames target
    });
    coverageZones_.push_back({
        "TOP",
        cv::Rect(cellWidth, 0, cellWidth, cellHeight),
        0, 10, false  // EDGE: 10 frames target
    });
    coverageZones_.push_back({
        "TOP-RIGHT",
        cv::Rect(cellWidth * 2, 0, cellWidth, cellHeight),
        0, 10, false  // CORNER: 10 frames target
    });

    // Row 2: MIDDLE zones
    coverageZones_.push_back({
        "LEFT",
        cv::Rect(0, cellHeight, cellWidth, cellHeight),
        0, 10, false  // EDGE: 10 frames target
    });
    coverageZones_.push_back({
        "CENTER",
        cv::Rect(cellWidth, cellHeight, cellWidth, cellHeight),
        0, 20, false  // CENTER: 20 frames target (most important!)
    });
    coverageZones_.push_back({
        "RIGHT",
        cv::Rect(cellWidth * 2, cellHeight, cellWidth, cellHeight),
        0, 10, false  // EDGE: 10 frames target
    });

    // Row 3: BOTTOM zones
    coverageZones_.push_back({
        "BOTTOM-LEFT",
        cv::Rect(0, cellHeight * 2, cellWidth, cellHeight),
        0, 10, false  // CORNER: 10 frames target
    });
    coverageZones_.push_back({
        "BOTTOM",
        cv::Rect(cellWidth, cellHeight * 2, cellWidth, cellHeight),
        0, 10, false  // EDGE: 10 frames target
    });
    coverageZones_.push_back({
        "BOTTOM-RIGHT",
        cv::Rect(cellWidth * 2, cellHeight * 2, cellWidth, cellHeight),
        0, 10, false  // CORNER: 10 frames target
    });

    qDebug() << "[Coverage] Initialized 9 coverage zones for 1280x720 image";
}

void DatasetCaptureWidget::updateCoverageTracking(const std::vector<cv::Point2f>& corners) {
    if (corners.empty() || coverageZones_.empty()) {
        return;
    }

    // Calculate bounding box of detected checkerboard corners
    float minX = corners[0].x, maxX = corners[0].x;
    float minY = corners[0].y, maxY = corners[0].y;

    for (const auto& corner : corners) {
        minX = std::min(minX, corner.x);
        maxX = std::max(maxX, corner.x);
        minY = std::min(minY, corner.y);
        maxY = std::max(maxY, corner.y);
    }

    // Create bounding rectangle (convert from VGA 640x360 to HD 1280x720)
    // corners are detected on VGA, so we need to scale 2x for HD zones
    cv::Rect checkerboardBounds(
        static_cast<int>(minX * 2.0f),
        static_cast<int>(minY * 2.0f),
        static_cast<int>((maxX - minX) * 2.0f),
        static_cast<int>((maxY - minY) * 2.0f)
    );

    // Check which zones overlap with checkerboard bounding box
    for (auto& zone : coverageZones_) {
        // Check for intersection between zone and checkerboard bounds
        cv::Rect intersection = zone.rect & checkerboardBounds;

        if (intersection.area() > 0) {
            // Zone is covered by this checkerboard position
            zone.captureCount++;

            // Update adequately covered status (uses targetCount for each zone)
            if (zone.captureCount >= zone.targetCount) {
                zone.adequatelyCovered = true;
            }
        }
    }

    totalFramesCaptured_++;
}

void DatasetCaptureWidget::drawCoverageOverlay(cv::Mat& image, const std::vector<cv::Point2f>& corners) {
    if (coverageZones_.empty()) {
        return;
    }

    // Draw semi-transparent overlay for each zone
    for (const auto& zone : coverageZones_) {
        // Determine color based on coverage status
        cv::Scalar color;
        if (zone.captureCount == 0) {
            color = cv::Scalar(0, 0, 255);      // RED - not covered
        } else if (zone.captureCount < zone.targetCount) {
            color = cv::Scalar(0, 165, 255);    // ORANGE - partial coverage
        } else {
            color = cv::Scalar(0, 255, 0);      // GREEN - adequate coverage (reached target!)
        }

        // Draw semi-transparent filled rectangle
        cv::Mat overlay = image.clone();
        cv::rectangle(overlay, zone.rect, color, -1);  // -1 = filled
        cv::addWeighted(overlay, 0.15, image, 0.85, 0, image);  // 15% overlay, 85% original

        // Draw zone border
        cv::rectangle(image, zone.rect, color, 2);

        // Draw capture progress text in zone center: "count/target"
        std::string countText = std::to_string(zone.captureCount) + "/" + std::to_string(zone.targetCount);
        int baseline = 0;
        cv::Size textSize = cv::getTextSize(countText, cv::FONT_HERSHEY_SIMPLEX, 0.6, 2, &baseline);

        cv::Point textPos(
            zone.rect.x + (zone.rect.width - textSize.width) / 2,
            zone.rect.y + (zone.rect.height + textSize.height) / 2
        );

        // Draw text background for readability
        cv::rectangle(image,
            cv::Point(textPos.x - 5, textPos.y - textSize.height - 5),
            cv::Point(textPos.x + textSize.width + 5, textPos.y + 5),
            cv::Scalar(0, 0, 0), -1);

        cv::putText(image, countText, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                   cv::Scalar(255, 255, 255), 2);
    }

    // If corners provided, draw checkerboard bounding box (scaled from VGA to HD)
    if (!corners.empty()) {
        float minX = corners[0].x, maxX = corners[0].x;
        float minY = corners[0].y, maxY = corners[0].y;

        for (const auto& corner : corners) {
            minX = std::min(minX, corner.x);
            maxX = std::max(maxX, corner.x);
            minY = std::min(minY, corner.y);
            maxY = std::max(maxY, corner.y);
        }

        // Scale from VGA (640x360) to HD (1280x720) - 2x
        cv::Rect checkerboardBounds(
            static_cast<int>(minX * 2.0f),
            static_cast<int>(minY * 2.0f),
            static_cast<int>((maxX - minX) * 2.0f),
            static_cast<int>((maxY - minY) * 2.0f)
        );

        // Draw bright yellow bounding box for current checkerboard position
        cv::rectangle(image, checkerboardBounds, cv::Scalar(0, 255, 255), 3);
    }

    // Draw coverage summary text at bottom
    std::string summary = getCoverageSummary();
    cv::putText(image, summary, cv::Point(10, image.rows - 10),
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
}

std::string DatasetCaptureWidget::getCoverageSummary() const {
    if (coverageZones_.empty()) {
        return "Coverage: Not initialized";
    }

    // Count adequately covered zones
    int adequatelyCovered = 0;
    int totalZones = static_cast<int>(coverageZones_.size());

    for (const auto& zone : coverageZones_) {
        if (zone.adequatelyCovered) {
            adequatelyCovered++;
        }
    }

    // Find zones needing coverage
    std::vector<std::string> zonesNeedingCoverage;
    for (const auto& zone : coverageZones_) {
        if (!zone.adequatelyCovered) {
            zonesNeedingCoverage.push_back(zone.name);
        }
    }

    // Build summary string
    std::string summary = "Coverage: " + std::to_string(adequatelyCovered) + "/" +
                         std::to_string(totalZones) + " zones";

    if (!zonesNeedingCoverage.empty() && zonesNeedingCoverage.size() <= 3) {
        summary += " | Need: ";
        for (size_t i = 0; i < zonesNeedingCoverage.size(); i++) {
            if (i > 0) summary += ", ";
            summary += zonesNeedingCoverage[i];
        }
    }

    return summary;
}

} // namespace gui
} // namespace unlook
