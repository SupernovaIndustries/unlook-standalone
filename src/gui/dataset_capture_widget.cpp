#include "unlook/gui/dataset_capture_widget.hpp"
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

namespace unlook {
namespace gui {

DatasetCaptureWidget::DatasetCaptureWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , cameraSystem_(camera_system)
    , isCapturing_(false)
    , previewActive_(false)
    , captureCount_(0)
    , targetCaptures_(50)
{
    // Get LED controller singleton instance (same as camera_preview_widget)
    ledController_ = hardware::AS1170Controller::getInstance();

    // Setup UI
    setupUi();

    // Initialize pattern detector with default config
    calibration::PatternConfig defaultConfig;
    patternDetector_ = std::make_unique<calibration::PatternDetector>(defaultConfig);

    // Initialize capture timer (not started yet)
    captureTimer_ = new QTimer(this);
    captureTimer_->setSingleShot(false);
    captureTimer_->setInterval(5000);  // 5 seconds between captures
    connect(captureTimer_, &QTimer::timeout, this, &DatasetCaptureWidget::onCaptureFrame);

    // Start camera preview using callback system (same as camera_preview_widget)
    startPreviewCapture();
}

DatasetCaptureWidget::~DatasetCaptureWidget() {
    if (isCapturing_) {
        captureTimer_->stop();
        ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
    }
}

void DatasetCaptureWidget::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // PREVIEW SECTION
    QHBoxLayout* previewLayout = new QHBoxLayout();

    // Left camera preview
    QVBoxLayout* leftLayout = new QVBoxLayout();
    QLabel* leftLabel = new QLabel("Left Camera");
    leftLabel->setStyleSheet("font-weight: bold; font-size: 14pt; color: #0e0e0e;");
    leftLabel->setAlignment(Qt::AlignCenter);

    leftPreview_ = new QLabel();
    leftPreview_->setMinimumSize(640, 360);
    leftPreview_->setScaledContents(true);
    leftPreview_->setStyleSheet("border: 2px solid #bebebe; background-color: #000000;");

    patternOverlayLeft_ = new QLabel("Move checkerboard into view");
    patternOverlayLeft_->setAlignment(Qt::AlignCenter);
    patternOverlayLeft_->setStyleSheet("font-size: 11pt; color: #FFA500;");

    leftLayout->addWidget(leftLabel);
    leftLayout->addWidget(leftPreview_);
    leftLayout->addWidget(patternOverlayLeft_);

    // Right camera preview
    QVBoxLayout* rightLayout = new QVBoxLayout();
    QLabel* rightLabel = new QLabel("Right Camera");
    rightLabel->setStyleSheet("font-weight: bold; font-size: 14pt; color: #0e0e0e;");
    rightLabel->setAlignment(Qt::AlignCenter);

    rightPreview_ = new QLabel();
    rightPreview_->setMinimumSize(640, 360);
    rightPreview_->setScaledContents(true);
    rightPreview_->setStyleSheet("border: 2px solid #bebebe; background-color: #000000;");

    patternOverlayRight_ = new QLabel("Move checkerboard into view");
    patternOverlayRight_->setAlignment(Qt::AlignCenter);
    patternOverlayRight_->setStyleSheet("font-size: 11pt; color: #FFA500;");

    rightLayout->addWidget(rightLabel);
    rightLayout->addWidget(rightPreview_);
    rightLayout->addWidget(patternOverlayRight_);

    previewLayout->addLayout(leftLayout);
    previewLayout->addLayout(rightLayout);
    mainLayout->addLayout(previewLayout);

    // PATTERN CONFIGURATION
    QGroupBox* configGroup = new QGroupBox("Pattern Configuration");
    configGroup->setStyleSheet("QGroupBox { font-weight: bold; color: #0e0e0e; }");
    QVBoxLayout* configLayout = new QVBoxLayout();

    // Pattern Type with cycle buttons
    QHBoxLayout* patternTypeLayout = new QHBoxLayout();
    QLabel* patternTypeLabel = new QLabel("Pattern Type:");
    patternTypeLabel->setStyleSheet("font-size: 11pt;");
    patternTypeCombo_ = new QComboBox();
    patternTypeCombo_->addItem("Checkerboard", (int)calibration::PatternType::CHECKERBOARD);
    patternTypeCombo_->addItem("ChArUco (Recommended)", (int)calibration::PatternType::CHARUCO);
    patternTypeCombo_->addItem("Circle Grid", (int)calibration::PatternType::CIRCLE_GRID);
    patternTypeCombo_->setCurrentIndex(1);  // ChArUco default
    patternTypeCombo_->setMinimumHeight(50);
    patternTypeCombo_->setStyleSheet("QComboBox { font-size: 11pt; padding: 8px; }");
    connect(patternTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DatasetCaptureWidget::onPatternTypeChanged);

    QPushButton* patternPrevBtn = new QPushButton("◀");
    QPushButton* patternNextBtn = new QPushButton("▶");
    patternPrevBtn->setMinimumSize(50, 50);
    patternNextBtn->setMinimumSize(50, 50);
    patternPrevBtn->setStyleSheet("QPushButton { font-size: 16pt; background-color: #3b82f6; color: white; }");
    patternNextBtn->setStyleSheet("QPushButton { font-size: 16pt; background-color: #3b82f6; color: white; }");
    connect(patternPrevBtn, &QPushButton::clicked, [this]() {
        int idx = patternTypeCombo_->currentIndex();
        patternTypeCombo_->setCurrentIndex((idx - 1 + patternTypeCombo_->count()) % patternTypeCombo_->count());
    });
    connect(patternNextBtn, &QPushButton::clicked, [this]() {
        int idx = patternTypeCombo_->currentIndex();
        patternTypeCombo_->setCurrentIndex((idx + 1) % patternTypeCombo_->count());
    });

    patternTypeLayout->addWidget(patternTypeLabel);
    patternTypeLayout->addWidget(patternPrevBtn);
    patternTypeLayout->addWidget(patternTypeCombo_, 1);
    patternTypeLayout->addWidget(patternNextBtn);
    configLayout->addLayout(patternTypeLayout);

    // Rows with +/- buttons
    QHBoxLayout* rowsLayout = new QHBoxLayout();
    QLabel* rowsLabel = new QLabel("Rows:");
    rowsLabel->setStyleSheet("font-size: 11pt; min-width: 150px;");
    rowsSpinBox_ = new QSpinBox();
    rowsSpinBox_->setRange(4, 20);
    rowsSpinBox_->setValue(7);
    rowsSpinBox_->setReadOnly(true);
    rowsSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    rowsSpinBox_->setMinimumHeight(50);
    rowsSpinBox_->setAlignment(Qt::AlignCenter);
    rowsSpinBox_->setStyleSheet("QSpinBox { font-size: 12pt; font-weight: bold; }");
    QPushButton* rowsMinusBtn = new QPushButton("-");
    QPushButton* rowsPlusBtn = new QPushButton("+");
    rowsMinusBtn->setMinimumSize(50, 50);
    rowsPlusBtn->setMinimumSize(50, 50);
    rowsMinusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #ef4444; color: white; font-weight: bold; }");
    rowsPlusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #10b981; color: white; font-weight: bold; }");
    connect(rowsMinusBtn, &QPushButton::clicked, [this]() { rowsSpinBox_->setValue(rowsSpinBox_->value() - 1); });
    connect(rowsPlusBtn, &QPushButton::clicked, [this]() { rowsSpinBox_->setValue(rowsSpinBox_->value() + 1); });
    rowsLayout->addWidget(rowsLabel);
    rowsLayout->addWidget(rowsMinusBtn);
    rowsLayout->addWidget(rowsSpinBox_, 1);
    rowsLayout->addWidget(rowsPlusBtn);
    configLayout->addLayout(rowsLayout);

    // Columns with +/- buttons
    QHBoxLayout* colsLayout = new QHBoxLayout();
    QLabel* colsLabel = new QLabel("Columns:");
    colsLabel->setStyleSheet("font-size: 11pt; min-width: 150px;");
    colsSpinBox_ = new QSpinBox();
    colsSpinBox_->setRange(4, 20);
    colsSpinBox_->setValue(10);
    colsSpinBox_->setReadOnly(true);
    colsSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    colsSpinBox_->setMinimumHeight(50);
    colsSpinBox_->setAlignment(Qt::AlignCenter);
    colsSpinBox_->setStyleSheet("QSpinBox { font-size: 12pt; font-weight: bold; }");
    QPushButton* colsMinusBtn = new QPushButton("-");
    QPushButton* colsPlusBtn = new QPushButton("+");
    colsMinusBtn->setMinimumSize(50, 50);
    colsPlusBtn->setMinimumSize(50, 50);
    colsMinusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #ef4444; color: white; font-weight: bold; }");
    colsPlusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #10b981; color: white; font-weight: bold; }");
    connect(colsMinusBtn, &QPushButton::clicked, [this]() { colsSpinBox_->setValue(colsSpinBox_->value() - 1); });
    connect(colsPlusBtn, &QPushButton::clicked, [this]() { colsSpinBox_->setValue(colsSpinBox_->value() + 1); });
    colsLayout->addWidget(colsLabel);
    colsLayout->addWidget(colsMinusBtn);
    colsLayout->addWidget(colsSpinBox_, 1);
    colsLayout->addWidget(colsPlusBtn);
    configLayout->addLayout(colsLayout);

    // Square Size with +/- buttons (step 0.5mm)
    QHBoxLayout* squareLayout = new QHBoxLayout();
    QLabel* squareLabel = new QLabel("Square Size:");
    squareLabel->setStyleSheet("font-size: 11pt; min-width: 150px;");
    squareSizeSpinBox_ = new QDoubleSpinBox();
    squareSizeSpinBox_->setRange(5.0, 100.0);
    squareSizeSpinBox_->setValue(24.0);
    squareSizeSpinBox_->setSuffix(" mm");
    squareSizeSpinBox_->setDecimals(1);
    squareSizeSpinBox_->setSingleStep(0.5);
    squareSizeSpinBox_->setReadOnly(true);
    squareSizeSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    squareSizeSpinBox_->setMinimumHeight(50);
    squareSizeSpinBox_->setAlignment(Qt::AlignCenter);
    squareSizeSpinBox_->setStyleSheet("QDoubleSpinBox { font-size: 12pt; font-weight: bold; }");
    QPushButton* squareMinusBtn = new QPushButton("-");
    QPushButton* squarePlusBtn = new QPushButton("+");
    squareMinusBtn->setMinimumSize(50, 50);
    squarePlusBtn->setMinimumSize(50, 50);
    squareMinusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #ef4444; color: white; font-weight: bold; }");
    squarePlusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #10b981; color: white; font-weight: bold; }");
    connect(squareMinusBtn, &QPushButton::clicked, [this]() { squareSizeSpinBox_->setValue(squareSizeSpinBox_->value() - 0.5); });
    connect(squarePlusBtn, &QPushButton::clicked, [this]() { squareSizeSpinBox_->setValue(squareSizeSpinBox_->value() + 0.5); });
    squareLayout->addWidget(squareLabel);
    squareLayout->addWidget(squareMinusBtn);
    squareLayout->addWidget(squareSizeSpinBox_, 1);
    squareLayout->addWidget(squarePlusBtn);
    configLayout->addLayout(squareLayout);

    // ArUco Marker Size with +/- buttons (step 0.5mm)
    QHBoxLayout* arucoLayout = new QHBoxLayout();
    QLabel* arucoLabel = new QLabel("ArUco Size:");
    arucoLabel->setStyleSheet("font-size: 11pt; min-width: 150px;");
    arucoSizeSpinBox_ = new QDoubleSpinBox();
    arucoSizeSpinBox_->setRange(5.0, 50.0);
    arucoSizeSpinBox_->setValue(17.0);
    arucoSizeSpinBox_->setSuffix(" mm");
    arucoSizeSpinBox_->setDecimals(1);
    arucoSizeSpinBox_->setSingleStep(0.5);
    arucoSizeSpinBox_->setReadOnly(true);
    arucoSizeSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    arucoSizeSpinBox_->setMinimumHeight(50);
    arucoSizeSpinBox_->setAlignment(Qt::AlignCenter);
    arucoSizeSpinBox_->setStyleSheet("QDoubleSpinBox { font-size: 12pt; font-weight: bold; }");
    QPushButton* arucoMinusBtn = new QPushButton("-");
    QPushButton* arucoPlusBtn = new QPushButton("+");
    arucoMinusBtn->setMinimumSize(50, 50);
    arucoPlusBtn->setMinimumSize(50, 50);
    arucoMinusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #ef4444; color: white; font-weight: bold; }");
    arucoPlusBtn->setStyleSheet("QPushButton { font-size: 20pt; background-color: #10b981; color: white; font-weight: bold; }");
    connect(arucoMinusBtn, &QPushButton::clicked, [this]() { arucoSizeSpinBox_->setValue(arucoSizeSpinBox_->value() - 0.5); });
    connect(arucoPlusBtn, &QPushButton::clicked, [this]() { arucoSizeSpinBox_->setValue(arucoSizeSpinBox_->value() + 0.5); });
    arucoLayout->addWidget(arucoLabel);
    arucoLayout->addWidget(arucoMinusBtn);
    arucoLayout->addWidget(arucoSizeSpinBox_, 1);
    arucoLayout->addWidget(arucoPlusBtn);
    configLayout->addLayout(arucoLayout);

    configGroup->setLayout(configLayout);
    mainLayout->addWidget(configGroup);

    // CAPTURE CONTROLS
    QHBoxLayout* controlsLayout = new QHBoxLayout();

    startCaptureButton_ = new QPushButton("Start Dataset Capture (50 pairs)");
    startCaptureButton_->setStyleSheet(
        "QPushButton { background-color: #059669; color: white; font-size: 14pt; "
        "padding: 12px; border-radius: 8px; font-weight: bold; } "
        "QPushButton:hover { background-color: #047857; } "
        "QPushButton:pressed { background-color: #065f46; }");
    connect(startCaptureButton_, &QPushButton::clicked, this, &DatasetCaptureWidget::onStartCapture);

    controlsLayout->addWidget(startCaptureButton_);
    mainLayout->addLayout(controlsLayout);

    // STATUS
    QHBoxLayout* statusLayout = new QHBoxLayout();

    statusLabel_ = new QLabel("Ready to capture - Configure pattern and click Start");
    statusLabel_->setStyleSheet("font-size: 12pt; color: #0e0e0e;");

    detectionStatusLabel_ = new QLabel("Move checkerboard into view");
    detectionStatusLabel_->setStyleSheet("font-size: 12pt; color: #FFA500; font-weight: bold;");

    statusLayout->addWidget(statusLabel_);
    statusLayout->addStretch();
    statusLayout->addWidget(detectionStatusLabel_);
    mainLayout->addLayout(statusLayout);

    // PROGRESS BAR
    captureProgress_ = new QProgressBar();
    captureProgress_->setRange(0, 50);
    captureProgress_->setValue(0);
    captureProgress_->setTextVisible(true);
    captureProgress_->setFormat("Captured: %v / %m pairs");
    captureProgress_->setStyleSheet(
        "QProgressBar { border: 2px solid #bebebe; border-radius: 5px; text-align: center; "
        "background-color: #f0f0f0; color: #0e0e0e; font-weight: bold; } "
        "QProgressBar::chunk { background-color: #059669; }");
    mainLayout->addWidget(captureProgress_);
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

    // Frame skipping for performance (process every 3rd frame like camera_preview_widget)
    static int frame_skip_counter = 0;
    if (++frame_skip_counter % 3 != 0) {
        return;
    }

    // Extract frames (already in BGRA format from core::CameraFrame)
    cv::Mat leftRGB, rightRGB;
    cv::cvtColor(frame_pair.left_frame.image, leftRGB, cv::COLOR_BGRA2RGB);
    cv::cvtColor(frame_pair.right_frame.image, rightRGB, cv::COLOR_BGRA2RGB);

    // Downsample to HD 1280x720
    cv::Mat leftHD, rightHD;
    cv::resize(leftRGB, leftHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
    cv::resize(rightRGB, rightHD, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);

    // Detect pattern and draw overlay (thread-safe)
    std::vector<cv::Point2f> cornersLeft, cornersRight;
    cv::Mat overlayLeft, overlayRight;

    bool leftDetected, rightDetected;
    {
        std::lock_guard<std::mutex> lock(patternDetectorMutex_);
        if (patternDetector_) {
            leftDetected = patternDetector_->detect(leftHD, cornersLeft, overlayLeft);
            rightDetected = patternDetector_->detect(rightHD, cornersRight, overlayRight);
        } else {
            leftDetected = false;
            rightDetected = false;
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
        isCapturing_ = false;
        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        startCaptureButton_->setStyleSheet(
            "QPushButton { background-color: #059669; color: white; font-size: 14pt; "
            "padding: 12px; border-radius: 8px; font-weight: bold; }");
        statusLabel_->setText("Capture stopped");

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

    // Pattern config
    datasetInfo_["pattern_config"]["type"] = patternTypeCombo_->currentText().toStdString();
    datasetInfo_["pattern_config"]["rows"] = rowsSpinBox_->value();
    datasetInfo_["pattern_config"]["cols"] = colsSpinBox_->value();
    datasetInfo_["pattern_config"]["square_size_mm"] = squareSizeSpinBox_->value();
    datasetInfo_["pattern_config"]["aruco_marker_size_mm"] = arucoSizeSpinBox_->value();

    // Capture config
    datasetInfo_["capture_config"]["image_width"] = 1280;
    datasetInfo_["capture_config"]["image_height"] = 720;
    datasetInfo_["capture_config"]["capture_delay_seconds"] = 5;
    datasetInfo_["capture_config"]["target_image_pairs"] = 50;
    datasetInfo_["capture_config"]["vcsel_enabled"] = true;
    datasetInfo_["capture_config"]["vcsel_current_ma"] = 280;

    // Initialize and enable VCSEL LED (LED1 only, >= 250mA)
    if (ledController_) {
        // Initialize if not already done (same as camera_preview_widget)
        if (!ledController_->isInitialized()) {
            qDebug() << "[DatasetCapture] Initializing AS1170 controller";

            // CRITICAL: Force reset hardware BEFORE initialization
            qDebug() << "[DatasetCapture] Forcing AS1170 hardware reset to clear stuck state";
            ledController_->forceResetHardware();

            if (!ledController_->initialize()) {
                qWarning() << "[DatasetCapture] Failed to initialize AS1170 controller - LEDs will not work";
                statusLabel_->setText("WARNING: LED controller initialization failed");
            } else {
                qDebug() << "[DatasetCapture] AS1170 controller initialized successfully";
            }
        }

        // Enable LED1 (VCSEL) at 280mA (>= 250mA minimum requirement)
        bool led1_success = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 280);

        // Ensure LED2 is OFF (user requirement: LED1 ON, LED2 OFF)
        bool led2_success = ledController_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

        if (led1_success) {
            qDebug() << "[DatasetCapture] LED1 (VCSEL) activated at 280mA";
        } else {
            qWarning() << "[DatasetCapture] Failed to activate LED1 (VCSEL)";
        }

        if (led2_success) {
            qDebug() << "[DatasetCapture] LED2 (Flood) disabled";
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
    statusLabel_->setText("Capturing... move checkerboard between captures");

    captureTimer_->start();

    // Capture first frame immediately
    onCaptureFrame();
}

void DatasetCaptureWidget::onCaptureFrame() {
    if (captureCount_ >= targetCaptures_) {
        // Capture complete
        captureTimer_->stop();
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

        // Save dataset info
        saveDatasetInfo();

        startCaptureButton_->setText("Start Dataset Capture (50 pairs)");
        startCaptureButton_->setStyleSheet(
            "QPushButton { background-color: #059669; color: white; font-size: 14pt; "
            "padding: 12px; border-radius: 8px; font-weight: bold; }");
        statusLabel_->setText("Dataset capture complete!");

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

    statusLabel_->setText(QString("Captured %1 / %2 pairs - move checkerboard for next capture")
                         .arg(captureCount_).arg(targetCaptures_));
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

void DatasetCaptureWidget::onPatternTypeChanged(int index) {
    updatePatternDetector();
}

void DatasetCaptureWidget::updatePatternDetector() {
    calibration::PatternConfig config;
    config.type = static_cast<calibration::PatternType>(
        patternTypeCombo_->currentData().toInt());
    config.rows = rowsSpinBox_->value();
    config.cols = colsSpinBox_->value();
    config.squareSizeMM = squareSizeSpinBox_->value();
    config.arucoMarkerSizeMM = arucoSizeSpinBox_->value();

    // Thread-safe pattern detector recreation
    std::lock_guard<std::mutex> lock(patternDetectorMutex_);
    patternDetector_ = std::make_unique<calibration::PatternDetector>(config);
}

} // namespace gui
} // namespace unlook
