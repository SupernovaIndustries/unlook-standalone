#include "unlook/gui/handheld_scan_widget.hpp"
#include "ui_handheld_scan_widget.h"
#include "unlook/hardware/AS1170Controller.hpp"
#include "unlook/api/HandheldScanPipeline.hpp"
#include "unlook/camera/CameraSystem.hpp"

#include <QtConcurrent>
#include <QDebug>
#include <QShowEvent>
#include <QHideEvent>
#include <QImage>
#include <QPixmap>
#include <QPainter>
#include <QDialog>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>

namespace unlook {
namespace gui {

HandheldScanWidget::HandheldScanWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::HandheldScanWidget)
    , camera_system_(camera_system)
    , update_timer_(nullptr)
    , scan_state_(ScanState::IDLE)
    , frames_captured_(0)
    , total_frames_(TARGET_FRAMES)
    , current_stability_(0.0f)
    , achieved_precision_mm_(0.0f)
    , point_count_(0)
    , has_calibrated_params_(false)
    , calibrated_exposure_us_(10000.0)
    , calibrated_gain_(1.0)
    , last_frame_time_(std::chrono::steady_clock::now())
    , scan_start_time_(std::chrono::steady_clock::now())
    , scan_watcher_(nullptr)
{
    qDebug() << "[HandheldScanWidget] Constructor - using shared GUI camera system + .ui file";

    // Setup UI from .ui file
    ui->setupUi(this);

    // Get LED controller singleton instance (CRITICAL: same as other widgets)
    led_controller_ = hardware::AS1170Controller::getInstance();
    if (!led_controller_) {
        qWarning() << "[HandheldScanWidget] Failed to get AS1170Controller singleton";
    }

    setupUI();

    // Setup update timer (30 Hz for smooth UI updates)
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &HandheldScanWidget::updateUI);
    update_timer_->start(1000 / UI_UPDATE_HZ);  // 33ms interval

    // Initialize FPS sample buffer
    fps_samples_.reserve(FPS_SAMPLE_COUNT);

    qDebug() << "[HandheldScanWidget] Initialized with 30 Hz UI updates and camera preview";
}

HandheldScanWidget::~HandheldScanWidget() {
    if (update_timer_) {
        update_timer_->stop();
    }

    if (scan_watcher_) {
        scan_watcher_->cancel();
        scan_watcher_->waitForFinished();
    }

    // DISABLED: Camera preview disabled to avoid monopolizing cameras
    // stopCameraPreview();

    // CRITICAL: Disable all LEDs before destroying widget (prevent stuck LED state)
    if (led_controller_) {
        qDebug() << "[HandheldScanWidget] Disabling all LEDs in destructor";
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
    }

    // NOTE: We do NOT shutdown the API camera system here or re-initialize GUI system
    // The camera systems are singletons that persist across widget lifecycles
    // Other widgets will re-initialize GUI system when they need it

    delete ui;
    qDebug() << "[HandheldScanWidget] Destroyed";
}

void HandheldScanWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);

    qDebug() << "[HandheldScanWidget::showEvent] Widget now visible";

    // CRITICAL: Initialize LED controller when widget becomes visible
    // This ensures AS1170 is ready for both CALIBRATE and START buttons
    if (led_controller_) {
        // Initialize if not already done
        if (!led_controller_->isInitialized()) {
            qDebug() << "[HandheldScanWidget] Initializing AS1170 controller";

            // CRITICAL: Force reset hardware BEFORE initialization
            qDebug() << "[HandheldScanWidget] Forcing AS1170 hardware reset to clear stuck state";
            led_controller_->forceResetHardware();

            if (!led_controller_->initialize()) {
                qWarning() << "[HandheldScanWidget] Failed to initialize AS1170 controller - LEDs will not work";
            } else {
                qDebug() << "[HandheldScanWidget] AS1170 controller initialized successfully";
            }
        } else {
            // Already initialized, but force hardware reset anyway for safety
            qDebug() << "[HandheldScanWidget] AS1170 already initialized, forcing hardware reset";
            led_controller_->forceResetHardware();
        }
    }

    // Start camera preview (now working with HardwareSyncCapture!)
    startCameraPreview();
}

void HandheldScanWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    qDebug() << "[HandheldScanWidget::hideEvent] Widget hidden";

    // Stop camera preview when widget hidden
    stopCameraPreview();

    // CRITICAL: Disable all LEDs when hiding widget (user switched tab)
    if (led_controller_) {
        qDebug() << "[HandheldScanWidget::hideEvent] Disabling all LEDs on hide";
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
    }

    // Camera system is shared with GUI, no shutdown needed
}

void HandheldScanWidget::setupUI() {
    // UI is loaded from .ui file via ui->setupUi(this) in constructor
    // Here we only connect signals and setup initial state

    // Connect button signals
    connect(ui->scanButton, &QPushButton::clicked, this, &HandheldScanWidget::onStartScan);
    connect(ui->stopButton, &QPushButton::clicked, this, &HandheldScanWidget::onStopScan);
    connect(ui->calibrateButton, &QPushButton::clicked, this, &HandheldScanWidget::onAutoCalibrate);
    connect(ui->infoButton, &QPushButton::clicked, this, &HandheldScanWidget::onShowInfo);

    // All statistics hidden (shown in INFO popup)
    // Initialize progress bars (HIDDEN)
    // HIDDEN:     ui->stabilityProgressBar->setMinimum(0);
    // HIDDEN:     ui->stabilityProgressBar->setMaximum(100);
    // HIDDEN:     ui->stabilityProgressBar->setValue(0);

    // HIDDEN:     ui->captureProgressBar->setMinimum(0);
    // HIDDEN:     ui->captureProgressBar->setMaximum(TARGET_FRAMES);
    // HIDDEN:     ui->captureProgressBar->setValue(0);

    // Set initial button visibility (IDLE state: Start + Calibrate visible, Stop hidden)
    ui->scanButton->setVisible(true);
    ui->calibrateButton->setVisible(true);
    ui->stopButton->setVisible(false);
    // HIDDEN:     ui->captureProgressBar->setVisible(true);  // Keep visible but at 0

    qDebug() << "[HandheldScanWidget::setupUI] UI initialized from .ui file with camera preview";
}

void HandheldScanWidget::onStartScan() {
    qDebug() << "[HandheldScanWidget] Starting handheld scan...";

    // Check camera system is ready
    if (!camera_system_ || !camera_system_->isReady()) {
        qCritical() << "[HandheldScanWidget] Camera system not ready!";
    // HIDDEN:         ui->statusLabel->setText("ERROR: Camera system not ready");
    // HIDDEN:         ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
        return;  // Abort scan
    }

    qDebug() << "[HandheldScanWidget] Camera system ready, initializing LED controller...";

    // CRITICAL: Enable VCSEL for AD-CENSUS stereo matching
    // AS1170 already initialized in showEvent(), just enable LED here
    if (led_controller_) {
        // CRITICAL: ENABLE VCSEL ONLY for scanning
        // LED2 (Flood): DISABLED - too much current, causes Raspberry Pi shutdown
        // LED1 (VCSEL): 340mA - safe power for VCSEL dots without overloading power supply

        qDebug() << "[HandheldScanWidget] LED2 (Flood) DISABLED - too much current";

        // ENABLE VCSEL at 340mA (safe limit to prevent Raspberry Pi shutdown)
        bool led1_success = led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 340);
        if (led1_success) {
            qDebug() << "[HandheldScanWidget] LED1 (VCSEL) ENABLED at 340mA (safe power)";
        } else {
            qCritical() << "[HandheldScanWidget] CRITICAL: Failed to enable LED1 (VCSEL) - scan may fail!";
    // HIDDEN:             ui->statusLabel->setText("ERROR: Failed to enable VCSEL");
    // HIDDEN:             ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
            return;  // Abort scan if VCSEL fails
        }
    } else {
        qWarning() << "[HandheldScanWidget] LED controller not available - proceeding without LED control";
    }

    // CRITICAL: Wait 200ms for LEDs to fully stabilize before capturing
    // The LEDs need time to reach full brightness and stable output
    qDebug() << "[HandheldScanWidget] Waiting 200ms for LEDs to stabilize...";
    QThread::msleep(200);

    qDebug() << "[HandheldScanWidget] LED controller ready, starting scan...";

    // Reset state - SKIP stability wait (bugged) and go directly to CAPTURING
    scan_state_ = ScanState::CAPTURING;
    frames_captured_ = 0;
    current_stability_ = 1.0f;  // Set to 100% to skip stability check
    achieved_precision_mm_ = 0.0f;
    point_count_ = 0;
    scan_start_time_ = std::chrono::steady_clock::now();
    fps_samples_.clear();

    // Update UI: SCANNING state (hide Start/Calibrate, show Stop)
    ui->scanButton->setVisible(false);
    ui->calibrateButton->setVisible(false);
    ui->stopButton->setVisible(true);
    // HIDDEN:     ui->captureProgressBar->setVisible(true);
    // removed capture_count_label(true);
    // HIDDEN:     ui->captureProgressBar->setValue(0);
    // ui->statusLabel->setText("0/" + QString::number(TARGET_FRAMES) + " frames");

    // HIDDEN:     ui->statusLabel->setText("Waiting for stability...");
    // HIDDEN:     ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FFA500;");

    // Start background scan thread
    startScanThread();
}

void HandheldScanWidget::onStopScan() {
    qDebug() << "[HandheldScanWidget] Stopping scan...";

    // Cancel ongoing scan
    if (scan_watcher_) {
        scan_watcher_->cancel();
    }

    // CRITICAL: Disable all LEDs when stopping scan
    if (led_controller_) {
        qDebug() << "[HandheldScanWidget] Disabling all LEDs after scan stop";
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
    }

    // Reset UI
    resetUI();

    // HIDDEN:     ui->statusLabel->setText("Scan cancelled");
    // HIDDEN:     ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
}

void HandheldScanWidget::onShowInfo() {
    qDebug() << "[HandheldScanWidget] Showing info popup";

    // Create modal dialog for statistics
    QDialog* infoDialog = new QDialog(this);
    infoDialog->setWindowTitle("Scan Information");
    infoDialog->setModal(true);
    infoDialog->setMinimumSize(400, 350);
    infoDialog->setStyleSheet(
        "QDialog {"
        "  background-color: #1a1a1a;"
        "  color: #FFFFFF;"
        "}"
        "QLabel {"
        "  padding: 8px;"
        "  font-size: 14pt;"
        "}"
    );

    // Create layout
    QVBoxLayout* layout = new QVBoxLayout(infoDialog);
    layout->setSpacing(15);
    layout->setContentsMargins(20, 20, 20, 20);

    // Title
    QLabel* titleLabel = new QLabel("SCAN STATISTICS", infoDialog);
    titleLabel->setStyleSheet("font-size: 20pt; font-weight: bold; color: #00D9FF;");
    titleLabel->setAlignment(Qt::AlignCenter);
    layout->addWidget(titleLabel);

    // Separator
    QFrame* separator = new QFrame(infoDialog);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Sunken);
    separator->setStyleSheet("background-color: #333333;");
    layout->addWidget(separator);

    // Stability
    QLabel* stabilityLabel = new QLabel(
        QString("Stability: %1%").arg(qRound(current_stability_ * 100)), infoDialog);
    stabilityLabel->setStyleSheet("color: #FFA500;");
    layout->addWidget(stabilityLabel);

    // Capture progress
    QLabel* captureLabel = new QLabel(
        QString("Frames Captured: %1/%2").arg(frames_captured_).arg(total_frames_), infoDialog);
    captureLabel->setStyleSheet("color: #00D9FF;");
    layout->addWidget(captureLabel);

    // FPS
    float fps = calculateFPS();
    QLabel* fpsLabel = new QLabel(QString("FPS: %1").arg(fps, 0, 'f', 1), infoDialog);
    fpsLabel->setStyleSheet("color: #00FF88;");
    layout->addWidget(fpsLabel);

    // Precision
    QLabel* precisionLabel = new QLabel(
        QString("Target Precision: 0.10 mm"), infoDialog);
    if (achieved_precision_mm_ > 0) {
        precisionLabel->setText(QString("Achieved Precision: %1 mm")
            .arg(achieved_precision_mm_, 0, 'f', 3));
    }
    precisionLabel->setStyleSheet("color: #FFD700;");
    layout->addWidget(precisionLabel);

    // Point count
    if (point_count_ > 0) {
        QLabel* pointsLabel = new QLabel(
            QString("Point Cloud: %1 points").arg(point_count_), infoDialog);
        pointsLabel->setStyleSheet("color: #FF88FF;");
        layout->addWidget(pointsLabel);
    }

    // Scan state
    QString stateText;
    switch (scan_state_) {
        case ScanState::IDLE:
            stateText = "State: Ready";
            break;
        case ScanState::WAITING_STABILITY:
            stateText = "State: Waiting for stability...";
            break;
        case ScanState::CAPTURING:
            stateText = "State: Capturing frames...";
            break;
        case ScanState::PROCESSING:
            stateText = "State: Processing depth maps...";
            break;
        case ScanState::COMPLETED:
            stateText = "State: Scan completed!";
            break;
        case ScanState::FAILED:
            stateText = "State: Scan failed";
            break;
    }
    QLabel* stateLabel = new QLabel(stateText, infoDialog);
    stateLabel->setStyleSheet("color: #AAAAAA; font-size: 12pt;");
    layout->addWidget(stateLabel);

    // Add stretch to push everything to top
    layout->addStretch();

    // Close button
    QPushButton* closeButton = new QPushButton("CLOSE", infoDialog);
    closeButton->setMinimumHeight(50);
    closeButton->setStyleSheet(
        "QPushButton {"
        "  font-size: 16pt;"
        "  font-weight: bold;"
        "  background-color: #0066AA;"
        "  border: 2px solid #0099FF;"
        "  border-radius: 8px;"
        "  padding: 10px;"
        "}"
        "QPushButton:hover {"
        "  background-color: #0088CC;"
        "}"
        "QPushButton:pressed {"
        "  background-color: #004488;"
        "}"
    );
    connect(closeButton, &QPushButton::clicked, infoDialog, &QDialog::accept);
    layout->addWidget(closeButton);

    // Show dialog (modal, does not interrupt scan)
    infoDialog->exec();
    delete infoDialog;
}

void HandheldScanWidget::updateUI() {
    if (scan_state_ == ScanState::IDLE) {
        // Not scanning, show static values
        return;
    }

    // STABILITY CHECK DISABLED - bugged variable, skip directly to capture
    if (scan_state_ == ScanState::CAPTURING) {
        // Simulate frame capture progress (would come from actual capture pipeline)
        static int frame_counter = 0;
        frame_counter++;

        if (frame_counter % 5 == 0 && frames_captured_ < TARGET_FRAMES) {
            frames_captured_++;
            updateCaptureProgress(frames_captured_, TARGET_FRAMES);

            // Update FPS
            auto now = std::chrono::steady_clock::now();
            auto duration = now - last_frame_time_;
            float fps = 1000.0f / std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
            last_frame_time_ = now;

            // Add to rolling average
            fps_samples_.push_back(fps);
            if (fps_samples_.size() > FPS_SAMPLE_COUNT) {
                fps_samples_.erase(fps_samples_.begin());
            }

            float avg_fps = calculateFPS();
    // HIDDEN:             ui->fpsLabel->setText("FPS: " + QString::number(avg_fps, 'f', 1));

            qDebug() << "[HandheldScanWidget] Frame" << frames_captured_ << "captured, FPS:" << avg_fps;

            if (frames_captured_ >= TARGET_FRAMES) {
                scan_state_ = ScanState::PROCESSING;
    // HIDDEN:                 ui->statusLabel->setText("Processing depth maps...");
    // HIDDEN:                 ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00E5CC;");
                qDebug() << "[HandheldScanWidget] All frames captured, processing...";
            }
        }
    } else if (scan_state_ == ScanState::PROCESSING) {
        // Processing happens in background thread
        // UI shows animated status (handled by background thread completion)
        static int dots = 0;
        dots = (dots + 1) % 4;
        QString dot_string = QString(".").repeated(dots);
    // HIDDEN:         ui->statusLabel->setText("Processing" + dot_string);
    }
}

void HandheldScanWidget::updateStabilityIndicator(float score) {
    // All UI updates hidden - data stored internally for INFO popup
    current_stability_ = score;

    // HIDDEN: Update progress bar color and text
    /*
    int percentage = static_cast<int>(score * 100.0f);
    ui->stabilityProgressBar->setValue(percentage);

    QColor color = getStabilityColor(score);
    QString color_str = color.name();

    ui->stabilityProgressBar->setStyleSheet(
        "QProgressBar {"
        "    border: 2px solid #555555;"
        "    border-radius: 5px;"
        "    text-align: center;"
        "    font-size: 14pt;"
        "    font-weight: bold;"
        "    background-color: #1A1A1A;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: " + color_str + ";"
        "    border-radius: 3px;"
        "}"
    );

    QString text = getStabilityText(score);
    ui->stabilityTextLabel->setText(text);
    ui->stabilityTextLabel->setStyleSheet("font-size: 18pt; font-weight: bold; color: " + color_str + ";");
    */
}

void HandheldScanWidget::updateCaptureProgress(int current, int total) {
    // All UI updates hidden - data stored internally for INFO popup
    frames_captured_ = current;
    total_frames_ = total;

    // HIDDEN: Update progress bar
    /*
    ui->captureProgressBar->setValue(current);

    float percentage = static_cast<float>(current) / static_cast<float>(total);
    QColor color;
    if (percentage < 0.5f) {
        color = QColor("#FFA500");
    } else if (percentage < 1.0f) {
        color = QColor("#00E5CC");
    } else {
        color = QColor("#00FF00");
    }

    ui->captureProgressBar->setStyleSheet(
        "QProgressBar {"
        "    border: 2px solid #555555;"
        "    border-radius: 5px;"
        "    text-align: center;"
        "    font-size: 12pt;"
        "    background-color: #1A1A1A;"
        "}"
        "QProgressBar::chunk {"
        "    background-color: " + color.name() + ";"
        "    border-radius: 3px;"
        "}"
    );
    */
}

float HandheldScanWidget::calculateFPS() {
    if (fps_samples_.empty()) {
        return 0.0f;
    }

    // Calculate average FPS from samples
    float sum = std::accumulate(fps_samples_.begin(), fps_samples_.end(), 0.0f);
    return sum / static_cast<float>(fps_samples_.size());
}

QColor HandheldScanWidget::getStabilityColor(float score) const {
    if (score < 0.70f) {
        return QColor("#FF4444");  // Red
    } else if (score < 0.90f) {
        return QColor("#FFA500");  // Orange
    } else {
        return QColor("#00FF00");  // Green
    }
}

QString HandheldScanWidget::getStabilityText(float score) const {
    if (score < 0.70f) {
        return "Hold steady...";
    } else if (score < 0.90f) {
        return "Almost there...";
    } else {
        return "Stable!";
    }
}

void HandheldScanWidget::resetUI() {
    scan_state_ = ScanState::IDLE;
    // IDLE state: show Start + Calibrate, hide Stop
    ui->scanButton->setVisible(true);
    ui->calibrateButton->setVisible(true);
    ui->stopButton->setVisible(false);
    // HIDDEN:     ui->captureProgressBar->setVisible(false);
    // removed capture_count_label(false);

    // HIDDEN:     ui->stabilityProgressBar->setValue(0);
    // HIDDEN:     ui->stabilityTextLabel->setText("Hold steady...");
    // HIDDEN:     ui->fpsLabel->setText("FPS: --");
    // HIDDEN:     ui->precisionLabel->setText("Precision: 0.10mm target");

    // HIDDEN:     ui->statusLabel->setText("Ready to scan");
    // HIDDEN:     ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00E5CC;");
}

void HandheldScanWidget::startScanThread() {
    // Create background scan thread using QtConcurrent
    QFuture<bool> future = QtConcurrent::run([this]() -> bool {
        qDebug() << "[HandheldScanWidget::ScanThread] Starting frame capture with continuous callback...";

        try {
            // CRITICAL: Stop any existing capture first (from preview or other widgets)
            qDebug() << "[HandheldScanWidget::ScanThread] Stopping any existing capture...";
            camera_system_->stopCapture();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief delay for cleanup

            // ========== PER-FRAME AUTO-CALIBRATION ==========
            // For each frame, analyze scene and optimize exposure/gain for that specific object position
            qDebug() << "[HandheldScanWidget::ScanThread] Starting PER-FRAME calibrated capture for" << TARGET_FRAMES << "frames";

            std::vector<core::StereoFramePair> captured_frames;
            captured_frames.reserve(TARGET_FRAMES);

            // CRITICAL: Use saved calibrated parameters from performAutoCalibration()
            double current_exposure = calibrated_exposure_us_;
            double current_gain = calibrated_gain_;

            qDebug() << "[HandheldScanWidget::ScanThread] Initial calibrated params: exposure=" << current_exposure << "us, gain=" << current_gain << "x";

            // CRITICAL: Disable auto-exposure and auto-gain FIRST
            camera_system_->setAutoExposure(core::CameraId::LEFT, false);
            camera_system_->setAutoExposure(core::CameraId::RIGHT, false);
            camera_system_->setAutoGain(core::CameraId::LEFT, false);
            camera_system_->setAutoGain(core::CameraId::RIGHT, false);

            // Apply calibrated parameters to BOTH cameras before starting per-frame loop
            qDebug() << "[HandheldScanWidget::ScanThread] Applying calibrated params to BOTH cameras...";
            camera_system_->setExposureTime(core::CameraId::LEFT, current_exposure);
            camera_system_->setExposureTime(core::CameraId::RIGHT, current_exposure);
            camera_system_->setGain(core::CameraId::LEFT, current_gain);
            camera_system_->setGain(core::CameraId::RIGHT, current_gain);

            // Per-frame capture loop with scene-specific optimization
            for (int frame_idx = 0; frame_idx < TARGET_FRAMES; frame_idx++) {
                qDebug() << "[HandheldScanWidget::ScanThread] ========== FRAME" << (frame_idx + 1) << "/" << TARGET_FRAMES << "==========";

                // Check for cancellation
                if (QThread::currentThread()->isInterruptionRequested()) {
                    qDebug() << "[HandheldScanWidget::ScanThread] Scan cancelled by user";
                    camera_system_->stopCapture();
                    return false;
                }

                // CRITICAL FIX: Reapply parameters before EACH frame to prevent drift/override
                qDebug() << "[HandheldScanWidget::ScanThread] Reapplying params before frame capture: exp=" << current_exposure << "us, gain=" << current_gain << "x";
                camera_system_->setExposureTime(core::CameraId::LEFT, current_exposure);
                camera_system_->setExposureTime(core::CameraId::RIGHT, current_exposure);
                camera_system_->setGain(core::CameraId::LEFT, current_gain);
                camera_system_->setGain(core::CameraId::RIGHT, current_gain);
                std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Stabilization DOPO riapply

                // STEP 1: Capture test frame for histogram analysis
                qDebug() << "[HandheldScanWidget::ScanThread] Capturing test frame for scene analysis...";

                std::mutex test_mutex;
                std::condition_variable test_cv;
                core::StereoFramePair test_frame;
                int frames_received = 0;
                const int FRAMES_TO_SKIP = 3; // Skip first 3 frames (old parameters)

                auto test_callback = [&](const core::StereoFramePair& frame) {
                    std::lock_guard<std::mutex> lock(test_mutex);
                    frames_received++;

                    // CRITICAL: Skip first 3 frames - they have OLD parameters!
                    // New parameters need 2-3 frames to take effect
                    if (frames_received <= FRAMES_TO_SKIP) {
                        qDebug() << "[HandheldScanWidget::ScanThread] Skipping frame" << frames_received << "(old parameters)";
                        return;
                    }

                    // Take the 4th frame - this has NEW parameters applied
                    if (frames_received == FRAMES_TO_SKIP + 1) {
                        qDebug() << "[HandheldScanWidget::ScanThread] Taking frame" << frames_received << "(new parameters)";
                        test_frame = frame;
                        test_cv.notify_one();
                    }
                };

                if (!camera_system_->startCapture(test_callback)) {
                    qCritical() << "[HandheldScanWidget::ScanThread] Failed to start test capture";
                    return false;
                }

                // Wait for stabilized frame (4th frame)
                {
                    std::unique_lock<std::mutex> lock(test_mutex);
                    test_cv.wait_for(lock, std::chrono::milliseconds(1000), [&]() { return frames_received > FRAMES_TO_SKIP; });
                }
                camera_system_->stopCapture();

                if (frames_received <= FRAMES_TO_SKIP || test_frame.left_frame.image.empty()) {
                    qWarning() << "[HandheldScanWidget::ScanThread] Failed to capture stabilized test frame (received" << frames_received << "frames)";
                    continue;
                }

                // STEP 2: Analyze histogram of BOTH cameras
                // CRITICAL: Check if already grayscale (YUV420 frames are 1-channel)
                cv::Mat gray_left, gray_right;

                if (test_frame.left_frame.image.channels() == 1) {
                    gray_left = test_frame.left_frame.image;
                } else {
                    cv::cvtColor(test_frame.left_frame.image, gray_left, cv::COLOR_BGR2GRAY);
                }

                if (test_frame.right_frame.image.channels() == 1) {
                    gray_right = test_frame.right_frame.image;
                } else {
                    cv::cvtColor(test_frame.right_frame.image, gray_right, cv::COLOR_BGR2GRAY);
                }

                double mean_left = cv::mean(gray_left)[0];
                double mean_right = cv::mean(gray_right)[0];
                double mean_combined = (mean_left + mean_right) / 2.0;

                qDebug() << "[HandheldScanWidget::ScanThread] Scene brightness: LEFT=" << mean_left
                         << ", RIGHT=" << mean_right << ", COMBINED=" << mean_combined;

                // STEP 3: Calculate optimal exposure for THIS scene
                const double TARGET_BRIGHTNESS = 60.0;
                const double MIN_BRIGHTNESS = 50.0;
                const double MAX_BRIGHTNESS = 70.0;

                double new_exposure = current_exposure;
                bool needs_adjustment = false;

                if (mean_combined < MIN_BRIGHTNESS) {
                    // Scene too dark - increase exposure
                    double factor = TARGET_BRIGHTNESS / std::max(mean_combined, 20.0);
                    factor = std::min(factor, 1.5);  // Max 50% increase per step
                    new_exposure = current_exposure * factor;
                    needs_adjustment = true;
                    qDebug() << "[HandheldScanWidget::ScanThread] Scene too dark, increasing exposure by" << (factor * 100 - 100) << "%";
                } else if (mean_combined > MAX_BRIGHTNESS) {
                    // Scene too bright - decrease exposure
                    double factor = TARGET_BRIGHTNESS / mean_combined;
                    factor = std::max(factor, 0.7);  // Max 30% decrease per step
                    new_exposure = current_exposure * factor;
                    needs_adjustment = true;
                    qDebug() << "[HandheldScanWidget::ScanThread] Scene too bright, decreasing exposure by" << (100 - factor * 100) << "%";
                } else {
                    qDebug() << "[HandheldScanWidget::ScanThread] Scene brightness optimal, no adjustment needed";
                }

                // STEP 4: Apply optimized parameters if needed
                if (needs_adjustment && std::abs(new_exposure - current_exposure) > 500) {
                    qDebug() << "[HandheldScanWidget::ScanThread] Applying new exposure:" << current_exposure << "→" << new_exposure << "us";

                    camera_system_->setExposureTime(core::CameraId::LEFT, new_exposure);
                    camera_system_->setExposureTime(core::CameraId::RIGHT, new_exposure);
                    current_exposure = new_exposure;

                    // Wait for stabilization
                    qDebug() << "[HandheldScanWidget::ScanThread] Stabilizing for 250ms...";
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }

                // STEP 5: Capture final optimized frame
                qDebug() << "[HandheldScanWidget::ScanThread] Capturing final optimized frame...";

                std::mutex final_mutex;
                std::condition_variable final_cv;
                core::StereoFramePair final_frame;
                int final_frames_received = 0;

                auto final_callback = [&](const core::StereoFramePair& frame) {
                    std::lock_guard<std::mutex> lock(final_mutex);
                    final_frames_received++;

                    // CRITICAL: Skip first 3 frames (old parameters)
                    if (final_frames_received <= FRAMES_TO_SKIP) {
                        qDebug() << "[HandheldScanWidget::ScanThread] Skipping final frame" << final_frames_received << "(old parameters)";
                        return;
                    }

                    // Take the 4th frame (new parameters)
                    if (final_frames_received == FRAMES_TO_SKIP + 1) {
                        qDebug() << "[HandheldScanWidget::ScanThread] Taking final frame" << final_frames_received << "(new parameters)";
                        final_frame = frame;
                        final_cv.notify_one();
                    }
                };

                if (!camera_system_->startCapture(final_callback)) {
                    qCritical() << "[HandheldScanWidget::ScanThread] Failed to start final capture";
                    return false;
                }

                // Wait for stabilized final frame (4th frame)
                {
                    std::unique_lock<std::mutex> lock(final_mutex);
                    final_cv.wait_for(lock, std::chrono::milliseconds(1000), [&]() { return final_frames_received > FRAMES_TO_SKIP; });
                }
                camera_system_->stopCapture();

                if (final_frames_received <= FRAMES_TO_SKIP || final_frame.left_frame.image.empty()) {
                    qWarning() << "[HandheldScanWidget::ScanThread] Failed to capture stabilized final frame (received" << final_frames_received << "frames)";
                    continue;
                }

                // Add to captured frames
                captured_frames.push_back(final_frame);
                frames_captured_ = captured_frames.size();

                qDebug() << "[HandheldScanWidget::ScanThread] Frame" << (frame_idx + 1) << "/" << TARGET_FRAMES
                         << "captured successfully (exposure=" << current_exposure << "us)";

                // Brief delay between frames for user positioning
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            qDebug() << "[HandheldScanWidget::ScanThread] Per-frame capture complete, captured" << captured_frames.size() << "frames";

            // Check if cancelled
            if (QThread::currentThread()->isInterruptionRequested()) {
                qDebug() << "[HandheldScanWidget::ScanThread] Scan cancelled by user";
                return false;
            }

            // Check if we got enough frames
            if (captured_frames.size() < TARGET_FRAMES) {
                qCritical() << "[HandheldScanWidget::ScanThread] Insufficient frames captured:"
                           << captured_frames.size() << "/" << TARGET_FRAMES;
                return false;
            }

            // Process frames with HandheldScanPipeline
            qDebug() << "[HandheldScanWidget::ScanThread] Starting processing with AD-Census pipeline...";
            qDebug() << "  Frames captured:" << captured_frames.size();

            try {
                // CRITICAL FIX: Do NOT create camera::CameraSystem::getInstance() here!
                // It would create a SECOND HardwareSyncCapture instance conflicting with GUI camera system
                // HandheldScanPipeline doesn't need CameraSystem since frames are already captured
                auto pipeline = std::make_unique<api::HandheldScanPipeline>(nullptr);

                if (!pipeline->initialize()) {
                    qCritical() << "[HandheldScanWidget::ScanThread] Failed to initialize pipeline";
                    return false;
                }

                // CRITICAL: Save captured frames for debugging in timestamped scan folder
                QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
                QString debug_dir = QString("/home/alessandro/unlook_debug/scan_%1").arg(timestamp);
                QDir().mkpath(debug_dir);

                qDebug() << "[HandheldScanWidget] Debug output directory:" << debug_dir;

                // Enable debug output for intermediate images (rectified, disparity, depth)
                pipeline->setDebugOutput(true, debug_dir.toStdString());

                // Convert gui frames to api format and save debug images
                std::vector<api::HandheldScanPipeline::StereoFrame> api_frames;
                api_frames.reserve(captured_frames.size());

                for (size_t i = 0; i < captured_frames.size(); i++) {
                    const auto& gui_frame = captured_frames[i];

                    api::HandheldScanPipeline::StereoFrame api_frame;
                    api_frame.leftImage = gui_frame.left_frame.image.clone();
                    api_frame.rightImage = gui_frame.right_frame.image.clone();
                    api_frame.timestampUs = gui_frame.left_frame.timestamp_ns / 1000;
                    api_frame.leftVCSEL = api_frame.leftImage;   // VCSEL is same as main image
                    api_frame.rightVCSEL = api_frame.rightImage;
                    api_frame.stabilityScore = 1.0f;  // Assumed stable since frames were captured

                    // Save debug images (YUV420 converted to grayscale for visualization)
                    try {
                        cv::Mat left_gray, right_gray;
                        if (api_frame.leftImage.channels() == 1) {
                            left_gray = api_frame.leftImage;
                            right_gray = api_frame.rightImage;
                        } else {
                            cv::cvtColor(api_frame.leftImage, left_gray, cv::COLOR_YUV2GRAY_I420);
                            cv::cvtColor(api_frame.rightImage, right_gray, cv::COLOR_YUV2GRAY_I420);
                        }

                        QString left_path = QString("%1/00_raw_frame%2_left.png").arg(debug_dir).arg(i, 2, 10, QChar('0'));
                        QString right_path = QString("%1/00_raw_frame%2_right.png").arg(debug_dir).arg(i, 2, 10, QChar('0'));

                        cv::imwrite(left_path.toStdString(), left_gray);
                        cv::imwrite(right_path.toStdString(), right_gray);

                        qDebug() << "[HandheldScanWidget] Saved debug frame" << i << "to" << debug_dir;
                    } catch (const std::exception& e) {
                        qWarning() << "[HandheldScanWidget] Failed to save debug frame" << i << ":" << e.what();
                    }

                    api_frames.push_back(api_frame);
                }

                qDebug() << "[HandheldScanWidget::ScanThread] Processing" << api_frames.size() << "frames with VCSELStereoMatcher...";

                // Show progress frame on main thread
                QMetaObject::invokeMethod(this, [this]() {
                    ui->progressFrame->setVisible(true);
                    ui->progressBar->setValue(0);
                    ui->statusLabel->setText("Starting AD-CENSUS processing...");
                }, Qt::QueuedConnection);

                // Get stereo parameters
                auto stereo_params = pipeline->getStereoParams();

                // Track processing start time for ETA calculation
                auto processing_start = std::chrono::steady_clock::now();
                std::atomic<int> frames_completed{0};
                const int total_frames = api_frames.size();

                // Process frames to depth maps using AD-Census with progress callback
                auto depth_maps = pipeline->processFrames(api_frames, stereo_params,
                    [this, &processing_start, &frames_completed, total_frames](float progress, const std::string& message) {
                        // Called from background thread - use QMetaObject::invokeMethod for thread safety
                        frames_completed++;

                        // Calculate ETA based on average time per frame
                        auto elapsed = std::chrono::steady_clock::now() - processing_start;
                        auto elapsed_sec = std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();

                        int eta_sec = 0;
                        if (frames_completed > 0) {
                            double avg_sec_per_frame = static_cast<double>(elapsed_sec) / frames_completed;
                            int remaining_frames = total_frames - frames_completed;
                            eta_sec = static_cast<int>(avg_sec_per_frame * remaining_frames);
                        }

                        // Format status message with ETA
                        QString status_msg = QString::fromStdString(message);
                        if (eta_sec > 0) {
                            int eta_min = eta_sec / 60;
                            int eta_sec_remainder = eta_sec % 60;
                            status_msg += QString(" (ETA: %1:%2)")
                                .arg(eta_min)
                                .arg(eta_sec_remainder, 2, 10, QChar('0'));
                        }

                        // Update UI on main thread
                        QMetaObject::invokeMethod(this, [this, progress, status_msg]() {
                            ui->progressBar->setValue(static_cast<int>(progress * 100));
                            ui->statusLabel->setText(status_msg);
                        }, Qt::QueuedConnection);
                    }
                );

                if (depth_maps.empty()) {
                    qCritical() << "[HandheldScanWidget::ScanThread] No depth maps generated";
                    return false;
                }

                qDebug() << "[HandheldScanWidget::ScanThread] Generated" << depth_maps.size() << "depth maps, fusing...";

                // Fuse depth maps with outlier rejection
                cv::Mat fused_depth = pipeline->fuseDepthMaps(depth_maps, 2.5f);

                if (fused_depth.empty()) {
                    qCritical() << "[HandheldScanWidget::ScanThread] Depth map fusion failed";
                    return false;
                }

                qDebug() << "[HandheldScanWidget::ScanThread] Depth fusion complete, generating point cloud...";

                // Generate point cloud
                cv::Mat point_cloud = pipeline->generatePointCloud(fused_depth, api_frames[0].leftImage);

                if (point_cloud.empty()) {
                    qCritical() << "[HandheldScanWidget::ScanThread] Point cloud generation failed";
                    return false;
                }

                // Calculate achieved precision
                achieved_precision_mm_ = pipeline->calculatePrecision(depth_maps);
                point_count_ = point_cloud.rows;  // Number of points

                qDebug() << "[HandheldScanWidget::ScanThread] Scan completed successfully!";
                qDebug() << "  Point count:" << point_count_;
                qDebug() << "  Achieved precision:" << achieved_precision_mm_ << "mm";

                // CRITICAL: Save all debug output (rectified, epipolar, disparity, depth, point cloud)
                qDebug() << "[HandheldScanWidget] Saving comprehensive debug output to" << debug_dir;
                try {
                    bool debug_saved = pipeline->saveDebugOutput(
                        debug_dir.toStdString(),
                        api_frames,
                        depth_maps,
                        fused_depth,
                        point_cloud
                    );

                    if (debug_saved) {
                        qDebug() << "[HandheldScanWidget] All debug images saved successfully";
                    } else {
                        qWarning() << "[HandheldScanWidget] Failed to save some debug images";
                    }
                } catch (const std::exception& e) {
                    qWarning() << "[HandheldScanWidget] Debug save error:" << e.what();
                }

                // Shutdown pipeline
                pipeline->shutdown();

                return true;

            } catch (const std::exception& e) {
                qCritical() << "[HandheldScanWidget::ScanThread] Processing exception:" << e.what();
                return false;
            }

        } catch (const std::exception& e) {
            qCritical() << "[HandheldScanWidget::ScanThread] Scan failed with exception:" << e.what();
            camera_system_->stopCapture();  // Ensure capture is stopped
            return false;
        }
    });

    // Setup watcher for completion
    scan_watcher_ = new QFutureWatcher<bool>(this);

    connect(scan_watcher_, &QFutureWatcher<bool>::finished, this, [this]() {
        bool success = scan_watcher_->result();

        if (success) {
            onScanCompleted();
        } else {
            onScanFailed("Scan processing failed");
        }

        // Cleanup watcher
        scan_watcher_->deleteLater();
        scan_watcher_ = nullptr;
    });

    scan_watcher_->setFuture(future);
}

void HandheldScanWidget::onScanCompleted() {
    qDebug() << "[HandheldScanWidget] Scan completed successfully";

    // CRITICAL: Disable all LEDs after scan completion
    if (led_controller_) {
        qDebug() << "[HandheldScanWidget] Disabling all LEDs after scan completion";
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
    }

    scan_state_ = ScanState::COMPLETED;

    // Update UI with results
    auto scan_duration = std::chrono::steady_clock::now() - scan_start_time_;
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(scan_duration).count();

    // Hide progress frame
    ui->progressFrame->setVisible(false);

    // Update camera preview with success message
    ui->cameraPreviewLabel->setText(
        QString("Scan Complete!\n\nDuration: %1s\nPoints: %2\nPrecision: %3 mm")
            .arg(duration_ms / 1000.0, 0, 'f', 1)
            .arg(point_count_)
            .arg(achieved_precision_mm_, 0, 'f', 3)
    );
    ui->cameraPreviewLabel->setStyleSheet("font-size: 20pt; font-weight: bold; color: #00FF88; background-color: #0a0a0a;");

    // Hide stop button, show scan button
    ui->stopButton->setVisible(false);
    ui->scanButton->setVisible(true);

    // Emit completion signal
    emit scanCompleted(point_count_, achieved_precision_mm_);

    qDebug() << "[HandheldScanWidget] Emitted scanCompleted signal";
}

void HandheldScanWidget::onScanFailed(const QString& error) {
    qCritical() << "[HandheldScanWidget] Scan failed:" << error;

    // CRITICAL: Disable all LEDs after scan failure
    if (led_controller_) {
        qDebug() << "[HandheldScanWidget] Disabling all LEDs after scan failure";
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
    }

    scan_state_ = ScanState::FAILED;

    // Hide progress frame
    ui->progressFrame->setVisible(false);

    // Show error message
    ui->cameraPreviewLabel->setText("Scan Failed\n\n" + error);
    ui->cameraPreviewLabel->setStyleSheet("font-size: 20pt; font-weight: bold; color: #FF4444; background-color: #0a0a0a;");

    // Reset UI after delay
    QTimer::singleShot(3000, this, [this]() {
        resetUI();
    });

    // Emit failure signal
    emit scanFailed(error);
}

// Camera preview functions
void HandheldScanWidget::startCameraPreview() {
    if (!camera_system_) {
        qWarning() << "[HandheldScanWidget] Cannot start preview: camera system not available";
        return;
    }

    qDebug() << "[HandheldScanWidget] Starting camera preview...";

    // CRITICAL: Reapply calibrated parameters if available
    if (has_calibrated_params_) {
        qDebug() << "[HandheldScanWidget] Reapplying calibrated parameters: exposure=" << calibrated_exposure_us_ << "us, gain=" << calibrated_gain_ << "x";

        // Disable auto-exposure/auto-gain
        camera_system_->setAutoExposure(core::CameraId::LEFT, false);
        camera_system_->setAutoExposure(core::CameraId::RIGHT, false);
        camera_system_->setAutoGain(core::CameraId::LEFT, false);
        camera_system_->setAutoGain(core::CameraId::RIGHT, false);

        // Apply calibrated parameters
        camera_system_->setExposureTime(core::CameraId::LEFT, calibrated_exposure_us_);
        camera_system_->setExposureTime(core::CameraId::RIGHT, calibrated_exposure_us_);
        camera_system_->setGain(core::CameraId::LEFT, calibrated_gain_);
        camera_system_->setGain(core::CameraId::RIGHT, calibrated_gain_);

        qDebug() << "[HandheldScanWidget] Calibrated parameters reapplied successfully";
    }

    // Register callback with camera system
    bool success = camera_system_->startCapture([this](const core::StereoFramePair& frame) {
        // Use QMetaObject::invokeMethod to safely call from camera thread to GUI thread
        QMetaObject::invokeMethod(this, [this, frame]() {
            onPreviewFrame(frame);
        }, Qt::QueuedConnection);
    });

    if (success) {
        qDebug() << "[HandheldScanWidget] Camera preview started successfully";
    } else {
        qWarning() << "[HandheldScanWidget] Failed to start camera preview";
    }
}

void HandheldScanWidget::stopCameraPreview() {
    // Camera system is shared - don't stop it, just disconnect our callback
    // The callback will be replaced by the next widget that calls startCapture
    qDebug() << "[HandheldScanWidget] Camera preview stopped (callback will be replaced)";
}

void HandheldScanWidget::onPreviewFrame(const core::StereoFramePair& frame) {
    if (!frame.left_frame.valid || frame.left_frame.image.empty()) {
        return;
    }

    // Convert left frame to QImage for display
    cv::Mat rgb_frame;
    if (frame.left_frame.image.channels() == 1) {
        cv::cvtColor(frame.left_frame.image, rgb_frame, cv::COLOR_GRAY2RGB);
    } else {
        cv::cvtColor(frame.left_frame.image, rgb_frame, cv::COLOR_YUV2RGB_I420);
    }

    // Draw crosshair at center
    int center_x = rgb_frame.cols / 2;
    int center_y = rgb_frame.rows / 2;
    int crosshair_size = 40;

    // Green crosshair for better visibility
    cv::Scalar color(0, 255, 0);  // RGB green
    int thickness = 2;

    // Horizontal line
    cv::line(rgb_frame,
             cv::Point(center_x - crosshair_size, center_y),
             cv::Point(center_x + crosshair_size, center_y),
             color, thickness);

    // Vertical line
    cv::line(rgb_frame,
             cv::Point(center_x, center_y - crosshair_size),
             cv::Point(center_x, center_y + crosshair_size),
             color, thickness);

    // Center dot
    cv::circle(rgb_frame, cv::Point(center_x, center_y), 3, color, -1);

    // Convert to QPixmap and display
    QImage qimg(rgb_frame.data, rgb_frame.cols, rgb_frame.rows,
                rgb_frame.step, QImage::Format_RGB888);

    QPixmap pixmap = QPixmap::fromImage(qimg);

    // Scale to fit label while maintaining aspect ratio
    ui->cameraPreviewLabel->setPixmap(pixmap.scaled(
        ui->cameraPreviewLabel->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    ));
}

// Auto-calibration for VCSEL structured light
void HandheldScanWidget::onAutoCalibrate() {
    qDebug() << "============================================";
    qDebug() << "[HandheldScanWidget] AUTO-CALIBRATION BUTTON CLICKED!";
    qDebug() << "============================================";

    // Disable calibrate button during calibration
    ui->calibrateButton->setEnabled(false);
    ui->calibrateButton->setText("⚙ CALIBRATING...");
    qDebug() << "[HandheldScanWidget] Button disabled and text updated";

    // Show status message
    ui->cameraPreviewLabel->setText("Auto-Calibrating Camera\nfor VCSEL Dots...\n\nPlease wait...");
    ui->cameraPreviewLabel->setStyleSheet("font-size: 18pt; color: #FFAA00; background-color: #0a0a0a;");
    qDebug() << "[HandheldScanWidget] Status message displayed";

    // Force GUI update
    QApplication::processEvents();
    qDebug() << "[HandheldScanWidget] GUI update forced";

    // Perform calibration
    qDebug() << "[HandheldScanWidget] About to call performAutoCalibration()...";
    bool success = performAutoCalibration();
    qDebug() << "[HandheldScanWidget] performAutoCalibration() returned: " << success;

    // Re-enable button
    ui->calibrateButton->setEnabled(true);
    ui->calibrateButton->setText("⚙ CALIBRATE");

    if (success) {
        qDebug() << "[HandheldScanWidget] Auto-calibration completed successfully";

        // Get final parameters to display
        auto final_config = camera_system_->getCameraConfig(core::CameraId::LEFT);
        QString message = QString("Calibration Complete!\n\n"
                                 "Exposure: %1 µs\n"
                                 "Gain: %2x\n\n"
                                 "Auto-exposure/gain DISABLED\n"
                                 "Parameters locked for VCSEL scanning")
                         .arg(static_cast<int>(final_config.exposure_time_us))
                         .arg(final_config.gain, 0, 'f', 2);

        ui->cameraPreviewLabel->setText(message);
        ui->cameraPreviewLabel->setStyleSheet("font-size: 14pt; color: #00FF00; background-color: #0a0a0a;");

        // Popup removed per user request - info shown on preview label only
        // QMessageBox::information(this, "Auto-Calibration Complete", message);
    } else {
        qCritical() << "[HandheldScanWidget] Auto-calibration failed";
        ui->cameraPreviewLabel->setText("Calibration Failed!\n\nPlease check camera connection");
        ui->cameraPreviewLabel->setStyleSheet("font-size: 18pt; color: #FF4444; background-color: #0a0a0a;");
    }

    // Reset preview after 5 seconds (give user time to read results)
    QTimer::singleShot(5000, this, [this]() {
        startCameraPreview();
    });
}

bool HandheldScanWidget::performAutoCalibration() {
    using namespace unlook::core;

    qDebug() << "[AutoCalibration] Starting camera auto-calibration for VCSEL dots...";

    if (!camera_system_) {
        qCritical() << "[AutoCalibration] Camera system not available";
        return false;
    }

    if (!led_controller_) {
        qCritical() << "[AutoCalibration] LED controller not available";
        return false;
    }

    // Stop preview temporarily
    stopCameraPreview();

    // STEP 1: Enable ONLY VCSEL LED1 for calibration
    // LED2 (flood) DISABLED - too much current, causes Raspberry Pi shutdown
    // LED1 at 340mA (safe power to prevent Raspberry Pi shutdown)

    qDebug() << "[AutoCalibration] LED2 (Flood) DISABLED - too much current";

    qDebug() << "[AutoCalibration] Enabling VCSEL LED1 at 340mA (safe power)...";
    bool led_success = led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 340);
    qDebug() << "[AutoCalibration] LED1 result: " << (led_success ? "SUCCESS" : "FAILED");
    if (!led_success) {
        qCritical() << "[AutoCalibration] Failed to enable VCSEL LED1";
        // Ensure all LEDs are off before returning
        led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        return false;
    }

    // Wait for LEDs to stabilize
    QThread::msleep(200);

    // STEP 2: Iterative calibration loop
    // NOTE: VCSEL ONLY (380mA), no flood LED2
    const int MAX_ITERATIONS = 7;
    const int TARGET_MEAN_MIN = 50;   // Lower target with VCSEL only (no flood)
    const int TARGET_MEAN_MAX = 75;   // VCSEL dots should be clearly visible
    const float MAX_SATURATED_PERCENT = 2.0f;  // 2% max saturated pixels

    // Get current camera parameters from config
    auto left_config = camera_system_->getCameraConfig(CameraId::LEFT);
    double current_exposure = left_config.exposure_time_us;
    double current_gain = left_config.gain;

    qDebug() << "[AutoCalibration] Initial parameters: exposure=" << current_exposure << "us, gain=" << current_gain << "x";

    bool calibration_success = false;

    for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
        qDebug() << "[AutoCalibration] Iteration" << (iteration + 1) << "/" << MAX_ITERATIONS;
        qDebug() << "[AutoCalibration]   Current: exposure=" << current_exposure << "us, gain=" << current_gain << "x";

        // Update status label
        ui->cameraPreviewLabel->setText(QString("Calibrating... %1/%2\n\nExposure: %3 µs\nGain: %4x")
            .arg(iteration + 1).arg(MAX_ITERATIONS)
            .arg(QString::number(current_exposure, 'f', 0))
            .arg(QString::number(current_gain, 'f', 1)));
        QApplication::processEvents();

        // Apply current parameters to both cameras
        camera_system_->setExposureTime(CameraId::LEFT, current_exposure);
        camera_system_->setExposureTime(CameraId::RIGHT, current_exposure);
        camera_system_->setGain(CameraId::LEFT, current_gain);
        camera_system_->setGain(CameraId::RIGHT, current_gain);

        // Wait for settings to apply
        QThread::msleep(200);

        // Capture test frame
        StereoFramePair test_frame;
        bool capture_success = false;

        auto callback = [&test_frame, &capture_success](const StereoFramePair& frame) {
            test_frame = frame;
            capture_success = true;
        };

        camera_system_->startCapture(callback);
        QThread::msleep(100);  // Wait for one frame
        camera_system_->stopCapture();

        if (!capture_success || test_frame.left_frame.image.empty() || test_frame.right_frame.image.empty()) {
            qWarning() << "[AutoCalibration] Failed to capture test frame";
            continue;
        }

        // STEP 3: Analyze histogram of BOTH left and right frames
        // CRITICAL FIX: Optimize for both cameras, not just left!
        cv::Mat gray_left, gray_right;
        if (test_frame.left_frame.image.channels() == 3) {
            cv::cvtColor(test_frame.left_frame.image, gray_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(test_frame.right_frame.image, gray_right, cv::COLOR_BGR2GRAY);
        } else {
            gray_left = test_frame.left_frame.image;
            gray_right = test_frame.right_frame.image;
        }

        // Calculate statistics for LEFT camera
        int saturated_count_left = 0;
        long long sum_left = 0;
        int total_pixels_left = gray_left.rows * gray_left.cols;

        for (int y = 0; y < gray_left.rows; y++) {
            const uint8_t* row = gray_left.ptr<uint8_t>(y);
            for (int x = 0; x < gray_left.cols; x++) {
                uint8_t val = row[x];
                sum_left += val;
                if (val >= 250) saturated_count_left++;
            }
        }

        double mean_brightness_left = static_cast<double>(sum_left) / total_pixels_left;
        float saturated_percent_left = 100.0f * saturated_count_left / total_pixels_left;

        // Calculate statistics for RIGHT camera
        int saturated_count_right = 0;
        long long sum_right = 0;
        int total_pixels_right = gray_right.rows * gray_right.cols;

        for (int y = 0; y < gray_right.rows; y++) {
            const uint8_t* row = gray_right.ptr<uint8_t>(y);
            for (int x = 0; x < gray_right.cols; x++) {
                uint8_t val = row[x];
                sum_right += val;
                if (val >= 250) saturated_count_right++;
            }
        }

        double mean_brightness_right = static_cast<double>(sum_right) / total_pixels_right;
        float saturated_percent_right = 100.0f * saturated_count_right / total_pixels_right;

        // COMBINED metrics: average of both cameras for optimization
        double mean_brightness = (mean_brightness_left + mean_brightness_right) / 2.0;
        float saturated_percent = (saturated_percent_left + saturated_percent_right) / 2.0f;

        qDebug() << "[AutoCalibration]   LEFT:  mean=" << mean_brightness_left << ", saturated=" << saturated_percent_left << "%";
        qDebug() << "[AutoCalibration]   RIGHT: mean=" << mean_brightness_right << ", saturated=" << saturated_percent_right << "%";
        qDebug() << "[AutoCalibration]   COMBINED: mean=" << mean_brightness << ", saturated=" << saturated_percent << "%";

        // STEP 4: Check if calibration target achieved
        // CRITICAL: Both cameras must be in acceptable range, not just the average!
        bool left_in_range = (mean_brightness_left >= TARGET_MEAN_MIN - 10 &&
                              mean_brightness_left <= TARGET_MEAN_MAX + 10 &&
                              saturated_percent_left < MAX_SATURATED_PERCENT);
        bool right_in_range = (mean_brightness_right >= TARGET_MEAN_MIN - 10 &&
                               mean_brightness_right <= TARGET_MEAN_MAX + 10 &&
                               saturated_percent_right < MAX_SATURATED_PERCENT);
        bool combined_in_range = (mean_brightness >= TARGET_MEAN_MIN &&
                                  mean_brightness <= TARGET_MEAN_MAX &&
                                  saturated_percent < MAX_SATURATED_PERCENT);

        if (left_in_range && right_in_range && combined_in_range) {
            qDebug() << "[AutoCalibration] ✓ Target achieved for BOTH cameras!";
            qDebug() << "[AutoCalibration]   LEFT:  mean=" << mean_brightness_left << ", saturated=" << saturated_percent_left << "%";
            qDebug() << "[AutoCalibration]   RIGHT: mean=" << mean_brightness_right << ", saturated=" << saturated_percent_right << "%";
            qDebug() << "[AutoCalibration]   COMBINED: mean=" << mean_brightness << ", saturated=" << saturated_percent << "%";
            calibration_success = true;
            break;
        }

        // STEP 5: Adjust parameters for VCSEL structured light
        if (saturated_percent > 5.0f || mean_brightness > 80) {
            // Too bright - reduce exposure
            qDebug() << "[AutoCalibration]   → Too bright, reducing exposure by 20%";
            current_exposure *= 0.8;
            current_exposure = std::max(current_exposure, 5000.0);  // Min 5ms
        } else if (mean_brightness < 20) {
            // Very dark - increase both exposure AND gain aggressively
            qDebug() << "[AutoCalibration]   → Very dark, increasing exposure by 30% and gain";
            current_exposure *= 1.30;  // More aggressive for VCSEL
            current_exposure = std::min(current_exposure, 50000.0);  // Max 50ms
            current_gain = std::min(current_gain * 1.15, 16.0);  // Also increase gain
        } else if (mean_brightness < TARGET_MEAN_MIN) {
            // Slightly dark - fine-tune with gain
            qDebug() << "[AutoCalibration]   → Slightly dark, increasing gain";
            current_gain = std::min(current_gain * 1.2, 16.0);  // More aggressive gain
        } else if (mean_brightness > TARGET_MEAN_MAX) {
            // Slightly bright - fine-tune with gain
            qDebug() << "[AutoCalibration]   → Slightly bright, reducing gain";
            current_gain = std::max(current_gain * 0.9, 1.0);
        }
    }

    // STEP 6: Apply final parameters BEFORE disabling VCSEL and stopping capture
    // CRITICAL: Must apply while camera is still capturing!
    if (calibration_success) {
        qDebug() << "[AutoCalibration] ✓ Target achieved!";
        qDebug() << "[AutoCalibration]   Final exposure:" << current_exposure << "us";
        qDebug() << "[AutoCalibration]   Final gain:" << current_gain << "x";

        // SAVE calibrated parameters for later use
        has_calibrated_params_ = true;
        calibrated_exposure_us_ = current_exposure;
        calibrated_gain_ = current_gain;
        qDebug() << "[AutoCalibration] Parameters SAVED for automatic reapplication";

        // CRITICAL: Disable auto-exposure and auto-gain FIRST to prevent override
        qDebug() << "[AutoCalibration] Disabling auto-exposure and auto-gain...";
        camera_system_->setAutoExposure(CameraId::LEFT, false);
        camera_system_->setAutoExposure(CameraId::RIGHT, false);
        camera_system_->setAutoGain(CameraId::LEFT, false);
        camera_system_->setAutoGain(CameraId::RIGHT, false);

        // Apply final parameters while camera is STILL ACTIVE
        qDebug() << "[AutoCalibration] Applying final parameters (camera still active)...";
        bool exp_left = camera_system_->setExposureTime(CameraId::LEFT, current_exposure);
        bool exp_right = camera_system_->setExposureTime(CameraId::RIGHT, current_exposure);
        bool gain_left = camera_system_->setGain(CameraId::LEFT, current_gain);
        bool gain_right = camera_system_->setGain(CameraId::RIGHT, current_gain);

        qDebug() << "[AutoCalibration] Parameter application results:"
                 << "exp_left=" << exp_left << "exp_right=" << exp_right
                 << "gain_left=" << gain_left << "gain_right=" << gain_right;

        // Wait for parameters to settle
        QThread::msleep(200);

        // Verify parameters were applied
        auto verify_config = camera_system_->getCameraConfig(CameraId::LEFT);
        qDebug() << "[AutoCalibration] Verification - Applied exposure:" << verify_config.exposure_time_us
                 << "us, gain:" << verify_config.gain << "x";
    } else {
        qWarning() << "[AutoCalibration] ✗ Calibration did not converge after" << MAX_ITERATIONS << "iterations";
        qWarning() << "[AutoCalibration]   Best parameters: exposure=" << current_exposure << "us, gain=" << current_gain << "x";
    }

    // STEP 7: Now disable all LEDs (VCSEL and flood)
    qDebug() << "[AutoCalibration] Disabling all LEDs...";
    led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
    led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

    return calibration_success;
}

} // namespace gui
} // namespace unlook
