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
    connect(ui->infoButton, &QPushButton::clicked, this, &HandheldScanWidget::onShowInfo);

    // All statistics hidden (shown in INFO popup)
    // Initialize progress bars (HIDDEN)
    // HIDDEN:     ui->stabilityProgressBar->setMinimum(0);
    // HIDDEN:     ui->stabilityProgressBar->setMaximum(100);
    // HIDDEN:     ui->stabilityProgressBar->setValue(0);

    // HIDDEN:     ui->captureProgressBar->setMinimum(0);
    // HIDDEN:     ui->captureProgressBar->setMaximum(TARGET_FRAMES);
    // HIDDEN:     ui->captureProgressBar->setValue(0);

    // Set initial state
    ui->stopButton->setEnabled(false);
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

    // CRITICAL: Initialize and reset LED controller BEFORE starting capture
    // This prevents crashes from stuck LED states from previous widgets
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

        // CRITICAL: ENABLE VCSEL for AD-CENSUS stereo matching!
        // AD-CENSUS requires structured light pattern from VCSEL for texture on smooth surfaces
        // LED1 (VCSEL): ENABLED at 280mA for structured light projection
        // LED2 (Flood): DISABLED (not needed for structured light)

        // Disable flood LED first
        bool led2_success = led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
        if (led2_success) {
            qDebug() << "[HandheldScanWidget] LED2 (Flood) disabled";
        } else {
            qWarning() << "[HandheldScanWidget] Failed to disable LED2 (Flood)";
        }

        // ENABLE VCSEL at 280mA for structured light
        bool led1_success = led_controller_->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 280);
        if (led1_success) {
            qDebug() << "[HandheldScanWidget] LED1 (VCSEL) ENABLED at 280mA for AD-CENSUS structured light";
        } else {
            qCritical() << "[HandheldScanWidget] CRITICAL: Failed to enable LED1 (VCSEL) - AD-CENSUS will fail!";
    // HIDDEN:             ui->statusLabel->setText("ERROR: Failed to enable VCSEL");
    // HIDDEN:             ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
            return;  // Abort scan if VCSEL fails
        }
    } else {
        qWarning() << "[HandheldScanWidget] LED controller not available - proceeding without LED control";
    }

    qDebug() << "[HandheldScanWidget] LED controller ready, starting scan...";

    // Reset state
    scan_state_ = ScanState::WAITING_STABILITY;
    frames_captured_ = 0;
    current_stability_ = 0.0f;
    achieved_precision_mm_ = 0.0f;
    point_count_ = 0;
    scan_start_time_ = std::chrono::steady_clock::now();
    fps_samples_.clear();

    // Update UI
    ui->scanButton->setVisible(false);
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

    // Simulate stability calculation (replace with actual StabilityDetector)
    // For now, use a simple simulation that increases stability over time
    if (scan_state_ == ScanState::WAITING_STABILITY) {
        // Simulate increasing stability (would come from StabilityDetector)
        static float simulated_stability = 0.0f;
        simulated_stability += 0.02f;  // Increase by 2% per update
        if (simulated_stability > 1.0f) {
            simulated_stability = 1.0f;
        }
        current_stability_ = simulated_stability;

        updateStabilityIndicator(current_stability_);

        // Auto-transition to CAPTURING when stable
        if (current_stability_ >= STABILITY_THRESHOLD) {
            scan_state_ = ScanState::CAPTURING;
    // HIDDEN:             ui->statusLabel->setText("Stable! Capturing frames...");
    // HIDDEN:             ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00FF00;");
            qDebug() << "[HandheldScanWidget] Stability threshold reached, starting capture";
        }
    } else if (scan_state_ == ScanState::CAPTURING) {
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
    ui->scanButton->setVisible(true);
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
            // Atomic counter and mutex for thread-safe frame collection
            std::atomic<int> frames_received{0};
            std::mutex frames_mutex;
            std::condition_variable frames_cv;
            std::vector<core::StereoFramePair> captured_frames;
            captured_frames.reserve(TARGET_FRAMES);

            // Frame callback to collect frames
            auto frame_callback = [&](const core::StereoFramePair& frame) {
                std::lock_guard<std::mutex> lock(frames_mutex);

                if (captured_frames.size() < TARGET_FRAMES) {
                    captured_frames.push_back(frame);
                    int count = captured_frames.size();
                    frames_received = count;

                    qDebug() << "[HandheldScanWidget::ScanThread] Received frame" << count << "/" << TARGET_FRAMES;

                    // Update UI progress
                    frames_captured_ = count;

                    // Notify if we have enough frames
                    if (count >= TARGET_FRAMES) {
                        frames_cv.notify_one();
                    }
                }
            };

            // CRITICAL: Stop any existing capture first (from preview or other widgets)
            // startCapture() returns false if capture is already running
            qDebug() << "[HandheldScanWidget::ScanThread] Stopping any existing capture...";
            camera_system_->stopCapture();
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Brief delay for cleanup

            // Start continuous capture with our callback
            qDebug() << "[HandheldScanWidget::ScanThread] Starting continuous capture...";
            if (!camera_system_->startCapture(frame_callback)) {
                qCritical() << "[HandheldScanWidget::ScanThread] Failed to start continuous capture";
                return false;
            }

            qDebug() << "[HandheldScanWidget::ScanThread] Continuous capture started, waiting for" << TARGET_FRAMES << "frames...";

            // Wait for TARGET_FRAMES frames (with timeout)
            {
                std::unique_lock<std::mutex> lock(frames_mutex);
                bool success = frames_cv.wait_for(lock, std::chrono::seconds(10), [&]() {
                    return captured_frames.size() >= TARGET_FRAMES ||
                           QThread::currentThread()->isInterruptionRequested();
                });

                if (!success) {
                    qWarning() << "[HandheldScanWidget::ScanThread] Timeout waiting for frames, got"
                              << captured_frames.size() << "/" << TARGET_FRAMES;
                }
            }

            // Stop capture
            camera_system_->stopCapture();
            qDebug() << "[HandheldScanWidget::ScanThread] Capture stopped";

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

                // Convert gui frames to api format
                std::vector<api::HandheldScanPipeline::StereoFrame> api_frames;
                api_frames.reserve(captured_frames.size());

                for (const auto& gui_frame : captured_frames) {
                    api::HandheldScanPipeline::StereoFrame api_frame;
                    api_frame.leftImage = gui_frame.left_frame.image.clone();
                    api_frame.rightImage = gui_frame.right_frame.image.clone();
                    api_frame.timestampUs = gui_frame.left_frame.timestamp_ns / 1000;
                    api_frame.leftVCSEL = api_frame.leftImage;   // VCSEL is same as main image
                    api_frame.rightVCSEL = api_frame.rightImage;
                    api_frame.stabilityScore = 1.0f;  // Assumed stable since frames were captured

                    api_frames.push_back(api_frame);
                }

                qDebug() << "[HandheldScanWidget::ScanThread] Processing" << api_frames.size() << "frames with VCSELStereoMatcher...";

                // Get stereo parameters
                auto stereo_params = pipeline->getStereoParams();

                // Process frames to depth maps using AD-Census
                auto depth_maps = pipeline->processFrames(api_frames, stereo_params);

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

    // HIDDEN: Update UI with results
    // ui->precisionLabel->setText("Precision: " + QString::number(achieved_precision_mm_, 'f', 2) + " mm");
    // ui->precisionLabel->setStyleSheet("font-size: 16pt; font-weight: bold; color: #00FF00;");
    // ui->statusLabel->setText("Scan complete! Duration: " + QString::number(duration_ms) + " ms | Points: " + QString::number(point_count_));
    // ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00FF00;");

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

    // HIDDEN:     ui->statusLabel->setText("Scan failed: " + error);
    // HIDDEN:     ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");

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

} // namespace gui
} // namespace unlook
