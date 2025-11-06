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

    // Stop camera preview
    stopCameraPreview();

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

    qDebug() << "[HandheldScanWidget::showEvent] Widget now visible - starting camera preview";

    // Start camera preview for real-time feedback
    startCameraPreview();
}

void HandheldScanWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    qDebug() << "[HandheldScanWidget::hideEvent] Widget hidden";

    // Stop camera preview
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

    // Initialize progress bars
    ui->stabilityProgressBar->setMinimum(0);
    ui->stabilityProgressBar->setMaximum(100);
    ui->stabilityProgressBar->setValue(0);

    ui->captureProgressBar->setMinimum(0);
    ui->captureProgressBar->setMaximum(TARGET_FRAMES);
    ui->captureProgressBar->setValue(0);

    // Set initial state
    ui->stopButton->setEnabled(false);
    ui->captureProgressBar->setVisible(true);  // Keep visible but at 0

    qDebug() << "[HandheldScanWidget::setupUI] UI initialized from .ui file with camera preview";
}

void HandheldScanWidget::onStartScan() {
    qDebug() << "[HandheldScanWidget] Starting handheld scan...";

    // Check camera system is ready
    if (!camera_system_ || !camera_system_->isReady()) {
        qCritical() << "[HandheldScanWidget] Camera system not ready!";
        ui->statusLabel->setText("ERROR: Camera system not ready");
        ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
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
            ui->statusLabel->setText("ERROR: Failed to enable VCSEL");
            ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
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
    ui->captureProgressBar->setVisible(true);
    // removed capture_count_label(true);
    ui->captureProgressBar->setValue(0);
    // ui->statusLabel->setText("0/" + QString::number(TARGET_FRAMES) + " frames");

    ui->statusLabel->setText("Waiting for stability...");
    ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FFA500;");

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

    ui->statusLabel->setText("Scan cancelled");
    ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");
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
            ui->statusLabel->setText("Stable! Capturing frames...");
            ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00FF00;");
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
            ui->fpsLabel->setText("FPS: " + QString::number(avg_fps, 'f', 1));

            qDebug() << "[HandheldScanWidget] Frame" << frames_captured_ << "captured, FPS:" << avg_fps;

            if (frames_captured_ >= TARGET_FRAMES) {
                scan_state_ = ScanState::PROCESSING;
                ui->statusLabel->setText("Processing depth maps...");
                ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00E5CC;");
                qDebug() << "[HandheldScanWidget] All frames captured, processing...";
            }
        }
    } else if (scan_state_ == ScanState::PROCESSING) {
        // Processing happens in background thread
        // UI shows animated status (handled by background thread completion)
        static int dots = 0;
        dots = (dots + 1) % 4;
        QString dot_string = QString(".").repeated(dots);
        ui->statusLabel->setText("Processing" + dot_string);
    }
}

void HandheldScanWidget::updateStabilityIndicator(float score) {
    int percentage = static_cast<int>(score * 100.0f);
    ui->stabilityProgressBar->setValue(percentage);

    // Update color based on score
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

    // Update text label
    QString text = getStabilityText(score);
    ui->stabilityTextLabel->setText(text);
    ui->stabilityTextLabel->setStyleSheet("font-size: 18pt; font-weight: bold; color: " + color_str + ";");
}

void HandheldScanWidget::updateCaptureProgress(int current, int total) {
    ui->captureProgressBar->setValue(current);
    // ui->statusLabel->setText(QString::number(current) + "/" + QString::number(total) + " frames");

    // Calculate percentage for styling
    float percentage = static_cast<float>(current) / static_cast<float>(total);
    QColor color;
    if (percentage < 0.5f) {
        color = QColor("#FFA500");  // Orange
    } else if (percentage < 1.0f) {
        color = QColor("#00E5CC");  // Cyan
    } else {
        color = QColor("#00FF00");  // Green
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
    ui->captureProgressBar->setVisible(false);
    // removed capture_count_label(false);

    ui->stabilityProgressBar->setValue(0);
    ui->stabilityTextLabel->setText("Hold steady...");
    ui->fpsLabel->setText("FPS: --");
    ui->precisionLabel->setText("Precision: 0.10mm target");

    ui->statusLabel->setText("Ready to scan");
    ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00E5CC;");
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
                // Create HandheldScanPipeline with real camera system singleton
                auto api_camera_system = camera::CameraSystem::getInstance();
                auto pipeline = std::make_unique<api::HandheldScanPipeline>(api_camera_system);

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

    ui->precisionLabel->setText("Precision: " + QString::number(achieved_precision_mm_, 'f', 2) + " mm");
    ui->precisionLabel->setStyleSheet("font-size: 16pt; font-weight: bold; color: #00FF00;");

    ui->statusLabel->setText(
        "Scan complete! Duration: " + QString::number(duration_ms) + " ms | Points: " +
        QString::number(point_count_)
    );
    ui->statusLabel->setStyleSheet("font-size: 14pt; color: #00FF00;");

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

    ui->statusLabel->setText("Scan failed: " + error);
    ui->statusLabel->setStyleSheet("font-size: 14pt; color: #FF4444;");

    // Reset UI after delay
    QTimer::singleShot(3000, this, [this]() {
        resetUI();
    });

    // Emit failure signal
    emit scanFailed(error);
}

void HandheldScanWidget::startCameraPreview() {
    if (!camera_system_ || !camera_system_->isReady()) {
        qWarning() << "[HandheldScanWidget::startCameraPreview] Camera system not ready";
        return;
    }

    qDebug() << "[HandheldScanWidget::startCameraPreview] Starting real-time preview";

    // Start continuous capture with callback for preview
    bool success = camera_system_->startCapture([this](const core::StereoFramePair& frame) {
        // This callback runs in camera thread, post to main thread for UI update
        QMetaObject::invokeMethod(this, [this, frame]() {
            onPreviewFrame(frame);
        }, Qt::QueuedConnection);
    });

    if (!success) {
        qWarning() << "[HandheldScanWidget::startCameraPreview] Failed to start camera capture";
    }
}

void HandheldScanWidget::stopCameraPreview() {
    if (!camera_system_) {
        return;
    }

    qDebug() << "[HandheldScanWidget::stopCameraPreview] Stopping preview";
    camera_system_->stopCapture();

    // Clear preview
    ui->cameraPreviewLabel->setText("Camera Preview\n(Stopped)");
}

void HandheldScanWidget::onPreviewFrame(const core::StereoFramePair& frame) {
    if (!frame.synchronized || !frame.left_frame.valid) {
        return;
    }

    // Convert left camera image to QImage for display
    const cv::Mat& img = frame.left_frame.image;

    if (img.empty()) {
        return;
    }

    // Convert to RGB for display
    cv::Mat rgb;
    if (img.channels() == 1) {
        cv::cvtColor(img, rgb, cv::COLOR_GRAY2RGB);
    } else if (img.channels() == 3) {
        cv::cvtColor(img, rgb, cv::COLOR_BGR2RGB);
    } else {
        return;
    }

    // Create QImage
    QImage qimg(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);

    // Scale to fit preview label while maintaining aspect ratio
    QPixmap pixmap = QPixmap::fromImage(qimg.copy());  // copy() to detach from cv::Mat memory
    QPixmap scaled = pixmap.scaled(ui->cameraPreviewLabel->size(),
                                   Qt::KeepAspectRatio,
                                   Qt::SmoothTransformation);

    // Draw crosshair overlay
    QPainter painter(&scaled);
    painter.setPen(QPen(QColor(0, 217, 255), 2));  // Cyan color
    int cx = scaled.width() / 2;
    int cy = scaled.height() / 2;
    int crosshair_size = 30;

    // Horizontal line
    painter.drawLine(cx - crosshair_size, cy, cx + crosshair_size, cy);
    // Vertical line
    painter.drawLine(cx, cy - crosshair_size, cx, cy + crosshair_size);
    // Circle
    painter.drawEllipse(QPoint(cx, cy), crosshair_size / 2, crosshair_size / 2);

    painter.end();

    // Update preview label
    ui->cameraPreviewLabel->setPixmap(scaled);
}

} // namespace gui
} // namespace unlook
