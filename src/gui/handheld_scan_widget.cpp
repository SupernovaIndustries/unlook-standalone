#include "unlook/gui/handheld_scan_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/api/HandheldScanPipeline.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFrame>
#include <QtConcurrent>
#include <QDebug>
#include <algorithm>
#include <numeric>
#include <cmath>

namespace unlook {
namespace gui {

HandheldScanWidget::HandheldScanWidget(QWidget* parent)
    : QWidget(parent)
    , camera_system_(camera::CameraSystem::getInstance())
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
    // Configure Logger for file output
    unlook::core::Logger::getInstance().initialize(
        unlook::core::LogLevel::DEBUG,  // Log level
        true,                            // Console output
        true,                            // File output
        "/unlook_logs/handheld_scan.log" // Log file path
    );

    // Create debug directories
    system("mkdir -p /unlook_logs");
    system("mkdir -p /unlook_debug");

    qDebug() << "[HandheldScanWidget] Logger configured with file output: /unlook_logs/handheld_scan.log";
    qDebug() << "[HandheldScanWidget] Debug directories created: /unlook_logs, /unlook_debug";

    // Initialize camera system if not already done
    if (!camera_system_->isInitialized()) {
        qDebug() << "[HandheldScanWidget] Camera system not initialized, initializing now...";

        if (!camera_system_->initialize()) {
            qCritical() << "[HandheldScanWidget] CRITICAL: Failed to initialize camera system!";
            // Widget will be created but scanning will fail with proper error message
        } else {
            qDebug() << "[HandheldScanWidget] Camera system initialized successfully";
        }
    } else {
        qDebug() << "[HandheldScanWidget] Camera system already initialized";
    }

    setupUI();
    applySupernovanStyling();

    // Setup update timer (30 Hz for smooth UI updates)
    update_timer_ = new QTimer(this);
    connect(update_timer_, &QTimer::timeout, this, &HandheldScanWidget::updateUI);
    update_timer_->start(1000 / UI_UPDATE_HZ);  // 33ms interval

    // Initialize FPS sample buffer
    fps_samples_.reserve(FPS_SAMPLE_COUNT);

    qDebug() << "[HandheldScanWidget] Initialized with 30 Hz UI updates";
}

HandheldScanWidget::~HandheldScanWidget() {
    if (update_timer_) {
        update_timer_->stop();
    }

    if (scan_watcher_) {
        scan_watcher_->cancel();
        scan_watcher_->waitForFinished();
    }

    qDebug() << "[HandheldScanWidget] Destroyed";
}

void HandheldScanWidget::setupUI() {
    // Main layout
    QVBoxLayout* main_layout = new QVBoxLayout(this);
    main_layout->setSpacing(20);
    main_layout->setContentsMargins(30, 30, 30, 30);

    // Title
    title_label_ = new QLabel("HANDHELD SCAN", this);
    title_label_->setAlignment(Qt::AlignCenter);
    title_label_->setStyleSheet("font-size: 24pt; font-weight: bold; color: #00E5CC;");
    main_layout->addWidget(title_label_);

    // Stability Section
    QGroupBox* stability_group = new QGroupBox("Stability Indicator", this);
    QVBoxLayout* stability_layout = new QVBoxLayout(stability_group);
    stability_layout->setSpacing(10);

    stability_bar_ = new QProgressBar(this);
    stability_bar_->setMinimum(0);
    stability_bar_->setMaximum(100);
    stability_bar_->setValue(0);
    stability_bar_->setTextVisible(true);
    stability_bar_->setFormat("%p%");
    stability_bar_->setMinimumHeight(40);
    stability_layout->addWidget(stability_bar_);

    stability_text_label_ = new QLabel("Hold steady...", this);
    stability_text_label_->setAlignment(Qt::AlignCenter);
    stability_text_label_->setStyleSheet("font-size: 18pt; font-weight: bold;");
    stability_layout->addWidget(stability_text_label_);

    main_layout->addWidget(stability_group);

    // Capture Progress Section
    QGroupBox* capture_group = new QGroupBox("Capture Progress", this);
    QVBoxLayout* capture_layout = new QVBoxLayout(capture_group);
    capture_layout->setSpacing(10);

    capture_progress_bar_ = new QProgressBar(this);
    capture_progress_bar_->setMinimum(0);
    capture_progress_bar_->setMaximum(TARGET_FRAMES);
    capture_progress_bar_->setValue(0);
    capture_progress_bar_->setTextVisible(false);
    capture_progress_bar_->setMinimumHeight(30);
    capture_progress_bar_->setVisible(false);  // Hidden until capture starts
    capture_layout->addWidget(capture_progress_bar_);

    capture_count_label_ = new QLabel("0/" + QString::number(TARGET_FRAMES) + " frames", this);
    capture_count_label_->setAlignment(Qt::AlignCenter);
    capture_count_label_->setStyleSheet("font-size: 14pt;");
    capture_count_label_->setVisible(false);  // Hidden until capture starts
    capture_layout->addWidget(capture_count_label_);

    main_layout->addWidget(capture_group);

    // Metrics Section
    QGroupBox* metrics_group = new QGroupBox("Metrics", this);
    QHBoxLayout* metrics_layout = new QHBoxLayout(metrics_group);
    metrics_layout->setSpacing(20);

    fps_label_ = new QLabel("FPS: --", this);
    fps_label_->setAlignment(Qt::AlignCenter);
    fps_label_->setStyleSheet("font-size: 16pt; font-weight: bold;");
    metrics_layout->addWidget(fps_label_);

    // Separator
    QFrame* separator = new QFrame(this);
    separator->setFrameShape(QFrame::VLine);
    separator->setFrameShadow(QFrame::Sunken);
    metrics_layout->addWidget(separator);

    precision_label_ = new QLabel("Precision: 0.10mm target", this);
    precision_label_->setAlignment(Qt::AlignCenter);
    precision_label_->setStyleSheet("font-size: 16pt; font-weight: bold;");
    metrics_layout->addWidget(precision_label_);

    main_layout->addWidget(metrics_group);

    // Control Buttons
    QHBoxLayout* button_layout = new QHBoxLayout();
    button_layout->setSpacing(15);

    scan_button_ = new QPushButton("START HANDHELD SCAN", this);
    scan_button_->setMinimumHeight(60);
    scan_button_->setStyleSheet(
        "QPushButton {"
        "    font-size: 18pt;"
        "    font-weight: bold;"
        "    padding: 15px;"
        "    border-radius: 10px;"
        "    background-color: #00E5CC;"
        "    color: #000000;"
        "}"
        "QPushButton:hover {"
        "    background-color: #00B8A3;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #008B7A;"
        "}"
        "QPushButton:disabled {"
        "    background-color: #666666;"
        "    color: #AAAAAA;"
        "}"
    );
    connect(scan_button_, &QPushButton::clicked, this, &HandheldScanWidget::onStartScan);
    button_layout->addWidget(scan_button_);

    stop_button_ = new QPushButton("STOP SCAN", this);
    stop_button_->setMinimumHeight(60);
    stop_button_->setVisible(false);  // Hidden until scan starts
    stop_button_->setStyleSheet(
        "QPushButton {"
        "    font-size: 18pt;"
        "    font-weight: bold;"
        "    padding: 15px;"
        "    border-radius: 10px;"
        "    background-color: #FF4444;"
        "    color: #FFFFFF;"
        "}"
        "QPushButton:hover {"
        "    background-color: #CC0000;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #990000;"
        "}"
    );
    connect(stop_button_, &QPushButton::clicked, this, &HandheldScanWidget::onStopScan);
    button_layout->addWidget(stop_button_);

    main_layout->addLayout(button_layout);

    // Status Section
    status_label_ = new QLabel("Ready to scan", this);
    status_label_->setAlignment(Qt::AlignCenter);
    status_label_->setStyleSheet("font-size: 14pt; color: #00E5CC;");
    status_label_->setWordWrap(true);
    main_layout->addWidget(status_label_);

    // Add stretch to push everything to top
    main_layout->addStretch();

    setLayout(main_layout);
}

void HandheldScanWidget::applySupernovanStyling() {
    // Apply Supernova theme to the widget
    setStyleSheet(
        "QGroupBox {"
        "    font-size: 14pt;"
        "    font-weight: bold;"
        "    color: #00E5CC;"
        "    border: 2px solid #00E5CC;"
        "    border-radius: 8px;"
        "    margin-top: 10px;"
        "    padding-top: 10px;"
        "}"
        "QGroupBox::title {"
        "    subcontrol-origin: margin;"
        "    subcontrol-position: top center;"
        "    padding: 0 10px;"
        "    background-color: #000000;"
        "}"
    );
}

void HandheldScanWidget::onStartScan() {
    qDebug() << "[HandheldScanWidget] Starting handheld scan...";

    // Reset state
    scan_state_ = ScanState::WAITING_STABILITY;
    frames_captured_ = 0;
    current_stability_ = 0.0f;
    achieved_precision_mm_ = 0.0f;
    point_count_ = 0;
    scan_start_time_ = std::chrono::steady_clock::now();
    fps_samples_.clear();

    // Update UI
    scan_button_->setVisible(false);
    stop_button_->setVisible(true);
    capture_progress_bar_->setVisible(true);
    capture_count_label_->setVisible(true);
    capture_progress_bar_->setValue(0);
    capture_count_label_->setText("0/" + QString::number(TARGET_FRAMES) + " frames");

    status_label_->setText("Waiting for stability...");
    status_label_->setStyleSheet("font-size: 14pt; color: #FFA500;");

    // Start background scan thread
    startScanThread();
}

void HandheldScanWidget::onStopScan() {
    qDebug() << "[HandheldScanWidget] Stopping scan...";

    // Cancel ongoing scan
    if (scan_watcher_) {
        scan_watcher_->cancel();
    }

    // Reset UI
    resetUI();

    status_label_->setText("Scan cancelled");
    status_label_->setStyleSheet("font-size: 14pt; color: #FF4444;");
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
            status_label_->setText("Stable! Capturing frames...");
            status_label_->setStyleSheet("font-size: 14pt; color: #00FF00;");
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
            fps_label_->setText("FPS: " + QString::number(avg_fps, 'f', 1));

            qDebug() << "[HandheldScanWidget] Frame" << frames_captured_ << "captured, FPS:" << avg_fps;

            if (frames_captured_ >= TARGET_FRAMES) {
                scan_state_ = ScanState::PROCESSING;
                status_label_->setText("Processing depth maps...");
                status_label_->setStyleSheet("font-size: 14pt; color: #00E5CC;");
                qDebug() << "[HandheldScanWidget] All frames captured, processing...";
            }
        }
    } else if (scan_state_ == ScanState::PROCESSING) {
        // Processing happens in background thread
        // UI shows animated status (handled by background thread completion)
        static int dots = 0;
        dots = (dots + 1) % 4;
        QString dot_string = QString(".").repeated(dots);
        status_label_->setText("Processing" + dot_string);
    }
}

void HandheldScanWidget::updateStabilityIndicator(float score) {
    int percentage = static_cast<int>(score * 100.0f);
    stability_bar_->setValue(percentage);

    // Update color based on score
    QColor color = getStabilityColor(score);
    QString color_str = color.name();

    stability_bar_->setStyleSheet(
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
    stability_text_label_->setText(text);
    stability_text_label_->setStyleSheet("font-size: 18pt; font-weight: bold; color: " + color_str + ";");
}

void HandheldScanWidget::updateCaptureProgress(int current, int total) {
    capture_progress_bar_->setValue(current);
    capture_count_label_->setText(QString::number(current) + "/" + QString::number(total) + " frames");

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

    capture_progress_bar_->setStyleSheet(
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
    scan_button_->setVisible(true);
    stop_button_->setVisible(false);
    capture_progress_bar_->setVisible(false);
    capture_count_label_->setVisible(false);

    stability_bar_->setValue(0);
    stability_text_label_->setText("Hold steady...");
    fps_label_->setText("FPS: --");
    precision_label_->setText("Precision: 0.10mm target");

    status_label_->setText("Ready to scan");
    status_label_->setStyleSheet("font-size: 14pt; color: #00E5CC;");
}

void HandheldScanWidget::startScanThread() {
    // Create background scan thread using QtConcurrent
    QFuture<bool> future = QtConcurrent::run([this]() -> bool {
        qDebug() << "[HandheldScanWidget::ScanThread] Starting background scan with real pipeline...";

        try {
            // Create HandheldScanPipeline instance
            auto pipeline = std::make_shared<unlook::api::HandheldScanPipeline>(camera_system_);

            // Initialize pipeline
            if (!pipeline->initialize()) {
                qCritical() << "[HandheldScanWidget::ScanThread] Failed to initialize pipeline";
                return false;
            }

            qDebug() << "[HandheldScanWidget::ScanThread] Pipeline initialized successfully";

            // Configure scan parameters
            unlook::api::HandheldScanPipeline::ScanParams params;
            params.numFrames = TARGET_FRAMES;
            params.targetPrecisionMM = 0.1f;
            params.stabilityThreshold = STABILITY_THRESHOLD;
            params.useWLSFilter = true;
            params.useVCSEL = false;  // VCSEL disabled for now (API mismatch issue)

            qDebug() << "[HandheldScanWidget::ScanThread] Starting scan with params:";
            qDebug() << "  numFrames:" << params.numFrames;
            qDebug() << "  targetPrecision:" << params.targetPrecisionMM << "mm";
            qDebug() << "  stabilityThreshold:" << params.stabilityThreshold;

            // Run scan with progress callback
            auto result = pipeline->scanWithStability(params,
                [this](float progress, const std::string& message) {
                    // Update UI via signals (thread-safe)
                    qDebug() << "[Pipeline Progress]" << progress << ":" << QString::fromStdString(message);
                });

            // Check if cancelled
            if (QThread::currentThread()->isInterruptionRequested()) {
                qDebug() << "[HandheldScanWidget::ScanThread] Scan cancelled by user";
                pipeline->shutdown();
                return false;
            }

            // Check result
            if (!result.success) {
                qCritical() << "[HandheldScanWidget::ScanThread] Scan failed:"
                           << QString::fromStdString(result.errorMessage);
                pipeline->shutdown();
                return false;
            }

            // Extract results
            achieved_precision_mm_ = result.achievedPrecisionMM;
            point_count_ = result.pointCloud.rows;

            qDebug() << "[HandheldScanWidget::ScanThread] Scan completed successfully";
            qDebug() << "  Precision:" << achieved_precision_mm_ << "mm";
            qDebug() << "  Points:" << point_count_;
            qDebug() << "  Valid pixels:" << result.validPixelPercentage << "%";
            qDebug() << "  Scan duration:" << result.scanDuration.count() << "ms";
            qDebug() << "  Frames captured:" << result.framesCaptures;
            qDebug() << "  Frames used:" << result.framesUsed;

            // Shutdown pipeline
            pipeline->shutdown();

            return true;

        } catch (const std::exception& e) {
            qCritical() << "[HandheldScanWidget::ScanThread] Scan failed with exception:" << e.what();
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

    scan_state_ = ScanState::COMPLETED;

    // Update UI with results
    auto scan_duration = std::chrono::steady_clock::now() - scan_start_time_;
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(scan_duration).count();

    precision_label_->setText("Precision: " + QString::number(achieved_precision_mm_, 'f', 2) + " mm");
    precision_label_->setStyleSheet("font-size: 16pt; font-weight: bold; color: #00FF00;");

    status_label_->setText(
        "Scan complete! Duration: " + QString::number(duration_ms) + " ms | Points: " +
        QString::number(point_count_)
    );
    status_label_->setStyleSheet("font-size: 14pt; color: #00FF00;");

    // Hide stop button, show scan button
    stop_button_->setVisible(false);
    scan_button_->setVisible(true);

    // Emit completion signal
    emit scanCompleted(point_count_, achieved_precision_mm_);

    qDebug() << "[HandheldScanWidget] Emitted scanCompleted signal";
}

void HandheldScanWidget::onScanFailed(const QString& error) {
    qCritical() << "[HandheldScanWidget] Scan failed:" << error;

    scan_state_ = ScanState::FAILED;

    status_label_->setText("Scan failed: " + error);
    status_label_->setStyleSheet("font-size: 14pt; color: #FF4444;");

    // Reset UI after delay
    QTimer::singleShot(3000, this, [this]() {
        resetUI();
    });

    // Emit failure signal
    emit scanFailed(error);
}

} // namespace gui
} // namespace unlook
