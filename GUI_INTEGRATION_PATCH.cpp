/**
 * @file GUI_INTEGRATION_PATCH.cpp
 * @brief PHASE 5.3: GUI Integration patches for HandheldScanWidget
 *
 * This file contains the key modifications needed for GUI integration
 * with the new backend pipeline. Apply these changes to handheld_scan_widget.cpp
 */

// ========== KEY MODIFICATION 1: Update startScanThread() ==========
// Location: Around line 710 in handheld_scan_widget.cpp
// Replace the processing section with this improved version:

void HandheldScanWidget::startScanThread() {
    // ... existing capture code ...

    // After frames are captured, around line 710:
    qDebug() << "[HandheldScanWidget::ScanThread] Processing" << api_frames.size() << "frames with NEW BACKEND...";

    // Show progress frame on main thread
    QMetaObject::invokeMethod(this, [this]() {
        ui->progressFrame->setVisible(true);
        ui->progressBar->setValue(0);
        ui->statusLabel->setText("Initializing stereo pipeline...");
    }, Qt::QueuedConnection);

    // Get stereo parameters
    auto stereo_params = pipeline->getStereoParams();

    // Track processing start time for ETA calculation
    auto processing_start = std::chrono::steady_clock::now();
    std::atomic<int> frames_completed{0};
    const int total_frames = api_frames.size();

    // NEW: Process with granular progress reporting
    auto depth_maps = pipeline->processFrames(api_frames, stereo_params,
        [this, &processing_start, &frames_completed, total_frames](float progress, const std::string& message) {
            frames_completed++;

            // Calculate detailed progress stages
            float overall_progress = 0.0f;
            QString detailed_status;

            // Parse message to determine stage
            if (message.find("Rectifying") != std::string::npos) {
                overall_progress = 0.1f + progress * 0.2f;  // 10-30%
                detailed_status = QString("Stage 1/5: Rectifying images...");
            }
            else if (message.find("Computing disparity") != std::string::npos) {
                overall_progress = 0.3f + progress * 0.3f;  // 30-60%
                detailed_status = QString("Stage 2/5: Computing disparity (AD-CENSUS)...");
            }
            else if (message.find("Filtering") != std::string::npos) {
                overall_progress = 0.6f + progress * 0.15f; // 60-75%
                detailed_status = QString("Stage 3/5: Filtering depth map...");
            }
            else if (message.find("Converting") != std::string::npos) {
                overall_progress = 0.75f + progress * 0.15f; // 75-90%
                detailed_status = QString("Stage 4/5: Converting to depth...");
            }
            else if (message.find("Generating") != std::string::npos) {
                overall_progress = 0.9f + progress * 0.1f;  // 90-100%
                detailed_status = QString("Stage 5/5: Generating point cloud...");
            }
            else {
                // Default: use message as-is with frame counter
                overall_progress = progress;
                detailed_status = QString::fromStdString(message);
            }

            // Calculate ETA
            auto elapsed = std::chrono::steady_clock::now() - processing_start;
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

            QString eta_text;
            if (frames_completed > 0 && progress < 1.0f) {
                float ms_per_progress = elapsed_ms / progress;
                float remaining_ms = ms_per_progress * (1.0f - progress);
                int eta_sec = static_cast<int>(remaining_ms / 1000.0f);

                if (eta_sec > 0) {
                    int eta_min = eta_sec / 60;
                    int eta_sec_remainder = eta_sec % 60;
                    eta_text = QString(" (ETA: %1:%2)")
                        .arg(eta_min)
                        .arg(eta_sec_remainder, 2, 10, QChar('0'));
                }
            }

            // Check if GPU is being used (from statistics if available)
            QString gpu_status;
            auto stats = pipeline->getStatistics();
            if (stats.find("gpu_available") != stats.end() && stats["gpu_available"] > 0) {
                gpu_status = " [GPU ACCELERATED]";
            } else {
                gpu_status = " [CPU/NEON]";
            }

            // Build complete status message
            QString complete_status = detailed_status + eta_text + gpu_status;

            // Update UI on main thread with smooth animation
            QMetaObject::invokeMethod(this, [this, overall_progress, complete_status]() {
                // Smooth progress bar animation
                int target_value = static_cast<int>(overall_progress * 100);
                int current_value = ui->progressBar->value();

                // Animate progress bar smoothly
                if (target_value > current_value) {
                    ui->progressBar->setValue(target_value);
                }

                // Update status text
                ui->statusLabel->setText(complete_status);

                // Color-code progress bar based on stage
                QString style_sheet;
                if (overall_progress < 0.3f) {
                    // Rectification stage - blue
                    style_sheet = "QProgressBar::chunk { background-color: #4169E1; }";
                } else if (overall_progress < 0.6f) {
                    // Disparity stage - purple
                    style_sheet = "QProgressBar::chunk { background-color: #9370DB; }";
                } else if (overall_progress < 0.9f) {
                    // Filtering stage - orange
                    style_sheet = "QProgressBar::chunk { background-color: #FFA500; }";
                } else {
                    // Final stage - green
                    style_sheet = "QProgressBar::chunk { background-color: #00FF00; }";
                }
                ui->progressBar->setStyleSheet(style_sheet);
            }, Qt::QueuedConnection);
        }
    );

    // ... rest of processing code ...
}

// ========== KEY MODIFICATION 2: Add GPU status display ==========
// Location: Add new method to HandheldScanWidget class

void HandheldScanWidget::updateGPUStatus() {
    if (!pipeline_) return;

    auto stats = pipeline_->getStatistics();

    // Check GPU availability
    bool gpu_available = false;
    float gpu_utilization = 0.0f;
    QString gpu_name = "None";

    if (stats.find("gpu_available") != stats.end()) {
        gpu_available = stats["gpu_available"] > 0;
    }

    if (gpu_available) {
        gpu_name = "VideoCore VII";  // Raspberry Pi 5 GPU

        if (stats.find("gpu_utilization") != stats.end()) {
            gpu_utilization = stats["gpu_utilization"];
        }
    }

    // Update UI (could be shown in info dialog or status bar)
    QString gpu_text;
    if (gpu_available) {
        gpu_text = QString("GPU: %1 (%2% usage)")
            .arg(gpu_name)
            .arg(static_cast<int>(gpu_utilization));
    } else {
        gpu_text = "GPU: Not available (using CPU/NEON)";
    }

    // Store for info dialog
    gpu_status_text_ = gpu_text;
}

// ========== KEY MODIFICATION 3: Enhanced error handling ==========
// Location: Add to scan thread error handling

// In startScanThread(), wrap pipeline operations with better error handling:
try {
    // Create pipeline with proper error checking
    auto pipeline = std::make_unique<api::HandheldScanPipeline>(nullptr);

    // Initialize with validation
    if (!pipeline->initialize()) {
        QString error_msg = "Pipeline initialization failed!\n\n"
                          "Possible causes:\n"
                          "• Calibration file missing or invalid\n"
                          "• Calibration resolution mismatch\n"
                          "• GPU driver not available\n\n"
                          "Please check /unlook_calib/ for valid calibration files.";

        QMetaObject::invokeMethod(this, [this, error_msg]() {
            QMessageBox::critical(this, "Initialization Error", error_msg);
            resetUI();
        }, Qt::QueuedConnection);

        return false;
    }

    // ... processing code ...

} catch (const unlook::core::Exception& e) {
    // Handle Unlook-specific exceptions
    QString error_msg = QString("Processing error:\n%1\n\nError code: %2")
        .arg(QString::fromStdString(e.what()))
        .arg(static_cast<int>(e.getCode()));

    QMetaObject::invokeMethod(this, [this, error_msg]() {
        QMessageBox::critical(this, "Processing Error", error_msg);
        resetUI();
    }, Qt::QueuedConnection);

    return false;

} catch (const std::exception& e) {
    // Handle standard exceptions
    QString error_msg = QString("Unexpected error:\n%1")
        .arg(QString::fromStdString(e.what()));

    QMetaObject::invokeMethod(this, [this, error_msg]() {
        QMessageBox::critical(this, "Error", error_msg);
        resetUI();
    }, Qt::QueuedConnection);

    return false;
}

// ========== KEY MODIFICATION 4: Show calibration validation in UI ==========
// Location: Add to onStartScan() method

void HandheldScanWidget::onStartScan() {
    qDebug() << "[HandheldScanWidget] Starting handheld scan...";

    // Show initial status
    ui->progressFrame->setVisible(true);
    ui->progressBar->setValue(0);
    ui->statusLabel->setText("Validating calibration...");

    // Quick calibration check before starting
    QTimer::singleShot(100, [this]() {
        // This runs after UI updates
        try {
            // Create temporary pipeline to validate calibration
            auto test_pipeline = std::make_unique<api::HandheldScanPipeline>(nullptr);

            if (!test_pipeline->initialize()) {
                ui->statusLabel->setText("ERROR: Calibration validation failed!");
                ui->progressBar->setStyleSheet("QProgressBar::chunk { background-color: #FF0000; }");

                QMessageBox::critical(this, "Calibration Error",
                    "Calibration validation failed!\n\n"
                    "The calibration file does not match the expected image resolution.\n"
                    "Expected: 1280x720\n\n"
                    "Please recalibrate the cameras.");

                resetUI();
                return;
            }

            // Calibration OK, proceed with scan
            ui->statusLabel->setText("Calibration validated, starting capture...");
            ui->progressBar->setValue(10);

            // Continue with actual scan
            startScanThread();

        } catch (const std::exception& e) {
            ui->statusLabel->setText("ERROR: " + QString::fromStdString(e.what()));
            resetUI();
        }
    });
}

// ========== KEY MODIFICATION 5: Multi-frame progress ==========
// Location: Add to capture phase

// In the capture phase, show per-frame progress:
auto frame_callback = [&](const core::StereoFramePair& frame) {
    std::lock_guard<std::mutex> lock(frames_mutex);

    if (captured_frames.size() < TARGET_FRAMES) {
        captured_frames.push_back(frame);
        int count = captured_frames.size();
        frames_received = count;

        // Update UI with frame capture progress
        QMetaObject::invokeMethod(this, [this, count, TARGET_FRAMES]() {
            float capture_progress = static_cast<float>(count) / TARGET_FRAMES;
            int progress_value = static_cast<int>(capture_progress * 20); // 0-20% for capture

            ui->progressBar->setValue(progress_value);
            ui->statusLabel->setText(QString("Capturing frame %1/%2...")
                .arg(count)
                .arg(TARGET_FRAMES));

            // Green pulse effect on capture
            if (count == TARGET_FRAMES) {
                ui->progressBar->setStyleSheet(
                    "QProgressBar::chunk { "
                    "background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0, "
                    "stop:0 #00FF00, stop:1 #00AA00); }");
            }
        }, Qt::QueuedConnection);

        if (count >= TARGET_FRAMES) {
            frames_cv.notify_one();
        }
    }
};

// ========== NOTES FOR INTEGRATION ==========
/*
 * 1. These modifications enable real-time progress updates in the GUI
 * 2. The progress bar shows 5 distinct stages with color coding
 * 3. GPU acceleration status is displayed when available
 * 4. Calibration validation happens BEFORE capture starts
 * 5. Error messages are user-friendly with actionable information
 *
 * TO APPLY:
 * - Copy relevant sections into handheld_scan_widget.cpp
 * - Ensure ui->progressFrame, ui->progressBar, ui->statusLabel exist in .ui file
 * - Test with both valid and invalid calibration files
 * - Monitor GPU usage with VideoCore VII tools if available
 */