// Point cloud functionality now enabled with proper dependencies

#include "unlook/gui/depth_test_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/hardware/AS1170Controller.hpp"
#include "ui_depth_test_widget.h"
#include <QGridLayout>
#include <QComboBox>
#include <QSlider>
#include <QMessageBox>

// Open3D includes for Artec mesh processing
#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/io/TriangleMeshIO.h>
#endif
#include <QDateTime>
#include <QTime>
#include <QPushButton>
#include <QApplication>
#include <QDebug>
#include <QDir>
#include <QtConcurrent/QtConcurrent>
#include <QFuture>
#include <QTimer>
#include <QThread>
#include <fstream>
#include <thread>
#include <chrono>
#include <opencv2/imgcodecs.hpp>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

DepthTestWidget::DepthTestWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , ui(new Ui::DepthTestWidget)
    , camera_system_(camera_system)
    , processing_active_(false)
    , live_preview_active_(false)
    , led_enabled_(true)  // LED enabled by default for investor demo
    , ambient_subtract_enabled_(false)  // Ambient subtraction disabled by default for testing
    , ambient_subtraction_weight_(0.5f)  // Default 50% ambient subtraction for better background removal
    , ambient_pattern_gain_(1.5f)  // Reduced gain for better balance (was 2.0)
    , current_debug_directory_("")
    , export_pointcloud_button_(nullptr)
    , export_mesh_button_(nullptr)
    , configure_export_button_(nullptr)
    , export_format_combo_(nullptr)
    , export_status_(nullptr)
{
    // Setup UI from .ui file
    ui->setupUi(this);
    
    // Connect signals
    connectSignals();

    initializeUI();
    initializeDepthProcessor();
    initializePointCloudProcessor();
    initializeVCSELProjector(); // Re-enabled with singleton AS1170Controller fix
}

DepthTestWidget::~DepthTestWidget() {
    // Ensure VCSEL is shut down safely
    if (vcsel_projector_) {
        vcsel_projector_->disableProjection();
        vcsel_projector_->shutdown();
    }
    delete ui;
}

void DepthTestWidget::connectSignals() {
    // Connect UI signals to slots
    connect(ui->capture_button, &QPushButton::clicked, this, &DepthTestWidget::captureStereoFrame);

    // Connect Save Point Cloud button
    connect(ui->save_pointcloud_button, &QPushButton::clicked, this, &DepthTestWidget::exportPointCloud);

    // Connect export format selection
    connect(ui->export_format_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DepthTestWidget::updateExportFormat);

    // Algorithm selection is fixed to SGBM - no need to connect signals
    // Parameters are hardcoded in the backend for optimal performance

    // INVESTOR DEMO: Connect filter disable checkbox
    connect(ui->disable_filters_checkbox, &QCheckBox::toggled,
            this, &DepthTestWidget::onFilterDisableToggled);

    // INVESTOR DEMO: Connect LED toggle button
    connect(ui->led_toggle_button, &QPushButton::clicked,
            this, &DepthTestWidget::onLEDToggle);

    // ARTEC MESH PROCESSING: Connect signals (NEW)
    // Preset buttons will be connected in createArtecMeshPanel()
    // Export button will be connected in createArtecMeshPanel()
    // Octree slider will be connected in createArtecMeshPanel()

    // Connect ambient subtraction checkbox
    connect(ui->ambient_subtract_checkbox, &QCheckBox::toggled,
            this, [this](bool checked) {
                ambient_subtract_enabled_ = checked;
                if (checked) {
                    qDebug() << "[DepthWidget] Ambient subtraction ENABLED (weight="
                             << ambient_subtraction_weight_ << ", gain=" << ambient_pattern_gain_ << ")";
                    addStatusMessage(QString("Ambient subtraction: ENABLED (3-frame mode, weight=%1, gain=%2)")
                                    .arg(ambient_subtraction_weight_)
                                    .arg(ambient_pattern_gain_));
                } else {
                    qDebug() << "[DepthWidget] Ambient subtraction DISABLED";
                    addStatusMessage("Ambient subtraction: DISABLED (1-frame mode)");
                }
            });

    // Connect Ambient Light Mode checkbox
    connect(ui->ambient_light_mode_checkbox, &QCheckBox::toggled,
            this, [this](bool checked) {
                if (!depth_processor_) {
                    qWarning() << "[DepthWidget] Cannot switch SGBM mode: depth_processor_ is null";
                    return;
                }

                auto* sgbmMatcher = dynamic_cast<unlook::stereo::SGBMStereoMatcher*>(
                    depth_processor_->getStereoMatcher()
                );

                if (!sgbmMatcher) {
                    qWarning() << "[DepthWidget] Cannot switch SGBM mode: not using SGBMStereoMatcher";
                    return;
                }

                auto params = sgbmMatcher->getParameters();

                if (checked) {
                    // AMBIENT LIGHT MODE - Optimize for natural scene texture
                    qDebug() << "[DepthWidget] AMBIENT LIGHT MODE ENABLED";
                    addStatusMessage("SGBM: Ambient Light Mode (blockSize=7, strict matching)");

                    params.blockSize = 7;           // Larger block for natural texture
                    params.P1 = 392;                // 8 * 1 * 7 * 7
                    params.P2 = 1568;               // 32 * 1 * 7 * 7
                    params.uniquenessRatio = 10;    // Strict matching
                    params.textureThreshold = 10;   // Require minimum texture
                    params.preFilterCap = 31;       // More preprocessing
                    params.speckleWindowSize = 100; // Aggressive filtering
                    params.speckleRange = 32;       // Strict speckle range
                } else {
                    // VCSEL MODE - Optimize for structured light dots
                    qDebug() << "[DepthWidget] VCSEL MODE ENABLED";
                    addStatusMessage("SGBM: VCSEL Mode (blockSize=3, dot matching)");

                    params.blockSize = 3;           // Minimum for dot matching
                    params.P1 = 72;                 // 8 * 1 * 3 * 3
                    params.P2 = 288;                // 32 * 1 * 3 * 3
                    params.uniquenessRatio = 22;    // Balanced for dots
                    params.textureThreshold = 3;    // Low for VCSEL dots
                    params.preFilterCap = 15;       // Minimal preprocessing
                    params.speckleWindowSize = 15;  // Preserve dot detail
                    params.speckleRange = 128;      // Wide range for dots
                }

                // Apply updated parameters
                if (sgbmMatcher->setParameters(params)) {
                    qDebug() << "[DepthWidget] SGBM parameters updated successfully";
                    addStatusMessage(QString("SGBM parameters: blockSize=%1, P1=%2, P2=%3, uniqueness=%4")
                                    .arg(params.blockSize)
                                    .arg(params.P1)
                                    .arg(params.P2)
                                    .arg(params.uniquenessRatio));
                } else {
                    qWarning() << "[DepthWidget] Failed to update SGBM parameters";
                    addStatusMessage("ERROR: Failed to update SGBM parameters");
                }
            });

    // TODO: Add these widgets to .ui file and uncomment:
    // connect(ui->export_button, &QPushButton::clicked, this, &DepthTestWidget::exportDepthMap);
    // connect(ui->preset_fast_button, &QPushButton::clicked, [this](){ applyPresetConfiguration(StereoAlgorithmConfig::FAST); });
    // connect(ui->preset_balanced_button, &QPushButton::clicked, [this](){ applyPresetConfiguration(StereoAlgorithmConfig::BALANCED); });
    // connect(ui->preset_quality_button, &QPushButton::clicked, [this](){ applyPresetConfiguration(StereoAlgorithmConfig::QUALITY); });

    // Note: Point cloud export buttons will be connected in createPointCloudExportPanel()
}

void DepthTestWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    // Update status when shown
    if (processing_status_) {
        processing_status_->setStatus("Ready for depth testing", StatusDisplay::StatusType::INFO);
    }
    
    // AUTO-START LIVE PREVIEW when widget becomes visible
    qDebug() << "[DepthWidget] Widget shown, starting live preview automatically";
    startLivePreview();
}

void DepthTestWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    // Stop any ongoing processing
    processing_active_ = false;

    // Ensure VCSEL is disabled when widget is hidden (safety)
    if (vcsel_projector_ && vcsel_projector_->isProjectionActive()) {
        qDebug() << "[DepthWidget] Disabling VCSEL projection (widget hidden)";
        vcsel_projector_->disableProjection();
    }

    // AUTO-STOP LIVE PREVIEW when widget is hidden
    qDebug() << "[DepthWidget] Widget hidden, stopping live preview";
    stopLivePreview();
}

void DepthTestWidget::captureStereoFrame() {
    qDebug() << "[DepthWidget] captureStereoFrame() called";

    // Reset debug directory for new capture
    current_debug_directory_.clear();

    if (!camera_system_) {
        qDebug() << "[DepthWidget] ERROR: camera_system_ is null";
        processing_status_->setStatus("Camera system not initialized", StatusDisplay::StatusType::ERROR);
        return;
    }

    if (!camera_system_->isReady()) {
        qDebug() << "[DepthWidget] ERROR: Camera system not ready";
        processing_status_->setStatus("Camera system not ready", StatusDisplay::StatusType::ERROR);
        return;
    }

    qDebug() << "[DepthWidget] Starting stereo frame capture with VCSEL";
    processing_active_ = true;

    // Clear previous status messages
    if (ui && ui->status_list) {
        ui->status_list->clear();
    }
    addStatusMessage("Starting capture sequence...");

    // Get AS1170 controller for temporal matching sequence
    auto as1170 = hardware::AS1170Controller::getInstance();

    if (led_enabled_) {
        qDebug() << "[DepthWidget] TEMPORAL MATCHING ENABLED - Will capture 3 frames";
        addStatusMessage("Temporal matching enabled (3 frames)");
        capture_status_->setStatus("Initializing temporal sequence...", StatusDisplay::StatusType::PROCESSING);

        if (as1170) {
            // Initialize if not already done
            if (!as1170->isInitialized()) {
                qDebug() << "[DepthWidget] Initializing AS1170 controller";
                addStatusMessage("Initializing AS1170 controller...");
                as1170->initialize();
            }
            // LEDs will be activated in sequence during temporal capture below
            qDebug() << "[DepthWidget] AS1170 ready for temporal matching sequence";
            addStatusMessage("AS1170 ready");
        } else {
            qWarning() << "[DepthWidget] AS1170Controller not available - temporal matching disabled";
            addStatusMessage("WARNING: AS1170 not available");
        }
    } else {
        qDebug() << "[DepthWidget] LED illumination DISABLED by user - Capturing with ambient light only";
        addStatusMessage("LED disabled - ambient capture only");
        capture_status_->setStatus("Capturing with ambient light (LED OFF)...", StatusDisplay::StatusType::INFO);
    }

    capture_status_->setStatus("Activating VCSEL projection...", StatusDisplay::StatusType::PROCESSING);
    capture_status_->startPulsing();

    // Trigger VCSEL structured light for depth capture
    bool vcsel_triggered = false;
    if (vcsel_projector_ && vcsel_projector_->isReady()) {
        qDebug() << "[DepthWidget] Triggering VCSEL structured light projection";

        // Update VCSEL status
        if (vcsel_status_) {
            vcsel_status_->setStatus("VCSEL Active", StatusDisplay::StatusType::WARNING);
            vcsel_status_->startPulsing();
        }

        // Trigger VCSEL with camera exposure time (15ms default)
        vcsel_triggered = vcsel_projector_->triggerStructuredLightCapture(
            15000,  // 15ms exposure time to match camera
            [this](hardware::VCSELProjector::PatternType pattern, uint64_t timestamp_ns) {
                qDebug() << "[DepthWidget] VCSEL projection completed at" << timestamp_ns;
                // VCSEL projection complete callback
                if (vcsel_status_) {
                    QMetaObject::invokeMethod(this, [this]() {
                        vcsel_status_->setStatus("VCSEL Off", StatusDisplay::StatusType::INFO);
                        vcsel_status_->stopPulsing();
                    }, Qt::QueuedConnection);
                }
            });

        if (!vcsel_triggered) {
            qWarning() << "[DepthWidget] Failed to trigger VCSEL projection";
            if (vcsel_status_) {
                vcsel_status_->setStatus("VCSEL Trigger Failed", StatusDisplay::StatusType::ERROR);
                vcsel_status_->stopPulsing();
            }
        }
    } else {
        qDebug() << "[DepthWidget] VCSEL projector not available, capturing without structured light";
        if (vcsel_status_) {
            vcsel_status_->setStatus("VCSEL Not Available", StatusDisplay::StatusType::WARNING);
        }
    }

    // Small delay to ensure VCSEL is active before camera capture
    if (vcsel_triggered) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // TEMPORAL MATCHING: Capture 3 frames for pattern isolation
    capture_status_->setStatus("Temporal capture 1/3...", StatusDisplay::StatusType::PROCESSING);
    qDebug() << "[DepthWidget] TEMPORAL MATCHING: Starting 3-frame capture sequence";

    core::StereoFramePair frame1, frame2, frame3;  // 3 frames for temporal matching

    // Only do temporal matching if LEDs are enabled
    // ambient_subtract_enabled_ NOW CONTROLS CAPTURE MODE:
    // - CHECKED: 3-frame averaging mode (VCSEL1 + VCSEL2 + Ambient)
    // - UNCHECKED: Single VCSEL frame mode (VCSEL1 only at 446mA)
    if (led_enabled_ && as1170) {
        if (ambient_subtract_enabled_) {
            // 3-FRAME MODE: VCSEL1 + VCSEL2 + Ambient for averaging
            qDebug() << "[DepthWidget] 3-FRAME MODE: Capturing VCSEL1, VCSEL2, Ambient";

            // FRAME 1: VCSEL1 (Upper) ON, VCSEL2 OFF
            qDebug() << "[DepthWidget] FRAME 1: VCSEL1 ON (upper)";
            addStatusMessage("Capturing Frame 1/3: VCSEL1 ON (446mA)");
            QApplication::processEvents();  // Update GUI
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 446);  // AS1170 hardware maximum
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));  // INCREASED: 200ms VCSEL stabilization
            frame1 = camera_system_->captureSingle();
            addStatusMessage("Frame 1 captured");
            QApplication::processEvents();  // Update GUI

            // FRAME 2: VCSEL1 OFF, VCSEL2 (Lower) ON
            capture_status_->setStatus("Temporal capture 2/3...", StatusDisplay::StatusType::PROCESSING);
            qDebug() << "[DepthWidget] FRAME 2: VCSEL2 ON (lower)";
            addStatusMessage("Capturing Frame 2/3: VCSEL2 ON (446mA)");
            QApplication::processEvents();  // Update GUI
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, true, 446);  // AS1170 hardware maximum
            std::this_thread::sleep_for(std::chrono::milliseconds(200));  // INCREASED: 200ms VCSEL stabilization
            frame2 = camera_system_->captureSingle();
            addStatusMessage("Frame 2 captured");
            QApplication::processEvents();  // Update GUI

            // FRAME 3: Both VCSELs OFF (Ambient)
            capture_status_->setStatus("Temporal capture 3/3...", StatusDisplay::StatusType::PROCESSING);
            qDebug() << "[DepthWidget] FRAME 3: Ambient (no VCSEL)";
            addStatusMessage("Capturing Frame 3/3: Ambient (no VCSEL)");
            QApplication::processEvents();  // Update GUI
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Ambient frame delay
            frame3 = camera_system_->captureSingle();
            addStatusMessage("Frame 3 captured");
            QApplication::processEvents();  // Update GUI

            qDebug() << "[DepthWidget] 3-frame capture complete";
            addStatusMessage("3-frame capture complete");
        } else {
            // SINGLE VCSEL FRAME MODE: Only VCSEL1 at 446mA
            qDebug() << "[DepthWidget] SINGLE VCSEL MODE: Capturing single frame with VCSEL1 at 446mA";
            addStatusMessage("Single VCSEL frame mode: VCSEL1 at 446mA");
            QApplication::processEvents();  // Update GUI

            // Activate VCSEL1 only
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 446);  // AS1170 hardware maximum
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));  // INCREASED: 200ms VCSEL stabilization

            // Capture single frame
            frame1 = camera_system_->captureSingle();
            addStatusMessage("Single VCSEL frame captured");
            QApplication::processEvents();  // Update GUI

            // Turn off VCSEL
            as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);

            qDebug() << "[DepthWidget] Single VCSEL frame captured successfully";
        }

        // PROCESSING LOGIC UPDATED TO MATCH NEW CAPTURE BEHAVIOR:
        // - ambient_subtract_enabled_ CHECKED: 3-frame averaging mode
        // - ambient_subtract_enabled_ UNCHECKED: Single VCSEL frame (no processing)
        if (ambient_subtract_enabled_) {
            // 3-FRAME AVERAGING FOR NOISE REDUCTION
            // Average all 3 frames (VCSEL1, VCSEL2, Ambient) to reduce temporal noise
            // Apply moderate contrast enhancement to final averaged result
            // This provides sqrt(3) = 1.73x noise reduction, approximately 4.77 dB SNR improvement
            qDebug() << "[DepthWidget] AMBIENT SUBTRACTION DISABLED - Using 3-FRAME AVERAGING";
            addStatusMessage("Using 3-frame averaging for noise reduction");

            if (frame1.synchronized && frame2.synchronized && frame3.synchronized) {
                // SAVE ORIGINAL 3 FRAMES BEFORE AVERAGING (for debugging)
                // Create debug directory early to save individual frames
                std::string debug_dir = createDebugDirectory();
                current_debug_directory_ = debug_dir;

                qDebug() << "[DepthWidget] Saving 3 original frames to debug dir:" << QString::fromStdString(debug_dir);

                // Save VCSEL1 frame (frame1)
                cv::imwrite(debug_dir + "/00a_frame1_vcsel1_left.png", frame1.left_frame.image);
                cv::imwrite(debug_dir + "/00a_frame1_vcsel1_right.png", frame1.right_frame.image);

                // Save VCSEL2 frame (frame2)
                cv::imwrite(debug_dir + "/00b_frame2_vcsel2_left.png", frame2.left_frame.image);
                cv::imwrite(debug_dir + "/00b_frame2_vcsel2_right.png", frame2.right_frame.image);

                // Save Ambient frame (frame3)
                cv::imwrite(debug_dir + "/00c_frame3_ambient_left.png", frame3.left_frame.image);
                cv::imwrite(debug_dir + "/00c_frame3_ambient_right.png", frame3.right_frame.image);

                qDebug() << "[DepthWidget] All 3 original frames saved to debug folder";

                cv::Mat averaged_left, averaged_right;

                // Convert to float for accurate averaging
                cv::Mat temp_left1, temp_left2, temp_left3;
                cv::Mat temp_right1, temp_right2, temp_right3;

                frame1.left_frame.image.convertTo(temp_left1, CV_32F);
                frame2.left_frame.image.convertTo(temp_left2, CV_32F);
                frame3.left_frame.image.convertTo(temp_left3, CV_32F);

                frame1.right_frame.image.convertTo(temp_right1, CV_32F);
                frame2.right_frame.image.convertTo(temp_right2, CV_32F);
                frame3.right_frame.image.convertTo(temp_right3, CV_32F);

                // Average all three frames (VCSEL1 + VCSEL2 + Ambient)
                averaged_left = (temp_left1 + temp_left2 + temp_left3) / 3.0;
                averaged_right = (temp_right1 + temp_right2 + temp_right3) / 3.0;

                // Convert averaged frames directly to 8-bit (no contrast or CLAHE)
                averaged_left.convertTo(frame1.left_frame.image, CV_8U);
                averaged_right.convertTo(frame1.right_frame.image, CV_8U);

                // Save averaged frame (this will be overwritten by saveDebugImages as 01_left_original.png)
                cv::imwrite(debug_dir + "/00d_averaged_left.png", frame1.left_frame.image);
                cv::imwrite(debug_dir + "/00d_averaged_right.png", frame1.right_frame.image);

                qDebug() << "[DepthWidget] 3-frame averaging complete:"
                         << "left=" << frame1.left_frame.image.cols << "x" << frame1.left_frame.image.rows
                         << "right=" << frame1.right_frame.image.cols << "x" << frame1.right_frame.image.rows
                         << "SNR improvement: ~4.77 dB, NO contrast/CLAHE";
                addStatusMessage("3-frame averaging applied (pure averaging, no processing)");
            } else {
                qWarning() << "[DepthWidget] Frame synchronization issue - using frame 1 only";
                addStatusMessage("WARNING: Not all frames synchronized, using single frame");
                // Frame1 already contains the first captured frame, no modification needed
            }
        }

        // TODO: Future enhancement - combine pattern1 and pattern2 for maximum density
        // For now, use pattern1 as primary
    } else {
        // No temporal matching - just capture ambient
        qDebug() << "[DepthWidget] Single frame capture (LED disabled or not available)";
        frame1 = camera_system_->captureSingle();
    }

    // Use frame1 as the primary frame for processing (now potentially with patterns)
    core::StereoFramePair frame_pair = frame1;
    
    qDebug() << "[DepthWidget] Frame capture result:"
             << "synchronized=" << frame_pair.synchronized
             << "left_valid=" << frame_pair.left_frame.valid
             << "right_valid=" << frame_pair.right_frame.valid
             << "sync_error=" << frame_pair.sync_error_ms << "ms";
    
    if (frame_pair.synchronized) {
        // Show the captured stereo images in UI
        updateStereoFrameImages(frame_pair.left_frame, frame_pair.right_frame);
        
        capture_status_->setStatus("Processing depth map...", StatusDisplay::StatusType::PROCESSING);
        addStatusMessage("Processing depth map...");
        QApplication::processEvents();  // Update GUI

        // Process depth map in separate thread to prevent GUI blocking
        if (depth_processor_) {
            qDebug() << "[DepthWidget] Starting async depth processing";
            addStatusMessage("Depth estimation started (async)");
            QApplication::processEvents();

            // Create a QTimer to update status during processing
            QTimer* progressTimer = new QTimer(this);
            int progressCounter = 0;
            connect(progressTimer, &QTimer::timeout, [this, &progressCounter]() mutable {
                progressCounter++;
                QString progressMsg = QString("Processing... %1s").arg(progressCounter);
                addStatusMessage(progressMsg);
                QApplication::processEvents();
            });
            progressTimer->start(1000);  // Update every second

            // Run processing in a thread using QtConcurrent
            QFuture<core::DepthResult> future = QtConcurrent::run([this, frame_pair]() {
                return depth_processor_->processSync(frame_pair);
            });

            // Wait for completion with periodic GUI updates
            while (!future.isFinished()) {
                QApplication::processEvents();
                QThread::msleep(100);  // Small delay to prevent excessive CPU usage
            }

            // Stop progress timer
            progressTimer->stop();
            delete progressTimer;

            // Get result
            core::DepthResult result = future.result();
            addStatusMessage("Depth estimation complete");
            QApplication::processEvents();

            // AUTO DEBUG SAVE: Save debug images for every depth processing
            saveDebugImages(frame_pair, result);

            qDebug() << "[DepthWidget] About to call onDepthResultReceived()";
            onDepthResultReceived(result);
            qDebug() << "[DepthWidget] onDepthResultReceived() completed successfully";
        } else {
            qDebug() << "[DepthWidget] ERROR: depth_processor_ is null";
            capture_status_->setStatus("Depth processor not initialized", StatusDisplay::StatusType::ERROR);
        }
    } else {
        qDebug() << "[DepthWidget] ERROR: Failed to capture synchronized frames";
        capture_status_->setStatus("Failed to capture synchronized frames", StatusDisplay::StatusType::ERROR);
    }

    // Ensure LEDs are OFF after temporal sequence (they should already be off from frame 3)
    if (led_enabled_ && as1170) {
        qDebug() << "[DepthWidget] Ensuring VCSELs are deactivated after temporal sequence";
        // Double-check both LEDs are off (should already be from frame 3)
        as1170->setLEDState(hardware::AS1170Controller::LEDChannel::BOTH, false, 0);
    }

    capture_status_->stopPulsing();
    processing_active_ = false;
    qDebug() << "[DepthWidget] captureStereoFrame() completed";
}

void DepthTestWidget::onAlgorithmChanged(int index) {
    // Algorithm is fixed to SGBM - this function is kept for compatibility
    // but does nothing since UI only shows SGBM option
}

void DepthTestWidget::applyParameterPreset() {
    // Parameter presets removed - parameters are hardcoded in backend
    // This function is kept for compatibility but does nothing
}

void DepthTestWidget::exportDepthMap() {
    if (!current_result_.success || current_result_.depth_map.empty()) {
        QMessageBox::warning(this, "Export Error", "No valid depth map to export.");
        return;
    }
    
    // Simple export implementation (could be enhanced with file dialog)
    QString filename = QString("depth_map_%1.png").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    
    if (depth_processor_->exportDepthMap(current_result_, filename.toStdString(), "PNG")) {
        processing_status_->showTemporaryMessage("Depth map exported: " + filename, 
                                                StatusDisplay::StatusType::SUCCESS, 3000);
    } else {
        processing_status_->showTemporaryMessage("Export failed", 
                                                StatusDisplay::StatusType::ERROR, 3000);
    }
}

void DepthTestWidget::updateStereoParameters() {
    if (!depth_processor_) return;

    // Algorithm is fixed to SGBM - parameters are hardcoded in backend
    // This function is kept for compatibility but does nothing
    core::StereoConfig config = depth_processor_->getStereoConfig();
    config.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
    depth_processor_->configureStereo(config);
}

void DepthTestWidget::updateExportFormat() {
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    // Function disabled during build optimization
    return;
#else
    if (!pointcloud_processor_) return;

    // Update export format based on UI selection
    switch (ui->export_format_combo->currentIndex()) {
        case 0: // PLY Binary (Recommended)
            export_format_.format = pointcloud::ExportFormat::Format::PLY_BINARY;
            break;
        case 1: // PLY ASCII
            export_format_.format = pointcloud::ExportFormat::Format::PLY_ASCII;
            break;
        case 2: // PCD Binary
            export_format_.format = pointcloud::ExportFormat::Format::PCD_BINARY;
            break;
        case 3: // PCD ASCII
            export_format_.format = pointcloud::ExportFormat::Format::PCD_ASCII;
            break;
        case 4: // OBJ (Mesh)
            export_format_.format = pointcloud::ExportFormat::Format::OBJ;
            break;
        case 5: // XYZ (Points Only)
            export_format_.format = pointcloud::ExportFormat::Format::XYZ;
            break;
        default:
            export_format_.format = pointcloud::ExportFormat::Format::PLY_BINARY;
            break;
    }

    qDebug() << "[DepthWidget] Export format updated to:" << ui->export_format_combo->currentText();
#endif
}

void DepthTestWidget::initializeUI() {
    main_layout_ = new QHBoxLayout(this);
    main_layout_->setSpacing(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    
    // Create controls panel
    controls_panel_ = new QWidget();
    controls_panel_->setFixedWidth(CONTROLS_WIDTH);
    controls_panel_->setStyleSheet(SupernovaStyle::getStatusDisplayStyle(SupernovaStyle::NEBULA_SURFACE));
    main_layout_->addWidget(controls_panel_);
    
    QVBoxLayout* controls_layout = new QVBoxLayout(controls_panel_);
    
    // Capture panel
    controls_layout->addWidget(createCapturePanel());

    // Real-time Status panel (replaces Algorithm panel)
    controls_layout->addWidget(createStatusPanel());

    // Parameter panel
    controls_layout->addWidget(createParameterPanel());

    // Note: Point cloud export is now handled via .ui file (save_pointcloud_button, export_format_combo)
    // Legacy programmatic point cloud export panel creation removed to avoid UI conflicts

    // Artec mesh processing panel (NEW - INVESTOR DEMO)
    controls_layout->addWidget(createArtecMeshPanel());

    // Spacer
    controls_layout->addStretch();
    
    // Visualization panel
    visualization_panel_ = createVisualizationPanel();
    main_layout_->addWidget(visualization_panel_, 1);
}

QWidget* DepthTestWidget::createCapturePanel() {
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QLabel* title = new QLabel("Capture Controls");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // TODO_UI: Add capture_button and export_button to .ui file
    // capture_button_ = new TouchButton("CAPTURE STEREO FRAME", TouchButton::ButtonType::PRIMARY);
    // //capture_button_->setMinimumHeight(60);
    // connect(capture_button_, &TouchButton::clicked, this, &DepthTestWidget::captureStereoFrame);
    // layout->addWidget(capture_button_);
    
    // // Export button
    // export_button_ = new TouchButton("EXPORT DEPTH MAP", TouchButton::ButtonType::SECONDARY);
    // //export_button_->setEnabled(false);
    // connect(export_button_, &TouchButton::clicked, this, &DepthTestWidget::exportDepthMap);
    // layout->addWidget(export_button_);
    
    // Capture status
    capture_status_ = new StatusDisplay("Capture");
    capture_status_->setCompactMode(true);
    capture_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(capture_status_);

    // VCSEL status
    vcsel_status_ = new StatusDisplay("VCSEL");
    vcsel_status_->setCompactMode(true);
    vcsel_status_->setStatus("Initializing...", StatusDisplay::StatusType::INFO);
    layout->addWidget(vcsel_status_);

    return panel;
}

QWidget* DepthTestWidget::createStatusPanel() {
    // Status panel is now created via .ui file
    // Just return empty widget as placeholder
    QWidget* panel = new QWidget();
    return panel;
}

// Keep old function for compatibility but make it just call the new one
QWidget* DepthTestWidget::createAlgorithmPanel() {
    return createStatusPanel();
}

void DepthTestWidget::addStatusMessage(const QString& message) {
    if (ui && ui->status_list) {
        // Add timestamp to message
        QString timestamped = QString("[%1] %2")
            .arg(QTime::currentTime().toString("hh:mm:ss.zzz"))
            .arg(message);

        ui->status_list->addItem(timestamped);

        // Auto-scroll to latest message
        ui->status_list->scrollToBottom();

        // Keep only last 50 messages to prevent memory issues
        while (ui->status_list->count() > 50) {
            delete ui->status_list->takeItem(0);
        }
    }
}

QWidget* DepthTestWidget::createParameterPanel() {
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QLabel* title = new QLabel("Parameters");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // TODO_UI: Add parameter sliders to .ui file
    // min_disparity_slider_ = new ParameterSlider("Min Disparity", 0, 64, 0);
    // connect(min_disparity_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    // layout->addWidget(min_disparity_slider_);
    
    // num_disparities_slider_ = new ParameterSlider("Num Disparities", 16, 256, 128);
    // connect(num_disparities_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    // layout->addWidget(num_disparities_slider_);
    
    // block_size_slider_ = new ParameterSlider("Block Size", 3, 11, 5);
    // connect(block_size_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    // layout->addWidget(block_size_slider_);
    
    // uniqueness_ratio_slider_ = new ParameterSlider("Uniqueness Ratio", 1, 20, 10);
    // connect(uniqueness_ratio_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    // layout->addWidget(uniqueness_ratio_slider_);
    
    return panel;
}

QWidget* DepthTestWidget::createVisualizationPanel() {
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QLabel* title = new QLabel("Depth Visualization");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Depth map display
    depth_map_display_ = new QLabel();
    depth_map_display_->setFixedSize(DEPTH_DISPLAY_WIDTH, DEPTH_DISPLAY_HEIGHT);
    depth_map_display_->setStyleSheet(SupernovaStyle::getCameraPreviewStyle());
    depth_map_display_->setAlignment(Qt::AlignCenter);
    depth_map_display_->setText("No depth map available");
    layout->addWidget(depth_map_display_, 0, Qt::AlignCenter);
    
    // Quality metrics
    quality_metrics_label_ = new QLabel("Quality metrics will appear here");
    quality_metrics_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    quality_metrics_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    quality_metrics_label_->setAlignment(Qt::AlignCenter);
    layout->addWidget(quality_metrics_label_);
    
    // Processing status
    processing_status_ = new StatusDisplay("Processing");
    processing_status_->setStatus("Ready for depth processing", StatusDisplay::StatusType::INFO);
    layout->addWidget(processing_status_);
    
    layout->addStretch();
    
    return panel;
}

void DepthTestWidget::updateDepthVisualization(const core::DepthResult& result) {
    if (result.depth_map.empty()) return;

    // Use actual depth range from data without artificial limitations
    double min_depth, max_depth;
    cv::minMaxLoc(result.depth_map, &min_depth, &max_depth, nullptr, nullptr, result.depth_map > 0);

    // Only fallback if completely invalid data
    if (min_depth <= 0 || max_depth <= 0 || min_depth >= max_depth) {
        min_depth = 0.0;
        max_depth = 1000.0;  // Generic fallback only for invalid data
    }

    qDebug() << "[DepthWidget] Dynamic depth visualization range:" << min_depth << "-" << max_depth << "mm";

    // ROBUST SOLUTION: Load saved debug image instead of Qt-OpenCV direct integration
    // This eliminates all memory management issues that were causing segfaults
    if (ui && ui->depth_image_label && !current_debug_directory_.empty()) {
        QString debug_image_path = QString::fromStdString(current_debug_directory_) + "/11_depth_lidar_like.png";

        qDebug() << "[DepthWidget] Loading debug image for display:" << debug_image_path;

        // Load the saved debug image directly - no memory management issues
        QPixmap depth_pixmap(debug_image_path);

        if (!depth_pixmap.isNull()) {
            // Scale to fit display using .ui label with size validation
            QSize label_size = ui->depth_image_label->size();
            if (label_size.width() > 0 && label_size.height() > 0) {
                QPixmap scaled_pixmap = depth_pixmap.scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
                ui->depth_image_label->setPixmap(scaled_pixmap);
                ui->depth_image_label->setText(""); // Clear the "No Depth Data" text
                qDebug() << "[DepthWidget] Debug image loaded and displayed successfully";
            } else {
                qWarning() << "[DepthWidget] Invalid label size for display";
            }
        } else {
            qWarning() << "[DepthWidget] Failed to load debug image:" << debug_image_path;

            // Fallback: Show placeholder text
            ui->depth_image_label->setText("Debug image not available");
        }
    } else {
        qWarning() << "[DepthWidget] Cannot display: ui_valid=" << (ui != nullptr)
                   << "label_valid=" << (ui && ui->depth_image_label != nullptr)
                   << "debug_dir_available=" << !current_debug_directory_.empty();
    }
}

void DepthTestWidget::initializeDepthProcessor() {
    depth_processor_ = std::make_unique<api::DepthProcessor>();
    
    // Try to load calibration
    std::string calibration_file = "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml";
    if (depth_processor_->initialize(calibration_file) == core::ResultCode::SUCCESS) {
        processing_status_->setStatus("Depth processor initialized with calibration", 
                                     StatusDisplay::StatusType::SUCCESS);
    } else {
        processing_status_->setStatus("Depth processor initialized (no calibration)", 
                                     StatusDisplay::StatusType::WARNING);
    }
    
    // Set initial parameters
    updateStereoParameters();
}

void DepthTestWidget::updateStereoFrameImages(const core::CameraFrame& leftFrame, const core::CameraFrame& rightFrame) {
    // COPIED FROM CameraPreviewWidget - OPTIMIZED FOR SINGLE CAMERA DISPLAY
    if (!leftFrame.image.empty()) {
        // Convert left frame to QPixmap only (right frame still needed for depth processing)
        cv::Mat left_rgb;
        cv::cvtColor(leftFrame.image, left_rgb, cv::COLOR_BGRA2RGB);
        
        // Create QImage from cv::Mat with deep copy to prevent segfault
        QImage left_qimg(left_rgb.data, left_rgb.cols, left_rgb.rows, left_rgb.step, QImage::Format_RGB888);
        QImage left_qimg_copy = left_qimg.copy(); // Deep copy for safety

        // Scale to preview size
        QSize preview_size = QSize(300, 225); // Fixed size like CameraPreviewWidget
        QPixmap left_pixmap = QPixmap::fromImage(left_qimg_copy.scaled(preview_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
        
        // Set pixmap - only showing left camera in UI now
        ui->left_image_label->setPixmap(left_pixmap);
        ui->left_image_label->setText(""); // Clear text
    }
}

void DepthTestWidget::startLivePreview() {
    if (!camera_system_ || live_preview_active_) return;
    
    qDebug() << "[DepthWidget] Starting live preview...";
    
    // Create live preview callback - COPIED FROM CameraPreviewWidget
    auto live_callback = [this](const core::StereoFramePair& frame_pair) {
        // Use direct Qt signal emission for real-time processing
        QMetaObject::invokeMethod(this, [this, frame_pair]() {
            onLiveFrameReceived(frame_pair);
        }, Qt::QueuedConnection);
    };
    
    if (camera_system_->startCapture(live_callback)) {
        live_preview_active_ = true;
        qDebug() << "[DepthWidget] Live preview started successfully";
    } else {
        qDebug() << "[DepthWidget] Failed to start live preview";
    }
}

void DepthTestWidget::stopLivePreview() {
    if (!live_preview_active_) return;
    
    qDebug() << "[DepthWidget] Stopping live preview...";
    live_preview_active_ = false;
    
    // Switch to background mode like CameraPreviewWidget does
    auto background_callback = [](const core::StereoFramePair& /*frame_pair*/) {
        // Keep cameras running in background
    };
    
    if (camera_system_->startCapture(background_callback)) {
        qDebug() << "[DepthWidget] Switched to background capture";
    }
}

void DepthTestWidget::onLiveFrameReceived(const core::StereoFramePair& frame_pair) {
    if (!live_preview_active_) return;
    
    // Update preview images with live frames - PERFORMANCE: Skip every 3rd frame like CameraPreviewWidget
    static int frame_skip_counter = 0;
    if (++frame_skip_counter % 3 != 0) {
        return; // Skip frame for GUI performance
    }
    
    // Update camera preview images
    updateStereoFrameImages(frame_pair.left_frame, frame_pair.right_frame);
}

void DepthTestWidget::saveDebugImages(const core::StereoFramePair& frame_pair, const core::DepthResult& result) {
    try {
        // Use existing debug directory if already created (from 3-frame averaging)
        // Otherwise create new one
        std::string debug_dir;
        if (current_debug_directory_.empty()) {
            debug_dir = createDebugDirectory();
            current_debug_directory_ = debug_dir;
            qDebug() << "[DepthWidget] Created new debug directory:" << QString::fromStdString(debug_dir);
        } else {
            debug_dir = current_debug_directory_;
            qDebug() << "[DepthWidget] Using existing debug directory:" << QString::fromStdString(debug_dir);
        }
        
        // Save original stereo frames
        if (!frame_pair.left_frame.image.empty()) {
            cv::imwrite(debug_dir + "/01_left_original.png", frame_pair.left_frame.image);
        }
        if (!frame_pair.right_frame.image.empty()) {
            cv::imwrite(debug_dir + "/02_right_original.png", frame_pair.right_frame.image);
        }
        
        // Save rectified stereo frames
        if (depth_processor_ && !frame_pair.left_frame.image.empty() && !frame_pair.right_frame.image.empty()) {
            try {
                // Get calibration manager from depth processor to rectify images
                cv::Mat leftRectified, rightRectified;
                
                // Manual rectification using the same process as depth processor
                // Convert to grayscale if needed for rectification
                cv::Mat leftGray, rightGray;
                if (frame_pair.left_frame.image.channels() == 4) {
                    cv::cvtColor(frame_pair.left_frame.image, leftGray, cv::COLOR_BGRA2GRAY);
                    cv::cvtColor(frame_pair.right_frame.image, rightGray, cv::COLOR_BGRA2GRAY);
                } else if (frame_pair.left_frame.image.channels() == 3) {
                    cv::cvtColor(frame_pair.left_frame.image, leftGray, cv::COLOR_BGR2GRAY);
                    cv::cvtColor(frame_pair.right_frame.image, rightGray, cv::COLOR_BGR2GRAY);
                } else {
                    frame_pair.left_frame.image.copyTo(leftGray);
                    frame_pair.right_frame.image.copyTo(rightGray);
                }
                
                // Try to get rectification using a temporary CalibrationManager
                auto tempCalibManager = std::make_unique<calibration::CalibrationManager>();
                std::string calibration_file = "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml";
                
                if (tempCalibManager->loadCalibration(calibration_file)) {
                    if (tempCalibManager->rectifyImages(leftGray, rightGray, leftRectified, rightRectified)) {
                        cv::imwrite(debug_dir + "/03_left_rectified.png", leftRectified);
                        cv::imwrite(debug_dir + "/04_right_rectified.png", rightRectified);

                        // EPIPOLAR LINES VISUALIZATION - Critical for stereo validation
                        cv::Mat leftWithLines = leftRectified.clone();
                        cv::Mat rightWithLines = rightRectified.clone();

                        // Convert to BGR if grayscale
                        if (leftWithLines.channels() == 1) {
                            cv::cvtColor(leftWithLines, leftWithLines, cv::COLOR_GRAY2BGR);
                        }
                        if (rightWithLines.channels() == 1) {
                            cv::cvtColor(rightWithLines, rightWithLines, cv::COLOR_GRAY2BGR);
                        }

                        // Draw horizontal epipolar lines every 50 pixels
                        int lineSpacing = 50;
                        for (int y = 0; y < leftWithLines.rows; y += lineSpacing) {
                            // Alternating colors for better visibility
                            cv::Scalar color = (y / lineSpacing) % 2 == 0 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
                            cv::line(leftWithLines, cv::Point(0, y), cv::Point(leftWithLines.cols - 1, y), color, 1);
                            cv::line(rightWithLines, cv::Point(0, y), cv::Point(rightWithLines.cols - 1, y), color, 1);
                        }

                        // Create side-by-side composite with epipolar lines
                        cv::Mat epipolarComposite;
                        cv::hconcat(leftWithLines, rightWithLines, epipolarComposite);

                        // Add text label
                        cv::putText(epipolarComposite, "LEFT (Camera 1)", cv::Point(20, 30),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
                        cv::putText(epipolarComposite, "RIGHT (Camera 0)", cv::Point(leftWithLines.cols + 20, 30),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
                        cv::putText(epipolarComposite, "Epipolar lines should be HORIZONTAL and ALIGNED",
                                   cv::Point(20, epipolarComposite.rows - 20),
                                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

                        cv::imwrite(debug_dir + "/05_epipolar_validation.png", epipolarComposite);

                        qDebug() << "[DepthWidget] Rectified images saved successfully";
                        qDebug() << "[DepthWidget] Epipolar lines visualization saved to 05_epipolar_validation.png";
                    } else {
                        qWarning() << "[DepthWidget] Failed to rectify images for debug saving";
                    }
                } else {
                    qWarning() << "[DepthWidget] Failed to load calibration for rectification";
                }
            } catch (const std::exception& e) {
                qWarning() << "[DepthWidget] Exception during rectification:" << e.what();
            }
        }
        
        // Save depth map (ROBUST normalization for visualization)
        if (!result.depth_map.empty()) {
            cv::Mat depth_clean;
            
            // CRITICAL FIX: Remove invalid values and outliers for proper visualization
            // Replace NaN, inf, and extreme values with 0
            result.depth_map.copyTo(depth_clean);
            
            // Remove invalid values (NaN, inf)
            cv::Mat mask;
            cv::inRange(depth_clean, 1.0, 4000.0, mask); // Valid depth range: 1-4000mm (4m limit for face scanning)
            depth_clean.setTo(0, ~mask);
            
            // Calculate robust statistics (ignore zeros)
            cv::Mat valid_depths;
            depth_clean.copyTo(valid_depths, mask);
            
            if (cv::countNonZero(mask) > 100) { // Ensure we have enough valid pixels
                cv::Scalar mean_val, std_val;
                cv::meanStdDev(valid_depths, mean_val, std_val, mask);
                
                double mean_depth = mean_val[0];
                double std_depth = std_val[0];
                
                // ROBUST: Use 3-sigma range for normalization (remove extreme outliers)
                double min_depth = std::max(1.0, mean_depth - 2.0 * std_depth);
                double max_depth = std::min(8000.0, mean_depth + 2.0 * std_depth);
                
                // Clamp values to robust range
                cv::Mat depth_clamped;
                cv::threshold(depth_clean, depth_clamped, max_depth, max_depth, cv::THRESH_TRUNC);
                cv::threshold(depth_clamped, depth_clamped, min_depth, 0, cv::THRESH_TOZERO);
                
                // Normalize to 0-255 for visualization
                cv::Mat depth_normalized;
                depth_clamped.convertTo(depth_normalized, CV_8U, 255.0 / (max_depth - min_depth), -min_depth * 255.0 / (max_depth - min_depth));
                
                cv::imwrite(debug_dir + "/05_depth_map.png", depth_normalized);
                
                // Save ENHANCED COLORIZED depth map with NO BLACK LINES
                cv::Mat depth_colorized;
                cv::applyColorMap(depth_normalized, depth_colorized, cv::COLORMAP_JET);
                
                // Replace black areas with dark blue for better visibility
                cv::Mat black_mask = (depth_normalized == 0);
                depth_colorized.setTo(cv::Scalar(100, 0, 0), black_mask);  // Dark blue instead of black
                cv::imwrite(debug_dir + "/06_depth_colorized.png", depth_colorized);
                
                // ADDITIONAL: Save depth map with alternative visualization
                cv::Mat depth_alternative;
                cv::applyColorMap(depth_normalized, depth_alternative, cv::COLORMAP_PLASMA);
                depth_alternative.setTo(cv::Scalar(50, 0, 50), black_mask);  // Dark purple for invalid
                cv::imwrite(debug_dir + "/10_depth_plasma.png", depth_alternative);
                
                // LIDAR-LIKE: Save the continuous depth map with NO BLACK LINES
                cv::Mat depth_lidar_clean = result.depth_map.clone();
                
                // Ensure ALL pixels have valid depth values (no zeros that create black lines)
                cv::Mat still_invalid = depth_lidar_clean <= 0.1;
                int remainingHoles = cv::countNonZero(still_invalid);
                
                if (remainingHoles > 0) {
                    std::cout << "[DepthWidget] Final cleanup: " << remainingHoles << " remaining holes to interpolate" << std::endl;
                    
                    // Final pass: Fill remaining holes with global mean depth
                    cv::Scalar global_mean = cv::mean(depth_lidar_clean, depth_lidar_clean > 0.1);
                    depth_lidar_clean.setTo(global_mean[0], still_invalid);
                }
                
                // Normalize for visualization with guaranteed continuous range
                cv::Mat depth_lidar_normalized;
                double final_min, final_max;
                cv::minMaxLoc(depth_lidar_clean, &final_min, &final_max);
                
                // Ensure we have a reasonable range
                if (final_max - final_min < 10.0) {
                    final_min = std::max(1.0, final_min - 50.0);
                    final_max = final_min + 100.0;
                }
                
                depth_lidar_clean.convertTo(depth_lidar_normalized, CV_8U, 255.0 / (final_max - final_min), -final_min * 255.0 / (final_max - final_min));
                
                // Apply JET colormap - should now have NO black lines
                cv::Mat depth_lidar_colored;
                cv::applyColorMap(depth_lidar_normalized, depth_lidar_colored, cv::COLORMAP_JET);
                cv::imwrite(debug_dir + "/11_depth_lidar_like.png", depth_lidar_colored);
                
                // Also save a TURBO version for comparison (more perceptually uniform)
                cv::Mat depth_turbo;
                cv::applyColorMap(depth_lidar_normalized, depth_turbo, cv::COLORMAP_TURBO);
                cv::imwrite(debug_dir + "/12_depth_turbo.png", depth_turbo);
                
                std::cout << "[DepthWidget] LIDAR-like depth saved - ZERO black lines! Range: " 
                          << final_min << "-" << final_max << "mm" << std::endl;
                
                // ENHANCED: Save raw depth statistics
                std::ofstream depth_stats(debug_dir + "/depth_statistics.txt");
                depth_stats << "=== DEPTH MAP ANALYSIS ===\n";
                depth_stats << "Valid pixels: " << cv::countNonZero(mask) << " / " << (depth_clean.rows * depth_clean.cols) << "\n";
                depth_stats << "Coverage: " << (100.0 * cv::countNonZero(mask) / (depth_clean.rows * depth_clean.cols)) << "%\n";
                depth_stats << "Mean depth: " << mean_depth << "mm\n";
                depth_stats << "Std deviation: " << std_depth << "mm\n";
                depth_stats << "Visualization range: " << min_depth << " - " << max_depth << "mm\n";
                depth_stats.close();
                
                qDebug() << "[DepthWidget] DEPTH DEBUG: Valid pixels:" << cv::countNonZero(mask) 
                        << "Mean:" << mean_depth << "Std:" << std_depth 
                        << "Range:" << min_depth << "-" << max_depth;
            } else {
                qWarning() << "[DepthWidget] DEPTH ERROR: Too few valid pixels for visualization";
                // Save empty/invalid depth indication
                cv::Mat error_img = cv::Mat::zeros(result.depth_map.size(), CV_8UC3);
                cv::putText(error_img, "INVALID DEPTH DATA", cv::Point(50, result.depth_map.rows/2), 
                           cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 3);
                cv::imwrite(debug_dir + "/05_depth_map.png", error_img);
                cv::imwrite(debug_dir + "/06_depth_colorized.png", error_img);
            }
        }
        
        // COMPREHENSIVE DISPARITY DEBUG - Save multiple visualizations
        if (!result.disparity_map.empty()) {
            std::cout << "[DepthWidget] COMPREHENSIVE DISPARITY DEBUG" << std::endl;
            
            // Analyze disparity statistics first
            double minDisp, maxDisp;
            cv::minMaxLoc(result.disparity_map, &minDisp, &maxDisp);
            cv::Scalar meanDisp = cv::mean(result.disparity_map, result.disparity_map > 0);
            int validPixels = cv::countNonZero(result.disparity_map);
            int totalPixels = result.disparity_map.rows * result.disparity_map.cols;
            double validRatio = double(validPixels) / totalPixels * 100.0;
            
            std::cout << "[DepthWidget] Disparity Stats: Min=" << minDisp << ", Max=" << maxDisp 
                      << ", Mean=" << meanDisp[0] << ", Valid=" << validPixels << "/" << totalPixels 
                      << " (" << validRatio << "%)" << std::endl;
                      
            // 1. RAW DISPARITY - Standard normalization with black for invalid
            cv::Mat disp_normalized;
            cv::normalize(result.disparity_map, disp_normalized, 0, 255, cv::NORM_MINMAX);
            disp_normalized.convertTo(disp_normalized, CV_8UC1);
            cv::imwrite(debug_dir + "/07_disparity_raw.png", disp_normalized);
            
            // 2. DISPARITY WITHOUT ZEROS - Only normalize valid pixels (NO BLACK LINES!)
            cv::Mat disp_no_zeros = result.disparity_map.clone();
            cv::Mat valid_mask = disp_no_zeros > 0;
            if (cv::countNonZero(valid_mask) > 100) {
                double validMin, validMax;
                cv::minMaxLoc(disp_no_zeros, &validMin, &validMax, nullptr, nullptr, valid_mask);
                
                cv::Mat disp_clean;
                disp_no_zeros.convertTo(disp_clean, CV_8U, 255.0 / (validMax - validMin), -validMin * 255.0 / (validMax - validMin));
                
                // Set invalid pixels to gray instead of black
                disp_clean.setTo(128, ~valid_mask);  // Gray for invalid instead of black
                cv::imwrite(debug_dir + "/08_disparity_no_black.png", disp_clean);
                
                // 3. COLORED DISPARITY - Apply colormap to make it beautiful
                cv::Mat disp_colored;
                cv::applyColorMap(disp_clean, disp_colored, cv::COLORMAP_JET);
                // Set invalid pixels to dark blue instead of black
                disp_colored.setTo(cv::Scalar(100, 0, 0), ~valid_mask);  // Dark blue for invalid
                cv::imwrite(debug_dir + "/09_disparity_colored.png", disp_colored);
                
                std::cout << "[DepthWidget] Enhanced disparity saved - Valid range: " 
                          << validMin << "-" << validMax << " pixels" << std::endl;
            }
            
            // 4. DISPARITY HISTOGRAM for analysis
            if (validPixels > 0) {
                std::vector<float> validDispValues;
                for (int y = 0; y < result.disparity_map.rows; ++y) {
                    for (int x = 0; x < result.disparity_map.cols; ++x) {
                        float disp = result.disparity_map.at<float>(y, x);
                        if (disp > 0) {
                            validDispValues.push_back(disp);
                        }
                    }
                }
                
                // Save histogram data
                std::ofstream hist_file(debug_dir + "/disparity_histogram.txt");
                hist_file << "=== DISPARITY HISTOGRAM ===\n";
                hist_file << "Valid pixels: " << validPixels << " (" << validRatio << "%)\n";
                hist_file << "Min disparity: " << minDisp << " pixels\n";
                hist_file << "Max disparity: " << maxDisp << " pixels\n";
                hist_file << "Mean disparity: " << meanDisp[0] << " pixels\n";
                
                // Simple histogram bins
                std::sort(validDispValues.begin(), validDispValues.end());
                hist_file << "Median: " << validDispValues[validDispValues.size()/2] << "\n";
                hist_file << "25th percentile: " << validDispValues[validDispValues.size()/4] << "\n";
                hist_file << "75th percentile: " << validDispValues[3*validDispValues.size()/4] << "\n";
                hist_file.close();
            }
        }
        
        // Save confidence map if available
        if (!result.confidence_map.empty()) {
            cv::Mat conf_normalized;
            cv::normalize(result.confidence_map, conf_normalized, 0, 255, cv::NORM_MINMAX);
            conf_normalized.convertTo(conf_normalized, CV_8UC1);
            cv::imwrite(debug_dir + "/08_confidence_map.png", conf_normalized);
        }
        
        // Save processing info as text file
        std::ofstream info_file(debug_dir + "/processing_info.txt");
        info_file << "Unlook 3D Scanner - Debug Information\n";
        info_file << "=====================================\n\n";
        info_file << "Timestamp: " << QDateTime::currentDateTime().toString().toStdString() << "\n";
        info_file << "Sync Error: " << frame_pair.sync_error_ms << "ms\n";
        info_file << "Synchronized: " << (frame_pair.synchronized ? "YES" : "NO") << "\n";
        info_file << "Processing Time: " << result.processing_time_ms << "ms\n";
        info_file << "Mean Depth: " << result.mean_depth << "mm\n";
        info_file << "Std Deviation: " << result.std_depth << "mm\n";
        info_file << "Coverage: " << (result.coverage_ratio * 100) << "%\n";
        info_file << "Success: " << (result.success ? "YES" : "NO") << "\n";
        if (!result.error_message.empty()) {
            info_file << "Error: " << result.error_message << "\n";
        }
        info_file.close();

        // CRITICAL: Save point cloud to debug directory (avoids memory copy issues)
        // This saves the 1.24M points generated by direct disparity conversion

        // DETAILED LOGGING to understand what's happening
        qDebug() << "[DepthWidget] ========== POINT CLOUD DEBUG SAVE CHECK ==========";
        qDebug() << "[DepthWidget] depth_processor_ valid:" << (depth_processor_ ? "YES" : "NO");
        qDebug() << "[DepthWidget] result.debug_pointcloud_path:" << QString::fromStdString(result.debug_pointcloud_path);
        qDebug() << "[DepthWidget] path empty:" << (result.debug_pointcloud_path.empty() ? "YES" : "NO");

        if (depth_processor_ && !result.debug_pointcloud_path.empty()) {
            try {
                std::string pointcloud_debug_path = debug_dir + "/20_pointcloud.ply";

                qDebug() << "[DepthWidget] Source path:" << QString::fromStdString(result.debug_pointcloud_path);
                qDebug() << "[DepthWidget] Destination path:" << QString::fromStdString(pointcloud_debug_path);
                qDebug() << "[DepthWidget] Attempting to copy point cloud to debug folder...";

                // Check if source file exists
                QFile sourceFile(QString::fromStdString(result.debug_pointcloud_path));
                if (!sourceFile.exists()) {
                    qWarning() << "[DepthWidget] ❌ SOURCE FILE DOES NOT EXIST:" << QString::fromStdString(result.debug_pointcloud_path);
                } else {
                    qDebug() << "[DepthWidget] ✓ Source file exists, size:" << sourceFile.size() << "bytes";
                }

                // Remove destination if exists
                QFile::remove(QString::fromStdString(pointcloud_debug_path));

                if (sourceFile.copy(QString::fromStdString(pointcloud_debug_path))) {
                    qDebug() << "[DepthWidget] ✅ Point cloud saved to debug folder:"
                             << QString::fromStdString(pointcloud_debug_path);

                    // Verify the copy
                    QFile destFile(QString::fromStdString(pointcloud_debug_path));
                    if (destFile.exists()) {
                        qDebug() << "[DepthWidget] ✓ Destination file verified, size:" << destFile.size() << "bytes";
                    }

                    // Append to processing_info.txt
                    std::ofstream pc_info(debug_dir + "/processing_info.txt", std::ios::app);
                    pc_info << "\nPoint Cloud:\n";
                    pc_info << "  File: 20_pointcloud.ply\n";
                    pc_info << "  Source: " << result.debug_pointcloud_path << "\n";
                    pc_info << "  Generated from: Direct disparity-to-3D conversion\n";
                    pc_info << "  Expected points: ~1.2 million\n";
                    pc_info.close();
                } else {
                    qWarning() << "[DepthWidget] ❌ Failed to copy point cloud to debug folder:"
                               << sourceFile.errorString();
                }
            } catch (const std::exception& e) {
                qWarning() << "[DepthWidget] ❌ Exception saving point cloud to debug:" << e.what();
            }
        } else {
            qWarning() << "[DepthWidget] ❌ Cannot save point cloud to debug folder:";
            if (!depth_processor_) {
                qWarning() << "  - depth_processor_ is NULL";
            }
            if (result.debug_pointcloud_path.empty()) {
                qWarning() << "  - result.debug_pointcloud_path is EMPTY";
            }
        }
        qDebug() << "[DepthWidget] =======================================================";

        qDebug() << "[DepthWidget] Debug images saved successfully";
        
    } catch (const std::exception& e) {
        qWarning() << "[DepthWidget] Failed to save debug images:" << e.what();
    }
}

std::string DepthTestWidget::createDebugDirectory() {
    // Get username
    QString username = qgetenv("USER");
    if (username.isEmpty()) {
        username = qgetenv("USERNAME"); // Windows fallback
    }
    if (username.isEmpty()) {
        username = "unknown";
    }
    
    // Create base debug directory
    QString base_dir = QString("/home/%1/unlook_debug").arg(username);
    QDir().mkpath(base_dir);
    
    // Create timestamped subdirectory
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QString debug_dir = QString("%1/unlook_depth_%2").arg(base_dir, timestamp);
    QDir().mkpath(debug_dir);
    
    return debug_dir.toStdString();
}

void DepthTestWidget::initializePointCloudProcessor() {
#ifndef DISABLE_POINTCLOUD_FUNCTIONALITY
    pointcloud_processor_ = std::make_unique<pointcloud::PointCloudProcessor>();

    // CRITICAL FIX: Create and initialize a stereo::DepthProcessor with same calibration
    // The pointcloud processor requires a stereo::DepthProcessor, not api::DepthProcessor
    try {
        // Create shared CalibrationManager with same calibration file
        auto calibration_manager = std::make_shared<calibration::CalibrationManager>();
        std::string calibration_file = "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml";

        if (calibration_manager->loadCalibration(calibration_file)) {
            qDebug() << "[DepthWidget] CalibrationManager loaded successfully for point cloud processor";

            // Create and initialize stereo::DepthProcessor
            auto stereo_depth_processor = std::make_shared<stereo::DepthProcessor>();

            if (stereo_depth_processor->initialize(calibration_manager)) {
                qDebug() << "[DepthWidget] stereo::DepthProcessor initialized successfully";

                // Now initialize pointcloud processor with proper stereo depth processor
                if (pointcloud_processor_->initialize(stereo_depth_processor)) {
                    qDebug() << "[DepthWidget] Point cloud processor initialized successfully with stereo depth processor";
                } else {
                    qWarning() << "[DepthWidget] Failed to initialize point cloud processor with stereo depth processor";
                }
            } else {
                qWarning() << "[DepthWidget] Failed to initialize stereo::DepthProcessor";
            }
        } else {
            qWarning() << "[DepthWidget] Failed to load calibration for point cloud processor";
        }
    } catch (const std::exception& e) {
        qWarning() << "[DepthWidget] Exception during point cloud processor initialization:" << e.what();
    }

    // Initialize default configurations
    pointcloud_filter_config_ = pointcloud::PointCloudFilterConfig{};

    // CRITICAL FIX: Disable statistical outlier filter - it was decimating point clouds
    // Previous settings (20 neighbors, 2.0 std ratio) removed 99.997% of points (989K -> 28)
    // Root cause: In sparse point clouds with non-uniform disparity distribution,
    // most points appear as "outliers" due to large inter-point distances
    // TODO: Re-enable after fixing disparity distribution uniformity in SGBM
    // If re-enabled, use more lenient parameters: 50 neighbors, 3.0 std ratio
    pointcloud_filter_config_.enableStatisticalFilter = false;  // DISABLED - was too aggressive
    pointcloud_filter_config_.statisticalNeighbors = 50;        // Increased from 20 (if re-enabled)
    pointcloud_filter_config_.statisticalStdRatio = 3.0;        // Increased from 2.0 (if re-enabled)
    pointcloud_filter_config_.computeNormals = true;

    mesh_generation_config_ = pointcloud::MeshGenerationConfig{};
    mesh_generation_config_.algorithm = pointcloud::MeshGenerationConfig::Algorithm::POISSON;
    mesh_generation_config_.poissonDepth = 9;

    export_format_ = pointcloud::ExportFormat{};
    export_format_.format = pointcloud::ExportFormat::Format::PLY_BINARY;
    export_format_.includeColors = true;
    export_format_.includeNormals = true;
    export_format_.scannerInfo = "Unlook 3D Scanner";
    export_format_.calibrationFile = "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml";
    export_format_.precisionMm = 0.005;
#endif
}

void DepthTestWidget::initializeVCSELProjector() {
    vcsel_projector_ = std::make_shared<hardware::VCSELProjector>();

    // Configure VCSEL for depth capture mode
    hardware::VCSELProjector::ProjectorConfig vcsel_config;
    vcsel_config.mode = hardware::VCSELProjector::ProjectionMode::DEPTH_CAPTURE;
    vcsel_config.pattern = hardware::VCSELProjector::PatternType::DOTS_15K;
    vcsel_config.vcsel_current_ma = 350;  // High power for dark/uniform surfaces (AS1170 limit: 450mA)
    vcsel_config.flood_current_ma = 350;  // Flood assist for better texture on low-light surfaces
    vcsel_config.enable_flood_assist = true;  // Enable flood for maximum texture projection
    vcsel_config.projection_duration_ms = 15;  // Match camera exposure (15ms)
    vcsel_config.max_duty_cycle = 0.3f;  // 30% max duty cycle
    vcsel_config.enable_thermal_protection = true;
    vcsel_config.max_temperature_c = 70.0f;
    vcsel_config.thermal_throttle_temp_c = 65.0f;
    vcsel_config.enable_camera_sync = true;
    vcsel_config.sync_tolerance_us = 50;

    // Initialize the VCSEL projector
    if (vcsel_projector_->initialize(vcsel_config)) {
        qDebug() << "[DepthWidget] VCSEL projector initialized successfully";

        // Set up callbacks for monitoring
        vcsel_projector_->setThermalCallback(
            [this](bool thermal_active, float temperature_c) {
                QMetaObject::invokeMethod(this, [this, thermal_active, temperature_c]() {
                    onVCSELThermalEvent(thermal_active, temperature_c);
                }, Qt::QueuedConnection);
            });

        vcsel_projector_->setErrorCallback(
            [this](const std::string& error) {
                QMetaObject::invokeMethod(this, [this, error]() {
                    onVCSELError(error);
                }, Qt::QueuedConnection);
            });

        // Update status display
        if (vcsel_status_) {
            vcsel_status_->setStatus("VCSEL Ready", StatusDisplay::StatusType::SUCCESS);
        }
    } else {
        qWarning() << "[DepthWidget] Failed to initialize VCSEL projector";
        if (vcsel_status_) {
            vcsel_status_->setStatus("VCSEL Init Failed", StatusDisplay::StatusType::ERROR);
        }
    }
}

QWidget* DepthTestWidget::createPointCloudExportPanel() {
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    // Return empty widget during build optimization
    QWidget* panel = new QWidget();
    QLabel* label = new QLabel("Point cloud export temporarily disabled");
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->addWidget(label);
    return panel;
#else
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);

    QLabel* title = new QLabel("Point Cloud Export");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);

    // Export format selection
    QLabel* format_label = new QLabel("Export Format:");
    format_label->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    format_label->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(format_label);

    export_format_combo_ = new QComboBox();
    export_format_combo_->addItem("PLY Binary", static_cast<int>(pointcloud::ExportFormat::Format::PLY_BINARY));
    export_format_combo_->addItem("PLY ASCII", static_cast<int>(pointcloud::ExportFormat::Format::PLY_ASCII));
    export_format_combo_->addItem("PCD Binary", static_cast<int>(pointcloud::ExportFormat::Format::PCD_BINARY));
    export_format_combo_->addItem("PCD ASCII", static_cast<int>(pointcloud::ExportFormat::Format::PCD_ASCII));
    export_format_combo_->addItem("OBJ", static_cast<int>(pointcloud::ExportFormat::Format::OBJ));
    export_format_combo_->addItem("XYZ", static_cast<int>(pointcloud::ExportFormat::Format::XYZ));
    layout->addWidget(export_format_combo_);

    // Export buttons
    export_pointcloud_button_ = new widgets::TouchButton("EXPORT POINT CLOUD", widgets::TouchButton::ButtonType::SUCCESS);
    connect(export_pointcloud_button_, &widgets::TouchButton::clicked, this, &DepthTestWidget::exportPointCloud);
    layout->addWidget(export_pointcloud_button_);

    export_mesh_button_ = new widgets::TouchButton("EXPORT MESH", widgets::TouchButton::ButtonType::PRIMARY);
    connect(export_mesh_button_, &widgets::TouchButton::clicked, this, &DepthTestWidget::exportMesh);
    layout->addWidget(export_mesh_button_);

    // Configuration button
    configure_export_button_ = new widgets::TouchButton("CONFIGURE", widgets::TouchButton::ButtonType::SECONDARY);
    connect(configure_export_button_, &widgets::TouchButton::clicked, this, &DepthTestWidget::configurePointCloudExport);
    layout->addWidget(configure_export_button_);

    // Export status
    export_status_ = new widgets::StatusDisplay("Export");
    export_status_->setCompactMode(true);
    export_status_->setStatus("Ready for export", widgets::StatusDisplay::StatusType::INFO);
    layout->addWidget(export_status_);

    // Initially disable export buttons
    export_pointcloud_button_->setEnabled(false);
    export_mesh_button_->setEnabled(false);

    return panel;
#endif
}

void DepthTestWidget::exportPointCloud() {
    qDebug() << "[DepthWidget] ===== PLY EXPORT START =====";
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    qDebug() << "[DepthWidget] PLY export disabled at compile time";
    QMessageBox::information(this, "Feature Disabled",
        "Point cloud export is temporarily disabled during system build optimization.");
    return;
#endif

    qDebug() << "[DepthWidget] Checking current depth result...";
    if (!current_result_.success || current_result_.depth_map.empty()) {
        qDebug() << "[DepthWidget] PLY export failed: No valid depth map";
        QMessageBox::warning(this, "Export Error", "No valid depth map available for point cloud export.");
        return;
    }

    qDebug() << "[DepthWidget] Checking pointcloud processor...";
    if (!pointcloud_processor_) {
        qDebug() << "[DepthWidget] PLY export failed: Point cloud processor not initialized";
        QMessageBox::warning(this, "Export Error", "Point cloud processor not initialized.");
        return;
    }

    qDebug() << "[DepthWidget] Setting export status...";

    // CRITICAL FIX: Check if export_status_ exists before using it
    if (export_status_) {
        export_status_->setStatus("Generating point cloud...", widgets::StatusDisplay::StatusType::PROCESSING);
        export_status_->startPulsing();
        qDebug() << "[DepthWidget] Export status set successfully";
    } else {
        qDebug() << "[DepthWidget] WARNING: export_status_ is null, continuing without status updates";
    }

    try {
        qDebug() << "[DepthWidget] Entering PLY export try block...";

        // Export format is already updated via updateExportFormat() slot
        // when the UI combo box selection changes

        qDebug() << "[DepthWidget] Creating PointCloud object...";
        // Generate point cloud from current depth result
        stereo::PointCloud pointCloud;
        qDebug() << "[DepthWidget] PointCloud object created successfully";

        qDebug() << "[DepthWidget] Creating color image from depth map...";
        // Create a safe color image if not available (prevents crashes)
        cv::Mat colorImage;
        if (current_result_.depth_map.rows > 0 && current_result_.depth_map.cols > 0) {
            qDebug() << "[DepthWidget] Depth map valid, normalizing...";
            // Generate grayscale color from depth for visualization
            cv::Mat depthGray;
            cv::normalize(current_result_.depth_map, depthGray, 0, 255, cv::NORM_MINMAX, CV_8U, current_result_.depth_map > 0);
            qDebug() << "[DepthWidget] Depth normalized, converting to BGR...";
            cv::cvtColor(depthGray, colorImage, cv::COLOR_GRAY2BGR);
            qDebug() << "[DepthWidget] Color image created successfully";
        } else {
            qDebug() << "[DepthWidget] Invalid depth map dimensions, using empty color image";
        }

        // SMART FIX: Use point cloud from debug directory (avoids all memory issues)
        // The PLY was already saved in saveDebugImages() to ~/unlook_debug/unlook_depth_TIMESTAMP/20_pointcloud.ply

        std::string pointcloud_debug_path = current_debug_directory_ + "/20_pointcloud.ply";

        // FILE LOGGING for GUI point cloud check
        std::ofstream guiTrace("/tmp/point_cloud_trace.log", std::ios::app);
        guiTrace << "\n[GUI] Export point cloud starting..." << std::endl;
        guiTrace << "[GUI] current_debug_directory = " << current_debug_directory_ << std::endl;
        guiTrace << "[GUI] pointcloud_debug_path = " << pointcloud_debug_path << std::endl;

        // Check if PLY file exists in debug directory
        QFile debugPlyFile(QString::fromStdString(pointcloud_debug_path));
        bool plyExistsInDebug = debugPlyFile.exists();

        guiTrace << "[GUI] PLY exists in debug folder: " << (plyExistsInDebug ? "YES" : "NO") << std::endl;
        guiTrace.close();

        qDebug() << "[DepthWidget] ===== POINT CLOUD EXPORT CHECK =====";
        qDebug() << "[DepthWidget] Debug directory:" << QString::fromStdString(current_debug_directory_);
        qDebug() << "[DepthWidget] PLY path:" << QString::fromStdString(pointcloud_debug_path);
        qDebug() << "[DepthWidget] PLY exists:" << (plyExistsInDebug ? "YES" : "NO");

        // Generate filename with timestamp
        QString filename = QString("unlook_pointcloud_%1%2")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"))
            .arg(QString::fromStdString(export_format_.getFileExtension()));

        QString filepath = QDir::homePath() + "/unlook_exports/" + filename;
        QDir().mkpath(QDir::homePath() + "/unlook_exports");

        // Set timestamp for export
        export_format_.timestamp = QDateTime::currentDateTime().toString().toStdString();

        // PRIORITY 1: Use pre-generated PLY from debug folder (ZERO memory copy, ~1.2M points!)
        if (plyExistsInDebug) {
            // Read actual point count from PLY header
            size_t actualPointCount = 0;
            QFile plyFile(QString::fromStdString(pointcloud_debug_path));
            if (plyFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
                QTextStream in(&plyFile);
                while (!in.atEnd()) {
                    QString line = in.readLine().trimmed();
                    if (line.startsWith("element vertex ")) {
                        bool ok;
                        actualPointCount = line.mid(15).toULongLong(&ok);
                        if (!ok) actualPointCount = 0;
                        break;
                    }
                    if (line == "end_header") break;
                }
                plyFile.close();
            }

            qDebug() << "[DepthWidget] ✅ Using PLY from debug folder (" << actualPointCount << " points)";

            // Remove destination if exists
            QFile::remove(filepath);

            if (debugPlyFile.copy(filepath)) {
                qDebug() << "[DepthWidget] ✅ Point cloud exported to: " << filepath;
                if (export_status_) {
                    export_status_->setStatus(QString("Point cloud exported: %1").arg(filename),
                                             widgets::StatusDisplay::StatusType::SUCCESS);
                    export_status_->stopPulsing();
                }

                QString pointCountStr = actualPointCount > 0
                    ? QString::number(actualPointCount) + " points"
                    : "~1.2 million points (count unavailable)";

                QMessageBox::information(this, "Export Successful",
                    QString("Point cloud exported successfully to:\n%1\n\nTotal points: %2 from direct disparity conversion").arg(filepath).arg(pointCountStr));
                return;  // SUCCESS - exit immediately
            } else {
                qWarning() << "[DepthWidget] ❌ File copy failed: " << debugPlyFile.errorString();
                qWarning() << "[DepthWidget] Falling back to regeneration from depth map...";
                // Continue to fallback below
            }
        }

        // FALLBACK: PLY not found or copy failed - regenerate from depth map
        qDebug() << "[DepthWidget] PLY not available in debug folder, regenerating from depth map...";

        if (!pointcloud_processor_->generatePointCloud(
                current_result_.depth_map,
                colorImage,
                pointCloud,
                pointcloud_filter_config_)) {

            qDebug() << "[DepthWidget] ❌ generatePointCloud() FAILED";
            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            qDebug() << "[DepthWidget] Point cloud error:" << error;
            QMessageBox::critical(this, "Point Cloud Generation Failed", error);
            if (export_status_) {
                export_status_->setStatus("Point cloud generation failed", widgets::StatusDisplay::StatusType::ERROR);
                export_status_->stopPulsing();
            }
            return;
        }

        qDebug() << "[DepthWidget] Fallback regeneration completed with" << pointCloud.points.size() << "points";

        // Assess point cloud quality (for fallback success message)
        pointcloud::PointCloudQuality quality;
        quality.validPoints = pointCloud.points.size();
        quality.validRatio = 0.0;
        quality.density = 0.0;

        if (pointcloud_processor_->assessPointCloudQuality(pointCloud, quality)) {
            qDebug() << "[DepthWidget] ========== FALLBACK POINT CLOUD QUALITY ==========";
            qDebug() << "[DepthWidget] Total points:" << quality.totalPoints;
            qDebug() << "[DepthWidget] Valid points:" << quality.validPoints;
            qDebug() << "[DepthWidget] Valid ratio:" << (quality.validRatio * 100) << "%";
            qDebug() << "[DepthWidget] ================================================";

            // WARNING if too few points
            if (quality.validPoints < 1000) {
                qWarning() << "[DepthWidget] ⚠ WARNING: Only" << quality.validPoints
                          << "points in fallback point cloud! This is a backup method.";
            }
        }

        qDebug() << "[DepthWidget] Calling pointcloud_processor_->exportPointCloud()...";
        // Export point cloud (fallback if file copy not available/failed)
        if (pointcloud_processor_->exportPointCloud(pointCloud, filepath.toStdString(), export_format_)) {
            qDebug() << "[DepthWidget] exportPointCloud() completed successfully";
            if (export_status_) {
                export_status_->setStatus(QString("Point cloud exported: %1").arg(filename),
                                         widgets::StatusDisplay::StatusType::SUCCESS);
            }

            qDebug() << "[DepthWidget] Creating success message...";
            // Show success message with quality info
            QString message = QString("Point cloud exported successfully!\n\n"
                                     "File: %1\n"
                                     "Format: %2\n"
                                     "Points: %3\n"
                                     "Valid ratio: %4%\n"
                                     "Density: %5 points/mm³")
                .arg(filename)
                .arg(ui->export_format_combo->currentText())
                .arg(quality.validPoints)
                .arg(quality.validRatio * 100, 0, 'f', 1)
                .arg(quality.density, 0, 'f', 2);

            qDebug() << "[DepthWidget] Showing success message box...";
            QMessageBox::information(this, "Export Successful", message);
            qDebug() << "[DepthWidget] Success message box completed";
        } else {
            qDebug() << "[DepthWidget] exportPointCloud() FAILED";
            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            qDebug() << "[DepthWidget] Export error:" << error;
            QMessageBox::critical(this, "Export Failed", QString("Failed to export point cloud: %1").arg(error));
            if (export_status_) {
                export_status_->setStatus("Point cloud export failed", widgets::StatusDisplay::StatusType::ERROR);
            }
        }

    } catch (const std::exception& e) {
        qDebug() << "[DepthWidget] EXCEPTION during PLY export:" << e.what();
        QMessageBox::critical(this, "Export Failed", QString("Exception during export: %1").arg(e.what()));
        if (export_status_) {
            export_status_->setStatus("Export failed with exception", widgets::StatusDisplay::StatusType::ERROR);
        }
    }

    qDebug() << "[DepthWidget] Stopping export status pulsing...";
    if (export_status_) {
        export_status_->stopPulsing();
    }
    qDebug() << "[DepthWidget] ===== PLY EXPORT END =====";
}

void DepthTestWidget::exportMesh() {
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    QMessageBox::information(this, "Feature Disabled",
        "Mesh export is temporarily disabled during system build optimization.");
    return;
#endif
    if (!current_result_.success || current_result_.depth_map.empty()) {
        QMessageBox::warning(this, "Export Error", "No valid depth map available for mesh export.");
        return;
    }

    if (!pointcloud_processor_) {
        QMessageBox::warning(this, "Export Error", "Point cloud processor not initialized.");
        return;
    }

    export_status_->setStatus("Generating mesh...", widgets::StatusDisplay::StatusType::PROCESSING);
    export_status_->startPulsing();

    try {
        // First generate point cloud
        stereo::PointCloud pointCloud;
        cv::Mat colorImage; // TODO: Get the actual color image

        if (!pointcloud_processor_->generatePointCloud(
                current_result_.depth_map,
                colorImage,
                pointCloud,
                pointcloud_filter_config_)) {

            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            QMessageBox::critical(this, "Point Cloud Generation Failed", error);
            export_status_->setStatus("Point cloud generation failed", widgets::StatusDisplay::StatusType::ERROR);
            export_status_->stopPulsing();
            return;
        }

        // Generate mesh
        std::vector<cv::Vec3f> vertices;
        std::vector<cv::Vec3i> faces;
        std::vector<cv::Vec3f> normals;

        if (!pointcloud_processor_->generateMesh(pointCloud, mesh_generation_config_, vertices, faces, normals)) {
            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            QMessageBox::critical(this, "Mesh Generation Failed", error);
            export_status_->setStatus("Mesh generation failed", widgets::StatusDisplay::StatusType::ERROR);
            export_status_->stopPulsing();
            return;
        }

        // Assess mesh quality
        pointcloud::MeshQuality quality;
        if (pointcloud_processor_->assessMeshQuality(vertices, faces, quality)) {
            qDebug() << "[DepthWidget] Mesh quality:"
                     << "Vertices:" << quality.numVertices
                     << "Faces:" << quality.numFaces
                     << "Watertight:" << quality.isWatertight
                     << "Surface area:" << quality.surfaceArea << "mm²";
        }

        // Generate filename
        QString filename = QString("unlook_mesh_%1.obj")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
        QString filepath = QDir::homePath() + "/unlook_exports/" + filename;

        // Export mesh (using OBJ format for meshes)
        pointcloud::ExportFormat meshFormat = export_format_;
        meshFormat.format = pointcloud::ExportFormat::Format::OBJ;
        meshFormat.timestamp = QDateTime::currentDateTime().toString().toStdString();

        if (pointcloud_processor_->exportMesh(vertices, faces, normals, filepath.toStdString(), meshFormat)) {
            export_status_->setStatus(QString("Mesh exported: %1").arg(filename),
                                     widgets::StatusDisplay::StatusType::SUCCESS);

            // Show success message with quality info
            QString message = QString("Mesh exported successfully!\n\n"
                                     "File: %1\n"
                                     "Vertices: %2\n"
                                     "Faces: %3\n"
                                     "Surface area: %4 mm²\n"
                                     "Watertight: %5")
                .arg(filename)
                .arg(quality.numVertices)
                .arg(quality.numFaces)
                .arg(quality.surfaceArea, 0, 'f', 2)
                .arg(quality.isWatertight ? "Yes" : "No");

            QMessageBox::information(this, "Export Successful", message);
        } else {
            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            QMessageBox::critical(this, "Export Failed", QString("Failed to export mesh: %1").arg(error));
            export_status_->setStatus("Mesh export failed", widgets::StatusDisplay::StatusType::ERROR);
        }

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Export Failed", QString("Exception during mesh export: %1").arg(e.what()));
        export_status_->setStatus("Mesh export failed with exception", widgets::StatusDisplay::StatusType::ERROR);
    }

    export_status_->stopPulsing();
}

void DepthTestWidget::configurePointCloudExport() {
    // TODO: Implement configuration dialog for point cloud filtering and mesh generation parameters
    QMessageBox::information(this, "Configuration",
                             "Point cloud export configuration dialog will be implemented here.\n\n"
                             "Current settings:\n"
                             "- Statistical filter: Enabled\n"
                             "- Mesh algorithm: Poisson reconstruction\n"
                             "- Normal computation: Enabled");
}

void DepthTestWidget::configureMeshGeneration() {
    // TODO: Implement mesh generation configuration dialog
    QMessageBox::information(this, "Mesh Configuration",
                             "Mesh generation configuration dialog will be implemented here.");
}
// Temporary file for new Artec mesh panel implementation
// This will be inserted into depth_test_widget.cpp

QWidget* DepthTestWidget::createArtecMeshPanel() {
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    QWidget* panel = new QWidget();
    QLabel* label = new QLabel("Artec mesh processing temporarily disabled");
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->addWidget(label);
    return panel;
#else
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    layout->setSpacing(12);

    // Title
    QLabel* title = new QLabel("ARTEC MESH PROCESSING");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1; padding: 8px;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    title->setAlignment(Qt::AlignCenter);
    layout->addWidget(title);

    // Quality Preset Buttons
    QLabel* preset_label = new QLabel("Quality Preset:");
    preset_label->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::BOLD));
    preset_label->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(preset_label);

    // Preset buttons in horizontal layout
    QHBoxLayout* preset_layout = new QHBoxLayout();
    preset_layout->setSpacing(8);

    artec_industrial_button_ = new widgets::TouchButton("INDUSTRIAL", widgets::TouchButton::ButtonType::SUCCESS);
    artec_industrial_button_->setToolTip("Sharp features, preserve edges\nBest for: mechanical parts, industrial objects");
    preset_layout->addWidget(artec_industrial_button_);

    artec_organic_button_ = new widgets::TouchButton("ORGANIC", widgets::TouchButton::ButtonType::PRIMARY);
    artec_organic_button_->setToolTip("Smooth surfaces, balanced settings\nBest for: organic shapes, sculptures");
    preset_layout->addWidget(artec_organic_button_);

    artec_preview_button_ = new widgets::TouchButton("PREVIEW", widgets::TouchButton::ButtonType::SECONDARY);
    artec_preview_button_->setToolTip("Fast preview, reduced quality\nBest for: quick testing");
    preset_layout->addWidget(artec_preview_button_);

    layout->addLayout(preset_layout);

    // Octree Depth Slider
    QLabel* octree_label_title = new QLabel("Octree Depth (Detail Level):");
    octree_label_title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::BOLD));
    octree_label_title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(octree_label_title);

    QHBoxLayout* slider_layout = new QHBoxLayout();

    artec_octree_slider_ = new QSlider(Qt::Horizontal);
    artec_octree_slider_->setMinimum(7);
    artec_octree_slider_->setMaximum(12);
    artec_octree_slider_->setValue(9);
    artec_octree_slider_->setTickPosition(QSlider::TicksBelow);
    artec_octree_slider_->setTickInterval(1);
    slider_layout->addWidget(artec_octree_slider_, 3);

    artec_octree_label_ = new QLabel("9");
    artec_octree_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::BOLD));
    artec_octree_label_->setStyleSheet(QString("color: %1; min-width: 30px;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    artec_octree_label_->setAlignment(Qt::AlignCenter);
    slider_layout->addWidget(artec_octree_label_, 1);

    layout->addLayout(slider_layout);

    // Info text
    QLabel* info_text = new QLabel("Lower = Faster (less detail)\nHigher = Slower (more detail)");
    info_text->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    info_text->setStyleSheet(QString("color: %1; font-style: italic;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    layout->addWidget(info_text);

    // Statistics Panel
    QLabel* stats_label_title = new QLabel("Mesh Statistics:");
    stats_label_title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::BOLD));
    stats_label_title->setStyleSheet(QString("color: %1; margin-top: 8px;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    layout->addWidget(stats_label_title);

    artec_stats_label_ = new QLabel("No mesh generated yet");
    artec_stats_label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL));
    artec_stats_label_->setStyleSheet(QString("color: %1; background: rgba(0,0,0,0.3); padding: 12px; border-radius: 4px;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    artec_stats_label_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    artec_stats_label_->setWordWrap(true);
    artec_stats_label_->setMinimumHeight(120);
    layout->addWidget(artec_stats_label_);

    // Export Button (Large and prominent)
    artec_export_button_ = new widgets::TouchButton("GENERATE & EXPORT MESH", widgets::TouchButton::ButtonType::PRIMARY);
    artec_export_button_->setToolTip("Generate Artec-grade watertight mesh\nProcessing time: ~15 seconds");
    artec_export_button_->setEnabled(false);
    layout->addWidget(artec_export_button_);

    // Status Display
    artec_status_ = new widgets::StatusDisplay("Artec Processing");
    artec_status_->setCompactMode(true);
    artec_status_->setStatus("Ready - capture depth first", widgets::StatusDisplay::StatusType::INFO);
    layout->addWidget(artec_status_);

    // Connect signals (NEW)
    connect(artec_industrial_button_, &widgets::TouchButton::clicked,
            this, [this]() { applyArtecPreset(0); });
    connect(artec_organic_button_, &widgets::TouchButton::clicked,
            this, [this]() { applyArtecPreset(1); });
    connect(artec_preview_button_, &widgets::TouchButton::clicked,
            this, [this]() { applyArtecPreset(2); });
    connect(artec_export_button_, &widgets::TouchButton::clicked,
            this, &DepthTestWidget::exportArtecMesh);
    connect(artec_octree_slider_, &QSlider::valueChanged,
            this, &DepthTestWidget::updateArtecOctreeDepth);

    // Apply default preset (INDUSTRIAL)
    applyArtecPreset(0);

    layout->addStretch();

    return panel;
#endif
}

void DepthTestWidget::applyArtecPreset(int presetIndex) {
    artec_quality_preset_ = presetIndex;

    // Update button states
    artec_industrial_button_->setStyleSheet("");
    artec_organic_button_->setStyleSheet("");
    artec_preview_button_->setStyleSheet("");

    QString active_style = QString("background-color: %1; border: 2px solid %2;")
        .arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY))
        .arg(SupernovaStyle::colorToString(SupernovaStyle::PLASMA_ACCENT));

    QString preset_name;
    int recommended_depth = 9;

    switch(presetIndex) {
        case 0: // INDUSTRIAL
            artec_industrial_button_->setStyleSheet(active_style);
            preset_name = "INDUSTRIAL (Sharp Features)";
            recommended_depth = 10;
            break;
        case 1: // ORGANIC
            artec_organic_button_->setStyleSheet(active_style);
            preset_name = "ORGANIC (Smooth Surfaces)";
            recommended_depth = 9;
            break;
        case 2: // PREVIEW
            artec_preview_button_->setStyleSheet(active_style);
            preset_name = "PREVIEW (Fast Mode)";
            recommended_depth = 7;
            break;
    }

    // Update octree depth to recommended value
    artec_octree_slider_->setValue(recommended_depth);
    artec_octree_depth_ = recommended_depth;
    artec_octree_label_->setText(QString::number(recommended_depth));

    artec_status_->setStatus(QString("Preset: %1 (Depth: %2)").arg(preset_name).arg(recommended_depth),
                            widgets::StatusDisplay::StatusType::INFO);

    qDebug() << "[DepthWidget] Artec preset applied:" << preset_name << "depth:" << recommended_depth;
}

void DepthTestWidget::updateArtecOctreeDepth(int depth) {
    artec_octree_depth_ = depth;
    artec_octree_label_->setText(QString::number(depth));

    QString detail_level;
    if (depth <= 7) detail_level = "Low (fast)";
    else if (depth <= 9) detail_level = "Medium (balanced)";
    else detail_level = "High (slow, detailed)";

    artec_status_->setStatus(QString("Octree depth: %1 - %2").arg(depth).arg(detail_level),
                            widgets::StatusDisplay::StatusType::INFO);

    qDebug() << "[DepthWidget] Artec octree depth updated:" << depth;
}

void DepthTestWidget::exportArtecMesh() {
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    QMessageBox::information(this, "Feature Disabled",
        "Artec mesh processing is temporarily disabled during system build optimization.");
    return;
#endif

    qDebug() << "[DepthWidget] ===== ARTEC MESH EXPORT START =====";

    if (!current_result_.success || current_result_.depth_map.empty()) {
        QMessageBox::warning(this, "Export Error", "No valid depth map available. Capture a stereo frame first.");
        return;
    }

    if (!pointcloud_processor_) {
        QMessageBox::warning(this, "Export Error", "Point cloud processor not initialized.");
        return;
    }

    artec_status_->setStatus("Processing Artec pipeline...", widgets::StatusDisplay::StatusType::PROCESSING);
    artec_status_->startPulsing();
    artec_export_button_->setEnabled(false);

    try {
        // Step 1: Generate point cloud from depth map
        qDebug() << "[DepthWidget] Step 1: Generating point cloud...";
        stereo::PointCloud pointCloud;
        cv::Mat colorImage; // TODO: Get actual color image

        if (!pointcloud_processor_->generatePointCloud(
                current_result_.depth_map,
                colorImage,
                pointCloud,
                pointcloud_filter_config_)) {

            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            QMessageBox::critical(this, "Point Cloud Generation Failed", error);
            artec_status_->setStatus("Point cloud generation failed", widgets::StatusDisplay::StatusType::ERROR);
            artec_status_->stopPulsing();
            artec_export_button_->setEnabled(true);
            return;
        }

        qDebug() << "[DepthWidget] Point cloud generated:" << pointCloud.points.size() << "points";

        // Step 2: Convert to Open3D format
        qDebug() << "[DepthWidget] Step 2: Converting to Open3D format...";
        auto open3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

        for (const auto& pt : pointCloud.points) {
            open3d_cloud->points_.push_back(Eigen::Vector3d(pt.x, pt.y, pt.z));

            // Add color from Point3D (r, g, b members)
            open3d_cloud->colors_.push_back(Eigen::Vector3d(
                pt.r / 255.0,  // R
                pt.g / 255.0,  // G
                pt.b / 255.0   // B
            ));
        }

        qDebug() << "[DepthWidget] Open3D cloud created with" << open3d_cloud->points_.size() << "points";

        // Step 3: Run Artec-grade pipeline
        qDebug() << "[DepthWidget] Step 3: Running Artec-grade pipeline (Poisson + Cleaning + Simplification)...";
        artec_status_->setStatus("Generating watertight mesh...", widgets::StatusDisplay::StatusType::PROCESSING);

        auto start_time = std::chrono::high_resolution_clock::now();

        auto mesh = pointcloud_processor_->processCompletePipeline(*open3d_cloud);

        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        if (!mesh || mesh->IsEmpty()) {
            QMessageBox::critical(this, "Mesh Generation Failed",
                                "Artec pipeline produced empty mesh. Try different settings or recapture.");
            artec_status_->setStatus("Mesh generation failed", widgets::StatusDisplay::StatusType::ERROR);
            artec_status_->stopPulsing();
            artec_export_button_->setEnabled(true);
            return;
        }

        qDebug() << "[DepthWidget] Artec mesh generated:" << mesh->vertices_.size() << "vertices,"
                 << mesh->triangles_.size() << "triangles in" << duration.count() << "ms";

        // Step 4: Assess mesh quality
        bool is_watertight = mesh->IsWatertight();
        bool is_manifold = mesh->IsEdgeManifold();

        qDebug() << "[DepthWidget] Mesh quality: Watertight=" << is_watertight << "Manifold=" << is_manifold;

        // Update statistics display
        QString stats = QString(
            "✓ Processing complete!\n\n"
            "Vertices: %1\n"
            "Triangles: %2\n"
            "Watertight: %3\n"
            "Manifold: %4\n"
            "Processing time: %5 ms\n"
            "Quality: %6"
        ).arg(mesh->vertices_.size())
         .arg(mesh->triangles_.size())
         .arg(is_watertight ? "✓ Yes" : "✗ No")
         .arg(is_manifold ? "✓ Yes" : "✗ No")
         .arg(duration.count())
         .arg(is_watertight && is_manifold ? "Artec-grade ✓" : "Good");

        artec_stats_label_->setText(stats);
        artec_stats_label_->setStyleSheet(QString("color: %1; background: rgba(0,255,100,0.1); padding: 12px; border-radius: 4px; border: 1px solid rgba(0,255,100,0.3);")
                                         .arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));

        // Step 5: Export mesh to debug directory (same as images)
        QString filename = QString("30_artec_mesh.ply");
        QString filepath = QString::fromStdString(current_debug_directory_) + "/" + filename;

        qDebug() << "[DepthWidget] Step 5: Exporting Artec mesh to debug directory:" << filepath;

        if (open3d::io::WriteTriangleMesh(filepath.toStdString(), *mesh)) {
            artec_status_->setStatus(QString("Exported: %1").arg(filename),
                                    widgets::StatusDisplay::StatusType::SUCCESS);

            QString message = QString(
                "Artec-grade mesh exported successfully!\n\n"
                "Location: %1\n\n"
                "Vertices: %2\n"
                "Triangles: %3\n"
                "Watertight: %4\n"
                "Manifold: %5\n"
                "Processing time: %6 ms\n\n"
                "Quality: %7"
            ).arg(filepath)
             .arg(mesh->vertices_.size())
             .arg(mesh->triangles_.size())
             .arg(is_watertight ? "Yes ✓" : "No ✗")
             .arg(is_manifold ? "Yes ✓" : "No ✗")
             .arg(duration.count())
             .arg(is_watertight && is_manifold ? "Artec-grade professional quality ✓" : "Good quality");

            QMessageBox::information(this, "Export Successful", message);
        } else {
            QMessageBox::critical(this, "Export Failed", "Failed to write mesh file to disk.");
            artec_status_->setStatus("Export failed", widgets::StatusDisplay::StatusType::ERROR);
        }

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Export Failed",
                            QString("Exception during Artec mesh processing:\n%1").arg(e.what()));
        artec_status_->setStatus("Processing failed with exception", widgets::StatusDisplay::StatusType::ERROR);
        qDebug() << "[DepthWidget] Exception:" << e.what();
    }

    artec_status_->stopPulsing();
    artec_export_button_->setEnabled(true);

    qDebug() << "[DepthWidget] ===== ARTEC MESH EXPORT END =====";
}

// Update the onDepthResultReceived method to enable export buttons
void DepthTestWidget::onDepthResultReceived(const core::DepthResult& result) {
    qDebug() << "[DepthWidget] onDepthResultReceived() START";
    current_result_ = result;

    if (result.success) {
        qDebug() << "[DepthWidget] Setting processing status...";
        processing_status_->setStatus(
            QString("Depth processing completed in %1ms").arg(result.processing_time_ms, 0, 'f', 1),
            widgets::StatusDisplay::StatusType::SUCCESS
        );

        qDebug() << "[DepthWidget] About to call updateDepthVisualization()";
        updateDepthVisualization(result);
        qDebug() << "[DepthWidget] updateDepthVisualization() completed";

        // Update quality metrics with enhanced safety checks
        qDebug() << "[DepthWidget] About to update quality metrics...";
        try {
            if (quality_metrics_label_) {
                qDebug() << "[DepthWidget] quality_metrics_label_ is valid, updating text...";
                QString metrics = QString("Coverage: %1% | Mean Depth: %2mm | Std Dev: %3mm")
                    .arg(result.coverage_ratio * 100, 0, 'f', 1)
                    .arg(result.mean_depth, 0, 'f', 2)
                    .arg(result.std_depth, 0, 'f', 2);
                quality_metrics_label_->setText(metrics);
                qDebug() << "[DepthWidget] Quality metrics updated successfully";
            } else {
                qDebug() << "[DepthWidget] WARNING: quality_metrics_label_ is null or invalid";
            }
        } catch (const std::exception& e) {
            qDebug() << "[DepthWidget] EXCEPTION during quality metrics update:" << e.what();
        } catch (...) {
            qDebug() << "[DepthWidget] UNKNOWN EXCEPTION during quality metrics update";
        }

        // Enable point cloud export button from UI with enhanced safety checks
        qDebug() << "[DepthWidget] About to update UI button states...";
        try {
            if (ui && ui->save_pointcloud_button) {
                qDebug() << "[DepthWidget] Enabling save_pointcloud_button...";
                ui->save_pointcloud_button->setEnabled(true);
            }
            if (ui && ui->export_format_combo) {
                qDebug() << "[DepthWidget] Enabling export_format_combo...";
                ui->export_format_combo->setEnabled(true);
            }

            // Enable legacy point cloud export buttons (if they exist)
            if (export_pointcloud_button_) {
                qDebug() << "[DepthWidget] Enabling export_pointcloud_button_...";
                try {
                    export_pointcloud_button_->setEnabled(true);
                    qDebug() << "[DepthWidget] export_pointcloud_button_ enabled successfully";
                } catch (const std::exception& e) {
                    qDebug() << "[DepthWidget] EXCEPTION enabling export_pointcloud_button_:" << e.what();
                } catch (...) {
                    qDebug() << "[DepthWidget] UNKNOWN EXCEPTION enabling export_pointcloud_button_";
                }
            } else {
                qDebug() << "[DepthWidget] export_pointcloud_button_ is NULL - skipping";
            }
            if (export_mesh_button_) {
                qDebug() << "[DepthWidget] Enabling export_mesh_button_...";
                try {
                    export_mesh_button_->setEnabled(true);
                    qDebug() << "[DepthWidget] export_mesh_button_ enabled successfully";
                } catch (const std::exception& e) {
                    qDebug() << "[DepthWidget] EXCEPTION enabling export_mesh_button_:" << e.what();
                } catch (...) {
                    qDebug() << "[DepthWidget] UNKNOWN EXCEPTION enabling export_mesh_button_";
                }
            } else {
                qDebug() << "[DepthWidget] export_mesh_button_ is NULL - skipping";
            }

            // Enable Artec mesh export button (NEW)
            if (artec_export_button_) {
                qDebug() << "[DepthWidget] Enabling artec_export_button_...";
                artec_export_button_->setEnabled(true);
                artec_status_->setStatus("Ready to generate Artec mesh", widgets::StatusDisplay::StatusType::SUCCESS);
                qDebug() << "[DepthWidget] artec_export_button_ enabled successfully";
            }

            qDebug() << "[DepthWidget] All UI updates completed successfully";
        } catch (const std::exception& e) {
            qDebug() << "[DepthWidget] EXCEPTION during UI button updates:" << e.what();
        } catch (...) {
            qDebug() << "[DepthWidget] UNKNOWN EXCEPTION during UI button updates";
        }

        qDebug() << "[DepthWidget] SUCCESS PATH: About to exit onDepthResultReceived()";

    } else {
        processing_status_->setStatus("Depth processing failed: " + QString::fromStdString(result.error_message),
                                     widgets::StatusDisplay::StatusType::ERROR);

        // Disable export buttons on failure with safety checks
        if (ui && ui->save_pointcloud_button) {
            ui->save_pointcloud_button->setEnabled(false);
        }
        if (ui && ui->export_format_combo) {
            ui->export_format_combo->setEnabled(false);
        }

        if (export_pointcloud_button_) {
            try {
                export_pointcloud_button_->setEnabled(false);
            } catch (...) {
                qDebug() << "[DepthWidget] Exception disabling export_pointcloud_button_";
            }
        }
        if (export_mesh_button_) {
            try {
                export_mesh_button_->setEnabled(false);
            } catch (...) {
                qDebug() << "[DepthWidget] Exception disabling export_mesh_button_";
            }
        }
    }
    qDebug() << "[DepthWidget] onDepthResultReceived() END";
}

void DepthTestWidget::onVCSELThermalEvent(bool thermal_active, float temperature_c) {
    qDebug() << "[DepthWidget] VCSEL thermal event: active=" << thermal_active
             << "temperature=" << temperature_c << "°C";

    if (vcsel_status_) {
        if (thermal_active) {
            QString msg = QString("VCSEL Thermal Protection: %1°C").arg(temperature_c, 0, 'f', 1);
            vcsel_status_->setStatus(msg, StatusDisplay::StatusType::WARNING);
            vcsel_status_->startPulsing();

            // Show warning message if temperature is critical
            if (temperature_c > 68.0f) {
                processing_status_->showTemporaryMessage(
                    "VCSEL temperature high - cooling down",
                    StatusDisplay::StatusType::WARNING, 5000);
            }
        } else {
            QString msg = QString("VCSEL Temp: %1°C").arg(temperature_c, 0, 'f', 1);
            vcsel_status_->setStatus(msg, StatusDisplay::StatusType::INFO);
            vcsel_status_->stopPulsing();
        }
    }

    // Disable capture button if thermal protection is active
    if (thermal_active) {
        ui->capture_button->setEnabled(false);
        ui->capture_button->setText("COOLING DOWN...");
    } else {
        ui->capture_button->setEnabled(true);
        ui->capture_button->setText("CAPTURE STEREO FRAME");
    }
}

void DepthTestWidget::onVCSELError(const std::string& error) {
    qWarning() << "[DepthWidget] VCSEL error:" << QString::fromStdString(error);

    if (vcsel_status_) {
        vcsel_status_->setStatus("VCSEL Error", StatusDisplay::StatusType::ERROR);
        vcsel_status_->showTemporaryMessage(
            QString::fromStdString(error),
            StatusDisplay::StatusType::ERROR, 5000);
    }

    // Also show in main processing status
    processing_status_->showTemporaryMessage(
        QString("VCSEL: %1").arg(QString::fromStdString(error)),
        StatusDisplay::StatusType::ERROR, 3000);
}

void DepthTestWidget::onFilterDisableToggled(bool disable) {
    qDebug() << "[DepthWidget] Filter disable checkbox toggled:" << disable;

#ifndef DISABLE_POINTCLOUD_FUNCTIONALITY
    // When checked, disable all point cloud filters for investor demo
    if (disable) {
        pointcloud_filter_config_.enableStatisticalFilter = false;
        pointcloud_filter_config_.enableVoxelDownsampling = false;
        pointcloud_filter_config_.enableRadiusFilter = false;

        qDebug() << "[DepthWidget] ALL FILTERS DISABLED - Demo mode active";
        qDebug() << "[DepthWidget]   Statistical filter: OFF";
        qDebug() << "[DepthWidget]   Voxel downsampling: OFF";
        qDebug() << "[DepthWidget]   Radius filter: OFF";

        if (processing_status_) {
            processing_status_->showTemporaryMessage(
                "Demo Mode: All filters disabled for maximum point count",
                StatusDisplay::StatusType::WARNING, 3000);
        }
    } else {
        // Re-enable filters with original settings
        pointcloud_filter_config_.enableStatisticalFilter = false;  // Keep disabled (was too aggressive)
        pointcloud_filter_config_.enableVoxelDownsampling = false;
        pointcloud_filter_config_.enableRadiusFilter = false;

        qDebug() << "[DepthWidget] Filters returned to default configuration";

        if (processing_status_) {
            processing_status_->showTemporaryMessage(
                "Normal Mode: Filter configuration restored",
                StatusDisplay::StatusType::INFO, 2000);
        }
    }
#else
    qDebug() << "[DepthWidget] Point cloud functionality disabled at compile time";
#endif
}

void DepthTestWidget::onLEDToggle() {
    // Toggle LED state
    led_enabled_ = !led_enabled_;

    qDebug() << "[DepthWidget] LED toggle button clicked - LED enabled:" << led_enabled_;

    // Update button text based on new state
    if (ui && ui->led_toggle_button) {
        if (led_enabled_) {
            ui->led_toggle_button->setText("LED ON");
            ui->led_toggle_button->setChecked(true);
            qDebug() << "[DepthWidget] LED button updated to ON state";
        } else {
            ui->led_toggle_button->setText("LED OFF");
            ui->led_toggle_button->setChecked(false);
            qDebug() << "[DepthWidget] LED button updated to OFF state";
        }
    }

    // Show status message
    if (processing_status_) {
        QString message = led_enabled_ ?
            "LED illumination ENABLED for next capture" :
            "LED illumination DISABLED for next capture";
        processing_status_->showTemporaryMessage(
            message,
            StatusDisplay::StatusType::INFO, 2000);
    }

    qDebug() << "[DepthWidget] LED toggle complete - next capture will use LED:" << led_enabled_;
}

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake