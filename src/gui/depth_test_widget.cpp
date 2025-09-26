// Point cloud functionality now enabled with proper dependencies

#include "unlook/gui/depth_test_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/hardware/AS1170Controller.hpp"
#include "ui_depth_test_widget.h"
#include <QGridLayout>
#include <QComboBox>
#include <QMessageBox>
#include <QDateTime>
#include <QDebug>
#include <QDir>
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

    // Connect parameter sliders
    connect(ui->min_disparity_slider, QOverload<int>::of(&QSlider::valueChanged),
            this, &DepthTestWidget::updateStereoParameters);
    connect(ui->num_disparities_slider, QOverload<int>::of(&QSlider::valueChanged),
            this, &DepthTestWidget::updateStereoParameters);
    connect(ui->block_size_slider, QOverload<int>::of(&QSlider::valueChanged),
            this, &DepthTestWidget::updateStereoParameters);

    // Connect algorithm selection
    connect(ui->algorithm_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &DepthTestWidget::updateStereoParameters);

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

    // SYNCHRONIZATION STEP 1: Activate LEDs for optimal illumination
    qDebug() << "[DepthWidget] Activating AS1170 LEDs for depth capture";
    capture_status_->setStatus("Activating LED illumination...", StatusDisplay::StatusType::PROCESSING);

    auto as1170 = hardware::AS1170Controller::getInstance();
    bool leds_activated = false;
    if (as1170) {
        // Initialize if not already done
        if (!as1170->isInitialized()) {
            qDebug() << "[DepthWidget] Initializing AS1170 controller";
            as1170->initialize();
        }

        // Activate both LED1 (VCSEL) and LED2 (Flood) at 150mA for optimal depth capture illumination
        bool led1_success = as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, true, 150);
        bool led2_success = as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, true, 150);

        leds_activated = led1_success && led2_success;
        if (leds_activated) {
            qDebug() << "[DepthWidget] Both LEDs activated successfully at 150mA for depth capture";
        } else {
            qWarning() << "[DepthWidget] LED activation failed - LED1:" << led1_success << "LED2:" << led2_success;
        }
    } else {
        qWarning() << "[DepthWidget] AS1170Controller not available";
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

    // Capture single frame (now with VCSEL projection active if available)
    capture_status_->setStatus("Capturing stereo frame...", StatusDisplay::StatusType::PROCESSING);
    qDebug() << "[DepthWidget] Calling camera_system_->captureSingle()";
    core::StereoFramePair frame_pair = camera_system_->captureSingle();
    
    qDebug() << "[DepthWidget] Frame capture result:"
             << "synchronized=" << frame_pair.synchronized
             << "left_valid=" << frame_pair.left_frame.valid
             << "right_valid=" << frame_pair.right_frame.valid
             << "sync_error=" << frame_pair.sync_error_ms << "ms";
    
    if (frame_pair.synchronized) {
        // Show the captured stereo images in UI
        updateStereoFrameImages(frame_pair.left_frame, frame_pair.right_frame);
        
        capture_status_->setStatus("Processing depth map...", StatusDisplay::StatusType::PROCESSING);
        
        // Process depth map
        if (depth_processor_) {
            qDebug() << "[DepthWidget] Processing depth with depth_processor_";
            core::DepthResult result = depth_processor_->processSync(frame_pair);
            
            // AUTO DEBUG SAVE: Save debug images for every depth processing
            saveDebugImages(frame_pair, result);
            
            onDepthResultReceived(result);
        } else {
            qDebug() << "[DepthWidget] ERROR: depth_processor_ is null";
            capture_status_->setStatus("Depth processor not initialized", StatusDisplay::StatusType::ERROR);
        }
    } else {
        qDebug() << "[DepthWidget] ERROR: Failed to capture synchronized frames";
        capture_status_->setStatus("Failed to capture synchronized frames", StatusDisplay::StatusType::ERROR);
    }

    // SYNCHRONIZATION STEP 2: Deactivate LEDs after capture and processing complete
    qDebug() << "[DepthWidget] Deactivating AS1170 LEDs after depth capture";
    if (as1170) {
        // Deactivate both LEDs
        bool led1_off = as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED1, false, 0);
        bool led2_off = as1170->setLEDState(hardware::AS1170Controller::LEDChannel::LED2, false, 0);

        if (led1_off && led2_off) {
            qDebug() << "[DepthWidget] Both LEDs deactivated successfully after depth capture";
        } else {
            qWarning() << "[DepthWidget] LED deactivation failed - LED1:" << led1_off << "LED2:" << led2_off;
        }
    }

    capture_status_->stopPulsing();
    processing_active_ = false;
    qDebug() << "[DepthWidget] captureStereoFrame() completed";
}

void DepthTestWidget::onAlgorithmChanged(int index) {
    updateStereoParameters();
}

void DepthTestWidget::applyParameterPreset() {
    // TODO_UI: Add preset buttons to .ui file and reconnect
    TouchButton* sender_button = qobject_cast<TouchButton*>(sender());
    if (!sender_button) return;
    
    core::DepthQuality quality = core::DepthQuality::BALANCED;
    
    // if (sender_button == preset_fast_button_) {
    //     quality = core::DepthQuality::FAST_LOW;
    // } else if (sender_button == preset_balanced_button_) {
    //     quality = core::DepthQuality::BALANCED;
    // } else if (sender_button == preset_quality_button_) {
    //     quality = core::DepthQuality::SLOW_HIGH;
    // }
    
    // Update parameters based on preset
    core::StereoConfig preset_config = api::DepthProcessor::createPreset(
        quality, core::StereoAlgorithm::SGBM_OPENCV);
    
    // Update UI sliders
    ui->num_disparities_slider->setValue(preset_config.num_disparities);
    ui->block_size_slider->setValue(preset_config.block_size);
    // ui->uniqueness_ratio_slider->setValue(preset_config.uniqueness_ratio); // TODO: Add to .ui
    
    updateStereoParameters();
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

    core::StereoConfig config = depth_processor_->getStereoConfig();

    // Update config from UI
    config.min_disparity = ui->min_disparity_slider->value();
    config.num_disparities = ui->num_disparities_slider->value();
    config.block_size = ui->block_size_slider->value();
    config.uniqueness_ratio = 10; // Default value - TODO: Add slider to .ui

    // Set algorithm from combo box
    switch (ui->algorithm_combo->currentIndex()) {
        case 0:
            config.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
        case 1:
            config.algorithm = core::StereoAlgorithm::BOOFCV_DENSE_BM;
            break;
        case 2:
            config.algorithm = core::StereoAlgorithm::BOOFCV_DENSE_SGM;
            break;
        case 3:
            config.algorithm = core::StereoAlgorithm::BOOFCV_SUBPIXEL_BM;
            break;
        case 4:
            config.algorithm = core::StereoAlgorithm::BOOFCV_SUBPIXEL_SGM;
            break;
        default:
            config.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
    }

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
    
    // Algorithm panel
    controls_layout->addWidget(createAlgorithmPanel());
    
    // Parameter panel
    controls_layout->addWidget(createParameterPanel());

    // Note: Point cloud export is now handled via .ui file (save_pointcloud_button, export_format_combo)
    // Legacy programmatic point cloud export panel creation removed to avoid UI conflicts

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

QWidget* DepthTestWidget::createAlgorithmPanel() {
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QLabel* title = new QLabel("Algorithm Selection");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // TODO_UI: Add algorithm_combo to .ui file
    // algorithm_combo_ = new QComboBox();
    // //algorithm_combo_->addItem("OpenCV SGBM");
    // //algorithm_combo_->addItem("BoofCV Basic");
    // //algorithm_combo_->addItem("BoofCV Precise");
    // connect(algorithm_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
    //         this, &DepthTestWidget::onAlgorithmChanged);
    // layout->addWidget(algorithm_combo_);
    
    // Preset buttons
    QHBoxLayout* preset_layout = new QHBoxLayout();
    
    // TODO_UI: Add preset buttons to .ui file
    // preset_fast_button_ = new TouchButton("Fast", TouchButton::ButtonType::WARNING);
    // connect(preset_fast_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    // preset_layout->addWidget(preset_fast_button_);
    
    // preset_balanced_button_ = new TouchButton("Balanced", TouchButton::ButtonType::PRIMARY);
    // connect(preset_balanced_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    // preset_layout->addWidget(preset_balanced_button_);
    
    // preset_quality_button_ = new TouchButton("Quality", TouchButton::ButtonType::SUCCESS);
    // connect(preset_quality_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    // preset_layout->addWidget(preset_quality_button_);
    
    layout->addLayout(preset_layout);
    
    return panel;
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

    // Use dynamic depth range instead of fixed 0-1000mm to prevent visualization issues
    double min_depth, max_depth;
    cv::minMaxLoc(result.depth_map, &min_depth, &max_depth, nullptr, nullptr, result.depth_map > 0);

    // Ensure valid depth range for face scanning (use robust range if available)
    if (min_depth <= 0 || max_depth <= 0 || min_depth >= max_depth) {
        // Fallback to face scanning range for invalid depth maps
        min_depth = 400.0;   // Minimum face distance
        max_depth = 4000.0;  // Maximum face distance (4m limit)
    } else if (max_depth > 4000.0) {
        // Clamp visualization to 4m maximum
        max_depth = 4000.0;
    }

    qDebug() << "[DepthWidget] Dynamic depth visualization range:" << min_depth << "-" << max_depth << "mm";

    // Create visualization using depth processor with dynamic range
    cv::Mat visualization = depth_processor_->visualizeDepthMap(result.depth_map, min_depth, max_depth);

    if (!visualization.empty() && ui && ui->depth_image_label) {
        // CRITICAL FIX: Create deep copy to prevent dangling pointer in Qt rendering
        // The original shallow copy caused segfault when visualization went out of scope
        cv::Mat display_image;
        if (visualization.channels() == 3 && visualization.type() == CV_8UC3) {
            display_image = visualization.clone();  // Deep copy for safety
        } else {
            // Convert to proper format if needed
            if (visualization.channels() == 1) {
                cv::cvtColor(visualization, display_image, cv::COLOR_GRAY2BGR);
            } else {
                visualization.convertTo(display_image, CV_8UC3);
            }
        }

        // Safety check for image data and dimensions
        if (display_image.data && display_image.cols > 0 && display_image.rows > 0) {
            // CRITICAL FIX: Create QImage with deep copy to prevent dangling pointer segfault
            // The original code caused segfault because QImage held a pointer to cv::Mat data
            // that was deallocated when display_image went out of scope
            QImage qimg(display_image.data, display_image.cols, display_image.rows,
                       display_image.step, QImage::Format_RGB888);

            // Make a deep copy to ensure data ownership and prevent segfault
            QImage qimg_copy = qimg.copy();
            QPixmap pixmap = QPixmap::fromImage(qimg_copy.rgbSwapped());

            // Scale to fit display using .ui label with size validation
            QSize label_size = ui->depth_image_label->size();
            if (label_size.width() > 0 && label_size.height() > 0) {
                pixmap = pixmap.scaled(label_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
                ui->depth_image_label->setPixmap(pixmap);
                ui->depth_image_label->setText(""); // Clear the "No Depth Data" text
            }
        }
    } else {
        qWarning() << "[DepthWidget] Visualization failed: empty=" << visualization.empty()
                   << "ui_valid=" << (ui != nullptr) << "label_valid=" << (ui && ui->depth_image_label != nullptr);
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
        // Create debug directory with timestamp
        std::string debug_dir = createDebugDirectory();
        
        qDebug() << "[DepthWidget] Saving debug images to:" << QString::fromStdString(debug_dir);
        
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
                        qDebug() << "[DepthWidget] Rectified images saved successfully";
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

    // Initialize with depth processor
    if (depth_processor_) {
        std::shared_ptr<stereo::DepthProcessor> stereo_depth_processor;
        // Note: This might need adaptation depending on the depth processor structure
        // For now, we'll create a shared pointer wrapper or modify the API

        // TODO: Implement proper depth processor sharing or initialization
        if (pointcloud_processor_->initialize(stereo_depth_processor)) {
            qDebug() << "[DepthWidget] Point cloud processor initialized successfully";
        } else {
            qWarning() << "[DepthWidget] Failed to initialize point cloud processor";
        }
    } else {
        qWarning() << "[DepthWidget] Cannot initialize point cloud processor - depth processor not available";
    }

    // Initialize default configurations
    pointcloud_filter_config_ = pointcloud::PointCloudFilterConfig{};
    pointcloud_filter_config_.enableStatisticalFilter = true;
    pointcloud_filter_config_.statisticalNeighbors = 20;
    pointcloud_filter_config_.statisticalStdRatio = 2.0;
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
    vcsel_config.vcsel_current_ma = 250;  // Safe operating current
    vcsel_config.flood_current_ma = 150;  // Flood illumination
    vcsel_config.enable_flood_assist = true;
    vcsel_config.projection_duration_ms = 50;  // Short burst for safety
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
#ifdef DISABLE_POINTCLOUD_FUNCTIONALITY
    QMessageBox::information(this, "Feature Disabled",
        "Point cloud export is temporarily disabled during system build optimization.");
    return;
#endif
    if (!current_result_.success || current_result_.depth_map.empty()) {
        QMessageBox::warning(this, "Export Error", "No valid depth map available for point cloud export.");
        return;
    }

    if (!pointcloud_processor_) {
        QMessageBox::warning(this, "Export Error", "Point cloud processor not initialized.");
        return;
    }

    export_status_->setStatus("Generating point cloud...", widgets::StatusDisplay::StatusType::PROCESSING);
    export_status_->startPulsing();

    try {
        // Export format is already updated via updateExportFormat() slot
        // when the UI combo box selection changes

        // Generate point cloud from current depth result
        stereo::PointCloud pointCloud;

        // Create a safe color image if not available (prevents crashes)
        cv::Mat colorImage;
        if (current_result_.depth_map.rows > 0 && current_result_.depth_map.cols > 0) {
            // Generate grayscale color from depth for visualization
            cv::Mat depthGray;
            cv::normalize(current_result_.depth_map, depthGray, 0, 255, cv::NORM_MINMAX, CV_8U, current_result_.depth_map > 0);
            cv::cvtColor(depthGray, colorImage, cv::COLOR_GRAY2BGR);
        }

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

        // Assess point cloud quality
        pointcloud::PointCloudQuality quality;
        if (pointcloud_processor_->assessPointCloudQuality(pointCloud, quality)) {
            qDebug() << "[DepthWidget] Point cloud quality:"
                     << "Valid points:" << quality.validPoints
                     << "Density:" << quality.density
                     << "Valid ratio:" << (quality.validRatio * 100) << "%";
        }

        // Generate filename with timestamp
        QString filename = QString("unlook_pointcloud_%1%2")
            .arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"))
            .arg(QString::fromStdString(export_format_.getFileExtension()));

        QString filepath = QDir::homePath() + "/unlook_exports/" + filename;
        QDir().mkpath(QDir::homePath() + "/unlook_exports");

        // Set timestamp for export
        export_format_.timestamp = QDateTime::currentDateTime().toString().toStdString();

        // Export point cloud
        if (pointcloud_processor_->exportPointCloud(pointCloud, filepath.toStdString(), export_format_)) {
            export_status_->setStatus(QString("Point cloud exported: %1").arg(filename),
                                     widgets::StatusDisplay::StatusType::SUCCESS);

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

            QMessageBox::information(this, "Export Successful", message);
        } else {
            QString error = QString::fromStdString(pointcloud_processor_->getLastError());
            QMessageBox::critical(this, "Export Failed", QString("Failed to export point cloud: %1").arg(error));
            export_status_->setStatus("Point cloud export failed", widgets::StatusDisplay::StatusType::ERROR);
        }

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Export Failed", QString("Exception during export: %1").arg(e.what()));
        export_status_->setStatus("Export failed with exception", widgets::StatusDisplay::StatusType::ERROR);
    }

    export_status_->stopPulsing();
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

// Update the onDepthResultReceived method to enable export buttons
void DepthTestWidget::onDepthResultReceived(const core::DepthResult& result) {
    current_result_ = result;

    if (result.success) {
        processing_status_->setStatus(
            QString("Depth processing completed in %1ms").arg(result.processing_time_ms, 0, 'f', 1),
            widgets::StatusDisplay::StatusType::SUCCESS
        );

        updateDepthVisualization(result);

        // Update quality metrics
        QString metrics = QString("Coverage: %1% | Mean Depth: %2mm | Std Dev: %3mm")
            .arg(result.coverage_ratio * 100, 0, 'f', 1)
            .arg(result.mean_depth, 0, 'f', 2)
            .arg(result.std_depth, 0, 'f', 2);
        quality_metrics_label_->setText(metrics);

        // Enable point cloud export button from UI
        ui->save_pointcloud_button->setEnabled(true);
        ui->export_format_combo->setEnabled(true);

        // Enable legacy point cloud export buttons (if they exist)
        if (export_pointcloud_button_) {
            export_pointcloud_button_->setEnabled(true);
        }
        if (export_mesh_button_) {
            export_mesh_button_->setEnabled(true);
        }

    } else {
        processing_status_->setStatus("Depth processing failed: " + QString::fromStdString(result.error_message),
                                     widgets::StatusDisplay::StatusType::ERROR);

        // Disable export buttons on failure
        ui->save_pointcloud_button->setEnabled(false);
        ui->export_format_combo->setEnabled(false);

        if (export_pointcloud_button_) {
            export_pointcloud_button_->setEnabled(false);
        }
        if (export_mesh_button_) {
            export_mesh_button_->setEnabled(false);
        }
    }
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

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake