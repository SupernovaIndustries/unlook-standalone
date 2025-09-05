#include "unlook/gui/depth_test_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include <QGridLayout>
#include <QComboBox>
#include <QMessageBox>
#include <QDateTime>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

DepthTestWidget::DepthTestWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , camera_system_(camera_system)
    , processing_active_(false)
{
    initializeUI();
    initializeDepthProcessor();
}

DepthTestWidget::~DepthTestWidget() = default;

void DepthTestWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    // Update status when shown
    if (processing_status_) {
        processing_status_->setStatus("Ready for depth testing", StatusDisplay::StatusType::INFO);
    }
}

void DepthTestWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    // Stop any ongoing processing
    processing_active_ = false;
}

void DepthTestWidget::captureStereoFrame() {
    if (!camera_system_ || !camera_system_->isReady()) {
        processing_status_->setStatus("Camera system not ready", StatusDisplay::StatusType::ERROR);
        return;
    }
    
    processing_active_ = true;
    capture_status_->setStatus("Capturing stereo frame...", StatusDisplay::StatusType::PROCESSING);
    capture_status_->startPulsing();
    
    // Capture single frame
    core::StereoFramePair frame_pair = camera_system_->captureSingle();
    
    if (frame_pair.synchronized) {
        capture_status_->setStatus("Processing depth map...", StatusDisplay::StatusType::PROCESSING);
        
        // Process depth map
        if (depth_processor_) {
            core::DepthResult result = depth_processor_->processSync(frame_pair);
            onDepthResultReceived(result);
        } else {
            capture_status_->setStatus("Depth processor not initialized", StatusDisplay::StatusType::ERROR);
        }
    } else {
        capture_status_->setStatus("Failed to capture synchronized frames", StatusDisplay::StatusType::ERROR);
    }
    
    capture_status_->stopPulsing();
    processing_active_ = false;
}

void DepthTestWidget::onDepthResultReceived(const core::DepthResult& result) {
    current_result_ = result;
    
    if (result.success) {
        processing_status_->setStatus(
            QString("Depth processing completed in %1ms").arg(result.processing_time_ms, 0, 'f', 1),
            StatusDisplay::StatusType::SUCCESS
        );
        
        updateDepthVisualization(result);
        
        // Update quality metrics
        QString metrics = QString("Coverage: %1% | Mean Depth: %2mm | Std Dev: %3mm")
            .arg(result.coverage_ratio * 100, 0, 'f', 1)
            .arg(result.mean_depth, 0, 'f', 2)
            .arg(result.std_depth, 0, 'f', 2);
        quality_metrics_label_->setText(metrics);
        
        export_button_->setEnabled(true);
    } else {
        processing_status_->setStatus("Depth processing failed: " + QString::fromStdString(result.error_message),
                                     StatusDisplay::StatusType::ERROR);
        export_button_->setEnabled(false);
    }
}

void DepthTestWidget::onAlgorithmChanged(int index) {
    updateStereoParameters();
}

void DepthTestWidget::applyParameterPreset() {
    TouchButton* sender_button = qobject_cast<TouchButton*>(sender());
    if (!sender_button) return;
    
    core::DepthQuality quality = core::DepthQuality::BALANCED;
    
    if (sender_button == preset_fast_button_) {
        quality = core::DepthQuality::FAST_LOW;
    } else if (sender_button == preset_balanced_button_) {
        quality = core::DepthQuality::BALANCED;
    } else if (sender_button == preset_quality_button_) {
        quality = core::DepthQuality::SLOW_HIGH;
    }
    
    // Update parameters based on preset
    core::StereoConfig preset_config = api::DepthProcessor::createPreset(
        quality, core::StereoAlgorithm::SGBM_OPENCV);
    
    // Update UI sliders
    num_disparities_slider_->setValue(preset_config.num_disparities);
    block_size_slider_->setValue(preset_config.block_size);
    uniqueness_ratio_slider_->setValue(preset_config.uniqueness_ratio);
    
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
    config.min_disparity = static_cast<int>(min_disparity_slider_->getValue());
    config.num_disparities = static_cast<int>(num_disparities_slider_->getValue());
    config.block_size = static_cast<int>(block_size_slider_->getValue());
    config.uniqueness_ratio = static_cast<int>(uniqueness_ratio_slider_->getValue());
    
    // Set algorithm from combo box
    switch (algorithm_combo_->currentIndex()) {
        case 0:
            config.algorithm = core::StereoAlgorithm::SGBM_OPENCV;
            break;
        case 1:
            config.algorithm = core::StereoAlgorithm::BOOFCV_BASIC;
            break;
        case 2:
            config.algorithm = core::StereoAlgorithm::BOOFCV_PRECISE;
            break;
    }
    
    depth_processor_->configureStereo(config);
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
    
    // Capture button
    capture_button_ = new TouchButton("CAPTURE STEREO FRAME", TouchButton::ButtonType::PRIMARY);
    capture_button_->setMinimumHeight(60);
    connect(capture_button_, &TouchButton::clicked, this, &DepthTestWidget::captureStereoFrame);
    layout->addWidget(capture_button_);
    
    // Export button
    export_button_ = new TouchButton("EXPORT DEPTH MAP", TouchButton::ButtonType::SECONDARY);
    export_button_->setEnabled(false);
    connect(export_button_, &TouchButton::clicked, this, &DepthTestWidget::exportDepthMap);
    layout->addWidget(export_button_);
    
    // Capture status
    capture_status_ = new StatusDisplay("Capture");
    capture_status_->setCompactMode(true);
    capture_status_->setStatus("Ready", StatusDisplay::StatusType::INFO);
    layout->addWidget(capture_status_);
    
    return panel;
}

QWidget* DepthTestWidget::createAlgorithmPanel() {
    QWidget* panel = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(panel);
    
    QLabel* title = new QLabel("Algorithm Selection");
    title->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SUBTITLE, SupernovaStyle::FontWeight::BOLD));
    title->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title);
    
    // Algorithm combo box
    algorithm_combo_ = new QComboBox();
    algorithm_combo_->addItem("OpenCV SGBM");
    algorithm_combo_->addItem("BoofCV Basic");
    algorithm_combo_->addItem("BoofCV Precise");
    connect(algorithm_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &DepthTestWidget::onAlgorithmChanged);
    layout->addWidget(algorithm_combo_);
    
    // Preset buttons
    QHBoxLayout* preset_layout = new QHBoxLayout();
    
    preset_fast_button_ = new TouchButton("Fast", TouchButton::ButtonType::WARNING);
    connect(preset_fast_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    preset_layout->addWidget(preset_fast_button_);
    
    preset_balanced_button_ = new TouchButton("Balanced", TouchButton::ButtonType::PRIMARY);
    connect(preset_balanced_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    preset_layout->addWidget(preset_balanced_button_);
    
    preset_quality_button_ = new TouchButton("Quality", TouchButton::ButtonType::SUCCESS);
    connect(preset_quality_button_, &TouchButton::clicked, this, &DepthTestWidget::applyParameterPreset);
    preset_layout->addWidget(preset_quality_button_);
    
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
    
    // Parameter sliders
    min_disparity_slider_ = new ParameterSlider("Min Disparity", 0, 64, 0);
    connect(min_disparity_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    layout->addWidget(min_disparity_slider_);
    
    num_disparities_slider_ = new ParameterSlider("Num Disparities", 16, 256, 128);
    connect(num_disparities_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    layout->addWidget(num_disparities_slider_);
    
    block_size_slider_ = new ParameterSlider("Block Size", 3, 11, 5);
    connect(block_size_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    layout->addWidget(block_size_slider_);
    
    uniqueness_ratio_slider_ = new ParameterSlider("Uniqueness Ratio", 1, 20, 10);
    connect(uniqueness_ratio_slider_, &ParameterSlider::valueChanged, this, &DepthTestWidget::updateStereoParameters);
    layout->addWidget(uniqueness_ratio_slider_);
    
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
    
    // Create visualization using depth processor
    cv::Mat visualization = depth_processor_->visualizeDepthMap(result.depth_map, 0.0, 1000.0);
    
    if (!visualization.empty()) {
        // Convert to QPixmap
        QImage qimg(visualization.data, visualization.cols, visualization.rows, 
                   visualization.step, QImage::Format_BGR888);
        QPixmap pixmap = QPixmap::fromImage(qimg.rgbSwapped());
        
        // Scale to fit display
        pixmap = pixmap.scaled(depth_map_display_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
        depth_map_display_->setPixmap(pixmap);
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

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake