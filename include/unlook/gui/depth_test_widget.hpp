#pragma once

#include <QWidget>
#include <QLabel>
#include <QComboBox>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <memory>

#include "unlook/camera/camera_system.hpp"
#include "unlook/api/depth_processor.h"
#ifndef DISABLE_POINTCLOUD_FUNCTIONALITY
#include "unlook/pointcloud/PointCloudProcessor.hpp"
#endif
#include "unlook/hardware/VCSELProjector.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/parameter_slider.hpp"
#include "unlook/gui/widgets/status_display.hpp"

// Forward declarations
QT_BEGIN_NAMESPACE
namespace Ui { class DepthTestWidget; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

/**
 * @brief Depth test widget with stereo processing interface
 * 
 * Features:
 * - Stereo pair capture button (large, touch-friendly)
 * - Algorithm selection dropdown (SGBM, BoofCV options)
 * - Parameter adjustment panel with real-time sliders
 * - Depth map visualization with false-color display
 * - Quality metrics overlay and depth histogram
 * - Export depth map functionality
 */
class DepthTestWidget : public QWidget {
    Q_OBJECT
    
public:
    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance
     * @param parent Parent widget
     */
    explicit DepthTestWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                            QWidget* parent = nullptr);
    
    /**
     * @brief Destructor
     */
    ~DepthTestWidget();

protected:
    /**
     * @brief Handle show events
     */
    void showEvent(QShowEvent* event) override;
    
    /**
     * @brief Handle hide events
     */
    void hideEvent(QHideEvent* event) override;

private slots:
    /**
     * @brief Capture stereo frame pair for depth processing
     */
    void captureStereoFrame();
    
    /**
     * @brief Start live camera preview
     */
    void startLivePreview();
    
    /**
     * @brief Stop live camera preview
     */
    void stopLivePreview();
    
    /**
     * @brief Handle live frame updates for preview
     */
    void onLiveFrameReceived(const core::StereoFramePair& frame_pair);
    
    /**
     * @brief Handle depth result from processor
     */
    void onDepthResultReceived(const core::DepthResult& result);
    
    /**
     * @brief Change stereo algorithm
     */
    void onAlgorithmChanged(int index);
    
    /**
     * @brief Apply parameter preset
     */
    void applyParameterPreset();
    
    /**
     * @brief Export depth map
     */
    void exportDepthMap();

    /**
     * @brief Export point cloud in selected format
     */
    void exportPointCloud();

    /**
     * @brief Export mesh from point cloud
     */
    void exportMesh();

    /**
     * @brief Export mesh using Artec-grade pipeline (NEW)
     */
    void exportArtecMesh();

    /**
     * @brief Apply Artec quality preset (INDUSTRIAL/ORGANIC/PREVIEW)
     */
    void applyArtecPreset(int presetIndex);

    /**
     * @brief Update Artec octree depth from slider
     */
    void updateArtecOctreeDepth(int depth);

    /**
     * @brief Configure point cloud export options
     */
    void configurePointCloudExport();

    /**
     * @brief Configure mesh generation options
     */
    void configureMeshGeneration();

    /**
     * @brief Update stereo parameters from UI
     */
    void updateStereoParameters();

    /**
     * @brief Update export format from UI selection
     */
    void updateExportFormat();

    /**
     * @brief Handle VCSEL thermal callback
     */
    void onVCSELThermalEvent(bool thermal_active, float temperature_c);

    /**
     * @brief Handle VCSEL error callback
     */
    void onVCSELError(const std::string& error);

    /**
     * @brief Handle filter disable checkbox toggle for investor demo
     */
    void onFilterDisableToggled(bool disable);

    /**
     * @brief Handle LED toggle button for investor demo
     */
    void onLEDToggle();

    /**
     * @brief Set ambient subtraction weight (0.0 = no subtraction, 1.0 = full subtraction)
     * @param weight Weight value between 0.0 and 1.0
     */
    void setAmbientSubtractionWeight(float weight) {
        ambient_subtraction_weight_ = std::max(0.0f, std::min(1.0f, weight));
    }

    /**
     * @brief Set pattern gain after ambient subtraction (default 1.5)
     * @param gain Gain value (typically 1.0 to 2.0)
     */
    void setAmbientPatternGain(float gain) {
        ambient_pattern_gain_ = std::max(0.5f, std::min(3.0f, gain));
    }

private:
    /**
     * @brief Initialize the widget UI
     */
    void initializeUI();
    
    /**
     * @brief Update depth visualization 
     */
    void updateDepthVisualization(const core::DepthResult& result);
    
    /**
     * @brief Update stereo frame images in UI labels
     */
    void updateStereoFrameImages(const core::CameraFrame& leftFrame, const core::CameraFrame& rightFrame);
    
    /**
     * @brief Save debug images to timestamped directory
     */
    void saveDebugImages(const core::StereoFramePair& frame_pair, const core::DepthResult& result);
    
    /**
     * @brief Create debug directory with timestamp
     */
    std::string createDebugDirectory();
    
    /**
     * @brief Connect UI signals to slots
     */
    void connectSignals();
    
    // UI Components  
    Ui::DepthTestWidget *ui;
    
    /**
     * @brief Create capture controls panel
     */
    QWidget* createCapturePanel();
    
    /**
     * @brief Create algorithm selection panel (deprecated, calls createStatusPanel)
     */
    QWidget* createAlgorithmPanel();

    /**
     * @brief Create real-time status panel
     */
    QWidget* createStatusPanel();

    /**
     * @brief Add message to status list
     * @param message Status message to display
     */
    void addStatusMessage(const QString& message);

    /**
     * @brief Create parameter adjustment panel
     */
    QWidget* createParameterPanel();
    
    /**
     * @brief Create depth visualization panel
     */
    QWidget* createVisualizationPanel();
    
    /**
     * @brief Initialize depth processor
     */
    void initializeDepthProcessor();

    /**
     * @brief Initialize point cloud processor
     */
    void initializePointCloudProcessor();

    /**
     * @brief Initialize VCSEL projector
     */
    void initializeVCSELProjector();

    /**
     * @brief Create point cloud export panel
     */
    QWidget* createPointCloudExportPanel();

    /**
     * @brief Create Artec-grade mesh processing panel (NEW)
     */
    QWidget* createArtecMeshPanel();

    // System integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    std::unique_ptr<api::DepthProcessor> depth_processor_;
#ifndef DISABLE_POINTCLOUD_FUNCTIONALITY
    std::unique_ptr<pointcloud::PointCloudProcessor> pointcloud_processor_;
#endif
    std::shared_ptr<hardware::VCSELProjector> vcsel_projector_;
    
    // UI Layout
    QHBoxLayout* main_layout_;
    QWidget* controls_panel_;
    QWidget* visualization_panel_;
    
    // UI widgets accessed via ui-> (defined in .ui file)
    // Removed duplicates: capture_button, algorithm_combo, sliders, etc.
    
    // Only non-.ui widgets kept:
    widgets::StatusDisplay* capture_status_;
    widgets::StatusDisplay* processing_status_;
    widgets::StatusDisplay* vcsel_status_;

    // Visualization
    QLabel* depth_map_display_;
    QLabel* quality_metrics_label_;
    
    // Current data
    core::DepthResult current_result_;
    bool processing_active_;
    bool live_preview_active_;
    bool led_enabled_;  // Track LED state for investor demo
    bool ambient_subtract_enabled_;  // Track ambient subtraction state
    float ambient_subtraction_weight_;  // Weight for ambient subtraction (0.0-1.0, default 0.3)
    float ambient_pattern_gain_;  // Gain to apply after ambient subtraction (default 1.5)
    std::string current_debug_directory_;  // Store current debug directory for image loading

#ifndef DISABLE_POINTCLOUD_FUNCTIONALITY
    // Point cloud export configuration
    pointcloud::PointCloudFilterConfig pointcloud_filter_config_;
    pointcloud::MeshGenerationConfig mesh_generation_config_;
    pointcloud::ExportFormat export_format_;
#endif

    // Point cloud export UI components
    widgets::TouchButton* export_pointcloud_button_;
    widgets::TouchButton* export_mesh_button_;
    widgets::TouchButton* configure_export_button_;
    QComboBox* export_format_combo_;
    widgets::StatusDisplay* export_status_;

    // Artec mesh processing UI components (NEW)
    widgets::TouchButton* artec_industrial_button_;
    widgets::TouchButton* artec_organic_button_;
    widgets::TouchButton* artec_preview_button_;
    widgets::TouchButton* artec_export_button_;
    QSlider* artec_octree_slider_;
    QLabel* artec_octree_label_;
    QLabel* artec_stats_label_;
    widgets::StatusDisplay* artec_status_;

    // Artec processing state
    int artec_octree_depth_{9};  // Default octree depth
    int artec_quality_preset_{0};  // 0=INDUSTRIAL, 1=ORGANIC, 2=PREVIEW

    // Constants
    static const int DEPTH_DISPLAY_WIDTH = 500;
    static const int DEPTH_DISPLAY_HEIGHT = 375;
    static const int CONTROLS_WIDTH = 350;
};

} // namespace gui
} // namespace unlook