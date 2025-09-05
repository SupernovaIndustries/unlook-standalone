#pragma once

#include <QWidget>
#include <QLabel>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <memory>

#include "unlook/camera/camera_system.hpp"
#include "unlook/api/depth_processor.h"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/parameter_slider.hpp"
#include "unlook/gui/widgets/status_display.hpp"

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
     * @brief Update stereo parameters from UI
     */
    void updateStereoParameters();

private:
    /**
     * @brief Initialize the widget UI
     */
    void initializeUI();
    
    /**
     * @brief Create capture controls panel
     */
    QWidget* createCapturePanel();
    
    /**
     * @brief Create algorithm selection panel
     */
    QWidget* createAlgorithmPanel();
    
    /**
     * @brief Create parameter adjustment panel
     */
    QWidget* createParameterPanel();
    
    /**
     * @brief Create depth visualization panel
     */
    QWidget* createVisualizationPanel();
    
    /**
     * @brief Update depth map display
     */
    void updateDepthVisualization(const core::DepthResult& result);
    
    /**
     * @brief Initialize depth processor
     */
    void initializeDepthProcessor();
    
    // System integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    std::unique_ptr<api::DepthProcessor> depth_processor_;
    
    // UI Layout
    QHBoxLayout* main_layout_;
    QWidget* controls_panel_;
    QWidget* visualization_panel_;
    
    // Capture controls
    widgets::TouchButton* capture_button_;
    widgets::TouchButton* export_button_;
    widgets::StatusDisplay* capture_status_;
    
    // Algorithm selection
    QComboBox* algorithm_combo_;
    widgets::TouchButton* preset_fast_button_;
    widgets::TouchButton* preset_balanced_button_;
    widgets::TouchButton* preset_quality_button_;
    
    // Parameter controls
    widgets::ParameterSlider* min_disparity_slider_;
    widgets::ParameterSlider* num_disparities_slider_;
    widgets::ParameterSlider* block_size_slider_;
    widgets::ParameterSlider* uniqueness_ratio_slider_;
    
    // Visualization
    QLabel* depth_map_display_;
    QLabel* quality_metrics_label_;
    widgets::StatusDisplay* processing_status_;
    
    // Current data
    core::DepthResult current_result_;
    bool processing_active_;
    
    // Constants
    static const int DEPTH_DISPLAY_WIDTH = 500;
    static const int DEPTH_DISPLAY_HEIGHT = 375;
    static const int CONTROLS_WIDTH = 350;
};

} // namespace gui
} // namespace unlook