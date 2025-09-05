#pragma once

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QPixmap>
#include <memory>

#include "unlook/camera/camera_system.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/parameter_slider.hpp"
#include "unlook/gui/widgets/status_display.hpp"

namespace unlook {
namespace gui {

/**
 * @brief Camera preview widget with dual camera display and controls
 * 
 * Features:
 * - Dual camera preview (LEFT/RIGHT side-by-side)
 * - Grayscale display optimized for IMX296 sensors
 * - Camera controls panel (exposure, gain, FPS)
 * - Auto-exposure enable/disable toggle
 * - Camera swap functionality
 * - Real-time status display
 * - Touch-optimized interface
 */
class CameraPreviewWidget : public QWidget {
    Q_OBJECT
    
public:
    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance
     * @param parent Parent widget
     */
    explicit CameraPreviewWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                                QWidget* parent = nullptr);
    
    /**
     * @brief Destructor
     */
    ~CameraPreviewWidget();

protected:
    /**
     * @brief Handle show events to start preview
     */
    void showEvent(QShowEvent* event) override;
    
    /**
     * @brief Handle hide events to stop preview
     */
    void hideEvent(QHideEvent* event) override;

private slots:
    /**
     * @brief Handle stereo frame updates
     */
    void onStereoFrameReceived(const core::StereoFramePair& frame_pair);
    
    /**
     * @brief Update FPS display
     */
    void updateFPSDisplay();
    
    /**
     * @brief Handle left camera exposure change
     */
    void onLeftExposureChanged(double value);
    
    /**
     * @brief Handle right camera exposure change
     */
    void onRightExposureChanged(double value);
    
    /**
     * @brief Handle left camera gain change
     */
    void onLeftGainChanged(double value);
    
    /**
     * @brief Handle right camera gain change
     */
    void onRightGainChanged(double value);
    
    /**
     * @brief Handle FPS change
     */
    void onFPSChanged(double value);
    
    /**
     * @brief Toggle auto exposure for left camera
     */
    void toggleLeftAutoExposure();
    
    /**
     * @brief Toggle auto exposure for right camera
     */
    void toggleRightAutoExposure();
    
    /**
     * @brief Toggle auto gain for left camera
     */
    void toggleLeftAutoGain();
    
    /**
     * @brief Toggle auto gain for right camera
     */
    void toggleRightAutoGain();
    
    /**
     * @brief Swap left and right camera displays
     */
    void swapCameraDisplays();
    
    /**
     * @brief Start camera capture
     */
    void startCapture();
    
    /**
     * @brief Stop camera capture
     */
    void stopCapture();

private:
    /**
     * @brief Initialize the widget UI
     */
    void initializeUI();
    
    /**
     * @brief Create camera preview area
     */
    QWidget* createPreviewArea();
    
    /**
     * @brief Create camera controls panel
     */
    QWidget* createControlsPanel();
    
    /**
     * @brief Create status panel
     */
    QWidget* createStatusPanel();
    
    /**
     * @brief Update camera parameter controls
     */
    void updateCameraControls();
    
    /**
     * @brief Convert OpenCV Mat to QPixmap for display
     */
    QPixmap matToQPixmap(const cv::Mat& mat);
    
    /**
     * @brief Scale pixmap to fit display while maintaining aspect ratio
     */
    QPixmap scalePixmapToFit(const QPixmap& pixmap, const QSize& size);
    
    /**
     * @brief Update preview images
     */
    void updatePreviewImages(const core::CameraFrame& left_frame, 
                            const core::CameraFrame& right_frame);
    
    /**
     * @brief Get responsive preview size based on screen resolution
     */
    QSize getResponsivePreviewSize();
    
    /**
     * @brief Create compact side panel for small screens (combines controls + status)
     */
    QWidget* createCompactSidePanel();
    
    // Camera system
    std::shared_ptr<camera::CameraSystem> camera_system_;
    
    // UI Layout
    QHBoxLayout* main_layout_;
    QWidget* preview_area_;
    QWidget* controls_panel_;
    QWidget* status_panel_;
    
    // Preview displays
    QLabel* left_camera_label_;
    QLabel* right_camera_label_;
    QLabel* left_title_label_;
    QLabel* right_title_label_;
    bool cameras_swapped_;
    
    // Control widgets
    widgets::ParameterSlider* left_exposure_slider_;
    widgets::ParameterSlider* right_exposure_slider_;
    widgets::ParameterSlider* left_gain_slider_;
    widgets::ParameterSlider* right_gain_slider_;
    widgets::ParameterSlider* fps_slider_;
    
    widgets::TouchButton* left_auto_exposure_button_;
    widgets::TouchButton* right_auto_exposure_button_;
    widgets::TouchButton* left_auto_gain_button_;
    widgets::TouchButton* right_auto_gain_button_;
    widgets::TouchButton* swap_cameras_button_;
    widgets::TouchButton* start_capture_button_;
    widgets::TouchButton* stop_capture_button_;
    
    // Status displays
    widgets::StatusDisplay* left_camera_status_;
    widgets::StatusDisplay* right_camera_status_;
    widgets::StatusDisplay* fps_status_;
    widgets::StatusDisplay* sync_status_;
    
    // State management
    bool capture_active_;
    bool widget_visible_;
    
    // FPS calculation
    QTimer* fps_timer_;
    int frame_count_;
    double current_fps_;
    
    // Constants - dimensions now calculated dynamically via DisplayMetrics
    static constexpr double MIN_EXPOSURE_US = 100.0;
    static constexpr double MAX_EXPOSURE_US = 100000.0;
    static constexpr double MIN_GAIN = 1.0;
    static constexpr double MAX_GAIN = 16.0;
    static constexpr double MIN_FPS = 1.0;
    static constexpr double MAX_FPS = 60.0;
};

} // namespace gui
} // namespace unlook