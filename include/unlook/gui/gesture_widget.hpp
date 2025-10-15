#pragma once

#include <QWidget>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QPixmap>
#include <memory>
#include <map>

#include "unlook/camera/camera_system.hpp"
#include "unlook/gesture/GestureRecognitionSystem.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/parameter_slider.hpp"
#include "unlook/gui/widgets/status_display.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class GestureWidget; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

// Forward declarations
namespace widgets {
class TouchButton;
class ParameterSlider;
class StatusDisplay;
}

/**
 * @brief Gesture recognition widget with live camera preview and gesture detection
 *
 * Features:
 * - Live camera preview with gesture visualization overlays
 * - Real-time gesture detection and classification
 * - Gesture history display with confidence scores
 * - Performance statistics (FPS, processing time)
 * - Configuration controls (confidence threshold, debug visualization)
 * - Gesture statistics (count by type)
 * - Touch-optimized interface
 */
class GestureWidget : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance
     * @param parent Parent widget
     */
    explicit GestureWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                          QWidget* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~GestureWidget();

protected:
    /**
     * @brief Handle show events to start gesture recognition
     */
    void showEvent(QShowEvent* event) override;

    /**
     * @brief Handle hide events to stop gesture recognition
     */
    void hideEvent(QHideEvent* event) override;

signals:
    /**
     * @brief Emitted when a gesture is detected
     * @param gestureName Name of detected gesture
     * @param confidence Confidence score [0-1]
     */
    void gestureDetected(const QString& gestureName, float confidence);

    /**
     * @brief Emitted when an error occurs
     * @param error Error message
     */
    void errorOccurred(const QString& error);

private slots:

    /**
     * @brief Start gesture recognition
     */
    void startGestureRecognition();

    /**
     * @brief Stop gesture recognition
     */
    void stopGestureRecognition();

    /**
     * @brief Handle confidence threshold change
     */
    void onConfidenceChanged(double value);

    /**
     * @brief Toggle debug visualization
     */
    void onDebugVisualizationToggled(bool enabled);

    /**
     * @brief Clear gesture history
     */
    void onClearHistoryClicked();

    /**
     * @brief Update FPS display
     */
    void updateFPSDisplay();

private:
    /**
     * @brief Connect UI signals to slots
     */
    void connectSignals();

    /**
     * @brief Initialize additional UI components
     */
    void initializeAdditionalComponents();

    /**
     * @brief Initialize gesture recognition system
     */
    void initializeGestureSystem();

    /**
     * @brief Process frame from camera callback (NOT a slot - called from lambda)
     * @param frame_pair Stereo frame pair from camera
     */
    void processFrame(const core::StereoFramePair& frame_pair);

    /**
     * @brief Update gesture history display
     * @param gesture Gesture name
     * @param confidence Confidence score
     */
    void updateGestureHistory(const QString& gesture, float confidence);

    /**
     * @brief Update performance statistics display
     * @param fps Current FPS
     * @param processingTime Processing time in ms
     */
    void updatePerformanceStats(double fps, double processingTime);

    /**
     * @brief Update gesture statistics display
     */
    void updateGestureStatistics();

    /**
     * @brief Display camera frame with gesture overlays
     * @param frame OpenCV frame to display
     */
    void displayCameraFrame(const cv::Mat& frame);

    /**
     * @brief Convert OpenCV Mat to QPixmap
     */
    QPixmap matToQPixmap(const cv::Mat& mat);

    /**
     * @brief Scale pixmap to fit display
     */
    QPixmap scalePixmapToFit(const QPixmap& pixmap, const QSize& size);

    /**
     * @brief Get responsive preview size
     */
    QSize getResponsivePreviewSize();

    // UI Components
    Ui::GestureWidget *ui;

    // Gesture recognition system
    std::unique_ptr<gesture::GestureRecognitionSystem> gesture_system_;
    std::shared_ptr<camera::CameraSystem> camera_system_;

    // Processing state
    bool is_running_;
    bool widget_visible_;
    bool debug_visualization_enabled_;
    QTimer* fps_timer_;  // Only for FPS statistics display

    // Camera feed display
    QLabel* camera_feed_label_;

    // Statistics
    int frame_count_;
    int gesture_count_;
    double current_fps_;
    double avg_processing_time_;
    std::map<gesture::GestureType, int> gesture_histogram_;

    // Status displays
    widgets::StatusDisplay* gesture_status_;
    widgets::StatusDisplay* processing_status_;
    widgets::StatusDisplay* camera_status_;

    // Constants
    static constexpr int CAMERA_FEED_WIDTH = 640;
    static constexpr int CAMERA_FEED_HEIGHT = 480;
    static constexpr int MAX_HISTORY_ENTRIES = 50;
    static constexpr double DEFAULT_CONFIDENCE_THRESHOLD = 0.7;
    static constexpr int FRAME_PROCESSING_INTERVAL_MS = 33;  // ~30 FPS
};

} // namespace gui
} // namespace unlook
