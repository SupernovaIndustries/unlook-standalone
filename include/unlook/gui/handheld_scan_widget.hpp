#pragma once

#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QTimer>
#include <QFuture>
#include <QFutureWatcher>
#include <QPixmap>
#include <memory>
#include <chrono>

#include "unlook/camera/CameraSystemGUI.hpp"
#include "unlook/core/types.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class HandheldScanWidget; }
QT_END_NAMESPACE

namespace unlook {

// Forward declarations
namespace hardware { class AS1170Controller; }

namespace gui {

/**
 * @brief Handheld scanning widget with real-time stability feedback
 *
 * Features:
 * - Real-time stability indicator (0-100% with color coding)
 * - Multi-frame capture progress (10 frames)
 * - FPS display
 * - Precision estimate
 * - Smooth animations and feedback
 * - Automated capture when stability threshold reached
 *
 * User Experience Flow:
 * 1. User clicks "Start Handheld Scan"
 * 2. Stability indicator shows real-time feedback (red -> orange -> green)
 * 3. When stable (>90%), capture automatically starts
 * 4. Progress bar shows frame capture (0-10 frames)
 * 5. Processing status updates in real-time
 * 6. Final point cloud displayed with precision achieved
 */
class HandheldScanWidget : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance (GUI version)
     * @param parent Parent widget
     */
    explicit HandheldScanWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system,
                                QWidget* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~HandheldScanWidget();

signals:
    /**
     * @brief Emitted when scan completes successfully
     * @param point_count Number of points in generated point cloud
     * @param precision_mm Achieved precision in millimeters
     */
    void scanCompleted(int point_count, float precision_mm);

    /**
     * @brief Emitted when scan fails
     * @param error_message Human-readable error description
     */
    void scanFailed(const QString& error_message);

protected:
    /**
     * @brief Handle show events to initialize camera system
     */
    void showEvent(QShowEvent* event) override;

    /**
     * @brief Handle hide events to restore GUI camera system
     */
    void hideEvent(QHideEvent* event) override;

private slots:
    /**
     * @brief Handle scan button click
     */
    void onStartScan();

    /**
     * @brief Handle stop button click
     */
    void onStopScan();

    /**
     * @brief Handle info button click - show statistics popup
     */
    void onShowInfo();

    /**
     * @brief Handle calibrate button click - auto-calibrate camera exposure/gain for VCSEL
     */
    void onAutoCalibrate();

    /**
     * @brief Update UI components (called at 30 Hz)
     */
    void updateUI();

    /**
     * @brief Handle scan completion from background thread
     */
    void onScanCompleted();

    /**
     * @brief Handle scan failure from background thread
     */
    void onScanFailed(const QString& error);

private:
    /**
     * @brief Setup UI components from .ui file
     */
    void setupUI();

    /**
     * @brief Start camera preview in background
     */
    void startCameraPreview();

    /**
     * @brief Stop camera preview
     */
    void stopCameraPreview();

    /**
     * @brief Handle preview frame from camera callback
     */
    void onPreviewFrame(const core::StereoFramePair& frame);

    /**
     * @brief Update stability indicator with current score
     * @param score Stability score (0.0 - 1.0)
     */
    void updateStabilityIndicator(float score);

    /**
     * @brief Update capture progress display
     * @param current Current frame number
     * @param total Total frames to capture
     */
    void updateCaptureProgress(int current, int total);

    /**
     * @brief Calculate FPS from frame timestamps
     * @return Current FPS
     */
    float calculateFPS();

    /**
     * @brief Get color for stability score
     * @param score Stability score (0.0 - 1.0)
     * @return QColor for progress bar
     */
    QColor getStabilityColor(float score) const;

    /**
     * @brief Get text for stability score
     * @param score Stability score (0.0 - 1.0)
     * @return Human-readable status text
     */
    QString getStabilityText(float score) const;

    /**
     * @brief Reset UI to initial state
     */
    void resetUI();

    /**
     * @brief Start background scan thread
     */
    void startScanThread();

    /**
     * @brief Perform automatic camera calibration for VCSEL dots visibility
     *
     * Analyzes histogram of captured frames and iteratively adjusts exposure/gain
     * to avoid saturation while maintaining good VCSEL dot visibility.
     *
     * Target: mean brightness 100-120, <1% saturated pixels (>=250)
     *
     * @return true if calibration successful, false otherwise
     */
    bool performAutoCalibration();

    // UI from .ui file
    Ui::HandheldScanWidget* ui;

    // Backend Integration
    std::shared_ptr<camera::gui::CameraSystem> camera_system_;  // Shared GUI camera system
    std::shared_ptr<hardware::AS1170Controller> led_controller_;  // LED controller singleton
    QTimer* update_timer_;                                      // 30 Hz UI updates

    // Scan State Management
    enum class ScanState {
        IDLE,                              // Not scanning
        WAITING_STABILITY,                 // Waiting for user to stabilize
        CAPTURING,                         // Capturing frames
        PROCESSING,                        // Processing depth maps
        COMPLETED,                         // Scan completed successfully
        FAILED                             // Scan failed
    };

    ScanState scan_state_;

    // Scan Progress Tracking
    int frames_captured_;                  // Current frame count
    int total_frames_;                     // Target frame count (10)
    float current_stability_;              // Current stability score (0.0-1.0)
    float achieved_precision_mm_;          // Final precision achieved
    int point_count_;                      // Number of points in cloud

    // Calibrated Camera Parameters (saved from auto-calibration)
    bool has_calibrated_params_;           // True if auto-calibration was run
    double calibrated_exposure_us_;        // Calibrated exposure time
    double calibrated_gain_;               // Calibrated gain

    // FPS Calculation
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point scan_start_time_;
    std::vector<float> fps_samples_;       // Rolling window for FPS averaging

    // Thread Management
    QFutureWatcher<bool>* scan_watcher_;   // Monitors background scan thread

    // Constants
    static constexpr int TARGET_FRAMES = 3;  // Reduced from 10 to 3 for faster testing
    static constexpr float STABILITY_THRESHOLD = 0.90f;  // 90% stability required
    static constexpr int UI_UPDATE_HZ = 30;
    static constexpr int FPS_SAMPLE_COUNT = 10;
};

} // namespace gui
} // namespace unlook
