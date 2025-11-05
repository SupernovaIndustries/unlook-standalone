#pragma once

#include <QWidget>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QFuture>
#include <QFutureWatcher>
#include <memory>
#include <chrono>

#include "unlook/camera/camera_system.hpp"

namespace unlook {
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
     * @param camera_system Shared camera system for frame acquisition
     * @param parent Parent widget
     */
    explicit HandheldScanWidget(std::shared_ptr<camera::CameraSystem> camera_system,
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
     * @brief Setup UI components and layout
     */
    void setupUI();

    /**
     * @brief Apply Supernova styling to all components
     */
    void applySupernovanStyling();

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

    // UI Components
    QLabel* title_label_;                  // "Handheld Scan" title

    // Stability Section
    QLabel* stability_title_label_;        // "Stability" section label
    QProgressBar* stability_bar_;          // 0-100% stability indicator
    QLabel* stability_text_label_;         // "Hold steady..." / "Stable!"

    // Capture Progress Section
    QLabel* capture_title_label_;          // "Capture Progress" section label
    QProgressBar* capture_progress_bar_;   // Multi-frame progress (0-10)
    QLabel* capture_count_label_;          // "Frame X/10"

    // Metrics Section
    QLabel* metrics_title_label_;          // "Metrics" section label
    QLabel* fps_label_;                    // "FPS: X.X"
    QLabel* precision_label_;              // "Precision: X.XXmm"

    // Control Buttons
    QPushButton* scan_button_;             // "START HANDHELD SCAN"
    QPushButton* stop_button_;             // "STOP SCAN"

    // Status Section
    QLabel* status_label_;                 // Current pipeline status

    // Backend Integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    QTimer* update_timer_;                 // 30 Hz UI updates

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

    // FPS Calculation
    std::chrono::steady_clock::time_point last_frame_time_;
    std::chrono::steady_clock::time_point scan_start_time_;
    std::vector<float> fps_samples_;       // Rolling window for FPS averaging

    // Thread Management
    QFutureWatcher<bool>* scan_watcher_;   // Monitors background scan thread

    // Constants
    static constexpr int TARGET_FRAMES = 10;
    static constexpr float STABILITY_THRESHOLD = 0.90f;  // 90% stability required
    static constexpr int UI_UPDATE_HZ = 30;
    static constexpr int FPS_SAMPLE_COUNT = 10;
};

} // namespace gui
} // namespace unlook
