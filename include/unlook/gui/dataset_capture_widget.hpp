#pragma once

#include <QWidget>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QProgressBar>
#include <QTimer>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include "unlook/calibration/PatternDetector.hpp"
#include "unlook/core/types.hpp"
#include "unlook/camera/CameraSystemGUI.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class DatasetCaptureWidget; }
QT_END_NAMESPACE

namespace unlook {

// Forward declarations
namespace camera { class CameraSystem; }
namespace hardware { class AS1170Controller; }

namespace gui {

/**
 * @brief Dataset capture widget - capture stereo calibration image pairs
 *
 * Features:
 * - Dual camera preview with pattern detection overlay
 * - Pattern configuration (Checkerboard/ChArUco/Circle Grid)
 * - Auto-capture workflow (50 pairs with 5 second delay)
 * - Real-time pattern detection feedback
 * - VCSEL LED control for illumination
 * - Dataset saved with JSON metadata
 */
class DatasetCaptureWidget : public QWidget {
    Q_OBJECT

public:
    explicit DatasetCaptureWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system, QWidget* parent = nullptr);
    ~DatasetCaptureWidget();

protected:
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;

signals:
    void datasetCaptureCompleted(const QString& datasetPath);

private slots:
    void onStartCapture();
    void onCaptureFrame();
    void onCountdownTick();
    void onBoardOptionsClicked();
    void onBoardConfigChanged();

private:
    void setupUi();
    void setupConnections();
    void startPreviewCapture();
    void stopPreviewCapture();
    void updatePreview(const core::StereoFramePair& frame_pair);
    void captureAndSaveFrame();
    void saveDatasetInfo();
    void updatePatternDetector();

    // UI from .ui file
    Ui::DatasetCaptureWidget* ui;

    // Preview (pointers to widgets in ui)
    QLabel* leftPreview_;
    QLabel* rightPreview_;
    QLabel* patternOverlayLeft_;
    QLabel* patternOverlayRight_;

    // Capture controls (pointers to widgets in ui)
    QPushButton* boardOptionsButton_;
    QPushButton* startCaptureButton_;
    QProgressBar* captureProgress_;
    QLabel* detectionStatusLabel_;
    QLabel* countdownLabel_;

    // Capture state
    bool isCapturing_;
    bool previewActive_;  // Preview using camera callback system
    int captureCount_;
    int targetCaptures_;
    QTimer* captureTimer_;
    QTimer* countdownTimer_;
    int countdownValue_;

    // Camera and LED
    std::shared_ptr<camera::gui::CameraSystem> cameraSystem_;
    std::shared_ptr<hardware::AS1170Controller> ledController_;

    // Latest frame from preview callback (for capture)
    core::StereoFramePair latestFramePair_;
    std::mutex latestFrameMutex_;  // Thread safety for latest frame access

    // Pattern detector
    std::unique_ptr<calibration::PatternDetector> patternDetector_;
    std::mutex patternDetectorMutex_;  // Thread safety for pattern detector access
    calibration::PatternConfig currentPatternConfig_;  // Current pattern configuration (for board options dialog)

    // Dataset storage
    QString currentDatasetPath_;
    nlohmann::json datasetInfo_;

    // CRITICAL: Coverage tracking for guided calibration
    // Tracks which image regions have been covered by checkerboard
    // to ensure complete field-of-view calibration
    struct CoverageZone {
        std::string name;          // Zone name (e.g., "TOP-LEFT", "CENTER")
        cv::Rect rect;             // Zone rectangle in image coordinates
        int captureCount;          // Number of captures covering this zone
        int targetCount;           // Target number of captures for this zone
        bool adequatelyCovered;    // True if captureCount >= targetCount
    };
    std::vector<CoverageZone> coverageZones_;  // 9 zones: 4 corners + 4 edges + center
    int totalFramesCaptured_;                  // Total frames captured for coverage tracking

    // Continuous detection tracking for countdown trigger
    std::chrono::steady_clock::time_point firstDetectionTime_;
    bool isDetectedContinuously_;
    double continuousDetectionSeconds_;

    void initializeCoverageZones();
    void updateCoverageTracking(const std::vector<cv::Point2f>& corners);
    void drawCoverageOverlay(cv::Mat& image, const std::vector<cv::Point2f>& corners);
    std::string getCoverageSummary() const;
};

} // namespace gui
} // namespace unlook
