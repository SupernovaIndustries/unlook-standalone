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
    explicit DatasetCaptureWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent = nullptr);
    ~DatasetCaptureWidget();

signals:
    void datasetCaptureCompleted(const QString& datasetPath);

private slots:
    void onStartCapture();
    void onCaptureFrame();
    void onPatternTypeChanged(int index);

private:
    void setupUi();
    void startPreviewCapture();
    void stopPreviewCapture();
    void updatePreview(const core::StereoFramePair& frame_pair);
    void captureAndSaveFrame();
    void saveDatasetInfo();
    void updatePatternDetector();

    // Preview
    QLabel* leftPreview_;
    QLabel* rightPreview_;
    QLabel* patternOverlayLeft_;
    QLabel* patternOverlayRight_;

    // Pattern configuration controls
    QComboBox* patternTypeCombo_;
    QSpinBox* rowsSpinBox_;
    QSpinBox* colsSpinBox_;
    QDoubleSpinBox* squareSizeSpinBox_;
    QDoubleSpinBox* arucoSizeSpinBox_;

    // Capture controls
    QPushButton* startCaptureButton_;
    QProgressBar* captureProgress_;
    QLabel* statusLabel_;
    QLabel* detectionStatusLabel_;

    // Capture state
    bool isCapturing_;
    bool previewActive_;  // Preview using camera callback system
    int captureCount_;
    int targetCaptures_;
    QTimer* captureTimer_;

    // Camera and LED
    std::shared_ptr<camera::CameraSystem> cameraSystem_;
    std::shared_ptr<hardware::AS1170Controller> ledController_;

    // Latest frame from preview callback (for capture)
    core::StereoFramePair latestFramePair_;
    std::mutex latestFrameMutex_;  // Thread safety for latest frame access

    // Pattern detector
    std::unique_ptr<calibration::PatternDetector> patternDetector_;
    std::mutex patternDetectorMutex_;  // Thread safety for pattern detector access

    // Dataset storage
    QString currentDatasetPath_;
    nlohmann::json datasetInfo_;
};

} // namespace gui
} // namespace unlook
