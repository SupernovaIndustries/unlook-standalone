#pragma once

#include <QWidget>
#include <QTabWidget>
#include <memory>

namespace unlook {

// Forward declarations
namespace camera { class CameraSystem; }

namespace gui {

// Forward declarations
class DatasetCaptureWidget;
class DatasetProcessingWidget;

/**
 * @brief Main calibration widget - tab container
 *
 * Contains two tabs:
 * 1. Dataset Capture - Capture stereo image pairs with pattern detection
 * 2. Dataset Processing - Process dataset and compute calibration
 *
 * Auto-switches to processing tab when capture completes.
 */
class CalibrationWidget : public QWidget {
    Q_OBJECT

public:
    explicit CalibrationWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system, QWidget* parent = nullptr);
    ~CalibrationWidget() = default;

private:
    QTabWidget* tabWidget_;
    DatasetCaptureWidget* captureWidget_;
    DatasetProcessingWidget* processingWidget_;
    std::shared_ptr<camera::gui::CameraSystem> camera_system_;
};

} // namespace gui
} // namespace unlook
