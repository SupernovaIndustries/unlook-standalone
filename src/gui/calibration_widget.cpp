#include "unlook/gui/calibration_widget.hpp"
#include "unlook/gui/dataset_capture_widget.hpp"
#include "unlook/gui/dataset_processing_widget.hpp"
#include "unlook/camera/CameraSystem.hpp"
#include <QVBoxLayout>

namespace unlook {
namespace gui {

CalibrationWidget::CalibrationWidget(std::shared_ptr<camera::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , tabWidget_(new QTabWidget(this))
    , captureWidget_(new DatasetCaptureWidget(camera_system, this))
    , processingWidget_(new DatasetProcessingWidget(this))
    , camera_system_(camera_system)
{
    // Create main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // Add tabs
    tabWidget_->addTab(captureWidget_, "Dataset Capture");
    tabWidget_->addTab(processingWidget_, "Dataset Processing");

    mainLayout->addWidget(tabWidget_);

    // Connect capture completion to auto-switch tabs
    connect(captureWidget_, &DatasetCaptureWidget::datasetCaptureCompleted,
            this, [this](const QString& path) {
                // Switch to processing tab
                tabWidget_->setCurrentIndex(1);
                // Load the dataset
                processingWidget_->loadAndProcessDataset(path);
            });
}

} // namespace gui
} // namespace unlook
