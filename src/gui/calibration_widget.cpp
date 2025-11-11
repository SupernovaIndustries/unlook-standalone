#include "unlook/gui/calibration_widget.hpp"
#include "unlook/gui/dataset_capture_widget.hpp"
#include "unlook/gui/dataset_processing_widget.hpp"
#include "unlook/camera/CameraSystem.hpp"
#include <QVBoxLayout>

namespace unlook {
namespace gui {

CalibrationWidget::CalibrationWidget(std::shared_ptr<camera::gui::CameraSystem> camera_system, QWidget* parent)
    : QWidget(parent)
    , tabWidget_(new QTabWidget(this))
    , captureWidget_(new DatasetCaptureWidget(camera_system, this))
    , processingWidget_(new DatasetProcessingWidget(this))
    , camera_system_(camera_system)
{
    // Create main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // Style the tab widget with neomorphism and larger tabs
    tabWidget_->setStyleSheet(
        "QTabWidget::pane {"
        "    border: none;"
        "    background-color: #e0e5ec;"
        "}"
        "QTabBar::tab {"
        "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,"
        "        stop:0 #f0f0f0, stop:1 #d0d0d0);"
        "    color: #475569;"
        "    border: none;"
        "    border-radius: 15px;"
        "    padding: 15px 40px;"
        "    margin: 5px;"
        "    font-size: 14pt;"
        "    font-weight: bold;"
        "    min-width: 200px;"
        "    /* Neomorphism shadow */"
        "    border-left: 2px solid #ffffff;"
        "    border-top: 2px solid #ffffff;"
        "    border-right: 2px solid #bebebe;"
        "    border-bottom: 2px solid #bebebe;"
        "}"
        "QTabBar::tab:selected {"
        "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,"
        "        stop:0 #8b5cf6, stop:1 #6d28d9);"
        "    color: white;"
        "    /* Pressed effect */"
        "    border-left: 2px solid #bebebe;"
        "    border-top: 2px solid #bebebe;"
        "    border-right: 2px solid #ffffff;"
        "    border-bottom: 2px solid #ffffff;"
        "}"
        "QTabBar::tab:hover:!selected {"
        "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:1, y2:1,"
        "        stop:0 #f5f5f5, stop:1 #d5d5d5);"
        "    color: #334155;"
        "}"
    );

    // Add tabs with icons/emoji
    tabWidget_->addTab(captureWidget_, "📸 Dataset Capture");
    tabWidget_->addTab(processingWidget_, "⚙️ Process Dataset");

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
