#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QTextEdit>
#include <QGroupBox>
#include <QTableWidget>
#include <QThread>
#include <memory>
#include "unlook/calibration/StereoCalibrationProcessor.hpp"

namespace unlook {
namespace gui {

/**
 * @brief Dataset processing widget - process calibration dataset
 *
 * Features:
 * - Load latest or selected dataset
 * - Background threaded calibration processing
 * - Real-time log output
 * - Calibration results display with color-coded quality
 * - Statistics table
 * - Auto-save calibration and set as system default
 */
class DatasetProcessingWidget : public QWidget {
    Q_OBJECT

public:
    explicit DatasetProcessingWidget(QWidget* parent = nullptr);
    ~DatasetProcessingWidget();

public slots:
    void loadAndProcessDataset(const QString& datasetPath = "");

private slots:
    void onProcessDataset();
    void onSelectDataset();
    void onProcessingComplete(const calibration::CalibrationResult& result);
    void onProcessingError(const QString& error);
    void onProgressMessage(const QString& message);

private:
    void setupUi();
    void loadLatestDataset();
    void displayResults(const calibration::CalibrationResult& result);
    void updateLog(const QString& message);
    void clearResults();

    // Dataset info display
    QLabel* datasetPathLabel_;
    QLabel* datasetTimestampLabel_;
    QLabel* imagePairsLabel_;
    QLabel* patternInfoLabel_;

    // Processing controls
    QPushButton* processButton_;
    QPushButton* selectDatasetButton_;
    QProgressBar* processingProgress_;

    // Log output
    QTextEdit* logOutput_;

    // Results display
    QGroupBox* resultsGroup_;
    QLabel* rmsErrorLabel_;
    QLabel* baselineLabel_;
    QLabel* epipolarErrorLabel_;
    QLabel* qualityStatusLabel_;

    // Statistics table
    QTableWidget* statisticsTable_;

    // Processor
    std::unique_ptr<calibration::StereoCalibrationProcessor> processor_;

    // Processing thread
    QThread* processingThread_;

    // Current dataset
    QString currentDatasetPath_;
};

// Worker class for background processing
class CalibrationWorker : public QObject {
    Q_OBJECT

public:
    explicit CalibrationWorker(calibration::StereoCalibrationProcessor* processor,
                              const QString& datasetPath)
        : processor_(processor), datasetPath_(datasetPath) {}

public slots:
    void process();

signals:
    void progressMessage(const QString& message);
    void finished(const calibration::CalibrationResult& result);
    void error(const QString& error);

private:
    calibration::StereoCalibrationProcessor* processor_;
    QString datasetPath_;
};

} // namespace gui
} // namespace unlook
