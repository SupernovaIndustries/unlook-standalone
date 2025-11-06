#include "unlook/gui/dataset_processing_widget.hpp"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QDateTime>
#include <QThread>
#include <QMetaType>
#include <fstream>
#include <iostream>

namespace unlook {
namespace gui {

DatasetProcessingWidget::DatasetProcessingWidget(QWidget* parent)
    : QWidget(parent)
    , processingThread_(nullptr)
{
    // Register CalibrationResult for Qt signal/slot across threads
    qRegisterMetaType<calibration::CalibrationResult>("calibration::CalibrationResult");

    processor_ = std::make_unique<calibration::StereoCalibrationProcessor>();
    setupUi();
}

DatasetProcessingWidget::~DatasetProcessingWidget() {
    if (processingThread_ && processingThread_->isRunning()) {
        processingThread_->quit();
        processingThread_->wait();
    }
}

void DatasetProcessingWidget::setupUi() {
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Dataset info
    QGroupBox* infoGroup = new QGroupBox("Dataset Information");
    QFormLayout* infoLayout = new QFormLayout();

    datasetPathLabel_ = new QLabel("No dataset loaded");
    datasetTimestampLabel_ = new QLabel("-");
    imagePairsLabel_ = new QLabel("-");
    patternInfoLabel_ = new QLabel("-");

    infoLayout->addRow("Path:", datasetPathLabel_);
    infoLayout->addRow("Timestamp:", datasetTimestampLabel_);
    infoLayout->addRow("Image Pairs:", imagePairsLabel_);
    infoLayout->addRow("Pattern:", patternInfoLabel_);

    infoGroup->setLayout(infoLayout);
    mainLayout->addWidget(infoGroup);

    // Processing controls
    QHBoxLayout* controlsLayout = new QHBoxLayout();

    selectDatasetButton_ = new QPushButton("Select Dataset...");
    selectDatasetButton_->setStyleSheet("background-color: #6366f1; color: white; padding: 10px; font-size: 12pt;");
    connect(selectDatasetButton_, &QPushButton::clicked, this, &DatasetProcessingWidget::onSelectDataset);

    processButton_ = new QPushButton("Process Dataset");
    processButton_->setStyleSheet("background-color: #059669; color: white; padding: 10px; font-size: 14pt; font-weight: bold;");
    processButton_->setEnabled(false);
    connect(processButton_, &QPushButton::clicked, this, &DatasetProcessingWidget::onProcessDataset);

    controlsLayout->addWidget(selectDatasetButton_);
    controlsLayout->addWidget(processButton_);
    mainLayout->addLayout(controlsLayout);

    // Progress bar
    processingProgress_ = new QProgressBar();
    processingProgress_->setRange(0, 0);  // Indeterminate
    processingProgress_->setVisible(false);
    mainLayout->addWidget(processingProgress_);

    // Log output
    QLabel* logLabel = new QLabel("Processing Log:");
    logLabel->setStyleSheet("font-weight: bold;");
    mainLayout->addWidget(logLabel);

    logOutput_ = new QTextEdit();
    logOutput_->setReadOnly(true);
    logOutput_->setStyleSheet("background-color: #1e1e1e; color: #00ff00; font-family: monospace;");
    logOutput_->setMinimumHeight(150);
    mainLayout->addWidget(logOutput_);

    // Results display
    resultsGroup_ = new QGroupBox("Calibration Results");
    resultsGroup_->setVisible(false);
    QVBoxLayout* resultsLayout = new QVBoxLayout();

    rmsErrorLabel_ = new QLabel("-");
    baselineLabel_ = new QLabel("-");
    epipolarErrorLabel_ = new QLabel("-");
    qualityStatusLabel_ = new QLabel("-");

    QFormLayout* resultsFormLayout = new QFormLayout();
    resultsFormLayout->addRow("RMS Error:", rmsErrorLabel_);
    resultsFormLayout->addRow("Baseline:", baselineLabel_);
    resultsFormLayout->addRow("Epipolar Error:", epipolarErrorLabel_);
    resultsFormLayout->addRow("Quality Status:", qualityStatusLabel_);
    resultsLayout->addLayout(resultsFormLayout);

    resultsGroup_->setLayout(resultsLayout);
    mainLayout->addWidget(resultsGroup_);

    // Statistics table
    statisticsTable_ = new QTableWidget();
    statisticsTable_->setVisible(false);
    mainLayout->addWidget(statisticsTable_);
}

void DatasetProcessingWidget::loadAndProcessDataset(const QString& datasetPath) {
    currentDatasetPath_ = datasetPath;

    // Load dataset info
    QString jsonPath = datasetPath + "/dataset_info.json";
    if (!QFile::exists(jsonPath)) {
        updateLog("ERROR: dataset_info.json not found");
        return;
    }

    try {
        std::ifstream file(jsonPath.toStdString());
        nlohmann::json datasetInfo;
        file >> datasetInfo;
        file.close();

        datasetPathLabel_->setText(datasetPath);
        datasetTimestampLabel_->setText(QString::fromStdString(
            datasetInfo["dataset_info"]["timestamp"]));
        imagePairsLabel_->setText(QString::number(
            datasetInfo["capture_config"]["target_image_pairs"].get<int>()));
        patternInfoLabel_->setText(QString::fromStdString(
            datasetInfo["pattern_config"]["type"]));

        processButton_->setEnabled(true);
        updateLog("Dataset loaded successfully");
    } catch (const std::exception& e) {
        updateLog(QString("ERROR loading dataset: ") + e.what());
    }
}

void DatasetProcessingWidget::onSelectDataset() {
    QString dir = QFileDialog::getExistingDirectory(this, "Select Dataset Directory",
                                                    "/unlook_calib_dataset");
    if (!dir.isEmpty()) {
        loadAndProcessDataset(dir);
    }
}

void DatasetProcessingWidget::onProcessDataset() {
    if (currentDatasetPath_.isEmpty()) {
        QMessageBox::warning(this, "No Dataset", "Please select a dataset first");
        return;
    }

    clearResults();
    updateLog("Starting calibration processing...");
    processingProgress_->setVisible(true);
    processButton_->setEnabled(false);

    // Create worker and thread
    processingThread_ = new QThread(this);
    CalibrationWorker* worker = new CalibrationWorker(processor_.get(), currentDatasetPath_);
    worker->moveToThread(processingThread_);

    connect(processingThread_, &QThread::started, worker, &CalibrationWorker::process);
    connect(worker, &CalibrationWorker::progressMessage, this, &DatasetProcessingWidget::onProgressMessage);
    connect(worker, &CalibrationWorker::finished, this, &DatasetProcessingWidget::onProcessingComplete);
    connect(worker, &CalibrationWorker::error, this, &DatasetProcessingWidget::onProcessingError);
    connect(worker, &CalibrationWorker::finished, processingThread_, &QThread::quit);
    connect(processingThread_, &QThread::finished, worker, &QObject::deleteLater);

    processingThread_->start();
}

void DatasetProcessingWidget::onProcessingComplete(const calibration::CalibrationResult& result) {
    processingProgress_->setVisible(false);
    processButton_->setEnabled(true);
    updateLog("Calibration complete!");

    // Save calibration to YAML file
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString calibPath = QString("/unlook_calib/calib-%1.yaml").arg(timestamp);

    updateLog("Saving calibration to: " + calibPath);
    bool saved = processor_->saveCalibration(result, calibPath.toStdString());

    if (saved) {
        updateLog("✓ Calibration saved successfully!");

        // If calibration passed all quality checks, set as system default
        if (result.qualityPassed) {
            updateLog("Quality checks PASSED - setting as system default...");
            bool setDefault = processor_->setAsSystemDefault(calibPath.toStdString());
            if (setDefault) {
                updateLog("✓ Set as system default calibration: /unlook_calib/default.yaml");
            } else {
                updateLog("⚠ Warning: Could not set as system default");
            }
        } else {
            updateLog("⚠ Quality checks FAILED - NOT setting as system default");
            updateLog("  Fix issues and recalibrate to get a system-ready calibration");
        }
    } else {
        updateLog("✗ ERROR: Failed to save calibration file!");
    }

    displayResults(result);
}

void DatasetProcessingWidget::onProcessingError(const QString& error) {
    processingProgress_->setVisible(false);
    processButton_->setEnabled(true);
    updateLog("ERROR: " + error);
    QMessageBox::critical(this, "Calibration Error", error);
}

void DatasetProcessingWidget::onProgressMessage(const QString& message) {
    updateLog(message);
}

void DatasetProcessingWidget::displayResults(const calibration::CalibrationResult& result) {
    resultsGroup_->setVisible(true);

    // RMS Error (color-coded)
    QString rmsStyle = result.rmsReprojectionError < 0.3 ? "color: #00ff00;" :
                      result.rmsReprojectionError < 0.6 ? "color: #ffa500;" : "color: #ff0000;";
    rmsErrorLabel_->setText(QString::number(result.rmsReprojectionError, 'f', 3) + " px");
    rmsErrorLabel_->setStyleSheet(rmsStyle);

    // Baseline (color-coded)
    QString baselineStyle = result.baselineErrorMM < 0.5 ? "color: #00ff00;" : "color: #ffa500;";
    baselineLabel_->setText(QString::number(result.baselineMM, 'f', 2) + " mm (error: " +
                           QString::number(result.baselineErrorMM, 'f', 2) + " mm)");
    baselineLabel_->setStyleSheet(baselineStyle);

    // Epipolar Error
    QString epiStyle = result.meanEpipolarError < 0.5 ? "color: #00ff00;" : "color: #ffa500;";
    epipolarErrorLabel_->setText(QString::number(result.meanEpipolarError, 'f', 3) + " px");
    epipolarErrorLabel_->setStyleSheet(epiStyle);

    // Overall quality
    QString qualityText = result.qualityPassed ? "✓ PASS" : "✗ FAIL";
    QString qualityStyle = result.qualityPassed ? "color: #00ff00; font-weight: bold; font-size: 14pt;" :
                                                  "color: #ff0000; font-weight: bold; font-size: 14pt;";
    qualityStatusLabel_->setText(qualityText);
    qualityStatusLabel_->setStyleSheet(qualityStyle);

    updateLog(QString("Final RMS Error: %1 px").arg(result.rmsReprojectionError, 0, 'f', 3));
    updateLog(QString("Baseline: %1 mm").arg(result.baselineMM, 0, 'f', 2));
    updateLog(QString("Epipolar Error: %1 px").arg(result.meanEpipolarError, 0, 'f', 3));
}

void DatasetProcessingWidget::updateLog(const QString& message) {
    logOutput_->append("[" + QTime::currentTime().toString() + "] " + message);
}

void DatasetProcessingWidget::clearResults() {
    resultsGroup_->setVisible(false);
    logOutput_->clear();
}

// CalibrationWorker implementation
void CalibrationWorker::process() {
    try {
        std::cout << "==========================================" << std::endl;
        std::cout << "WORKER STARTED - Thread ID: " << QThread::currentThreadId() << std::endl;
        std::cout << "Dataset path: " << datasetPath_.toStdString() << std::endl;
        std::cout << "Processor ptr: " << (void*)processor_ << std::endl;
        std::cout << "==========================================" << std::endl;
        std::cout.flush();

        // Setup progress callback to emit messages to GUI
        processor_->setProgressCallback([this](const std::string& message) {
            std::cout << "[CALLBACK] " << message << std::endl;
            std::cout.flush();
            emit progressMessage(QString::fromStdString(message));
        });

        std::cout << "Callback set, emitting start message..." << std::endl;
        std::cout.flush();

        emit progressMessage("Starting calibration...");

        std::cout << "Calling calibrateFromDataset()..." << std::endl;
        std::cout.flush();

        auto result = processor_->calibrateFromDataset(datasetPath_.toStdString());

        std::cout << "calibrateFromDataset() returned!" << std::endl;
        std::cout.flush();

        emit finished(result);
    } catch (const std::exception& e) {
        std::cout << "EXCEPTION: " << e.what() << std::endl;
        std::cout.flush();
        emit error(QString("Calibration failed: ") + e.what());
    } catch (...) {
        std::cout << "UNKNOWN EXCEPTION!" << std::endl;
        std::cout.flush();
        emit error("Unknown error during calibration");
    }
}

} // namespace gui
} // namespace unlook
