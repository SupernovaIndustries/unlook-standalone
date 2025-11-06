#include "unlook/gui/board_options_dialog.hpp"
#include "ui_board_options_dialog.h"
#include <opencv2/aruco.hpp>
#include <QDebug>

namespace unlook {
namespace gui {

BoardOptionsDialog::BoardOptionsDialog(QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::BoardOptionsDialog)
{
    ui->setupUi(this);

    // Set widget pointers
    patternTypeCombo_ = ui->pattern_type_combo;
    arucoDictCombo_ = ui->aruco_dict_combo;
    rowsSpinBox_ = ui->rows_spinbox;
    colsSpinBox_ = ui->cols_spinbox;
    squareSizeSpinBox_ = ui->square_size_spinbox;
    arucoSizeSpinBox_ = ui->aruco_size_spinbox;

    // Set data for pattern type combo
    patternTypeCombo_->setItemData(0, static_cast<int>(calibration::PatternType::CHECKERBOARD));
    patternTypeCombo_->setItemData(1, static_cast<int>(calibration::PatternType::CHARUCO));
    patternTypeCombo_->setItemData(2, static_cast<int>(calibration::PatternType::CIRCLE_GRID));

    // Set default to ChArUco
    patternTypeCombo_->setCurrentIndex(1);

    // Set default ArUco dict to DICT_4X4_250 (index 2)
    arucoDictCombo_->setCurrentIndex(2);

    // Setup signal/slot connections
    setupConnections();
}

BoardOptionsDialog::~BoardOptionsDialog() {
    delete ui;
}

void BoardOptionsDialog::setupConnections() {
    // Pattern type cycle buttons
    connect(ui->pattern_prev_button, &QPushButton::clicked, [this]() {
        int idx = patternTypeCombo_->currentIndex();
        patternTypeCombo_->setCurrentIndex((idx - 1 + patternTypeCombo_->count()) % patternTypeCombo_->count());
    });
    connect(ui->pattern_next_button, &QPushButton::clicked, [this]() {
        int idx = patternTypeCombo_->currentIndex();
        patternTypeCombo_->setCurrentIndex((idx + 1) % patternTypeCombo_->count());
    });
    connect(patternTypeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &BoardOptionsDialog::onPatternTypeChanged);

    // ArUco dictionary cycle buttons
    connect(ui->dict_prev_button, &QPushButton::clicked, [this]() {
        int idx = arucoDictCombo_->currentIndex();
        arucoDictCombo_->setCurrentIndex((idx - 1 + arucoDictCombo_->count()) % arucoDictCombo_->count());
    });
    connect(ui->dict_next_button, &QPushButton::clicked, [this]() {
        int idx = arucoDictCombo_->currentIndex();
        arucoDictCombo_->setCurrentIndex((idx + 1) % arucoDictCombo_->count());
    });
    connect(arucoDictCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &BoardOptionsDialog::onArucoDictChanged);

    // Rows +/- buttons
    connect(ui->rows_minus_button, &QPushButton::clicked, [this]() {
        rowsSpinBox_->setValue(rowsSpinBox_->value() - 1);
    });
    connect(ui->rows_plus_button, &QPushButton::clicked, [this]() {
        rowsSpinBox_->setValue(rowsSpinBox_->value() + 1);
    });

    // Columns +/- buttons
    connect(ui->cols_minus_button, &QPushButton::clicked, [this]() {
        colsSpinBox_->setValue(colsSpinBox_->value() - 1);
    });
    connect(ui->cols_plus_button, &QPushButton::clicked, [this]() {
        colsSpinBox_->setValue(colsSpinBox_->value() + 1);
    });

    // Square size +/- buttons
    connect(ui->square_minus_button, &QPushButton::clicked, [this]() {
        squareSizeSpinBox_->setValue(squareSizeSpinBox_->value() - squareSizeSpinBox_->singleStep());
    });
    connect(ui->square_plus_button, &QPushButton::clicked, [this]() {
        squareSizeSpinBox_->setValue(squareSizeSpinBox_->value() + squareSizeSpinBox_->singleStep());
    });

    // ArUco size +/- buttons
    connect(ui->aruco_minus_button, &QPushButton::clicked, [this]() {
        arucoSizeSpinBox_->setValue(arucoSizeSpinBox_->value() - arucoSizeSpinBox_->singleStep());
    });
    connect(ui->aruco_plus_button, &QPushButton::clicked, [this]() {
        arucoSizeSpinBox_->setValue(arucoSizeSpinBox_->value() + arucoSizeSpinBox_->singleStep());
    });

    // Close button
    connect(ui->close_button, &QPushButton::clicked, [this]() {
        emit configurationChanged();
        accept();
    });

    // Emit signal on any value change
    connect(rowsSpinBox_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &BoardOptionsDialog::configurationChanged);
    connect(colsSpinBox_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &BoardOptionsDialog::configurationChanged);
    connect(squareSizeSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &BoardOptionsDialog::configurationChanged);
    connect(arucoSizeSpinBox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &BoardOptionsDialog::configurationChanged);

    // Initialize UI state based on current pattern type
    onPatternTypeChanged(patternTypeCombo_->currentIndex());
}

void BoardOptionsDialog::onPatternTypeChanged(int index) {
    qDebug() << "Pattern type changed to index:" << index;

    // Get the pattern type
    calibration::PatternType type = static_cast<calibration::PatternType>(
        patternTypeCombo_->currentData().toInt());

    // Show/hide ArUco-specific widgets based on pattern type
    bool showAruco = (type == calibration::PatternType::CHARUCO);

    // ArUco Dictionary row widgets
    ui->aruco_dict_label->setVisible(showAruco);
    ui->dict_prev_button->setVisible(showAruco);
    ui->aruco_dict_combo->setVisible(showAruco);
    ui->dict_next_button->setVisible(showAruco);

    // ArUco Size row widgets
    ui->aruco_size_label->setVisible(showAruco);
    ui->aruco_minus_button->setVisible(showAruco);
    ui->aruco_size_spinbox->setVisible(showAruco);
    ui->aruco_plus_button->setVisible(showAruco);

    qDebug() << "ArUco widgets visibility:" << (showAruco ? "SHOWN" : "HIDDEN");

    emit configurationChanged();
}

void BoardOptionsDialog::onArucoDictChanged(int index) {
    qDebug() << "ArUco dictionary changed to index:" << index;
    emit configurationChanged();
}

calibration::PatternConfig BoardOptionsDialog::getPatternConfig() const {
    calibration::PatternConfig config;

    // Pattern type
    config.type = static_cast<calibration::PatternType>(
        patternTypeCombo_->currentData().toInt());

    // Dimensions
    config.rows = rowsSpinBox_->value();
    config.cols = colsSpinBox_->value();
    config.squareSizeMM = static_cast<float>(squareSizeSpinBox_->value());
    config.arucoMarkerSizeMM = static_cast<float>(arucoSizeSpinBox_->value());

    // ArUco dictionary mapping
    const std::vector<cv::aruco::PREDEFINED_DICTIONARY_NAME> dictMap = {
        cv::aruco::DICT_4X4_50,
        cv::aruco::DICT_4X4_100,
        cv::aruco::DICT_4X4_250,
        cv::aruco::DICT_4X4_1000,
        cv::aruco::DICT_5X5_50,
        cv::aruco::DICT_5X5_100,
        cv::aruco::DICT_5X5_250,
        cv::aruco::DICT_5X5_1000,
        cv::aruco::DICT_6X6_50,
        cv::aruco::DICT_6X6_100,
        cv::aruco::DICT_6X6_250,
        cv::aruco::DICT_6X6_1000,
        cv::aruco::DICT_7X7_50,
        cv::aruco::DICT_7X7_100,
        cv::aruco::DICT_7X7_250,
        cv::aruco::DICT_7X7_1000,
        cv::aruco::DICT_ARUCO_ORIGINAL
    };

    int dictIndex = arucoDictCombo_->currentIndex();
    if (dictIndex >= 0 && dictIndex < static_cast<int>(dictMap.size())) {
        config.arucoDict = dictMap[dictIndex];
    } else {
        config.arucoDict = cv::aruco::DICT_4X4_250;  // Default fallback
    }

    return config;
}

void BoardOptionsDialog::setPatternConfig(const calibration::PatternConfig& config) {
    // Set pattern type
    int patternIndex = 0;
    switch (config.type) {
        case calibration::PatternType::CHECKERBOARD: patternIndex = 0; break;
        case calibration::PatternType::CHARUCO: patternIndex = 1; break;
        case calibration::PatternType::CIRCLE_GRID: patternIndex = 2; break;
    }
    patternTypeCombo_->setCurrentIndex(patternIndex);

    // Set dimensions
    rowsSpinBox_->setValue(config.rows);
    colsSpinBox_->setValue(config.cols);
    squareSizeSpinBox_->setValue(config.squareSizeMM);
    arucoSizeSpinBox_->setValue(config.arucoMarkerSizeMM);

    // Set ArUco dictionary (reverse mapping)
    const std::vector<cv::aruco::PREDEFINED_DICTIONARY_NAME> dictMap = {
        cv::aruco::DICT_4X4_50,
        cv::aruco::DICT_4X4_100,
        cv::aruco::DICT_4X4_250,
        cv::aruco::DICT_4X4_1000,
        cv::aruco::DICT_5X5_50,
        cv::aruco::DICT_5X5_100,
        cv::aruco::DICT_5X5_250,
        cv::aruco::DICT_5X5_1000,
        cv::aruco::DICT_6X6_50,
        cv::aruco::DICT_6X6_100,
        cv::aruco::DICT_6X6_250,
        cv::aruco::DICT_6X6_1000,
        cv::aruco::DICT_7X7_50,
        cv::aruco::DICT_7X7_100,
        cv::aruco::DICT_7X7_250,
        cv::aruco::DICT_7X7_1000,
        cv::aruco::DICT_ARUCO_ORIGINAL
    };

    for (size_t i = 0; i < dictMap.size(); ++i) {
        if (dictMap[i] == config.arucoDict) {
            arucoDictCombo_->setCurrentIndex(static_cast<int>(i));
            break;
        }
    }
}

} // namespace gui
} // namespace unlook
