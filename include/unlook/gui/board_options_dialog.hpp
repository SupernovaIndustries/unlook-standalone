#pragma once

#include <QDialog>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include "unlook/calibration/StereoCalibrationProcessor.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class BoardOptionsDialog; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

/**
 * @brief Dialog for configuring calibration board pattern options
 *
 * Features:
 * - Pattern type selection (Checkerboard/ChArUco/Circle Grid)
 * - ArUco dictionary selection (for ChArUco)
 * - Pattern dimensions (rows, columns)
 * - Physical sizes (square size, ArUco marker size)
 * - Touch-friendly interface with large buttons
 */
class BoardOptionsDialog : public QDialog {
    Q_OBJECT

public:
    explicit BoardOptionsDialog(QWidget* parent = nullptr);
    ~BoardOptionsDialog();

    /**
     * @brief Get current pattern configuration
     * @return PatternConfig with all current settings
     */
    calibration::PatternConfig getPatternConfig() const;

    /**
     * @brief Set pattern configuration
     * @param config Pattern configuration to apply
     */
    void setPatternConfig(const calibration::PatternConfig& config);

signals:
    /**
     * @brief Emitted when configuration changes
     */
    void configurationChanged();

private slots:
    void onPatternTypeChanged(int index);
    void onArucoDictChanged(int index);

private:
    void setupConnections();

    // UI from .ui file
    Ui::BoardOptionsDialog* ui;

    // Widget pointers for convenience
    QComboBox* patternTypeCombo_;
    QComboBox* arucoDictCombo_;
    QSpinBox* rowsSpinBox_;
    QSpinBox* colsSpinBox_;
    QDoubleSpinBox* squareSizeSpinBox_;
    QDoubleSpinBox* arucoSizeSpinBox_;
};

} // namespace gui
} // namespace unlook
