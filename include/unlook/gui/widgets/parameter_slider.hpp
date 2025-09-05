#pragma once

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QList>

namespace unlook {
namespace gui {
namespace widgets {

/**
 * @brief Touch-optimized parameter slider with value display and input
 * 
 * Features:
 * - Large touch targets for industrial use
 * - Real-time value display and editing
 * - Integer and floating-point support
 * - Supernova-tech styling
 * - Preset value buttons
 */
class ParameterSlider : public QWidget {
    Q_OBJECT
    
public:
    enum class ValueType {
        INTEGER,
        FLOATING_POINT
    };
    
    /**
     * @brief Constructor
     * @param label Parameter label text
     * @param min_value Minimum value
     * @param max_value Maximum value
     * @param initial_value Initial value
     * @param type Value type (integer or floating-point)
     * @param parent Parent widget
     */
    explicit ParameterSlider(const QString& label,
                            double min_value,
                            double max_value, 
                            double initial_value,
                            ValueType type = ValueType::INTEGER,
                            QWidget* parent = nullptr);
    
    /**
     * @brief Get current value
     */
    double getValue() const;
    
    /**
     * @brief Set current value
     */
    void setValue(double value);
    
    /**
     * @brief Set value range
     */
    void setRange(double min_value, double max_value);
    
    /**
     * @brief Set number of decimal places for floating-point values
     */
    void setDecimals(int decimals);
    
    /**
     * @brief Set single step size
     */
    void setSingleStep(double step);
    
    /**
     * @brief Add preset value button
     */
    void addPresetButton(const QString& label, double value);
    
    /**
     * @brief Clear all preset buttons
     */
    void clearPresetButtons();
    
    /**
     * @brief Enable/disable the slider
     */
    void setEnabled(bool enabled);
    
    /**
     * @brief Set suffix for value display (e.g., "ms", "%", "px")
     */
    void setSuffix(const QString& suffix);
    
    /**
     * @brief Set whether to show value input box
     */
    void setValueInputVisible(bool visible);

signals:
    /**
     * @brief Emitted when value changes
     */
    void valueChanged(double value);
    
    /**
     * @brief Emitted when slider is pressed
     */
    void sliderPressed();
    
    /**
     * @brief Emitted when slider is released
     */
    void sliderReleased();

private slots:
    /**
     * @brief Handle slider value change
     */
    void onSliderValueChanged(int value);
    
    /**
     * @brief Handle integer spin box value change
     */
    void onIntSpinBoxValueChanged(int value);
    
    /**
     * @brief Handle double spin box value change
     */
    void onDoubleSpinBoxValueChanged(double value);
    
    /**
     * @brief Handle preset button clicked
     */
    void onPresetButtonClicked();

private:
    /**
     * @brief Initialize the widget layout and styling
     */
    void initializeWidget();
    
    /**
     * @brief Update slider position from current value
     */
    void updateSliderFromValue();
    
    /**
     * @brief Update value from slider position
     */
    void updateValueFromSlider();
    
    /**
     * @brief Convert slider position to actual value
     */
    double sliderPositionToValue(int position) const;
    
    /**
     * @brief Convert actual value to slider position
     */
    int valueToSliderPosition(double value) const;
    
    // Configuration
    ValueType value_type_;
    double min_value_;
    double max_value_;
    double current_value_;
    double single_step_;
    int decimals_;
    QString suffix_;
    
    // UI Components
    QVBoxLayout* main_layout_;
    QHBoxLayout* control_layout_;
    QHBoxLayout* preset_layout_;
    QLabel* label_;
    QSlider* slider_;
    QSpinBox* int_spinbox_;
    QDoubleSpinBox* double_spinbox_;
    
    // Preset buttons
    QList<QPushButton*> preset_buttons_;
    
    // Internal state
    bool updating_from_slider_;
    bool updating_from_spinbox_;
    static const int SLIDER_RESOLUTION = 10000; // High resolution for smooth operation
};

} // namespace widgets
} // namespace gui
} // namespace unlook