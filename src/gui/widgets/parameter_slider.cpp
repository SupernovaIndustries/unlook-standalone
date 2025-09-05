#include "unlook/gui/widgets/parameter_slider.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include <QLabel>
#include <QPushButton>
#include <QSpacerItem>

using namespace unlook::gui::styles;

namespace unlook {
namespace gui {
namespace widgets {

ParameterSlider::ParameterSlider(const QString& label,
                                double min_value,
                                double max_value, 
                                double initial_value,
                                ValueType type,
                                QWidget* parent)
    : QWidget(parent)
    , value_type_(type)
    , min_value_(min_value)
    , max_value_(max_value)
    , current_value_(initial_value)
    , single_step_(1.0)
    , decimals_(2)
    , updating_from_slider_(false)
    , updating_from_spinbox_(false)
{
    initializeWidget();
    setValue(initial_value);
}

double ParameterSlider::getValue() const {
    return current_value_;
}

void ParameterSlider::setValue(double value) {
    if (value < min_value_) value = min_value_;
    if (value > max_value_) value = max_value_;
    
    current_value_ = value;
    
    if (!updating_from_slider_) {
        updateSliderFromValue();
    }
    
    if (!updating_from_spinbox_) {
        if (value_type_ == ValueType::INTEGER && int_spinbox_) {
            int_spinbox_->setValue(static_cast<int>(value));
        } else if (value_type_ == ValueType::FLOATING_POINT && double_spinbox_) {
            double_spinbox_->setValue(value);
        }
    }
    
    emit valueChanged(current_value_);
}

void ParameterSlider::setRange(double min_value, double max_value) {
    min_value_ = min_value;
    max_value_ = max_value;
    
    if (slider_) {
        // Keep current slider setup
    }
    
    if (int_spinbox_) {
        int_spinbox_->setRange(static_cast<int>(min_value), static_cast<int>(max_value));
    }
    if (double_spinbox_) {
        double_spinbox_->setRange(min_value, max_value);
    }
    
    // Clamp current value to new range
    if (current_value_ < min_value_ || current_value_ > max_value_) {
        setValue(qBound(min_value_, current_value_, max_value_));
    }
}

void ParameterSlider::setDecimals(int decimals) {
    decimals_ = decimals;
    if (double_spinbox_) {
        double_spinbox_->setDecimals(decimals);
    }
}

void ParameterSlider::setSingleStep(double step) {
    single_step_ = step;
    if (int_spinbox_) {
        int_spinbox_->setSingleStep(static_cast<int>(step));
    }
    if (double_spinbox_) {
        double_spinbox_->setSingleStep(step);
    }
}

void ParameterSlider::addPresetButton(const QString& label, double value) {
    QPushButton* preset_button = new QPushButton(label, this);
    preset_button->setStyleSheet(SupernovaStyle::getTouchButtonStyle(
        SupernovaStyle::NEBULA_SURFACE, SupernovaStyle::TEXT_SECONDARY, 30));
    
    connect(preset_button, &QPushButton::clicked, [this, value]() {
        setValue(value);
    });
    
    preset_buttons_.append(preset_button);
    if (preset_layout_) {
        preset_layout_->addWidget(preset_button);
    }
}

void ParameterSlider::clearPresetButtons() {
    for (QPushButton* button : preset_buttons_) {
        button->deleteLater();
    }
    preset_buttons_.clear();
}

void ParameterSlider::setEnabled(bool enabled) {
    QWidget::setEnabled(enabled);
    if (slider_) slider_->setEnabled(enabled);
    if (int_spinbox_) int_spinbox_->setEnabled(enabled);
    if (double_spinbox_) double_spinbox_->setEnabled(enabled);
}

void ParameterSlider::setSuffix(const QString& suffix) {
    suffix_ = suffix;
    if (int_spinbox_) int_spinbox_->setSuffix(suffix);
    if (double_spinbox_) double_spinbox_->setSuffix(suffix);
}

void ParameterSlider::setValueInputVisible(bool visible) {
    if (int_spinbox_) int_spinbox_->setVisible(visible && value_type_ == ValueType::INTEGER);
    if (double_spinbox_) double_spinbox_->setVisible(visible && value_type_ == ValueType::FLOATING_POINT);
}

void ParameterSlider::onSliderValueChanged(int value) {
    if (updating_from_spinbox_) return;
    
    updating_from_slider_ = true;
    double actual_value = sliderPositionToValue(value);
    current_value_ = actual_value;
    
    if (value_type_ == ValueType::INTEGER && int_spinbox_) {
        int_spinbox_->setValue(static_cast<int>(actual_value));
    } else if (value_type_ == ValueType::FLOATING_POINT && double_spinbox_) {
        double_spinbox_->setValue(actual_value);
    }
    
    emit valueChanged(current_value_);
    updating_from_slider_ = false;
}

void ParameterSlider::onIntSpinBoxValueChanged(int value) {
    if (updating_from_slider_) return;
    
    updating_from_spinbox_ = true;
    setValue(static_cast<double>(value));
    updating_from_spinbox_ = false;
}

void ParameterSlider::onDoubleSpinBoxValueChanged(double value) {
    if (updating_from_slider_) return;
    
    updating_from_spinbox_ = true;
    setValue(value);
    updating_from_spinbox_ = false;
}

void ParameterSlider::onPresetButtonClicked() {
    // Handled in lambda connections
}

void ParameterSlider::initializeWidget() {
    main_layout_ = new QVBoxLayout(this);
    main_layout_->setSpacing(SupernovaStyle::Spacing::PADDING_SMALL);
    
    // Create label
    label_ = new QLabel(this);
    label_->setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY, SupernovaStyle::FontWeight::MEDIUM));
    label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_PRIMARY)));
    main_layout_->addWidget(label_);
    
    // Create control layout
    control_layout_ = new QHBoxLayout();
    
    // Create slider
    slider_ = new QSlider(Qt::Horizontal, this);
    slider_->setRange(0, SLIDER_RESOLUTION);
    slider_->setStyleSheet(SupernovaStyle::getSliderStyle());
    connect(slider_, &QSlider::valueChanged, this, &ParameterSlider::onSliderValueChanged);
    control_layout_->addWidget(slider_, 1);
    
    // Create spinbox based on type
    if (value_type_ == ValueType::INTEGER) {
        int_spinbox_ = new QSpinBox(this);
        int_spinbox_->setRange(static_cast<int>(min_value_), static_cast<int>(max_value_));
        connect(int_spinbox_, QOverload<int>::of(&QSpinBox::valueChanged), 
                this, &ParameterSlider::onIntSpinBoxValueChanged);
        control_layout_->addWidget(int_spinbox_);
        double_spinbox_ = nullptr;
    } else {
        double_spinbox_ = new QDoubleSpinBox(this);
        double_spinbox_->setRange(min_value_, max_value_);
        double_spinbox_->setDecimals(decimals_);
        connect(double_spinbox_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), 
                this, &ParameterSlider::onDoubleSpinBoxValueChanged);
        control_layout_->addWidget(double_spinbox_);
        int_spinbox_ = nullptr;
    }
    
    main_layout_->addLayout(control_layout_);
    
    // Create preset layout
    preset_layout_ = new QHBoxLayout();
    main_layout_->addLayout(preset_layout_);
}

void ParameterSlider::updateSliderFromValue() {
    if (slider_) {
        int slider_pos = valueToSliderPosition(current_value_);
        slider_->setValue(slider_pos);
    }
}

void ParameterSlider::updateValueFromSlider() {
    if (slider_) {
        current_value_ = sliderPositionToValue(slider_->value());
    }
}

double ParameterSlider::sliderPositionToValue(int position) const {
    double normalized = static_cast<double>(position) / SLIDER_RESOLUTION;
    return min_value_ + normalized * (max_value_ - min_value_);
}

int ParameterSlider::valueToSliderPosition(double value) const {
    double normalized = (value - min_value_) / (max_value_ - min_value_);
    return static_cast<int>(normalized * SLIDER_RESOLUTION);
}

} // namespace widgets
} // namespace gui
} // namespace unlook