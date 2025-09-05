#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include <QMouseEvent>
#include <QGraphicsDropShadowEffect>
#include <QTimer>
#include <QtGlobal>

using namespace unlook::gui::styles;

namespace unlook {
namespace gui {
namespace widgets {

TouchButton::TouchButton(const QString& text, ButtonType type, QWidget* parent)
    : QPushButton(text, parent)
    , button_type_(type)
    , animation_enabled_(true)
    , using_custom_colors_(false)
    , press_animation_(nullptr)
    , release_animation_(nullptr)
    , shadow_effect_(nullptr)
{
    initializeButton();
}

TouchButton::TouchButton(const QIcon& icon, const QString& text, ButtonType type, QWidget* parent)
    : QPushButton(icon, text, parent)
    , button_type_(type)
    , animation_enabled_(true)
    , using_custom_colors_(false)
    , press_animation_(nullptr)
    , release_animation_(nullptr)
    , shadow_effect_(nullptr)
{
    initializeButton();
}

void TouchButton::setButtonType(ButtonType type) {
    button_type_ = type;
    using_custom_colors_ = false;
    updateStyleSheet();
}

void TouchButton::setTouchHeight(int height) {
    // Ensure height meets minimum touch target requirements
    int final_height = qMax(height, SupernovaStyle::Spacing::TOUCH_TARGET_MIN);
    setMinimumHeight(final_height);
}

void TouchButton::setCompactSize() {
    // Compact size optimized for dense UI areas
    // Still maintains minimum 44px touch target (iOS standard)
    const int COMPACT_HEIGHT = 44;
    const int COMPACT_WIDTH = 100;
    
    // Use setFixedSize for maximum stability - prevents any resizing
    setFixedSize(COMPACT_WIDTH, COMPACT_HEIGHT);
    
    // Smaller font for compact buttons
    setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::SMALL, SupernovaStyle::FontWeight::MEDIUM));
    
    // Update stylesheet after size change
    updateStyleSheet();
}

void TouchButton::setCustomColor(const QColor& background, const QColor& text) {
    custom_background_ = background;
    custom_text_ = text.isValid() ? text : SupernovaStyle::TEXT_PRIMARY;
    using_custom_colors_ = true;
    updateStyleSheet();
}

void TouchButton::mousePressEvent(QMouseEvent* event) {
    // Visual feedback handled by CSS :pressed state
    // No geometry animations to prevent button displacement
    QPushButton::mousePressEvent(event);
}

void TouchButton::mouseReleaseEvent(QMouseEvent* event) {
    // Visual feedback handled by CSS :hover state
    // No geometry animations to prevent button displacement
    QPushButton::mouseReleaseEvent(event);
}

void TouchButton::enterEvent(QEvent* event) {
    // Hover effects handled entirely by CSS to prevent infinite scaling
    QPushButton::enterEvent(event);
}

void TouchButton::leaveEvent(QEvent* event) {
    // CSS automatically handles :hover state removal
    QPushButton::leaveEvent(event);
}

void TouchButton::onAnimationFinished() {
    // Animation completed
}

void TouchButton::initializeButton() {
    // Set stable minimum sizes for touch targets
    // These will be overridden by setFixedSize() or setCompactSize() calls
    if (maximumHeight() == QWIDGETSIZE_MAX) {
        setMinimumHeight(SupernovaStyle::Spacing::TOUCH_TARGET_MIN);
    }
    if (maximumWidth() == QWIDGETSIZE_MAX) {
        setMinimumWidth(120);
    }
    
    // Set font
    setFont(SupernovaStyle::getFont(SupernovaStyle::FontSize::BODY, SupernovaStyle::FontWeight::MEDIUM));
    
    // DISABLED geometry animations - they cause position/size instability
    // Visual feedback is now handled entirely through CSS styles
    press_animation_ = nullptr;
    release_animation_ = nullptr;
    
    // Add shadow effect for professional appearance
    shadow_effect_ = new QGraphicsDropShadowEffect(this);
    shadow_effect_->setBlurRadius(8);
    shadow_effect_->setOffset(0, 2);
    shadow_effect_->setColor(QColor(0, 0, 0, 60));
    setGraphicsEffect(shadow_effect_);
    
    // Ensure button doesn't accept focus to prevent unexpected behavior
    setFocusPolicy(Qt::NoFocus);
    
    // Apply initial styling
    updateStyleSheet();
}

void TouchButton::updateStyleSheet() {
    QColor background_color;
    QColor text_color = SupernovaStyle::TEXT_PRIMARY;
    
    if (using_custom_colors_) {
        background_color = custom_background_;
        text_color = custom_text_;
    } else {
        switch (button_type_) {
            case ButtonType::PRIMARY:
                background_color = SupernovaStyle::ELECTRIC_PRIMARY;
                break;
            case ButtonType::SECONDARY:
                background_color = SupernovaStyle::PLASMA_ACCENT;
                break;
            case ButtonType::SUCCESS:
                background_color = SupernovaStyle::SUCCESS_STATE;
                break;
            case ButtonType::WARNING:
                background_color = SupernovaStyle::WARNING_STATE;
                break;
            case ButtonType::ERROR:
                background_color = SupernovaStyle::ERROR_STATE;
                break;
        }
    }
    
    setStyleSheet(SupernovaStyle::getTouchButtonStyle(background_color, text_color, minimumHeight()));
}

// DISABLED: Geometry animations removed to fix button displacement and sizing bugs
// All visual feedback is now handled through stable CSS styling

} // namespace widgets
} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake