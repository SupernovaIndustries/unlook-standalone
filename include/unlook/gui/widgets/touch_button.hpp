#pragma once

#include <QPushButton>
#include <QPropertyAnimation>
#include <QGraphicsDropShadowEffect>

namespace unlook {
namespace gui {
namespace widgets {

/**
 * @brief Touch-optimized button with Supernova-tech styling and smooth animations
 * 
 * Features:
 * - Large touch targets (minimum 48px height)
 * - Smooth press/release animations
 * - Visual feedback for industrial use
 * - Consistent Supernova styling
 */
class TouchButton : public QPushButton {
    Q_OBJECT
    
public:
    enum class ButtonType {
        PRIMARY,    // Electric primary color
        SECONDARY,  // Plasma accent color
        SUCCESS,    // Success state color
        WARNING,    // Warning state color
        ERROR       // Error state color
    };
    
    /**
     * @brief Constructor
     * @param text Button text
     * @param type Button color type
     * @param parent Parent widget
     */
    explicit TouchButton(const QString& text = "", 
                        ButtonType type = ButtonType::PRIMARY, 
                        QWidget* parent = nullptr);
    
    /**
     * @brief Constructor with icon
     */
    TouchButton(const QIcon& icon, 
               const QString& text = "", 
               ButtonType type = ButtonType::PRIMARY,
               QWidget* parent = nullptr);
    
    /**
     * @brief Set button type (changes colors)
     */
    void setButtonType(ButtonType type);
    
    /**
     * @brief Get current button type
     */
    ButtonType getButtonType() const { return button_type_; }
    
    /**
     * @brief Set minimum height for touch targets
     */
    void setTouchHeight(int height);
    
    /**
     * @brief Set compact size for dense UI areas (camera preview, controls)
     * Reduces button size while maintaining touch accessibility
     */
    void setCompactSize();
    
    /**
     * @brief Enable/disable press animation
     */
    void setPressAnimationEnabled(bool enabled) { animation_enabled_ = enabled; }
    
    /**
     * @brief Set custom background color
     */
    void setCustomColor(const QColor& background, const QColor& text = QColor());

protected:
    /**
     * @brief Override mouse press event for animation
     */
    void mousePressEvent(QMouseEvent* event) override;
    
    /**
     * @brief Override mouse release event for animation
     */
    void mouseReleaseEvent(QMouseEvent* event) override;
    
    /**
     * @brief Override enter event for hover effects
     */
    void enterEvent(QEvent* event) override;
    
    /**
     * @brief Override leave event for hover effects
     */
    void leaveEvent(QEvent* event) override;

private slots:
    /**
     * @brief Handle animation finished
     */
    void onAnimationFinished();

private:
    /**
     * @brief Initialize button styling and properties
     */
    void initializeButton();
    
    /**
     * @brief Update stylesheet based on current type
     */
    void updateStyleSheet();
    
    // Animation methods disabled - kept for compatibility but do nothing
    void startPressAnimation() { /* NO-OP */ }
    void startReleaseAnimation() { /* NO-OP */ }
    
    ButtonType button_type_;
    bool animation_enabled_;
    QColor custom_background_;
    QColor custom_text_;
    bool using_custom_colors_;
    
    // Animation system disabled to fix layout stability bugs
    // Visual feedback handled entirely by CSS styling
    QPropertyAnimation* press_animation_;  // Always nullptr - kept for compatibility
    QPropertyAnimation* release_animation_;  // Always nullptr - kept for compatibility
    
    // Effects
    QGraphicsDropShadowEffect* shadow_effect_;
};

} // namespace widgets
} // namespace gui
} // namespace unlook