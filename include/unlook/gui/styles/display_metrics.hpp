#pragma once

#include <QWidget>
#include <QSize>
#include <QGuiApplication>
#include <QScreen>
#include <QFont>
#include <QColor>

namespace unlook {
namespace gui {
namespace styles {

/**
 * @brief Responsive display metrics system for auto-adapting UI to different screen sizes
 * 
 * Automatically detects display resolution and provides scaled dimensions for:
 * - 840x480 (target user display)
 * - 1024x768 (standard development)
 * - Other common resolutions
 * 
 * Ensures 48px minimum touch targets while optimizing space usage.
 */
class DisplayMetrics {
public:
    /**
     * @brief Screen size categories for different scaling approaches
     */
    enum class ScreenSize {
        SMALL,    // <= 800px wide (like 840x480)
        MEDIUM,   // 800-1200px wide 
        LARGE,    // >= 1200px wide
        UNKNOWN
    };
    
    /**
     * @brief Initialize display metrics (call once at application startup)
     */
    static void initialize();
    
    /**
     * @brief Get singleton instance
     */
    static DisplayMetrics& instance();
    
    /**
     * @brief Get current screen size
     */
    QSize screenSize() const { return screen_size_; }
    
    /**
     * @brief Get screen size category
     */
    ScreenSize sizeCategory() const { return size_category_; }
    
    /**
     * @brief Get DPI-aware scale factor
     */
    float scaleFactor() const { return scale_factor_; }
    
    /**
     * @brief Get responsive font size
     */
    QFont getResponsiveFont(int base_size, QFont::Weight weight = QFont::Medium) const;
    
    // Responsive UI element dimensions
    
    /**
     * @brief Get responsive button dimensions
     */
    QSize getButtonSize(bool compact = false) const;
    
    /**
     * @brief Get responsive touch target minimum
     */
    int getTouchTargetMin() const;
    
    /**
     * @brief Get responsive spacing values
     */
    int getSpacing(int base_spacing) const;
    int getMargin(int base_margin) const;
    int getPadding(int base_padding) const;
    
    /**
     * @brief Get responsive title bar height
     */
    int getTitleBarHeight() const;
    
    /**
     * @brief Get responsive status bar height
     */
    int getStatusBarHeight() const;
    
    /**
     * @brief Get responsive main button grid dimensions
     */
    QSize getMainButtonSize() const;
    int getButtonSpacing() const;
    
    /**
     * @brief Check if display should use compact mode
     */
    bool shouldUseCompactMode() const;
    
    /**
     * @brief Get display info for debugging
     */
    QString getDisplayInfo() const;

private:
    DisplayMetrics() = default;
    
    void calculateMetrics();
    void detectScreenSize();
    float calculateScaleFactor() const;
    
    // Display properties
    QSize screen_size_;
    ScreenSize size_category_;
    float scale_factor_;
    float dpi_ratio_;
    bool compact_mode_;
    
    // Responsive constants based on screen size
    int title_bar_height_;
    int status_bar_height_;
    int button_height_;
    int button_width_;
    int touch_target_min_;
    int base_spacing_;
    int base_margin_;
    int base_padding_;
    
    static DisplayMetrics* instance_;
};

} // namespace styles
} // namespace gui
} // namespace unlook