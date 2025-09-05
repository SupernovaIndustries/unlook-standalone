#pragma once

#include <QWidget>
#include <QColor>
#include <QString>
#include <QFont>

namespace unlook {
namespace gui {
namespace styles {

/**
 * @brief Supernova-tech design system for the Unlook 3D Scanner
 * 
 * Implements the futuristic industrial design language with electric blues/teals
 * on dark backgrounds. Optimized for touch interfaces and professional use.
 */
class SupernovaStyle {
public:
    // Primary color palette (exact Supernova-tech specification)
    static const QColor ELECTRIC_PRIMARY;    // #00E5CC - Primary electric teal
    static const QColor PLASMA_ACCENT;       // #00B8A3 - Plasma accent
    static const QColor QUANTUM_DEPTH;       // #008B7A - Quantum depth
    static const QColor VOID_BACKGROUND;     // #000000 - Void backgrounds
    static const QColor NEBULA_SURFACE;      // #1A2B2A - Nebula surfaces
    
    // State colors
    static const QColor SUCCESS_STATE;       // #00FF88 - Success indication
    static const QColor WARNING_STATE;       // #FFB800 - Warning state
    static const QColor ERROR_STATE;         // #FF3B30 - Error state
    static const QColor INACTIVE_STATE;      // #555555 - Inactive elements
    
    // Text colors
    static const QColor TEXT_PRIMARY;        // #FFFFFF - Primary text
    static const QColor TEXT_SECONDARY;      // #CCCCCC - Secondary text
    static const QColor TEXT_DISABLED;       // #666666 - Disabled text
    
    // Gradient definitions
    struct Gradient {
        QColor start;
        QColor end;
        QString css() const;
    };
    
    static const Gradient ELECTRIC_GRADIENT;
    static const Gradient PLASMA_GRADIENT;
    static const Gradient DEPTH_GRADIENT;
    
    // Typography system
    enum class FontWeight {
        LIGHT = 300,
        REGULAR = 400,
        MEDIUM = 500,
        BOLD = 700
    };
    
    enum class FontSize {
        SMALL = 12,
        BODY = 14,
        SUBTITLE = 16,
        TITLE = 20,
        HEADING = 24,
        DISPLAY = 32
    };
    
    /**
     * @brief Get font with specified size and weight
     */
    static QFont getFont(FontSize size, FontWeight weight = FontWeight::REGULAR);
    
    /**
     * @brief Apply Supernova styling to a widget
     */
    static void applyStyle(QWidget* widget);
    
    /**
     * @brief Get CSS stylesheet for the entire application
     */
    static QString getApplicationStyleSheet();
    
    /**
     * @brief Get touch-optimized button style
     */
    static QString getTouchButtonStyle(const QColor& background = ELECTRIC_PRIMARY,
                                      const QColor& text = TEXT_PRIMARY,
                                      int min_height = 60);
    
    /**
     * @brief Get slider style for parameter controls
     */
    static QString getSliderStyle();
    
    /**
     * @brief Get status display style
     */
    static QString getStatusDisplayStyle(const QColor& background = NEBULA_SURFACE);
    
    /**
     * @brief Get camera preview frame style
     */
    static QString getCameraPreviewStyle();
    
    /**
     * @brief Get main window style
     */
    static QString getMainWindowStyle();
    
    /**
     * @brief Animation constants for consistent timing
     */
    struct Animation {
        static const int FAST_DURATION_MS = 150;
        static const int NORMAL_DURATION_MS = 250;
        static const int SLOW_DURATION_MS = 400;
    };
    
    /**
     * @brief Spacing constants for consistent layouts
     */
    struct Spacing {
        static const int TOUCH_TARGET_MIN = 48;  // Minimum touch target size
        static const int PADDING_SMALL = 8;
        static const int PADDING_MEDIUM = 16;
        static const int PADDING_LARGE = 24;
        static const int MARGIN_SMALL = 12;
        static const int MARGIN_MEDIUM = 20;
        static const int MARGIN_LARGE = 32;
    };
    
    /**
     * @brief Border radius constants
     */
    struct BorderRadius {
        static const int SMALL = 4;
        static const int MEDIUM = 8;
        static const int LARGE = 12;
        static const int BUTTON = 6;
    };

    /**
     * @brief Convert QColor to CSS color string
     */
    static QString colorToString(const QColor& color);

private:
    static QString generateButtonHoverStyle(const QColor& base_color);
    static QString generateButtonPressedStyle(const QColor& base_color);
};

} // namespace styles
} // namespace gui
} // namespace unlook