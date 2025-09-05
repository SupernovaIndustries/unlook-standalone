#include "unlook/gui/styles/display_metrics.hpp"
#include <QDebug>
#include <QFontMetrics>
#include <algorithm>

namespace unlook {
namespace gui {
namespace styles {

DisplayMetrics* DisplayMetrics::instance_ = nullptr;

void DisplayMetrics::initialize() {
    if (!instance_) {
        instance_ = new DisplayMetrics();
        instance_->calculateMetrics();
        
        // Debug output
        qDebug() << "DisplayMetrics initialized:" << instance_->getDisplayInfo();
    }
}

DisplayMetrics& DisplayMetrics::instance() {
    if (!instance_) {
        initialize();
    }
    return *instance_;
}

void DisplayMetrics::calculateMetrics() {
    detectScreenSize();
    
    // Get DPI information
    QScreen* primary_screen = QGuiApplication::primaryScreen();
    if (primary_screen) {
        dpi_ratio_ = primary_screen->devicePixelRatio();
        scale_factor_ = calculateScaleFactor();
    } else {
        dpi_ratio_ = 1.0f;
        scale_factor_ = 1.0f;
    }
    
    // Calculate responsive dimensions based on screen size category
    switch (size_category_) {
        case ScreenSize::SMALL:
            // 800x480 ultra-compact - CRITICAL FIX for overlapping elements
            compact_mode_ = true;
            title_bar_height_ = 35;    // DRASTICALLY reduced for 480px height
            status_bar_height_ = 25;   // DRASTICALLY reduced for 480px height
            button_height_ = 45;       // Max 45px as requested, still >= 48px after scaling
            button_width_ = 140;       // Max 140px as requested for 2x2 grid to fit
            touch_target_min_ = 48;    // Minimum touch area (always respected)
            base_spacing_ = 4;         // MINIMAL spacing to prevent overlap
            base_margin_ = 4;          // MINIMAL margins to prevent overlap
            base_padding_ = 4;         // MINIMAL padding to prevent overlap
            break;
            
        case ScreenSize::MEDIUM:
            // 1024x768 and similar
            compact_mode_ = false;
            title_bar_height_ = 60;
            status_bar_height_ = 40;
            button_height_ = 64;
            button_width_ = 220;
            touch_target_min_ = 48;
            base_spacing_ = 12;
            base_margin_ = 16;
            base_padding_ = 12;
            break;
            
        case ScreenSize::LARGE:
            // 1200+ px wide screens
            compact_mode_ = false;
            title_bar_height_ = 70;
            status_bar_height_ = 45;
            button_height_ = 72;
            button_width_ = 260;
            touch_target_min_ = 48;
            base_spacing_ = 16;
            base_margin_ = 24;
            base_padding_ = 16;
            break;
            
        default:
            // Safe fallback
            compact_mode_ = true;
            title_bar_height_ = 50;
            status_bar_height_ = 32;
            button_height_ = 56;
            button_width_ = 180;
            touch_target_min_ = 48;
            base_spacing_ = 8;
            base_margin_ = 12;
            base_padding_ = 10;
            break;
    }
    
    // Apply scale factor while maintaining minimum touch targets
    title_bar_height_ = static_cast<int>(title_bar_height_ * scale_factor_);
    status_bar_height_ = static_cast<int>(status_bar_height_ * scale_factor_);
    button_height_ = std::max(touch_target_min_, static_cast<int>(button_height_ * scale_factor_));
    button_width_ = static_cast<int>(button_width_ * scale_factor_);
}

void DisplayMetrics::detectScreenSize() {
    QScreen* primary_screen = QGuiApplication::primaryScreen();
    if (primary_screen) {
        screen_size_ = primary_screen->availableSize();
        
        int width = screen_size_.width();
        
        if (width <= 850) {
            // 840x480 and similar small displays - TARGET USER CASE
            size_category_ = ScreenSize::SMALL;
            qDebug() << "Detected SMALL screen (target: 840x480):" << screen_size_;
        } else if (width <= 1200) {
            // 1024x768 and similar medium displays  
            size_category_ = ScreenSize::MEDIUM;
            qDebug() << "Detected MEDIUM screen:" << screen_size_;
        } else {
            // Large displays
            size_category_ = ScreenSize::LARGE;
            qDebug() << "Detected LARGE screen:" << screen_size_;
        }
    } else {
        // Fallback if screen detection fails
        screen_size_ = QSize(840, 480);  // Assume target display
        size_category_ = ScreenSize::SMALL;
        qWarning() << "Screen detection failed, assuming 840x480";
    }
}

float DisplayMetrics::calculateScaleFactor() const {
    // Base scale factor calculation
    float base_scale = 1.0f;
    
    if (size_category_ == ScreenSize::SMALL) {
        // For 800x480 screens, scale DOWN MORE to prevent overlapping
        base_scale = 1.0f; // NO scaling - use raw dimensions to prevent overflow
    } else if (size_category_ == ScreenSize::LARGE) {
        // For large screens, scale UP for better readability
        base_scale = 1.2f;
    }
    
    // Apply DPI scaling
    return base_scale * dpi_ratio_;
}

QFont DisplayMetrics::getResponsiveFont(int base_size, QFont::Weight weight) const {
    // Scale font size responsively while maintaining readability
    int scaled_size = base_size;
    
    switch (size_category_) {
        case ScreenSize::SMALL:
            // For 800x480, use significantly smaller fonts to prevent overflow
            scaled_size = std::max(8, static_cast<int>(base_size * 0.7f)); // Much more aggressive scaling
            break;
        case ScreenSize::MEDIUM:
            scaled_size = base_size;
            break;
        case ScreenSize::LARGE:
            scaled_size = static_cast<int>(base_size * 1.1f);
            break;
        default:
            scaled_size = base_size;
            break;
    }
    
    QFont font("Roboto", scaled_size, weight);
    font.setStyleHint(QFont::SansSerif);
    return font;
}

QSize DisplayMetrics::getButtonSize(bool compact) const {
    if (compact || size_category_ == ScreenSize::SMALL) {
        // Ensure buttons fit in 2x2 grid on small screens while maintaining 48px minimum
        int available_width = screen_size_.width() - (base_margin_ * 4) - base_spacing_;
        int max_button_width = available_width / 2;
        
        return QSize(std::min(button_width_, max_button_width), button_height_);
    }
    
    return QSize(button_width_, button_height_);
}

int DisplayMetrics::getTouchTargetMin() const {
    return touch_target_min_;
}

int DisplayMetrics::getSpacing(int base_spacing) const {
    return std::max(2, static_cast<int>(base_spacing * (base_spacing_ / 12.0f)));
}

int DisplayMetrics::getMargin(int base_margin) const {
    return std::max(4, static_cast<int>(base_margin * (base_margin_ / 16.0f)));
}

int DisplayMetrics::getPadding(int base_padding) const {
    return std::max(2, static_cast<int>(base_padding * (base_padding_ / 12.0f)));
}

int DisplayMetrics::getTitleBarHeight() const {
    return title_bar_height_;
}

int DisplayMetrics::getStatusBarHeight() const {
    return status_bar_height_;
}

QSize DisplayMetrics::getMainButtonSize() const {
    return getButtonSize(compact_mode_);
}

int DisplayMetrics::getButtonSpacing() const {
    return getSpacing(base_spacing_);
}

bool DisplayMetrics::shouldUseCompactMode() const {
    return compact_mode_;
}

QString DisplayMetrics::getDisplayInfo() const {
    QString info = QString("Screen: %1x%2, Category: %3, Scale: %4x, Compact: %5")
                   .arg(screen_size_.width())
                   .arg(screen_size_.height())
                   .arg(size_category_ == ScreenSize::SMALL ? "SMALL" :
                        size_category_ == ScreenSize::MEDIUM ? "MEDIUM" : "LARGE")
                   .arg(scale_factor_, 0, 'f', 2)
                   .arg(compact_mode_ ? "YES" : "NO");
    
    info += QString("\nButtons: %1x%2, TitleBar: %3px, StatusBar: %4px")
            .arg(button_width_).arg(button_height_)
            .arg(title_bar_height_).arg(status_bar_height_);
            
    return info;
}

} // namespace styles
} // namespace gui  
} // namespace unlook