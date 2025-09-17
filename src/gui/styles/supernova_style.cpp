#include "unlook/gui/styles/supernova_style.hpp"
#include <QApplication>

namespace unlook {
namespace gui {
namespace styles {

// Primary color palette (exact Supernova-tech specification)
const QColor SupernovaStyle::ELECTRIC_PRIMARY(0x00, 0xE5, 0xCC);    // #00E5CC
const QColor SupernovaStyle::PLASMA_ACCENT(0x00, 0xB8, 0xA3);       // #00B8A3
const QColor SupernovaStyle::QUANTUM_DEPTH(0x00, 0x8B, 0x7A);       // #008B7A
const QColor SupernovaStyle::VOID_BACKGROUND(0x00, 0x00, 0x00);     // #000000
const QColor SupernovaStyle::NEBULA_SURFACE(0x1A, 0x2B, 0x2A);      // #1A2B2A

// State colors
const QColor SupernovaStyle::SUCCESS_STATE(0x00, 0xFF, 0x88);       // #00FF88
const QColor SupernovaStyle::WARNING_STATE(0xFF, 0xB8, 0x00);       // #FFB800
const QColor SupernovaStyle::ERROR_STATE(0xFF, 0x3B, 0x30);         // #FF3B30
const QColor SupernovaStyle::INACTIVE_STATE(0x55, 0x55, 0x55);      // #555555

// Text colors
const QColor SupernovaStyle::TEXT_PRIMARY(0xFF, 0xFF, 0xFF);         // #FFFFFF
const QColor SupernovaStyle::TEXT_SECONDARY(0xCC, 0xCC, 0xCC);      // #CCCCCC
const QColor SupernovaStyle::TEXT_DISABLED(0x66, 0x66, 0x66);       // #666666

// Gradients
const SupernovaStyle::Gradient SupernovaStyle::ELECTRIC_GRADIENT = {
    ELECTRIC_PRIMARY, PLASMA_ACCENT
};

const SupernovaStyle::Gradient SupernovaStyle::PLASMA_GRADIENT = {
    PLASMA_ACCENT, QUANTUM_DEPTH
};

const SupernovaStyle::Gradient SupernovaStyle::DEPTH_GRADIENT = {
    QUANTUM_DEPTH, NEBULA_SURFACE
};

QString SupernovaStyle::Gradient::css() const {
    return QString("qlineargradient(x1: 0, y1: 0, x2: 1, y2: 1, "
                   "stop: 0 %1, stop: 1 %2)")
           .arg(colorToString(start))
           .arg(colorToString(end));
}

QFont SupernovaStyle::getFont(FontSize size, FontWeight weight) {
    QFont font("Inter", static_cast<int>(size), static_cast<int>(weight));
    font.setHintingPreference(QFont::PreferNoHinting);
    font.setStyleStrategy(QFont::PreferAntialias);
    return font;
}

void SupernovaStyle::applyStyle(QWidget* widget) {
    if (!widget) return;
    
    widget->setStyleSheet(getApplicationStyleSheet());
    widget->setFont(getFont(FontSize::BODY));
}

QString SupernovaStyle::getApplicationStyleSheet() {
    return QString(R"(
        /* Main application styling */
        QMainWindow {
            background-color: %1;
            color: %2;
            font-family: "Inter", sans-serif;
        }
        
        QWidget {
            background-color: %1;
            color: %2;
            border: none;
        }
        
        /* Scrollbars */
        QScrollBar:vertical {
            background: %3;
            width: 12px;
            border-radius: 6px;
        }
        
        QScrollBar::handle:vertical {
            background: %4;
            border-radius: 6px;
            min-height: 20px;
        }
        
        QScrollBar::handle:vertical:hover {
            background: %5;
        }
        
        QScrollBar:horizontal {
            background: %3;
            height: 12px;
            border-radius: 6px;
        }
        
        QScrollBar::handle:horizontal {
            background: %4;
            border-radius: 6px;
            min-width: 20px;
        }
        
        QScrollBar::handle:horizontal:hover {
            background: %5;
        }
        
        /* Remove scroll arrows */
        QScrollBar::add-line, QScrollBar::sub-line {
            border: none;
            background: none;
        }
    )")
    .arg(colorToString(VOID_BACKGROUND))    // %1 - Main background
    .arg(colorToString(TEXT_PRIMARY))       // %2 - Text color
    .arg(colorToString(NEBULA_SURFACE))     // %3 - Scrollbar track
    .arg(colorToString(QUANTUM_DEPTH))      // %4 - Scrollbar handle
    .arg(colorToString(PLASMA_ACCENT));     // %5 - Scrollbar handle hover
}

QString SupernovaStyle::getTouchButtonStyle(const QColor& background, 
                                           const QColor& text, 
                                           int min_height) {
    return QString(R"(
        QPushButton {
            background: %1;
            color: %2;
            border: 2px solid transparent;
            border-radius: %3px;
            padding: %4px %5px;
            min-height: %6px;
            font-weight: 500;
            font-size: 16px;
        }
        
        QPushButton:hover {
            background: %7;
            border: 2px solid %8;
        }

        QPushButton:pressed {
            background: %9;
        }
        
        QPushButton:disabled {
            background: %10;
            color: %11;
            border: 2px solid transparent;
        }
    )")
    .arg(colorToString(background))                           // %1 - Background
    .arg(colorToString(text))                                // %2 - Text color
    .arg(BorderRadius::BUTTON)                               // %3 - Border radius
    .arg(Spacing::PADDING_MEDIUM)                           // %4 - Vertical padding
    .arg(Spacing::PADDING_LARGE)                            // %5 - Horizontal padding
    .arg(min_height)                                        // %6 - Minimum height
    .arg(generateButtonHoverStyle(background))              // %7 - Hover background
    .arg(colorToString(ELECTRIC_PRIMARY))                   // %8 - Hover border
    .arg(generateButtonPressedStyle(background))            // %9 - Pressed background
    .arg(colorToString(INACTIVE_STATE))                     // %10 - Disabled background
    .arg(colorToString(TEXT_DISABLED));                     // %11 - Disabled text
}

QString SupernovaStyle::getSliderStyle() {
    return QString(R"(
        QSlider::groove:horizontal {
            background: %1;
            height: 6px;
            border-radius: 3px;
        }
        
        QSlider::handle:horizontal {
            background: %2;
            border: 2px solid %3;
            width: 20px;
            height: 20px;
            border-radius: 12px;
            margin: -8px 0;
        }
        
        QSlider::handle:horizontal:hover {
            background: %4;
            border: 2px solid %5;
        }
        
        QSlider::handle:horizontal:pressed {
            background: %6;
        }
        
        QSlider::sub-page:horizontal {
            background: %7;
            border-radius: 3px;
        }
    )")
    .arg(colorToString(NEBULA_SURFACE))      // %1 - Groove background
    .arg(colorToString(ELECTRIC_PRIMARY))    // %2 - Handle background
    .arg(colorToString(PLASMA_ACCENT))       // %3 - Handle border
    .arg(colorToString(PLASMA_ACCENT))       // %4 - Handle hover background
    .arg(colorToString(ELECTRIC_PRIMARY))    // %5 - Handle hover border
    .arg(colorToString(QUANTUM_DEPTH))       // %6 - Handle pressed background
    .arg(colorToString(ELECTRIC_PRIMARY));   // %7 - Sub-page (filled) background
}

QString SupernovaStyle::getStatusDisplayStyle(const QColor& background) {
    return QString(R"(
        QLabel {
            background: %1;
            color: %2;
            border: 1px solid %3;
            border-radius: %4px;
            padding: %5px %6px;
            font-weight: 400;
        }
    )")
    .arg(colorToString(background))          // %1 - Background
    .arg(colorToString(TEXT_PRIMARY))        // %2 - Text color
    .arg(colorToString(QUANTUM_DEPTH))       // %3 - Border color
    .arg(BorderRadius::MEDIUM)               // %4 - Border radius
    .arg(Spacing::PADDING_SMALL)             // %5 - Vertical padding
    .arg(Spacing::PADDING_MEDIUM);           // %6 - Horizontal padding
}

QString SupernovaStyle::getCameraPreviewStyle() {
    return QString(R"(
        QLabel {
            background: %1;
            border: 2px solid %2;
            border-radius: %3px;
        }
        
        QLabel:hover {
            border: 2px solid %4;
        }
    )")
    .arg(colorToString(VOID_BACKGROUND))     // %1 - Background
    .arg(colorToString(NEBULA_SURFACE))      // %2 - Border
    .arg(BorderRadius::MEDIUM)               // %3 - Border radius
    .arg(colorToString(QUANTUM_DEPTH));      // %4 - Hover border
}

QString SupernovaStyle::getMainWindowStyle() {
    return QString(R"(
        QMainWindow {
            background: %1;
        }
        
        QStackedWidget {
            background: %1;
        }
        
        QFrame {
            background: %1;
            border: none;
        }
    )")
    .arg(colorToString(VOID_BACKGROUND));    // %1 - Main background
}

QString SupernovaStyle::colorToString(const QColor& color) {
    return QString("rgb(%1, %2, %3)")
           .arg(color.red())
           .arg(color.green())
           .arg(color.blue());
}

QString SupernovaStyle::generateButtonHoverStyle(const QColor& base_color) {
    // Use fixed, stable hover colors to prevent infinite scaling
    QColor hover_color;
    
    if (base_color == ELECTRIC_PRIMARY) {
        hover_color = QColor(0x00, 0xFF, 0xE6);  // Slightly brighter electric
    } else if (base_color == PLASMA_ACCENT) {
        hover_color = QColor(0x00, 0xD4, 0xBB);  // Slightly brighter plasma
    } else if (base_color == SUCCESS_STATE) {
        hover_color = QColor(0x33, 0xFF, 0xAA);  // Slightly brighter success
    } else if (base_color == WARNING_STATE) {
        hover_color = QColor(0xFF, 0xCC, 0x33);  // Slightly brighter warning
    } else if (base_color == ERROR_STATE) {
        hover_color = QColor(0xFF, 0x66, 0x55);  // Slightly brighter error
    } else {
        // Default: use plasma accent for unknown colors
        hover_color = QColor(0x00, 0xD4, 0xBB);
    }
    
    return colorToString(hover_color);
}

QString SupernovaStyle::generateButtonPressedStyle(const QColor& base_color) {
    // Use fixed, stable pressed colors to prevent cascading effects
    QColor pressed_color;
    
    if (base_color == ELECTRIC_PRIMARY) {
        pressed_color = QColor(0x00, 0xB8, 0xA3);  // Darker electric (plasma accent)
    } else if (base_color == PLASMA_ACCENT) {
        pressed_color = QColor(0x00, 0x8B, 0x7A);  // Darker plasma (quantum depth)
    } else if (base_color == SUCCESS_STATE) {
        pressed_color = QColor(0x00, 0xCC, 0x66);  // Darker success
    } else if (base_color == WARNING_STATE) {
        pressed_color = QColor(0xCC, 0x99, 0x00);  // Darker warning
    } else if (base_color == ERROR_STATE) {
        pressed_color = QColor(0xCC, 0x22, 0x18);  // Darker error
    } else {
        // Default: use quantum depth for unknown colors
        pressed_color = QColor(0x00, 0x8B, 0x7A);
    }
    
    return colorToString(pressed_color);
}

} // namespace styles
} // namespace gui
} // namespace unlook