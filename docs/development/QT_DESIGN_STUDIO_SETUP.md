# Unlook 3D Scanner - Qt Design Studio Integration Guide

## Overview

The Unlook 3D Scanner GUI has been restructured to be fully compatible with Qt Design Studio for visual editing. This guide explains how to use the new .ui file-based architecture for designing and maintaining the user interface.

## Architecture Changes

### Previous Architecture (Hardcoded)
- All layouts created programmatically in C++ constructors
- Widget properties set manually in code
- Difficult to modify visual appearance without recompilation
- No visual editing capabilities

### New Architecture (Qt Design Studio Compatible)
- UI layouts defined in `.ui` files (XML format)
- C++ code uses `setupUi()` pattern to load designs
- Full compatibility with Qt Creator and Qt Design Studio
- Visual editing with live preview
- Separation of design from logic

## Project Structure

### UI Files Location
```
src/gui/ui/
├── main_window.ui               # Main application window
├── camera_preview_widget.ui     # Dual camera preview interface
├── depth_test_widget.ui         # Depth processing controls
└── options_widget.ui            # System configuration panel
```

### Qt Project Files
```
src/gui/
├── unlook_scanner.pro           # Qt Creator project file
├── unlook_scanner.qbs          # Qt Build Suite project file
└── CMakeLists.txt              # CMake with Qt Design Studio compatibility
```

### Design System Integration

The UI files implement the Supernova-tech design system:

#### Color Palette
- **Electric Primary**: `#00E5CC` (primary buttons, headings)
- **Plasma Accent**: `#00B8A3` (secondary elements) 
- **Quantum Depth**: `#008B7A` (borders, dividers)
- **Void Background**: `#000000` (backgrounds)
- **Nebula Surface**: `#1A2B2A` (panels, surfaces)

#### Typography
- **Display**: 24pt bold (welcome text)
- **Title**: 18pt bold (window titles)
- **Heading**: 16pt bold (section headers)
- **Subtitle**: 14pt normal (descriptions)
- **Body**: 12pt normal (labels)
- **Small**: 10pt normal (status text)

#### Touch Targets
- **Minimum Size**: 48x48px (accessibility requirement)
- **Button Heights**: 40-60px (primary actions)
- **Slider Heights**: 30px (parameter controls)

## Opening in Qt Design Studio

### Method 1: Via CMake (Recommended)
```bash
cd /home/alessandro/unlook-standalone
mkdir build-design && cd build-design
cmake -DCMAKE_PREFIX_PATH=/path/to/qt5 ..
qtcreator ../CMakeLists.txt
```

### Method 2: Via .pro File
```bash
cd /home/alessandro/unlook-standalone/src/gui
qtcreator unlook_scanner.pro
```

### Method 3: Direct UI File Editing
```bash
designer src/gui/ui/main_window.ui
```

## Editing UI Files

### Main Window (main_window.ui)
**Components:**
- Title bar with navigation buttons
- Stacked widget for screen switching
- Main menu with 2x2 button grid
- Status bar with system information

**Key Objects:**
- `screen_stack`: QStackedWidget for navigation
- `main_menu_screen`: Welcome screen with buttons
- `camera_preview_button`: Navigate to camera view
- `depth_test_button`: Navigate to depth processing
- `options_button`: Navigate to settings
- `exit_button`: Application exit

### Camera Preview (camera_preview_widget.ui)
**Components:**
- Dual camera preview area
- Controls panel with sliders and buttons
- Status panel with live information

**Key Objects:**
- `left_camera_label`, `right_camera_label`: Video displays
- `start_capture_button`, `stop_capture_button`: Capture controls
- `left_exposure_slider`, `right_exposure_slider`: Camera settings
- `fps_slider`: Frame rate control

### Depth Test (depth_test_widget.ui)
**Components:**
- Image visualization area
- Algorithm selection controls  
- Parameter adjustment sliders
- Processing status display

**Key Objects:**
- `left_image_label`, `right_image_label`: Input images
- `depth_image_label`: Depth map display
- `algorithm_combo`: Algorithm selection
- `capture_button`: Process depth map

### Options (options_widget.ui)
**Components:**
- Configuration buttons
- System information panel
- Hardware details
- Software versions

**Key Objects:**
- `calibration_validation_button`: Calibration tools
- `advanced_camera_button`: Camera settings
- `system_diagnostics_button`: Hardware tests

## Design Guidelines

### Layout Principles
1. **Responsive Design**: Use percentage-based sizing where possible
2. **Touch-Friendly**: Minimum 48px touch targets
3. **High Contrast**: Dark theme with electric blue accents
4. **Information Hierarchy**: Clear visual grouping of related controls

### Widget Naming Convention
- Use descriptive names with underscores: `camera_preview_button`
- Include widget type suffix: `_button`, `_slider`, `_label`
- Group related widgets: `left_camera_*`, `right_camera_*`

### Styling Approach
- Base styles defined in .ui files
- Dynamic styling applied in C++ for state changes
- Custom widgets (TouchButton, StatusDisplay) maintain their styling
- Color values embedded directly for Qt Designer compatibility

## Integration with C++ Code

### setupUi() Pattern
Each widget class follows this pattern:
```cpp
// Header file
QT_BEGIN_NAMESPACE
namespace Ui { class MyWidget; }
QT_END_NAMESPACE

class MyWidget : public QWidget {
    Q_OBJECT
private:
    Ui::MyWidget *ui;
};

// Implementation file
#include "ui_mywidget.h"

MyWidget::MyWidget(QWidget* parent) 
    : QWidget(parent), ui(new Ui::MyWidget) {
    ui->setupUi(this);
    // Connect signals, additional setup
}

MyWidget::~MyWidget() {
    delete ui;
}
```

### Signal Connection
UI signals are connected in `connectSignals()` method:
```cpp
void MyWidget::connectSignals() {
    connect(ui->my_button, &QPushButton::clicked, 
            this, &MyWidget::onButtonClicked);
}
```

### Accessing UI Elements
Access widgets through the `ui` pointer:
```cpp
ui->status_label->setText("Ready");
ui->progress_bar->setValue(50);
ui->camera_display->setPixmap(frame);
```

## Building and Testing

### CMake Build (Production)
```bash
./build.sh -t Release
```

### Qt Creator Build (Development)
1. Open `unlook_scanner.pro` in Qt Creator
2. Configure build settings
3. Build → Build Project
4. Run → Run without Debugging

### Design-Time Testing
1. Open `.ui` file in Qt Designer
2. Use Form → Preview to test responsive behavior
3. Verify touch target sizes and color contrast
4. Test on different screen resolutions

## Troubleshooting

### Common Issues

**Issue**: `ui_*.h` files not found
**Solution**: Ensure CMAKE_AUTOUIC is enabled and .ui files are in correct location

**Issue**: Qt Designer can't find custom widgets
**Solution**: Custom widgets (TouchButton, etc.) need designer plugins or use base Qt widgets

**Issue**: Styling not applied correctly
**Solution**: Check that styleSheet properties are set in .ui file, not just C++ code

### Build Debugging
```bash
# Verbose build to see UIC processing
./build.sh -v -t Debug

# Manual UIC generation
uic src/gui/ui/main_window.ui -o ui_main_window.h
```

## Extending the Design

### Adding New Screens
1. Create new `.ui` file in `src/gui/ui/`
2. Add corresponding C++ widget class
3. Update CMakeLists.txt with new files
4. Add navigation logic in main window

### Modifying Existing Designs
1. Open `.ui` file in Qt Designer
2. Make visual changes using drag-and-drop
3. Save and rebuild project
4. Test functionality in application

### Custom Widget Integration
For custom widgets to appear in Qt Designer:
1. Create designer plugin
2. Or use promotion feature to convert standard widgets
3. Ensure proper inheritance hierarchy

## Performance Considerations

### Design-Time Optimization
- Use placeholder images during design
- Minimize complex nested layouts
- Test on target hardware resolution

### Runtime Optimization  
- UI loading happens once during widget creation
- Dynamic updates only modify widget properties
- Responsive sizing calculated once during show events

## Conclusion

The new Qt Design Studio integration provides:
- ✅ Visual editing capability
- ✅ Separation of design from logic  
- ✅ Professional design tool workflow
- ✅ Maintains all existing functionality
- ✅ Supernova-tech design system compliance
- ✅ Touch-optimized interface design

The GUI is now fully editable using industry-standard Qt design tools while maintaining the high-performance, industrial-grade requirements of the Unlook 3D Scanner system.