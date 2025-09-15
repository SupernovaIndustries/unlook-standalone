QT += core widgets gui opengl

TARGET = unlook_scanner
TEMPLATE = app

# C++ standard
CONFIG += c++17

# Source files
SOURCES += \
    ../main.cpp \
    main_window.cpp \
    camera_preview_widget.cpp \
    depth_test_widget.cpp \
    options_widget.cpp \
    widgets/parameter_slider.cpp \
    widgets/status_display.cpp \
    widgets/touch_button.cpp \
    styles/supernova_style.cpp \
    styles/display_metrics.cpp

# Header files
HEADERS += \
    ../../include/unlook/gui/main_window.hpp \
    ../../include/unlook/gui/camera_preview_widget.hpp \
    ../../include/unlook/gui/depth_test_widget.hpp \
    ../../include/unlook/gui/options_widget.hpp \
    ../../include/unlook/gui/widgets/parameter_slider.hpp \
    ../../include/unlook/gui/widgets/status_display.hpp \
    ../../include/unlook/gui/widgets/touch_button.hpp \
    ../../include/unlook/gui/styles/display_metrics.hpp \
    ../../include/unlook/gui/styles/supernova_style.hpp

# UI files - Qt Design Studio compatible
FORMS += \
    ui/main_window.ui \
    ui/camera_preview_widget.ui \
    ui/depth_test_widget.ui \
    ui/options_widget.ui

# Resources
RESOURCES += \
    resources/gui_resources.qrc \
    resources/icons.qrc \
    resources/styles.qrc

# Include paths
INCLUDEPATH += \
    . \
    .. \
    ../../include \
    ../../include/unlook \
    ../../include/unlook/gui

# Defines
DEFINES += QT_DEPRECATED_WARNINGS

# For Qt Design Studio integration
QML_IMPORT_PATH =
QML_DESIGNER_IMPORT_PATH =

# Output directory
DESTDIR = ../../build/bin

# Build configuration
CONFIG(debug, debug|release) {
    TARGET = $$TARGET_d
}

# Platform specific configurations
win32 {
    CONFIG += console
}

unix:!macx {
    CONFIG += link_pkgconfig
}

macx {
    CONFIG += app_bundle
    QMAKE_INFO_PLIST = Info.plist
}