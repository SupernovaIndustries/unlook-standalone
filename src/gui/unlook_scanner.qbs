import qbs

Application {
    name: "unlook_scanner"
    
    Depends { name: "Qt"; submodules: ["core", "widgets", "gui", "opengl"] }
    Depends { name: "cpp" }
    
    cpp.cxxLanguageVersion: "c++17"
    cpp.defines: ["QT_DEPRECATED_WARNINGS"]
    
    cpp.includePaths: [
        ".",
        "..",
        "../../include",
        "../../include/unlook",
        "../../include/unlook/gui"
    ]
    
    files: [
        "../main.cpp",
        "main_window.cpp",
        "camera_preview_widget.cpp", 
        "depth_test_widget.cpp",
        "options_widget.cpp",
        "widgets/parameter_slider.cpp",
        "widgets/status_display.cpp",
        "widgets/touch_button.cpp",
        "styles/supernova_style.cpp",
        "styles/display_metrics.cpp",
        
        "../../include/unlook/gui/main_window.hpp",
        "../../include/unlook/gui/camera_preview_widget.hpp",
        "../../include/unlook/gui/depth_test_widget.hpp", 
        "../../include/unlook/gui/options_widget.hpp",
        "../../include/unlook/gui/widgets/parameter_slider.hpp",
        "../../include/unlook/gui/widgets/status_display.hpp",
        "../../include/unlook/gui/widgets/touch_button.hpp",
        "../../include/unlook/gui/styles/display_metrics.hpp",
        "../../include/unlook/gui/styles/supernova_style.hpp",
        
        "ui/main_window.ui",
        "ui/camera_preview_widget.ui",
        "ui/depth_test_widget.ui", 
        "ui/options_widget.ui",
        
        "resources/icons.qrc",
        "resources/styles.qrc"
    ]
    
    Group {
        name: "UI Files"
        files: ["ui/*.ui"]
        fileTags: ["qt.designer.ui"]
    }
    
    Group {
        name: "Resources" 
        files: ["resources/*.qrc"]
        fileTags: ["qt.core.resource_data"]
    }
}