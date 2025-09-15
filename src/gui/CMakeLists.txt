# GUI Application - Qt-based calibration interface

# Check if Qt5 is available
if(NOT BUILD_GUI)
    message(STATUS "GUI disabled, skipping")
    return()
endif()

# Find Qt5 components
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui OpenGL)

# Enable Qt5 MOC, UIC, RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Set UIC search paths
set(CMAKE_AUTOUIC_SEARCH_PATHS ui)

# GUI source files (only existing files)
set(GUI_SOURCES
    ../main.cpp
    main_window.cpp
    camera_preview_widget.cpp
    depth_test_widget.cpp
    options_widget.cpp
    widgets/parameter_slider.cpp
    widgets/status_display.cpp
    widgets/touch_button.cpp
    styles/supernova_style.cpp
    styles/display_metrics.cpp
)

# GUI header files
set(GUI_HEADERS
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/main_window.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/camera_preview_widget.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/depth_test_widget.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/options_widget.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/widgets/parameter_slider.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/widgets/status_display.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/widgets/touch_button.hpp
    ${CMAKE_SOURCE_DIR}/include/unlook/gui/styles/display_metrics.hpp
)

# UI files
set(GUI_UI_FILES
    ui/main_window.ui
    ui/camera_preview_widget.ui
    ui/depth_test_widget.ui
    ui/options_widget.ui
)

# Resource files
set(GUI_RESOURCES
    resources/icons.qrc
    resources/styles.qrc
)

# Create directories for UI files and resources
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ui)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/resources/styles)

# Create the GUI executable (AUTOMOC handles headers automatically)
add_executable(unlook_scanner
    ${GUI_SOURCES}
    ${GUI_HEADERS}
    ${GUI_UI_FILES}
    ${GUI_RESOURCES}
)

# Set executable properties
set_target_properties(unlook_scanner PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    WIN32_EXECUTABLE TRUE
)

# Include directories
target_include_directories(unlook_scanner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Link libraries
target_link_libraries(unlook_scanner PRIVATE
    unlook                          # Core Unlook library (includes stereo and calibration)
    unlook_stereo                   # Explicitly link stereo library
    unlook_calibration              # Explicitly link calibration library
    unlook_validation               # Explicitly link validation library
    ${OpenCV_LIBS}                  # Explicitly link OpenCV (including ximgproc, photo)
    Qt5::Core
    Qt5::Widgets
    Qt5::Gui
    Qt5::OpenGL
)

# OpenGL support for 3D visualization
find_package(OpenGL REQUIRED)
target_link_libraries(unlook_scanner PRIVATE ${OPENGL_LIBRARIES})

# Platform-specific configurations
if(WIN32)
    # Windows-specific settings
    set_target_properties(unlook_scanner PROPERTIES
        WIN32_EXECUTABLE TRUE
    )
elseif(APPLE)
    # macOS-specific settings
    set_target_properties(unlook_scanner PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_BUNDLE_NAME "Unlook Scanner"
        MACOSX_BUNDLE_GUI_IDENTIFIER "tech.unlook.scanner"
    )
elseif(UNIX)
    # Linux-specific settings
    target_compile_definitions(unlook_scanner PRIVATE UNLOOK_LINUX=1)
    
    # Desktop integration (commented out - missing desktop file)
    # configure_file(
    #     ${CMAKE_CURRENT_SOURCE_DIR}/resources/unlook-scanner.desktop.in
    #     ${CMAKE_CURRENT_BINARY_DIR}/unlook-scanner.desktop
    #     @ONLY
    # )
endif()

# Install the executable
install(TARGETS unlook_scanner
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    BUNDLE DESTINATION .
)

# Install desktop files on Linux
if(UNIX AND NOT APPLE)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/unlook-scanner.desktop
        DESTINATION ${CMAKE_INSTALL_DATADIR}/applications
        OPTIONAL
    )
    
    # Install icon
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/unlook-scanner.png)
        install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons/unlook-scanner.png
            DESTINATION ${CMAKE_INSTALL_DATADIR}/pixmaps
        )
    endif()
endif()

# Create resource files if they don't exist
set(ICONS_QRC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/resources/icons.qrc)
if(NOT EXISTS ${ICONS_QRC_FILE})
    file(WRITE ${ICONS_QRC_FILE} "<!DOCTYPE RCC>\n<RCC version=\"1.0\">\n<qresource>\n</qresource>\n</RCC>\n")
endif()

set(STYLES_QRC_FILE ${CMAKE_CURRENT_SOURCE_DIR}/resources/styles.qrc)  
if(NOT EXISTS ${STYLES_QRC_FILE})
    # Create styles.qrc with tailwind theme
    file(WRITE ${STYLES_QRC_FILE} "<!DOCTYPE RCC>\n<RCC version=\"1.0\">\n<qresource prefix=\"/styles\">\n    <file>styles/tailwind_theme.qss</file>\n</qresource>\n</RCC>\n")
endif()

# Example applications (commented out - no gui examples directory)
# if(BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

message(STATUS "GUI application configured: unlook_scanner")
message(STATUS "GUI components: calibration, camera control, depth processing")