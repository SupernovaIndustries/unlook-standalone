#include "unlook/gui/main_window.hpp"
#include "unlook/gui/camera_preview_widget.hpp"
#include "unlook/gui/depth_test_widget.hpp"
#include "unlook/gui/options_widget.hpp"
#include "unlook/gui/styles/supernova_style.hpp"
#include "unlook/gui/styles/display_metrics.hpp"

#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QSpacerItem>
#include <QLabel>
#include <QKeyEvent>
#include <QCloseEvent>
#include <QShowEvent>
#include <QTimer>
#include <QMessageBox>
#include <QDebug>
#include <QDateTime>

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

UnlookMainWindow::UnlookMainWindow(QWidget* parent)
    : QMainWindow(parent)
    , central_widget_(nullptr)
    , main_layout_(nullptr)
    , title_bar_(nullptr)
    , title_label_(nullptr)
    , version_label_(nullptr)
    , back_button_(nullptr)
    , fullscreen_toggle_button_(nullptr)
    , status_bar_(nullptr)
    , system_status_(nullptr)
    , camera_status_(nullptr)
    , screen_stack_(nullptr)
    , current_screen_(Screen::MAIN_MENU)
    , main_menu_screen_(nullptr)
    , camera_preview_button_(nullptr)
    , depth_test_button_(nullptr)
    , options_button_(nullptr)
    , exit_button_(nullptr)
    , camera_system_(nullptr)
    , is_fullscreen_(true)
    , camera_system_initialized_(false)
    , status_update_timer_(nullptr)
{
    setWindowTitle("Unlook 3D Scanner v1.0");
    
    // Initialize responsive display metrics system FIRST
    styles::DisplayMetrics::initialize();
    
    // Set adaptive minimum size based on screen detection
    const auto& metrics = styles::DisplayMetrics::instance();
    QSize screen_size = metrics.screenSize();
    
    // Adaptive minimum size: use 95% of detected screen size to ensure fit
    int min_width = static_cast<int>(screen_size.width() * 0.95);
    int min_height = static_cast<int>(screen_size.height() * 0.95);
    setMinimumSize(min_width, min_height);
    
    qDebug() << "Adaptive sizing:" << metrics.getDisplayInfo();
    qDebug() << "Window minimum size set to:" << min_width << "x" << min_height;
    
    // Initialize UI first
    initializeUI();
    
    // Apply Supernova styling
    applySupernovanStyling();
    
    // Initialize camera system
    initializeCameraSystem();
    
    // Setup navigation
    setupNavigation();
    
    // Start fullscreen
    showFullScreen();
    is_fullscreen_ = true;
    
    // Setup status update timer
    status_update_timer_ = new QTimer(this);
    connect(status_update_timer_, &QTimer::timeout, this, &UnlookMainWindow::updateSystemStatus);
    status_update_timer_->start(1000); // Update every second
    
    qDebug() << "UnlookMainWindow initialized";
}

UnlookMainWindow::~UnlookMainWindow() {
    if (camera_system_) {
        camera_system_->shutdown();
    }
    qDebug() << "UnlookMainWindow destroyed";
}

void UnlookMainWindow::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Escape) {
        toggleFullscreen();
        event->accept();
        return;
    }
    
    // Handle back navigation
    if (event->key() == Qt::Key_Backspace || event->key() == Qt::Key_Back) {
        if (current_screen_ != Screen::MAIN_MENU) {
            showMainMenu();
            event->accept();
            return;
        }
    }
    
    QMainWindow::keyPressEvent(event);
}

void UnlookMainWindow::closeEvent(QCloseEvent* event) {
    // Graceful shutdown
    if (camera_system_) {
        camera_system_->shutdown();
    }
    
    event->accept();
    QApplication::quit();
}

void UnlookMainWindow::showEvent(QShowEvent* event) {
    QMainWindow::showEvent(event);
    
    // Ensure fullscreen on show
    if (is_fullscreen_) {
        showFullScreen();
    }
}

void UnlookMainWindow::showCameraPreview() {
    navigateToScreen(Screen::CAMERA_PREVIEW);
}

void UnlookMainWindow::showDepthTest() {
    navigateToScreen(Screen::DEPTH_TEST);
}

void UnlookMainWindow::showOptions() {
    navigateToScreen(Screen::OPTIONS);
}

void UnlookMainWindow::showMainMenu() {
    navigateToScreen(Screen::MAIN_MENU);
}

void UnlookMainWindow::exitApplication() {
    // Show confirmation dialog if in fullscreen
    if (is_fullscreen_) {
        QMessageBox msgBox;
        msgBox.setWindowTitle("Exit Confirmation");
        msgBox.setText("Are you sure you want to exit the Unlook Scanner?");
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        msgBox.setDefaultButton(QMessageBox::No);
        
        // Apply Supernova styling to message box
        SupernovaStyle::applyStyle(&msgBox);
        
        if (msgBox.exec() == QMessageBox::Yes) {
            close();
        }
    } else {
        close();
    }
}

void UnlookMainWindow::toggleFullscreen() {
    if (is_fullscreen_) {
        showNormal();
        is_fullscreen_ = false;
        fullscreen_toggle_button_->setText("Fullscreen");
        system_status_->setStatus("Windowed mode (ESC to toggle)", StatusDisplay::StatusType::INFO);
    } else {
        showFullScreen();
        is_fullscreen_ = true;
        fullscreen_toggle_button_->setText("Window");
        system_status_->setStatus("Fullscreen mode", StatusDisplay::StatusType::SUCCESS);
    }
}

void UnlookMainWindow::onCameraSystemStatusChanged() {
    updateSystemStatus();
}

void UnlookMainWindow::onCameraSystemError(const QString& error) {
    camera_status_->setStatus("Camera Error: " + error, StatusDisplay::StatusType::ERROR);
    qWarning() << "Camera system error:" << error;
}

void UnlookMainWindow::updateSystemStatus() {
    if (!camera_system_) return;
    
    // Update camera status
    bool left_ready = camera_system_->getCameraState(core::CameraId::LEFT) == core::CameraState::READY;
    bool right_ready = camera_system_->getCameraState(core::CameraId::RIGHT) == core::CameraState::READY;
    
    if (left_ready && right_ready) {
        camera_status_->setStatus("Cameras Ready", StatusDisplay::StatusType::SUCCESS);
    } else if (left_ready || right_ready) {
        camera_status_->setStatus("Partial Camera Connection", StatusDisplay::StatusType::WARNING);
    } else {
        camera_status_->setStatus("Cameras Disconnected", StatusDisplay::StatusType::ERROR);
    }
}

void UnlookMainWindow::initializeUI() {
    // Create central widget
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);
    
    // Create main layout
    main_layout_ = new QVBoxLayout(central_widget_);
    main_layout_->setSpacing(0);
    main_layout_->setContentsMargins(0, 0, 0, 0);
    
    // Create title bar
    title_bar_ = createTitleBar();
    main_layout_->addWidget(title_bar_);
    
    // Create screen stack
    screen_stack_ = new QStackedWidget();
    main_layout_->addWidget(screen_stack_, 1); // Stretch factor 1
    
    // Create main menu screen
    main_menu_screen_ = createMainMenuScreen();
    screen_stack_->addWidget(main_menu_screen_);
    
    // Create status bar
    status_bar_ = createStatusBar();
    main_layout_->addWidget(status_bar_);
    
    // Set initial screen
    screen_stack_->setCurrentWidget(main_menu_screen_);
}

QWidget* UnlookMainWindow::createMainMenuScreen() {
    const auto& metrics = styles::DisplayMetrics::instance();
    
    QWidget* menu_widget = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(menu_widget);
    
    // Use ultra-compact spacing for SMALL screens
    int margin_large = metrics.shouldUseCompactMode() ? 4 : metrics.getMargin(SupernovaStyle::Spacing::MARGIN_LARGE);
    int spacing = metrics.shouldUseCompactMode() ? 2 : metrics.getSpacing(SupernovaStyle::Spacing::MARGIN_LARGE);
    
    layout->setSpacing(spacing);
    layout->setContentsMargins(margin_large, margin_large, margin_large, margin_large);
    
    // Conditional title display - hide on SMALL screens to save space
    if (!metrics.shouldUseCompactMode()) {
        // Add title with responsive fonts (only on larger screens)
        QLabel* welcome_label = new QLabel("UNLOOK 3D SCANNER");
        welcome_label->setAlignment(Qt::AlignCenter);
        welcome_label->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::DISPLAY), QFont::Bold));
        welcome_label->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
        layout->addWidget(welcome_label);
        
        QLabel* subtitle_label = new QLabel("Professional 3D Scanning System");
        subtitle_label->setAlignment(Qt::AlignCenter);
        subtitle_label->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::SUBTITLE)));
        subtitle_label->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
        layout->addWidget(subtitle_label);
        
        // Add spacer for non-compact mode only
        layout->addSpacerItem(new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding));
    }
    
    // Create responsive button grid optimized for screen size
    QGridLayout* button_layout = new QGridLayout();
    
    int button_spacing = metrics.shouldUseCompactMode() ? 4 : metrics.getButtonSpacing();
    int margin_medium = metrics.shouldUseCompactMode() ? 2 : metrics.getMargin(SupernovaStyle::Spacing::MARGIN_MEDIUM);
    int margin_small = metrics.shouldUseCompactMode() ? 2 : metrics.getMargin(SupernovaStyle::Spacing::MARGIN_SMALL);
    
    button_layout->setSpacing(button_spacing);
    button_layout->setContentsMargins(margin_medium, margin_small, margin_medium, margin_small);
    
    // Get responsive button size - optimized for 840x480 2x2 grid
    QSize button_size = metrics.getMainButtonSize();
    
    // Create main buttons with responsive sizes
    camera_preview_button_ = new TouchButton("CAMERA PREVIEW", TouchButton::ButtonType::PRIMARY);
    camera_preview_button_->setFixedSize(button_size);
    camera_preview_button_->setIcon(QIcon(":/icons/camera.png"));
    connect(camera_preview_button_, &TouchButton::clicked, this, &UnlookMainWindow::showCameraPreview);
    
    depth_test_button_ = new TouchButton("DEPTH TEST", TouchButton::ButtonType::SECONDARY);
    depth_test_button_->setFixedSize(button_size);
    depth_test_button_->setIcon(QIcon(":/icons/depth.png"));
    connect(depth_test_button_, &TouchButton::clicked, this, &UnlookMainWindow::showDepthTest);
    
    options_button_ = new TouchButton("OPTIONS", TouchButton::ButtonType::PRIMARY);
    options_button_->setFixedSize(button_size);
    options_button_->setIcon(QIcon(":/icons/settings.png"));
    connect(options_button_, &TouchButton::clicked, this, &UnlookMainWindow::showOptions);
    
    exit_button_ = new TouchButton("EXIT", TouchButton::ButtonType::ERROR);
    exit_button_->setFixedSize(button_size);
    exit_button_->setIcon(QIcon(":/icons/exit.png"));
    connect(exit_button_, &TouchButton::clicked, this, &UnlookMainWindow::exitApplication);
    
    // Arrange buttons in grid
    button_layout->addWidget(camera_preview_button_, 0, 0);
    button_layout->addWidget(depth_test_button_, 0, 1);
    button_layout->addWidget(options_button_, 1, 0);
    button_layout->addWidget(exit_button_, 1, 1);
    
    layout->addLayout(button_layout);
    
    // Add minimal spacer only for non-compact mode
    if (!metrics.shouldUseCompactMode()) {
        layout->addSpacerItem(new QSpacerItem(20, 20, QSizePolicy::Minimum, QSizePolicy::Expanding));
    }
    
    return menu_widget;
}

QWidget* UnlookMainWindow::createTitleBar() {
    const auto& metrics = styles::DisplayMetrics::instance();
    
    QWidget* title_widget = new QWidget();
    title_widget->setFixedHeight(metrics.getTitleBarHeight());
    
    QHBoxLayout* layout = new QHBoxLayout(title_widget);
    
    int padding_medium = metrics.shouldUseCompactMode() ? 2 : metrics.getPadding(SupernovaStyle::Spacing::PADDING_MEDIUM);
    int padding_small = metrics.shouldUseCompactMode() ? 1 : metrics.getPadding(SupernovaStyle::Spacing::PADDING_SMALL);
    
    layout->setContentsMargins(padding_medium, padding_small, padding_medium, padding_small);
    
    // Back button (hidden on main menu) - more compact on SMALL screens
    back_button_ = new TouchButton("◀ BACK", TouchButton::ButtonType::SECONDARY);
    back_button_->setVisible(false);
    back_button_->setMaximumWidth(metrics.shouldUseCompactMode() ? 80 : 120);
    if (metrics.shouldUseCompactMode()) {
        back_button_->setMaximumHeight(30);
    }
    connect(back_button_, &TouchButton::clicked, this, &UnlookMainWindow::showMainMenu);
    layout->addWidget(back_button_);
    
    // Title with responsive font
    title_label_ = new QLabel("UNLOOK SCANNER");
    title_label_->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::TITLE), QFont::Bold));
    title_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::ELECTRIC_PRIMARY)));
    layout->addWidget(title_label_, 1); // Stretch factor 1
    
    // Version label with responsive font
    version_label_ = new QLabel("v1.0");
    version_label_->setFont(metrics.getResponsiveFont(static_cast<int>(SupernovaStyle::FontSize::SMALL)));
    version_label_->setStyleSheet(QString("color: %1;").arg(SupernovaStyle::colorToString(SupernovaStyle::TEXT_SECONDARY)));
    layout->addWidget(version_label_);
    
    // Fullscreen toggle button - more compact on SMALL screens
    fullscreen_toggle_button_ = new TouchButton(metrics.shouldUseCompactMode() ? "Win" : "Window", TouchButton::ButtonType::SECONDARY);
    fullscreen_toggle_button_->setMaximumWidth(metrics.shouldUseCompactMode() ? 60 : 120);
    if (metrics.shouldUseCompactMode()) {
        fullscreen_toggle_button_->setMaximumHeight(30);
    }
    connect(fullscreen_toggle_button_, &TouchButton::clicked, this, &UnlookMainWindow::toggleFullscreen);
    layout->addWidget(fullscreen_toggle_button_);
    
    // Apply title bar styling
    title_widget->setStyleSheet(QString("background: %1; border-bottom: 2px solid %2;")
                               .arg(SupernovaStyle::colorToString(SupernovaStyle::NEBULA_SURFACE))
                               .arg(SupernovaStyle::colorToString(SupernovaStyle::QUANTUM_DEPTH)));
    
    return title_widget;
}

QWidget* UnlookMainWindow::createStatusBar() {
    const auto& metrics = styles::DisplayMetrics::instance();
    
    QWidget* status_widget = new QWidget();
    status_widget->setFixedHeight(metrics.getStatusBarHeight());
    
    QHBoxLayout* layout = new QHBoxLayout(status_widget);
    
    int padding_medium = metrics.shouldUseCompactMode() ? 2 : metrics.getPadding(SupernovaStyle::Spacing::PADDING_MEDIUM);
    int padding_small = metrics.shouldUseCompactMode() ? 1 : metrics.getPadding(SupernovaStyle::Spacing::PADDING_SMALL);
    
    layout->setContentsMargins(padding_medium, padding_small, padding_medium, padding_small);
    
    // System status
    system_status_ = new StatusDisplay("System");
    system_status_->setCompactMode(true);
    system_status_->setStatus("Initializing...", StatusDisplay::StatusType::INFO);
    layout->addWidget(system_status_, 1);
    
    // Camera status
    camera_status_ = new StatusDisplay("Cameras");
    camera_status_->setCompactMode(true);
    camera_status_->setStatus("Disconnected", StatusDisplay::StatusType::WARNING);
    layout->addWidget(camera_status_, 1);
    
    // Apply status bar styling
    status_widget->setStyleSheet(QString("background: %1; border-top: 1px solid %2;")
                                .arg(SupernovaStyle::colorToString(SupernovaStyle::NEBULA_SURFACE))
                                .arg(SupernovaStyle::colorToString(SupernovaStyle::QUANTUM_DEPTH)));
    
    return status_widget;
}

void UnlookMainWindow::applySupernovanStyling() {
    // Apply main window style
    setStyleSheet(SupernovaStyle::getMainWindowStyle() + SupernovaStyle::getApplicationStyleSheet());
}

void UnlookMainWindow::initializeCameraSystem() {
    try {
        camera_system_ = camera::CameraSystem::getInstance();
        
        // Set error callback
        camera_system_->setErrorCallback([this](const std::string& error) {
            QTimer::singleShot(0, this, [this, error]() {
                onCameraSystemError(QString::fromStdString(error));
            });
        });
        
        // Initialize in background thread to avoid blocking UI
        QTimer::singleShot(100, this, [this]() {
            bool success = camera_system_->initialize();
            camera_system_initialized_ = success;
            
            if (success) {
                system_status_->setStatus("System Ready", StatusDisplay::StatusType::SUCCESS);
            } else {
                system_status_->setStatus("Initialization Failed", StatusDisplay::StatusType::ERROR);
            }
        });
        
    } catch (const std::exception& e) {
        system_status_->setStatus("System Error: " + QString::fromStdString(e.what()), 
                                 StatusDisplay::StatusType::ERROR);
        qCritical() << "Camera system initialization error:" << e.what();
    }
}

void UnlookMainWindow::setupNavigation() {
    // Create screen widgets (lazy initialization)
    // They will be created when first accessed
}

void UnlookMainWindow::navigateToScreen(Screen screen) {
    // Hide/show back button based on screen
    back_button_->setVisible(screen != Screen::MAIN_MENU);
    
    switch (screen) {
        case Screen::MAIN_MENU:
            screen_stack_->setCurrentWidget(main_menu_screen_);
            title_label_->setText("UNLOOK SCANNER");
            break;
            
        case Screen::CAMERA_PREVIEW:
            if (!camera_preview_widget_) {
                camera_preview_widget_ = std::make_unique<CameraPreviewWidget>(camera_system_);
                screen_stack_->addWidget(camera_preview_widget_.get());
            }
            screen_stack_->setCurrentWidget(camera_preview_widget_.get());
            title_label_->setText("CAMERA PREVIEW");
            break;
            
        case Screen::DEPTH_TEST:
            if (!depth_test_widget_) {
                depth_test_widget_ = std::make_unique<DepthTestWidget>(camera_system_);
                screen_stack_->addWidget(depth_test_widget_.get());
            }
            screen_stack_->setCurrentWidget(depth_test_widget_.get());
            title_label_->setText("DEPTH TEST");
            break;
            
        case Screen::OPTIONS:
            if (!options_widget_) {
                options_widget_ = std::make_unique<OptionsWidget>(camera_system_);
                screen_stack_->addWidget(options_widget_.get());
            }
            screen_stack_->setCurrentWidget(options_widget_.get());
            title_label_->setText("OPTIONS");
            break;
    }
    
    current_screen_ = screen;
    updateNavigationButtons();
}

void UnlookMainWindow::updateNavigationButtons() {
    // Enable/disable navigation buttons based on camera system state
    bool camera_ready = camera_system_initialized_ && camera_system_ && camera_system_->isReady();
    
    camera_preview_button_->setEnabled(camera_ready);
    depth_test_button_->setEnabled(camera_ready);
    
    // Options is always available
    options_button_->setEnabled(true);
}

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake