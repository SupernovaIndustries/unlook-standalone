#include "unlook/gui/main_window.hpp"
#include "unlook/gui/camera_preview_widget.hpp"
#include "unlook/gui/handheld_scan_widget.hpp"
// #include "unlook/gui/face_enrollment_widget.hpp"  // Temporarily disabled
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
#include <QFile>
#include <QPalette>

#include "ui_main_window.h"

using namespace unlook::gui::styles;
using namespace unlook::gui::widgets;

namespace unlook {
namespace gui {

UnlookMainWindow::UnlookMainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::UnlookMainWindow)
    , current_screen_(Screen::MAIN_MENU)
    , camera_system_(nullptr)
    , is_fullscreen_(true)
    , camera_system_initialized_(false)
    , status_update_timer_(nullptr)
{
    // Setup UI from .ui file
    ui->setupUi(this);

    // Remove window title bar and frame for clean 800x400 display
    setWindowFlags(Qt::Window | Qt::FramelessWindowHint);

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
    
    // Initialize additional UI components not in .ui file
    initializeAdditionalComponents();
    
    // NOTE: applySupernovanStyling() moved to showEvent() to apply AFTER all child widgets
    
    // Initialize camera system
    initializeCameraSystem();
    
    // Setup navigation
    setupNavigation();
    
    // ULTIMATE BRUTAL FIX: Set central widget explicitly and force layout
    setCentralWidget(ui->central_widget);  // FORCE QMainWindow to use our central widget!
    
    // BRUTAL TEST: Force background with QPalette instead of QSS
    QPalette palette = this->palette();
    palette.setColor(QPalette::Window, QColor("#e0e5ec"));
    palette.setColor(QPalette::Base, QColor("#e0e5ec"));
    this->setPalette(palette);
    this->setAutoFillBackground(true);
    
    // Also force on central widget
    if (ui->central_widget) {
        ui->central_widget->setPalette(palette);
        ui->central_widget->setAutoFillBackground(true);
        ui->central_widget->show();  // FORCE VISIBILITY!
        ui->central_widget->setVisible(true);  // DOUBLE FORCE!
        qDebug() << "[MainWindow] FORCED central_widget visibility: show() + setVisible(true)";
    }
    
    // Also force on stacked widget
    if (ui->screen_stack) {
        ui->screen_stack->setPalette(palette);
        ui->screen_stack->setAutoFillBackground(true);
        ui->screen_stack->show();  // FORCE VISIBILITY!
        ui->screen_stack->setVisible(true);  // DOUBLE FORCE!
        qDebug() << "[MainWindow] FORCED screen_stack visibility: show() + setVisible(true)";
    }
    
    // DEBUG: Check critical widgets after forcing visibility
    if (ui->central_widget) {
        qDebug() << "[MainWindow] central_widget AFTER FORCE - geometry:" << ui->central_widget->geometry() 
                 << "visible:" << ui->central_widget->isVisible() << "size:" << ui->central_widget->size();
    }
    if (ui->screen_stack) {
        qDebug() << "[MainWindow] screen_stack AFTER FORCE - geometry:" << ui->screen_stack->geometry() 
                 << "visible:" << ui->screen_stack->isVisible() << "size:" << ui->screen_stack->size();
    }
    
    // Force layout update and proper sizing
    this->updateGeometry();
    this->resize(1824, 1026);  // From the adaptive sizing
    
    // Force layout processing
    QApplication::processEvents();
    
    // Force layout update on all widgets
    if (ui->central_widget) {
        ui->central_widget->updateGeometry();
    }
    if (ui->screen_stack) {
        ui->screen_stack->updateGeometry();
    }
    
    // DEBUG: List all child widgets to see if something is covering
    QObjectList children = this->children();
    qDebug() << "[MainWindow] Total child widgets:" << children.size();
    for (QObject* child : children) {
        QWidget* widget = qobject_cast<QWidget*>(child);
        if (widget) {
            qDebug() << "  Child widget:" << widget->objectName() << "geometry:" << widget->geometry() 
                     << "visible:" << widget->isVisible() << "styleSheet:" << widget->styleSheet().left(50);
        }
    }
    
    qDebug() << "[MainWindow] BRUTAL PALETTE TEST: Forced #e0e5ec background with QPalette and AutoFill";
    
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
    delete ui;
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
    
    // Apply Tailwind theme AFTER all child widgets have been initialized and styled
    // This ensures our theme overrides any SupernovaStyle applied by child widgets
    static bool theme_applied = false;
    if (!theme_applied) {
        // TEST: Apply direct background color to debug
        QString testStyle = "QMainWindow { background-color: #e0e5ec !important; } QWidget { background-color: #e0e5ec !important; }";
        setStyleSheet(testStyle);
        qDebug() << "[MainWindow] DIRECT TEST STYLE applied: " << testStyle;
        
        // Also try applySupernovanStyling
        applySupernovanStyling();
        theme_applied = true;
        qDebug() << "[MainWindow] Both direct and file theme applied in showEvent";
    }
    
    // Ensure fullscreen on show
    if (is_fullscreen_) {
        showFullScreen();
    }
}

void UnlookMainWindow::showCameraPreview() {
    navigateToScreen(Screen::CAMERA_PREVIEW);
}

void UnlookMainWindow::showHandheldScan() {
    navigateToScreen(Screen::HANDHELD_SCAN);
}

// void UnlookMainWindow::showFaceEnrollment() {
//     navigateToScreen(Screen::FACE_ENROLLMENT);
// }

void UnlookMainWindow::showOptions() {
    navigateToScreen(Screen::OPTIONS);
}

void UnlookMainWindow::showMainMenu() {
    navigateToScreen(Screen::MAIN_MENU);
}

void UnlookMainWindow::exitApplication() {
    // Exit immediately without confirmation popup
    close();
}

void UnlookMainWindow::toggleFullscreen() {
    if (is_fullscreen_) {
        showNormal();
        is_fullscreen_ = false;
        system_status_->setStatus("Windowed mode (ESC to toggle)", StatusDisplay::StatusType::INFO);
    } else {
        showFullScreen();
        is_fullscreen_ = true;
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
        ui->camera_status_label->setText("Cameras: Ready");
        ui->camera_status_label->setStyleSheet("color: #00FF00;");
    } else if (left_ready || right_ready) {
        ui->camera_status_label->setText("Cameras: Partial Connection");
        ui->camera_status_label->setStyleSheet("color: #FFA500;");
    } else {
        ui->camera_status_label->setText("Cameras: Disconnected");
        ui->camera_status_label->setStyleSheet("color: #FF0000;");
    }
}

void UnlookMainWindow::initializeAdditionalComponents() {
    // Connect UI signals to slots
    connect(ui->camera_preview_button, &QPushButton::clicked, this, &UnlookMainWindow::showCameraPreview);
    connect(ui->depth_test_button, &QPushButton::clicked, this, &UnlookMainWindow::showHandheldScan);
    // connect(ui->face_enrollment_button, &QPushButton::clicked, this, &UnlookMainWindow::showFaceEnrollment);  // Temporarily disabled
    connect(ui->options_button, &QPushButton::clicked, this, &UnlookMainWindow::showOptions);
    connect(ui->exit_button, &QPushButton::clicked, this, &UnlookMainWindow::exitApplication);
    connect(ui->back_button, &QPushButton::clicked, this, &UnlookMainWindow::showMainMenu);
    // fullscreen_toggle_button removed from UI
    
    // Initialize status displays (these will be custom widgets)
    system_status_ = new widgets::StatusDisplay("System");
    system_status_->setCompactMode(true);
    system_status_->setStatus("Initializing...", widgets::StatusDisplay::StatusType::INFO);
    
    camera_status_ = new widgets::StatusDisplay("Cameras");
    camera_status_->setCompactMode(true);
    camera_status_->setStatus("Disconnected", widgets::StatusDisplay::StatusType::WARNING);
    
    // Set initial screen visibility
    ui->back_button->setVisible(false);
}

// UI creation methods removed - now using .ui file

void UnlookMainWindow::applySupernovanStyling() {
    // Load and apply Tailwind theme from resources
    QFile styleFile(":/styles/styles/tailwind_theme.qss");
    qDebug() << "[MainWindow] Attempting to load theme from:" << styleFile.fileName();
    
    if (styleFile.open(QFile::ReadOnly)) {
        QString styleSheet = QString::fromUtf8(styleFile.readAll());
        qDebug() << "[MainWindow] Theme file size:" << styleSheet.length() << "characters";
        qDebug() << "[MainWindow] Theme preview:" << styleSheet.left(200) << "...";
        
        // Apply to main window with debug info
        setStyleSheet(styleSheet);
        
        qDebug() << "[MainWindow] Tailwind theme applied to MainWindow";
        qDebug() << "[MainWindow] MainWindow stylesheet length:" << this->styleSheet().length();
    } else {
        qWarning() << "[MainWindow] Failed to load Tailwind theme from resources, error:" << styleFile.errorString();
        qWarning() << "[MainWindow] Falling back to SupernovaStyle";
        // Fallback to SupernovaStyle if resource loading fails
        setStyleSheet(SupernovaStyle::getMainWindowStyle() + SupernovaStyle::getApplicationStyleSheet());
    }
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
                ui->system_status_label->setText("System: Ready");
                ui->system_status_label->setStyleSheet("color: #00FF00;");
                
                // UX IMPROVEMENT: Auto-start camera capture for better user experience
                qDebug() << "[MainWindow] Auto-starting camera capture for better UX...";
                
                // Create a dummy callback for continuous capture
                auto dummy_callback = [](const core::StereoFramePair& /*frame_pair*/) {
                    // Frame received but not processed - just keeps cameras running
                };
                
                if (camera_system_->startCapture(dummy_callback)) {
                    ui->system_status_label->setText("System: Ready & Capturing");
                    qDebug() << "[MainWindow] Camera auto-start successful";
                } else {
                    qDebug() << "[MainWindow] Camera auto-start failed - will remain in Ready state";
                }
            } else {
                ui->system_status_label->setText("System: Initialization Failed");
                ui->system_status_label->setStyleSheet("color: #FF0000;");
            }
        });
        
    } catch (const std::exception& e) {
        ui->system_status_label->setText("System Error: " + QString::fromStdString(e.what()));
        ui->system_status_label->setStyleSheet("color: #FF0000;");
        qCritical() << "Camera system initialization error:" << e.what();
    }
}

void UnlookMainWindow::setupNavigation() {
    // Create screen widgets (lazy initialization)
    // They will be created when first accessed
}

void UnlookMainWindow::navigateToScreen(Screen screen) {
    // Hide/show back button based on screen
    ui->back_button->setVisible(screen != Screen::MAIN_MENU);
    
    switch (screen) {
        case Screen::MAIN_MENU:
            ui->screen_stack->setCurrentWidget(ui->main_menu_screen);
            ui->title_label->setText("UNLOOK SCANNER");
            break;
            
        case Screen::CAMERA_PREVIEW:
            if (!camera_preview_widget_) {
                camera_preview_widget_ = std::make_unique<CameraPreviewWidget>(camera_system_);
                ui->screen_stack->addWidget(camera_preview_widget_.get());
            }
            ui->screen_stack->setCurrentWidget(camera_preview_widget_.get());
            ui->title_label->setText("CAMERA PREVIEW");
            break;
            
        case Screen::HANDHELD_SCAN:
            if (!handheld_scan_widget_) {
                handheld_scan_widget_ = std::make_unique<HandheldScanWidget>(camera_system_);
                ui->screen_stack->addWidget(handheld_scan_widget_.get());
            }
            ui->screen_stack->setCurrentWidget(handheld_scan_widget_.get());
            ui->title_label->setText("HANDHELD SCAN");
            break;

        // case Screen::FACE_ENROLLMENT:
        //     if (!face_enrollment_widget_) {
        //         face_enrollment_widget_ = std::make_unique<FaceEnrollmentWidget>(camera_system_);
        //         ui->screen_stack->addWidget(face_enrollment_widget_.get());
        //     }
        //     ui->screen_stack->setCurrentWidget(face_enrollment_widget_.get());
        //     ui->title_label->setText("FACE ENROLLMENT");
        //     break;

        case Screen::OPTIONS:
            if (!options_widget_) {
                options_widget_ = std::make_unique<OptionsWidget>(camera_system_);
                ui->screen_stack->addWidget(options_widget_.get());
            }
            ui->screen_stack->setCurrentWidget(options_widget_.get());
            ui->title_label->setText("OPTIONS");
            break;
    }
    
    current_screen_ = screen;
    updateNavigationButtons();
}

void UnlookMainWindow::updateNavigationButtons() {
    // Enable/disable navigation buttons based on camera system state
    bool camera_ready = camera_system_initialized_ && camera_system_ && camera_system_->isReady();
    
    ui->camera_preview_button->setEnabled(camera_ready);
    ui->depth_test_button->setEnabled(camera_ready);
    
    // Options is always available
    ui->options_button->setEnabled(true);
}

} // namespace gui
} // namespace unlook

// moc include removed - handled by CMake