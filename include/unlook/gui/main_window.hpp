#pragma once

#include <QMainWindow>
#include <QStackedWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QKeyEvent>
#include <QTimer>
#include <memory>

#include "unlook/camera/camera_system.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/status_display.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class UnlookMainWindow; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

// Forward declarations
class CameraPreviewWidget;
class DepthTestWidget;
class OptionsWidget;
class FaceEnrollmentWidget;

/**
 * @brief Main window for Unlook 3D Scanner with fullscreen touch interface
 * 
 * Features:
 * - Always starts in fullscreen mode (hide window decorations)
 * - Touch-optimized interface for Raspberry Pi touchscreen
 * - Escape key toggles windowed mode for development
 * - Screen navigation with stack widget
 * - Shared camera system integration
 * - Professional industrial interface design
 */
class UnlookMainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    enum class Screen {
        MAIN_MENU,
        CAMERA_PREVIEW,
        DEPTH_TEST,
        FACE_ENROLLMENT,
        OPTIONS
    };
    
    /**
     * @brief Constructor
     * @param parent Parent widget
     */
    explicit UnlookMainWindow(QWidget* parent = nullptr);
    
    /**
     * @brief Destructor
     */
    ~UnlookMainWindow();

protected:
    /**
     * @brief Handle key press events (Escape for window mode toggle)
     */
    void keyPressEvent(QKeyEvent* event) override;
    
    /**
     * @brief Handle close events
     */
    void closeEvent(QCloseEvent* event) override;
    
    /**
     * @brief Handle window show events
     */
    void showEvent(QShowEvent* event) override;

private slots:
    /**
     * @brief Navigate to camera preview screen
     */
    void showCameraPreview();
    
    /**
     * @brief Navigate to depth test screen
     */
    void showDepthTest();
    
    /**
     * @brief Navigate to face enrollment screen
     */
    void showFaceEnrollment();

    /**
     * @brief Navigate to options screen
     */
    void showOptions();

    /**
     * @brief Navigate back to main menu
     */
    void showMainMenu();
    
    /**
     * @brief Exit application
     */
    void exitApplication();
    
    /**
     * @brief Toggle between fullscreen and windowed mode
     */
    void toggleFullscreen();
    
    /**
     * @brief Handle camera system status updates
     */
    void onCameraSystemStatusChanged();
    
    /**
     * @brief Handle camera system errors
     */
    void onCameraSystemError(const QString& error);
    
    /**
     * @brief Update system status display
     */
    void updateSystemStatus();

private:
    /**
     * @brief Initialize additional UI components not in .ui file
     */
    void initializeAdditionalComponents();
    
    /**
     * @brief Apply Supernova styling to all components
     */
    void applySupernovanStyling();
    
    /**
     * @brief Initialize camera system
     */
    void initializeCameraSystem();
    
    /**
     * @brief Setup screen navigation
     */
    void setupNavigation();
    
    /**
     * @brief Navigate to specific screen
     */
    void navigateToScreen(Screen screen);
    
    /**
     * @brief Update navigation buttons based on current screen
     */
    void updateNavigationButtons();
    
    // UI Components
    Ui::UnlookMainWindow *ui;
    
    // Custom status displays (not in .ui file)
    widgets::StatusDisplay* system_status_;
    widgets::StatusDisplay* camera_status_;
    
    // Screen management
    Screen current_screen_;
    
    // Screen widgets
    std::unique_ptr<CameraPreviewWidget> camera_preview_widget_;
    std::unique_ptr<DepthTestWidget> depth_test_widget_;
    std::unique_ptr<FaceEnrollmentWidget> face_enrollment_widget_;
    std::unique_ptr<OptionsWidget> options_widget_;
    
    // System integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    
    // State management
    bool is_fullscreen_;
    bool camera_system_initialized_;
    QTimer* status_update_timer_;
};

} // namespace gui
} // namespace unlook