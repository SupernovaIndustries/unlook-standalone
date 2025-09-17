#pragma once

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QScrollArea>
#include <memory>

#include "unlook/camera/camera_system.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/status_display.hpp"

// Forward declarations
QT_BEGIN_NAMESPACE
namespace Ui { class OptionsWidget; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

/**
 * @brief Options widget for system configuration and status
 * 
 * Features:
 * - Calibration status display
 * - Camera configuration display
 * - System information panel
 * - Advanced settings access
 * - Calibration validation tools
 */
class OptionsWidget : public QWidget {
    Q_OBJECT
    
public:
    /**
     * @brief Constructor
     * @param camera_system Shared camera system instance
     * @param parent Parent widget
     */
    explicit OptionsWidget(std::shared_ptr<camera::CameraSystem> camera_system,
                          QWidget* parent = nullptr);
    
    /**
     * @brief Destructor
     */
    ~OptionsWidget();

protected:
    /**
     * @brief Handle show events
     */
    void showEvent(QShowEvent* event) override;

private slots:
    /**
     * @brief Refresh system status
     */
    void refreshSystemStatus();
    
    /**
     * @brief Open calibration validation tools
     */
    void openCalibrationValidation();
    
    /**
     * @brief Open advanced camera settings
     */
    void openAdvancedCameraSettings();
    
    /**
     * @brief Reset to factory defaults
     */
    void resetToDefaults();
    
    /**
     * @brief Show about dialog
     */
    void showAboutDialog();

    /**
     * @brief Open AS1170 LED debug system
     */
    void openAS1170DebugSystem();

private:
    /**
     * @brief Initialize the widget UI
     */
    void initializeUI();
    
    /**
     * @brief Connect UI signals to slots
     */
    void connectSignals();
    
    // UI Components  
    Ui::OptionsWidget *ui;
    
    /**
     * @brief Create system status panel
     */
    QWidget* createSystemStatusPanel();
    
    /**
     * @brief Create calibration panel
     */
    QWidget* createCalibrationPanel();
    
    /**
     * @brief Create camera configuration panel
     */
    QWidget* createCameraConfigPanel();
    
    /**
     * @brief Create system information panel
     */
    QWidget* createSystemInfoPanel();
    
    /**
     * @brief Create action buttons panel
     */
    QWidget* createActionButtonsPanel();
    
    /**
     * @brief Update all status displays
     */
    void updateStatusDisplays();
    
    // System integration
    std::shared_ptr<camera::CameraSystem> camera_system_;
    
    // UI Layout
    QVBoxLayout* main_layout_;
    QScrollArea* scroll_area_;
    QWidget* content_widget_;
    
    // Status displays
    widgets::StatusDisplay* system_status_;
    widgets::StatusDisplay* calibration_status_;
    widgets::StatusDisplay* left_camera_status_;
    widgets::StatusDisplay* right_camera_status_;
    widgets::StatusDisplay* sync_status_;
    
    // Information displays
    QLabel* system_info_label_;
    QLabel* camera_info_label_;
    QLabel* version_info_label_;
    
    // UI widgets accessed via ui-> (defined in .ui file)
    // Removed duplicates: all TouchButton members
};

} // namespace gui
} // namespace unlook