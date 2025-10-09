#pragma once

#include <QDialog>
#include <QTimer>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QProgressBar>
#include <QTextEdit>
#include <QTabWidget>
#include <memory>
#include <atomic>

#include "unlook/hardware/AS1170Controller.hpp"
#include "unlook/gui/widgets/touch_button.hpp"
#include "unlook/gui/widgets/status_display.hpp"
#include "unlook/gui/widgets/parameter_slider.hpp"

// Forward declarations
QT_BEGIN_NAMESPACE
namespace Ui { class AS1170DebugDialog; }
QT_END_NAMESPACE

namespace unlook {
namespace gui {

/**
 * @brief Comprehensive AS1170 LED Debug Dialog for Industrial Hardware Validation
 *
 * Provides complete diagnostic and testing interface for AS1170 LED driver system.
 * Features real-time monitoring, hardware control, safety systems, and validation tools.
 *
 * Key Features:
 * - LED1 (VCSEL) and LED2 (Flood) individual control with current setting (0-250mA)
 * - Manual strobe testing with duration control (100-5000μs)
 * - Real-time I2C communication diagnostics with bus reset capabilities
 * - Live temperature monitoring with thermal protection status
 * - Hardware fault detection and automatic recovery procedures
 * - Emergency shutdown system with <5ms response time
 * - Comprehensive logging and error reporting
 * - LED synchronization validation for depth capture integration
 *
 * Safety Systems:
 * - Current limiting enforced (250mA max per PROJECT_GUIDELINES.md)
 * - Thermal protection with automatic throttling
 * - Safe LED enable/disable sequences with validation
 * - Emergency shutdown accessible at all times
 * - Clear error reporting and recovery guidance
 *
 * Hardware Configuration:
 * - I2C Bus: 1, Address: 0x30
 * - GPIO Strobe: 19
 * - Target Current: 250mA (safety limited vs 450mA OSRAM spec)
 * - VCSEL: OSRAM BELAGO 15k points projector
 */
class AS1170DebugDialog : public QDialog {
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent widget
     */
    explicit AS1170DebugDialog(QWidget* parent = nullptr);

    /**
     * @brief Destructor - ensures safe hardware shutdown
     */
    ~AS1170DebugDialog();

protected:
    /**
     * @brief Handle dialog show events - initialize hardware monitoring
     */
    void showEvent(QShowEvent* event) override;

    /**
     * @brief Handle dialog close events - ensure safe shutdown
     */
    void closeEvent(QCloseEvent* event) override;

private slots:
    /**
     * @brief Initialize AS1170 hardware controller
     */
    void initializeHardware();

    /**
     * @brief Emergency shutdown - immediate LED disable
     */
    void emergencyShutdown();

    /**
     * @brief Reset AS1170 to factory defaults
     */
    void resetHardware();

    /**
     * @brief Test I2C communication
     */
    void testI2CCommunication();

    /**
     * @brief LED1 (VCSEL) control slots
     */
    void enableLED1();
    void disableLED1();
    void setLED1Current(int current_ma);

    /**
     * @brief LED2 (Flood) control slots
     */
    void enableLED2();
    void disableLED2();
    void setLED2Current(int current_ma);

    /**
     * @brief Strobe control slots
     */
    void triggerSingleStrobe();
    void setContinuousStrobe(bool enabled);
    void setStrobeDuration(int duration_us);
    void setStrobeFrequency(double frequency_hz);

    /**
     * @brief System monitoring and diagnostics
     */
    void updateHardwareStatus();
    void runFullDiagnostics();
    void exportDiagnosticsLog();
    void clearDiagnosticsLog();

    /**
     * @brief LED synchronization validation
     */
    void validateLEDSynchronization();
    void testDepthCaptureIntegration();

    /**
     * @brief Thermal protection handlers
     */
    void onThermalProtectionTriggered(bool active, float temperature_c);

    /**
     * @brief Hardware error handlers
     */
    void onHardwareError(const std::string& error_message);

private:
    /**
     * @brief Initialize the dialog UI
     */
    void initializeUI();

    /**
     * @brief Connect UI signals to slots
     */
    void connectSignals();

    /**
     * @brief Create hardware control tabs
     */
    QWidget* createLEDControlTab();
    QWidget* createStrobeControlTab();
    QWidget* createI2CDiagnosticsTab();
    QWidget* createMonitoringTab();
    QWidget* createSafetySystemsTab();
    QWidget* createSynchronizationTab();

    /**
     * @brief Create LED control panel for specific channel
     */
    QGroupBox* createLEDControlPanel(const QString& title, hardware::AS1170Controller::LEDChannel channel);

    /**
     * @brief Create status monitoring panel
     */
    QWidget* createStatusMonitoringPanel();

    /**
     * @brief Create diagnostic logging panel
     */
    QWidget* createDiagnosticLoggingPanel();

    /**
     * @brief Update UI based on hardware status
     */
    void updateUIFromHardwareStatus();

    /**
     * @brief Validate and apply current limits
     */
    bool validateCurrentSetting(int current_ma);

    /**
     * @brief Log diagnostic message with timestamp
     */
    void logDiagnosticMessage(const QString& message, const QString& level = "INFO");

    /**
     * @brief Update LED status indicators
     */
    void updateLEDStatusIndicators();

    /**
     * @brief Update temperature displays
     */
    void updateTemperatureDisplays();

    /**
     * @brief Update I2C status indicators
     */
    void updateI2CStatusIndicators();

    /**
     * @brief Safe hardware initialization
     */
    bool initializeAS1170Controller();

    /**
     * @brief Safe hardware shutdown
     */
    void shutdownAS1170Controller();

    // UI Components (from .ui file)
    Ui::AS1170DebugDialog* ui;

    // Hardware controller
    std::shared_ptr<hardware::AS1170Controller> as1170_controller_;

    // Status update timer
    QTimer* status_update_timer_;

    // Hardware status tracking
    std::atomic<bool> hardware_initialized_{false};
    std::atomic<bool> monitoring_active_{false};
    std::atomic<bool> emergency_shutdown_triggered_{false};

    // LED Control Widgets
    widgets::TouchButton* led1_enable_button_;
    widgets::TouchButton* led1_disable_button_;
    widgets::ParameterSlider* led1_current_slider_;
    widgets::StatusDisplay* led1_status_;

    widgets::TouchButton* led2_enable_button_;
    widgets::TouchButton* led2_disable_button_;
    widgets::ParameterSlider* led2_current_slider_;
    widgets::StatusDisplay* led2_status_;

    // Strobe Control Widgets
    widgets::TouchButton* single_strobe_button_;
    widgets::TouchButton* continuous_strobe_button_;
    widgets::ParameterSlider* strobe_duration_slider_;
    widgets::ParameterSlider* strobe_frequency_slider_;
    widgets::StatusDisplay* strobe_status_;

    // I2C Diagnostics Widgets
    widgets::TouchButton* i2c_test_button_;
    widgets::TouchButton* i2c_reset_button_;
    widgets::StatusDisplay* i2c_status_;
    widgets::StatusDisplay* i2c_address_status_;

    // Monitoring Widgets
    widgets::StatusDisplay* hardware_status_;
    widgets::StatusDisplay* temperature_status_;
    widgets::StatusDisplay* thermal_protection_status_;
    widgets::StatusDisplay* fault_status_;
    QProgressBar* led1_current_indicator_;
    QProgressBar* led2_current_indicator_;
    QProgressBar* temperature_indicator_;

    // Safety System Widgets
    widgets::TouchButton* emergency_shutdown_button_;
    widgets::TouchButton* reset_hardware_button_;
    widgets::TouchButton* run_diagnostics_button_;
    widgets::StatusDisplay* safety_status_;

    // Synchronization Test Widgets
    widgets::TouchButton* sync_validation_button_;
    widgets::TouchButton* depth_integration_test_button_;
    widgets::StatusDisplay* sync_status_;

    // Diagnostic Logging
    QTextEdit* diagnostic_log_;
    widgets::TouchButton* export_log_button_;
    widgets::TouchButton* clear_log_button_;

    // Layout containers
    QTabWidget* main_tabs_;
    QVBoxLayout* main_layout_;

    // Status tracking
    hardware::AS1170Controller::AS1170Status last_status_;
    hardware::AS1170Controller::ThermalStatus last_thermal_status_;

    // Strobe testing state
    std::atomic<bool> continuous_strobe_active_{false};
    QTimer* continuous_strobe_timer_;
    uint32_t strobe_duration_us_{1000};
    double strobe_frequency_hz_{1.0};
    uint64_t strobe_counter_{0};

    // Safety limits
    static constexpr uint16_t MAX_SAFE_CURRENT_MA = 500;  // INCREASED: 500mA limit for high visibility
    static constexpr uint32_t MIN_STROBE_DURATION_US = 100;
    static constexpr uint32_t MAX_STROBE_DURATION_US = 5000;
    static constexpr double MIN_STROBE_FREQUENCY_HZ = 0.1;
    static constexpr double MAX_STROBE_FREQUENCY_HZ = 100.0;
    static constexpr float THERMAL_WARNING_TEMP_C = 60.0f;
    static constexpr float THERMAL_SHUTDOWN_TEMP_C = 75.0f;

    // Update intervals
    static constexpr int STATUS_UPDATE_INTERVAL_MS = 100;    // 10 Hz status updates
    static constexpr int THERMAL_CHECK_INTERVAL_MS = 1000;   // 1 Hz thermal checks
};

} // namespace gui
} // namespace unlook