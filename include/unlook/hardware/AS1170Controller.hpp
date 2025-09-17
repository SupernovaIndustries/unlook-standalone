#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <functional>
#include <map>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace unlook {
namespace hardware {

/**
 * AS1170 LED Driver Controller for OSRAM BELAGO VCSEL Projector
 *
 * Provides low-level I2C communication and GPIO control for AS1170 LED driver.
 * Implements the exact OSRAM initialization sequence adapted for 250mA operation.
 *
 * Hardware Configuration (FINAL):
 * - I2C Bus: 1 (confirmed working, final configuration)
 * - I2C Address: 0x30 (final production address)
 * - GPIO Strobe: 19 (final strobe control pin for CM5)
 * - Target Current: 250mA (industrial safety limit vs 450mA OSRAM example)
 * - VCSEL: OSRAM BELAGO 15k points projector
 *
 * Safety Features:
 * - Current limiting with thermal protection
 * - Emergency shutdown capability (<5ms response)
 * - I2C communication validation with retry logic
 * - Temperature monitoring and automatic throttling
 *
 * Thread Safety: All public methods are thread-safe with mutex protection
 */
class AS1170Controller {
public:
    enum class LEDChannel {
        LED1 = 1,  // VCSEL projector (primary)
        LED2 = 2,  // Flood illuminator (auxiliary)
        BOTH = 3   // Both channels simultaneously
    };

    enum class FlashMode {
        DISABLED = 0,
        FLASH_MODE = 3,      // Flash mode (strobe controlled)
        TORCH_MODE = 1,      // Continuous mode (always on)
        INDICATOR_MODE = 2   // Indicator mode (low power)
    };

    enum class StrobeType {
        EDGE_SENSITIVE = 0,  // Rising edge triggered
        LEVEL_SENSITIVE = 1  // Level sensitive (recommended for sync)
    };

    struct AS1170Config {
        uint8_t i2c_bus;
        uint8_t i2c_address;
        uint32_t strobe_gpio;
        uint16_t target_current_ma;
        FlashMode flash_mode;
        StrobeType strobe_type;
        uint32_t flash_timeout_ms;
        bool enable_thermal_protection;
        float max_temperature_c;

        // Default constructor with FINAL CONFIG values
        AS1170Config() :
            i2c_bus(1),                           // I2C bus number (FINAL CONFIG: bus 1)
            i2c_address(0x30),                    // I2C address (FINAL CONFIG: 0x30)
            strobe_gpio(19),                      // GPIO pin for strobe control (FINAL CONFIG: GPIO 19)
            target_current_ma(250),               // Target current in mA (safety limited)
            flash_mode(FlashMode::FLASH_MODE),
            strobe_type(StrobeType::LEVEL_SENSITIVE),
            flash_timeout_ms(129),                // Flash timeout in ms
            enable_thermal_protection(true),
            max_temperature_c(70.0f)              // Maximum operating temperature
        {}
    };

    struct AS1170Status {
        bool initialized = false;
        bool i2c_connected = false;
        bool gpio_configured = false;
        uint8_t detected_address = 0;
        float temperature_c = 0.0f;
        uint16_t led1_current_ma = 0;
        uint16_t led2_current_ma = 0;
        FlashMode current_mode = FlashMode::DISABLED;
        bool thermal_throttling = false;
        uint64_t strobe_count = 0;
        std::string error_message;
        std::chrono::steady_clock::time_point last_update;
    };

    struct ThermalStatus {
        float current_temp_c = 0.0f;
        float max_safe_temp_c = 70.0f;
        bool thermal_protection_active = false;
        uint16_t throttled_current_ma = 0;
        std::chrono::steady_clock::time_point last_measurement;
    };

    AS1170Controller();
    ~AS1170Controller();

    /**
     * Initialize AS1170 controller with hardware detection and setup
     * Follows exact OSRAM initialization sequence with 250mA adaptation
     * @param config Hardware configuration parameters
     * @return true if initialization successful
     */
    bool initialize(const AS1170Config& config = AS1170Config{});

    /**
     * Shutdown and cleanup hardware resources
     */
    void shutdown();

    /**
     * Enable/disable LED channels with specified current
     * @param channel LED channel to control
     * @param enable Enable or disable the channel
     * @param current_ma Current in milliamps (0-450mA, limited to 250mA by config)
     * @return true if successful
     */
    bool setLEDState(LEDChannel channel, bool enable, uint16_t current_ma = 0);

    /**
     * Set LED current for specific channel
     * Automatically calculates proper register values for AS1170
     * @param channel LED channel
     * @param current_ma Target current in mA (safety limited to config max)
     * @return true if successful
     */
    bool setLEDCurrent(LEDChannel channel, uint16_t current_ma);

    /**
     * Configure flash mode operation
     * @param mode Flash mode (FLASH_MODE recommended for sync)
     * @param timeout_ms Flash timeout in milliseconds
     * @return true if successful
     */
    bool setFlashMode(FlashMode mode, uint32_t timeout_ms = 129);

    /**
     * Configure strobe signal characteristics
     * @param type Strobe type (LEVEL_SENSITIVE recommended)
     * @param enable_external Enable external strobe control
     * @return true if successful
     */
    bool configureStrobe(StrobeType type, bool enable_external = true);

    /**
     * Generate strobe pulse for synchronized capture
     * Non-blocking GPIO control with microsecond precision
     * @param duration_us Strobe duration in microseconds (default: 1000μs)
     * @return true if successful
     */
    bool generateStrobe(uint32_t duration_us = 1000);

    /**
     * Emergency shutdown - immediate LED disable (<5ms response)
     * Thread-safe and callable from any context including signal handlers
     */
    void emergencyShutdown();

    /**
     * Read current temperature from AS1170 internal sensor
     * @return Temperature in Celsius, or -999.0f on error
     */
    float readTemperature();

    /**
     * Get comprehensive controller status
     */
    AS1170Status getStatus() const;

    /**
     * Get thermal protection status
     */
    ThermalStatus getThermalStatus() const;

    /**
     * Check if hardware is properly initialized and functional
     */
    bool isInitialized() const { return initialized_.load(); }

    /**
     * Test I2C communication with device
     * @return true if communication successful
     */
    bool testCommunication();

    /**
     * Reset AS1170 to power-on defaults
     * @return true if successful
     */
    bool resetToDefaults();

    /**
     * Register callback for thermal protection events
     * Called when thermal throttling is activated/deactivated
     */
    using ThermalCallback = std::function<void(bool thermal_active, float temperature_c)>;
    void setThermalCallback(ThermalCallback callback);

private:
    // AS1170 Register Definitions (from datasheet)
    enum class AS1170Register : uint8_t {
        CURRENT_SET_LED1 = 0x01,    // LED1 current control
        CURRENT_SET_LED2 = 0x02,    // LED2 current control
        PRIVACY_CURRENT = 0x03,     // Privacy current limit
        PRIVACY_PWM = 0x04,         // Privacy PWM control
        FLASH_TIMER = 0x05,         // Flash timeout timer
        CONTROL = 0x06,             // Main control register
        STROBE_SIGNALLING = 0x07,   // Strobe signal configuration
        FAULT = 0x08,              // Fault status register
        PWM1 = 0x09,               // PWM1 control
        PWM2 = 0x0A,               // PWM2 control
        MINIMUM_LED_CURRENT = 0x0E, // Minimum current setting
        ACTUAL_LED1_CURRENT = 0x0F, // LED1 actual current readback
        ACTUAL_LED2_CURRENT = 0x10, // LED2 actual current readback
        TEMPERATURE = 0x11,         // Temperature sensor
        ADC_VALUE = 0x12,          // ADC readback
        DEVICE_ID = 0x13           // Device identification
    };

    // Control register bit masks
    static constexpr uint8_t CONTROL_MODE_MASK = 0x03;
    static constexpr uint8_t CONTROL_OUT_ON = 0x04;
    static constexpr uint8_t CONTROL_AUTO_STROBE = 0x08;
    static constexpr uint8_t CONTROL_STROBE_RAMP = 0x10;

    // Strobe signalling bit masks
    static constexpr uint8_t STROBE_TYPE_MASK = 0x40;
    static constexpr uint8_t STROBE_ON = 0x80;

    // Hardware configuration
    AS1170Config config_;
    mutable std::mutex mutex_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> emergency_shutdown_{false};

    // I2C and GPIO resources
    int i2c_fd_ = -1;
    int gpio_fd_ = -1;
    bool gpio_exported_ = false;
    bool gpio_initialized_ = false;

    // Status tracking
    mutable std::mutex status_mutex_;
    AS1170Status status_;
    ThermalStatus thermal_status_;
    ThermalCallback thermal_callback_;

    // Current calculation constants (AS1170 specific)
    static constexpr float CURRENT_REGISTER_STEP_MA = 3.515625f; // mA per register step
    static constexpr uint8_t MAX_CURRENT_REGISTER = 0x7F;       // Maximum register value
    static constexpr float MAX_CURRENT_MA = MAX_CURRENT_REGISTER * CURRENT_REGISTER_STEP_MA;

    // Temperature conversion constants
    static constexpr float TEMP_REGISTER_STEP_C = 1.0f;        // °C per register step
    static constexpr float TEMP_OFFSET_C = -40.0f;             // Temperature offset

    // Hardware initialization methods
    bool initializeI2C();
    bool initializeGPIO();
    bool detectI2CAddress();
    bool configureAS1170Registers();
    void cleanupResources();

    // I2C communication methods
    bool writeRegister(AS1170Register reg, uint8_t value);
    bool readRegister(AS1170Register reg, uint8_t& value);
    bool writeRegisterWithRetry(AS1170Register reg, uint8_t value, int max_retries = 3);
    bool readRegisterWithRetry(AS1170Register reg, uint8_t& value, int max_retries = 3);

    // GPIO control methods
    bool exportGPIO(uint32_t gpio);
    void unexportGPIO(uint32_t gpio);
    bool setGPIODirection(uint32_t gpio, const std::string& direction);
    bool setGPIOValue(uint32_t gpio, bool value);
    bool getGPIOValue(uint32_t gpio);

    // Current conversion methods
    uint8_t currentToRegisterValue(uint16_t current_ma) const;
    uint16_t registerValueToCurrent(uint8_t register_value) const;

    // Temperature monitoring
    void updateThermalStatus();
    bool checkThermalThrottling();
    void handleThermalProtection(bool activate);

    // Safety and validation
    bool validateCurrent(uint16_t current_ma) const;
    bool validateConfiguration(const AS1170Config& config) const;
    void updateStatus();

    // Error handling
    void setErrorState(const std::string& error);
    void clearErrorState();
    std::string getLastI2CError() const;
};

} // namespace hardware
} // namespace unlook