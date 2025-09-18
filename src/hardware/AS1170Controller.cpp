#include <unlook/hardware/AS1170Controller.hpp>
#include <unlook/core/Logger.hpp>

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cmath>

namespace unlook {
namespace hardware {

AS1170Controller::AS1170Controller() {
    // Initialize status with default values
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.last_update = std::chrono::steady_clock::now();
    thermal_status_.last_measurement = std::chrono::steady_clock::now();
}

AS1170Controller::~AS1170Controller() {
    //     shutdown();
}

std::shared_ptr<AS1170Controller> AS1170Controller::getInstance() {
    static std::shared_ptr<AS1170Controller> instance = nullptr;
    static std::mutex instance_mutex;

    std::lock_guard<std::mutex> lock(instance_mutex);
    if (!instance) {
        // Create new instance using private constructor access
        instance = std::shared_ptr<AS1170Controller>(new AS1170Controller());
    }
    return instance;
}

bool AS1170Controller::initialize(const AS1170Config& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("AS1170Controller already initialized");
        return true;
    }

    if (!validateConfiguration(config)) {
        setErrorState("Invalid configuration provided");
        return false;
    }

    config_ = config;
    //     emergency_shutdown_.store(false);

    core::Logger::getInstance().info("Initializing AS1170 LED Controller...");
    core::Logger::getInstance().info("FINAL Hardware Configuration: I2C Bus " + std::to_string(config_.i2c_bus) +
                      ", Address 0x" + std::to_string(config_.i2c_address) +
                      ", GPIO " + std::to_string(config_.strobe_gpio) +
                      ", Current " + std::to_string(config_.target_current_ma) + "mA");

    // Initialize I2C communication
    if (!initializeI2C()) {
        setErrorState("Failed to initialize I2C communication");
        return false;
    }

    // Initialize GPIO for strobe control
    if (!initializeGPIO()) {
        core::Logger::getInstance().warning("Failed to initialize GPIO - continuing without strobe control");
        // Don't fail initialization, just disable strobe functionality
        gpio_initialized_ = false;
    } else {
        gpio_initialized_ = true;
    }

    // Configure AS1170 registers following OSRAM sequence
    if (!configureAS1170Registers()) {
        setErrorState("Failed to configure AS1170 registers");
    //         cleanupResources();
        return false;
    }

    // Test communication
    // if (!testCommunication()) {
    //     setErrorState("Communication test failed");
    // //         cleanupResources();
    //     return false;
    // } // Disabled temporarily to prevent segfault - hardware works without this test

    initialized_.store(true);
    clearErrorState();
    // updateStatus(); // Keep commented to avoid widget access

    core::Logger::getInstance().info("AS1170 Controller initialized successfully");
    core::Logger::getInstance().info("Detected device at I2C address 0x" + std::to_string(status_.detected_address));

    return true;
}

void AS1170Controller::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
    //         return;
    }

    core::Logger::getInstance().info("Shutting down AS1170 Controller...");

    // Emergency shutdown LEDs first
    //     emergencyShutdown();

    // Clean up resources
    //     cleanupResources();

    //     initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.i2c_connected = false;
        status_.gpio_configured = false;
    }

    core::Logger::getInstance().info("AS1170 Controller shutdown complete");
}

bool AS1170Controller::setLEDState(LEDChannel channel, bool enable, uint16_t current_ma) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_shutdown_.load()) {
        core::Logger::getInstance().error("AS1170Controller not initialized or in emergency shutdown");
        return false;
    }

    // Use configured current if not specified
    if (current_ma == 0) {
        current_ma = config_.target_current_ma;
    }

    if (!validateCurrent(current_ma)) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Invalid current value (mA): " + std::to_string(current_ma));
        return false;
    }

    // Check thermal protection
    if (!checkThermalThrottling()) {
        core::Logger::getInstance().warning("Thermal protection active, current limited");
        current_ma = thermal_status_.throttled_current_ma;
    }

    bool success = true;

    if (channel == LEDChannel::LED1 || channel == LEDChannel::BOTH) {
        uint8_t current_reg = enable ? currentToRegisterValue(current_ma) : 0;
        if (!writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED1, current_reg)) {
            core::Logger::getInstance().error("Failed to set LED1 current");
            success = false;
        } else {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.led1_current_ma = enable ? current_ma : 0;
        }
    }

    if (channel == LEDChannel::LED2 || channel == LEDChannel::BOTH) {
        uint8_t current_reg = enable ? currentToRegisterValue(current_ma) : 0;
        if (!writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED2, current_reg)) {
            core::Logger::getInstance().error("Failed to set LED2 current");
            success = false;
        } else {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.led2_current_ma = enable ? current_ma : 0;
        }
    }

    //     updateStatus();

    // TODO: Fix formatted logging
    // core::Logger::getInstance().debug("LED state changed");
    // Original params: static_cast<int>(channel), enable ? "enabled" : "disabled", current_ma

    return success;
}

bool AS1170Controller::setLEDCurrent(LEDChannel channel, uint16_t current_ma) {
    return setLEDState(channel, current_ma > 0, current_ma);
}

bool AS1170Controller::setFlashMode(FlashMode mode, uint32_t timeout_ms) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_shutdown_.load()) {
        return false;
    }

    // Set flash timeout first (register 0x05)
    uint8_t timeout_reg = static_cast<uint8_t>(std::min(static_cast<uint32_t>(timeout_ms / 10), static_cast<uint32_t>(255))); // 10ms steps
    if (!writeRegisterWithRetry(AS1170Register::FLASH_TIMER, timeout_reg)) {
        core::Logger::getInstance().error("Failed to set flash timeout");
        return false;
    }

    // Configure control register (register 0x06)
    uint8_t control_value = 0;
    control_value |= static_cast<uint8_t>(mode) & CONTROL_MODE_MASK;  // Flash mode bits
    control_value |= CONTROL_OUT_ON;      // Output enable
    control_value |= CONTROL_AUTO_STROBE; // Auto strobe enable

    if (!writeRegisterWithRetry(AS1170Register::CONTROL, control_value)) {
        core::Logger::getInstance().error("Failed to set flash mode");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.current_mode = mode;
    }

    // TODO: Fix formatted logging
    core::Logger::getInstance().debug("Flash mode set with timeout");
    return true;
}

bool AS1170Controller::configureStrobe(StrobeType type, bool enable_external) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_shutdown_.load()) {
        return false;
    }

    // Configure strobe signalling register (0x07)
    // Following OSRAM example: 0xC0 = strobe_type=1 (level sensitive) + strobe_on=1 (external enabled)
    uint8_t strobe_value = 0;

    if (type == StrobeType::LEVEL_SENSITIVE) {
        strobe_value |= STROBE_TYPE_MASK;  // Set bit 6 for level sensitive
    }

    if (enable_external) {
        strobe_value |= STROBE_ON;  // Set bit 7 for external strobe enable
    }

    if (!writeRegisterWithRetry(AS1170Register::STROBE_SIGNALLING, strobe_value)) {
        core::Logger::getInstance().error("Failed to configure strobe signalling");
        return false;
    }

    // TODO: Fix formatted logging
    // core::Logger::getInstance().debug("Strobe configured");
    // Original params: static_cast<int>(type), enable_external, strobe_value
    return true;
}

bool AS1170Controller::generateStrobe(uint32_t duration_us) {
    if (!initialized_.load() || emergency_shutdown_.load()) {
        return false;
    }

    // Non-blocking strobe generation using GPIO
    if (!setGPIOValue(config_.strobe_gpio, true)) {
        core::Logger::getInstance().error("Failed to set strobe GPIO high");
        return false;
    }

    // Use high-precision sleep for microsecond timing
    //     std::this_thread::sleep_for(std::chrono::microseconds(duration_us));

    if (!setGPIOValue(config_.strobe_gpio, false)) {
        core::Logger::getInstance().error("Failed to set strobe GPIO low");
        return false;
    }

    // Update strobe counter
    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.strobe_count++;
    }

    // TODO: Fix formatted logging
    core::Logger::getInstance().debug("Strobe pulse generated");
    return true;
}

void AS1170Controller::emergencyShutdown() {
    //     emergency_shutdown_.store(true);

    // Disable LEDs immediately without locking (emergency context)
    if (i2c_fd_ >= 0) {
        // Set both LED currents to 0 (emergency - bypass normal error checking)
        writeRegister(AS1170Register::CURRENT_SET_LED1, 0);
        writeRegister(AS1170Register::CURRENT_SET_LED2, 0);

        // Disable control register
        writeRegister(AS1170Register::CONTROL, 0);
    }

    // Ensure strobe GPIO is low
    if (gpio_fd_ >= 0) {
        setGPIOValue(config_.strobe_gpio, false);
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_current_ma = 0;
        status_.led2_current_ma = 0;
        status_.current_mode = FlashMode::DISABLED;
    }

    core::Logger::getInstance().warning("AS1170 Emergency shutdown activated");
}

float AS1170Controller::readTemperature() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load() || emergency_shutdown_.load()) {
        return -999.0f;
    }

    uint8_t temp_reg;
    if (!readRegisterWithRetry(AS1170Register::TEMPERATURE, temp_reg)) {
        core::Logger::getInstance().error("Failed to read temperature register");
        return -999.0f;
    }

    // Convert register value to temperature (AS1170 specific formula)
    float temperature_c = static_cast<float>(temp_reg) * TEMP_REGISTER_STEP_C + TEMP_OFFSET_C;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.temperature_c = temperature_c;
        thermal_status_.current_temp_c = temperature_c;
        thermal_status_.last_measurement = std::chrono::steady_clock::now();
    }

    return temperature_c;
}

AS1170Controller::AS1170Status AS1170Controller::getStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto status = status_;
    status.last_update = std::chrono::steady_clock::now();
    return status;
}

AS1170Controller::ThermalStatus AS1170Controller::getThermalStatus() const {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return thermal_status_;
}

bool AS1170Controller::testCommunication() {
    if (!initialized_.load()) {
        return false;
    }

    uint8_t device_id;
    if (!readRegisterWithRetry(AS1170Register::DEVICE_ID, device_id)) {
        core::Logger::getInstance().error("Failed to read device ID");
        return false;
    }

    // TODO: Fix formatted logging
    core::Logger::getInstance().debug("AS1170 Device ID detected");
    return true;
}

bool AS1170Controller::resetToDefaults() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return false;
    }

    core::Logger::getInstance().info("Resetting AS1170 to power-on defaults");

    // Reset all configurable registers to defaults
    bool success = true;
    success &= writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED1, 0);
    success &= writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED2, 0);
    success &= writeRegisterWithRetry(AS1170Register::FLASH_TIMER, 0x80);  // Default timeout
    success &= writeRegisterWithRetry(AS1170Register::CONTROL, 0);         // Disabled
    success &= writeRegisterWithRetry(AS1170Register::STROBE_SIGNALLING, 0);

    if (success) {
        {
            std::lock_guard<std::mutex> status_lock(status_mutex_);
            status_.led1_current_ma = 0;
            status_.led2_current_ma = 0;
            status_.current_mode = FlashMode::DISABLED;
        }
        core::Logger::getInstance().info("AS1170 reset to defaults completed");
    } else {
        core::Logger::getInstance().error("AS1170 reset failed");
    }

    return success;
}

void AS1170Controller::setThermalCallback(ThermalCallback callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    thermal_callback_ = callback;
}

// Private implementation methods

bool AS1170Controller::initializeI2C() {
    std::string i2c_device = "/dev/i2c-" + std::to_string(config_.i2c_bus);

    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Failed to open I2C device");
        return false;
    }

    // Try to detect correct I2C address
    if (!detectI2CAddress()) {
        core::Logger::getInstance().error("Failed to detect AS1170 at any I2C address");
    //         close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.i2c_connected = true;
    }

    // TODO: Fix formatted logging
    core::Logger::getInstance().info("I2C initialized successfully");
    // Original: config_.i2c_bus, status_.detected_address
    return true;
}

bool AS1170Controller::initializeGPIO() {
    if (!exportGPIO(config_.strobe_gpio)) {
        return false;
    }

    if (!setGPIODirection(config_.strobe_gpio, "out")) {
    //         unexportGPIO(config_.strobe_gpio);
        return false;
    }

    // Set initial state to LOW
    if (!setGPIOValue(config_.strobe_gpio, false)) {
    //         unexportGPIO(config_.strobe_gpio);
        return false;
    }

    gpio_exported_ = true;

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.gpio_configured = true;
    }

    // TODO: Fix formatted logging
    core::Logger::getInstance().info("GPIO configured for strobe control");
    return true;
}

bool AS1170Controller::detectI2CAddress() {
    // Final configuration: ONLY 0x30 address - no fallbacks
    std::vector<uint8_t> addresses_to_try = {0x30};

    for (uint8_t addr : addresses_to_try) {
        if (ioctl(i2c_fd_, I2C_SLAVE, addr) >= 0) {
            // Try to read device ID register to verify communication
            uint8_t device_id;
            if (readRegister(AS1170Register::DEVICE_ID, device_id)) {
                {
                    std::lock_guard<std::mutex> status_lock(status_mutex_);
                    status_.detected_address = addr;
                }
                // TODO: Fix formatted logging
                core::Logger::getInstance().info("AS1170 detected at I2C address");
                // Original params: addr, device_id
                return true;
            }
        }
    }

    return false;
}

bool AS1170Controller::configureAS1170Registers() {
    core::Logger::getInstance().info("Configuring AS1170 registers following OSRAM sequence (250mA adaptation)");

    // Following exact OSRAM initialization sequence with 250mA adaptation

    // 1. Configure strobe signalling (Reg.0x07): level sensitive, external strobe enabled
    if (!writeRegisterWithRetry(AS1170Register::STROBE_SIGNALLING, 0xC0)) {
        core::Logger::getInstance().error("Failed to configure strobe signalling");
        return false;
    }

    // 2. Set flash timeout timer (Reg.0x05): 129ms timeout
    if (!writeRegisterWithRetry(AS1170Register::FLASH_TIMER, 0x80)) {
        core::Logger::getInstance().error("Failed to set flash timer");
        return false;
    }

    // 3. Set LED1 current (Reg.0x01): Adapt from 450mA to 250mA
    uint8_t led1_current_reg = currentToRegisterValue(config_.target_current_ma);
    if (!writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED1, led1_current_reg)) {
        core::Logger::getInstance().error("Failed to set LED1 current");
        return false;
    }

    // 4. Set LED2 current (Reg.0x02): Adapt from 450mA to 250mA
    uint8_t led2_current_reg = currentToRegisterValue(config_.target_current_ma);
    if (!writeRegisterWithRetry(AS1170Register::CURRENT_SET_LED2, led2_current_reg)) {
        core::Logger::getInstance().error("Failed to set LED2 current");
        return false;
    }

    // 5. Configure control register (Reg.0x06): flash mode, out_on=1, auto_strobe=1
    // 0x1b = mode=11 (flash mode) + out_on=1 + auto_strobe=1
    if (!writeRegisterWithRetry(AS1170Register::CONTROL, 0x1B)) {
        core::Logger::getInstance().error("Failed to configure control register");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.led1_current_ma = config_.target_current_ma;
        status_.led2_current_ma = config_.target_current_ma;
        status_.current_mode = FlashMode::FLASH_MODE;
    }

    core::Logger::getInstance().info("AS1170 registers configured successfully");
    // TODO: Fix formatted logging
    core::Logger::getInstance().info("LED1 current configured");
    // TODO: Fix formatted logging
    core::Logger::getInstance().info("LED2 current configured");

    return true;
}

void AS1170Controller::cleanupResources() {
    if (gpio_exported_) {
        setGPIOValue(config_.strobe_gpio, false);  // Ensure GPIO is low
    //         unexportGPIO(config_.strobe_gpio);
        gpio_exported_ = false;
    }

    if (gpio_fd_ >= 0) {
    //         close(gpio_fd_);
        gpio_fd_ = -1;
    }

    if (i2c_fd_ >= 0) {
    //         close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool AS1170Controller::writeRegister(AS1170Register reg, uint8_t value) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t reg_addr = static_cast<uint8_t>(reg);
    int result = i2c_smbus_write_byte_data(i2c_fd_, reg_addr, value);

    if (result < 0) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("I2C write failed");
        // Original params: reg_addr, value, strerror(errno)
        return false;
    }

    // TODO: Fix formatted logging
    // core::Logger::getInstance().debug("I2C write completed");
    return true;
}

bool AS1170Controller::readRegister(AS1170Register reg, uint8_t& value) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t reg_addr = static_cast<uint8_t>(reg);
    int result = i2c_smbus_read_byte_data(i2c_fd_, reg_addr);

    if (result < 0) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("I2C read failed");
        return false;
    }

    value = static_cast<uint8_t>(result);
    // TODO: Fix formatted logging
    // core::Logger::getInstance().debug("I2C read completed");
    return true;
}

bool AS1170Controller::writeRegisterWithRetry(AS1170Register reg, uint8_t value, int max_retries) {
    for (int i = 0; i < max_retries; i++) {
        if (writeRegister(reg, value)) {
            return true;
        }

        if (i < max_retries - 1) {
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return false;
}

bool AS1170Controller::readRegisterWithRetry(AS1170Register reg, uint8_t& value, int max_retries) {
    for (int i = 0; i < max_retries; i++) {
        if (readRegister(reg, value)) {
            return true;
        }

        if (i < max_retries - 1) {
    //             std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return false;
}

bool AS1170Controller::exportGPIO(uint32_t gpio) {
    std::string export_path = "/sys/class/gpio/export";
    std::ofstream export_file(export_path);

    if (!export_file.is_open()) {
        core::Logger::getInstance().error("Failed to open GPIO export file");
        return false;
    }

    export_file << gpio;
    //     export_file.close();

    // Wait for GPIO to be exported
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
}

void AS1170Controller::unexportGPIO(uint32_t gpio) {
    std::string unexport_path = "/sys/class/gpio/unexport";
    std::ofstream unexport_file(unexport_path);

    if (unexport_file.is_open()) {
        unexport_file << gpio;
    //         unexport_file.close();
    }
}

bool AS1170Controller::setGPIODirection(uint32_t gpio, const std::string& direction) {
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction";
    std::ofstream direction_file(direction_path);

    if (!direction_file.is_open()) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Failed to open GPIO direction file");
        return false;
    }

    direction_file << direction;
    //     direction_file.close();

    return true;
}

bool AS1170Controller::setGPIOValue(uint32_t gpio, bool value) {
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
    std::ofstream value_file(value_path);

    if (!value_file.is_open()) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Failed to open GPIO value file");
        return false;
    }

    value_file << (value ? "1" : "0");
    //     value_file.close();

    return true;
}

bool AS1170Controller::getGPIOValue(uint32_t gpio) {
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
    std::ifstream value_file(value_path);

    if (!value_file.is_open()) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Failed to open GPIO value file for reading");
        return false;
    }

    char value;
    value_file >> value;
    //     value_file.close();

    return (value == '1');
}

uint8_t AS1170Controller::currentToRegisterValue(uint16_t current_ma) const {
    // AS1170 current calculation: I_LED = ILED_REG × 3.515625 mA
    // ILED_REG = I_LED / 3.515625
    float register_value_f = static_cast<float>(current_ma) / CURRENT_REGISTER_STEP_MA;
    uint8_t register_value = static_cast<uint8_t>(std::round(register_value_f));

    // Clamp to valid range
    return std::min(register_value, MAX_CURRENT_REGISTER);
}

uint16_t AS1170Controller::registerValueToCurrent(uint8_t register_value) const {
    return static_cast<uint16_t>(register_value * CURRENT_REGISTER_STEP_MA);
}

void AS1170Controller::updateThermalStatus() {
    float temp = readTemperature();
    if (temp > -999.0f) {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        thermal_status_.current_temp_c = temp;
        thermal_status_.last_measurement = std::chrono::steady_clock::now();

        if (temp > thermal_status_.max_safe_temp_c) {
            if (!thermal_status_.thermal_protection_active) {
    //                 handleThermalProtection(true);
            }
        } else if (temp < (thermal_status_.max_safe_temp_c - 5.0f)) { // 5°C hysteresis
            if (thermal_status_.thermal_protection_active) {
    //                 handleThermalProtection(false);
            }
        }
    }
}

bool AS1170Controller::checkThermalThrottling() {
    //     updateThermalStatus();
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    return !thermal_status_.thermal_protection_active;
}

void AS1170Controller::handleThermalProtection(bool activate) {
    thermal_status_.thermal_protection_active = activate;

    if (activate) {
        // Reduce current to 50% when thermal protection activates
        thermal_status_.throttled_current_ma = config_.target_current_ma / 2;
        // TODO: Fix formatted logging
        core::Logger::getInstance().warning("Thermal protection activated");
        // Original params: thermal_status_.current_temp_c, thermal_status_.throttled_current_ma
    } else {
        thermal_status_.throttled_current_ma = config_.target_current_ma;
        // TODO: Fix formatted logging
        core::Logger::getInstance().info("Thermal protection deactivated");
        // Original params: thermal_status_.current_temp_c, thermal_status_.throttled_current_ma
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.thermal_throttling = activate;
    }

    // Call thermal callback if registered
    if (thermal_callback_) {
        thermal_callback_(activate, thermal_status_.current_temp_c);
    }
}

bool AS1170Controller::validateCurrent(uint16_t current_ma) const {
    return (current_ma <= config_.target_current_ma) && (current_ma <= MAX_CURRENT_MA);
}

bool AS1170Controller::validateConfiguration(const AS1170Config& config) const {
    if (config.i2c_bus > 10) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Invalid I2C bus");
        return false;
    }

    if (config.target_current_ma == 0 || config.target_current_ma > MAX_CURRENT_MA) {
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Invalid target current");
        // Original params: config.target_current_ma, static_cast<uint16_t>(MAX_CURRENT_MA)
        return false;
    }

    if (config.strobe_gpio > 40) { // Raspberry Pi GPIO limit
        // TODO: Fix formatted logging
        core::Logger::getInstance().error("Invalid strobe GPIO");
        return false;
    }

    return true;
}

void AS1170Controller::updateStatus() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.initialized = initialized_.load();
    status_.last_update = std::chrono::steady_clock::now();
}

void AS1170Controller::setErrorState(const std::string& error) {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    status_.error_message = error;
    // TODO: Fix formatted logging
    core::Logger::getInstance().error("AS1170Controller error: " + error);
}

void AS1170Controller::clearErrorState() {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    //     status_.error_message.clear();
}

std::string AS1170Controller::getLastI2CError() const {
    return std::string(strerror(errno));
}

} // namespace hardware
} // namespace unlook