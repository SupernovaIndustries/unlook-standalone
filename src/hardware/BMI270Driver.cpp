#include <unlook/hardware/BMI270Driver.hpp>
#include <unlook/core/Logger.hpp>

#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <cmath>

namespace unlook {
namespace hardware {

BMI270Driver::BMI270Driver() :
    initialized_(false),
    mock_mode_(false),
    i2c_fd_(-1),
    gyro_scale_(1.0f),
    accel_scale_(1.0f),
    sample_count_(0) {

    status_.last_update = std::chrono::steady_clock::now();
    mock_start_time_ = std::chrono::steady_clock::now();
}

BMI270Driver::~BMI270Driver() {
    shutdown();
}

std::shared_ptr<BMI270Driver> BMI270Driver::getInstance() {
    static std::shared_ptr<BMI270Driver> instance = nullptr;
    static std::mutex instance_mutex;

    std::lock_guard<std::mutex> lock(instance_mutex);
    if (!instance) {
        instance = std::shared_ptr<BMI270Driver>(new BMI270Driver());
    }
    return instance;
}

bool BMI270Driver::initialize(const BMI270Config& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_.load()) {
        core::Logger::getInstance().warning("BMI270Driver already initialized");
        return true;
    }

    if (!validateConfiguration(config)) {
        setErrorState("Invalid configuration provided");
        return false;
    }

    config_ = config;
    mock_mode_.store(config.enable_mock_mode);

    core::Logger::getInstance().info("Initializing BMI270 IMU Driver...");
    core::Logger::getInstance().info("Configuration: I2C Bus " + std::to_string(config_.i2c_bus) +
                      ", Address 0x" + std::to_string(config_.i2c_address) +
                      ", Sample Rate " + std::to_string(config_.sample_rate_hz) + "Hz");

    if (mock_mode_.load()) {
        core::Logger::getInstance().warning("BMI270 running in MOCK MODE - no hardware access");
        initialized_.store(true);
        clearErrorState();
        return true;
    }

    // Initialize I2C communication
    if (!initializeI2C()) {
        setErrorState("Failed to initialize I2C communication");
        return false;
    }

    // Verify chip ID
    if (!verifyChipID()) {
        setErrorState("Failed to verify BMI270 chip ID");
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // Soft reset
    if (!softReset()) {
        core::Logger::getInstance().warning("Soft reset failed - continuing anyway");
    }

    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configure IMU
    if (!configureIMU()) {
        setErrorState("Failed to configure BMI270");
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    // Calculate conversion scales based on configuration
    // Gyro scale: ±range_dps / 32768 (int16_t max)
    gyro_scale_ = static_cast<float>(config_.gyro_range_dps) / 32768.0f;

    // Accel scale: ±range_g × 9.81 m/s² / 32768
    accel_scale_ = (static_cast<float>(config_.accel_range_g) * 9.81f) / 32768.0f;

    initialized_.store(true);
    clearErrorState();

    core::Logger::getInstance().info("BMI270 IMU Driver initialized successfully");
    core::Logger::getInstance().info("Gyro scale: " + std::to_string(gyro_scale_) +
                      " deg/s per LSB, Accel scale: " + std::to_string(accel_scale_) + " m/s² per LSB");

    return true;
}

void BMI270Driver::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!initialized_.load()) {
        return;
    }

    core::Logger::getInstance().info("Shutting down BMI270 IMU Driver...");

    if (!mock_mode_.load() && i2c_fd_ >= 0) {
        // Disable accelerometer and gyroscope
        writeRegister(REG_PWR_CTRL, 0x00);
        close(i2c_fd_);
        i2c_fd_ = -1;
    }

    initialized_.store(false);

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.initialized = false;
        status_.i2c_connected = false;
    }

    core::Logger::getInstance().info("BMI270 IMU Driver shutdown complete");
}

bool BMI270Driver::readIMUData(IMUData& data) {
    if (!initialized_.load()) {
        data.valid = false;
        return false;
    }

    if (mock_mode_.load()) {
        data = generateMockData();
        sample_count_++;
        return true;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // Read accelerometer data (6 bytes starting at 0x0C)
    uint8_t accel_buffer[6];
    if (!readRegisters(REG_DATA_ACC_X_LSB, accel_buffer, 6)) {
        data.valid = false;
        return false;
    }

    // Read gyroscope data (6 bytes starting at 0x12)
    uint8_t gyro_buffer[6];
    if (!readRegisters(REG_DATA_GYR_X_LSB, gyro_buffer, 6)) {
        data.valid = false;
        return false;
    }

    // Convert raw data to int16_t (little-endian)
    int16_t accel_x_raw = static_cast<int16_t>((accel_buffer[1] << 8) | accel_buffer[0]);
    int16_t accel_y_raw = static_cast<int16_t>((accel_buffer[3] << 8) | accel_buffer[2]);
    int16_t accel_z_raw = static_cast<int16_t>((accel_buffer[5] << 8) | accel_buffer[4]);

    int16_t gyro_x_raw = static_cast<int16_t>((gyro_buffer[1] << 8) | gyro_buffer[0]);
    int16_t gyro_y_raw = static_cast<int16_t>((gyro_buffer[3] << 8) | gyro_buffer[2]);
    int16_t gyro_z_raw = static_cast<int16_t>((gyro_buffer[5] << 8) | gyro_buffer[4]);

    // Convert to physical units
    data.accel_x = convertAccelRaw(accel_x_raw);
    data.accel_y = convertAccelRaw(accel_y_raw);
    data.accel_z = convertAccelRaw(accel_z_raw);

    data.gyro_x = convertGyroRaw(gyro_x_raw);
    data.gyro_y = convertGyroRaw(gyro_y_raw);
    data.gyro_z = convertGyroRaw(gyro_z_raw);

    data.timestamp_us = getMicroseconds();
    data.valid = true;

    sample_count_++;

    return true;
}

BMI270Driver::BMI270Status BMI270Driver::getStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return status_;
}

bool BMI270Driver::softReset() {
    core::Logger::getInstance().info("Performing BMI270 soft reset...");

    if (!writeRegister(REG_CMD, CMD_SOFT_RESET)) {
        return false;
    }

    // Wait for reset to complete (450 μs typical)
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    return true;
}

void BMI270Driver::setMockMode(bool enable) {
    mock_mode_.store(enable);
    if (enable) {
        core::Logger::getInstance().info("BMI270 mock mode enabled");
    } else {
        core::Logger::getInstance().info("BMI270 mock mode disabled");
    }
}

void BMI270Driver::setMockData(const IMUData& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    mock_data_ = data;
}

// Private methods

bool BMI270Driver::initializeI2C() {
    std::string i2c_device = "/dev/i2c-" + std::to_string(config_.i2c_bus);

    i2c_fd_ = open(i2c_device.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        core::Logger::getInstance().error("Failed to open I2C device: " + i2c_device);
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, config_.i2c_address) < 0) {
        core::Logger::getInstance().error("Failed to set I2C slave address 0x" +
                          std::to_string(config_.i2c_address));
        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.i2c_connected = true;
    }

    core::Logger::getInstance().info("I2C communication initialized successfully");
    return true;
}

bool BMI270Driver::configureIMU() {
    core::Logger::getInstance().info("Configuring BMI270 registers...");

    // Configure accelerometer
    uint8_t acc_conf = 0;
    switch (config_.sample_rate_hz) {
        case 25:   acc_conf = ODR_25HZ; break;
        case 50:   acc_conf = ODR_50HZ; break;
        case 100:  acc_conf = ODR_100HZ; break;
        case 200:  acc_conf = ODR_200HZ; break;
        case 400:  acc_conf = ODR_400HZ; break;
        case 800:  acc_conf = ODR_800HZ; break;
        case 1600: acc_conf = ODR_1600HZ; break;
        default:   acc_conf = ODR_100HZ; break;
    }

    if (!writeRegister(REG_ACC_CONF, acc_conf)) {
        core::Logger::getInstance().error("Failed to configure accelerometer ODR");
        return false;
    }

    // Configure accelerometer range
    uint8_t acc_range = 0;
    switch (config_.accel_range_g) {
        case 2:  acc_range = 0x00; break;
        case 4:  acc_range = 0x01; break;
        case 8:  acc_range = 0x02; break;
        case 16: acc_range = 0x03; break;
        default: acc_range = 0x00; break;
    }

    if (!writeRegister(REG_ACC_RANGE, acc_range)) {
        core::Logger::getInstance().error("Failed to configure accelerometer range");
        return false;
    }

    // Configure gyroscope
    uint8_t gyr_conf = acc_conf;  // Use same ODR as accelerometer
    if (!writeRegister(REG_GYR_CONF, gyr_conf)) {
        core::Logger::getInstance().error("Failed to configure gyroscope ODR");
        return false;
    }

    // Configure gyroscope range
    uint8_t gyr_range = 0;
    switch (config_.gyro_range_dps) {
        case 125:  gyr_range = 0x04; break;
        case 250:  gyr_range = 0x03; break;
        case 500:  gyr_range = 0x02; break;
        case 1000: gyr_range = 0x01; break;
        case 2000: gyr_range = 0x00; break;
        default:   gyr_range = 0x02; break;
    }

    if (!writeRegister(REG_GYR_RANGE, gyr_range)) {
        core::Logger::getInstance().error("Failed to configure gyroscope range");
        return false;
    }

    // Enable accelerometer and gyroscope
    uint8_t pwr_ctrl = PWR_CTRL_ACC_EN | PWR_CTRL_GYR_EN | PWR_CTRL_TEMP_EN;
    if (!writeRegister(REG_PWR_CTRL, pwr_ctrl)) {
        core::Logger::getInstance().error("Failed to enable sensors");
        return false;
    }

    // Wait for sensors to power up
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    core::Logger::getInstance().info("BMI270 configuration complete");
    return true;
}

bool BMI270Driver::writeRegister(uint8_t reg, uint8_t value) {
    if (i2c_fd_ < 0) {
        return false;
    }

    uint8_t buffer[2] = {reg, value};
    if (write(i2c_fd_, buffer, 2) != 2) {
        core::Logger::getInstance().error("Failed to write register 0x" +
                          std::to_string(reg));
        return false;
    }

    return true;
}

bool BMI270Driver::readRegister(uint8_t reg, uint8_t& value) {
    if (i2c_fd_ < 0) {
        return false;
    }

    if (write(i2c_fd_, &reg, 1) != 1) {
        core::Logger::getInstance().error("Failed to write register address 0x" +
                          std::to_string(reg));
        return false;
    }

    if (read(i2c_fd_, &value, 1) != 1) {
        core::Logger::getInstance().error("Failed to read register 0x" +
                          std::to_string(reg));
        return false;
    }

    return true;
}

bool BMI270Driver::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
    if (i2c_fd_ < 0 || buffer == nullptr) {
        return false;
    }

    if (write(i2c_fd_, &reg, 1) != 1) {
        core::Logger::getInstance().error("Failed to write register address 0x" +
                          std::to_string(reg));
        return false;
    }

    if (read(i2c_fd_, buffer, length) != static_cast<ssize_t>(length)) {
        core::Logger::getInstance().error("Failed to read " + std::to_string(length) +
                          " bytes from register 0x" + std::to_string(reg));
        return false;
    }

    return true;
}

bool BMI270Driver::verifyChipID() {
    uint8_t chip_id = 0;
    if (!readRegister(REG_CHIP_ID, chip_id)) {
        core::Logger::getInstance().error("Failed to read chip ID");
        return false;
    }

    {
        std::lock_guard<std::mutex> status_lock(status_mutex_);
        status_.chip_id = chip_id;
    }

    if (chip_id != CHIP_ID_BMI270) {
        std::stringstream ss;
        ss << "Invalid chip ID: expected 0x" << std::hex << static_cast<int>(CHIP_ID_BMI270)
           << ", got 0x" << std::hex << static_cast<int>(chip_id);
        core::Logger::getInstance().error(ss.str());
        return false;
    }

    core::Logger::getInstance().info("BMI270 chip ID verified: 0x" + std::to_string(chip_id));
    return true;
}

float BMI270Driver::convertGyroRaw(int16_t raw) const {
    return static_cast<float>(raw) * gyro_scale_;
}

float BMI270Driver::convertAccelRaw(int16_t raw) const {
    return static_cast<float>(raw) * accel_scale_;
}

uint64_t BMI270Driver::getMicroseconds() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

bool BMI270Driver::validateConfiguration(const BMI270Config& config) const {
    // Validate I2C bus
    if (config.i2c_bus > 10) {
        core::Logger::getInstance().error("Invalid I2C bus: " + std::to_string(config.i2c_bus));
        return false;
    }

    // Validate I2C address
    if (config.i2c_address != 0x68 && config.i2c_address != 0x69) {
        core::Logger::getInstance().error("Invalid I2C address: 0x" +
                          std::to_string(config.i2c_address) + " (expected 0x68 or 0x69)");
        return false;
    }

    // Validate gyro range
    if (config.gyro_range_dps != 125 && config.gyro_range_dps != 250 &&
        config.gyro_range_dps != 500 && config.gyro_range_dps != 1000 &&
        config.gyro_range_dps != 2000) {
        core::Logger::getInstance().error("Invalid gyro range: " +
                          std::to_string(config.gyro_range_dps) + " dps");
        return false;
    }

    // Validate accel range
    if (config.accel_range_g != 2 && config.accel_range_g != 4 &&
        config.accel_range_g != 8 && config.accel_range_g != 16) {
        core::Logger::getInstance().error("Invalid accel range: " +
                          std::to_string(config.accel_range_g) + " g");
        return false;
    }

    // Validate sample rate
    if (config.sample_rate_hz != 25 && config.sample_rate_hz != 50 &&
        config.sample_rate_hz != 100 && config.sample_rate_hz != 200 &&
        config.sample_rate_hz != 400 && config.sample_rate_hz != 800 &&
        config.sample_rate_hz != 1600) {
        core::Logger::getInstance().error("Invalid sample rate: " +
                          std::to_string(config.sample_rate_hz) + " Hz");
        return false;
    }

    return true;
}

void BMI270Driver::setErrorState(const std::string& error) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.error_message = error;
    core::Logger::getInstance().error("BMI270: " + error);
}

void BMI270Driver::clearErrorState() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    status_.error_message.clear();
    status_.initialized = true;
}

BMI270Driver::IMUData BMI270Driver::generateMockData() {
    IMUData data;

    // Generate smooth sinusoidal motion to simulate handheld movement
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - mock_start_time_).count();
    float t = elapsed / 1000.0f;  // Time in seconds

    // Simulate slow rotation around Z axis (yaw)
    data.gyro_x = 0.2f * std::sin(2.0f * M_PI * 0.1f * t);
    data.gyro_y = 0.15f * std::cos(2.0f * M_PI * 0.15f * t);
    data.gyro_z = 0.3f * std::sin(2.0f * M_PI * 0.2f * t);

    // Simulate gravity + small vibrations
    data.accel_x = 0.5f * std::sin(2.0f * M_PI * 0.5f * t);
    data.accel_y = 0.3f * std::cos(2.0f * M_PI * 0.7f * t);
    data.accel_z = 9.81f + 0.2f * std::sin(2.0f * M_PI * 1.0f * t);

    data.timestamp_us = getMicroseconds();
    data.valid = true;

    return data;
}

} // namespace hardware
} // namespace unlook
