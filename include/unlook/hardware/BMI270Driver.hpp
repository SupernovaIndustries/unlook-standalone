#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <vector>
#include <cstdint>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace unlook {
namespace hardware {

/**
 * BMI270 6-Axis IMU Driver (Bosch Sensortec)
 *
 * Provides low-level I2C communication for BMI270 6-axis IMU (3-axis gyro + 3-axis accel).
 * Used for handheld scanner stability detection and motion analysis.
 *
 * Hardware Configuration:
 * - I2C Bus: 1 (/dev/i2c-1)
 * - I2C Address: 0x69 (SDO pulled high)
 * - Update Rate: 100 Hz (configurable)
 * - Gyro Range: ±500 deg/sec (configurable)
 * - Accel Range: ±2g (configurable)
 *
 * Features:
 * - Real-time IMU data acquisition at up to 1600 Hz
 * - Automatic sensor calibration and bias compensation
 * - Low-latency data access (<10ms from hardware to application)
 * - Thread-safe operation with mutex protection
 * - Mock mode for testing without hardware
 *
 * Thread Safety: All public methods are thread-safe
 */
class BMI270Driver {
public:
    /**
     * IMU data structure with calibrated sensor readings
     */
    struct IMUData {
        // Gyroscope data (deg/sec)
        float gyro_x;
        float gyro_y;
        float gyro_z;

        // Accelerometer data (m/s²)
        float accel_x;
        float accel_y;
        float accel_z;

        // Timestamp in microseconds
        uint64_t timestamp_us;

        // Data validity flag
        bool valid;

        IMUData() :
            gyro_x(0), gyro_y(0), gyro_z(0),
            accel_x(0), accel_y(0), accel_z(0),
            timestamp_us(0), valid(false) {}
    };

    /**
     * Configuration structure for BMI270
     */
    struct BMI270Config {
        uint8_t i2c_bus;              // I2C bus number (default: 1)
        uint8_t i2c_address;          // I2C address (default: 0x69)
        uint16_t gyro_range_dps;      // Gyro range in deg/sec (125, 250, 500, 1000, 2000)
        uint8_t accel_range_g;        // Accel range in g (2, 4, 8, 16)
        uint16_t sample_rate_hz;      // Sample rate in Hz (25, 50, 100, 200, 400, 800, 1600)
        bool enable_mock_mode;        // Enable mock mode for testing

        BMI270Config() :
            i2c_bus(1),
            i2c_address(0x69),
            gyro_range_dps(500),
            accel_range_g(2),
            sample_rate_hz(100),
            enable_mock_mode(false) {}
    };

    /**
     * Status structure for BMI270
     */
    struct BMI270Status {
        bool initialized = false;
        bool i2c_connected = false;
        uint8_t chip_id = 0;
        float temperature_c = 0.0f;
        uint64_t sample_count = 0;
        std::string error_message;
        std::chrono::steady_clock::time_point last_update;
    };

    /**
     * Constructor
     */
    BMI270Driver();

    /**
     * Destructor
     */
    ~BMI270Driver();

    /**
     * Get singleton instance
     */
    static std::shared_ptr<BMI270Driver> getInstance();

    /**
     * Initialize BMI270 with configuration
     * @param config BMI270 configuration
     * @return true if initialization successful
     */
    bool initialize(const BMI270Config& config = BMI270Config());

    /**
     * Shutdown BMI270 and cleanup resources
     */
    void shutdown();

    /**
     * Read current IMU data
     * @param data Output IMU data structure
     * @return true if read successful
     */
    bool readIMUData(IMUData& data);

    /**
     * Get current status
     * @return BMI270 status structure
     */
    BMI270Status getStatus() const;

    /**
     * Check if BMI270 is initialized
     * @return true if initialized
     */
    bool isInitialized() const { return initialized_.load(); }

    /**
     * Perform soft reset
     * @return true if reset successful
     */
    bool softReset();

    /**
     * Enable mock mode with simulated data
     * @param enable Enable/disable mock mode
     */
    void setMockMode(bool enable);

    /**
     * Set mock IMU data (for testing)
     * @param data Mock IMU data to return
     */
    void setMockData(const IMUData& data);

private:
    // I2C communication methods
    bool initializeI2C();
    bool configureIMU();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);
    bool verifyChipID();

    // Data conversion methods
    float convertGyroRaw(int16_t raw) const;
    float convertAccelRaw(int16_t raw) const;
    uint64_t getMicroseconds() const;

    // Configuration validation
    bool validateConfiguration(const BMI270Config& config) const;

    // Error handling
    void setErrorState(const std::string& error);
    void clearErrorState();

    // Mock mode methods
    IMUData generateMockData();

    // Member variables
    BMI270Config config_;
    BMI270Status status_;
    std::atomic<bool> initialized_;
    std::atomic<bool> mock_mode_;
    mutable std::mutex mutex_;
    mutable std::mutex status_mutex_;

    int i2c_fd_;

    // Conversion factors (calculated based on config)
    float gyro_scale_;
    float accel_scale_;

    // Mock data
    IMUData mock_data_;
    std::chrono::steady_clock::time_point mock_start_time_;

    // Sample counter
    std::atomic<uint64_t> sample_count_;

    // BMI270 Register Map
    static constexpr uint8_t REG_CHIP_ID = 0x00;
    static constexpr uint8_t REG_ERR_REG = 0x02;
    static constexpr uint8_t REG_STATUS = 0x03;
    static constexpr uint8_t REG_DATA_0 = 0x04;  // Start of sensor data (ACC_X_LSB)
    static constexpr uint8_t REG_DATA_ACC_X_LSB = 0x0C;
    static constexpr uint8_t REG_DATA_ACC_X_MSB = 0x0D;
    static constexpr uint8_t REG_DATA_GYR_X_LSB = 0x12;
    static constexpr uint8_t REG_DATA_GYR_X_MSB = 0x13;
    static constexpr uint8_t REG_SENSORTIME_0 = 0x18;
    static constexpr uint8_t REG_INT_STATUS_0 = 0x1C;
    static constexpr uint8_t REG_TEMPERATURE_0 = 0x22;
    static constexpr uint8_t REG_ACC_CONF = 0x40;
    static constexpr uint8_t REG_ACC_RANGE = 0x41;
    static constexpr uint8_t REG_GYR_CONF = 0x42;
    static constexpr uint8_t REG_GYR_RANGE = 0x43;
    static constexpr uint8_t REG_PWR_CONF = 0x7C;
    static constexpr uint8_t REG_PWR_CTRL = 0x7D;
    static constexpr uint8_t REG_CMD = 0x7E;

    // BMI270 Constants
    static constexpr uint8_t CHIP_ID_BMI270 = 0x24;
    static constexpr uint8_t CMD_SOFT_RESET = 0xB6;

    // Power control bits
    static constexpr uint8_t PWR_CTRL_ACC_EN = 0x04;
    static constexpr uint8_t PWR_CTRL_GYR_EN = 0x02;
    static constexpr uint8_t PWR_CTRL_TEMP_EN = 0x08;

    // ODR values (Output Data Rate)
    static constexpr uint8_t ODR_25HZ = 0x06;
    static constexpr uint8_t ODR_50HZ = 0x07;
    static constexpr uint8_t ODR_100HZ = 0x08;
    static constexpr uint8_t ODR_200HZ = 0x09;
    static constexpr uint8_t ODR_400HZ = 0x0A;
    static constexpr uint8_t ODR_800HZ = 0x0B;
    static constexpr uint8_t ODR_1600HZ = 0x0C;
};

} // namespace hardware
} // namespace unlook
