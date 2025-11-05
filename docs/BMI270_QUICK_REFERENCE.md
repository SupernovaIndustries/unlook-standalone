# BMI270 IMU Driver - Quick Reference Card

## Quick Start (5 Minutes)

### 1. Basic Usage

```cpp
#include <unlook/hardware/BMI270Driver.hpp>
#include <unlook/hardware/StabilityDetector.hpp>

// Initialize IMU
auto imu = unlook::hardware::BMI270Driver::getInstance();
imu->initialize();

// Initialize stability detector
auto stability = std::make_shared<unlook::hardware::StabilityDetector>(imu);
stability->initialize();

// Main loop
while (running) {
    stability->update();

    if (stability->isStable()) {
        std::cout << "STABLE - Ready to scan!" << std::endl;
    }

    float score = stability->getStabilityScore();
    std::cout << "Stability: " << (score * 100) << "%" << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(33)); // 30 Hz
}
```

### 2. Run Example

```bash
# Build
./build.sh

# Run with hardware
./build/examples/bmi270_stability_example

# Run without hardware (mock mode)
./build/examples/bmi270_stability_example --mock
```

---

## Configuration Cheat Sheet

### Default Configuration (Works Out of Box)
```cpp
I2C Bus:        1
I2C Address:    0x69
Gyro Range:     ±500 deg/sec
Accel Range:    ±2g
Sample Rate:    100 Hz
```

### Custom Configuration
```cpp
BMI270Driver::BMI270Config config;
config.i2c_bus = 1;
config.i2c_address = 0x69;
config.gyro_range_dps = 500;      // 125, 250, 500, 1000, 2000
config.accel_range_g = 2;          // 2, 4, 8, 16
config.sample_rate_hz = 100;       // 25, 50, 100, 200, 400, 800, 1600
config.enable_mock_mode = false;   // true for testing without hardware

imu->initialize(config);
```

### Stability Parameters
```cpp
StabilityDetector::StabilityParams params;
params.gyro_threshold_dps = 0.5f;           // Max rotation (deg/sec)
params.accel_variance_threshold = 0.1f;     // Max movement (m/s²)
params.stable_duration_ms = 500;            // Time to be stable (ms)
params.history_window_ms = 1000;            // Analysis window (ms)
params.gyro_weight = 0.7f;                  // Gyro importance (70%)
params.accel_weight = 0.3f;                 // Accel importance (30%)

stability->initialize(params);
```

---

## API Quick Reference

### BMI270Driver

```cpp
// Initialization
bool initialize(const BMI270Config& config = BMI270Config());
void shutdown();
bool isInitialized() const;

// Data Reading
bool readIMUData(IMUData& data);

// Status
BMI270Status getStatus() const;

// Testing
void setMockMode(bool enable);
void setMockData(const IMUData& data);
bool softReset();
```

### IMUData Structure
```cpp
struct IMUData {
    float gyro_x, gyro_y, gyro_z;     // deg/sec
    float accel_x, accel_y, accel_z;  // m/s²
    uint64_t timestamp_us;             // microseconds
    bool valid;
};
```

### StabilityDetector

```cpp
// Initialization
bool initialize(const StabilityParams& params = StabilityParams());
void shutdown();
bool isInitialized() const;

// Updates (call at 30-100 Hz)
bool update();

// Stability Queries
bool isStable() const;                    // Is currently stable?
float getStabilityScore() const;          // Score 0.0-1.0
int getStableDuration() const;            // Duration in ms
StabilityStatus getStatus() const;        // Full status

// Configuration
void setParameters(const StabilityParams& params);
StabilityParams getParameters() const;
void reset();
```

---

## Common Patterns

### Pattern 1: Wait for Stability Before Scan
```cpp
while (!stability->isStable()) {
    stability->update();
    updateGUI(stability->getStabilityScore());
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
}

// Now stable - capture frames
captureHighQualityScan();
```

### Pattern 2: GUI Feedback
```cpp
void updateStabilityIndicator() {
    float score = stability->getStabilityScore();

    // Update progress bar
    progressBar->setValue(score * 100);

    // Update color
    if (score > 0.9f)       indicator->setColor(Qt::green);
    else if (score > 0.6f)  indicator->setColor(Qt::yellow);
    else                     indicator->setColor(Qt::red);

    // Update text
    if (stability->isStable()) {
        label->setText(QString("STABLE (%1ms)")
            .arg(stability->getStableDuration()));
    } else {
        label->setText("Hold steady...");
    }
}
```

### Pattern 3: Continuous Monitoring
```cpp
void monitoringThread() {
    while (running) {
        // Update at 30 Hz for smooth GUI
        stability->update();

        auto status = stability->getStatus();
        logData(status);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}
```

---

## Troubleshooting

### Error: "Failed to open I2C device"
```bash
# Enable I2C
sudo raspi-config  # → Interface Options → I2C → Enable

# Check device exists
ls -l /dev/i2c-*

# Fix permissions
sudo usermod -a -G i2c $USER
# (logout/login required)

# OR: Use mock mode for testing
config.enable_mock_mode = true;
```

### Error: "Invalid chip ID"
```bash
# Check I2C connection
sudo i2cdetect -y 1

# Should show 0x69:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 60: -- -- -- -- -- -- -- -- -- 69 -- -- -- -- -- --

# If not visible:
# - Check wiring (SDA, SCL, VCC, GND)
# - Check 3.3V power
# - Check pull-up resistors (4.7kΩ)
```

### Error: "Never reaches stable state"
```cpp
// Try relaxing thresholds
params.gyro_threshold_dps = 1.0f;        // was 0.5
params.accel_variance_threshold = 0.2f;  // was 0.1
params.stable_duration_ms = 250;         // was 500

stability->setParameters(params);
```

---

## Performance Targets

| Metric | Target | Typical |
|--------|--------|---------|
| Time to Stable | <500ms | ~400ms |
| IMU Read Latency | <10ms | ~5ms |
| Update Latency | <10ms | ~8ms |
| Sample Rate | 100 Hz | 100 Hz |
| I2C Success | >99% | >99.9% |

---

## Files Reference

```
Headers:
  include/unlook/hardware/BMI270Driver.hpp
  include/unlook/hardware/StabilityDetector.hpp

Implementation:
  src/hardware/BMI270Driver.cpp
  src/hardware/StabilityDetector.cpp

Example:
  examples/bmi270_stability_example.cpp

Documentation:
  docs/BMI270_STABILITY_DETECTOR.md       (full guide)
  docs/BMI270_IMPLEMENTATION_REPORT.md    (technical report)
  docs/BMI270_QUICK_REFERENCE.md          (this file)
```

---

## Integration Checklist

- [ ] Include headers in your source file
- [ ] Link `unlook_hardware` library in CMakeLists.txt
- [ ] Initialize BMI270Driver first
- [ ] Initialize StabilityDetector with BMI270 instance
- [ ] Call `update()` in main loop (30-100 Hz)
- [ ] Check `isStable()` before captures
- [ ] Display `getStabilityScore()` in GUI
- [ ] Handle errors gracefully
- [ ] Shutdown both objects on exit

---

## Hardware Connection

```
BMI270          Raspberry Pi
------          ------------
VCC     →       Pin 1  (3.3V)
GND     →       Pin 6  (GND)
SDA     →       Pin 3  (GPIO 2 / SDA1)
SCL     →       Pin 5  (GPIO 3 / SCL1)
SDO     →       3.3V (sets address to 0x69)
```

**Important**: 4.7kΩ pull-up resistors on SDA and SCL lines

---

## Support

- **Full Documentation**: `docs/BMI270_STABILITY_DETECTOR.md`
- **Technical Report**: `docs/BMI270_IMPLEMENTATION_REPORT.md`
- **Example Code**: `examples/bmi270_stability_example.cpp`
- **GitHub Issues**: [unlook-standalone repository]

---

**Last Updated**: 2025-11-04
**Version**: 1.0.0
**Status**: Production Ready ✅
