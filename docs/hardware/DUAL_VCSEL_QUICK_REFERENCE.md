# AS1170 Dual VCSEL Quick Reference

## 30-Second Integration Guide

### Minimal Working Code
```cpp
#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/camera/CameraSystem.hpp>

using namespace unlook;

int main() {
    // 1. Initialize camera
    auto camera = camera::CameraSystem::getInstance();
    camera->initialize();
    camera->startCapture();

    // 2. Initialize dual VCSEL
    auto vcsel = hardware::AS1170DualVCSELController::getInstance();
    vcsel->initialize(camera);

    // 3. Capture temporal sequence
    hardware::AS1170DualVCSELController::TripleFrameCapture capture;
    if (vcsel->captureTemporalSequence(capture)) {
        // Process 6 frames: vcsel1_left, vcsel1_right, vcsel2_left,
        //                   vcsel2_right, ambient_left, ambient_right
        processFrames(capture);
    }

    // 4. Cleanup
    vcsel->shutdown();
    camera->shutdown();
    return 0;
}
```

## Hardware Setup

### Physical Configuration
```
VCSEL1 ───2cm───> LEFT Camera (Camera 1, MASTER)
VCSEL2 ───2cm───> RIGHT Camera (Camera 0, SLAVE)

AS1170: I2C bus 1, address 0x30, GPIO 19
Current: 200mA per VCSEL (safe), 446mA max (hardware limit)
```

### Triple Frame Sequence
```
Time ──────────────────────────────────────>

Frame A: VCSEL1 ON  → Capture → VCSEL1 OFF (pattern from left)
Frame B: VCSEL2 ON  → Capture → VCSEL2 OFF (pattern from right)
Frame C: Both OFF   → Capture            (ambient for subtraction)

Total: ~400-500ms per sequence
```

## Key Functions

### Initialization
```cpp
// Basic (use defaults)
vcsel->initialize(camera_system);

// Advanced (custom config)
hardware::AS1170DualVCSELController::VCSELConfig config;
config.vcsel1_current_ma = 200;
config.vcsel2_current_ma = 200;
config.settle_time_ms = 50;
vcsel->initialize(camera_system, config);
```

### Capture
```cpp
TripleFrameCapture capture;
if (vcsel->captureTemporalSequence(capture)) {
    // SUCCESS: capture.is_valid == true
    // 6 frames available in capture struct
}
```

### Pattern Processing
```cpp
// Subtract ambient from pattern frames
cv::Mat clean1_left = capture.frame_vcsel1_left - capture.frame_ambient_left;
cv::Mat clean1_right = capture.frame_vcsel1_right - capture.frame_ambient_right;

cv::Mat clean2_left = capture.frame_vcsel2_left - capture.frame_ambient_left;
cv::Mat clean2_right = capture.frame_vcsel2_right - capture.frame_ambient_right;

// Now clean1_* contains ONLY VCSEL1 pattern
// And clean2_* contains ONLY VCSEL2 pattern
```

### Manual Control (debugging)
```cpp
vcsel->activateVCSEL1(200);     // Turn on VCSEL1 at 200mA
vcsel->deactivateVCSEL1();      // Turn off VCSEL1

vcsel->activateVCSEL2(200);     // Turn on VCSEL2 at 200mA
vcsel->deactivateVCSEL2();      // Turn off VCSEL2

vcsel->deactivateBoth();        // Turn off both immediately
vcsel->emergencyShutdown();     // Emergency stop (<5ms)
```

### Status Monitoring
```cpp
auto status = vcsel->getStatus();
std::cout << "VCSEL1 active: " << status.vcsel1_active << std::endl;
std::cout << "VCSEL2 active: " << status.vcsel2_active << std::endl;
std::cout << "Temperature: " << status.temperature_c << " C" << std::endl;
std::cout << "Total captures: " << status.total_captures << std::endl;
```

## Configuration Options

### VCSELConfig Structure
```cpp
struct VCSELConfig {
    uint16_t vcsel1_current_ma = 200;          // LED1 current (mA)
    uint16_t vcsel2_current_ma = 200;          // LED2 current (mA)
    uint32_t settle_time_ms = 50;              // Time between activations
    uint32_t capture_delay_ms = 10;            // Delay before capture
    uint32_t max_on_time_ms = 5000;            // Max continuous ON
    bool enable_thermal_monitoring = true;     // Temp monitoring
    float max_operating_temp_c = 70.0f;        // Max safe temp
    bool use_strobe_mode = false;              // Strobe vs continuous
    uint32_t strobe_duration_us = 1000;        // Strobe duration
};
```

### Timing Presets

**Fastest (not recommended for production)**
```cpp
config.settle_time_ms = 10;      // Minimum
config.capture_delay_ms = 0;      // No delay
// Risk: Thermal issues, pattern instability
// Time: ~200ms per sequence
```

**Balanced (recommended)**
```cpp
config.settle_time_ms = 50;      // Good thermal stability
config.capture_delay_ms = 10;     // Pattern fully stable
// Time: ~400ms per sequence
```

**Safe (maximum thermal protection)**
```cpp
config.settle_time_ms = 100;     // Excellent cooling
config.capture_delay_ms = 20;     // Extra stability
// Time: ~600ms per sequence
```

## TripleFrameCapture Data

### Frame Access
```cpp
TripleFrameCapture capture;
vcsel->captureTemporalSequence(capture);

// 6 frames available
cv::Mat& vcsel1_left = capture.frame_vcsel1_left;
cv::Mat& vcsel1_right = capture.frame_vcsel1_right;
cv::Mat& vcsel2_left = capture.frame_vcsel2_left;
cv::Mat& vcsel2_right = capture.frame_vcsel2_right;
cv::Mat& ambient_left = capture.frame_ambient_left;
cv::Mat& ambient_right = capture.frame_ambient_right;
```

### Metadata Access
```cpp
// Timing
uint64_t vcsel1_time = capture.vcsel1_activation_us;  // microseconds
uint64_t vcsel2_time = capture.vcsel2_activation_us;
uint64_t total_time = capture.total_sequence_us;

// Sync errors
double vcsel1_sync = capture.vcsel1_sync_error_ms;    // milliseconds
double vcsel2_sync = capture.vcsel2_sync_error_ms;
double ambient_sync = capture.ambient_sync_error_ms;

// Thermal
float temp = capture.temperature_c;                   // Celsius

// Validation
bool valid = capture.is_valid;
auto timestamp = capture.capture_time;
```

## Safety Features

### Automatic Protection
- **Current limiting**: 200mA safe, 446mA absolute max
- **Thermal monitoring**: Continuous, 70°C limit
- **ON time limiting**: 5 second max continuous
- **Emergency shutdown**: <5ms response

### Thermal Callback
```cpp
vcsel->setThermalCallback([](bool active, float temp) {
    if (active) {
        std::cerr << "THERMAL WARNING: " << temp << "C" << std::endl;
    }
});
```

## Common Patterns

### Save Frames
```cpp
cv::imwrite("vcsel1_left.png", capture.frame_vcsel1_left);
cv::imwrite("vcsel1_right.png", capture.frame_vcsel1_right);
cv::imwrite("vcsel2_left.png", capture.frame_vcsel2_left);
cv::imwrite("vcsel2_right.png", capture.frame_vcsel2_right);
cv::imwrite("ambient_left.png", capture.frame_ambient_left);
cv::imwrite("ambient_right.png", capture.frame_ambient_right);
```

### Display Frames
```cpp
cv::namedWindow("VCSEL1 Left", cv::WINDOW_NORMAL);
cv::imshow("VCSEL1 Left", capture.frame_vcsel1_left);
cv::waitKey(0);
```

### Pattern Enhancement Check
```cpp
cv::Scalar mean1, stddev1, meanA, stddevA;
cv::meanStdDev(capture.frame_vcsel1_left, mean1, stddev1);
cv::meanStdDev(capture.frame_ambient_left, meanA, stddevA);

double enhancement = stddev1[0] / (stddevA[0] + 1e-6);
std::cout << "Pattern enhancement: " << enhancement << "x" << std::endl;

if (enhancement < 1.2) {
    std::cerr << "WARNING: Low pattern visibility" << std::endl;
}
```

## Troubleshooting

### Pattern Not Visible
- Check VCSEL alignment (2cm from camera)
- Increase current (up to 200mA)
- Verify scene distance (0.3-2.0m optimal)
- Reduce ambient light

### Thermal Throttling
- Increase settle_time_ms (try 100ms)
- Reduce current (try 150mA)
- Add cooldown between captures
- Check ambient temperature

### High Sync Errors
- Verify hardware sync (XVS, XHS, MAS)
- Enable sync in camera config
- Reduce capture rate
- Check system load

### Capture Failures
- Check camera initialization
- Verify I2C (bus 1, addr 0x30)
- Check GPIO 19 permissions
- Review error message in status

## Build and Run

### Build
```bash
cd /home/alessandro/unlook-standalone
./build.sh
```

### Run Test
```bash
# Single capture
./build/examples/test_dual_vcsel_temporal

# Multiple captures
./build/examples/test_dual_vcsel_temporal 10
```

### Output Location
```
temporal_captures/
├── capture_0_vcsel1_left.png
├── capture_0_vcsel1_right.png
├── capture_0_vcsel2_left.png
├── capture_0_vcsel2_right.png
├── capture_0_ambient_left.png
└── capture_0_ambient_right.png
```

## Performance

### Typical Timing
- Frame A (VCSEL1): 62ms activation + 33ms capture = 95ms
- Frame B (VCSEL2): 62ms activation + 33ms capture = 95ms
- Frame C (Ambient): 33ms capture
- Total: ~400-500ms per sequence

### Thermal Behavior
- Operating current: 200mA per VCSEL
- Typical temp: 40-50°C (ambient 25°C)
- Max safe temp: 70°C (automatic throttling)

### Sync Precision
- Hardware sync: <1ms typical
- Tolerance: Configurable (default 1ms)

## Files Reference

### Implementation
- Header: `/home/alessandro/unlook-standalone/include/unlook/hardware/AS1170DualVCSELController.hpp`
- Source: `/home/alessandro/unlook-standalone/src/hardware/AS1170DualVCSELController.cpp`
- Example: `/home/alessandro/unlook-standalone/examples/test_dual_vcsel_temporal.cpp`

### Documentation
- User Guide: `/home/alessandro/unlook-standalone/DUAL_VCSEL_TEMPORAL_MATCHING.md`
- Summary: `/home/alessandro/unlook-standalone/AS1170_DUAL_VCSEL_IMPLEMENTATION_SUMMARY.md`
- Quick Ref: `/home/alessandro/unlook-standalone/DUAL_VCSEL_QUICK_REFERENCE.md`

---

**Quick Reference Version**: 1.0
**Last Updated**: 2025-10-08
**Status**: Production Ready
