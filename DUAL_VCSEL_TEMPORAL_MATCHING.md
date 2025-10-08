# AS1170 Dual VCSEL Temporal Matching System

## Overview

This document describes the dual VCSEL temporal matching implementation for the Unlook 3D Scanner, enabling high-precision depth estimation through temporal pattern projection.

## Hardware Configuration

### VCSEL Placement
```
        VCSEL1 (LED1)                    VCSEL2 (LED2)
             |                                |
             v                                v
    +----------------+              +----------------+
    |  LEFT CAMERA   |              | RIGHT CAMERA   |
    |   (MASTER)     |   70.017mm   |   (SLAVE)      |
    +----------------+              +----------------+
         Camera 1                        Camera 0
```

**Physical Layout:**
- VCSEL1 (LED1): Positioned 2cm from LEFT camera (Camera 1, MASTER)
  - 15K dot pattern (OSRAM BELAGO 1.1/1.2)
  - Controlled by AS1170 LED1 channel
  - Operating current: 200mA (safety margin from 446mA max)

- VCSEL2 (LED2): Positioned 2cm from RIGHT camera (Camera 0, SLAVE)
  - 15K dot pattern (identical to VCSEL1)
  - Controlled by AS1170 LED2 channel
  - Operating current: 200mA

### AS1170 LED Driver
- **I2C Bus**: 1
- **I2C Address**: 0x30
- **Strobe GPIO**: 19
- **Maximum Current**: 446mA per channel (hardware limit)
- **Operating Current**: 200mA per VCSEL (safety margin)
- **Operating Mode**: TORCH MODE (continuous illumination)
- **Flash Mode**: Optional strobe mode available

## Temporal Matching Strategy

### Why Temporal Matching?

Traditional stereo matching with a single pattern projector suffers from pattern correspondence ambiguity:
- Dot patterns look identical from both cameras
- Difficult to determine which dot corresponds to which
- Requires complex pattern matching algorithms

**Temporal matching solves this by:**
1. Projecting from different angles (left vs right)
2. Capturing ambient frame for pattern subtraction
3. Eliminating pattern ambiguity through geometric constraints

### Triple Frame Capture Sequence

The system captures three synchronized stereo frame pairs:

```
FRAME A: VCSEL1 ON (Pattern from left side)
  ├─ Left camera sees strong pattern (near VCSEL1)
  ├─ Right camera sees pattern at angle
  └─ Capture synchronized stereo pair

FRAME B: VCSEL2 ON (Pattern from right side)
  ├─ Left camera sees pattern at angle
  ├─ Right camera sees strong pattern (near VCSEL2)
  └─ Capture synchronized stereo pair

FRAME C: Both OFF (Ambient illumination)
  ├─ No pattern projection
  ├─ Ambient scene illumination only
  └─ Used for pattern subtraction
```

### Timing Diagram
```
Time ─────────────────────────────────────────────────────>

VCSEL1  ░░░░░░░░░ON░░░░░░░░░░░░░OFF░░░░░░░░░░░░░░░░░░░░OFF░
        ▲                      ▲
        │ Settle: 50ms        │ Settle: 50ms
        │ Capture delay: 10ms │

VCSEL2  ░░░░░░░░░OFF░░░░░░░░░░░ON░░░░░░░░░░░░░░OFF░░░░░░░░░
                              ▲                ▲
                              │ Settle: 50ms  │
                              │ Delay: 10ms   │

Capture    [A]                  [B]              [C]
           ▲                    ▲                ▲
           │                    │                │
     Frame A (VCSEL1)     Frame B (VCSEL2)  Frame C (Ambient)
```

**Total sequence time**: ~400-500ms (configurable)
- VCSEL1 activation: ~60ms (50ms settle + 10ms delay)
- Frame A capture: ~33ms (30 FPS)
- VCSEL1 → VCSEL2 transition: 50ms
- VCSEL2 activation: ~60ms
- Frame B capture: ~33ms
- VCSEL2 → OFF transition: 50ms
- Frame C capture: ~33ms

## API Usage

### Basic Usage

```cpp
#include <unlook/hardware/AS1170DualVCSELController.hpp>
#include <unlook/camera/CameraSystem.hpp>

using namespace unlook;

// 1. Initialize camera system
auto camera_system = camera::CameraSystem::getInstance();
camera::CameraConfig cam_config;
cam_config.enableSync = true;
cam_config.targetFps = 30.0;
camera_system->initialize(cam_config);
camera_system->startCapture();

// 2. Initialize dual VCSEL controller
auto vcsel_controller = hardware::AS1170DualVCSELController::getInstance();

hardware::AS1170DualVCSELController::VCSELConfig vcsel_config;
vcsel_config.vcsel1_current_ma = 200;    // 200mA for VCSEL1
vcsel_config.vcsel2_current_ma = 200;    // 200mA for VCSEL2
vcsel_config.settle_time_ms = 50;        // 50ms settle time
vcsel_config.capture_delay_ms = 10;      // 10ms capture delay

vcsel_controller->initialize(camera_system, vcsel_config);

// 3. Capture temporal sequence
hardware::AS1170DualVCSELController::TripleFrameCapture capture;
if (vcsel_controller->captureTemporalSequence(capture)) {
    // Process frames
    processVCSEL1Pattern(capture.frame_vcsel1_left, capture.frame_vcsel1_right);
    processVCSEL2Pattern(capture.frame_vcsel2_left, capture.frame_vcsel2_right);
    processAmbient(capture.frame_ambient_left, capture.frame_ambient_right);
}

// 4. Cleanup
vcsel_controller->shutdown();
camera_system->stopCapture();
camera_system->shutdown();
```

### Advanced Pattern Processing

```cpp
// Pattern subtraction (remove ambient illumination)
cv::Mat vcsel1_left_clean = capture.frame_vcsel1_left - capture.frame_ambient_left;
cv::Mat vcsel1_right_clean = capture.frame_vcsel1_right - capture.frame_ambient_right;

cv::Mat vcsel2_left_clean = capture.frame_vcsel2_left - capture.frame_ambient_left;
cv::Mat vcsel2_right_clean = capture.frame_vcsel2_right - capture.frame_ambient_right;

// Now vcsel1_left_clean contains ONLY the pattern from VCSEL1
// And vcsel2_left_clean contains ONLY the pattern from VCSEL2
```

### Manual VCSEL Control

```cpp
// Individual VCSEL control (for debugging)
vcsel_controller->activateVCSEL1(200);  // Activate VCSEL1 at 200mA
std::this_thread::sleep_for(std::chrono::milliseconds(50));
vcsel_controller->deactivateVCSEL1();

vcsel_controller->activateVCSEL2(200);  // Activate VCSEL2 at 200mA
std::this_thread::sleep_for(std::chrono::milliseconds(50));
vcsel_controller->deactivateVCSEL2();

// Emergency shutdown (immediate)
vcsel_controller->emergencyShutdown();
```

## Safety Features

### Thermal Protection

The system implements comprehensive thermal safety:

```cpp
vcsel_config.enable_thermal_monitoring = true;
vcsel_config.max_operating_temp_c = 70.0f;  // Maximum safe temperature

// Thermal callback for monitoring
vcsel_controller->setThermalCallback([](bool thermal_active, float temp_c) {
    if (thermal_active) {
        std::cout << "THERMAL WARNING: " << temp_c << "C" << std::endl;
    }
});
```

**Thermal Protection Features:**
- Continuous temperature monitoring
- Automatic throttling at 70°C
- 5°C hysteresis for thermal recovery
- Emergency shutdown on critical overheating
- Thermal callback notifications

### Current Limiting

```cpp
// Safe operating limits
static constexpr uint16_t MAX_SAFE_CURRENT_MA = 200;     // Conservative
static constexpr uint16_t ABSOLUTE_MAX_CURRENT_MA = 446; // Hardware limit

// Current validation
bool valid = vcsel_controller->validateCurrent(current_ma);
```

**Current Safety:**
- 200mA safe operating limit per VCSEL
- 446mA absolute hardware maximum (AS1170 limit)
- Automatic current validation before activation
- Warning on currents exceeding safe limits
- Hard error on currents exceeding hardware limits

### Maximum ON Time Protection

```cpp
vcsel_config.max_on_time_ms = 5000;  // 5 second maximum continuous ON

// Check total ON time
uint64_t total_on_ms = vcsel_controller->getTotalOnTime();
```

**ON Time Protection:**
- Configurable maximum continuous ON time
- Default: 5 seconds
- Accumulated ON time tracking
- Automatic cooldown enforcement
- Statistics reset capability

## Configuration Parameters

### VCSELConfig Structure

```cpp
struct VCSELConfig {
    // VCSEL currents (mA)
    uint16_t vcsel1_current_ma = 200;      // VCSEL1 current (LED1)
    uint16_t vcsel2_current_ma = 200;      // VCSEL2 current (LED2)

    // Timing parameters (ms)
    uint32_t settle_time_ms = 50;          // Time between activations
    uint32_t capture_delay_ms = 10;        // Delay after activation
    uint32_t max_on_time_ms = 5000;        // Maximum continuous ON

    // Thermal protection
    bool enable_thermal_monitoring = true;  // Enable temp monitoring
    float max_operating_temp_c = 70.0f;    // Maximum safe temperature

    // Strobe mode (alternative to torch mode)
    bool use_strobe_mode = false;          // Use strobe vs continuous
    uint32_t strobe_duration_us = 1000;    // Strobe pulse duration
};
```

**Parameter Guidelines:**
- **settle_time_ms**: 50ms recommended for thermal stability
  - Range: 10-1000ms
  - Lower values = faster capture
  - Higher values = better thermal management

- **capture_delay_ms**: 10ms recommended for pattern stabilization
  - Range: 0-100ms
  - Ensures pattern fully projected before capture

- **max_on_time_ms**: 5000ms (5 seconds) recommended
  - Range: 100-10000ms
  - Prevents thermal damage from extended activation

## Status Monitoring

### Get Current Status

```cpp
auto status = vcsel_controller->getStatus();

std::cout << "VCSEL1 active: " << status.vcsel1_active << std::endl;
std::cout << "VCSEL2 active: " << status.vcsel2_active << std::endl;
std::cout << "Temperature: " << status.temperature_c << " C" << std::endl;
std::cout << "Thermal throttling: " << status.thermal_throttling << std::endl;
std::cout << "Total captures: " << status.total_captures << std::endl;
std::cout << "Total ON time: " << status.total_on_time_ms << " ms" << std::endl;

if (!status.error_message.empty()) {
    std::cerr << "Error: " << status.error_message << std::endl;
}
```

### Capture Metadata

```cpp
if (capture.is_valid) {
    // Timing information
    std::cout << "VCSEL1 activation: " << capture.vcsel1_activation_us << " us" << std::endl;
    std::cout << "VCSEL2 activation: " << capture.vcsel2_activation_us << " us" << std::endl;
    std::cout << "Total sequence: " << capture.total_sequence_us << " us" << std::endl;

    // Synchronization errors
    std::cout << "VCSEL1 sync error: " << capture.vcsel1_sync_error_ms << " ms" << std::endl;
    std::cout << "VCSEL2 sync error: " << capture.vcsel2_sync_error_ms << " ms" << std::endl;
    std::cout << "Ambient sync error: " << capture.ambient_sync_error_ms << " ms" << std::endl;

    // Temperature during capture
    std::cout << "Temperature: " << capture.temperature_c << " C" << std::endl;
}
```

## Testing

### Run Temporal Matching Test

```bash
# Build the project
cd /home/alessandro/unlook-standalone
./build.sh

# Run dual VCSEL temporal test
./build/examples/test_dual_vcsel_temporal 5

# Output:
# - Console statistics for each capture
# - Saved frames in temporal_captures/ directory
# - Pattern quality analysis
# - Thermal monitoring results
```

### Expected Output

```
=== AS1170 Dual VCSEL Temporal Matching Test ===
Number of captures: 5

Initializing camera system...
Camera system initialized successfully

Initializing dual VCSEL controller...
Dual VCSEL controller initialized successfully

=== VCSEL Configuration ===
VCSEL1 current: 200 mA
VCSEL2 current: 200 mA
Settle time: 50 ms
Capture delay: 10 ms
Max operating temp: 70.0 C

=== Starting Temporal Captures ===

--- Capture 1 of 5 ---
[Frame A] Activating VCSEL1 (LEFT camera side)
[Frame A] VCSEL1 capture complete, sync error: 0.234ms
[Frame B] Activating VCSEL2 (RIGHT camera side)
[Frame B] VCSEL2 capture complete, sync error: 0.187ms
[Frame C] Capturing ambient (no VCSEL)
[Frame C] Ambient capture complete, sync error: 0.156ms

=== Temporal Capture Statistics ===
Valid: YES
Temperature: 45.3 C

Timing:
  VCSEL1 activation: 62.4 ms
  VCSEL2 activation: 61.8 ms
  Total sequence: 412.7 ms

=== Pattern Quality Analysis ===
Pattern Visibility (std dev of intensity):
  VCSEL1 Left:  45.23
  VCSEL1 Right: 38.67
  VCSEL2 Left:  39.12
  VCSEL2 Right: 44.89
  Ambient Left: 18.34
  Ambient Right: 17.92

Pattern Enhancement Factor:
  VCSEL1: 2.28x
  VCSEL2: 2.31x

Pattern quality: GOOD (enhancement >1.2x)

Frames saved to: temporal_captures/capture_0_*.png

...

=== Overall Statistics ===
Successful captures: 5 / 5
Average capture time: 415.3 ms
Average temperature: 47.2 C
Maximum temperature: 49.8 C

=== VCSEL Status ===
Total ON time: 1847 ms
Thermal throttling: NO

Test complete!
```

## Troubleshooting

### Pattern Not Visible

**Problem**: Pattern enhancement factor < 1.2x

**Solutions**:
1. Check VCSEL alignment (should be 2cm from camera)
2. Increase current (up to 200mA recommended, 446mA max)
3. Verify scene distance (optimal: 0.3-2.0m)
4. Check ambient lighting (reduce if too bright)

### Thermal Throttling

**Problem**: `thermal_throttling: YES` in status

**Solutions**:
1. Increase settle time (try 100ms instead of 50ms)
2. Reduce current (try 150mA instead of 200mA)
3. Add longer cooldown between captures
4. Check ambient temperature (should be <30°C)

### High Sync Errors

**Problem**: Sync errors > 1ms

**Solutions**:
1. Verify hardware sync connections (XVS, XHS, MAS)
2. Check camera configuration (enableSync = true)
3. Reduce capture rate (allow more time between frames)
4. Check system load (reduce CPU usage)

### Capture Failures

**Problem**: `captureTemporalSequence()` returns false

**Solutions**:
1. Check error message: `status.error_message`
2. Verify camera system is initialized
3. Check VCSEL activation state
4. Verify I2C communication (bus 1, address 0x30)
5. Check GPIO permissions for GPIO 19

## Performance Optimization

### Reduce Capture Time

Current default: ~415ms per sequence

**Options:**
1. Reduce settle time (minimum: 10ms)
   ```cpp
   vcsel_config.settle_time_ms = 30;  // Faster, but higher thermal risk
   ```

2. Reduce capture delay (minimum: 0ms)
   ```cpp
   vcsel_config.capture_delay_ms = 5;  // Pattern may not be fully stable
   ```

3. Increase camera frame rate
   ```cpp
   cam_config.targetFps = 60.0;  // Faster capture, but requires more light
   ```

**Fastest configuration** (not recommended for production):
```cpp
vcsel_config.settle_time_ms = 10;      // Minimum settle
vcsel_config.capture_delay_ms = 0;      // No delay
cam_config.targetFps = 60.0;            // 60 FPS

// Expected time: ~200ms per sequence
// WARNING: High thermal risk, pattern instability possible
```

### Recommended Production Configuration

```cpp
vcsel_config.vcsel1_current_ma = 200;
vcsel_config.vcsel2_current_ma = 200;
vcsel_config.settle_time_ms = 50;       // Good thermal stability
vcsel_config.capture_delay_ms = 10;     // Pattern fully stable
vcsel_config.max_on_time_ms = 5000;
vcsel_config.enable_thermal_monitoring = true;
vcsel_config.max_operating_temp_c = 70.0f;

cam_config.targetFps = 30.0;            // Standard frame rate

// Expected time: ~415ms per sequence
// Excellent thermal management and pattern quality
```

## Integration with Stereo Matching

### Pattern Subtraction

```cpp
// Capture temporal sequence
hardware::AS1170DualVCSELController::TripleFrameCapture capture;
vcsel_controller->captureTemporalSequence(capture);

// Subtract ambient from pattern frames
cv::Mat pattern1_left, pattern1_right;
cv::subtract(capture.frame_vcsel1_left, capture.frame_ambient_left, pattern1_left);
cv::subtract(capture.frame_vcsel1_right, capture.frame_ambient_right, pattern1_right);

cv::Mat pattern2_left, pattern2_right;
cv::subtract(capture.frame_vcsel2_left, capture.frame_ambient_left, pattern2_left);
cv::subtract(capture.frame_vcsel2_right, capture.frame_ambient_right, pattern2_right);

// Now pattern1_* contains ONLY VCSEL1 dots
// And pattern2_* contains ONLY VCSEL2 dots
```

### Temporal Stereo Matching

```cpp
// Use pattern correspondence to constrain stereo matching
// VCSEL1 projects from left, so dots are brighter in left camera
// VCSEL2 projects from right, so dots are brighter in right camera

// This geometric constraint eliminates pattern ambiguity
StereoMatcher matcher;
cv::Mat disparity1 = matcher.match(pattern1_left, pattern1_right);
cv::Mat disparity2 = matcher.match(pattern2_left, pattern2_right);

// Fuse disparity maps for higher accuracy
cv::Mat fused_disparity;
cv::addWeighted(disparity1, 0.5, disparity2, 0.5, 0, fused_disparity);
```

## Future Enhancements

1. **Strobe Mode**: Use hardware strobe for faster activation
2. **Adaptive Timing**: Automatically adjust settle time based on temperature
3. **Multi-Pattern**: Support different patterns (15K vs 10K dots)
4. **GPU Acceleration**: Offload pattern subtraction to GPU
5. **Depth Fusion**: Combine temporal data with traditional stereo

## References

- AS1170 Datasheet: ams OSRAM LED Driver
- OSRAM BELAGO 1.1/1.2: VCSEL Dot Projector Specifications
- Unlook Hardware Guide: `/home/alessandro/unlook-standalone/CLAUDE.md`
- AS1170 Controller API: `/home/alessandro/unlook-standalone/include/unlook/hardware/AS1170Controller.hpp`

## License

Copyright (c) 2025 Unlook 3D Scanner Project
See LICENSE file for details.
