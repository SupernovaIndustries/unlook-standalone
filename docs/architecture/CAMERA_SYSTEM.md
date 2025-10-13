# Unlook 3D Scanner - Camera System Documentation

## Overview

The Unlook Camera System provides hardware-synchronized stereo capture for the 3D scanner, achieving <1ms synchronization precision with IMX296 global shutter sensors.

## Hardware Configuration

### Camera Mapping (CRITICAL - DO NOT CHANGE)
```cpp
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
Resolution: 1456x1088 SBGGR10
Baseline: 70.017mm (calibrated)
```

### Hardware Synchronization Pins
- **XVS (GPIO17)**: External Vertical Sync
- **XHS (GPIO27)**: External Horizontal Sync (camera system)  
- **MAS (GPIO22)**: Master/Slave selection
- **GND**: Common ground (connected)

## Building the System

### Prerequisites
```bash
# Install dependencies
sudo apt-get update
sudo apt-get install -y \
    cmake \
    build-essential \
    libopencv-dev \
    pkg-config \
    libboost-dev
```

### Build Instructions
```bash
# Quick build
./build.sh

# Build with options
./build.sh -c --clean    # Clean build
./build.sh -t Debug      # Debug build
./build.sh --build-libcamera  # Build libcamera-sync if needed
```

## API Usage

### Basic Initialization
```cpp
#include <unlook/camera/CameraSystem.hpp>

using namespace unlook::camera;

// Get singleton instance
CameraSystem& system = CameraSystem::getInstance();

// Configure
CameraConfig config;
config.width = 1456;
config.height = 1088;
config.targetFps = 30.0;
config.autoExposure = true;
config.enableSync = true;
config.baseline = 70.017;

// Initialize
if (!system.initialize(config)) {
    // Handle error
}
```

### Capturing Frames
```cpp
// Single frame capture
StereoFrame frame;
if (system.captureStereoFrame(frame, 1000)) {
    // Process frame
    cv::Mat leftImage = frame.leftImage;
    cv::Mat rightImage = frame.rightImage;
    double syncError = frame.syncErrorMs;
}

// Continuous capture with callback
system.setFrameCallback([](const StereoFrame& frame) {
    // Process each frame
});
system.startCapture();
```

### Camera Controls
```cpp
// Manual exposure control
system.setExposure(10000.0);  // microseconds
system.setGain(2.0);

// Auto-exposure
system.setAutoExposure(true);

// Swap cameras (LEFT <-> RIGHT)
system.swapCameras();
```

### Monitoring Synchronization
```cpp
// Get sync statistics
double avgError, maxError;
uint64_t errorCount;
system.getSyncStats(avgError, maxError, errorCount);

// Check status
CameraStatus status = system.getStatus();
if (status.isSynchronized) {
    std::cout << "Sync error: " << status.avgSyncErrorMs << " ms" << std::endl;
}
```

## Testing

### Run Camera Test
```bash
# Display live feed
./build/bin/camera_test -d

# Test synchronization precision
./build/bin/camera_test -s

# Custom configuration
./build/bin/camera_test -f 60 -e 5000 -g 2.0

# Load config file
./build/bin/camera_test -c config/camera_config.conf
```

### Expected Output
```
=== Testing Hardware Synchronization ===
Capturing 100 frames for sync analysis...
Frame 0: sync error = 0.234 ms
Frame 10: sync error = 0.189 ms
...
Synchronization Statistics:
  Min error: 0.156 ms
  Max error: 0.412 ms
  Avg error: 0.287 ms
  Result: PASS - Synchronization within 1ms tolerance
```

## Architecture

### Class Hierarchy
```
CameraSystem (Singleton)
├── CameraDevice (LEFT/MASTER)
├── CameraDevice (RIGHT/SLAVE)
├── CameraSynchronizer
│   ├── Hardware Sync (XVS/XHS)
│   └── Software Sync (fallback)
└── AutoExposure
    ├── Histogram Analysis
    └── Adjustment Algorithm
```

### Thread Safety
- All public methods are thread-safe
- Singleton pattern for shared access
- Lock-free atomic operations for statistics
- Separate capture thread for continuous streaming

### Memory Management
- RAII pattern for resource management
- Memory-mapped buffers for zero-copy
- Frame buffer pooling (4 buffers per camera)
- Automatic cleanup on destruction

## Performance

### Target Specifications
- **Resolution**: 1456x1088 @ 30 FPS
- **Sync Precision**: <1ms error
- **Latency**: <50ms frame delivery
- **CPU Usage**: <30% on Raspberry Pi CM4
- **Memory**: <100MB runtime overhead

### Optimization Techniques
- SBGGR10 optimized Bayer conversion
- NEON SIMD for ARM64 (when available)
- Zero-copy buffer handling
- Parallel frame processing

## Troubleshooting

### Camera Not Detected
```bash
# Check camera connections
libcamera-hello --list-cameras

# Verify I2C devices
i2cdetect -y 0
i2cdetect -y 1
```

### Synchronization Errors
1. Check GPIO connections (XVS, XHS, MAS)
2. Verify MAS pin soldering on camera board
3. Ensure common ground between cameras
4. Check cable lengths (<30cm recommended)

### Performance Issues
1. Reduce resolution or FPS
2. Disable auto-exposure
3. Check CPU temperature
4. Use Release build (`-t Release`)

## Configuration File Format

```conf
# camera_config.conf
camera.width = 1456
camera.height = 1088
camera.fps = 30.0
camera.exposure = 10000.0
camera.gain = 1.0
camera.auto_exposure = true
camera.hardware_sync = true
camera.sync_tolerance = 1.0
camera.baseline = 70.017
```

## libcamera-sync Integration

The system automatically detects and uses libcamera-sync:

1. **System Installation** (preferred): `/usr/local/lib/`
2. **Third-party Fallback**: `third-party/libcamera-sync-fix/`

### Building libcamera-sync
```bash
# If not installed system-wide
cd third-party/libcamera-sync-fix
meson setup build
ninja -C build
sudo ninja -C build install  # Optional system install
```

## Hardware Sync Validation

### Test Setup
1. Connect oscilloscope to XVS pin (GPIO17)
2. Run sync test: `./camera_test -s`
3. Verify 30Hz pulse train
4. Measure pulse width (~10μs)

### Expected Waveform
```
XVS: ┌─┐     ┌─┐     ┌─┐
     │ │     │ │     │ │
     ┘ └─────┘ └─────┘ └
     |←  33ms →|
```

## Future Enhancements

- [ ] 10-bit depth support (currently 8-bit)
- [ ] Hardware trigger input support
- [ ] Multi-camera scaling (>2 cameras)
- [ ] ROI (Region of Interest) support
- [ ] Hardware ISP integration
- [ ] MIPI CSI-2 direct access

## Support

For issues or questions:
1. Check CAMERA_SYSTEM.md (this file)
2. Review test output logs
3. Verify hardware connections
4. Check system dmesg for errors