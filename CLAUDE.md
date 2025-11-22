# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Critical Constraints

**STEREO MATCHING**: Never use SGBM - we have VCSEL structured light projectors. Always use Census-based stereo matching in all its forms (AD-Census, Census transform, etc.).

**LANGUAGE**: C++17/20 exclusively. Zero Python in production. No exceptions.

**BUILD COMMAND**: Always use `./build.sh --cross rpi5 -j 4` for building.

## Quick Reference

```bash
# Build (ALWAYS use this command)
./build.sh --cross rpi5 -j 4

# Run the GUI
unlook                              # System-installed command
# Or with library path:
LD_LIBRARY_PATH=build/src:third-party/libcamera-sync-fix/build/src/libcamera:third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH ./build/src/gui/unlook_scanner

# Hardware tests
./build/test_hardware_sync_new
./validate_build_system.sh
```

## Dual Repository Workflow

| Repository | Purpose | Contains |
|------------|---------|----------|
| `unlook-standalone` (THIS) | Development | Everything: tests, docs, examples, .github |
| `unlook` | Production/Investors | Clean: src/, third-party/, CMake, README.md only |

**Workflow**: Develop here → Cherry-pick stable features to `unlook` production repo

## Project Overview

**Unlook** is a professional modular opensource 3D scanner with target precision **0.005mm**, designed for industrial, educational and professional applications. The system combines stereovision, structured light VCSEL, and ARM64 acceleration.

- **Target Precision**: 0.005mm repeatability
- **Platform**: Raspberry Pi CM5 (16GB RAM recommended)
- **Cameras**: 2x IMX296 Global Shutter (1456x1088 SBGGR10)
- **License**: MIT

## Hardware Configuration

### Camera Mapping (DO NOT CHANGE)
```
Camera 1 = LEFT/MASTER  (/base/soc/i2c0mux/i2c@1/imx296@1a)
Camera 0 = RIGHT/SLAVE  (/base/soc/i2c0mux/i2c@0/imx296@1a)
Baseline: 70.017mm (from /home/alessandro/unlook_calib/default.yaml)
Sync: XVS/XHS hardware sync, <1ms precision
```

### AS1170 LED Controller
- **I2C**: Bus 1, Address 0x30, Strobe GPIO 19
- **LED1**: VCSEL dot projector (structured light)
- **LED2**: Flood illuminator
- **Classes**: `AS1170Controller`, `AS1170DualVCSELController`

## Architecture

### Namespace Structure
```cpp
namespace unlook {
    namespace core     {}  // Logger, Configuration, Exception
    namespace api      {}  // UnlookScanner, CameraSystem, DepthProcessor, CalibrationManager
    namespace camera   {}  // CameraDevice, SynchronizedCameraSystem, HardwareSyncManager, AutoExposure
    namespace stereo   {}  // StereoMatcher, TemporalStereoProcessor, ProgressiveStereoMatcher
    namespace calibration {} // CalibrationManager, CalibrationValidator, StereoRectifier
    namespace hardware {}  // AS1170Controller, AS1170DualVCSELController, LEDSyncManager
    namespace pointcloud {} // PointCloudProcessor (Open3D integration)
    namespace mesh     {}  // MeshOptimizer, MeshValidator, IndustrialMeshExporter
    namespace face     {}  // FacialRecognitionSystem, LivenessDetector
    namespace realtime {}  // RealtimePipeline, NEON-optimized Bayer processing
    namespace gui      {}  // MainWindow, CameraPreviewWidget, DepthTestWidget
}
```

### Key Source Directories
- `src/api/` - Public API (UnlookScanner is the main entry point)
- `src/camera/` - libcamera-sync integration, hardware sync
- `src/stereo/` - Census-based stereo matching, temporal VCSEL processing
- `src/hardware/` - AS1170 LED controller, I2C/GPIO
- `src/gui/` - Qt5 fullscreen touch interface
- `/home/alessandro/unlook_calib/default.yaml` - Default stereo calibration (symlink)

## Key Patterns

### API Usage
```cpp
#include <unlook/unlook.h>

unlook::api::UnlookScanner scanner;
scanner.initialize(unlook::core::ScannerMode::STANDALONE);

auto* camera = scanner.getCameraSystem();
auto* depth = scanner.getDepthProcessor();
```

### Temporal Stereo with VCSEL
- `TemporalStereoProcessor` integrates dual VCSEL with Census stereo matching
- Pattern isolation extracts VCSEL dots from ambient light
- Multi-frame temporal averaging for noise reduction

### ARM64 NEON Optimization
- `BayerNEON.cpp` processes 16 pixels at once
- VGA <5ms, HD <15ms on CM5

## Runtime Library Paths

If not system-installed, set before running:
```bash
export LD_LIBRARY_PATH=build/src:third-party/libcamera-sync-fix/build/src/libcamera:third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH
```

## Testing

- Hardware tests require actual IMX296 cameras
- **GUI testing**: User validates manually with `unlook` command
- **DO NOT** use QT_QPA_PLATFORM=offscreen for GUI tests