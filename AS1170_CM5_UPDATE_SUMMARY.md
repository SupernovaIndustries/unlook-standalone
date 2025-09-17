# AS1170 CM5 Configuration Update Summary

## Overview
Updated AS1170 LED controller configuration for CM5 compatibility by changing GPIO strobe pin and I2C bus to avoid conflicts with available hardware.

## Configuration Changes

### Previous Configuration (CM4 Focused)
- I2C Bus: 4
- I2C Address: 0x30
- GPIO Strobe: 27

### New Configuration (CM5 Optimized)
- I2C Bus: 1 (available on CM5)
- I2C Address: 0x30 (unchanged)
- GPIO Strobe: 19 (available on CM5)

## Files Updated

### Core Hardware Files
- `include/unlook/hardware/AS1170Controller.hpp` - Updated default config
- `include/unlook/hardware/LEDSyncManager.hpp` - Updated GPIO reference
- `src/hardware/CMakeLists.txt` - Updated compile definitions

### Core Types
- `src/core/types.h` - Updated LEDConfig defaults

### Example Files
- `examples/vcsel_depth_capture_example.cpp` - Updated configuration

### Test Files
- `test_vcsel_integration.cpp` - Updated hardware check messages
- `test_sync_precision.cpp` - Added clarity on GPIO usage
- `test_synchronized_capture.sh` - Added AS1170 strobe reference

### Documentation Files
- `CLAUDE.md` - Updated hardware configuration section
- `README.md` - Updated GPIO connections information
- `VCSEL_INTEGRATION_SUMMARY.md` - Updated configuration
- `PROJECT_GUIDELINES.md` - Updated LED system config
- `docs/wiki/CM5-Migration.md` - Updated migration guide
- `CAMERA_SYSTEM.md` - Added clarity on camera vs LED GPIO
- `CAMERA_SYNC_SOLUTION.md` - Added AS1170 strobe reference

### Camera System Files (Clarified)
- `src/camera/camera_system.cpp` - Added comment clarifying XHS GPIO 27 is for camera sync
- `src/camera/CameraSynchronizer.cpp` - Added comment clarifying GPIO usage
- `include/unlook/camera/HardwareSyncManager.hpp` - Added camera system clarification

## GPIO Usage Summary

### Camera Synchronization (Unchanged)
- XVS: GPIO 17 (External Vertical Sync)
- XHS: GPIO 27 (External Horizontal Sync)
- MAS: GPIO 22 (Master/Slave selection)

### LED Control System (Updated)
- AS1170 Strobe: GPIO 19 (LED strobe control)

## I2C Usage Summary

### Available CM5 I2C Buses
- i2c-0, i2c-1, i2c-10, i2c-13, i2c-14

### Selected Configuration
- AS1170 LED Controller: I2C bus 1, address 0x30

## Build System Updates
- CMake definitions updated to reflect new GPIO and I2C configuration
- All build targets will now use the new configuration by default

## Compatibility Notes
- Camera synchronization system unchanged (uses GPIO 17, 27, 22)
- LED control system updated to use GPIO 19 for strobe
- I2C bus 1 is commonly available on CM5
- Configuration maintains backward compatibility through configurable parameters