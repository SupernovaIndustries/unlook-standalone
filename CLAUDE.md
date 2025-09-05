# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Unlook** is a professional modular opensource 3D scanner with target precision **0.005mm**, designed for industrial, educational and professional applications. The system combines stereovision, structured light VCSEL, and FPGA acceleration to achieve industrial-level performance at accessible costs.

### Target Applications
- Industrial quality control and inspection
- Educational 3D scanning projects  
- Professional prototyping and reverse engineering
- Research applications requiring high precision measurements

## Language Strategy - C++ FIRST

**C++ (99.9%): ALL CORE SYSTEMS**
- Camera management and hardware control
- High-precision stereo calibration
- Stereo matching and depth calculation
- Real-time processing pipeline
- I/O, configuration, threading
- Hardware synchronization
- Optimized memory management
- GUI (Qt C++)

**Python (0.1%): ABSOLUTE EMERGENCIES ONLY**
- Ultra-complex debug tools (TEMPORARY)
- Algorithm prototyping (before porting to C++)
- **RULE**: If using Python → plan IMMEDIATE C++ migration

**ZERO PYTHON in final production system**

### Technical Specifications
- **Target Precision**: 0.005mm repeatability
- **Architecture**: Modular with interchangeable components
- **License**: Completely opensource
- **Language**: **C++17/20 EXCLUSIVELY**
- **Paradigm**: Professional C++ Object-Oriented Programming
- **Deployment**: Custom OS based on Raspbian
- **Runtime**: **ZERO Python dependencies**

## Hardware Configuration

### Camera System
- **Sensors**: 2x Global Shutter Camera Raspberry Pi IMX296 (1456x1088 SBGGR10)
- **Camera Mapping (Scanner POV)**:
  - **Camera 0**: RIGHT camera (destra guardando dove guarda lo scanner)
  - **Camera 1**: LEFT camera (sinistra guardando dove guarda lo scanner)
- **Hardware Synchronization**: 
  - Camera 1 = MASTER (LEFT, /base/soc/i2c0mux/i2c@1/imx296@1a)
  - Camera 0 = SLAVE (RIGHT, /base/soc/i2c0mux/i2c@0/imx296@1a)
  - XVS (External Vertical Sync) - connected
  - XHS (External Horizontal Sync) - connected
  - GND - connected
  - MAS (Master/Slave) - soldered on camera sink
- **Optics**: 6mm focal length lens
- **Baseline**: 70.017mm (calibrated, from calib_boofcv_test3.yaml)
- **Mounting**: Rigid mount for thermal stability

### Calibration Targets
- **Standard Checkerboard**: 7x10 pattern, 24mm squares
- **ChArUco Board #1**: 7x10 pattern, 24mm squares, ArUco 17mm, DICT_4X4_250
- **ChArUco Board #2**: 7x10 pattern, 24mm squares, ArUco 17mm, DICT_4X4_250

### LED Controller System
- **Driver**: AS1170 LED driver
- **Communication**: I2C bus 4, device ID 0x30
- **Strobe Control**: GPIO 27
- **LED1**: ams OSRAM BELAGO1.1 VCSEL dot projector
- **LED2**: Flood illuminator (current)
- **Planned Upgrade**: BELAGO1.2 (15k points vs 10k current)

## Development Commands

### System Dependencies Installation
```bash
# Install all dependencies using build script
./build.sh --install-deps
```

### Build System (Primary Method)
```bash
# Build with default settings (Release, with examples and GUI)
./build.sh

# Build with specific options
./build.sh -t Debug --tests --clean    # Debug build with tests, clean first
./build.sh --validate                  # Validate dependencies only
./build.sh --package                   # Create installation package

# Cross-compile for Raspberry Pi
./build.sh -x rpi4 -j 4
```

### Manual Build (Alternative)
```bash
# Create build directory and configure
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Key executables will be in build/bin/:
# - unlook_calibration_gui      # Main calibration GUI
# - stereo_camera_example       # Basic stereo example
# - test_boofcv_calibration     # BoofCV calibration test
```

### Running Applications
```bash
# Main calibration GUI
./build/bin/unlook_calibration_gui

# Stereo camera example
./build/bin/stereo_camera_example

# Camera hardware tests (using libcamera-sync)
./test_camera_access
libcamera-hello --list-cameras  # Uses custom libcamera-sync system installation
```

### Testing and Validation
```bash
# Hardware validation tests
./test_cameras_basic.sh
./configure_camera_sync.sh

# Depth processing tests
./test_depth_gui.sh

# Comprehensive validation
./FINAL_VERIFICATION_TEST.sh
```

## Code Architecture and Structure

### C++ Core Implementation
The system is implemented using professional C++ with clear module separation:

```cpp
namespace unlook {
    namespace core { /* Base classes, logging, configuration */ }
    namespace camera { /* Camera management and stereo system */ }
    namespace calibration { /* Complete calibration pipeline */ }
    namespace stereo { /* Stereo matching and depth calculation */ }
    namespace hardware { /* I2C, GPIO, hardware interfacing */ }
    namespace math { /* Mathematical utilities and algorithms */ }
    namespace io { /* File I/O, data serialization */ }
    namespace utils { /* General utilities, debugging, profiling */ }
    namespace gui { /* Qt-based calibration interface */ }
    namespace realtime { /* Real-time processing pipeline */ }
}
```

### Project File Structure
```
unlook-standalone/
├── CMakeLists.txt              # Main CMake configuration
├── build.sh                    # Comprehensive build script
├── src/
│   ├── unlook.h                # Main API header
│   ├── core/                   # Core functionality
│   ├── camera/                 # Camera system integration
│   ├── stereo/                 # Depth processing
│   ├── calibration/            # Calibration pipeline
│   ├── hardware/               # I2C, GPIO, LED control
│   ├── gui/                    # Qt5 interface
│   └── ...
├── third-party/                # External dependencies
│   └── libcamera-sync-fix/     # Custom camera synchronization
├── calibration/                # Camera calibration files
│   └── calib_boofcv_test3.yaml # Current calibration (70.017mm baseline)
├── tests/                      # Comprehensive test suite
├── examples/                   # Example applications
└── cmake/                      # CMake modules
```

### Key Integration Points

#### libcamera-sync Integration (SYSTEM INSTALLED) ✅
The system has **completely replaced** standard libcamera with custom libcamera-sync:
- **Installation Status**: System-wide installation completed, standard libcamera removed
- **Location**: Installed at `/usr/local/` (system library paths)
- **Source Path**: Built from `/home/alessandro/libcamera-sync-fix/`
- **Hardware Sync Support**: Full XVS/XHS synchronization for IMX296 cameras
- **Integration**: Direct system library usage (no wrapper needed)
- **Performance**: <1ms synchronization precision achieved

#### API Design Pattern
All GUI components use the same API classes that external applications will use:
```cpp
#include <unlook/unlook.h>

// Initialize scanner
unlook::initialize(unlook::core::LogLevel::INFO);

// Use quickstart for common scenarios
auto camera_system = unlook::quickstart::setupStereoCapture();
auto depth_processor = unlook::quickstart::setupDepthProcessing();
auto calibration = unlook::quickstart::setupCalibration();
```

### Build System Features
- **Modern CMake 3.16+** with target-based configuration
- **Cross-compilation support** for ARM64/Raspberry Pi
- **Automatic dependency management** with system/third-party fallback
- **Professional build options**: Debug, Release, with sanitizers and LTO
- **Comprehensive testing** framework with Google Test
- **Package generation**: DEB, RPM, TGZ formats

## Current Implementation Status

### Phase 1 Status (FOUNDATION READY)
**Systems Ready:**
- ✅ libcamera-sync system installation completed (replaces standard libcamera)
- ✅ Hardware configuration documented (IMX296 sensors, baseline measurement)
- ✅ Build system with comprehensive CMake architecture
- ✅ API structure designed and headers created
- ✅ Qt5 GUI framework architecture planned

**Systems to Implement:**
- 🔄 Camera initialization and hardware synchronization validation
- 🔄 Stereo capture pipeline with IMX296 sensors  
- 🔄 Qt-based calibration GUI with live preview
- 🔄 Basic stereo matching with OpenCV SGBM
- 🔄 BoofCV integration for high-precision calibration

**Phase 1 Objectives:**
1. **Camera Management**: Hardware sync with <1ms precision
2. **Stereo Calibration**: High-precision using libcbdetect/BoofCV  
3. **Depth Map Generation**: Optimized for baseline measurement
4. **Qt GUI**: Complete calibration interface
5. **Validation Framework**: Precision assessment tools

### Calibration Integration (BASELINE AVAILABLE)
- **Current Calibration**: `calibration/calib_boofcv_test3.yaml` (BoofCV precision, 70.017mm baseline)
- **Status**: Working calibration available for initial development
- **BoofCV Support**: High-precision calibration via JNI wrapper (to implement)
- **Multiple Patterns**: Standard checkerboard and ChArUco boards support
- **Improvement Target**: Reduce RMS from 0.24px to <0.2px

### Camera System Status (TO BE VALIDATED)
```cpp
// Camera configuration (libcamera-sync installed, needs validation)
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
Resolution: 1456x1088 SBGGR10
Hardware sync: XVS/XHS enabled, MAS pin configured (needs testing)
```

## Development Guidelines & Success Criteria

### Code Standards (C++ EXCLUSIVE)
- **C++17/20 standards** exclusively
- **Object-Oriented Programming** professional C++
- **Thread-safe implementations** using std::mutex, std::atomic
- **Comprehensive error handling** C++ exceptions with context
- **Performance-optimized** for ARM64/Raspberry Pi
- **Zero Python runtime dependencies**

### Phase 1 Success Criteria (TO BE ACHIEVED)
- 🔄 **Camera Management**: Hardware sync <1ms precision (libcamera-sync installed)
- 🔄 **Stereo Calibration**: RMS < 0.2 pixel target  
- 🔄 **Rectification System**: Proper epipolar alignment
- 🔄 **GUI Implementation**: Complete calibration interface with visualization
- 🔄 **Depth Precision**: Target ≤ 0.005mm (hardware limited to 0.008mm at 100mm)
- 🔄 **Performance**: VGA stereo processing >20 FPS on CM4
- 🔄 **Code Quality**: 100% C++, zero Python runtime deps

### Memory Optimization (C++ Performance)
- **CM4 Constraints**: Optimized for 8GB constraints (CM5 16GB recommended)
- **Memory Pooling**: C++ for frequent allocations
- **NEON SIMD**: ARM64-specific optimizations
- **Multi-threading**: C++ stereo algorithms
- **Image Pyramid**: C++ for multi-scale processing

## Testing and Development Workflow

### Test Files in Repository
The repository contains several test files for hardware validation:
- `test_camera_sync.cpp` - Camera synchronization testing
- `test_hardware_sync_new.cpp` - Hardware sync validation
- `test_sbggr10_conversion.cpp` - SBGGR10 format handling
- `test_sync_precision.cpp` - Precision timing validation

### Development Workflow
```bash
# 1. Setup and validation
./validate_build_system.sh    # Verify system readiness
./build.sh --deps             # Build dependencies if needed

# 2. Development build
./build.sh -t Debug --clean   # Debug build with clean

# 3. Testing
./build.sh --tests            # Build with test suite
make test                     # Run all tests

# 4. Hardware validation (requires actual hardware)
./build/bin/test_camera_sync  # Camera sync testing
./test_synchronized_capture.sh # Full sync validation
```

### Agent Usage Guidelines
- **Always use specialized agents**: stereo-calibration-specialist, camera-sync-manager, point-cloud-processor, etc.
- **Never build simultaneously with multiple agents**: Risk of conflicts
- **Use libcamera-sync version**: System-installed at `/usr/local/` (replaces standard libcamera)
- **Respect camera mapping**: Camera 1=LEFT/MASTER, Camera 0=RIGHT/SLAVE

### Critical Hardware Info
```cpp
// HARDWARE CONFIGURATION (TO BE VALIDATED)
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE  
Resolution: 1456x1088 SBGGR10
Baseline: 70.017mm (from calib_boofcv_test3.yaml)
Hardware Sync: XVS/XHS enabled, MAS pin configured (needs testing)

// AS1170 LED SYSTEM (TO BE IMPLEMENTED)
I2C Bus: 4, Address: 0x30, Strobe GPIO: 27
LED1 (VCSEL): For structured light projection
LED2 (Flood): For stereo calibration illumination
Timing: Synchronized with camera capture
Safety: Temperature monitoring, emergency shutdown required
```

## AS1170 LED System Integration (TO BE IMPLEMENTED)

### **Phase 1 LED Configuration Plan**
- **LED1 (VCSEL Projector)**: For structured light projection (Phase 2)
- **LED2 (Flood Illuminator)**: For stereo calibration illumination
- **Hardware**: I2C bus 4, address 0x30, GPIO 27 strobe control
- **Timing**: LED-camera synchronization system
- **Safety**: Temperature monitoring, emergency shutdown, thermal protection

### **AS1170 Implementation Goals**
- **AS1170Controller**: Hardware I2C/GPIO control implementation
- **TimingController**: Microsecond precision timing system
- **LEDSyncManager**: Camera-LED synchronization
- **GUI Integration**: LED controls in calibration interface
- **Safety Systems**: Temperature monitoring and emergency shutdown

### **Target Performance**
- **Timing Precision**: ±50μs LED activation accuracy
- **Camera Sync**: Perfect LED-camera timing synchronization
- **Memory Usage**: <1MB overhead for LED system
- **Safety**: <5ms emergency shutdown response time
- **GUI**: Real-time LED status and control interface

## Common Development Tasks

### Building and Running
```bash
# Quick development cycle
./build.sh && ./build/bin/unlook_scanner

# Debug with specific component
./build.sh -t Debug --verbose
gdb ./build/bin/unlook_scanner

# Cross-compile for deployment
./build.sh --cross rpi4 --package
```

### Working with Calibration
```bash
# Load existing calibration
# File: calibration/calib_boofcv_test3.yaml
# Contains: 70.017mm baseline, RMS 0.24px

# Validate calibration quality in code:
auto calibManager = std::make_shared<CalibrationManager>();
calibManager->loadCalibration("calibration/calib_boofcv_test3.yaml");
```

### Hardware Testing Commands
```bash
# Camera detection
libcamera-hello --list-cameras

# Manual camera test
./test_camera_sync

# Timing precision validation
./test_sync_precision

# Hardware sync validation  
./test_synchronized_capture.sh
```

## Tips for Development

### Camera System
- Always use the libcamera-sync C++ API, not command-line tools
- Camera mapping is fixed: Camera 1=LEFT/MASTER, Camera 0=RIGHT/SLAVE
- Hardware sync requires XVS/XHS GPIO connections to work properly
- Mock mode available for development without hardware

### Build System
- Use `./build.sh` instead of direct CMake for dependency management
- Cross-compilation toolchain automatically detected for ARM64
- Clean builds recommended when switching between Debug/Release
- Build system validates dependencies automatically

### Code Organization
- All public APIs go through the main `unlook.h` header
- Use namespace organization: `unlook::camera`, `unlook::stereo`, etc.
- Thread-safe singleton pattern for hardware resources
- RAII principles throughout for resource management

### Testing
- Unit tests use Google Test framework
- Hardware tests require actual IMX296 cameras
- Performance tests validate real-time requirements
- Memory tests use AddressSanitizer and Valgrind