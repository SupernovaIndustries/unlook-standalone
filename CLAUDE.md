# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Quick Reference

### Essential Commands
```bash
# Build the project
./build.sh

# Run the GUI (system-installed command)
unlook

# Legacy run method (if needed)
LD_LIBRARY_PATH=build/src:third-party/libcamera-sync-fix/build/src/libcamera:third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH ./build/src/gui/unlook_scanner

# Run hardware tests
./build/test_hardware_sync_new

# Validate build system
./validate_build_system.sh

# Cross-compile for Raspberry Pi CM5
./build.sh --cross rpi5 -j 4
```

### Key Executables (after build)
- `unlook` - System-installed command (runs from anywhere)
- `build/src/gui/unlook_scanner` - Main GUI application
- `build/examples/camera_example` - Basic camera capture
- `build/test_hardware_sync_new` - Hardware sync validation

## Project Overview

**Unlook** is a professional modular opensource 3D scanner with target precision **0.005mm**, designed for industrial, educational and professional applications. The system combines stereovision, structured light VCSEL, and ARM64 acceleration to achieve industrial-level performance at accessible costs.

### Technical Specifications
- **Target Precision**: 0.005mm repeatability
- **Architecture**: Modular with interchangeable components
- **License**: MIT - Completely opensource
- **Language**: **C++17/20 EXCLUSIVELY**
- **Paradigm**: Professional C++ Object-Oriented Programming
- **Deployment**: Custom OS based on Raspbian
- **Runtime**: **ZERO Python dependencies**

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

### LED Controller System (IMPLEMENTED)
- **Driver**: AS1170 LED driver (AS1170Controller.cpp)
- **Communication**: I2C bus 1, device ID 0x30
- **Strobe Control**: GPIO 19
- **LED1**: ams OSRAM BELAGO1.1 VCSEL dot projector
- **LED2**: Flood illuminator
- **Dual VCSEL Support**: AS1170DualVCSELController for advanced structured light
- **Timing**: Camera-LED synchronization with microsecond precision
- **Safety**: Temperature monitoring, emergency shutdown, thermal protection

## Code Architecture and Structure

### C++ Core Implementation
The system is implemented using professional C++ with clear module separation:

```cpp
namespace unlook {
    namespace core {          // Base classes, logging, configuration
        class Logger;             // Singleton logging system
        class Configuration;      // System configuration management
        class Exception;          // Custom exception hierarchy
    }
    namespace api {           // Main API classes (public interface)
        class UnlookScanner;      // Primary scanner interface
        class CameraSystem;       // Synchronized camera control
        class DepthProcessor;     // Stereo matching and depth
        class CalibrationManager; // Calibration loading/validation
    }
    namespace camera {        // Camera management and sync
        class CameraDevice;                  // Individual camera control
        class SynchronizedCameraSystem;      // Hardware sync management
        class HardwareSyncManager;           // XVS/XHS sync coordination
        class LibcameraSyncDevice;           // libcamera-sync integration
        class AutoExposure;                  // Auto-exposure control
    }
    namespace stereo {        // Stereo processing algorithms
        class StereoMatcher;                 // Base stereo matching
        class SGBMStereoMatcher;             // OpenCV SGBM implementation
        class TemporalStereoProcessor;       // VCSEL-based temporal processing
        class ProgressiveStereoMatcher;      // Multi-scale progressive matching
        class DepthProcessor;                // Depth map generation
    }
    namespace calibration {   // High-precision calibration
        class CalibrationManager;            // Calibration loading/management
        class CalibrationValidator;          // Calibration quality assessment
        class StereoRectifier;               // Epipolar rectification
        class BoofCVWrapper;                 // BoofCV integration (JNI)
    }
    namespace hardware {      // I2C, GPIO, hardware interfacing
        class AS1170Controller;              // LED driver control (implemented)
        class AS1170DualVCSELController;     // Dual VCSEL control (implemented)
        class LEDSyncManager;                // Camera-LED synchronization
        class StructuredLightSystem;         // Structured light projection
    }
    namespace pointcloud {    // Point cloud processing
        class PointCloudProcessor;           // Open3D integration, filtering
    }
    namespace mesh {          // Mesh generation and optimization
        class MeshOptimizer;                 // Mesh decimation, smoothing
        class MeshValidator;                 // Mesh quality validation
        class IndustrialMeshExporter;        // PLY/OBJ/STL export
    }
    namespace face {          // Facial recognition (banking-grade)
        class FacialRecognitionSystem;       // Complete face recognition
        class FaceDetector;                  // Face detection
        class LandmarkExtractor;             // Facial landmark extraction
        class Face3DReconstructor;           // 3D face reconstruction
        class LivenessDetector;              // Anti-spoofing liveness
        class BankingComplianceValidator;    // Banking compliance checks
    }
    namespace realtime {      // Real-time processing pipeline
        class RealtimePipeline;              // Real-time orchestration
        void unpackSBGGR10_NEON_Optimized(); // ARM64 NEON Bayer processing
    }
    namespace gui {           // Qt-based touch interface
        class MainWindow;                    // Main fullscreen window
        class CameraPreviewWidget;           // Dual camera preview
        class DepthTestWidget;               // Depth processing UI
        class OptionsWidget;                 // System configuration
    }
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
│   ├── api/                    # Public API implementation
│   ├── camera/                 # Camera system integration
│   ├── stereo/                 # Depth processing
│   ├── calibration/            # Calibration pipeline
│   ├── hardware/               # I2C, GPIO, LED control (implemented)
│   ├── pointcloud/             # Point cloud processing (Open3D)
│   ├── mesh/                   # Mesh generation and optimization
│   ├── face/                   # Face recognition system (banking-grade)
│   ├── realtime/               # Real-time pipeline with NEON optimizations
│   ├── gui/                    # Qt5 interface
│   └── main.cpp                # Application entry point
├── third-party/                # External dependencies
│   └── libcamera-sync-fix/     # Custom camera synchronization
├── calibration/                # Camera calibration files
│   └── calib_boofcv_test3.yaml # Current calibration (70.017mm baseline)
├── tests/                      # Comprehensive test suite
├── examples/                   # Example applications
└── cmake/                      # CMake modules
```

## Key Architectural Patterns

### 1. Temporal Stereo Processing with VCSEL
The system implements advanced temporal stereo processing for VCSEL-based structured light:
- **TemporalStereoProcessor**: Integrates AS1170 dual VCSEL with stereo matching
- **Pattern Isolation**: Extracts VCSEL dot patterns from ambient illumination
- **Multi-frame Processing**: Temporal averaging for noise reduction
- **VCSEL-optimized SGBM**: Custom parameters for structured light stereo

### 2. Real-time Bayer Processing (ARM64 NEON)
High-performance SBGGR10 unpacking with ARM64 NEON vectorization:
- **BayerNEON.cpp**: Processes 16 pixels at once using NEON intrinsics
- **Performance**: VGA <5ms, HD <15ms on Raspberry Pi CM5
- **Pipeline Integration**: Zero-copy processing in RealtimePipeline

### 3. Banking-Grade Face Recognition
Complete facial recognition system with banking compliance:
- **FacialRecognitionSystem**: Integrates detection, landmarks, 3D reconstruction
- **Liveness Detection**: Anti-spoofing with multiple tests
- **Banking Compliance**: BankingComplianceValidator ensures regulatory requirements
- **Supernova ML Integration**: Cloud-based ML model management via SupernovaMLClient
- **3D Face Reconstruction**: Stereo-based 3D facial modeling
- **Performance Optimization**: ARM64-optimized for real-time processing

### 4. Point Cloud and Mesh Generation
Industrial-grade 3D reconstruction pipeline:
- **Open3D Integration**: Professional point cloud processing
- **Mesh Optimization**: Laplacian, Taubin, bilateral smoothing algorithms
- **Quality Validation**: Mesh quality assessment for manufacturing
- **Export Formats**: PLY, OBJ, STL with manufacturing precision

### 5. Shared Library API Design
All GUI components use the same API classes that external applications will use:
```cpp
#include <unlook/unlook.h>

// Initialize scanner
unlook::api::UnlookScanner scanner;
scanner.initialize(unlook::core::ScannerMode::STANDALONE);

// Use subsystems
auto* camera = scanner.getCameraSystem();
auto* depth = scanner.getDepthProcessor();
```

## Development Commands

### Build System
```bash
# Build with default settings (Release, with examples and GUI)
./build.sh

# Build with specific options
./build.sh -t Debug --clean         # Debug build with clean first
./build.sh --validate              # Validate build system only
./build.sh --package               # Create installation package

# Cross-compile for Raspberry Pi
./build.sh --cross rpi4 -j 4
./build.sh --cross rpi5 -j 4       # For CM5 with Cortex-A76 optimizations
```

### Running Applications
```bash
# Main scanner GUI (system-installed command)
unlook

# With library path (if not installed)
export LD_LIBRARY_PATH=build/src:third-party/libcamera-sync-fix/build/src/libcamera:third-party/libcamera-sync-fix/build/src/libcamera/base:$LD_LIBRARY_PATH
./build/src/gui/unlook_scanner

# Hardware sync validation
./build/test_hardware_sync_new
```

### Testing
```bash
# Build system validation
./validate_build_system.sh

# Hardware tests require actual IMX296 cameras
./build/test_hardware_sync_new
./test_synchronized_capture.sh
```

## Implementation Status

### Phase 1 - PRODUCTION READY ✅
- ✅ libcamera-sync system installation (replaces standard libcamera)
- ✅ Hardware synchronization system (<1ms precision)
- ✅ Camera management with AutoExposure
- ✅ Qt5 fullscreen touch interface
- ✅ OpenCV SGBM stereo matching
- ✅ Calibration loading and validation
- ✅ AS1170 LED controller implementation
- ✅ Dual VCSEL structured light system
- ✅ Temporal stereo processing
- ✅ Real-time Bayer processing with NEON
- ✅ Build system with cross-compilation

### Phase 2 - ADVANCED FEATURES ✅
- ✅ Point cloud processing (Open3D integration)
- ✅ Mesh generation and optimization
- ✅ Industrial mesh export (PLY/OBJ/STL)
- ✅ Face recognition system (banking-grade)
- ✅ Liveness detection and anti-spoofing
- ✅ Supernova ML cloud integration
- ✅ Banking compliance validation
- ✅ ARM64 performance optimizations

### Phase 3 - FUTURE ENHANCEMENTS 🔮
- 🔄 BoofCV integration for high-precision calibration
- 🔄 Advanced calibration validation tools
- 🔄 Multi-sensor fusion algorithms
- 🔄 Enhanced VCSEL pattern analysis

## Critical Hardware Info

```cpp
// HARDWARE CONFIGURATION
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
Resolution: 1456x1088 SBGGR10
Baseline: 70.017mm (from calibration/calib_boofcv_test3.yaml)
Hardware Sync: XVS/XHS enabled, MAS pin configured

// LIBRARY PATHS (CRITICAL FOR RUNTIME)
LD_LIBRARY_PATH must include:
- build/src (for main libraries)
- third-party/libcamera-sync-fix/build/src/libcamera (libcamera.so.0.5.1)
- third-party/libcamera-sync-fix/build/src/libcamera/base (libcamera-base.so.0.5.1)

// AS1170 LED SYSTEM (IMPLEMENTED)
I2C Bus: 1, Address: 0x30, Strobe GPIO: 19
LED1 (VCSEL): Structured light projection (AS1170Controller)
LED2 (Flood): Stereo calibration illumination
Dual VCSEL: AS1170DualVCSELController for temporal processing
Timing: Microsecond-precision camera-LED sync
Safety: Temperature monitoring, emergency shutdown

// SYSTEM INSTALLATION
Command: unlook (installed at /usr/local/bin/unlook)
Run from anywhere after system installation
```

## libcamera-sync Integration (SYSTEM INSTALLED) ✅

The system has **completely replaced** standard libcamera with custom libcamera-sync:
- **Installation Status**: System-wide installation completed, standard libcamera removed
- **Location**: Installed at `/usr/local/` (system library paths)
- **Source Path**: Built from `/home/alessandro/libcamera-sync-fix/`
- **Hardware Sync Support**: Full XVS/XHS synchronization for IMX296 cameras
- **Integration**: Direct system library usage (no wrapper needed)
- **Performance**: <1ms synchronization precision achieved

## Build System Features
- **Modern CMake 3.16+** with target-based configuration
- **Cross-compilation support** for ARM64/Raspberry Pi
- **Automatic dependency management** with system/third-party fallback
- **Professional build options**: Debug, Release, with sanitizers and LTO
- **Comprehensive testing** framework with Google Test
- **Package generation**: DEB, RPM, TGZ formats
- **ARM64 Optimizations**: Cortex-A72 (CM4) and Cortex-A76 (CM5) specific flags
- **NEON Vectorization**: Automatic detection and optimization

## Agent Usage Guidelines
- **Always use specialized agents**: stereo-calibration-specialist, camera-sync-manager, point-cloud-processor, mesh-generation-expert, etc.
- **Never build simultaneously with multiple agents**: Risk of conflicts
- **Use libcamera-sync version**: System-installed at `/usr/local/` (replaces standard libcamera)
- **Respect camera mapping**: Camera 1=LEFT/MASTER, Camera 0=RIGHT/SLAVE
- **DO NOT run GUI tests**: User tests manually with `unlook` command
- **DO NOT use QT_QPA_PLATFORM=offscreen for testing**: User validates GUI functionality

## Code Standards (C++ EXCLUSIVE)
- **C++17/20 standards** exclusively
- **Object-Oriented Programming** professional C++
- **Thread-safe implementations** using std::mutex, std::atomic
- **Comprehensive error handling** C++ exceptions with context
- **Performance-optimized** for ARM64/Raspberry Pi
- **Zero Python runtime dependencies**

## Testing Notes
- Unit tests use Google Test framework (BUILD_TESTS=ON)
- Hardware tests require actual IMX296 cameras
- Performance tests validate real-time requirements
- Memory tests use AddressSanitizer and Valgrind
- **GUI testing**: User validates with `unlook` command (DO NOT attempt automated GUI tests)
- **Library paths**: Critical for runtime (see Critical Hardware Info above)
