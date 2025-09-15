# PROJECT GUIDELINES - UNLOOK 3D SCANNER PHASE 1

**MANDATORY READING**: Every development session must read this file first.

This document contains critical development standards, architecture decisions, and implementation guidelines for the Unlook 3D Scanner Phase 1 production system.

---

## 🎯 PROJECT OVERVIEW

**Unlook** is a professional modular opensource 3D scanner with **0.005mm target precision**, designed for industrial, educational, and professional applications. The system combines stereovision, structured light VCSEL, and FPGA acceleration to achieve industrial-level performance.

### Target Applications
- Industrial quality control and inspection
- Educational 3D scanning projects  
- Professional prototyping and reverse engineering
- Research applications requiring high precision measurements

### Technical Specifications
- **Target Precision**: 0.005mm repeatability
- **Architecture**: Modular with interchangeable components
- **License**: Completely opensource
- **Language**: **C++17/20 EXCLUSIVELY** (ZERO Python in production)
- **Paradigm**: Professional C++ Object-Oriented Programming
- **Deployment**: Custom OS based on Raspbian
- **Runtime**: **ZERO Python dependencies**

---

## 🏗️ ARCHITECTURE OVERVIEW

### Core Design Principles

1. **Shared Library API Architecture**
   - Core Principle: The scanner operates in two modes using the **SAME** underlying API
   - Standalone Mode: Direct GUI operation on Raspberry Pi
   - Companion Mode: External PC control via shared library API
   - ALL GUI components use the same API classes that external applications will use
   - NO direct hardware access in GUI code

2. **Hardware Integration**
   - libcamera-sync C++ API (NOT command-line binaries)
   - Hardware synchronization with <1ms precision
   - Thread-safe camera access with singleton pattern
   - Auto-detection: system libcamera-sync first, fallback to third-party

3. **Industrial Grade Quality**
   - C++17/20 standards exclusively
   - Thread-safe implementations using std::mutex, std::atomic
   - RAII principles, smart pointers
   - Comprehensive error handling with custom C++ exceptions
   - Performance-optimized for ARM64/Raspberry Pi

---

## 🔧 HARDWARE CONFIGURATION

### Camera System (CRITICAL - DO NOT CHANGE)
```cpp
// FIXED camera mapping
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
Resolution: 1456x1088 YUV420 (correct format for stride handling)
Baseline: 70.017mm (from calib_boofcv_test3.yaml)
Hardware Sync: XVS/XHS enabled, MAS pin configured
Timeout Config: timeout.yaml MUST be exported before CameraManager
```

### Camera Hardware Synchronization
- **XVS** (External Vertical Sync): Physical pin connection between cameras
- **XHS** (External Horizontal Sync): Physical pin connection between cameras
- **MAS** (Master/Slave): Physical pin connection between cameras (soldered)
- **Target Precision**: <1ms synchronization accuracy
- **Implementation**: Hardware sync via physical connections, not Pi GPIO
- **Fallback**: Software synchronization if hardware sync fails

### LED Controller System (TO BE IMPLEMENTED)
```cpp
// AS1170 LED SYSTEM
I2C Bus: 4, Address: 0x30, Strobe GPIO: 27
LED1 (VCSEL): For structured light projection (Phase 2)
LED2 (Flood): For stereo calibration illumination
Timing: Synchronized with camera capture
Safety: Temperature monitoring, emergency shutdown required
```

---

## 🛠️ BUILD SYSTEM

### Primary Build Commands
```bash
# Default build (recommended)
./build.sh -j4

# Common options
./build.sh --clean              # Clean build
./build.sh --deps               # Build dependencies first
./build.sh --debug              # Debug build
./build.sh --install            # System installation
./build.sh --test               # Run validation tests
./build.sh --cross rpi4 -j4     # Cross-compile for Raspberry Pi
./build.sh --package            # Create installation package
```

### Build Output Structure
```
build/
├── bin/unlook_scanner      # Main GUI application
├── lib/libunlook.so       # Shared API library
├── include/unlook/        # API headers (installed)
└── test/                  # Validation tests
```

### Dependency Management
All external libraries in `third-party/` with automated builds:
- **libcamera-sync-fix**: Fallback if system version fails
- **BoofCV**: Dynamic library for high-precision calibration
- **BoofCPP**: C++ wrapper for BoofCV
- **Java Runtime**: Embedded for BoofCV
- **OpenCV**: System preferred, third-party fallback
- **Qt5**: Development libraries for GUI

---

## 💻 API ARCHITECTURE

### Core Namespace Structure
```cpp
namespace unlook {
    namespace core {      // Base classes, logging, configuration
        class Logger;
        class Configuration;
        class Exception;
    }
    namespace api {       // Main API classes
        class UnlookScanner;     // Main API class
        class CameraSystem;      // Synchronized camera control  
        class DepthProcessor;    // Stereo matching and depth
        class CalibrationManager; // Calibration loading/validation
    }
    namespace camera {    // Camera management
        class CameraDevice;
        class CameraSynchronizer;
        class AutoExposure;
    }
    namespace stereo {    // Stereo processing
        class StereoMatcher;
        class SGBMStereoMatcher;
    }
    namespace calibration { // Calibration system
        class CalibrationValidator;
        class BoofCVWrapper;
    }
}
```

### API Usage Pattern
```cpp
#include <unlook/unlook.h>

// Initialize scanner
unlook::api::UnlookScanner scanner;
scanner.initialize(unlook::core::ScannerMode::STANDALONE);

// Access subsystems
auto* camera = scanner.getCameraSystem();
auto* depth = scanner.getDepthProcessor();

// Perform operations
cv::Mat left, right, depth_map;
camera->captureSingleFrame(left, right);
depth->processFrames(left, right, depth_map);
```

---

## 🎨 GUI ARCHITECTURE

### Qt5 Fullscreen Touch Interface

**Core Requirements**:
- Always fullscreen on startup (hide window decorations)
- Touch-optimized for Raspberry Pi touchscreen
- ESC key toggles windowed mode for development
- Single CameraSystem instance shared across ALL GUI windows

**Screen Navigation**:
```
Main Menu
├── Camera Preview (CameraPreviewWidget)
├── Depth Test (DepthTestWidget)
├── Options (OptionsWidget)
└── Exit
```

**Supernova-tech Design System**:
- **Primary Color**: #00E5CC (Electric Primary)
- **Accent Color**: #00B8A3 (Plasma Accent)
- **Background**: #000000 (Void Background)
- **Surfaces**: #1A2B2A (Nebula Surfaces)
- **Typography**: Inter font family
- **Touch Targets**: Minimum 48px

### Custom Widgets
- **TouchButton**: Touch-friendly with animations
- **ParameterSlider**: High-resolution parameter control
- **StatusDisplay**: Color-coded status with progress

### Qt Designer Integration (MANDATORY)
- **ALL GUI widgets MUST use .ui files** for visual layout
- **setupUi() MUST be called** to load .ui file layouts
- **C++ code connects to existing widgets** via findChild<>() or object names
- **NO manual layout creation** that conflicts with .ui files
- **Qt Design Studio compatibility** maintained for visual editing
- **.ui files stored in** `src/gui/ui/` directory

---

## 🔍 CALIBRATION SYSTEM

### Current Calibration Status
- **File**: `calibration/calib_boofcv_test3.yaml`
- **Baseline**: 70.017mm (measured and calibrated)
- **RMS Error**: 0.24 pixels (target: <0.2px for improvement)
- **Image Size**: 1456x1088 (IMX296 sensors)
- **Precision**: 0.0035mm at reference distance

### Calibration Loading
```cpp
auto calibManager = std::make_shared<CalibrationManager>();
calibManager->loadCalibration("calibration/calib_boofcv_test3.yaml");

// Validate quality
CalibrationValidator validator;
validator.setCalibration(calibManager);
bool meetsIndustrial = validator.validateIndustrialPrecision();
```

---

## 📏 STEREO PROCESSING

### Depth Processing Pipeline
1. **NOT Real-Time**: Capture button → process stereo pair → display result
2. **Algorithm Options**: OpenCV SGBM (minimum), BoofCV advanced
3. **Parameter Adjustment**: All algorithm parameters exposed through GUI
4. **Quality Assessment**: Multiple validation layers
5. **Export Capabilities**: PNG depth maps, point clouds (future)

### Stereo Matcher Usage
```cpp
DepthProcessor depthProcessor;
depthProcessor.initialize(calibManager);
depthProcessor.setStereoMatcher(StereoAlgorithm::SGBM);

// Process stereo pair
cv::Mat leftImage, rightImage, depthMap;
depthProcessor.processStereoPair(leftImage, rightImage, depthMap);
```

---

## ⚡ PERFORMANCE REQUIREMENTS

### Target Performance
- **VGA Processing**: >20 FPS on CM4
- **HD Processing**: >10 FPS on CM4  
- **Camera Sync**: <1ms precision
- **GUI Response**: <100ms for all interactions
- **Memory Usage**: Optimized for CM4 8GB constraints

### ARM64 Optimizations
- **Compiler Flags**: `-march=armv8-a+crc -mtune=cortex-a72`
- **SIMD**: NEON optimizations where applicable
- **Memory**: Pool-based allocation for frequent operations
- **Threading**: Multi-core utilization for stereo processing

---

## 🧪 TESTING AND VALIDATION

### Test Categories
- **Unit Tests**: Google Test framework
- **Integration Tests**: Component interaction validation
- **Hardware Tests**: Camera sync, LED control, real hardware
- **Performance Tests**: Benchmarking and regression detection

### Validation Commands
```bash
# Build and run all tests
./build.sh --test

# Hardware-specific validation
./build/bin/camera_test -s          # Sync precision test
./build/bin/calibration_validation  # Calibration quality
```

---

## 📋 CODING STANDARDS

### Language Rules
- **C++17/20 exclusively** - ZERO Python in production code
- **No emojis** in source code, comments, or variable names
- **UTF-8 encoding** for all text files
- **Standard C++ conventions** for naming

### Code Structure
```cpp
// Namespace organization
namespace unlook {
namespace subsystem {

class ExampleClass {
public:
    // Public interface first
    void publicMethod();
    
private:
    // Private members
    std::mutex mutex_;
    std::atomic<bool> active_{false};
    
    // Private methods
    void privateMethod();
};

} // namespace subsystem
} // namespace unlook
```

### Error Handling
- **RAII principles** - automatic resource management
- **Custom exceptions** with context information
- **Thread-safe error reporting** via logging system
- **User-friendly error messages** in GUI

### Documentation
- **Every class documented** with purpose and usage
- **Public methods documented** with parameters and return values
- **Complex algorithms explained** with references
- **API examples provided** for all major functionality

---

## 🚀 DEVELOPMENT WORKFLOW

### Phase 1 Implementation Status
- ✅ **API Architecture**: Complete shared library design
- ✅ **Camera System**: Hardware sync implementation ready
- ✅ **Calibration System**: Loading and validation complete
- ✅ **Qt5 GUI**: Fullscreen touch interface complete
- ✅ **Build System**: Comprehensive CMake with dependencies
- 🔄 **Integration**: Connect all systems together
- 🔄 **Hardware Validation**: Test with real IMX296 cameras
- 🔄 **Final Build**: Single build after all development complete

### Development Rules
1. **NO building during development phases** - causes conflicts
2. **All agents coordinate** - no parallel builds by different agents  
3. **Single final build** only after ALL code complete
4. **User approval required** before git initialization
5. **Hardware testing** after software integration complete

### Git Workflow (AFTER successful build)
1. **Build validation**: Ensure all tests pass
2. **User approval**: Explicit confirmation required
3. **Repository initialization**: Clean git history
4. **SSH configuration**: Production deployment ready

---

## 🔒 CRITICAL RULES

### Build Protocol (MANDATORY)
- ❌ **NO BUILDING** during development phases
- ❌ **NO parallel builds** by different agents
- ❌ **NO partial system builds** 
- ✅ **SINGLE build** only after ALL code complete
- ✅ **ALL todos completed** BEFORE build
- ✅ **User approval required** before git initialization

### Hardware Rules
- ❌ **DO NOT change** camera mapping (Camera 1=LEFT, Camera 0=RIGHT)
- ❌ **DO NOT bypass** API for direct hardware access in GUI
- ❌ **DO NOT use** command-line libcamera tools in production
- ✅ **ALWAYS use** libcamera-sync C++ API
- ✅ **VALIDATE** hardware sync precision (<1ms requirement)

### Code Quality Rules
- ❌ **NO Python** in production code
- ❌ **NO placeholder** or TODO implementations
- ❌ **NO fake/mock** implementations in production paths
- ❌ **NO duplicate code** between components
- ✅ **100% functional** implementations required
- ✅ **Comprehensive error handling** required
- ✅ **Thread-safe** implementations required

---

## 📖 SUCCESS CRITERIA

### Phase 1 Complete When:
- ✅ Cameras initialize and sync automatically
- ✅ GUI runs fullscreen with touch controls
- ✅ Depth processing produces accurate results  
- ✅ All parameters adjustable in real-time
- ✅ Build system works with single command
- ✅ API ready for both standalone/companion modes
- ✅ Complete documentation and guidelines
- ✅ Git repository ready for production development

**ZERO tolerance for incomplete implementations. Every feature must be 100% functional.**

---

This document represents the complete development foundation for the Unlook 3D Scanner Phase 1. All development must adhere to these guidelines to ensure industrial-grade quality and consistency.

**Remember**: Read this file at the start of every development session.