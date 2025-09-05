# Unlook 3D Scanner - Shared Library API Architecture

## Overview

The **Unlook 3D Scanner API** is a comprehensive, industrial-grade C++ library providing complete control over the Unlook 3D scanning system. The API supports both **Standalone Mode** (direct GUI operation) and **Companion Mode** (external PC control), ensuring that all functionality uses the same underlying API architecture.

### Key Features

- **Hardware Synchronized Stereo**: <1ms precision IMX296 camera synchronization
- **High-Precision Calibration**: Sub-pixel calibration accuracy with BoofCV integration  
- **Real-time Processing**: Optimized stereo matching and depth computation
- **Industrial Grade**: Thread-safe, comprehensive error handling, performance monitoring
- **Modular Design**: Separate camera, depth processing, and calibration APIs
- **Cross-Platform**: Raspberry Pi CM4/CM5 optimized, ARM64 NEON accelerated
- **Multiple Modes**: Standalone GUI or companion library operation

## Architecture

### API Structure

```
include/unlook/
├── unlook.h                    # Main API header (include this)
├── core/
│   ├── types.h                 # Core type definitions
│   ├── logger.h                # Thread-safe logging system
│   ├── config.h                # Configuration management
│   └── exception.h             # Exception handling
└── api/
    ├── unlook_scanner.h        # Main scanner API
    ├── camera_system.h         # Camera control and sync
    ├── depth_processor.h       # Stereo matching and depth
    └── calibration_manager.h   # Calibration and validation
```

### Core API Classes

#### 1. UnlookScanner (Main API)
- **Purpose**: Primary interface for all scanner operations
- **Responsibilities**: 
  - System initialization and shutdown
  - Mode management (standalone/companion)
  - Subsystem coordination
  - Status monitoring and error handling

```cpp
unlook::api::UnlookScanner scanner;
auto result = scanner.initialize(unlook::core::ScannerMode::STANDALONE);
auto* camera = scanner.getCameraSystem();
auto* depth = scanner.getDepthProcessor();
```

#### 2. CameraSystem API
- **Purpose**: Hardware-synchronized stereo camera control
- **Key Features**:
  - Camera initialization with hardware sync validation
  - Exposure/gain controls (manual/auto modes)  
  - Resolution management (fixed 1456x1088 for calibration)
  - Camera swap functionality (LEFT ↔ RIGHT)
  - Frame capture (single and continuous with callbacks)

```cpp
auto* camera = scanner.getCameraSystem();
camera->setExposure(15000);  // 15ms exposure
camera->startCapture();

cv::Mat left, right;
camera->captureSingleFrame(left, right);
```

#### 3. DepthProcessor API  
- **Purpose**: Stereo matching and 3D reconstruction
- **Key Features**:
  - Calibration loading and validation
  - Multiple stereo algorithms (OpenCV SGBM, BoofCV)
  - Real-time depth map generation
  - Point cloud export (PLY, OBJ formats)
  - Performance monitoring

```cpp
auto* depth = scanner.getDepthProcessor();
depth->loadCalibration("calibration/calib_boofcv_test3.yaml");

cv::Mat depth_map;
depth->processFrames(left, right, depth_map);
depth->exportPointCloud(depth_map, "scan.ply");
```

#### 4. CalibrationManager API
- **Purpose**: Calibration capture, computation, and validation
- **Key Features**:
  - Multiple pattern support (checkerboard, ChArUco)
  - Calibration session management
  - Quality assessment and validation
  - Report generation
  - Pattern detection for live preview

```cpp
auto* calib = scanner.getCalibrationManager();
calib->startCalibrationSession();
calib->addCalibrationImage(left, right);

CalibrationResults results;
calib->computeCalibration(results);
```

## Hardware Integration

### Camera Configuration
- **Sensors**: 2x IMX296 Global Shutter (1456x1088 SBGGR10)
- **Mapping**: Camera 1=LEFT/MASTER, Camera 0=RIGHT/SLAVE
- **Synchronization**: XVS/XHS hardware sync, <1ms precision
- **Baseline**: 70.017mm (calibrated)

### Dependencies
- **libcamera-sync**: System installation at `/usr/local/` (replaces standard libcamera)
- **OpenCV 4.0+**: Core image processing and computer vision
- **Qt5**: GUI applications (optional)
- **Threading**: C++17 std::thread, std::mutex, std::atomic

## Build System

### CMake Integration

The API uses modern CMake (3.16+) with comprehensive configuration:

```cmake
# Find and use Unlook API
find_package(Unlook REQUIRED)
target_link_libraries(your_target Unlook::unlook)
```

### Build Script

Use the provided build script for easy compilation:

```bash
# Default Release build
./build.sh

# Debug build with clean
./build.sh -t Debug --clean

# Install system dependencies  
./build.sh --install-deps

# Cross-compile for Raspberry Pi
./build.sh -x rpi4 -j 4

# Create installation package
./build.sh --package
```

### Build Options
- `BUILD_SHARED_LIBS`: Build shared library (default: ON)
- `BUILD_EXAMPLES`: Build example applications (default: ON)  
- `BUILD_GUI`: Build Qt-based GUI apps (default: ON)
- `BUILD_TESTS`: Build test suite (default: OFF)
- `ENABLE_OPENMP`: Enable OpenMP support (default: ON)

## Usage Examples

### Basic API Usage

```cpp
#include <unlook/unlook.h>

int main() {
    // Initialize API (optional)
    unlook::initialize();
    
    // Create scanner
    unlook::api::UnlookScanner scanner;
    
    // Initialize in standalone mode
    auto result = scanner.initialize(unlook::core::ScannerMode::STANDALONE);
    if (result != unlook::core::ResultCode::SUCCESS) {
        std::cerr << "Initialization failed: " << scanner.getLastError() << std::endl;
        return 1;
    }
    
    // Use camera system
    auto* camera = scanner.getCameraSystem();
    if (camera) {
        cv::Mat left, right;
        camera->captureSingleFrame(left, right);
    }
    
    // Use depth processor
    auto* depth = scanner.getDepthProcessor();
    if (depth) {
        cv::Mat depth_map;
        depth->processFrames(left, right, depth_map);
        depth->exportPointCloud(depth_map, "scan.ply");
    }
    
    // Cleanup
    scanner.shutdown();
    unlook::shutdown();
    return 0;
}
```

### Camera System Example

```cpp
auto* camera = scanner.getCameraSystem();

// Set camera parameters
camera->setExposure(12000);  // 12ms
camera->setGain(1.5f);
camera->setExposureMode(unlook::api::ExposureMode::MANUAL);

// Single frame capture
cv::Mat left, right;
auto result = camera->captureSingleFrame(left, right, 5000); // 5s timeout

// Continuous capture with callback
camera->setFrameCallback([](const cv::Mat& left, const cv::Mat& right, 
                          uint64_t timestamp_us, void* user_data) {
    // Process frames in real-time
    std::cout << "Frame received: " << timestamp_us << " μs" << std::endl;
});

camera->startCapture();
std::this_thread::sleep_for(std::chrono::seconds(5));
camera->stopCapture();
```

### Depth Processing Example

```cpp
auto* depth = scanner.getDepthProcessor();

// Load calibration
depth->loadCalibration("calibration/calib_boofcv_test3.yaml");

// Configure stereo parameters
unlook::core::StereoParams params = depth->getStereoParams();
params.block_size = 21;
params.num_disparities = 64;
params.use_boofcv = true;  // High precision
depth->setStereoParams(params);

// Process frames
cv::Mat left, right, depth_map, disparity_map;
camera->captureSingleFrame(left, right);
depth->processFrames(left, right, depth_map, &disparity_map);

// Export point cloud
unlook::api::ExportOptions export_opts;
export_opts.format = unlook::core::ExportFormat::PLY_BINARY;
export_opts.include_colors = true;
export_opts.filters.remove_outliers = true;

depth->exportPointCloud(depth_map, "high_quality_scan.ply", export_opts, &left);
```

### C API Usage

```c
#include <stdio.h>

int main() {
    // Create scanner instance
    void* scanner = unlook_scanner_create();
    
    // Initialize (0 = STANDALONE mode)
    int result = unlook_scanner_initialize(scanner, 0, NULL);
    if (result != 0) {
        printf("Initialization failed: %d\n", result);
        return 1;
    }
    
    // Check status
    if (unlook_scanner_is_initialized(scanner)) {
        printf("Scanner ready!\n");
    }
    
    // Cleanup
    unlook_scanner_shutdown(scanner);
    unlook_scanner_destroy(scanner);
    return 0;
}
```

## Performance Characteristics

### Target Specifications
- **Precision**: 0.005mm repeatability at 100mm distance
- **Processing Speed**: >20 FPS stereo processing on CM4
- **Memory Usage**: Optimized for 8GB CM4 constraints  
- **Synchronization**: <1ms camera hardware sync precision
- **Resolution**: 1456x1088 (calibration optimized)

### Optimization Features
- **ARM64 NEON**: Hardware-accelerated vector operations
- **Multi-threading**: Parallel stereo processing
- **Memory Pooling**: Efficient memory management
- **OpenMP**: Parallel algorithm execution
- **Cache Optimization**: CM4/CM5 cache-friendly data structures

## Error Handling

The API uses comprehensive error handling with:

- **Result Codes**: Detailed error classification
- **Exception System**: C++ exceptions with context information
- **Logging**: Thread-safe logging with multiple levels
- **Status Callbacks**: Real-time status monitoring

```cpp
try {
    auto result = scanner.initialize();
    if (result != unlook::core::ResultCode::SUCCESS) {
        std::cerr << "Error: " << scanner.getLastError() << std::endl;
    }
} catch (const unlook::core::Exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    std::cerr << "Code: " << static_cast<int>(e.getResultCode()) << std::endl;
}
```

## Configuration Management

The API supports comprehensive configuration:

```cpp
// Global configuration
auto& config = unlook::core::Config::getInstance();
config.setValue("camera.exposure_us", 15000);
config.setValue("stereo.use_boofcv", true);

// Save/load configuration
scanner.saveConfiguration("my_config.conf");
scanner.loadConfiguration("my_config.conf");

// Sectioned configuration access
unlook::core::ConfigSection camera_config("camera");
uint32_t exposure = camera_config.get<uint32_t>("exposure_us", 10000);
```

## Thread Safety

All API classes are designed for thread-safe operation:

- **Shared Resources**: Protected by std::mutex
- **Atomic Operations**: std::atomic for status flags
- **Callback Safety**: Exception-safe callback execution
- **Concurrent Access**: Multiple threads can safely access different subsystems

## Installation

### From Source
```bash
git clone <repository-url>
cd unlook-standalone
./build.sh --install-deps  # Install dependencies
./build.sh                 # Build release version
sudo make install -C build # Install system-wide
```

### Using Package
```bash
./build.sh --package       # Create package
sudo tar -xzf build/unlook-*.tar.gz -C /
```

### CMake Integration
```cmake
find_package(Unlook 1.0 REQUIRED)
target_link_libraries(your_app Unlook::unlook)
```

### pkg-config Integration  
```bash
gcc -o myapp myapp.c `pkg-config --cflags --libs unlook`
```

## Language Bindings

The C API functions enable bindings for other languages:

- **Python**: Using ctypes or cffi
- **JavaScript/Node.js**: Using ffi-napi  
- **Go**: Using CGO
- **Rust**: Using bindgen
- **C#**: Using P/Invoke

See `examples/c_api_example.c` for detailed binding examples.

## File Structure

```
unlook-standalone/
├── include/unlook/           # Public API headers
│   ├── unlook.h             # Main API header
│   ├── core/                # Core system headers
│   └── api/                 # API class headers
├── src/                     # Implementation source
│   ├── core/                # Core system implementation  
│   └── api/                 # API implementation
├── examples/                # Example applications
├── calibration/             # Default calibration files
├── build.sh                 # Build script
├── CMakeLists.txt          # Main CMake configuration
└── README_API.md           # This documentation
```

## Support and Documentation

- **API Documentation**: Generated with Doxygen (`make doc`)
- **Examples**: See `examples/` directory
- **Build Issues**: Check `./build.sh --validate`
- **Hardware Issues**: Verify libcamera-sync installation
- **Performance**: Use `examples/performance_benchmark`

---

**Version**: 1.0.0  
**Target Platform**: Raspberry Pi CM4/CM5  
**License**: [Project License]  
**Maintainer**: Unlook Team