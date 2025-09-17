# UNLOOK 3D SCANNER

<p align="center">
  <a href="https://supernovaindustries.it">
    <img src="http://supernovaindustries.it/wp-content/uploads/2024/08/supernova-industries-logo.svg" alt="Supernova Industries Logo" width="200" />
  </a>
</p>

<p align="center">
  <img src="http://supernovaindustries.it/wp-content/uploads/2024/08/logo_full-white-unlook-1.svg" alt="Unlook Logo" width="300" />
</p>

<p align="center">
  <strong>Professional Modular Open Source 3D Scanner</strong><br>
  <strong>Target Precision: 0.005mm • Industrial Grade • Completely Open Source</strong>
</p>

<p align="center">
  Made with ❤️ by <a href="https://supernovaindustries.it">Supernova Industries</a>
</p>

<p align="center">
  <a href="https://supernovaindustries.it">Website</a> •
  <a href="https://github.com/SupernovaIndustries">GitHub</a> •
  <a href="#features">Features</a> •
  <a href="#architecture">Architecture</a> •
  <a href="#how-it-works">How It Works</a> •
  <a href="#hardware-requirements">Hardware</a> •
  <a href="#installation">Installation</a> •
  <a href="#usage">Usage</a> •
  <a href="#development">Development</a> •
  <a href="#license">License</a>
</p>

---

## 🎯 **Overview**

**Unlook** is a professional modular opensource 3D scanner designed for **industrial, educational, and professional applications**. The system combines **stereovision**, **structured light VCSEL**, and optimized **ARM64 acceleration** to achieve **industrial-level performance** at accessible costs.

### **Target Applications**
- 🏭 **Industrial quality control** and inspection  
- 🎓 **Educational 3D scanning** projects
- ⚙️ **Professional prototyping** and reverse engineering
- 🔬 **Research applications** requiring high precision measurements

### **Technical Specifications**
- 🎯 **Target Precision**: **0.005mm** repeatability
- 🏗️ **Architecture**: Modular with interchangeable components
- 📄 **License**: **MIT** - Completely opensource
- 💻 **Language**: **C++17/20** exclusively (ZERO Python in production)
- 🚀 **Performance**: VGA stereo processing >20 FPS on CM4/CM5
- 🔧 **Deployment**: Self-contained system on Raspberry Pi CM5

---

## ⭐ **Features**

### **🎛️ Industrial-Grade Hardware**
- **2x Global Shutter IMX296 Cameras** (1456x1088 SBGGR10)
- **Hardware Synchronization** with <1ms precision (XVS/XHS)
- **Calibrated Baseline**: 70.017mm with 0.24px RMS accuracy
- **AS1170 LED Controller** with VCSEL structured light projection
- **Optimized for Raspberry Pi CM5** (16GB RAM recommended)

### **🖥️ Fullscreen Touch Interface**
- **Industrial Touch Design** with Supernova-tech styling
- **Foolproof Operation** - 2-minute operator mastery
- **Real-time Controls** for all stereo parameters
- **Live Dual Camera Preview** with sync status
- **Touch-optimized** with 48px minimum targets

### **⚡ High-Performance Processing**
- **C++17/20 Exclusively** - Zero Python dependencies
- **Thread-safe Architecture** with professional C++ patterns
- **ARM64 NEON Optimizations** for Raspberry Pi
- **Multiple Stereo Algorithms**: OpenCV SGBM, BoofCV precision
- **Memory-optimized** for embedded systems

### **🔧 Complete Development Stack**
- **Single Command Build** - `./build.sh` does everything
- **Cross-compilation Support** for ARM64/Raspberry Pi
- **Comprehensive Testing** framework with Google Test
- **Professional Documentation** with API examples
- **GitHub Integration** ready for CI/CD

---

## 🏗️ **Architecture**

### **Shared Library API Design**
The scanner operates in two modes using the **same** underlying API:
- **Standalone Mode**: Direct GUI operation on Raspberry Pi
- **Companion Mode**: External PC control via shared library API

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

### **Professional C++ Namespaces**
```cpp
namespace unlook {
    namespace api {           // Main API classes
        class UnlookScanner;      // Primary interface
        class CameraSystem;       // Synchronized camera control  
        class DepthProcessor;     // Stereo matching and depth
        class CalibrationManager; // Calibration loading/validation
    }
    namespace core {          // Base classes, logging, configuration
        class Logger; class Configuration; class Exception;
    }
    namespace camera {        // Camera management and sync
        class CameraDevice; class CameraSynchronizer; class AutoExposure;
    }
    namespace stereo {        // Stereo processing algorithms
        class StereoMatcher; class SGBMStereoMatcher;
    }
    namespace calibration {   // High-precision calibration
        class CalibrationValidator; class BoofCVWrapper;
    }
    namespace gui {           // Qt5-based touch interface
        class MainWindow; class CameraPreviewWidget; class DepthTestWidget;
    }
}
```

---

## ⚙️ **How It Works**

### **1. Hardware Synchronization**
- **Camera 1 (LEFT)** = MASTER (`/base/soc/i2c0mux/i2c@1/imx296@1a`)
- **Camera 0 (RIGHT)** = SLAVE (`/base/soc/i2c0mux/i2c@0/imx296@1a`)
- **XVS/XHS GPIO** connections ensure <1ms sync precision
- **libcamera-sync** custom library handles synchronization

### **2. Stereo Calibration**
- **Existing Calibration**: `calib_boofcv_test3.yaml` (70.017mm baseline)
- **Multiple Targets**: Checkerboard, ChArUco boards supported
- **BoofCV Integration**: High-precision calibration via JNI wrapper
- **Validation Tools**: RMS error checking and precision assessment

### **3. Depth Processing Pipeline**
- **OpenCV SGBM**: Fast stereo matching for real-time preview
- **BoofCV Advanced**: High-precision algorithms for final processing
- **Parameter Tuning**: All algorithm parameters exposed via GUI
- **Export Options**: PNG depth maps, point clouds (future)

### **4. Touch Interface**
- **Supernova-tech Design**: Electric blues/teals on dark background
- **Three Main Screens**: Camera Preview, Depth Test, Options
- **Real-time Feedback**: Live parameter adjustment with instant results
- **Error Recovery**: Clear guidance for hardware/software issues

---

## 🔩 **Hardware Requirements**

### **Required Hardware**
- **Raspberry Pi CM5** (16GB RAM recommended, 8GB minimum)
- **2x Global Shutter Camera Raspberry Pi IMX296** (1456x1088)
- **6mm focal length lenses** (industrial grade recommended)
- **Rigid stereo mount** with 70mm baseline (thermal stability critical)
- **AS1170 LED Controller** with VCSEL projector (I2C bus 1, address 0x30)
- **GPIO Connections** for camera sync (XVS: GPIO 17, XHS: GPIO 27, MAS: GPIO 22) and LED strobe (GPIO 19)

### **Optional Hardware**
- **Raspberry Pi Official Touchscreen** (7" or 11" recommended)
- **Industrial Enclosure** with thermal management
- **External Power Supply** (5V 4A minimum for full system)

### **Calibration Targets**
- **Standard Checkerboard**: 7x10 pattern, 24mm squares
- **ChArUco Board**: 7x10 pattern, 24mm squares, ArUco 17mm, DICT_4X4_250

---

## 🚀 **Installation**

### **Quick Start (Recommended)**
```bash
# Clone the repository
git clone https://github.com/SupernovaIndustries/unlook-standalone.git
cd unlook-standalone

# Install all dependencies and build
./build.sh --install-deps

# Build the complete system
./build.sh
```

### **Manual Installation**
```bash
# Install system dependencies
sudo apt update
sudo apt install build-essential cmake qt5-default libopencv-dev libopencv-contrib-dev

# Build third-party dependencies
./build.sh --deps

# Configure and build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### **Cross-Compilation for Raspberry Pi**
```bash
# Cross-compile from x86_64 host
./build.sh --cross rpi4 -j 4

# Create deployment package
./build.sh --package
```

### **System Installation**
```bash
# Install system-wide (optional)
sudo make install -C build

# Run from anywhere
unlook_scanner_gui
```

---

## 💻 **Usage**

### **Running the Application**
```bash
# After build - run directly
./build/bin/unlook_scanner_gui

# Debug mode with verbose output
./build.sh -t Debug && ./build/bin/unlook_scanner_gui --debug

# Mock mode (no hardware required)
./build/bin/unlook_scanner_gui --mock
```

### **Interface Navigation**
1. **Main Menu**: Choose Camera Preview, Depth Test, or Options
2. **Camera Preview**: Live dual camera feeds with exposure/gain controls
3. **Depth Test**: Capture stereo pairs and process depth maps
4. **Options**: System status, calibration info, hardware diagnostics
5. **ESC Key**: Toggle fullscreen/windowed mode anytime

### **Stereo Processing Workflow**
1. **Navigate to Depth Test screen**
2. **Adjust stereo parameters** using touch sliders
3. **Press large CAPTURE button** to take stereo pair
4. **Select processing algorithm** (OpenCV SGBM or BoofCV)
5. **View depth map** with false-color visualization
6. **Export results** as PNG or analyze quality metrics

---

## 🛠️ **Development**

### **Build System Options**
```bash
# Development builds
./build.sh -t Debug --clean        # Debug build with clean
./build.sh --tests                 # Build with test suite
./build.sh --validate              # Validate dependencies only

# Production builds  
./build.sh -t Release -j 4          # Optimized release build
./build.sh --install               # System installation
./build.sh --package               # Create DEB/RPM packages
```

### **Testing and Validation**
```bash
# Run all tests
./build.sh --tests && make test -C build

# Hardware-specific tests (requires actual cameras)
./build/bin/test_camera_sync        # Camera synchronization test
./build/bin/test_sync_precision     # Timing precision validation
./test_synchronized_capture.sh     # Full system validation
```

### **Key Executables**
After successful build, key binaries are in `build/bin/`:
- `unlook_scanner_gui` - Main touch interface application
- `stereo_camera_example` - Basic stereo capture example
- `test_boofcv_calibration` - Calibration system test
- `test_camera_sync` - Hardware synchronization validation

### **Development Workflow**
1. **Study Documentation**: Read `CLAUDE.md` and `PROJECT_GUIDELINES.md`
2. **Mock Development**: Use `--mock` flag for interface development
3. **Hardware Testing**: Validate with actual IMX296 cameras
4. **Performance Optimization**: Profile with ARM64 tools
5. **Integration Testing**: Full system validation on Raspberry Pi

---

## 📁 **Project Structure**

```
unlook-standalone/
├── CMakeLists.txt                 # Main CMake configuration
├── build.sh                       # Comprehensive build script
├── CLAUDE.md                      # Complete development guidelines
├── PROJECT_GUIDELINES.md          # Architecture and standards
├── LICENSE                        # MIT License
├── src/                           # Source code
│   ├── unlook.h                   # Main API header
│   ├── core/                      # Core functionality (logging, config)
│   ├── api/                       # Main API classes
│   ├── camera/                    # Camera system integration
│   ├── stereo/                    # Depth processing algorithms
│   ├── calibration/               # Calibration loading and validation
│   ├── gui/                       # Qt5 touch interface
│   │   ├── main_window.cpp        # Main fullscreen window
│   │   ├── camera_preview_widget.cpp # Dual camera preview
│   │   ├── depth_test_widget.cpp  # Depth processing UI
│   │   ├── options_widget.cpp     # System configuration
│   │   └── widgets/               # Custom touch-optimized widgets
│   └── main.cpp                   # Application entry point
├── include/unlook/                # Public API headers
├── third-party/                   # External dependencies
│   ├── libcamera-sync-fix/        # Custom camera synchronization
│   └── boofcv/                    # High-precision calibration (future)
├── calibration/                   # Camera calibration files
│   └── calib_boofcv_test3.yaml    # Current calibration (70.017mm baseline)
├── tests/                         # Comprehensive test suite
├── examples/                      # Example applications and tutorials
├── cmake/                         # CMake modules and toolchain files
├── scripts/                       # Utility scripts for development
└── resources/                     # Icons, desktop files, documentation
```

---

## 🎨 **Design Philosophy**

### **"Industrial Foolproof" Interface**
- **2-minute mastery**: Industrial operators learn interface quickly
- **Automated workflows**: Minimal user decision-making required
- **Visual feedback**: Clear status indication for every operation
- **Error prevention**: Input validation and guided recovery
- **Touch-optimized**: All controls minimum 48px for industrial gloves

### **Professional C++ Architecture**
- **Thread-safe design**: Singleton patterns with std::mutex protection
- **RAII principles**: Automatic resource management throughout
- **Exception handling**: Comprehensive error recovery with user context
- **Performance optimization**: ARM64 NEON, memory pooling, multi-threading
- **Zero Python dependencies**: Pure C++ for maximum reliability

---

## 📊 **Performance Specifications**

### **Target Performance (Raspberry Pi CM5)**
- 🎯 **Precision**: ≤ 0.005mm (hardware limited to 0.008mm at 100mm)
- ⚡ **Processing Speed**: VGA stereo >20 FPS, HD stereo >10 FPS
- 🔄 **Camera Sync**: <1ms synchronization precision
- 📱 **GUI Response**: <100ms for all touch interactions
- 🧠 **Memory Usage**: Optimized for 8GB, recommended 16GB
- 🔋 **Power Consumption**: <15W total system power

### **Validation Requirements**
- **Calibration Quality**: RMS error <0.2 pixels (current: 0.24px)
- **Sync Precision**: Hardware validation <1ms (measured)
- **Processing Accuracy**: Depth precision validation at multiple distances
- **Interface Performance**: 60 FPS GUI on Raspberry Pi touchscreen

---

## 🚀 **Getting Started - Examples**

### **Basic Stereo Capture**
```cpp
#include <unlook/unlook.h>

int main() {
    // Initialize scanner in standalone mode
    unlook::api::UnlookScanner scanner;
    scanner.initialize(unlook::core::ScannerMode::STANDALONE);
    
    // Get camera system
    auto* camera = scanner.getCameraSystem();
    
    // Capture single stereo pair
    cv::Mat left_image, right_image;
    bool success = camera->captureSingleFrame(left_image, right_image);
    
    if (success) {
        cv::imwrite("stereo_left.png", left_image);
        cv::imwrite("stereo_right.png", right_image);
        std::cout << "Stereo pair captured successfully!" << std::endl;
    }
    
    return 0;
}
```

### **Depth Processing**
```cpp
#include <unlook/unlook.h>

int main() {
    // Initialize scanner
    unlook::api::UnlookScanner scanner;
    scanner.initialize(unlook::core::ScannerMode::STANDALONE);
    
    // Get depth processor
    auto* depth_processor = scanner.getDepthProcessor();
    
    // Load existing calibration
    auto* calib_manager = scanner.getCalibrationManager();
    calib_manager->loadCalibration("calibration/calib_boofcv_test3.yaml");
    
    // Process stereo pair
    cv::Mat left_image = cv::imread("stereo_left.png", cv::IMREAD_GRAYSCALE);
    cv::Mat right_image = cv::imread("stereo_right.png", cv::IMREAD_GRAYSCALE);
    cv::Mat depth_map;
    
    bool success = depth_processor->processFrames(left_image, right_image, depth_map);
    
    if (success) {
        cv::imwrite("depth_map.png", depth_map);
        std::cout << "Depth processing completed!" << std::endl;
    }
    
    return 0;
}
```

---

## 🔧 **Configuration**

### **Camera Configuration**
The scanner uses **fixed camera mapping** (DO NOT CHANGE):
```cpp
// Hardware configuration
Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
Resolution: 1456x1088 SBGGR10
Baseline: 70.017mm (calibrated)
Hardware Sync: XVS/XHS enabled, MAS pin configured
```

### **Build Configuration**
Key CMake options:
```bash
# Enable specific features
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DUNLOOK_BUILD_GUI=ON \
         -DUNLOOK_BUILD_TESTS=ON \
         -DUNLOOK_BUILD_EXAMPLES=ON \
         -DUNLOOK_ENABLE_BOOFCV=ON
```

---

## 🧪 **Testing**

### **Unit Tests**
```bash
# Build and run unit tests
./build.sh --tests
cd build && make test
```

### **Integration Tests**
```bash
# Hardware synchronization test
./build/bin/test_camera_sync

# Timing precision validation
./build/bin/test_sync_precision  

# Full system validation
./test_synchronized_capture.sh
```

### **Performance Tests**
```bash
# Stereo processing benchmark
./build/bin/benchmark_stereo

# Memory usage profiling
valgrind --tool=memcheck ./build/bin/unlook_scanner_gui --mock
```

---

## 🤝 **Contributing**

We welcome contributions! Please see our contributing guidelines:

### **Development Setup**
1. **Fork the repository** on GitHub
2. **Clone your fork**: `git clone https://github.com/yourusername/unlook-standalone.git`
3. **Create feature branch**: `git checkout -b feature/your-feature-name`
4. **Follow coding standards** in `PROJECT_GUIDELINES.md`
5. **Test thoroughly**: Run all tests and hardware validation
6. **Submit pull request** with clear description

### **Coding Standards**
- **C++17/20 exclusively** - No Python in production code
- **Professional C++ patterns** - RAII, smart pointers, thread safety
- **Comprehensive documentation** - Every public API documented
- **Industrial quality** - Zero tolerance for incomplete implementations

---

## 📋 **Roadmap**

### **Phase 1: Foundation** ✅
- [x] Complete C++ API architecture
- [x] Hardware synchronization system
- [x] Qt5 fullscreen touch interface  
- [x] OpenCV stereo processing
- [x] Build system with cross-compilation
- [x] Professional documentation

### **Phase 2: Advanced Features** 🔄
- [ ] BoofCV integration for high-precision calibration
- [ ] Point cloud export (PLY/OBJ formats)
- [ ] Advanced calibration validation tools
- [ ] Performance optimization for CM5

### **Phase 3: Production** 🔮
- [ ] Structured light VCSEL integration
- [ ] AS1170 LED controller implementation
- [ ] Advanced depth processing algorithms
- [ ] Industrial deployment packaging

---

## ❓ **FAQ**

### **Q: What's the difference between CM4 and CM5 versions?**
A: The CM5 version takes advantage of increased RAM (16GB vs 8GB) and improved ARM Cortex-A76 cores for better processing performance. The API remains identical.

### **Q: Can I run this without the actual hardware?**
A: Yes! Use `--mock` flag to run the complete interface with simulated cameras and processing.

### **Q: What precision can I expect?**
A: Hardware-limited to ~0.008mm at 100mm distance. Target precision is 0.005mm with optimal calibration.

### **Q: Is Python support planned?**
A: No. The project is designed for 100% C++ for maximum reliability and performance in industrial environments.

### **Q: Can I use different cameras?**
A: The system is optimized for IMX296 global shutter cameras. Other cameras would require significant calibration and synchronization changes.

---

## 📄 **License**

**MIT License** - See [LICENSE](LICENSE) file for details.

This project is completely open source and free for commercial use. We believe professional 3D scanning technology should be accessible to everyone.

---

## 🙏 **Acknowledgments**

- **Raspberry Pi Foundation** for excellent hardware platform
- **OpenCV Community** for robust computer vision libraries
- **Qt Project** for professional GUI framework
- **BoofCV Project** for high-precision calibration algorithms
- **libcamera Project** for professional camera control

---

## 📞 **Support & Contact**

- 🌐 **Website**: [https://supernovaindustries.it](https://supernovaindustries.it)
- 📧 **Email**: alessandro.cursoli@supernovaindustries.com
- 🐙 **GitHub**: [https://github.com/SupernovaIndustries](https://github.com/SupernovaIndustries)
- 📚 **Documentation**: Check `CLAUDE.md` and `PROJECT_GUIDELINES.md`
- 🐛 **Issues**: Use GitHub Issues for bug reports and feature requests

---

<p align="center">
  <strong>🚀 Professional 3D Scanning Made Accessible</strong><br>
  <em>Unlook 3D Scanner - Precision Engineering for Everyone</em>
</p>

<p align="center">
  Made with ❤️ by <a href="https://supernovaindustries.it">Supernova Industries</a>
</p>