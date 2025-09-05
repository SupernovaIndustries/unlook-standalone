# API Overview - Unlook 3D Scanner

The Unlook 3D Scanner provides a comprehensive **C++17/20 API** designed for both **standalone operation** and **external integration**. The API follows professional C++ patterns with thread-safe design and industrial-grade error handling.

---

## 🏗️ **Architecture Overview**

### **Shared Library Design**
The scanner operates in two modes using the **same** underlying API:

- **Standalone Mode**: Direct GUI operation on Raspberry Pi
- **Companion Mode**: External PC control via shared library API

All GUI components use the same API classes that external applications will use - **no direct hardware access** in GUI code.

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
    namespace hardware {      // I2C, GPIO, LED control
        class AS1170Controller; class GPIOManager; class I2CDevice;
    }
}
```

---

## 🚀 **Quick Start Example**

### **Basic Stereo Capture**
```cpp
#include <unlook/unlook.h>

int main() {
    try {
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
    } catch (const unlook::core::Exception& e) {
        std::cerr << "Unlook error: " << e.what() << std::endl;
        return -1;
    }
}
```

### **Depth Processing**
```cpp
#include <unlook/unlook.h>

int main() {
    try {
        // Initialize scanner
        unlook::api::UnlookScanner scanner;
        scanner.initialize(unlook::core::ScannerMode::COMPANION);
        
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
    } catch (const unlook::core::Exception& e) {
        std::cerr << "Processing error: " << e.what() << std::endl;
        return -1;
    }
}
```

---

## 📖 **Core API Classes**

### **1. UnlookScanner - Primary Interface**

```cpp
namespace unlook::api {
    class UnlookScanner {
    public:
        // Initialization
        void initialize(core::ScannerMode mode = core::ScannerMode::STANDALONE);
        void shutdown();
        bool isInitialized() const;
        
        // Component access
        CameraSystem* getCameraSystem();
        DepthProcessor* getDepthProcessor();
        CalibrationManager* getCalibrationManager();
        hardware::LEDController* getLEDController();
        
        // System information
        std::string getVersion() const;
        std::string getBuildInfo() const;
        core::SystemStatus getSystemStatus() const;
    };
}
```

### **2. CameraSystem - Synchronized Camera Control**

```cpp
namespace unlook::api {
    class CameraSystem {
    public:
        // Initialization
        bool initialize(const camera::CameraConfig& config = {});
        void shutdown();
        bool isRunning() const;
        
        // Camera control
        bool startCapture(std::function<void(cv::Mat, cv::Mat)> callback = nullptr);
        void stopCapture();
        bool captureSingleFrame(cv::Mat& left, cv::Mat& right);
        
        // Parameters
        void setExposureTime(std::chrono::microseconds exposure);
        void setGain(double gain);
        void setFrameRate(double fps);
        
        // Synchronization
        bool isHardwareSyncEnabled() const;
        std::chrono::microseconds getSyncPrecision() const;
        camera::SyncStatus getSyncStatus() const;
        
        // Configuration
        void setCameraMapping(camera::CameraMapping mapping);
        camera::CameraInfo getCameraInfo(camera::CameraID id) const;
    };
}
```

### **3. DepthProcessor - Stereo Processing**

```cpp
namespace unlook::api {
    class DepthProcessor {
    public:
        // Initialization
        bool initialize(CalibrationManager* calibration);
        void shutdown();
        
        // Processing methods
        bool processFrames(const cv::Mat& left, const cv::Mat& right, cv::Mat& depth);
        bool processFramesAsync(const cv::Mat& left, const cv::Mat& right, 
                               std::function<void(cv::Mat)> callback);
        
        // Algorithm selection
        void setStereoAlgorithm(stereo::StereoAlgorithm algorithm);
        stereo::StereoAlgorithm getCurrentAlgorithm() const;
        
        // Parameters (algorithm-specific)
        void setMinDisparity(int min_disparity);
        void setNumDisparities(int num_disparities);
        void setBlockSize(int block_size);
        void setUniquenessRatio(int ratio);
        void setSpeckleWindowSize(int size);
        void setSpeckleRange(int range);
        
        // Quality assessment
        stereo::DepthMetrics getLastProcessingMetrics() const;
        double getProcessingTime() const;
        
        // Export capabilities
        bool exportDepthMap(const cv::Mat& depth, const std::string& filename);
        bool exportPointCloud(const cv::Mat& depth, const std::string& filename); // Future
    };
}
```

### **4. CalibrationManager - High-Precision Calibration**

```cpp
namespace unlook::api {
    class CalibrationManager {
    public:
        // Calibration loading
        bool loadCalibration(const std::string& filepath);
        bool saveCalibration(const std::string& filepath) const;
        bool isCalibrationLoaded() const;
        
        // Calibration data access
        cv::Mat getCameraMatrix(camera::CameraID camera) const;
        cv::Mat getDistortionCoefficients(camera::CameraID camera) const;
        cv::Mat getRotationMatrix() const;
        cv::Mat getTranslationVector() const;
        cv::Mat getFundamentalMatrix() const;
        cv::Mat getEssentialMatrix() const;
        
        // Rectification maps
        bool getRectificationMaps(cv::Mat& map1_left, cv::Mat& map2_left,
                                 cv::Mat& map1_right, cv::Mat& map2_right) const;
        
        // Quality metrics
        double getReprojectionError() const;
        double getBaseline() const; // in mm
        cv::Size getImageSize() const;
        
        // Validation
        calibration::CalibrationQuality validateCalibration() const;
        bool meetsIndustrialStandards() const;
        
        // Advanced calibration (BoofCV integration - future)
        bool performAdvancedCalibration(const std::vector<cv::Mat>& images);
    };
}
```

---

## 🔧 **Configuration and Error Handling**

### **Configuration System**
```cpp
namespace unlook::core {
    class Configuration {
    public:
        // Load configuration from file
        static bool loadFromFile(const std::string& filepath);
        
        // Camera configuration
        static camera::CameraConfig getCameraConfig();
        static void setCameraConfig(const camera::CameraConfig& config);
        
        // Processing parameters
        static stereo::ProcessingConfig getProcessingConfig();
        static void setProcessingConfig(const stereo::ProcessingConfig& config);
        
        // Hardware settings
        static hardware::HardwareConfig getHardwareConfig();
        static void setHardwareConfig(const hardware::HardwareConfig& config);
    };
}
```

### **Exception Handling**
```cpp
namespace unlook::core {
    class Exception : public std::exception {
    public:
        Exception(const std::string& message, ErrorCode code = ErrorCode::GENERIC);
        
        const char* what() const noexcept override;
        ErrorCode getErrorCode() const;
        std::string getDetailedMessage() const;
    };
    
    // Specific exception types
    class CameraException : public Exception { /* ... */ };
    class CalibrationException : public Exception { /* ... */ };
    class ProcessingException : public Exception { /* ... */ };
    class HardwareException : public Exception { /* ... */ };
}
```

### **Logging System**
```cpp
namespace unlook::core {
    class Logger {
    public:
        enum class Level { DEBUG, INFO, WARNING, ERROR, CRITICAL };
        
        static void initialize(Level level = Level::INFO);
        static void log(Level level, const std::string& message);
        static void setOutputFile(const std::string& filepath);
        static void enableConsoleOutput(bool enable = true);
    };
}
```

---

## 📊 **Thread Safety and Performance**

### **Thread-Safe Design**
- **Singleton Pattern**: Camera system uses thread-safe singleton
- **Mutex Protection**: All shared resources protected with `std::mutex`
- **Atomic Operations**: Status flags use `std::atomic<bool>`
- **Qt Signal/Slot**: GUI communication via Qt's thread-safe mechanism

### **Performance Considerations**
- **Memory Pooling**: Frequent allocations use pre-allocated pools
- **ARM64 NEON**: Vectorized operations for stereo processing
- **Multi-threading**: Parallel processing for stereo algorithms
- **Zero-copy**: Direct memory access where possible

### **Example Thread-Safe Usage**
```cpp
// Multiple threads can safely access the same scanner instance
unlook::api::UnlookScanner scanner;
scanner.initialize();

// Thread 1: Camera control
std::thread camera_thread([&scanner]() {
    auto* camera = scanner.getCameraSystem();
    camera->startCapture([](cv::Mat left, cv::Mat right) {
        // Process frames in callback
    });
});

// Thread 2: Depth processing
std::thread processing_thread([&scanner]() {
    auto* depth_processor = scanner.getDepthProcessor();
    cv::Mat left, right, depth;
    if (scanner.getCameraSystem()->captureSingleFrame(left, right)) {
        depth_processor->processFrames(left, right, depth);
    }
});

// Threads are automatically synchronized by the API
```

---

## 🔗 **Integration Examples**

### **External Application Integration**
```cpp
// Your external application
#include <unlook/unlook.h>

class YourApplication {
private:
    std::unique_ptr<unlook::api::UnlookScanner> scanner_;
    
public:
    bool initialize() {
        scanner_ = std::make_unique<unlook::api::UnlookScanner>();
        return scanner_->initialize(unlook::core::ScannerMode::COMPANION);
    }
    
    bool captureAndProcess() {
        cv::Mat left, right, depth;
        
        // Capture stereo pair
        if (!scanner_->getCameraSystem()->captureSingleFrame(left, right)) {
            return false;
        }
        
        // Process depth
        return scanner_->getDepthProcessor()->processFrames(left, right, depth);
    }
};
```

### **Custom Algorithm Integration**
```cpp
// Extend the stereo processing system
namespace unlook::stereo {
    class CustomStereoMatcher : public StereoMatcher {
    public:
        bool processImages(const cv::Mat& left, const cv::Mat& right, 
                          cv::Mat& depth) override {
            // Your custom stereo algorithm here
            return true;
        }
        
        void setParameters(const StereoParameters& params) override {
            // Handle parameter updates
        }
    };
}

// Register custom algorithm
auto* depth_processor = scanner.getDepthProcessor();
depth_processor->registerCustomAlgorithm(std::make_unique<CustomStereoMatcher>());
```

---

## 📚 **Next Steps**

- 🎥 **[Camera System API](Camera-API)** - Detailed camera control documentation
- 📐 **[Stereo Processing API](Stereo-API)** - Depth processing algorithms
- 🔬 **[Calibration API](Calibration-API)** - High-precision calibration system
- 🔧 **[Hardware Integration](Hardware-Setup)** - Physical setup and connections
- 🏗️ **[Build System](Build-System)** - Compilation and deployment

---

<p align="center">
  <strong>Professional C++ API for Industrial 3D Scanning</strong><br>
  <em>Complete documentation • Thread-safe • High performance</em>
</p>