# Camera-LED Synchronization Integration for Calibration Dataset Capture

## Overview
This document describes the implementation of camera capture integration with LED synchronization for the Unlook 3D Scanner calibration system. The integration enables precise hardware-synchronized stereo capture with VCSEL illumination for high-quality calibration dataset acquisition.

## Implementation Status

### ✅ Completed Components

1. **DatasetCaptureWidget** (`include/unlook/gui/dataset_capture_widget.hpp`, `src/gui/dataset_capture_widget.cpp`)
   - Full GUI implementation for dataset capture
   - Real-time dual camera preview with pattern detection overlay
   - Automatic capture with configurable intervals (5 seconds default)
   - VCSEL LED control during capture
   - Hardware synchronization validation (<1ms error checking)
   - HD resolution output (1280x720) downsampled from native 1456x1088
   - JSON metadata generation with quality metrics

2. **Integration Points**
   - CameraSystem singleton for synchronized capture
   - AS1170Controller singleton for LED control
   - PatternDetector for calibration pattern detection
   - Thread-safe frame capture with mutex protection

### 🔧 Required Modifications

#### 1. CameraSystem Extensions
The existing `CameraSystem` class needs these helper methods for calibration:

```cpp
// Add to include/unlook/camera/CameraSystem.hpp
public:
    // Get latest frame from specific camera (for preview)
    cv::Mat getLeftFrame();
    cv::Mat getRightFrame();

    // Capture new synchronized frame pair
    cv::Mat captureLeftFrame();
    cv::Mat captureRightFrame();

    // Get precise timestamps in microseconds
    uint64_t getLastCaptureTimestamp(int cameraId);
```

**Implementation in `src/camera/CameraSystem.cpp`:**
```cpp
cv::Mat CameraSystem::getLeftFrame() {
    StereoFrame frame;
    if (captureStereoFrame(frame, 100)) {
        return frame.leftImage;
    }
    return cv::Mat();
}

cv::Mat CameraSystem::getRightFrame() {
    StereoFrame frame;
    if (captureStereoFrame(frame, 100)) {
        return frame.rightImage;
    }
    return cv::Mat();
}

uint64_t CameraSystem::getLastCaptureTimestamp(int cameraId) {
    std::lock_guard<std::mutex> lock(statusMutex_);
    // Return last timestamp from specified camera
    // Camera 0 = RIGHT, Camera 1 = LEFT (hardware mapping)
    return (cameraId == 0) ? lastRightTimestamp_ : lastLeftTimestamp_;
}
```

#### 2. PatternDetector Implementation
Create `src/calibration/PatternDetector.cpp`:

```cpp
#include "unlook/calibration/PatternDetector.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace unlook {
namespace calibration {

PatternDetector::PatternDetector(const PatternConfig& config)
    : config_(config)
    , confidenceScore_(0.0f) {

    if (config_.type == PatternType::CHARUCO) {
        // Initialize ChArUco detector
        arucoDict_ = cv::aruco::getPredefinedDictionary(config_.arucoDict);
        charucoBoard_ = cv::aruco::CharucoBoard::create(
            config_.cols, config_.rows,
            config_.squareSizeMM, config_.arucoMarkerSizeMM,
            arucoDict_);
        detectorParams_ = cv::aruco::DetectorParameters::create();
    }
}

bool PatternDetector::detect(const cv::Mat& image,
                            std::vector<cv::Point2f>& corners,
                            cv::Mat& overlayImage) {
    overlayImage = image.clone();

    switch (config_.type) {
        case PatternType::CHECKERBOARD:
            return detectCheckerboard(image, corners, overlayImage);
        case PatternType::CHARUCO:
            return detectCharuco(image, corners, overlayImage);
        case PatternType::CIRCLE_GRID:
            return detectCircleGrid(image, corners, overlayImage);
    }

    return false;
}

bool PatternDetector::detectCheckerboard(const cv::Mat& image,
                                        std::vector<cv::Point2f>& corners,
                                        cv::Mat& overlayImage) {
    cv::Size patternSize(config_.cols - 1, config_.rows - 1);

    bool found = cv::findChessboardCorners(image, patternSize, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        // Refine corners
        cv::Mat gray;
        if (image.channels() > 1) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image;
        }

        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        // Draw corners
        drawCheckerboardCorners(overlayImage, corners);

        // Calculate confidence based on corner quality
        confidenceScore_ = corners.size() / float((config_.rows - 1) * (config_.cols - 1));
    }

    return found;
}

bool PatternDetector::detectCharuco(const cv::Mat& image,
                                   std::vector<cv::Point2f>& corners,
                                   cv::Mat& overlayImage) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect ArUco markers
    cv::aruco::detectMarkers(image, arucoDict_, markerCorners, markerIds, detectorParams_);

    if (markerIds.size() > 0) {
        // Interpolate ChArUco corners
        std::vector<int> charucoIds;
        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, image,
            charucoBoard_, corners, charucoIds);

        if (corners.size() > 4) {
            // Draw detected corners
            cv::aruco::drawDetectedCornersCharuco(overlayImage, corners, charucoIds);

            // Calculate confidence
            int expectedCorners = (config_.rows - 1) * (config_.cols - 1);
            confidenceScore_ = corners.size() / float(expectedCorners);

            return true;
        }
    }

    return false;
}

// ... Additional methods implementation ...

} // namespace calibration
} // namespace unlook
```

#### 3. CMakeLists.txt Modifications

Add to `CMakeLists.txt`:

```cmake
# Check and install dependencies
find_package(nlohmann_json 3.10.0 QUIET)
if(NOT nlohmann_json_FOUND)
    message(STATUS "Installing nlohmann-json...")
    execute_process(
        COMMAND sudo apt-get update
        COMMAND sudo apt-get install -y nlohmann-json3-dev
        RESULT_VARIABLE INSTALL_RESULT
    )
    if(NOT INSTALL_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to install nlohmann-json3-dev")
    endif()
    find_package(nlohmann_json 3.10.0 REQUIRED)
endif()

# Add calibration components
add_library(unlook_calibration_gui STATIC
    src/gui/dataset_capture_widget.cpp
    src/calibration/PatternDetector.cpp
)

target_link_libraries(unlook_calibration_gui
    PUBLIC
        Qt5::Core
        Qt5::Widgets
        opencv_core
        opencv_imgproc
        opencv_calib3d
        opencv_aruco
        nlohmann_json::nlohmann_json
        unlook_camera
        unlook_hardware
        unlook_core
)

target_include_directories(unlook_calibration_gui
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

## Integration Architecture

### System Flow

```
┌─────────────────────┐
│DatasetCaptureWidget │
└──────────┬──────────┘
           │
     ┌─────┴─────┐
     │           │
     ▼           ▼
┌────────┐  ┌─────────┐
│Camera  │  │AS1170   │
│System  │  │Controller│
└────────┘  └─────────┘
     │           │
     ▼           ▼
┌────────┐  ┌─────────┐
│Stereo  │  │VCSEL    │
│Cameras │  │LED      │
└────────┘  └─────────┘
```

### Key Features

1. **Hardware Synchronization**
   - XVS/XHS sync validation
   - <1ms frame sync error checking
   - Real-time sync status display

2. **LED Integration**
   - Automatic VCSEL enable during capture
   - Configurable current (280mA default)
   - Safety shutdown on completion

3. **Pattern Detection**
   - Real-time overlay visualization
   - Support for Checkerboard, ChArUco, Circle Grid
   - Quality assessment and confidence scoring

4. **Dataset Management**
   - HD 1280x720 PNG image storage
   - JSON metadata with quality metrics
   - Automatic directory structure creation

## Verification Checklist

### Hardware Setup
- [ ] Camera 1 (LEFT/MASTER) connected to `/base/soc/i2c0mux/i2c@1/imx296@1a`
- [ ] Camera 0 (RIGHT/SLAVE) connected to `/base/soc/i2c0mux/i2c@0/imx296@1a`
- [ ] XVS/XHS signals connected between cameras
- [ ] MAS pin soldered on camera sink
- [ ] AS1170 LED controller on I2C bus 1, address 0x30
- [ ] VCSEL connected to LED1 channel
- [ ] GPIO 17 connected for strobe control

### Software Verification
- [ ] CameraSystem initializes successfully
- [ ] AS1170Controller initializes successfully
- [ ] Hardware sync active (<1ms error)
- [ ] VCSEL LED enables during capture
- [ ] Pattern detection works in real-time
- [ ] Images saved as HD 1280x720 PNG
- [ ] JSON metadata generated correctly

### Testing Commands

```bash
# Build with calibration support
./build.sh --clean -j4

# Test hardware sync
./build/test_hardware_sync_new

# Test LED controller
./build/examples/test_as1170_controller

# Run main application
unlook
# Navigate to Calibration → Dataset Capture
```

## Performance Metrics

- **Preview Rate**: 30 FPS (33ms timer)
- **Capture Resolution**: 1456x1088 native → 1280x720 HD
- **Sync Precision**: <1ms between cameras
- **VCSEL Current**: 280mA (configurable)
- **Capture Interval**: 5 seconds (configurable)
- **Dataset Size**: 50 pairs default

## Error Handling

### Camera Errors
- Non-critical: Warning dialog, continue without sync
- Critical: Error dialog, disable capture

### LED Errors
- Non-critical: Warning dialog, continue without VCSEL
- Critical: Emergency shutdown, error dialog

### Pattern Detection Errors
- No detection: Red overlay, status update
- Partial detection: Orange overlay, warning
- Full detection: Green overlay, ready to capture

## Future Enhancements

1. **Advanced Pattern Support**
   - Custom pattern configurations
   - Multi-pattern detection
   - Automatic pattern type detection

2. **Enhanced Quality Metrics**
   - Motion blur detection
   - Sharpness assessment
   - Coverage analysis

3. **Real-time Feedback**
   - Voice guidance for positioning
   - Auto-capture on optimal positioning
   - Quality score visualization

## Troubleshooting

### Common Issues

1. **"Failed to initialize camera system"**
   - Check camera connections
   - Verify libcamera-sync installation
   - Run `sudo systemctl restart camera`

2. **"Failed to initialize LED controller"**
   - Check I2C connection: `i2cdetect -y 1`
   - Verify AS1170 at address 0x30
   - Check GPIO 17 availability

3. **"Hardware synchronization not active"**
   - Verify XVS/XHS connections
   - Check MAS pin configuration
   - Review sync error in logs

4. **Pattern not detected**
   - Ensure proper lighting
   - Check pattern configuration matches physical board
   - Verify focus and exposure settings

## Conclusion

The camera-LED synchronization integration provides a robust foundation for high-precision stereo calibration dataset capture. The implementation ensures hardware-level synchronization, automated capture workflow, and comprehensive quality validation suitable for industrial 3D scanning applications with 0.005mm target precision.