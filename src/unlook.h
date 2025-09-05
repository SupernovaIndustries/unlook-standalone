/**
 * @file unlook.h
 * @brief Main API header for Unlook 3D Scanner Library
 * 
 * This is the primary include file for the Unlook 3D Scanner library.
 * Include this file to access all core functionality.
 * 
 * @copyright 2024 Unlook Project
 * @license MIT License
 */

#ifndef UNLOOK_H
#define UNLOOK_H

// Version information
#define UNLOOK_VERSION_MAJOR 1
#define UNLOOK_VERSION_MINOR 0
#define UNLOOK_VERSION_PATCH 0
#define UNLOOK_VERSION_STRING "1.0.0"

// Core system headers
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>

// OpenCV integration
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

// Core modules
#include "core/logger.h"
#include "core/config.h"
#include "core/exception.h"
#include "core/timer.h"
#include "core/types.h"

// Camera system
#include "camera/camera_manager.h"
#include "camera/stereo_camera.h"
#include "camera/libcamera_wrapper.h"
#include "camera/camera_sync.h"

// Calibration system
#include "calibration/calibration_manager.h"
#include "calibration/pattern_detector.h"
#include "calibration/stereo_calibration.h"
#include "calibration/validation.h"

// Stereo processing
#include "stereo/stereo_matcher.h"
#include "stereo/depth_processor.h"
#include "stereo/rectification.h"

// Hardware interface
#include "hardware/i2c_controller.h"
#include "hardware/gpio_controller.h"
#include "hardware/led_controller.h"

// Mathematical utilities
#include "math/geometry.h"
#include "math/optimization.h"
#include "math/transforms.h"

// I/O operations
#include "io/file_io.h"
#include "io/calibration_io.h"
#include "io/point_cloud_io.h"

// Utility functions
#include "utils/image_utils.h"
#include "utils/debug_utils.h"
#include "utils/profiler.h"

/**
 * @namespace unlook
 * @brief Main namespace for all Unlook 3D Scanner functionality
 * 
 * All classes, functions, and constants in the Unlook library
 * are contained within this namespace to avoid naming conflicts.
 */
namespace unlook {

/**
 * @brief Library version information
 */
struct Version {
    int major = UNLOOK_VERSION_MAJOR;
    int minor = UNLOOK_VERSION_MINOR;
    int patch = UNLOOK_VERSION_PATCH;
    std::string string = UNLOOK_VERSION_STRING;
};

/**
 * @brief Get library version information
 * @return Version structure with version details
 */
inline Version getVersion() {
    return Version{};
}

/**
 * @brief Initialize the Unlook library
 * 
 * This function must be called before using any other library functions.
 * It sets up logging, hardware interfaces, and other global state.
 * 
 * @param log_level Logging level (DEBUG, INFO, WARNING, ERROR)
 * @param config_path Optional path to configuration file
 * @return true if initialization successful, false otherwise
 */
bool initialize(core::LogLevel log_level = core::LogLevel::INFO, 
                const std::string& config_path = "");

/**
 * @brief Shutdown the Unlook library
 * 
 * Cleans up resources and shuts down hardware interfaces.
 * Call this before program termination.
 */
void shutdown();

/**
 * @brief Quick start configuration for common use cases
 */
namespace quickstart {
    
    /**
     * @brief Setup for stereo camera calibration
     * @return Configured calibration manager
     */
    std::unique_ptr<calibration::CalibrationManager> setupCalibration();
    
    /**
     * @brief Setup for real-time stereo capture
     * @return Configured stereo camera system
     */
    std::unique_ptr<camera::StereoCamera> setupStereoCapture();
    
    /**
     * @brief Setup for depth processing
     * @return Configured depth processor
     */
    std::unique_ptr<stereo::DepthProcessor> setupDepthProcessing();
    
} // namespace quickstart

} // namespace unlook

#endif // UNLOOK_H