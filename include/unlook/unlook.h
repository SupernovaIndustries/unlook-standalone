#pragma once

/**
 * @file unlook.h
 * @brief Main header for the Unlook 3D Scanner API
 * 
 * This header provides access to the complete Unlook 3D Scanner API.
 * Include this single header to access all functionality.
 * 
 * @version 1.0.0
 * @author Unlook Team
 * @copyright 2025 Unlook. All rights reserved.
 */

// Core types and utilities
#include "unlook/core/types.hpp"
#include "unlook/core/logger.h"
#include "unlook/core/config.h"
#include "unlook/core/exception.h"

// Main API classes
#include "unlook/api/unlook_scanner.h"
#include "unlook/api/camera_system.h"
#include "unlook/api/depth_processor.h"
#include "unlook/api/calibration_manager.h"

/**
 * @brief Main Unlook 3D Scanner API namespace
 * 
 * All Unlook API classes and functions are contained within this namespace.
 * Use this namespace to access the complete scanner functionality.
 */
namespace unlook {

/**
 * @brief Get Unlook API version
 * @return Version structure with major.minor.patch information
 */
inline core::Version getVersion() {
    return core::API_VERSION;
}

/**
 * @brief Get version string
 * @return Version string (e.g., "1.0.0-dev")
 */
inline std::string getVersionString() {
    return core::API_VERSION.toString();
}

/**
 * @brief Check API compatibility
 * @param required_major Required major version
 * @param required_minor Required minor version  
 * @return True if current API is compatible
 */
inline bool isCompatible(uint8_t required_major, uint8_t required_minor = 0) {
    const auto& current = core::API_VERSION;
    
    // Major version must match exactly
    if (current.major != required_major) {
        return false;
    }
    
    // Minor version must be >= required
    return current.minor >= required_minor;
}

/**
 * @brief Initialize Unlook API system
 * 
 * Optional initialization function for global API setup.
 * This is not required but can be used to configure
 * logging and other global settings.
 * 
 * @param log_level Minimum log level
 * @param log_to_file Enable file logging
 * @return ResultCode indicating success or failure
 */
inline core::ResultCode initialize(core::LogLevel log_level = core::LogLevel::INFO,
                                  bool log_to_file = true) {
    try {
        // Initialize global logger
        auto& logger = core::Logger::getInstance();
        logger.initialize(log_level, true, log_to_file, "unlook.log");
        
        // Initialize global configuration
        auto& config = core::Config::getInstance();
        config.initializeDefaults();
        
        UNLOOK_LOG_INFO("API") << "Unlook API initialized - Version " << getVersionString();
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        return core::ResultCode::ERROR_GENERIC;
    }
}

/**
 * @brief Shutdown Unlook API system
 * 
 * Optional cleanup function for global API shutdown.
 * Flushes logs and releases global resources.
 */
inline void shutdown() {
    UNLOOK_LOG_INFO("API") << "Unlook API shutdown";
    core::Logger::getInstance().flush();
}

} // namespace unlook

/**
 * @brief Usage examples and quick start guide
 * 
 * @code{.cpp}
 * #include <unlook/unlook.h>
 * 
 * int main() {
 *     // Initialize API (optional)
 *     unlook::initialize();
 *     
 *     // Create scanner instance
 *     unlook::api::UnlookScanner scanner;
 *     
 *     // Initialize in standalone mode
 *     auto result = scanner.initialize(unlook::core::ScannerMode::STANDALONE);
 *     if (result != unlook::core::ResultCode::SUCCESS) {
 *         std::cerr << "Failed to initialize scanner: " << scanner.getLastError() << std::endl;
 *         return 1;
 *     }
 *     
 *     // Get camera system
 *     auto* camera = scanner.getCameraSystem();
 *     if (camera) {
 *         // Start capture
 *         camera->startCapture();
 *         
 *         // Capture single frame
 *         cv::Mat left, right;
 *         camera->captureSingleFrame(left, right);
 *         
 *         // Stop capture
 *         camera->stopCapture();
 *     }
 *     
 *     // Get depth processor
 *     auto* depth = scanner.getDepthProcessor();
 *     if (depth && camera) {
 *         cv::Mat left, right, depth_map;
 *         camera->captureSingleFrame(left, right);
 *         
 *         // Process to depth map
 *         depth->processFrames(left, right, depth_map);
 *         
 *         // Export point cloud
 *         depth->exportPointCloud(depth_map, "scan.ply");
 *     }
 *     
 *     // Shutdown scanner
 *     scanner.shutdown();
 *     
 *     // Cleanup API (optional)
 *     unlook::shutdown();
 *     
 *     return 0;
 * }
 * @endcode
 * 
 * @section features Key Features
 * 
 * - **Hardware Synchronized Stereo**: <1ms precision IMX296 camera synchronization
 * - **High-Precision Calibration**: Sub-pixel calibration accuracy with BoofCV integration
 * - **Real-time Processing**: Optimized stereo matching and depth computation
 * - **Industrial Grade**: Thread-safe, comprehensive error handling, performance monitoring
 * - **Modular Design**: Separate camera, depth processing, and calibration APIs
 * - **Cross-Platform**: Raspberry Pi CM4/CM5 optimized, ARM64 NEON accelerated
 * - **Multiple Modes**: Standalone GUI or companion library operation
 * 
 * @section performance Performance Characteristics
 * 
 * - **Target Precision**: 0.005mm repeatability at 100mm distance
 * - **Camera Resolution**: 1456x1088 (calibration optimized)
 * - **Processing Speed**: >20 FPS stereo processing on CM4
 * - **Memory Usage**: Optimized for 8GB CM4 constraints
 * - **Synchronization**: <1ms camera hardware synchronization
 * 
 * @section compatibility Hardware Compatibility
 * 
 * - **Cameras**: 2x IMX296 Global Shutter sensors with hardware sync
 * - **Platform**: Raspberry Pi CM4/CM5 with custom carrier board
 * - **Dependencies**: libcamera-sync, OpenCV 4.0+, Qt5 (GUI only)
 * - **OS**: Custom Raspbian-based distribution
 */