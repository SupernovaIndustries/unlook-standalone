#pragma once

#include "unlook/core/types.hpp"
#include "unlook/core/Logger.hpp"
#include "unlook/core/config.h"
#include <memory>
#include <string>
#include <functional>
#include <atomic>
#include <mutex>

/**
 * @file unlook_scanner.h
 * @brief Main API class for the Unlook 3D Scanner system
 */

namespace unlook {
namespace api {

// Forward declarations
class CameraSystem;
class DepthProcessor;
class CalibrationManager;

/**
 * @brief Status callback function type
 * 
 * Called when scanner status changes or errors occur.
 * 
 * @param status Current scanner status
 * @param user_data User-provided data pointer
 */
using StatusCallback = std::function<void(const core::ScannerStatus& status, void* user_data)>;

/**
 * @brief Main Unlook 3D Scanner API class
 * 
 * This is the primary interface for controlling the Unlook 3D Scanner.
 * It provides unified access to all scanner subsystems and handles
 * initialization, configuration, and lifecycle management.
 * 
 * Thread-safe design allows concurrent access from multiple threads.
 * All operations return ResultCode for comprehensive error handling.
 */
class UnlookScanner {
public:
    /**
     * @brief Constructor
     */
    UnlookScanner();
    
    /**
     * @brief Destructor - automatically shuts down scanner
     */
    ~UnlookScanner();
    
    // Non-copyable, non-movable (hardware resource)
    UnlookScanner(const UnlookScanner&) = delete;
    UnlookScanner& operator=(const UnlookScanner&) = delete;
    UnlookScanner(UnlookScanner&&) = delete;
    UnlookScanner& operator=(UnlookScanner&&) = delete;
    
    /**
     * @brief Initialize the scanner system
     * 
     * Performs hardware detection, camera initialization, and system validation.
     * Must be called before using any other API functions.
     * 
     * @param mode Scanner operation mode (STANDALONE or COMPANION)
     * @param config_path Optional path to configuration file
     * @return ResultCode indicating success or failure
     */
    core::ResultCode initialize(core::ScannerMode mode = core::ScannerMode::STANDALONE,
                               const std::string& config_path = "");
    
    /**
     * @brief Shutdown the scanner system
     * 
     * Releases all hardware resources and stops all operations.
     * Scanner must be re-initialized before use after shutdown.
     * 
     * @return ResultCode indicating success or failure
     */
    core::ResultCode shutdown();
    
    /**
     * @brief Check if scanner is initialized
     * @return True if scanner is initialized and ready
     */
    bool isInitialized() const;
    
    /**
     * @brief Get current scanner status
     * @return Current scanner status structure
     */
    core::ScannerStatus getStatus() const;
    
    /**
     * @brief Get API version information
     * @return Version structure with major.minor.patch information
     */
    core::Version getVersion() const;
    
    /**
     * @brief Set status callback for asynchronous notifications
     * 
     * The callback will be invoked whenever scanner status changes
     * or errors occur. Callback is executed on internal thread.
     * 
     * @param callback Status callback function
     * @param user_data Optional user data passed to callback
     */
    void setStatusCallback(StatusCallback callback, void* user_data = nullptr);
    
    /**
     * @brief Get camera system interface
     * 
     * Provides access to camera control, synchronization, and preview.
     * 
     * @return Pointer to camera system (valid only when initialized)
     */
    CameraSystem* getCameraSystem() const;
    
    /**
     * @brief Get depth processor interface
     * 
     * Provides access to stereo matching, depth map generation,
     * and calibration loading.
     * 
     * @return Pointer to depth processor (valid only when initialized)
     */
    DepthProcessor* getDepthProcessor() const;
    
    /**
     * @brief Get calibration manager interface
     * 
     * Provides access to calibration validation, quality assessment,
     * and export functionality.
     * 
     * @return Pointer to calibration manager (valid only when initialized)
     */
    CalibrationManager* getCalibrationManager() const;
    
    /**
     * @brief Load configuration from file
     * 
     * Updates current configuration with values from file.
     * Scanner must be reinitialized for hardware changes to take effect.
     * 
     * @param config_path Path to configuration file
     * @return ResultCode indicating success or failure
     */
    core::ResultCode loadConfiguration(const std::string& config_path);
    
    /**
     * @brief Save current configuration to file
     * 
     * Saves all current settings to configuration file for persistence.
     * 
     * @param config_path Path to configuration file
     * @return ResultCode indicating success or failure
     */
    core::ResultCode saveConfiguration(const std::string& config_path) const;
    
    /**
     * @brief Perform system self-test
     * 
     * Validates hardware connectivity, camera synchronization,
     * and basic functionality. Useful for diagnostics.
     * 
     * @return ResultCode indicating test results
     */
    core::ResultCode performSelfTest();
    
    /**
     * @brief Get last error message
     * 
     * Returns detailed error information for the last failed operation.
     * 
     * @return Error message string (empty if no error)
     */
    std::string getLastError() const;

private:
    // Internal state
    mutable std::mutex mutex_;
    std::atomic<bool> initialized_;
    core::ScannerMode mode_;
    core::ScannerStatus status_;
    std::string last_error_;
    
    // Callback management
    StatusCallback status_callback_;
    void* callback_user_data_;
    
    // Subsystem interfaces
    std::unique_ptr<CameraSystem> camera_system_;
    std::unique_ptr<DepthProcessor> depth_processor_;
    std::unique_ptr<CalibrationManager> calibration_manager_;
    
    // Internal methods
    void updateStatus(const core::ScannerStatus& new_status);
    void notifyStatusChange();
    void setError(const std::string& error_message);
    core::ResultCode initializeSubsystems();
    core::ResultCode shutdownSubsystems();
    core::ResultCode detectHardware();
    core::ResultCode validateSystem();

    // Calibration auto-load helper
    std::string findLatestCalibration() const;

    // Hardware detection helpers
    bool detectLibCameraSync() const;
    bool detectIMX296Cameras() const;
    bool validateCameraMapping() const;
};

/**
 * @brief C-style API functions for language bindings
 * 
 * These functions provide a C interface for integration with
 * other languages and systems.
 */
extern "C" {
    /**
     * @brief Create scanner instance
     * @return Opaque pointer to scanner instance
     */
    void* unlook_scanner_create();
    
    /**
     * @brief Destroy scanner instance
     * @param scanner Scanner instance pointer
     */
    void unlook_scanner_destroy(void* scanner);
    
    /**
     * @brief Initialize scanner
     * @param scanner Scanner instance pointer
     * @param mode Scanner mode (0=STANDALONE, 1=COMPANION)
     * @param config_path Configuration file path (NULL for default)
     * @return Result code (0=success, negative=error)
     */
    int unlook_scanner_initialize(void* scanner, int mode, const char* config_path);
    
    /**
     * @brief Shutdown scanner
     * @param scanner Scanner instance pointer
     * @return Result code (0=success, negative=error)
     */
    int unlook_scanner_shutdown(void* scanner);
    
    /**
     * @brief Check if scanner is initialized
     * @param scanner Scanner instance pointer
     * @return 1 if initialized, 0 if not
     */
    int unlook_scanner_is_initialized(void* scanner);
    
    /**
     * @brief Get API version string
     * @return Version string (e.g., "1.0.0-dev")
     */
    const char* unlook_get_version();
}

} // namespace api
} // namespace unlook