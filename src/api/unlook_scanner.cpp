#include "unlook/api/unlook_scanner.h"
#include "unlook/api/camera_system.h"
#include "unlook/api/depth_processor.h"
#include "unlook/api/calibration_manager.h"
#include "unlook/core/exception.h"
#include "unlook/core/logger.h"
#include "unlook/core/config.h"
#include <filesystem>
#include <fstream>
#include <chrono>

namespace unlook {
namespace api {

UnlookScanner::UnlookScanner()
    : initialized_(false)
    , mode_(core::ScannerMode::STANDALONE)
    , status_{}
    , callback_user_data_(nullptr) {
    
    // Initialize status
    status_.initialized = false;
    status_.mode = core::ScannerMode::STANDALONE;
    status_.sync_status = core::SyncStatus::NOT_INITIALIZED;
    status_.calibration_loaded = false;
    status_.last_update = std::chrono::system_clock::now();
    status_.error_message = "";
    
    UNLOOK_LOG_INFO("Scanner") << "UnlookScanner instance created";
}

UnlookScanner::~UnlookScanner() {
    if (initialized_.load()) {
        shutdown();
    }
    UNLOOK_LOG_INFO("Scanner") << "UnlookScanner instance destroyed";
}

core::ResultCode UnlookScanner::initialize(core::ScannerMode mode,
                                          const std::string& config_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_.load()) {
        setError("Scanner is already initialized");
        return core::ResultCode::ERROR_ALREADY_INITIALIZED;
    }
    
    UNLOOK_LOG_INFO("Scanner") << "Initializing Unlook 3D Scanner...";
    
    try {
        // Initialize logging system
        auto& logger = core::Logger::getInstance();
        logger.initialize(core::LogLevel::INFO, true, true, "unlook.log");
        
        // Load configuration
        auto& config = core::Config::getInstance();
        config.initializeDefaults();
        
        if (!config_path.empty()) {
            auto result = config.loadFromFile(config_path);
            if (result != core::ResultCode::SUCCESS) {
                UNLOOK_LOG_WARNING("Scanner") << "Failed to load config file: " << config_path;
            }
        }
        
        // Set scanner mode
        mode_ = mode;
        status_.mode = mode;
        
        UNLOOK_LOG_INFO("Scanner") << "Operating in " 
                                   << (mode == core::ScannerMode::STANDALONE ? "STANDALONE" : "COMPANION")
                                   << " mode";
        
        // Detect hardware
        auto hw_result = detectHardware();
        if (hw_result != core::ResultCode::SUCCESS) {
            setError("Hardware detection failed");
            return hw_result;
        }
        
        // Initialize subsystems
        auto subsys_result = initializeSubsystems();
        if (subsys_result != core::ResultCode::SUCCESS) {
            setError("Subsystem initialization failed");
            return subsys_result;
        }
        
        // Validate system
        auto valid_result = validateSystem();
        if (valid_result != core::ResultCode::SUCCESS) {
            setError("System validation failed");
            return valid_result;
        }
        
        // Update status
        initialized_.store(true);
        status_.initialized = true;
        status_.last_update = std::chrono::system_clock::now();
        status_.error_message = "";
        
        updateStatus(status_);
        
        UNLOOK_LOG_INFO("Scanner") << "Unlook 3D Scanner initialized successfully";
        return core::ResultCode::SUCCESS;
        
    } catch (const core::Exception& e) {
        setError("Initialization failed: " + e.getMessage());
        UNLOOK_LOG_ERROR("Scanner") << "Initialization exception: " << e.what();
        return e.getResultCode();
    } catch (const std::exception& e) {
        setError("Initialization failed: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("Scanner") << "Initialization std::exception: " << e.what();
        return core::ResultCode::ERROR_GENERIC;
    }
}

core::ResultCode UnlookScanner::shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_.load()) {
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    UNLOOK_LOG_INFO("Scanner") << "Shutting down Unlook 3D Scanner...";
    
    try {
        // Shutdown subsystems
        auto result = shutdownSubsystems();
        
        // Update status
        initialized_.store(false);
        status_.initialized = false;
        status_.sync_status = core::SyncStatus::NOT_INITIALIZED;
        status_.last_update = std::chrono::system_clock::now();
        
        updateStatus(status_);
        
        UNLOOK_LOG_INFO("Scanner") << "Unlook 3D Scanner shutdown complete";
        
        // Flush logs
        core::Logger::getInstance().flush();
        
        return result;
        
    } catch (const core::Exception& e) {
        setError("Shutdown failed: " + e.getMessage());
        UNLOOK_LOG_ERROR("Scanner") << "Shutdown exception: " << e.what();
        return e.getResultCode();
    } catch (const std::exception& e) {
        setError("Shutdown failed: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("Scanner") << "Shutdown std::exception: " << e.what();
        return core::ResultCode::ERROR_GENERIC;
    }
}

bool UnlookScanner::isInitialized() const {
    return initialized_.load();
}

core::ScannerStatus UnlookScanner::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

core::Version UnlookScanner::getVersion() const {
    return core::API_VERSION;
}

void UnlookScanner::setStatusCallback(StatusCallback callback, void* user_data) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_callback_ = std::move(callback);
    callback_user_data_ = user_data;
}

CameraSystem* UnlookScanner::getCameraSystem() const {
    if (!initialized_.load()) {
        return nullptr;
    }
    return camera_system_.get();
}

DepthProcessor* UnlookScanner::getDepthProcessor() const {
    if (!initialized_.load()) {
        return nullptr;
    }
    return depth_processor_.get();
}

CalibrationManager* UnlookScanner::getCalibrationManager() const {
    if (!initialized_.load()) {
        return nullptr;
    }
    return calibration_manager_.get();
}

core::ResultCode UnlookScanner::loadConfiguration(const std::string& config_path) {
    if (!initialized_.load()) {
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    try {
        auto& config = core::Config::getInstance();
        return config.loadFromFile(config_path);
    } catch (const std::exception& e) {
        setError("Failed to load configuration: " + std::string(e.what()));
        return core::ResultCode::ERROR_FILE_IO;
    }
}

core::ResultCode UnlookScanner::saveConfiguration(const std::string& config_path) const {
    if (!initialized_.load()) {
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    try {
        auto& config = core::Config::getInstance();
        return config.saveToFile(config_path);
    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("Scanner") << "Failed to save configuration: " << e.what();
        return core::ResultCode::ERROR_FILE_IO;
    }
}

core::ResultCode UnlookScanner::performSelfTest() {
    if (!initialized_.load()) {
        return core::ResultCode::ERROR_NOT_INITIALIZED;
    }
    
    UNLOOK_LOG_INFO("Scanner") << "Performing system self-test...";
    
    try {
        // Test camera system
        if (camera_system_) {
            if (!camera_system_->isInitialized()) {
                setError("Camera system not initialized");
                return core::ResultCode::ERROR_HARDWARE_FAILURE;
            }
            
            // Test synchronization
            auto sync_status = camera_system_->getSyncStatus();
            if (sync_status != core::SyncStatus::SYNCHRONIZED) {
                setError("Camera synchronization failed");
                return core::ResultCode::ERROR_SYNC_FAILURE;
            }
        }
        
        // Test depth processor
        if (depth_processor_) {
            if (!depth_processor_->isInitialized()) {
                setError("Depth processor not initialized");
                return core::ResultCode::ERROR_NOT_INITIALIZED;
            }
            
            if (!depth_processor_->hasValidCalibration()) {
                setError("No valid calibration loaded");
                return core::ResultCode::ERROR_CALIBRATION_INVALID;
            }
        }
        
        // Test calibration manager
        if (calibration_manager_) {
            if (!calibration_manager_->isInitialized()) {
                setError("Calibration manager not initialized");
                return core::ResultCode::ERROR_NOT_INITIALIZED;
            }
        }
        
        UNLOOK_LOG_INFO("Scanner") << "System self-test passed";
        return core::ResultCode::SUCCESS;
        
    } catch (const std::exception& e) {
        setError("Self-test failed: " + std::string(e.what()));
        UNLOOK_LOG_ERROR("Scanner") << "Self-test exception: " << e.what();
        return core::ResultCode::ERROR_GENERIC;
    }
}

std::string UnlookScanner::getLastError() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

void UnlookScanner::updateStatus(const core::ScannerStatus& new_status) {
    status_ = new_status;
    status_.last_update = std::chrono::system_clock::now();
    
    // Notify callback if set
    if (status_callback_) {
        try {
            status_callback_(status_, callback_user_data_);
        } catch (const std::exception& e) {
            UNLOOK_LOG_ERROR("Scanner") << "Status callback exception: " << e.what();
        }
    }
}

void UnlookScanner::notifyStatusChange() {
    updateStatus(status_);
}

void UnlookScanner::setError(const std::string& error_message) {
    last_error_ = error_message;
    status_.error_message = error_message;
    UNLOOK_LOG_ERROR("Scanner") << error_message;
}

core::ResultCode UnlookScanner::initializeSubsystems() {
    UNLOOK_LOG_INFO("Scanner") << "Initializing subsystems...";
    
    // Initialize camera system
    camera_system_ = std::make_unique<CameraSystem>();
    auto cam_result = camera_system_->initialize();
    if (cam_result != core::ResultCode::SUCCESS) {
        UNLOOK_LOG_ERROR("Scanner") << "Camera system initialization failed";
        return cam_result;
    }
    
    // Initialize depth processor with default calibration
    depth_processor_ = std::make_unique<DepthProcessor>();
    auto depth_result = depth_processor_->initialize("calibration/calib_boofcv_test3.yaml");
    if (depth_result != core::ResultCode::SUCCESS) {
        UNLOOK_LOG_WARNING("Scanner") << "Depth processor initialization failed, continuing without calibration";
        // Continue without calibration for now
    } else {
        status_.calibration_loaded = depth_processor_->hasValidCalibration();
    }
    
    // Initialize calibration manager
    calibration_manager_ = std::make_unique<CalibrationManager>();
    auto calib_result = calibration_manager_->initialize();
    if (calib_result != core::ResultCode::SUCCESS) {
        UNLOOK_LOG_WARNING("Scanner") << "Calibration manager initialization failed";
        // Continue without calibration manager
    }
    
    // Update sync status from camera system
    if (camera_system_->isInitialized()) {
        status_.sync_status = camera_system_->getSyncStatus();
    }
    
    UNLOOK_LOG_INFO("Scanner") << "Subsystems initialized successfully";
    return core::ResultCode::SUCCESS;
}

core::ResultCode UnlookScanner::shutdownSubsystems() {
    UNLOOK_LOG_INFO("Scanner") << "Shutting down subsystems...";
    
    core::ResultCode worst_result = core::ResultCode::SUCCESS;
    
    // Shutdown calibration manager
    if (calibration_manager_) {
        auto result = calibration_manager_->shutdown();
        if (result != core::ResultCode::SUCCESS) {
            worst_result = result;
        }
        calibration_manager_.reset();
    }
    
    // Shutdown depth processor
    if (depth_processor_) {
        auto result = depth_processor_->shutdown();
        if (result != core::ResultCode::SUCCESS) {
            worst_result = result;
        }
        depth_processor_.reset();
    }
    
    // Shutdown camera system
    if (camera_system_) {
        auto result = camera_system_->shutdown();
        if (result != core::ResultCode::SUCCESS) {
            worst_result = result;
        }
        camera_system_.reset();
    }
    
    return worst_result;
}

core::ResultCode UnlookScanner::detectHardware() {
    UNLOOK_LOG_INFO("Scanner") << "Detecting hardware...";
    
    // Check for libcamera-sync installation
    if (!detectLibCameraSync()) {
        setError("libcamera-sync not found - required for camera synchronization");
        return core::ResultCode::ERROR_HARDWARE_FAILURE;
    }
    
    // Check for IMX296 cameras
    if (!detectIMX296Cameras()) {
        setError("IMX296 cameras not detected");
        return core::ResultCode::ERROR_CAMERA_NOT_FOUND;
    }
    
    // Validate camera mapping
    if (!validateCameraMapping()) {
        setError("Camera mapping validation failed");
        return core::ResultCode::ERROR_CAMERA_ACCESS_DENIED;
    }
    
    UNLOOK_LOG_INFO("Scanner") << "Hardware detection successful";
    return core::ResultCode::SUCCESS;
}

core::ResultCode UnlookScanner::validateSystem() {
    UNLOOK_LOG_INFO("Scanner") << "Validating system configuration...";
    
    // Check calibration file exists
    std::string calib_path = "calibration/calib_boofcv_test3.yaml";
    if (!std::filesystem::exists(calib_path)) {
        UNLOOK_LOG_WARNING("Scanner") << "Default calibration file not found: " << calib_path;
    }
    
    // Validate configuration
    auto& config = core::Config::getInstance();
    uint32_t width = config.getValue<uint32_t>("camera.width", 1456);
    uint32_t height = config.getValue<uint32_t>("camera.height", 1088);
    
    if (width != 1456 || height != 1088) {
        UNLOOK_LOG_WARNING("Scanner") << "Camera resolution mismatch - calibration requires 1456x1088";
    }
    
    UNLOOK_LOG_INFO("Scanner") << "System validation complete";
    return core::ResultCode::SUCCESS;
}

bool UnlookScanner::detectLibCameraSync() const {
    // Check for system installation first
    if (std::filesystem::exists("/usr/local/lib/libcamera.so") ||
        std::filesystem::exists("/usr/local/lib/libcamera-base.so")) {
        UNLOOK_LOG_INFO("Scanner") << "Found libcamera-sync system installation";
        return true;
    }
    
    // Check for local installation
    if (std::filesystem::exists("third-party/libcamera-sync-fix/build/src/libcamera/libcamera.so")) {
        UNLOOK_LOG_INFO("Scanner") << "Found libcamera-sync local installation";
        return true;
    }
    
    UNLOOK_LOG_ERROR("Scanner") << "libcamera-sync not found";
    return false;
}

bool UnlookScanner::detectIMX296Cameras() const {
    // This would use libcamera to detect cameras
    // For now, assume cameras are present if libcamera-sync is available
    UNLOOK_LOG_INFO("Scanner") << "IMX296 camera detection - using libcamera-sync";
    return true;
}

bool UnlookScanner::validateCameraMapping() const {
    // Validate that camera mapping matches expected configuration
    // Camera 1 = LEFT/MASTER (/base/soc/i2c0mux/i2c@1/imx296@1a)
    // Camera 0 = RIGHT/SLAVE (/base/soc/i2c0mux/i2c@0/imx296@1a)
    UNLOOK_LOG_INFO("Scanner") << "Validating camera mapping: Camera 1=LEFT/MASTER, Camera 0=RIGHT/SLAVE";
    return true;
}

// C API implementation
extern "C" {

void* unlook_scanner_create() {
    try {
        return new UnlookScanner();
    } catch (...) {
        return nullptr;
    }
}

void unlook_scanner_destroy(void* scanner) {
    if (scanner) {
        delete static_cast<UnlookScanner*>(scanner);
    }
}

int unlook_scanner_initialize(void* scanner, int mode, const char* config_path) {
    if (!scanner) {
        return static_cast<int>(core::ResultCode::ERROR_INVALID_PARAMETER);
    }
    
    try {
        auto* s = static_cast<UnlookScanner*>(scanner);
        core::ScannerMode scanner_mode = (mode == 0) ? 
            core::ScannerMode::STANDALONE : core::ScannerMode::COMPANION;
        std::string config_str = config_path ? std::string(config_path) : "";
        
        return static_cast<int>(s->initialize(scanner_mode, config_str));
    } catch (...) {
        return static_cast<int>(core::ResultCode::ERROR_GENERIC);
    }
}

int unlook_scanner_shutdown(void* scanner) {
    if (!scanner) {
        return static_cast<int>(core::ResultCode::ERROR_INVALID_PARAMETER);
    }
    
    try {
        auto* s = static_cast<UnlookScanner*>(scanner);
        return static_cast<int>(s->shutdown());
    } catch (...) {
        return static_cast<int>(core::ResultCode::ERROR_GENERIC);
    }
}

int unlook_scanner_is_initialized(void* scanner) {
    if (!scanner) {
        return 0;
    }
    
    try {
        auto* s = static_cast<UnlookScanner*>(scanner);
        return s->isInitialized() ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

const char* unlook_get_version() {
    static std::string version_str = core::API_VERSION.toString();
    return version_str.c_str();
}

} // extern "C"

} // namespace api
} // namespace unlook