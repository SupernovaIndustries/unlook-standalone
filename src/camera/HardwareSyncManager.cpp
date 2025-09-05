#include <unlook/camera/HardwareSyncManager.hpp>
#include <unlook/core/logger.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fstream>
#include <thread>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <iostream>

namespace unlook {
namespace camera {

// Temporary simple logging macros for compilation
// Simple logging to avoid macro conflicts  
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARNING
#undef LOG_DEBUG
#define LOG_INFO(msg) std::cout << "[INFO] HardwareSyncManager: " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] HardwareSyncManager: " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[WARN] HardwareSyncManager: " << msg << std::endl
#define LOG_DEBUG(msg) std::cout << "[DEBUG] HardwareSyncManager: " << msg << std::endl

// Temporary implementation class definition
class HardwareSyncManager::Impl {
public:
    Impl() {}
    ~Impl() {}
    // Add members as needed for compilation
    int dummy_member = 0;
};

HardwareSyncManager::HardwareSyncManager()
    : sync_active_(false) {
    LOG_DEBUG("HardwareSyncManager constructor");
}

HardwareSyncManager::~HardwareSyncManager() {
    LOG_DEBUG("HardwareSyncManager destructor");
    stopSync();
    
    // Cleanup GPIO
    cleanupGPIO(xvs_fd_);
    cleanupGPIO(xhs_fd_);
    cleanupGPIO(mas_fd_);
    
    // Unexport GPIOs
    if (status_.initialized) {
        unexportGPIO(config_.xvs_gpio);
        unexportGPIO(config_.xhs_gpio);
        unexportGPIO(config_.mas_gpio);
    }
}

bool HardwareSyncManager::initialize(const SyncConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (status_.initialized) {
        LOG_WARNING("HardwareSyncManager already initialized");
        return true;
    }
    
    LOG_INFO("Initializing hardware synchronization with mode: " + 
             std::to_string(static_cast<int>(config.mode)));
    
    config_ = config;
    status_.current_mode = config.mode;
    
    if (config.mode == SyncMode::DISABLED) {
        LOG_INFO("Hardware sync disabled");
        status_.initialized = true;
        return true;
    }
    
    // Initialize XVS pin (External Vertical Sync)
    if (!initializeGPIO(config.xvs_gpio, "out", xvs_fd_)) {
        status_.error_message = "Failed to initialize XVS GPIO " + std::to_string(config.xvs_gpio);
        LOG_ERROR(status_.error_message);
        return false;
    }
    LOG_INFO("XVS GPIO " + std::to_string(config.xvs_gpio) + " initialized");
    
    // Initialize XHS pin if using full sync
    if (config.mode == SyncMode::XVS_XHS) {
        if (!initializeGPIO(config.xhs_gpio, "out", xhs_fd_)) {
            status_.error_message = "Failed to initialize XHS GPIO " + std::to_string(config.xhs_gpio);
            LOG_ERROR(status_.error_message);
            cleanupGPIO(xvs_fd_);
            return false;
        }
        LOG_INFO("XHS GPIO " + std::to_string(config.xhs_gpio) + " initialized");
    }
    
    // Initialize MAS pin (Master/Slave select) - this is typically read-only
    // as it's soldered on the camera sink, but we initialize for monitoring
    if (!initializeGPIO(config.mas_gpio, "in", mas_fd_)) {
        LOG_WARNING("Failed to initialize MAS GPIO " + std::to_string(config.mas_gpio) + 
                    " (may be hardwired)");
        // Not a fatal error as MAS is often hardwired
    } else {
        bool mas_state = getGPIOState(mas_fd_);
        LOG_INFO("MAS GPIO " + std::to_string(config.mas_gpio) + 
                 " state: " + (mas_state ? "HIGH (Master)" : "LOW (Slave)"));
    }
    
    // Set initial states (sync signals low)
    setGPIOState(xvs_fd_, false);
    if (xhs_fd_ >= 0) {
        setGPIOState(xhs_fd_, false);
    }
    
    status_.initialized = true;
    status_.error_message.clear();
    
    LOG_INFO("Hardware synchronization initialized successfully");
    return true;
}

bool HardwareSyncManager::configureCameraRole(int camera_id, CameraRole role) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!status_.initialized) {
        LOG_ERROR("HardwareSyncManager not initialized");
        return false;
    }
    
    // Camera mapping according to project specs:
    // Camera 1 (-c 1) = /base/soc/i2c0mux/i2c@1/imx296@1a = LEFT/MASTER
    // Camera 0 (-c 2) = /base/soc/i2c0mux/i2c@0/imx296@1a = RIGHT/SLAVE
    
    std::string role_str = (role == CameraRole::MASTER) ? "MASTER" : "SLAVE";
    std::string expected_role;
    
    if (camera_id == 1) {
        expected_role = "MASTER";
        if (role != CameraRole::MASTER) {
            LOG_WARNING("Camera 1 should be MASTER but configured as " + role_str);
        }
    } else if (camera_id == 0) {
        expected_role = "SLAVE";
        if (role != CameraRole::SLAVE) {
            LOG_WARNING("Camera 0 should be SLAVE but configured as " + role_str);
        }
    }
    
    LOG_INFO("Configured camera " + std::to_string(camera_id) + " as " + role_str);
    
    // For master camera, we'll generate sync pulses
    // For slave camera, we'll wait for sync pulses
    // This is handled in the respective camera device implementations
    
    return true;
}

bool HardwareSyncManager::startSync() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!status_.initialized) {
        LOG_ERROR("Cannot start sync: not initialized");
        return false;
    }
    
    if (sync_active_) {
        LOG_WARNING("Sync already active");
        return true;
    }
    
    LOG_INFO("Starting hardware synchronization");
    
    sync_active_ = true;
    status_.sync_active = true;
    sync_start_time_ = std::chrono::steady_clock::now();
    last_sync_time_ = sync_start_time_;
    
    // Reset statistics
    resetStats();
    
    LOG_INFO("Hardware synchronization started");
    return true;
}

void HardwareSyncManager::stopSync() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!sync_active_) {
        return;
    }
    
    LOG_INFO("Stopping hardware synchronization");
    
    sync_active_ = false;
    status_.sync_active = false;
    
    // Set sync signals low
    if (xvs_fd_ >= 0) {
        setGPIOState(xvs_fd_, false);
    }
    if (xhs_fd_ >= 0) {
        setGPIOState(xhs_fd_, false);
    }
    
    // Log final statistics
    if (status_.frame_count > 0) {
        LOG_INFO("Sync statistics - Frames: " + std::to_string(status_.frame_count) +
                 ", Errors: " + std::to_string(status_.sync_errors) +
                 ", Max Error: " + std::to_string(status_.max_sync_error_ms) + "ms");
    }
    
    LOG_INFO("Hardware synchronization stopped");
}

bool HardwareSyncManager::enableXVS(bool enable) {
    if (xvs_fd_ < 0) {
        LOG_ERROR("XVS not initialized");
        return false;
    }
    
    if (setGPIOState(xvs_fd_, enable)) {
        LOG_DEBUG("XVS " + std::string(enable ? "enabled" : "disabled"));
        return true;
    }
    
    return false;
}

bool HardwareSyncManager::enableXHS(bool enable) {
    if (xhs_fd_ < 0) {
        LOG_ERROR("XHS not initialized");
        return false;
    }
    
    if (setGPIOState(xhs_fd_, enable)) {
        LOG_DEBUG("XHS " + std::string(enable ? "enabled" : "disabled"));
        return true;
    }
    
    return false;
}

bool HardwareSyncManager::generateSyncPulse() {
    if (!sync_active_) {
        return false;
    }
    
    auto now = std::chrono::steady_clock::now();
    
    // Generate XVS pulse (frame sync)
    if (xvs_fd_ >= 0) {
        // Pulse high for 100us
        setGPIOState(xvs_fd_, true);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        setGPIOState(xvs_fd_, false);
    }
    
    // Generate XHS pulse if in full sync mode
    if (config_.mode == SyncMode::XVS_XHS && xhs_fd_ >= 0) {
        // Pulse high for 50us
        setGPIOState(xhs_fd_, true);
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        setGPIOState(xhs_fd_, false);
    }
    
    // Update timing
    last_sync_time_ = now;
    status_.frame_count++;
    
    // Calculate actual FPS
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - sync_start_time_).count();
    if (elapsed > 0) {
        status_.measured_fps = (status_.frame_count * 1000.0) / elapsed;
    }
    
    return true;
}

bool HardwareSyncManager::waitForSync(uint32_t timeout_ms) {
    if (!sync_active_) {
        return false;
    }
    
    // In a real implementation, this would monitor the GPIO pins
    // For now, we'll use a simple timeout mechanism
    
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);
    
    while (std::chrono::steady_clock::now() - start < timeout) {
        // Check XVS state
        if (xvs_fd_ >= 0) {
            bool xvs_state = getGPIOState(xvs_fd_);
            if (xvs_state) {
                // Sync signal detected
                return true;
            }
        }
        
        // Small sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    
    return false;  // Timeout
}

double HardwareSyncManager::measureSyncError(uint64_t master_timestamp_ns, uint64_t slave_timestamp_ns) {
    int64_t diff_ns = static_cast<int64_t>(slave_timestamp_ns) - static_cast<int64_t>(master_timestamp_ns);
    double error_ms = std::abs(diff_ns) / 1000000.0;
    
    updateSyncStats(error_ms);
    
    if (error_ms > config_.sync_tolerance_ms) {
        status_.sync_errors++;
        if (config_.enable_diagnostics) {
            LOG_WARNING("Sync error exceeded tolerance: " + std::to_string(error_ms) + "ms");
        }
    }
    
    return error_ms;
}

void HardwareSyncManager::updateSyncStats(double error_ms) {
    status_.sync_error_ms = error_ms;
    
    if (error_ms > status_.max_sync_error_ms) {
        status_.max_sync_error_ms = error_ms;
    }
}

HardwareSyncManager::SyncStatus HardwareSyncManager::getStatus() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

void HardwareSyncManager::resetStats() {
    status_.frame_count = 0;
    status_.sync_errors = 0;
    status_.sync_error_ms = 0.0;
    status_.max_sync_error_ms = 0.0;
    status_.measured_fps = 0.0;
}

bool HardwareSyncManager::isHardwareSyncSupported() {
    // Check if we're on a Raspberry Pi with GPIO access
    struct stat st;
    return (stat("/sys/class/gpio", &st) == 0 && S_ISDIR(st.st_mode));
}

bool HardwareSyncManager::getXVSState() const {
    if (xvs_fd_ < 0) {
        return false;
    }
    return getGPIOState(xvs_fd_);
}

bool HardwareSyncManager::getXHSState() const {
    if (xhs_fd_ < 0) {
        return false;
    }
    return getGPIOState(xhs_fd_);
}

bool HardwareSyncManager::getMASState() const {
    if (mas_fd_ < 0) {
        return false;
    }
    return getGPIOState(mas_fd_);
}

bool HardwareSyncManager::initializeGPIO(uint32_t gpio, const std::string& direction, int& fd) {
    // Export GPIO
    if (!exportGPIO(gpio)) {
        return false;
    }
    
    // Set direction
    std::string direction_path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/direction";
    std::ofstream dir_file(direction_path);
    if (!dir_file) {
        LOG_ERROR("Failed to open GPIO direction file: " + direction_path);
        return false;
    }
    dir_file << direction;
    dir_file.close();
    
    // Open value file
    std::string value_path = "/sys/class/gpio/gpio" + std::to_string(gpio) + "/value";
    if (direction == "out") {
        fd = open(value_path.c_str(), O_WRONLY);
    } else {
        fd = open(value_path.c_str(), O_RDONLY);
    }
    
    if (fd < 0) {
        LOG_ERROR("Failed to open GPIO value file: " + value_path);
        return false;
    }
    
    return true;
}

void HardwareSyncManager::cleanupGPIO(int& fd) {
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
}

bool HardwareSyncManager::setGPIOState(int fd, bool state) {
    if (fd < 0) {
        return false;
    }
    
    const char* value = state ? "1" : "0";
    if (write(fd, value, 1) != 1) {
        LOG_ERROR("Failed to write GPIO state");
        return false;
    }
    
    return true;
}

bool HardwareSyncManager::getGPIOState(int fd) const {
    if (fd < 0) {
        return false;
    }
    
    char value;
    lseek(fd, 0, SEEK_SET);
    if (read(fd, &value, 1) != 1) {
        return false;
    }
    
    return (value == '1');
}

bool HardwareSyncManager::exportGPIO(uint32_t gpio) {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file) {
        LOG_ERROR("Failed to open GPIO export file");
        return false;
    }
    
    export_file << gpio;
    export_file.close();
    
    // Check if export was successful
    std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio);
    struct stat st;
    
    // Give the system time to create the GPIO files
    for (int i = 0; i < 10; ++i) {
        if (stat(gpio_path.c_str(), &st) == 0) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // GPIO might already be exported, which is okay
    return (stat(gpio_path.c_str(), &st) == 0);
}

void HardwareSyncManager::unexportGPIO(uint32_t gpio) {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (unexport_file) {
        unexport_file << gpio;
        unexport_file.close();
    }
}

} // namespace camera
} // namespace unlook