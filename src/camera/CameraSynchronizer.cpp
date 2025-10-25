#include <unlook/camera/CameraSynchronizer.hpp>
#include <unlook/core/Logger.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cmath>

namespace unlook {
namespace camera {

// GPIO pins for Raspberry Pi (adjust as needed)
constexpr int DEFAULT_XVS_PIN = 17;  // GPIO17 for XVS
constexpr int DEFAULT_XHS_PIN = 27;  // GPIO27 for XHS (Camera sync - different from AS1170 strobe GPIO 4)  
constexpr int DEFAULT_MAS_PIN = 22;  // GPIO22 for Master/Slave

CameraSynchronizer::CameraSynchronizer()
    : syncMode_(SyncMode::NONE)
    , syncFrequency_(30.0) {
    
    stats_ = {};
    LOG_DEBUG("CameraSynchronizer constructor");
}

CameraSynchronizer::~CameraSynchronizer() {
    LOG_DEBUG("CameraSynchronizer destructor");
    shutdown();
}

bool CameraSynchronizer::initialize(SyncMode mode) {
    if (initialized_) {
        LOG_WARNING("Synchronizer already initialized");
        return true;
    }
    
    LOG_INFO("Initializing synchronizer with mode: " + 
             std::to_string(static_cast<int>(mode)));
    
    syncMode_ = mode;
    
    if (mode == SyncMode::HARDWARE_XVS || mode == SyncMode::HARDWARE_FULL) {
        // Initialize GPIO for hardware sync
        if (!initializeGPIO()) {
            LOG_ERROR("Failed to initialize GPIO for hardware sync");
            
            // Fallback to software sync
            LOG_WARNING("Falling back to software synchronization");
            syncMode_ = SyncMode::SOFTWARE;
        }
    }
    
    // Calculate sync period based on frequency
    syncPeriod_ = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / syncFrequency_));
    
    initialized_ = true;
    synchronized_ = false;
    
    LOG_INFO("Synchronizer initialized successfully");
    return true;
}

void CameraSynchronizer::shutdown() {
    if (!initialized_) {
        return;
    }
    
    LOG_INFO("Shutting down synchronizer");
    
    // Stop sync thread if running
    stop();
    
    // Cleanup GPIO
    cleanupGPIO();
    
    initialized_ = false;
    synchronized_ = false;
    
    LOG_INFO("Synchronizer shutdown complete");
}

bool CameraSynchronizer::start() {
    if (!initialized_) {
        LOG_ERROR("Synchronizer not initialized");
        return false;
    }
    
    if (running_) {
        LOG_WARNING("Synchronizer already running");
        return true;
    }
    
    LOG_INFO("Starting synchronizer");
    
    // Start sync pulse thread for hardware modes
    if (syncMode_ == SyncMode::HARDWARE_XVS || syncMode_ == SyncMode::HARDWARE_FULL) {
        shouldStop_ = false;
        syncThread_ = std::thread(&CameraSynchronizer::syncPulseThread, this);
    }
    
    running_ = true;
    lastSyncTime_ = std::chrono::steady_clock::now();
    
    LOG_INFO("Synchronizer started");
    return true;
}

void CameraSynchronizer::stop() {
    if (!running_) {
        return;
    }
    
    LOG_INFO("Stopping synchronizer");
    
    // Stop sync thread
    shouldStop_ = true;
    if (syncThread_.joinable()) {
        syncThread_.join();
    }
    
    running_ = false;
    
    LOG_INFO("Synchronizer stopped");
}

bool CameraSynchronizer::configureGPIO(int xvsPin, int xhsPin, int masPin) {
    xvsPin_ = (xvsPin >= 0) ? xvsPin : DEFAULT_XVS_PIN;
    xhsPin_ = (xhsPin >= 0) ? xhsPin : DEFAULT_XHS_PIN;
    masPin_ = (masPin >= 0) ? masPin : DEFAULT_MAS_PIN;
    
    LOG_INFO("Configuring GPIO pins - XVS: " + std::to_string(xvsPin_) +
             ", XHS: " + std::to_string(xhsPin_) +
             ", MAS: " + std::to_string(masPin_));
    
    return true;
}

bool CameraSynchronizer::initializeGPIO() {
    LOG_INFO("Initializing GPIO for hardware sync");
    
    // Set default pins if not configured
    if (xvsPin_ < 0) xvsPin_ = DEFAULT_XVS_PIN;
    if (xhsPin_ < 0) xhsPin_ = DEFAULT_XHS_PIN;
    if (masPin_ < 0) masPin_ = DEFAULT_MAS_PIN;
    
    // Export GPIO pins
    if (!exportGPIO(xvsPin_)) {
        LOG_ERROR("Failed to export XVS pin " + std::to_string(xvsPin_));
        return false;
    }
    exportedPins_.push_back(xvsPin_);
    
    if (syncMode_ == SyncMode::HARDWARE_FULL) {
        if (!exportGPIO(xhsPin_)) {
            LOG_ERROR("Failed to export XHS pin " + std::to_string(xhsPin_));
            cleanupGPIO();
            return false;
        }
        exportedPins_.push_back(xhsPin_);
    }
    
    if (!exportGPIO(masPin_)) {
        LOG_ERROR("Failed to export MAS pin " + std::to_string(masPin_));
        cleanupGPIO();
        return false;
    }
    exportedPins_.push_back(masPin_);
    
    // Set pin directions
    if (!setGPIODirection(xvsPin_, true)) {  // XVS as output for master
        LOG_ERROR("Failed to set XVS pin direction");
        cleanupGPIO();
        return false;
    }
    
    if (syncMode_ == SyncMode::HARDWARE_FULL) {
        if (!setGPIODirection(xhsPin_, true)) {  // XHS as output for master
            LOG_ERROR("Failed to set XHS pin direction");
            cleanupGPIO();
            return false;
        }
    }
    
    if (!setGPIODirection(masPin_, true)) {  // MAS as output
        LOG_ERROR("Failed to set MAS pin direction");
        cleanupGPIO();
        return false;
    }
    
    // Set initial pin states
    setGPIOValue(xvsPin_, false);
    if (syncMode_ == SyncMode::HARDWARE_FULL) {
        setGPIOValue(xhsPin_, false);
    }
    
    LOG_INFO("GPIO initialized successfully");
    return true;
}

void CameraSynchronizer::cleanupGPIO() {
    LOG_INFO("Cleaning up GPIO");
    
    for (int pin : exportedPins_) {
        unexportGPIO(pin);
    }
    exportedPins_.clear();
}

bool CameraSynchronizer::exportGPIO(int pin) {
    std::ofstream exportFile("/sys/class/gpio/export");
    if (!exportFile.is_open()) {
        LOG_ERROR("Failed to open GPIO export file");
        return false;
    }
    
    exportFile << pin;
    exportFile.close();
    
    // Wait for the GPIO files to be created
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    return true;
}

bool CameraSynchronizer::unexportGPIO(int pin) {
    std::ofstream unexportFile("/sys/class/gpio/unexport");
    if (!unexportFile.is_open()) {
        LOG_WARNING("Failed to open GPIO unexport file");
        return false;
    }
    
    unexportFile << pin;
    unexportFile.close();
    
    return true;
}

bool CameraSynchronizer::setGPIODirection(int pin, bool output) {
    std::string directionFile = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::ofstream file(directionFile);
    
    if (!file.is_open()) {
        LOG_ERROR("Failed to open GPIO direction file: " + directionFile);
        return false;
    }
    
    file << (output ? "out" : "in");
    file.close();
    
    return true;
}

bool CameraSynchronizer::setGPIOValue(int pin, bool value) {
    std::string valueFile = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream file(valueFile);
    
    if (!file.is_open()) {
        LOG_ERROR("Failed to open GPIO value file: " + valueFile);
        return false;
    }
    
    file << (value ? "1" : "0");
    file.close();
    
    return true;
}

bool CameraSynchronizer::getGPIOValue(int pin) {
    std::string valueFile = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ifstream file(valueFile);
    
    if (!file.is_open()) {
        LOG_ERROR("Failed to open GPIO value file: " + valueFile);
        return false;
    }
    
    int value;
    file >> value;
    file.close();
    
    return value == 1;
}

bool CameraSynchronizer::triggerSync() {
    if (syncMode_ == SyncMode::NONE) {
        return true;  // No sync needed
    }
    
    if (syncMode_ == SyncMode::SOFTWARE) {
        // Software sync - just record timestamp
        lastSyncTime_ = std::chrono::steady_clock::now();
        synchronized_ = true;
        return true;
    }
    
    // Hardware sync - generate pulse
    if (xvsPin_ >= 0) {
        // Generate XVS pulse (active high)
        setGPIOValue(xvsPin_, true);
        std::this_thread::sleep_for(std::chrono::microseconds(10));  // 10us pulse
        setGPIOValue(xvsPin_, false);
        
        if (syncMode_ == SyncMode::HARDWARE_FULL && xhsPin_ >= 0) {
            // Generate XHS pulse for line sync
            setGPIOValue(xhsPin_, true);
            std::this_thread::sleep_for(std::chrono::microseconds(5));  // 5us pulse
            setGPIOValue(xhsPin_, false);
        }
        
        lastSyncTime_ = std::chrono::steady_clock::now();
        synchronized_ = true;
        
        // Update statistics
        stats_.totalSyncs++;
        
        return true;
    }
    
    return false;
}

bool CameraSynchronizer::waitForSync(int timeoutMs) {
    if (syncMode_ == SyncMode::NONE) {
        return true;  // No sync needed
    }
    
    if (syncMode_ == SyncMode::SOFTWARE) {
        // Software sync - just wait for next sync period
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - lastSyncTime_);
        
        if (elapsed < syncPeriod_) {
            std::this_thread::sleep_for(syncPeriod_ - elapsed);
        }
        
        return true;
    }
    
    // Hardware sync - wait for sync pulse on XVS pin (for slave)
    // In a real implementation, this would monitor the GPIO pin
    // For now, we simulate the wait
    auto startTime = std::chrono::steady_clock::now();
    
    while (true) {
        // Check for sync signal (this would read the actual GPIO in production)
        // For simulation, we just wait for the sync period
        
        auto now = std::chrono::steady_clock::now();
        auto waitTime = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime);
        
        if (waitTime.count() >= timeoutMs) {
            stats_.failedSyncs++;
            return false;  // Timeout
        }
        
        // Check if sync pulse received (simulated)
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - lastSyncTime_);
        if (elapsed >= syncPeriod_) {
            synchronized_ = true;
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void CameraSynchronizer::syncPulseThread() {
    LOG_INFO("Sync pulse thread started");
    
    auto nextSync = std::chrono::steady_clock::now();
    
    while (!shouldStop_) {
        // Wait until next sync time
        std::this_thread::sleep_until(nextSync);
        
        if (shouldStop_) {
            break;
        }
        
        // Trigger sync pulse
        triggerSync();
        
        // Calculate next sync time
        nextSync += syncPeriod_;
    }
    
    LOG_INFO("Sync pulse thread stopped");
}

double CameraSynchronizer::measureSyncError(uint64_t timestamp1, uint64_t timestamp2) const {
    int64_t diff = static_cast<int64_t>(timestamp1) - static_cast<int64_t>(timestamp2);
    return std::abs(diff) / 1000000.0;  // Convert ns to ms
}

bool CameraSynchronizer::validateSync(double errorToleranceMs) const {
    if (!synchronized_) {
        return false;
    }
    
    return stats_.avgSyncErrorMs <= errorToleranceMs;
}

CameraSynchronizer::SyncStats CameraSynchronizer::getStats() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    return stats_;
}

void CameraSynchronizer::resetStats() {
    std::lock_guard<std::mutex> lock(statsMutex_);
    stats_ = {};
}

bool CameraSynchronizer::setSyncFrequency(double frequencyHz) {
    if (frequencyHz <= 0 || frequencyHz > 120) {
        LOG_ERROR("Invalid sync frequency: " + std::to_string(frequencyHz));
        return false;
    }
    
    syncFrequency_ = frequencyHz;
    syncPeriod_ = std::chrono::microseconds(static_cast<int64_t>(1000000.0 / frequencyHz));
    
    LOG_INFO("Sync frequency set to " + std::to_string(frequencyHz) + " Hz");
    return true;
}

void CameraSynchronizer::updateStats(double syncErrorMs) {
    std::lock_guard<std::mutex> lock(statsMutex_);
    
    stats_.totalSyncs++;
    
    if (syncErrorMs > 1.0) {  // Consider >1ms as failed sync
        stats_.failedSyncs++;
    }
    
    // Update min/max
    if (stats_.totalSyncs == 1) {
        stats_.minSyncErrorMs = syncErrorMs;
        stats_.maxSyncErrorMs = syncErrorMs;
        stats_.avgSyncErrorMs = syncErrorMs;
    } else {
        stats_.minSyncErrorMs = std::min(stats_.minSyncErrorMs, syncErrorMs);
        stats_.maxSyncErrorMs = std::max(stats_.maxSyncErrorMs, syncErrorMs);
        
        // Running average
        stats_.avgSyncErrorMs = (stats_.avgSyncErrorMs * (stats_.totalSyncs - 1) + syncErrorMs) / stats_.totalSyncs;
    }
    
    stats_.lastSyncTime = std::chrono::steady_clock::now();
}

bool CameraSynchronizer::configureIMX296Sync(bool master) {
    LOG_INFO("Configuring IMX296 sync for " + std::string(master ? "MASTER" : "SLAVE"));
    
    // IMX296 sensor-specific sync configuration
    // This would typically involve I2C register writes to the sensor
    // The actual implementation is in libcamera-sync which handles this
    
    return true;
}

bool CameraSynchronizer::enableXVS(bool master) {
    LOG_INFO("Enabling XVS for " + std::string(master ? "MASTER" : "SLAVE"));
    
    if (master) {
        // Configure XVS pin as output
        return setGPIODirection(xvsPin_, true);
    } else {
        // Configure XVS pin as input
        return setGPIODirection(xvsPin_, false);
    }
}

bool CameraSynchronizer::enableXHS(bool master) {
    LOG_INFO("Enabling XHS for " + std::string(master ? "MASTER" : "SLAVE"));
    
    if (master) {
        // Configure XHS pin as output
        return setGPIODirection(xhsPin_, true);
    } else {
        // Configure XHS pin as input
        return setGPIODirection(xhsPin_, false);
    }
}

bool CameraSynchronizer::setMASPin(bool master) {
    LOG_INFO("Setting MAS pin for " + std::string(master ? "MASTER" : "SLAVE"));
    
    // MAS pin: HIGH for master, LOW for slave
    return setGPIOValue(masPin_, master);
}

} // namespace camera
} // namespace unlook