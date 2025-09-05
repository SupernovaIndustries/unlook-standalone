#pragma once

#include "types.hpp"
#include <string>
#include <map>
#include <variant>
#include <mutex>
#include <vector>

/**
 * @file config.h
 * @brief Configuration management system for the Unlook 3D Scanner API
 */

namespace unlook {
namespace core {

/**
 * @brief Configuration value type variant
 */
using ConfigValue = std::variant<bool, int32_t, uint32_t, float, double, std::string>;

/**
 * @brief Thread-safe configuration management
 * 
 * Provides hierarchical configuration with default values,
 * validation, and persistence to file.
 */
class Config {
public:
    /**
     * @brief Get the singleton configuration instance
     * @return Reference to the global configuration
     */
    static Config& getInstance();
    
    /**
     * @brief Load configuration from file
     * @param config_path Path to configuration file
     * @return ResultCode indicating success or failure
     */
    ResultCode loadFromFile(const std::string& config_path);
    
    /**
     * @brief Save configuration to file
     * @param config_path Path to configuration file
     * @return ResultCode indicating success or failure
     */
    ResultCode saveToFile(const std::string& config_path) const;
    
    /**
     * @brief Set configuration value
     * @param key Configuration key (dot-separated hierarchy)
     * @param value Configuration value
     */
    void setValue(const std::string& key, const ConfigValue& value);
    
    /**
     * @brief Get configuration value
     * @param key Configuration key
     * @param default_value Default value if key not found
     * @return Configuration value or default
     */
    template<typename T>
    T getValue(const std::string& key, const T& default_value = T{}) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = values_.find(key);
        if (it != values_.end()) {
            try {
                return std::get<T>(it->second);
            } catch (const std::bad_variant_access&) {
                // Type mismatch, return default
                return default_value;
            }
        }
        return default_value;
    }
    
    /**
     * @brief Check if configuration key exists
     * @param key Configuration key
     * @return True if key exists
     */
    bool hasKey(const std::string& key) const;
    
    /**
     * @brief Remove configuration key
     * @param key Configuration key to remove
     */
    void removeKey(const std::string& key);
    
    /**
     * @brief Clear all configuration values
     */
    void clear();
    
    /**
     * @brief Get all configuration keys
     * @return Vector of all configuration keys
     */
    std::vector<std::string> getAllKeys() const;
    
    /**
     * @brief Initialize with default Unlook configuration
     */
    void initializeDefaults();

private:
    Config() = default;
    ~Config() = default;
    
    // Non-copyable, non-movable
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(Config&&) = delete;
    
    mutable std::mutex mutex_;
    std::map<std::string, ConfigValue> values_;
    
    void setDefaultCameraConfig();
    void setDefaultStereoConfig();
    void setDefaultSystemConfig();
};

/**
 * @brief Convenience class for scoped configuration access
 */
class ConfigSection {
public:
    ConfigSection(const std::string& prefix) : prefix_(prefix) {}
    
    template<typename T>
    T get(const std::string& key, const T& default_value = T{}) const {
        return Config::getInstance().getValue<T>(prefix_ + "." + key, default_value);
    }
    
    template<typename T>
    void set(const std::string& key, const T& value) {
        Config::getInstance().setValue(prefix_ + "." + key, value);
    }

private:
    std::string prefix_;
};

// Default configuration keys
namespace config_keys {
    // System configuration
    constexpr const char* SYSTEM_LOG_LEVEL = "system.log_level";
    constexpr const char* SYSTEM_LOG_TO_FILE = "system.log_to_file";
    constexpr const char* SYSTEM_LOG_FILE_PATH = "system.log_file_path";
    constexpr const char* SYSTEM_SCANNER_MODE = "system.scanner_mode";
    
    // Camera configuration
    constexpr const char* CAMERA_WIDTH = "camera.width";
    constexpr const char* CAMERA_HEIGHT = "camera.height";
    constexpr const char* CAMERA_EXPOSURE_US = "camera.exposure_us";
    constexpr const char* CAMERA_GAIN = "camera.gain";
    constexpr const char* CAMERA_AUTO_EXPOSURE = "camera.auto_exposure";
    constexpr const char* CAMERA_AUTO_GAIN = "camera.auto_gain";
    constexpr const char* CAMERA_SYNC_TIMEOUT_MS = "camera.sync_timeout_ms";
    
    // Stereo configuration
    constexpr const char* STEREO_MIN_DISPARITY = "stereo.min_disparity";
    constexpr const char* STEREO_NUM_DISPARITIES = "stereo.num_disparities";
    constexpr const char* STEREO_BLOCK_SIZE = "stereo.block_size";
    constexpr const char* STEREO_P1 = "stereo.p1";
    constexpr const char* STEREO_P2 = "stereo.p2";
    constexpr const char* STEREO_DISP12_MAX_DIFF = "stereo.disp12_max_diff";
    constexpr const char* STEREO_PRE_FILTER_CAP = "stereo.pre_filter_cap";
    constexpr const char* STEREO_UNIQUENESS_RATIO = "stereo.uniqueness_ratio";
    constexpr const char* STEREO_SPECKLE_WINDOW_SIZE = "stereo.speckle_window_size";
    constexpr const char* STEREO_SPECKLE_RANGE = "stereo.speckle_range";
    constexpr const char* STEREO_USE_BOOFCV = "stereo.use_boofcv";
    
    // Calibration configuration
    constexpr const char* CALIBRATION_FILE_PATH = "calibration.file_path";
    constexpr const char* CALIBRATION_AUTO_LOAD = "calibration.auto_load";
    constexpr const char* CALIBRATION_MIN_RMS = "calibration.min_rms_threshold";
} // namespace config_keys

} // namespace core
} // namespace unlook