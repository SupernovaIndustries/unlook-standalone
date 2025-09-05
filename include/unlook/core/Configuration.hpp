#pragma once

#include <string>
#include <map>
#include <mutex>
#include <any>
#include <fstream>

namespace unlook {
namespace core {

/**
 * Configuration management class
 * 
 * Handles loading, saving, and accessing configuration parameters
 * with thread-safe operations and hot-reload capability.
 */
class Configuration {
public:
    /**
     * Get singleton instance
     */
    static Configuration& getInstance();
    
    /**
     * Load configuration from file
     */
    bool load(const std::string& filename);
    
    /**
     * Save configuration to file
     */
    bool save(const std::string& filename) const;
    
    /**
     * Reload configuration (hot-reload)
     */
    bool reload();
    
    /**
     * Clear all configuration
     */
    void clear();
    
    // Generic get/set methods
    
    /**
     * Set value
     */
    template<typename T>
    void set(const std::string& key, const T& value) {
        std::lock_guard<std::mutex> lock(mutex_);
        values_[key] = value;
        modified_ = true;
    }
    
    /**
     * Get value
     */
    template<typename T>
    T get(const std::string& key, const T& defaultValue = T{}) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = values_.find(key);
        if (it != values_.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (...) {
                return defaultValue;
            }
        }
        return defaultValue;
    }
    
    /**
     * Check if key exists
     */
    bool has(const std::string& key) const;
    
    /**
     * Remove key
     */
    void remove(const std::string& key);
    
    // Specialized accessors for common types
    
    int getInt(const std::string& key, int defaultValue = 0) const;
    double getDouble(const std::string& key, double defaultValue = 0.0) const;
    bool getBool(const std::string& key, bool defaultValue = false) const;
    std::string getString(const std::string& key, const std::string& defaultValue = "") const;
    
    void setInt(const std::string& key, int value);
    void setDouble(const std::string& key, double value);
    void setBool(const std::string& key, bool value);
    void setString(const std::string& key, const std::string& value);
    
    // Camera-specific configuration helpers
    
    /**
     * Load camera configuration section
     */
    struct CameraConfig {
        int width = 1456;
        int height = 1088;
        double fps = 30.0;
        double exposureUs = 10000.0;
        double gain = 1.0;
        bool autoExposure = true;
        bool hardwareSync = true;
        double syncToleranceMs = 1.0;
        double baseline = 70.017;
    };
    
    CameraConfig getCameraConfig() const;
    void setCameraConfig(const CameraConfig& config);
    
    /**
     * Check if configuration has been modified
     */
    bool isModified() const { return modified_; }
    
    /**
     * Get configuration filename
     */
    std::string getFilename() const { return currentFile_; }
    
private:
    Configuration() = default;
    ~Configuration() = default;
    
    // Delete copy/move
    Configuration(const Configuration&) = delete;
    Configuration& operator=(const Configuration&) = delete;
    
    // Internal methods
    bool parseFile(const std::string& filename);
    bool writeFile(const std::string& filename) const;
    
    // Member variables
    mutable std::mutex mutex_;
    std::map<std::string, std::any> values_;
    std::string currentFile_;
    bool modified_ = false;
};

} // namespace core
} // namespace unlook