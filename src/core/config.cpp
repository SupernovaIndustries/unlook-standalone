#include "unlook/core/config.h"
#include "unlook/core/exception.h"
#include "unlook/core/Logger.hpp"
#include <fstream>
#include <sstream>

namespace unlook {
namespace core {

Config& Config::getInstance() {
    static Config instance;
    return instance;
}

ResultCode Config::loadFromFile(const std::string& config_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ifstream file(config_path);
    if (!file.is_open()) {
        return ResultCode::ERROR_FILE_NOT_FOUND;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // Find the equals sign
        size_t equals_pos = line.find('=');
        if (equals_pos == std::string::npos) {
            continue;
        }
        
        std::string key = line.substr(0, equals_pos);
        std::string value_str = line.substr(equals_pos + 1);
        
        // Trim whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value_str.erase(0, value_str.find_first_not_of(" \t"));
        value_str.erase(value_str.find_last_not_of(" \t") + 1);
        
        // Parse value based on type hints or content
        ConfigValue value;
        
        if (value_str == "true" || value_str == "false") {
            value = (value_str == "true");
        } else if (value_str.find('.') != std::string::npos) {
            try {
                value = std::stod(value_str);
            } catch (...) {
                value = value_str;
            }
        } else {
            try {
                int32_t int_val = std::stoi(value_str);
                if (int_val >= 0) {
                    value = static_cast<uint32_t>(int_val);
                } else {
                    value = int_val;
                }
            } catch (...) {
                value = value_str;
            }
        }
        
        values_[key] = value;
    }
    
    return ResultCode::SUCCESS;
}

ResultCode Config::saveToFile(const std::string& config_path) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::ofstream file(config_path);
    if (!file.is_open()) {
        return ResultCode::ERROR_FILE_IO;
    }
    
    file << "# Unlook 3D Scanner Configuration File\n";
    file << "# Generated automatically - edit with care\n\n";
    
    for (const auto& [key, value] : values_) {
        file << key << "=";
        
        std::visit([&file](const auto& v) {
            if constexpr (std::is_same_v<std::decay_t<decltype(v)>, bool>) {
                file << (v ? "true" : "false");
            } else {
                file << v;
            }
        }, value);
        
        file << "\n";
    }
    
    return ResultCode::SUCCESS;
}

void Config::setValue(const std::string& key, const ConfigValue& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    values_[key] = value;
}

bool Config::hasKey(const std::string& key) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return values_.find(key) != values_.end();
}

void Config::removeKey(const std::string& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    values_.erase(key);
}

void Config::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    values_.clear();
}

std::vector<std::string> Config::getAllKeys() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> keys;
    keys.reserve(values_.size());
    
    for (const auto& [key, value] : values_) {
        keys.push_back(key);
    }
    
    return keys;
}

void Config::initializeDefaults() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    setDefaultSystemConfig();
    setDefaultCameraConfig();
    setDefaultStereoConfig();
}

void Config::setDefaultSystemConfig() {
    // System configuration defaults
    values_[config_keys::SYSTEM_LOG_LEVEL] = static_cast<int32_t>(LogLevel::INFO);
    values_[config_keys::SYSTEM_LOG_TO_FILE] = true;
    values_[config_keys::SYSTEM_LOG_FILE_PATH] = std::string("unlook.log");
    values_[config_keys::SYSTEM_SCANNER_MODE] = static_cast<int32_t>(ScannerMode::STANDALONE);
}

void Config::setDefaultCameraConfig() {
    // Camera configuration defaults (fixed to calibration resolution)
    values_[config_keys::CAMERA_WIDTH] = 1456u;
    values_[config_keys::CAMERA_HEIGHT] = 1088u;
    values_[config_keys::CAMERA_EXPOSURE_US] = 10000u;
    values_[config_keys::CAMERA_GAIN] = 1.0f;
    values_[config_keys::CAMERA_AUTO_EXPOSURE] = false;
    values_[config_keys::CAMERA_AUTO_GAIN] = false;
    values_[config_keys::CAMERA_SYNC_TIMEOUT_MS] = 5000u;
}

void Config::setDefaultStereoConfig() {
    // Stereo configuration defaults (optimized for Unlook hardware)
    values_[config_keys::STEREO_MIN_DISPARITY] = 0;
    values_[config_keys::STEREO_NUM_DISPARITIES] = 64;
    values_[config_keys::STEREO_BLOCK_SIZE] = 21;
    values_[config_keys::STEREO_P1] = 600;
    values_[config_keys::STEREO_P2] = 2400;
    values_[config_keys::STEREO_DISP12_MAX_DIFF] = 10;
    values_[config_keys::STEREO_PRE_FILTER_CAP] = 4;
    values_[config_keys::STEREO_UNIQUENESS_RATIO] = 5;
    values_[config_keys::STEREO_SPECKLE_WINDOW_SIZE] = 150;
    values_[config_keys::STEREO_SPECKLE_RANGE] = 2;
    values_[config_keys::STEREO_USE_BOOFCV] = true;
    
    // Calibration defaults
    values_[config_keys::CALIBRATION_FILE_PATH] = std::string("/home/alessandro/unlook_calib/default.yaml");
    values_[config_keys::CALIBRATION_AUTO_LOAD] = true;
    values_[config_keys::CALIBRATION_MIN_RMS] = 0.5; // pixels
}

} // namespace core
} // namespace unlook