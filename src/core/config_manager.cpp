#include <map>
#include <string>

// Minimal config manager implementation - placeholder for now
namespace unlook {
namespace core {

class ConfigManager {
private:
    std::map<std::string, std::string> config_data_;
    
public:
    static ConfigManager& getInstance() {
        static ConfigManager instance;
        return instance;
    }
    
    void setValue(const std::string& key, const std::string& value) {
        config_data_[key] = value;
    }
    
    std::string getValue(const std::string& key, const std::string& default_value = "") const {
        auto it = config_data_.find(key);
        return (it != config_data_.end()) ? it->second : default_value;
    }
};

} // namespace core
} // namespace unlook