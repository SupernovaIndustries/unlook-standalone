#include <yaml-cpp/yaml.h>
#include <string>
#include <map>

namespace unlook {
namespace core {

class Configuration {
public:
    static Configuration& getInstance() {
        static Configuration instance;
        return instance;
    }
    
    bool load(const std::string& filename) {
        try {
            config = YAML::LoadFile(filename);
            return true;
        } catch (const YAML::Exception& e) {
            return false;
        }
    }
    
    template<typename T>
    T get(const std::string& key, const T& defaultValue) const {
        try {
            return config[key].as<T>();
        } catch (...) {
            return defaultValue;
        }
    }
    
private:
    Configuration() = default;
    YAML::Node config;
};

} // namespace core
} // namespace unlook