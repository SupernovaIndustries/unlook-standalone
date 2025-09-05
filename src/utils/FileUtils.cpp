#include <fstream>
#include <filesystem>
#include <string>

namespace unlook {
namespace utils {

class FileUtils {
public:
    static bool fileExists(const std::string& path) {
        return std::filesystem::exists(path);
    }
    
    static bool createDirectory(const std::string& path) {
        return std::filesystem::create_directories(path);
    }
    
    static std::string getFileExtension(const std::string& filename) {
        std::filesystem::path p(filename);
        return p.extension().string();
    }
};

} // namespace utils
} // namespace unlook