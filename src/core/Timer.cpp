#include <chrono>
#include <string>
#include <map>

namespace unlook {
namespace core {

class Timer {
public:
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    
    void start(const std::string& name = "default") {
        startTimes[name] = Clock::now();
    }
    
    double stop(const std::string& name = "default") {
        auto endTime = Clock::now();
        auto it = startTimes.find(name);
        if (it != startTimes.end()) {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - it->second);
            return duration.count() / 1000.0;  // Return milliseconds
        }
        return 0.0;
    }
    
    double elapsed(const std::string& name = "default") const {
        auto now = Clock::now();
        auto it = startTimes.find(name);
        if (it != startTimes.end()) {
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - it->second);
            return duration.count() / 1000.0;  // Return milliseconds
        }
        return 0.0;
    }
    
private:
    std::map<std::string, TimePoint> startTimes;
};

} // namespace core
} // namespace unlook