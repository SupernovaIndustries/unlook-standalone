#include <chrono>
#include <thread>

namespace unlook {
namespace utils {

class TimingUtils {
public:
    static uint64_t getCurrentTimeNs() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    }
    
    static uint64_t getCurrentTimeUs() {
        return getCurrentTimeNs() / 1000;
    }
    
    static uint64_t getCurrentTimeMs() {
        return getCurrentTimeNs() / 1000000;
    }
    
    static void sleepMs(int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }
    
    static void sleepUs(int us) {
        std::this_thread::sleep_for(std::chrono::microseconds(us));
    }
};

} // namespace utils
} // namespace unlook