#include <cmath>
#include <algorithm>
#include <vector>

namespace unlook {
namespace utils {

class MathUtils {
public:
    static double degToRad(double degrees) {
        return degrees * M_PI / 180.0;
    }
    
    static double radToDeg(double radians) {
        return radians * 180.0 / M_PI;
    }
    
    static double clamp(double value, double min, double max) {
        return std::max(min, std::min(max, value));
    }
    
    static double median(std::vector<double> values) {
        if (values.empty()) return 0.0;
        
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        
        if (n % 2 == 0) {
            return (values[n/2 - 1] + values[n/2]) / 2.0;
        } else {
            return values[n/2];
        }
    }
};

} // namespace utils
} // namespace unlook