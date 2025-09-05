#include <iostream>
#include "unlook/calibration/CalibrationManager.hpp"

int main() {
    std::cout << "Testing Calibration Manager..." << std::endl;
    
    unlook::calibration::CalibrationManager calibManager;
    
    // Test loading calibration
    bool loaded = calibManager.loadCalibration("../calibration/calib_boofcv_test3.yaml");
    
    if (loaded) {
        std::cout << " Calibration loaded successfully" << std::endl;
        std::cout << "  RMS Error: " << calibManager.getRmsError() << " pixels" << std::endl;
        std::cout << "  Baseline: " << calibManager.getBaselineMm() << " mm" << std::endl;
        
        // Test validation
        bool valid = calibManager.validateCalibrationQuality();
        std::cout << " Calibration validation: " << (valid ? "PASSED" : "FAILED") << std::endl;
        
        return 0;
    } else {
        std::cout << " Failed to load calibration" << std::endl;
        return 1;
    }
}