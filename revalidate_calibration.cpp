#include "unlook/calibration/StereoCalibrationProcessor.hpp"
#include "unlook/calibration/CalibrationValidator.hpp"
#include "unlook/core/Logger.hpp"
#include <iostream>

int main() {
    unlook::core::Logger::getInstance().initialize(unlook::core::LogLevel::INFO);
    
    unlook::calibration::StereoCalibrationProcessor processor;
    
    // Load existing calibration
    auto result = processor.loadCalibration("/unlook_calib/calib-20251106_005327.yaml");
    
    std::cout << "=== LOADED CALIBRATION ===" << std::endl;
    std::cout << "RMS: " << result.rmsReprojectionError << " px" << std::endl;
    std::cout << "Baseline: " << result.baselineMM << " mm" << std::endl;
    std::cout << "Mean epipolar error: " << result.meanEpipolarError << " px" << std::endl;
    std::cout << "Max epipolar error: " << result.maxEpipolarError << " px" << std::endl;
    
    // Re-validate with NEW thresholds
    std::cout << "\n=== RE-VALIDATION WITH NEW THRESHOLDS ===" << std::endl;
    unlook::calibration::CalibrationValidator validator;
    bool passed = validator.validate(result);
    
    std::cout << "Quality passed: " << (result.qualityPassed ? "YES" : "NO") << std::endl;
    std::cout << "RMS check: " << result.rmsCheck << std::endl;
    std::cout << "Baseline check: " << result.baselineCheck << std::endl;
    std::cout << "Epipolar check: " << result.epipolarCheck << std::endl;
    
    return 0;
}
