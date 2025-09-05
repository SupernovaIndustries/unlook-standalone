#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/validation/CalibrationValidator.hpp"

using namespace unlook;
using namespace std;

int main(int argc, char** argv) {
    cout << "=== UNLOOK CALIBRATION VALIDATION ===" << endl;
    cout << "Industrial-Grade Calibration Quality Assessment\n" << endl;
    
    // Load calibration
    auto calibManager = make_shared<calibration::CalibrationManager>();
    
    string calibrationPath = "calibration/calib_boofcv_test3.yaml";
    if (argc > 1) {
        calibrationPath = argv[1];
    }
    
    cout << "Loading calibration from: " << calibrationPath << endl;
    if (!calibManager->loadCalibration(calibrationPath)) {
        cerr << "ERROR: Failed to load calibration file!" << endl;
        return -1;
    }
    
    cout << "Calibration loaded successfully!\n" << endl;
    
    // Display calibration info
    auto& calibData = calibManager->getCalibrationData();
    cout << fixed << setprecision(4);
    cout << "Calibration Information:" << endl;
    cout << "  Type: " << calibData.calibrationType << endl;
    cout << "  Image Size: " << calibData.imageSize.width << " x " << calibData.imageSize.height << endl;
    cout << "  RMS Error: " << calibData.rmsError << " pixels" << endl;
    cout << "  Baseline: " << calibData.baselineMm << " mm" << endl;
    cout << "  Precision: " << calibData.precisionMm << " mm\n" << endl;
    
    // Initialize validator
    validation::CalibrationValidator validator;
    validator.setCalibration(calibManager);
    validator.setValidationParameters(true, true);  // Strict mode, verbose
    
    cout << "Running comprehensive validation...\n" << endl;
    
    // Perform validation
    validation::ValidationResult result;
    if (!validator.validateComprehensive(result)) {
        cerr << "Validation failed to complete!" << endl;
        return -1;
    }
    
    // Display results
    cout << "VALIDATION RESULTS:" << endl;
    cout << "==================" << endl;
    cout << "Overall Status: " << (result.passed ? "PASSED" : "FAILED") << endl;
    cout << "Overall Score: " << result.overallScore << "/100\n" << endl;
    
    cout << "Detailed Metrics:" << endl;
    cout << "  RMS Error: " << result.rmsError << " pixels";
    if (result.rmsError < 0.2) {
        cout << " [EXCELLENT]" << endl;
    } else if (result.rmsError < 0.5) {
        cout << " [GOOD]" << endl;
    } else {
        cout << " [NEEDS IMPROVEMENT]" << endl;
    }
    
    cout << "  Baseline Error: " << result.baselineError << " mm";
    if (result.baselineError < 0.1) {
        cout << " [EXCELLENT]" << endl;
    } else if (result.baselineError < 0.5) {
        cout << " [ACCEPTABLE]" << endl;
    } else {
        cout << " [WARNING]" << endl;
    }
    
    // Check specific requirements
    cout << "\nIndustrial Requirements Check:" << endl;
    cout << "------------------------------" << endl;
    
    // RMS error requirement
    bool rmsOk = calibManager->getRmsError() < 0.2;
    cout << "  RMS < 0.2 pixels: " << (rmsOk ? "PASS" : "FAIL") << endl;
    
    // Baseline requirement
    bool baselineOk = validator.validateBaseline(70.0, 0.5);
    cout << "  Baseline within 0.5mm: " << (baselineOk ? "PASS" : "FAIL") << endl;
    
    // Parameter consistency
    double consistency = validator.validateParameterConsistency();
    cout << "  Parameter Consistency: " << (consistency * 100) << "%" << endl;
    
    // Expected precision at various depths
    cout << "\nExpected Precision Analysis:" << endl;
    cout << "----------------------------" << endl;
    for (double depth = 50; depth <= 300; depth += 50) {
        double precision = validator.computeExpectedPrecision(depth);
        cout << "  At " << setw(3) << depth << " mm: ±" << setprecision(3) 
             << precision << " mm";
        
        if (depth == 100 && precision > 0.005) {
            cout << " [WARNING: Exceeds 0.005mm target at 100mm]";
        }
        cout << endl;
    }
    
    // Industrial precision validation
    cout << "\nIndustrial Precision Validation:" << endl;
    cout << "--------------------------------" << endl;
    bool meetsIndustrial = validator.validateIndustrialPrecision();
    cout << "  Meets Industrial Standards: " << (meetsIndustrial ? "YES" : "NO") << endl;
    
    // Display warnings and errors
    if (!result.warnings.empty()) {
        cout << "\nWarnings:" << endl;
        for (const auto& warning : result.warnings) {
            cout << "  ⚠ " << warning << endl;
        }
    }
    
    if (!result.errors.empty()) {
        cout << "\nErrors:" << endl;
        for (const auto& error : result.errors) {
            cout << "  ✗ " << error << endl;
        }
    }
    
    // Generate test images for visualization
    cout << "\nGenerating test patterns for visualization..." << endl;
    cv::Mat leftTest, rightTest;
    validator.generateTestImages(leftTest, rightTest, "checkerboard");
    
    // Test epipolar geometry with sample points
    vector<cv::Point2f> testPoints = {
        cv::Point2f(100, 100),
        cv::Point2f(500, 100),
        cv::Point2f(100, 500),
        cv::Point2f(500, 500),
        cv::Point2f(728, 544)  // Center point
    };
    
    validation::EpipolarValidation epipolarVal;
    if (validator.validateEpipolarGeometry(testPoints, testPoints, epipolarVal)) {
        cout << "\nEpipolar Geometry Validation:" << endl;
        cout << "  Mean Distance: " << epipolarVal.meanDistance << " pixels" << endl;
        cout << "  Max Distance: " << epipolarVal.maxDistance << " pixels" << endl;
        cout << "  Std Deviation: " << epipolarVal.stdDistance << " pixels" << endl;
    }
    
    // Final recommendation
    cout << "\n" << string(60, '=') << endl;
    cout << "FINAL RECOMMENDATION:" << endl;
    cout << string(60, '=') << endl;
    
    if (meetsIndustrial && result.passed) {
        cout << "✓ Calibration APPROVED for industrial use" << endl;
        cout << "  The calibration meets all precision requirements" << endl;
        cout << "  for high-accuracy 3D scanning applications." << endl;
    } else if (result.passed) {
        cout << "⚠ Calibration ACCEPTABLE with reservations" << endl;
        cout << "  The calibration is functional but may not meet" << endl;
        cout << "  the strictest industrial precision requirements." << endl;
        cout << "  Consider recalibration for critical applications." << endl;
    } else {
        cout << "✗ Calibration REJECTED" << endl;
        cout << "  The calibration does not meet minimum quality" << endl;
        cout << "  standards. Recalibration is strongly recommended." << endl;
    }
    
    cout << "\nTarget Precision: 0.005 mm" << endl;
    cout << "Achieved RMS: " << calibManager->getRmsError() << " pixels" << endl;
    
    return 0;
}