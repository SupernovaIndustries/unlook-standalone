#include <iostream>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/validation/CalibrationValidator.hpp"

using namespace unlook;
using namespace std;

/**
 * @brief Example program demonstrating high-precision stereo calibration and depth processing
 * 
 * This example shows how to:
 * 1. Load calibration from YAML file
 * 2. Validate calibration quality
 * 3. Process stereo images to generate depth maps
 * 4. Achieve industrial-grade precision (0.005mm target)
 */

void printHeader(const string& text) {
    cout << "\n" << string(60, '=') << "\n";
    cout << "  " << text << "\n";
    cout << string(60, '=') << "\n\n";
}

void printCalibrationInfo(const calibration::CalibrationManager& calibMgr) {
    auto& calibData = calibMgr.getCalibrationData();
    
    cout << fixed << setprecision(4);
    cout << "Calibration Information:\n";
    cout << "  Type: " << calibData.calibrationType << "\n";
    cout << "  Image Size: " << calibData.imageSize.width << " x " << calibData.imageSize.height << "\n";
    cout << "  RMS Error: " << calibData.rmsError << " pixels";
    
    // Color code the RMS error
    if (calibData.rmsError < 0.2) {
        cout << " [EXCELLENT - Industrial Grade]\n";
    } else if (calibData.rmsError < 0.5) {
        cout << " [GOOD]\n";
    } else {
        cout << " [NEEDS IMPROVEMENT]\n";
    }
    
    cout << "  Baseline: " << calibData.baselineMm << " mm\n";
    cout << "  Expected Precision: " << calibData.precisionMm << " mm\n";
    
    // Compute field of view
    double fovX, fovY;
    calibMgr.computeFieldOfView(fovX, fovY);
    cout << "  Field of View: " << fovX << "° x " << fovY << "°\n";
}

int main(int argc, char** argv) {
    printHeader("UNLOOK STEREO CALIBRATION SYSTEM");
    cout << "Industrial-Grade 3D Scanner - Target Precision: 0.005mm\n\n";
    
    // Path to calibration file
    string calibrationPath = "calibration/calib_boofcv_test3.yaml";
    if (argc > 1) {
        calibrationPath = argv[1];
    }
    
    // =================================================================
    // Step 1: Load and Validate Calibration
    // =================================================================
    printHeader("STEP 1: LOADING CALIBRATION");
    
    auto calibManager = make_shared<calibration::CalibrationManager>();
    
    cout << "Loading calibration from: " << calibrationPath << "\n";
    if (!calibManager->loadCalibration(calibrationPath)) {
        cerr << "ERROR: Failed to load calibration file!\n";
        return -1;
    }
    
    cout << "Calibration loaded successfully!\n\n";
    printCalibrationInfo(*calibManager);
    
    // =================================================================
    // Step 2: Validate Calibration Quality
    // =================================================================
    printHeader("STEP 2: VALIDATING CALIBRATION QUALITY");
    
    validation::CalibrationValidator validator;
    validator.setCalibration(calibManager);
    validator.setValidationParameters(true, true);  // Strict mode for industrial use
    
    validation::ValidationResult validationResult;
    if (validator.validateComprehensive(validationResult)) {
        cout << "Validation Results:\n";
        cout << "  Overall Score: " << validationResult.overallScore << "/100\n";
        cout << "  Status: " << (validationResult.passed ? "PASSED" : "FAILED") << "\n";
        
        if (!validationResult.warnings.empty()) {
            cout << "\nWarnings:\n";
            for (const auto& warning : validationResult.warnings) {
                cout << "  - " << warning << "\n";
            }
        }
        
        if (!validationResult.errors.empty()) {
            cout << "\nErrors:\n";
            for (const auto& error : validationResult.errors) {
                cout << "  ! " << error << "\n";
            }
        }
    }
    
    // Get detailed validation report
    cout << "\n" << calibManager->getValidationReport();
    
    // =================================================================
    // Step 3: Compute Rectification Maps
    // =================================================================
    printHeader("STEP 3: COMPUTING RECTIFICATION MAPS");
    
    if (!calibManager->computeRectificationMaps(0.0)) {
        cerr << "ERROR: Failed to compute rectification maps!\n";
        return -1;
    }
    
    cout << "Rectification maps computed successfully!\n";
    cout << "Ready for stereo image processing.\n";
    
    // =================================================================
    // Step 4: Setup Stereo Matcher
    // =================================================================
    printHeader("STEP 4: CONFIGURING STEREO MATCHER");
    
    auto stereoMatcher = make_unique<stereo::SGBMStereoMatcher>();
    
    // Configure for high precision
    stereo::StereoMatchingParams params;
    params.minDisparity = 0;
    params.numDisparities = 128;  // Must be divisible by 16
    params.blockSize = 11;
    params.P1 = 8 * 3 * params.blockSize * params.blockSize;
    params.P2 = 32 * 3 * params.blockSize * params.blockSize;
    params.disp12MaxDiff = 1;
    params.uniquenessRatio = 5;
    params.speckleWindowSize = 200;
    params.speckleRange = 2;
    params.mode = cv::StereoSGBM::MODE_SGBM_3WAY;  // Best quality mode
    params.useWLSFilter = true;
    params.leftRightCheck = true;
    
    stereoMatcher->setParameters(params);
    stereoMatcher->setPrecisionMode(true);
    
    cout << "Stereo Matcher Configuration:\n";
    cout << params.toString();
    
    // =================================================================
    // Step 5: Create Test Images (if no real images available)
    // =================================================================
    printHeader("STEP 5: PREPARING TEST IMAGES");
    
    cv::Mat leftImage, rightImage;
    
    // For demonstration, create synthetic test pattern
    // In real use, load actual camera images here
    int width = calibManager->getCalibrationData().imageSize.width;
    int height = calibManager->getCalibrationData().imageSize.height;
    
    // Create checkerboard pattern for testing
    leftImage = cv::Mat::zeros(height, width, CV_8UC1);
    rightImage = cv::Mat::zeros(height, width, CV_8UC1);
    
    int squareSize = 50;
    for (int y = 0; y < height; y += squareSize) {
        for (int x = 0; x < width; x += squareSize) {
            bool white = ((x / squareSize) + (y / squareSize)) % 2 == 0;
            cv::Scalar color = white ? cv::Scalar(255) : cv::Scalar(0);
            cv::rectangle(leftImage, cv::Point(x, y), 
                         cv::Point(x + squareSize, y + squareSize), 
                         color, -1);
            cv::rectangle(rightImage, cv::Point(x, y), 
                         cv::Point(x + squareSize, y + squareSize), 
                         color, -1);
        }
    }
    
    cout << "Test images created: " << width << " x " << height << "\n";
    
    // =================================================================
    // Step 6: Rectify Images
    // =================================================================
    printHeader("STEP 6: RECTIFYING STEREO IMAGES");
    
    cv::Mat leftRectified, rightRectified;
    if (!calibManager->rectifyImages(leftImage, rightImage, 
                                     leftRectified, rightRectified)) {
        cerr << "ERROR: Failed to rectify images!\n";
        return -1;
    }
    
    cout << "Images rectified successfully!\n";
    cout << "Epipolar lines are now horizontal and aligned.\n";
    
    // =================================================================
    // Step 7: Compute Disparity Map
    // =================================================================
    printHeader("STEP 7: COMPUTING DISPARITY MAP");
    
    cv::Mat disparity, confidence;
    auto startTime = chrono::high_resolution_clock::now();
    
    if (!stereoMatcher->computeDisparityWithConfidence(leftRectified, rightRectified,
                                                       disparity, confidence)) {
        cerr << "ERROR: Failed to compute disparity!\n";
        return -1;
    }
    
    auto endTime = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime);
    
    cout << "Disparity computation completed in " << duration.count() << " ms\n";
    
    // Compute quality metrics
    stereo::StereoQualityMetrics metrics;
    stereoMatcher->computeQualityMetrics(disparity, metrics);
    cout << "\nQuality Metrics:\n";
    cout << metrics.toString();
    
    // =================================================================
    // Step 8: Convert to Depth Map
    // =================================================================
    printHeader("STEP 8: GENERATING DEPTH MAP");
    
    cv::Mat depth;
    if (!stereo::StereoMatcher::disparityToDepth(disparity, 
                                                 calibManager->getCalibrationData().Q,
                                                 depth, true)) {
        cerr << "ERROR: Failed to convert disparity to depth!\n";
        return -1;
    }
    
    // Compute depth statistics
    double minDepth, maxDepth;
    cv::minMaxLoc(depth, &minDepth, &maxDepth, nullptr, nullptr, depth > 0);
    
    cout << "Depth Map Statistics:\n";
    cout << "  Min Depth: " << minDepth << " mm\n";
    cout << "  Max Depth: " << maxDepth << " mm\n";
    cout << "  Depth Range: " << (maxDepth - minDepth) << " mm\n";
    
    // Compute expected precision at different depths
    cout << "\nExpected Precision:\n";
    for (double d = 100; d <= 500; d += 100) {
        double precision = validator.computeExpectedPrecision(d);
        cout << "  At " << d << " mm: ±" << precision << " mm\n";
    }
    
    // =================================================================
    // Step 9: Visualize Results (optional)
    // =================================================================
    printHeader("STEP 9: VISUALIZATION");
    
    // Create visualization
    cv::Mat disparityVis, depthVis;
    
    // Normalize disparity for visualization
    double minDisp, maxDisp;
    cv::minMaxLoc(disparity, &minDisp, &maxDisp, nullptr, nullptr, disparity > 0);
    disparity.convertTo(disparityVis, CV_8U, 255.0 / (maxDisp - minDisp), 
                       -minDisp * 255.0 / (maxDisp - minDisp));
    cv::applyColorMap(disparityVis, disparityVis, cv::COLORMAP_JET);
    
    // Normalize depth for visualization
    depth.convertTo(depthVis, CV_8U, 255.0 / (maxDepth - minDepth),
                   -minDepth * 255.0 / (maxDepth - minDepth));
    cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);
    
    cout << "Visualization created.\n";
    cout << "Note: In production, save or display these images.\n";
    
    // Save results (optional)
    // cv::imwrite("disparity.png", disparityVis);
    // cv::imwrite("depth.png", depthVis);
    
    // =================================================================
    // Summary
    // =================================================================
    printHeader("CALIBRATION SYSTEM SUMMARY");
    
    cout << "System Status: OPERATIONAL\n";
    cout << "Calibration Quality: " << 
         (calibManager->validateCalibrationQuality() ? "MEETS REQUIREMENTS" : "NEEDS IMPROVEMENT") << "\n";
    cout << "Target Precision: 0.005 mm\n";
    cout << "Achieved RMS Error: " << calibManager->getRmsError() << " pixels\n";
    cout << "Baseline: " << calibManager->getBaselineMm() << " mm\n";
    
    if (calibManager->getRmsError() < 0.2) {
        cout << "\nStatus: READY FOR INDUSTRIAL USE\n";
    } else {
        cout << "\nStatus: RECALIBRATION RECOMMENDED FOR INDUSTRIAL USE\n";
    }
    
    cout << "\n" << string(60, '=') << "\n";
    cout << "  Unlook 3D Scanner - Precision Matters\n";
    cout << string(60, '=') << "\n\n";
    
    return 0;
}