#include <iostream>
#include <opencv2/opencv.hpp>
#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/DepthProcessor.hpp"

using namespace unlook;
using namespace std;

int main(int argc, char** argv) {
    cout << "=== UNLOOK DEPTH PROCESSING EXAMPLE ===" << endl;
    cout << "Industrial 3D Scanner - Depth Map Generation\n" << endl;
    
    // Load calibration
    auto calibManager = make_shared<calibration::CalibrationManager>();
    
    string calibrationPath = "calibration/calib_boofcv_test3.yaml";
    if (!calibManager->loadCalibration(calibrationPath)) {
        cerr << "Failed to load calibration!" << endl;
        return -1;
    }
    
    cout << "Calibration loaded successfully" << endl;
    cout << "Baseline: " << calibManager->getBaselineMm() << " mm" << endl;
    cout << "RMS Error: " << calibManager->getRmsError() << " pixels\n" << endl;
    
    // Initialize depth processor
    stereo::DepthProcessor depthProcessor;
    if (!depthProcessor.initialize(calibManager)) {
        cerr << "Failed to initialize depth processor!" << endl;
        return -1;
    }
    
    // Set stereo matcher
    depthProcessor.setStereoMatcher(stereo::StereoAlgorithm::SGBM);
    
    // Configure depth processing
    stereo::DepthProcessingConfig config;
    config.minDepthMm = 50.0f;
    config.maxDepthMm = 500.0f;
    config.applyMedianFilter = true;
    config.applyBilateralFilter = true;
    depthProcessor.setConfiguration(config);
    
    cout << "Depth processor configured" << endl;
    cout << config.toString() << endl;
    
    // Create test images
    int width = calibManager->getCalibrationData().imageSize.width;
    int height = calibManager->getCalibrationData().imageSize.height;
    
    cv::Mat leftImage = cv::Mat::zeros(height, width, CV_8UC1);
    cv::Mat rightImage = cv::Mat::zeros(height, width, CV_8UC1);
    
    // Generate test pattern
    for (int y = 0; y < height; y += 50) {
        for (int x = 0; x < width; x += 50) {
            cv::rectangle(leftImage, cv::Point(x, y), cv::Point(x+40, y+40), 
                         cv::Scalar(255), -1);
            cv::rectangle(rightImage, cv::Point(x+5, y), cv::Point(x+45, y+40), 
                         cv::Scalar(255), -1);
        }
    }
    
    cout << "Processing stereo pair..." << endl;
    
    // Process stereo pair
    cv::Mat depthMap;
    if (!depthProcessor.processStereoPair(leftImage, rightImage, depthMap)) {
        cerr << "Failed to process stereo pair!" << endl;
        cerr << "Error: " << depthProcessor.getLastError() << endl;
        return -1;
    }
    
    cout << "Depth map generated successfully" << endl;
    
    // Compute statistics
    stereo::DepthStatistics stats;
    depthProcessor.computeDepthStatistics(depthMap, stats);
    cout << stats.toString() << endl;
    
    // Compute expected precision
    cout << "Expected Depth Precision:" << endl;
    for (double depth = 100; depth <= 300; depth += 50) {
        double precision = depthProcessor.computeDepthPrecision(depth);
        cout << "  At " << depth << " mm: ±" << precision << " mm" << endl;
    }
    
    // Visualize depth map
    cv::Mat depthVis;
    stereo::DepthProcessor::visualizeDepthMap(depthMap, depthVis, cv::COLORMAP_JET);
    
    cout << "\nDepth processing complete!" << endl;
    cout << "Target precision: 0.005 mm" << endl;
    
    return 0;
}