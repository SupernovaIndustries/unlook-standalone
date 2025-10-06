#include <opencv2/opencv.hpp>
#include <iostream>
#include "unlook/calibration/CalibrationManager.hpp"

int main() {
    std::cout << "Testing CalibrationManager loading and rectification..." << std::endl;

    // Create calibration manager
    auto calibManager = std::make_shared<unlook::calibration::CalibrationManager>();

    // Load calibration
    std::string calibFile = "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml";
    std::cout << "\nLoading calibration from: " << calibFile << std::endl;

    if (!calibManager->loadCalibration(calibFile)) {
        std::cerr << "Failed to load calibration!" << std::endl;
        return 1;
    }

    std::cout << "\nCalibration loaded successfully!" << std::endl;
    std::cout << "Baseline: " << calibManager->getBaselineMm() << "mm" << std::endl;
    std::cout << "RMS Error: " << calibManager->getRmsError() << "px" << std::endl;

    // Create dummy images for rectification
    cv::Mat leftImage = cv::Mat::zeros(1088, 1456, CV_8UC1);
    cv::Mat rightImage = cv::Mat::zeros(1088, 1456, CV_8UC1);
    cv::Mat leftRect, rightRect;

    std::cout << "\n--- Testing rectifyImages() ---" << std::endl;
    if (calibManager->rectifyImages(leftImage, rightImage, leftRect, rightRect)) {
        std::cout << "Rectification successful!" << std::endl;
    } else {
        std::cerr << "Rectification failed!" << std::endl;
        return 1;
    }

    return 0;
}