/**
 * @file test_opencv_sgbm.cpp
 * @brief Quick test using OpenCV StereoSGBM on rectified images
 *
 * This test uses standard OpenCV SGBM instead of AD-Census to diagnose
 * whether the problem is in the algorithm or in the images/calibration.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <left_rectified.png> <right_rectified.png>" << std::endl;
        return 1;
    }

    std::string leftPath = argv[1];
    std::string rightPath = argv[2];

    // Load rectified images
    cv::Mat leftRect = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat rightRect = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

    if (leftRect.empty() || rightRect.empty()) {
        std::cerr << "ERROR: Could not load images!" << std::endl;
        return 1;
    }

    std::cout << "Images loaded:" << std::endl;
    std::cout << "  Left: " << leftRect.size() << ", type: " << leftRect.type() << std::endl;
    std::cout << "  Right: " << rightRect.size() << ", type: " << rightRect.type() << std::endl;

    // Enhance contrast (CLAHE)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat leftEnhanced, rightEnhanced;
    clahe->apply(leftRect, leftEnhanced);
    clahe->apply(rightRect, rightEnhanced);

    std::cout << "\nContrast enhanced with CLAHE" << std::endl;

    // Save enhanced images
    cv::imwrite("test_left_enhanced.png", leftEnhanced);
    cv::imwrite("test_right_enhanced.png", rightEnhanced);

    // Create StereoSGBM matcher
    int minDisparity = 0;
    int numDisparities = 256;  // Must be divisible by 16
    int blockSize = 7;
    int P1 = 8 * blockSize * blockSize;
    int P2 = 32 * blockSize * blockSize;
    int disp12MaxDiff = 2;
    int preFilterCap = 63;
    int uniquenessRatio = 15;
    int speckleWindowSize = 100;
    int speckleRange = 16;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        minDisparity, numDisparities, blockSize,
        P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio,
        speckleWindowSize, speckleRange, cv::StereoSGBM::MODE_SGBM_3WAY
    );

    std::cout << "\nStereoSGBM parameters:" << std::endl;
    std::cout << "  minDisparity: " << minDisparity << std::endl;
    std::cout << "  numDisparities: " << numDisparities << std::endl;
    std::cout << "  blockSize: " << blockSize << std::endl;
    std::cout << "  P1: " << P1 << std::endl;
    std::cout << "  P2: " << P2 << std::endl;

    // Compute disparity on original images
    cv::Mat disparity16_orig, disparity_orig;
    sgbm->compute(leftRect, rightRect, disparity16_orig);
    disparity16_orig.convertTo(disparity_orig, CV_32F, 1.0/16.0);

    std::cout << "\nDisparity computed on ORIGINAL images:" << std::endl;
    double minVal, maxVal;
    cv::minMaxLoc(disparity_orig, &minVal, &maxVal);
    std::cout << "  Min: " << minVal << ", Max: " << maxVal << std::endl;
    int validPixels_orig = cv::countNonZero(disparity_orig > 0);
    std::cout << "  Valid pixels: " << validPixels_orig << " / " << disparity_orig.total()
              << " = " << (100.0 * validPixels_orig / disparity_orig.total()) << "%" << std::endl;

    // Normalize and save
    cv::Mat disparityVis_orig;
    disparity_orig.convertTo(disparityVis_orig, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    cv::imwrite("test_disparity_opencv_sgbm_original.png", disparityVis_orig);
    cv::imwrite("test_disparity_opencv_sgbm_original_raw.tiff", disparity_orig);

    // Compute disparity on enhanced images
    cv::Mat disparity16_enh, disparity_enh;
    sgbm->compute(leftEnhanced, rightEnhanced, disparity16_enh);
    disparity16_enh.convertTo(disparity_enh, CV_32F, 1.0/16.0);

    std::cout << "\nDisparity computed on ENHANCED images:" << std::endl;
    cv::minMaxLoc(disparity_enh, &minVal, &maxVal);
    std::cout << "  Min: " << minVal << ", Max: " << maxVal << std::endl;
    int validPixels_enh = cv::countNonZero(disparity_enh > 0);
    std::cout << "  Valid pixels: " << validPixels_enh << " / " << disparity_enh.total()
              << " = " << (100.0 * validPixels_enh / disparity_enh.total()) << "%" << std::endl;

    // Normalize and save
    cv::Mat disparityVis_enh;
    disparity_enh.convertTo(disparityVis_enh, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    cv::imwrite("test_disparity_opencv_sgbm_enhanced.png", disparityVis_enh);
    cv::imwrite("test_disparity_opencv_sgbm_enhanced_raw.tiff", disparity_enh);

    std::cout << "\n=== Test completed successfully! ===" << std::endl;
    std::cout << "Output files:" << std::endl;
    std::cout << "  test_disparity_opencv_sgbm_original.png" << std::endl;
    std::cout << "  test_disparity_opencv_sgbm_enhanced.png" << std::endl;
    std::cout << "  test_left_enhanced.png" << std::endl;
    std::cout << "  test_right_enhanced.png" << std::endl;

    return 0;
}
