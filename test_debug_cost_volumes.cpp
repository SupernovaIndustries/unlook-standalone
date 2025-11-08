/**
 * @file test_debug_cost_volumes.cpp
 * @brief Debug AD-Census cost volumes to find geometric pattern bug
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include "unlook/stereo/VCSELStereoMatcher.hpp"

void saveCostVolumeSlice(const cv::Mat& costVolume, const std::string& filename, int disparitySlice) {
    if (costVolume.dims != 3) {
        std::cerr << "ERROR: Cost volume is not 3D!" << std::endl;
        return;
    }

    const int height = costVolume.size[0];
    const int width = costVolume.size[1];
    const int disparities = costVolume.size[2];

    if (disparitySlice >= disparities) {
        std::cerr << "ERROR: Disparity slice out of bounds!" << std::endl;
        return;
    }

    // Extract slice at specific disparity
    cv::Mat slice(height, width, CV_32F);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            const float* costPtr = costVolume.ptr<float>(y, x);
            slice.at<float>(y, x) = costPtr[disparitySlice];
        }
    }

    // Normalize and save
    double minVal, maxVal;
    cv::minMaxLoc(slice, &minVal, &maxVal);
    cv::Mat sliceVis;
    slice.convertTo(sliceVis, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

    cv::imwrite(filename, sliceVis);
    std::cout << "Saved cost slice " << disparitySlice << ": " << filename
              << " (range: " << minVal << "-" << maxVal << ")" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <left_rect.png> <right_rect.png>" << std::endl;
        return 1;
    }

    // Load images
    cv::Mat leftRect = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::Mat rightRect = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

    if (leftRect.empty() || rightRect.empty()) {
        std::cerr << "ERROR: Could not load images!" << std::endl;
        return 1;
    }

    std::cout << "Images loaded: " << leftRect.size() << std::endl;

    // Apply CLAHE for better contrast
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
    cv::Mat leftEnhanced, rightEnhanced;
    clahe->apply(leftRect, leftEnhanced);
    clahe->apply(rightRect, rightEnhanced);

    std::cout << "Contrast enhanced" << std::endl;

    // Create matcher
    unlook::stereo::VCSELStereoMatcher matcher;

    // Compute disparity
    cv::Mat disparity;
    std::cout << "\nComputing disparity with AD-Census..." << std::endl;
    bool success = matcher.computeDisparity(leftEnhanced, rightEnhanced, disparity);

    if (!success) {
        std::cerr << "ERROR: Disparity computation failed!" << std::endl;
        return 1;
    }

    std::cout << "Disparity computed successfully!" << std::endl;

    // Save disparity
    double minVal, maxVal;
    cv::minMaxLoc(disparity, &minVal, &maxVal);
    std::cout << "Disparity range: " << minVal << " - " << maxVal << std::endl;

    cv::Mat disparityVis;
    disparity.convertTo(disparityVis, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    cv::imwrite("debug_disparity.png", disparityVis);
    cv::imwrite("debug_disparity_raw.tiff", disparity);

    std::cout << "\nSaved debug_disparity.png and debug_disparity_raw.tiff" << std::endl;

    // TODO: Add hooks to save intermediate cost volumes
    // For now, this just tests the end-to-end pipeline with enhanced images

    return 0;
}
