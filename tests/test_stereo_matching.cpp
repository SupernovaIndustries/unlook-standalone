#include <iostream>
#include <opencv2/opencv.hpp>
#include "unlook/stereo/SGBMStereoMatcher.hpp"

int main() {
    std::cout << "Testing Stereo Matching..." << std::endl;
    
    // Create stereo matcher
    unlook::stereo::SGBMStereoMatcher matcher;
    
    // Set parameters
    unlook::stereo::StereoMatchingParams params;
    params.numDisparities = 128;
    params.blockSize = 11;
    
    if (matcher.setParameters(params)) {
        std::cout << " Parameters set successfully" << std::endl;
    }
    
    // Create test images
    cv::Mat left = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right = cv::Mat::zeros(480, 640, CV_8UC1);
    
    // Add some features
    cv::rectangle(left, cv::Point(100, 100), cv::Point(200, 200), cv::Scalar(255), -1);
    cv::rectangle(right, cv::Point(90, 100), cv::Point(190, 200), cv::Scalar(255), -1);
    
    // Compute disparity
    cv::Mat disparity;
    if (matcher.computeDisparity(left, right, disparity)) {
        std::cout << " Disparity computed successfully" << std::endl;
        
        // Check disparity statistics
        double minVal, maxVal;
        cv::minMaxLoc(disparity, &minVal, &maxVal);
        std::cout << "  Disparity range: " << minVal << " to " << maxVal << std::endl;
        
        return 0;
    } else {
        std::cout << " Failed to compute disparity" << std::endl;
        return 1;
    }
}