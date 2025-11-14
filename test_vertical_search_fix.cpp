/**
 * @file test_vertical_search_fix.cpp
 * @brief Test if vertical search range fixes epipolar error issues
 *
 * This test compares Census matching WITH and WITHOUT vertical search
 * to verify that vertical search improves matching quality.
 */

#include "include/unlook/stereo/SGMCensus.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::cout << "\n=== TESTING VERTICAL SEARCH FIX FOR EPIPOLAR ERROR ===\n" << std::endl;

    // Load rectified + CLAHE images
    std::string leftPath = "/home/alessandro/unlook_debug/scan_20251114_205404/01b_clahe_frame0_left.png";
    std::string rightPath = "/home/alessandro/unlook_debug/scan_20251114_205404/01b_clahe_frame0_right.png";

    cv::Mat left = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

    if (left.empty() || right.empty()) {
        std::cerr << "ERROR: Could not load images" << std::endl;
        return 1;
    }

    std::cout << "Loaded images: " << left.cols << "x" << left.rows << std::endl;

    // Test 1: Census WITHOUT vertical search (original)
    std::cout << "\n=== TEST 1: Census WITHOUT vertical search ===" << std::endl;
    unlook::stereo::SGMCensus::Config config1;
    config1.censusWindowSize = 9;
    config1.numDisparities = 384;
    config1.P1 = 8;
    config1.P2 = 32;
    config1.verticalSearchRange = 0;  // NO vertical search (original behavior)
    config1.verbose = false;

    unlook::stereo::SGMCensus census1(config1);
    auto result1 = census1.compute(left, right);

    std::cout << "Valid pixels: " << result1.validPixels << " (" << result1.validPercent << "%)" << std::endl;
    std::cout << "Processing time: " << result1.totalTimeMs << " ms" << std::endl;

    // Save result
    cv::Mat disp1_8bit;
    result1.disparity.convertTo(disp1_8bit, CV_8U, 255.0 / (384.0 * 16.0));
    cv::imwrite("/tmp/census_no_vertical.png", disp1_8bit);

    // Test 2: Census WITH vertical search (±8 pixels)
    std::cout << "\n=== TEST 2: Census WITH vertical search (±8 pixels) ===" << std::endl;
    unlook::stereo::SGMCensus::Config config2;
    config2.censusWindowSize = 9;
    config2.numDisparities = 384;
    config2.P1 = 8;
    config2.P2 = 32;
    config2.verticalSearchRange = 8;  // ±8 pixels for epipolar error compensation
    config2.verbose = false;

    unlook::stereo::SGMCensus census2(config2);
    auto result2 = census2.compute(left, right);

    std::cout << "Valid pixels: " << result2.validPixels << " (" << result2.validPercent << "%)" << std::endl;
    std::cout << "Processing time: " << result2.totalTimeMs << " ms" << std::endl;

    // Save result
    cv::Mat disp2_8bit;
    result2.disparity.convertTo(disp2_8bit, CV_8U, 255.0 / (384.0 * 16.0));
    cv::imwrite("/tmp/census_with_vertical.png", disp2_8bit);

    // Comparison
    std::cout << "\n=== COMPARISON ===" << std::endl;
    std::cout << "WITHOUT vertical search: " << result1.validPercent << "% valid" << std::endl;
    std::cout << "WITH vertical search:    " << result2.validPercent << "% valid" << std::endl;

    double improvement = result2.validPercent - result1.validPercent;
    double improvementRatio = result2.validPercent / result1.validPercent;

    std::cout << "\nImprovement: +" << improvement << "% (ratio: " << improvementRatio << "x)" << std::endl;

    if (improvement > 20.0) {
        std::cout << "\n✓✓✓ EXCELLENT! Vertical search significantly improved matching!" << std::endl;
        std::cout << "    This confirms epipolar error was causing match failures." << std::endl;
    } else if (improvement > 5.0) {
        std::cout << "\n✓ Good improvement - vertical search helps" << std::endl;
    } else if (improvement > 0) {
        std::cout << "\n≈ Minor improvement - vertical search helps a bit" << std::endl;
    } else {
        std::cout << "\n❌ No improvement - epipolar error may not be the main issue" << std::endl;
    }

    std::cout << "\nSaved results to:" << std::endl;
    std::cout << "  /tmp/census_no_vertical.png" << std::endl;
    std::cout << "  /tmp/census_with_vertical.png" << std::endl;

    return 0;
}
