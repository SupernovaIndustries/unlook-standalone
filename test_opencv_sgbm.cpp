/**
 * @file test_opencv_sgbm.cpp
 * @brief Test OpenCV's standard SGBM instead of Census
 *
 * This test compares OpenCV's built-in StereoSGBM against our SGM-Census
 * to determine if Census Transform is the problem or if it's something else.
 */

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

int main() {
    std::cout << "\n=== TESTING OPENCV STANDARD SGBM VS CENSUS ===\n" << std::endl;

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

    // Configure OpenCV SGBM with parameters optimized for VCSEL dots
    const int numDisparities = 384;  // Must be divisible by 16
    const int blockSize = 5;         // 5x5 window (smaller than Census 9x9)

    auto sgbm = cv::StereoSGBM::create(
        0,                  // minDisparity
        numDisparities,     // numDisparities (384 like our Census)
        blockSize,          // blockSize (5x5)
        8 * blockSize * blockSize,   // P1 (small penalty)
        32 * blockSize * blockSize,  // P2 (large penalty)
        1,                  // disp12MaxDiff (consistency check)
        10,                 // preFilterCap
        10,                 // uniquenessRatio (15% in Census, trying 10%)
        100,                // speckleWindowSize (remove small regions)
        32,                 // speckleRange
        cv::StereoSGBM::MODE_SGBM_3WAY  // Full-DP SGM (most accurate)
    );

    std::cout << "\nOpenCV SGBM parameters:" << std::endl;
    std::cout << "  NumDisparities: " << numDisparities << std::endl;
    std::cout << "  BlockSize: " << blockSize << std::endl;
    std::cout << "  P1: " << sgbm->getP1() << std::endl;
    std::cout << "  P2: " << sgbm->getP2() << std::endl;
    std::cout << "  UniquenessRatio: " << sgbm->getUniquenessRatio() << std::endl;
    std::cout << "  Mode: MODE_SGBM_3WAY (full 8-path)" << std::endl;

    // Compute disparity
    std::cout << "\nComputing disparity with OpenCV SGBM..." << std::endl;
    cv::Mat disparity;
    sgbm->compute(left, right, disparity);

    std::cout << "Disparity computed, analyzing results..." << std::endl;

    // Analyze disparity
    std::cout << "\n=== DISPARITY ANALYSIS ===" << std::endl;
    std::cout << "Type: " << disparity.type() << " (CV_16S = 3)" << std::endl;
    std::cout << "Size: " << disparity.cols << "x" << disparity.rows << std::endl;

    // Count valid disparities
    int totalPixels = disparity.rows * disparity.cols;
    int validPixels = 0;
    double minDisp = 1e9, maxDisp = -1e9;

    for (int y = 0; y < disparity.rows; y++) {
        const int16_t* row = disparity.ptr<int16_t>(y);
        for (int x = 0; x < disparity.cols; x++) {
            int16_t d = row[x];
            if (d > 0) {  // Valid disparity (CV_16S, ×16)
                validPixels++;
                double realDisp = d / 16.0;
                minDisp = std::min(minDisp, realDisp);
                maxDisp = std::max(maxDisp, realDisp);
            }
        }
    }

    std::cout << "Valid pixels: " << validPixels << "/" << totalPixels
              << " (" << (100.0 * validPixels / totalPixels) << "%)" << std::endl;

    if (validPixels > 0) {
        std::cout << "Disparity range: [" << minDisp << ", " << maxDisp << "] pixels" << std::endl;
    } else {
        std::cout << "❌ NO VALID DISPARITIES FOUND!" << std::endl;
    }

    // Compare with Census result
    std::cout << "\n=== COMPARISON WITH CENSUS ===" << std::endl;
    cv::Mat censusDis = cv::imread("/home/alessandro/unlook_debug/scan_20251114_205404/02_disparity_frame0.png", cv::IMREAD_GRAYSCALE);

    if (!censusDis.empty()) {
        // Census disparity is saved as 8-bit normalized
        // We need to count non-zero pixels
        int censusValid = cv::countNonZero(censusDis);
        std::cout << "Census valid pixels: " << censusValid << "/" << totalPixels
                  << " (" << (100.0 * censusValid / totalPixels) << "%)" << std::endl;

        double censusPercent = 100.0 * censusValid / totalPixels;
        double sgbmPercent = 100.0 * validPixels / totalPixels;

        if (sgbmPercent > censusPercent * 1.5) {
            std::cout << "\n✓ OpenCV SGBM found " << (sgbmPercent / censusPercent)
                      << "x MORE matches than Census!" << std::endl;
            std::cout << "   → Census Transform may be the problem" << std::endl;
        } else if (sgbmPercent < censusPercent * 0.5) {
            std::cout << "\n✓ Census found " << (censusPercent / sgbmPercent)
                      << "x MORE matches than SGBM!" << std::endl;
            std::cout << "   → Census is actually working better" << std::endl;
        } else {
            std::cout << "\n≈ Similar performance between Census and SGBM" << std::endl;
            std::cout << "   → Algorithm choice is not the main issue" << std::endl;
        }
    }

    // Save OpenCV SGBM result for comparison
    cv::Mat disparity8;
    disparity.convertTo(disparity8, CV_8U, 255.0 / (numDisparities * 16.0));
    cv::imwrite("/tmp/opencv_sgbm_disparity.png", disparity8);
    std::cout << "\nSaved OpenCV SGBM disparity to: /tmp/opencv_sgbm_disparity.png" << std::endl;

    // Save disparity as colormap for better visualization
    cv::Mat disparityColor;
    cv::applyColorMap(disparity8, disparityColor, cv::COLORMAP_JET);
    cv::imwrite("/tmp/opencv_sgbm_disparity_color.png", disparityColor);
    std::cout << "Saved colormap to: /tmp/opencv_sgbm_disparity_color.png" << std::endl;

    std::cout << "\n=== TEST COMPLETE ===" << std::endl;
    std::cout << "\nRECOMMENDATION:" << std::endl;
    if (validPixels > totalPixels * 0.7) {
        std::cout << "✓ OpenCV SGBM works well - consider switching from Census" << std::endl;
    } else if (validPixels > totalPixels * 0.3) {
        std::cout << "⚠  OpenCV SGBM has moderate success - may need parameter tuning" << std::endl;
    } else {
        std::cout << "❌ OpenCV SGBM also fails - problem is NOT the matching algorithm" << std::endl;
        std::cout << "   Likely causes: calibration, rectification, or image quality" << std::endl;
    }

    return 0;
}
