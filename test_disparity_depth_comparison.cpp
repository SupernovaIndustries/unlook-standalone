/**
 * @file test_disparity_depth_comparison.cpp
 * @brief Diagnostic test to compare OpenCV reprojectImageTo3D vs manual depth calculation
 *
 * This test verifies if OpenCV's reprojectImageTo3D produces the same depth values
 * as our manual calculation using Z = (baseline * fx) / disparity
 *
 * CRITICAL FOR INVESTOR DEMO: Resolve 28% Z error discrepancy
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << std::fixed << std::setprecision(6);

    // Load Q matrix from calibration file
    cv::FileStorage fs("/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERROR: Could not open calibration file!" << std::endl;
        return 1;
    }

    cv::Mat Q;
    fs["Q"] >> Q;
    fs.release();

    std::cout << "\n===== Q MATRIX FROM CALIBRATION =====" << std::endl;
    std::cout << Q << std::endl;

    // Extract parameters from Q matrix
    double cx_rect = -Q.at<double>(0, 3);
    double cy_rect = -Q.at<double>(1, 3);
    double fx_rect = Q.at<double>(2, 3);
    double Tx_inv = Q.at<double>(3, 2);
    double baseline_mm = std::abs(1.0 / Tx_inv);

    std::cout << "\n===== EXTRACTED PARAMETERS =====" << std::endl;
    std::cout << "cx_rect = " << cx_rect << " px" << std::endl;
    std::cout << "cy_rect = " << cy_rect << " px" << std::endl;
    std::cout << "fx_rect = " << fx_rect << " px" << std::endl;
    std::cout << "baseline_mm = " << baseline_mm << " mm" << std::endl;
    std::cout << "Tx_inv = " << Tx_inv << std::endl;

    // Create test disparity map with known values
    cv::Mat disparity(1088, 1456, CV_32F);

    // Fill with test disparities
    std::vector<float> testDisparities = {
        90.125f,   // Sample from log: should give 1363.78mm
        102.125f,  // Mean disparity: should give 1202.97mm
        50.0f,     // Farther object
        150.0f,    // Closer object
        200.0f     // Very close object
    };

    std::cout << "\n===== COMPARISON TEST =====" << std::endl;
    std::cout << std::left;
    std::cout << std::setw(12) << "Disparity"
              << std::setw(20) << "Manual Z (mm)"
              << std::setw(20) << "OpenCV Z (mm)"
              << std::setw(15) << "Difference"
              << std::setw(15) << "Error %"
              << std::endl;
    std::cout << std::string(80, '-') << std::endl;

    for (float d : testDisparities) {
        // Set entire disparity map to this value for testing
        disparity.setTo(d);

        // Manual calculation
        double Z_manual = (baseline_mm * fx_rect) / d;

        // OpenCV reprojectImageTo3D
        cv::Mat points3D;
        cv::reprojectImageTo3D(disparity, points3D, Q, true);

        // Extract Z channel
        std::vector<cv::Mat> channels(3);
        cv::split(points3D, channels);
        cv::Mat depthMap = channels[2];

        // Get a sample depth value (center pixel)
        double Z_opencv = depthMap.at<float>(544, 728);

        // Calculate difference
        double diff = Z_opencv - Z_manual;
        double error_pct = (diff / Z_manual) * 100.0;

        std::cout << std::setw(12) << d
                  << std::setw(20) << Z_manual
                  << std::setw(20) << Z_opencv
                  << std::setw(15) << diff
                  << std::setw(15) << error_pct
                  << std::endl;
    }

    // Special test: Use actual image dimensions at different positions
    std::cout << "\n===== POSITION-DEPENDENT TEST =====" << std::endl;
    std::cout << std::left;
    std::cout << std::setw(8) << "X"
              << std::setw(8) << "Y"
              << std::setw(12) << "Disparity"
              << std::setw(18) << "Manual XYZ"
              << std::setw(18) << "OpenCV XYZ"
              << std::setw(12) << "Z Diff"
              << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    // Test at different pixel positions
    std::vector<std::pair<int, int>> testPositions = {
        {297, 0},      // From log sample
        {728, 544},    // Center
        {100, 100},    // Top-left area
        {1356, 988},   // Bottom-right area
    };

    disparity.setTo(0);
    disparity.at<float>(0, 297) = 90.125f;
    disparity.at<float>(544, 728) = 102.125f;
    disparity.at<float>(100, 100) = 120.0f;
    disparity.at<float>(988, 1356) = 80.0f;

    // OpenCV reprojectImageTo3D
    cv::Mat points3D_full;
    cv::reprojectImageTo3D(disparity, points3D_full, Q, true);

    for (const auto& pos : testPositions) {
        int x = pos.first;
        int y = pos.second;
        float d = disparity.at<float>(y, x);

        if (d > 1.0f) {
            // Manual calculation
            double Z_manual = (baseline_mm * fx_rect) / d;
            double X_manual = (x - cx_rect) * Z_manual / fx_rect;
            double Y_manual = (y - cy_rect) * Z_manual / fx_rect;

            // OpenCV result
            cv::Vec3f point_opencv = points3D_full.at<cv::Vec3f>(y, x);
            double X_opencv = point_opencv[0];
            double Y_opencv = point_opencv[1];
            double Z_opencv = point_opencv[2];

            double z_diff = Z_opencv - Z_manual;

            std::cout << std::setw(8) << x
                      << std::setw(8) << y
                      << std::setw(12) << d
                      << "(" << std::setw(6) << X_manual << "," << std::setw(6) << Y_manual << "," << std::setw(6) << Z_manual << ")"
                      << " (" << std::setw(6) << X_opencv << "," << std::setw(6) << Y_opencv << "," << std::setw(6) << Z_opencv << ")"
                      << std::setw(12) << z_diff
                      << std::endl;
        }
    }

    std::cout << "\n===== TEST COMPLETE =====" << std::endl;
    std::cout << "If Z values match within floating-point precision, OpenCV is correct." << std::endl;
    std::cout << "If Z values differ significantly, there's a Q matrix interpretation issue." << std::endl;

    return 0;
}
