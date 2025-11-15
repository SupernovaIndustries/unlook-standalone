/**
 * @brief Test rectification su immagini del dataset di calibrazione
 */
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::string calibFile = "/unlook_calib/default.yaml";
    std::string leftImg = "/unlook_calib_dataset/dataset_20251110_112023/left/frame_000.png";
    std::string rightImg = "/unlook_calib_dataset/dataset_20251110_112023/right/frame_000.png";

    // Load calibration
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Cannot open " << calibFile << std::endl;
        return 1;
    }

    cv::Mat camMatL, camMatR, distL, distR, R1, R2, P1, P2;
    int width, height;
    fs["camera_matrix_left"] >> camMatL;
    fs["camera_matrix_right"] >> camMatR;
    fs["distortion_coeffs_left"] >> distL;
    fs["distortion_coeffs_right"] >> distR;
    fs["rectification_transform_left"] >> R1;
    fs["rectification_transform_right"] >> R2;
    fs["projection_matrix_left"] >> P1;
    fs["projection_matrix_right"] >> P2;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    fs.release();

    std::cout << "Calibration loaded: " << width << "x" << height << std::endl;

    // Load images
    cv::Mat left = cv::imread(leftImg);
    cv::Mat right = cv::imread(rightImg);
    if (left.empty() || right.empty()) {
        std::cerr << "Cannot load images" << std::endl;
        return 1;
    }

    std::cout << "Images loaded: " << left.cols << "x" << left.rows << std::endl;

    // Compute rectification maps
    cv::Size imgSize(width, height);
    cv::Mat map1L, map2L, map1R, map2R;
    cv::initUndistortRectifyMap(camMatL, distL, R1, P1, imgSize, CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(camMatR, distR, R2, P2, imgSize, CV_32FC1, map1R, map2R);

    // Rectify
    cv::Mat leftRect, rightRect;
    cv::remap(left, leftRect, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(right, rightRect, map1R, map2R, cv::INTER_LINEAR);

    // Save
    cv::imwrite("calib_rectified_left.png", leftRect);
    cv::imwrite("calib_rectified_right.png", rightRect);

    std::cout << "Saved: calib_rectified_left.png, calib_rectified_right.png" << std::endl;
    std::cout << "Run: python3 test_epipolar_alignment.py calib_rectified_left.png calib_rectified_right.png" << std::endl;

    return 0;
}
