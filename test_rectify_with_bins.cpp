/**
 * @brief Test rectification usando i .bin files invece di ricalcolare
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

cv::Mat loadBinaryMap(const std::string& path, cv::Size size) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open " + path);
    }

    cv::Mat map(size, CV_32FC1);
    file.read(reinterpret_cast<char*>(map.data), map.total() * map.elemSize());

    if (!file.good()) {
        throw std::runtime_error("Failed to read " + path);
    }

    return map;
}

int main() {
    std::string basePath = "/unlook_calib/calib-20251115_022257";
    std::string leftImg = "/unlook_calib_dataset/dataset_20251110_112023/left/frame_000.png";
    std::string rightImg = "/unlook_calib_dataset/dataset_20251110_112023/right/frame_000.png";

    cv::Size imgSize(1280, 720);

    std::cout << "Loading rectification maps from .bin files..." << std::endl;
    cv::Mat map1L = loadBinaryMap(basePath + "-map-left-x.bin", imgSize);
    cv::Mat map2L = loadBinaryMap(basePath + "-map-left-y.bin", imgSize);
    cv::Mat map1R = loadBinaryMap(basePath + "-map-right-x.bin", imgSize);
    cv::Mat map2R = loadBinaryMap(basePath + "-map-right-y.bin", imgSize);

    std::cout << "Maps loaded successfully" << std::endl;

    // Load images
    cv::Mat left = cv::imread(leftImg);
    cv::Mat right = cv::imread(rightImg);
    if (left.empty() || right.empty()) {
        std::cerr << "Cannot load images" << std::endl;
        return 1;
    }

    // Rectify using .bin maps
    cv::Mat leftRect, rightRect;
    cv::remap(left, leftRect, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(right, rightRect, map1R, map2R, cv::INTER_LINEAR);

    // Save
    cv::imwrite("bin_rectified_left.png", leftRect);
    cv::imwrite("bin_rectified_right.png", rightRect);

    std::cout << "Saved: bin_rectified_left.png, bin_rectified_right.png" << std::endl;
    std::cout << "Run: python3 test_epipolar_alignment.py bin_rectified_left.png bin_rectified_right.png" << std::endl;

    return 0;
}
