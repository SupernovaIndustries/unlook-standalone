/**
 * @brief Test rectification con T invertito (Tx negato)
 */
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    std::string calibFile = "/unlook_calib/default.yaml";
    std::string leftImg = "/unlook_calib_dataset/dataset_20251110_112023/left/frame_000.png";
    std::string rightImg = "/unlook_calib_dataset/dataset_20251110_112023/right/frame_000.png";

    // Load calibration
    cv::FileStorage fs(calibFile, cv::FileStorage::READ);
    cv::Mat camMatL, camMatR, distL, distR, R, T;
    int width, height;
    fs["camera_matrix_left"] >> camMatL;
    fs["camera_matrix_right"] >> camMatR;
    fs["distortion_coeffs_left"] >> distL;
    fs["distortion_coeffs_right"] >> distR;
    fs["rotation_matrix"] >> R;
    fs["translation_vector"] >> T;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    fs.release();

    std::cout << "Original T: " << T.t() << std::endl;

    // INVERT translation vector
    cv::Mat T_inv = -T;
    std::cout << "Inverted T: " << T_inv.t() << std::endl;

    // Compute rectification with INVERTED T
    cv::Size imgSize(width, height);
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(camMatL, distL, camMatR, distR, imgSize, R, T_inv,
                     R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY);

    // Compute maps
    cv::Mat map1L, map2L, map1R, map2R;
    cv::initUndistortRectifyMap(camMatL, distL, R1, P1, imgSize, CV_32FC1, map1L, map2L);
    cv::initUndistortRectifyMap(camMatR, distR, R2, P2, imgSize, CV_32FC1, map1R, map2R);

    // Load images
    cv::Mat left = cv::imread(leftImg);
    cv::Mat right = cv::imread(rightImg);

    // Rectify
    cv::Mat leftRect, rightRect;
    cv::remap(left, leftRect, map1L, map2L, cv::INTER_LINEAR);
    cv::remap(right, rightRect, map1R, map2R, cv::INTER_LINEAR);

    // Save
    cv::imwrite("invT_rectified_left.png", leftRect);
    cv::imwrite("invT_rectified_right.png", rightRect);

    std::cout << "Saved: invT_rectified_left.png, invT_rectified_right.png" << std::endl;
    std::cout << "Run: python3 test_epipolar_alignment.py invT_rectified_left.png invT_rectified_right.png" << std::endl;

    return 0;
}
