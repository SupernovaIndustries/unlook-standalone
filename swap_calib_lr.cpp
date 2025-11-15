/**
 * @file swap_calib_lr.cpp
 * @brief Swap LEFT↔RIGHT in OpenCV stereo calibration YAML
 *
 * REASON:
 * Dataset capture (2025-11-10): Camera 0 SLAVE → /left/, Camera 1 MASTER → /right/
 * Runtime (current): Camera 1 MASTER → LEFT, Camera 0 SLAVE → RIGHT
 *
 * This swaps calibration parameters to match runtime camera assignment.
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.yaml> [output.yaml]" << std::endl;
        std::cerr << "Example: " << argv[0] << " /unlook_calib/default.yaml" << std::endl;
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputPath = (argc > 2) ? argv[2] : (inputPath.substr(0, inputPath.find_last_of('.')) + "_swapped.yaml");

    std::cout << "Loading calibration from: " << inputPath << std::endl;

    // Read input calibration
    cv::FileStorage fsIn(inputPath, cv::FileStorage::READ);
    if (!fsIn.isOpened()) {
        std::cerr << "Error: Cannot open " << inputPath << std::endl;
        return 1;
    }

    // Read all parameters
    std::string calibDate, datasetPath, patternType;
    int imageWidth, imageHeight, numPairs, validPairs;
    cv::Mat camMatL, camMatR, distL, distR;
    cv::Mat R, T, R1, R2, P1, P2, Q;

    fsIn["calibration_date"] >> calibDate;
    fsIn["dataset_path"] >> datasetPath;
    fsIn["pattern_type"] >> patternType;
    fsIn["image_width"] >> imageWidth;
    fsIn["image_height"] >> imageHeight;
    fsIn["num_image_pairs"] >> numPairs;
    fsIn["valid_image_pairs"] >> validPairs;

    fsIn["camera_matrix_left"] >> camMatL;
    fsIn["camera_matrix_right"] >> camMatR;
    fsIn["distortion_coeffs_left"] >> distL;
    fsIn["distortion_coeffs_right"] >> distR;

    fsIn["rotation_matrix"] >> R;
    fsIn["translation_vector"] >> T;

    fsIn["rectification_transform_left"] >> R1;
    fsIn["rectification_transform_right"] >> R2;
    fsIn["projection_matrix_left"] >> P1;
    fsIn["projection_matrix_right"] >> P2;
    fsIn["disparity_to_depth_matrix"] >> Q;

    fsIn.release();

    std::cout << "Original calibration:" << std::endl;
    std::cout << "  Image size: " << imageWidth << "x" << imageHeight << std::endl;
    std::cout << "  Baseline: " << cv::norm(T) << " mm" << std::endl;
    std::cout << "  Date: " << calibDate << std::endl;

    // SWAP LEFT ↔ RIGHT
    std::swap(camMatL, camMatR);
    std::swap(distL, distR);
    std::swap(R1, R2);
    std::swap(P1, P2);

    // Invert rotation R → R^T (transpose)
    cv::Mat R_inv = R.t();

    // Invert translation T → -T
    cv::Mat T_inv = -T;

    std::cout << "\nSwapping LEFT ↔ RIGHT..." << std::endl;
    std::cout << "  Swapped: camera matrices, distortion coeffs, R1↔R2, P1↔P2" << std::endl;
    std::cout << "  Inverted: R → R^T, T → -T" << std::endl;

    // Write swapped calibration
    std::cout << "\nSaving swapped calibration to: " << outputPath << std::endl;

    cv::FileStorage fsOut(outputPath, cv::FileStorage::WRITE);

    // Metadata
    fsOut << "calibration_date" << cv::format("%s_SWAPPED", calibDate.c_str());
    fsOut << "dataset_path" << datasetPath;
    fsOut << "pattern_type" << patternType;
    fsOut << "swap_note" << "LEFT↔RIGHT swapped to match runtime (Cam1→LEFT, Cam0→RIGHT)";

    // Image info
    fsOut << "image_width" << imageWidth;
    fsOut << "image_height" << imageHeight;
    fsOut << "num_image_pairs" << numPairs;
    fsOut << "valid_image_pairs" << validPairs;

    // Swapped camera parameters
    fsOut << "camera_matrix_left" << camMatL;      // Was camMatR
    fsOut << "camera_matrix_right" << camMatR;     // Was camMatL
    fsOut << "distortion_coeffs_left" << distL;  // Was distR
    fsOut << "distortion_coeffs_right" << distR; // Was distL

    // Inverted stereo transform
    fsOut << "rotation_matrix" << R_inv;           // Was R
    fsOut << "translation_vector" << T_inv;        // Was T
    fsOut << "baseline_mm" << cv::norm(T_inv);

    // Swapped rectification
    fsOut << "rectification_transform_left" << R1;  // Was R2
    fsOut << "rectification_transform_right" << R2; // Was R1
    fsOut << "projection_matrix_left" << P1;        // Was P2
    fsOut << "projection_matrix_right" << P2;       // Was P1
    fsOut << "disparity_to_depth_matrix" << Q;

    // Copy other fields (epipolar errors, etc)
    double meanEpipolar, maxEpipolar, rmsReprojection;
    fsIn.open(inputPath, cv::FileStorage::READ);
    fsIn["mean_epipolar_error"] >> meanEpipolar;
    fsIn["max_epipolar_error"] >> maxEpipolar;
    fsIn["rms_reprojection_error"] >> rmsReprojection;
    fsIn.release();

    fsOut << "mean_epipolar_error" << meanEpipolar;
    fsOut << "max_epipolar_error" << maxEpipolar;
    fsOut << "rms_reprojection_error" << rmsReprojection;
    fsOut << "opencv_version" << CV_VERSION;
    fsOut << "calibration_method" << "opencv_stereo_calibrate_SWAPPED_LR";

    fsOut.release();

    std::cout << "✓ Swapped calibration saved successfully!" << std::endl;
    std::cout << "\nTo test:" << std::endl;
    std::cout << "  sudo ln -sf " << outputPath << " /unlook_calib/default.yaml" << std::endl;
    std::cout << "  unlook  # Run scan and check if cone pattern is gone" << std::endl;

    return 0;
}
