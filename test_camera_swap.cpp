#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>

int main() {
    // Load calibration dataset frame pair
    std::string leftPath = "/unlook_calib_dataset/dataset_20251106_002658/left/frame_000.png";
    std::string rightPath = "/unlook_calib_dataset/dataset_20251106_002658/right/frame_000.png";

    cv::Mat leftImg = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
    cv::Mat rightImg = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);

    if (leftImg.empty() || rightImg.empty()) {
        std::cerr << "Error: Cannot load images" << std::endl;
        return 1;
    }

    std::cout << "Loaded images: " << leftImg.cols << "x" << leftImg.rows << std::endl;

    // Detect checkerboard (7x10 = 6x9 inner corners)
    cv::Size patternSize(9, 6);  // inner corners
    std::vector<cv::Point2f> cornersLeft, cornersRight;

    bool foundLeft = cv::findChessboardCorners(leftImg, patternSize, cornersLeft,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
    bool foundRight = cv::findChessboardCorners(rightImg, patternSize, cornersRight,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (!foundLeft || !foundRight) {
        std::cerr << "Error: Pattern not found in both images" << std::endl;
        std::cerr << "  Left: " << (foundLeft ? "FOUND" : "NOT FOUND") << std::endl;
        std::cerr << "  Right: " << (foundRight ? "FOUND" : "NOT FOUND") << std::endl;
        return 1;
    }

    // Calculate centroids
    float leftCentroidX = 0, rightCentroidX = 0;
    for (const auto& pt : cornersLeft) leftCentroidX += pt.x;
    for (const auto& pt : cornersRight) rightCentroidX += pt.x;
    leftCentroidX /= cornersLeft.size();
    rightCentroidX /= cornersRight.size();

    std::cout << "\n=== PARALLAX VERIFICATION ===" << std::endl;
    std::cout << "Left image centroid X:  " << leftCentroidX << " px" << std::endl;
    std::cout << "Right image centroid X: " << rightCentroidX << " px" << std::endl;
    std::cout << "Disparity (L - R):      " << (leftCentroidX - rightCentroidX) << " px" << std::endl;

    // In correct stereo setup:
    // - LEFT camera sees object more to the RIGHT
    // - RIGHT camera sees object more to the LEFT
    // - leftCentroidX > rightCentroidX (positive disparity)

    if (leftCentroidX > rightCentroidX) {
        std::cout << "\n✓ PARALLAX CORRECT: Left centroid is RIGHT of Right centroid" << std::endl;
        std::cout << "  This means:" << std::endl;
        std::cout << "  - left/ directory contains LEFT camera images" << std::endl;
        std::cout << "  - right/ directory contains RIGHT camera images" << std::endl;
        std::cout << "  - Camera mapping during capture is CORRECT" << std::endl;
        std::cout << "\n⚠ BUT Tx is NEGATIVE in calibration file!" << std::endl;
        std::cout << "  This indicates R1/R2, P1/P2 are swapped in calibration" << std::endl;
        std::cout << "  → SWAP in CalibrationManager IS NECESSARY AND CORRECT" << std::endl;
    } else {
        std::cout << "\n✗ PARALLAX INVERTED: Left centroid is LEFT of Right centroid" << std::endl;
        std::cout << "  This means:" << std::endl;
        std::cout << "  - left/ directory contains RIGHT camera images ← WRONG" << std::endl;
        std::cout << "  - right/ directory contains LEFT camera images ← WRONG" << std::endl;
        std::cout << "  - Camera mapping during capture is INVERTED" << std::endl;
        std::cout << "\n  → FIX: DatasetCaptureWidget is saving cameras to wrong directories" << std::endl;
    }

    return 0;
}
