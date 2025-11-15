// Simplified OpenCV stereo calibration test - checkerboard only, compatible with OpenCV 4.6
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <filesystem>

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: " << argv[0] << " <dataset_dir>" << endl;
        return 1;
    }

    string datasetDir = argv[1];
    string leftDir = datasetDir + "/left";
    string rightDir = datasetDir + "/right";

    // Checkerboard pattern (7x10 squares = 6x9 inner corners)
    Size boardSize(9, 6);
    float squareSize = 24.0f; // mm

    // Load images
    vector<string> leftFiles, rightFiles;
    for (const auto& entry : fs::directory_iterator(leftDir)) {
        if (entry.path().extension() == ".png") {
            leftFiles.push_back(entry.path().string());
        }
    }
    for (const auto& entry : fs::directory_iterator(rightDir)) {
        if (entry.path().extension() == ".png") {
            rightFiles.push_back(entry.path().string());
        }
    }

    sort(leftFiles.begin(), leftFiles.end());
    sort(rightFiles.begin(), rightFiles.end());

    cout << "Found " << leftFiles.size() << " image pairs" << endl;

    vector<vector<Point2f>> imagePoints[2];
    vector<vector<Point3f>> objectPoints;
    Size imageSize;

    int validPairs = 0;

    // Detect corners
    for (size_t i = 0; i < leftFiles.size(); i++) {
        Mat imgL = imread(leftFiles[i], IMREAD_GRAYSCALE);
        Mat imgR = imread(rightFiles[i], IMREAD_GRAYSCALE);

        if (imgL.empty() || imgR.empty()) continue;
        if (imageSize == Size()) imageSize = imgL.size();

        vector<Point2f> cornersL, cornersR;
        bool foundL = findChessboardCorners(imgL, boardSize, cornersL,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
        bool foundR = findChessboardCorners(imgR, boardSize, cornersR,
            CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);

        if (!foundL || !foundR) {
            cout << "Skipping pair " << i << " - pattern not found" << endl;
            continue;
        }

        // Refine corners with cornerSubPix
        cornerSubPix(imgL, cornersL, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01));
        cornerSubPix(imgR, cornersR, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01));

        imagePoints[0].push_back(cornersL);
        imagePoints[1].push_back(cornersR);

        // Generate object points
        vector<Point3f> objPts;
        for (int row = 0; row < boardSize.height; row++) {
            for (int col = 0; col < boardSize.width; col++) {
                objPts.push_back(Point3f(col * squareSize, row * squareSize, 0));
            }
        }
        objectPoints.push_back(objPts);

        validPairs++;
        cout << "." << flush;
    }

    cout << endl << validPairs << " pairs detected successfully" << endl;

    if (validPairs < 10) {
        cout << "ERROR: Need at least 10 valid pairs!" << endl;
        return 1;
    }

    // Initialize camera matrices
    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);

    cout << "\nRunning stereoCalibrate with OpenCV official flags..." << endl;

    Mat R, T, E, F;
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    CALIB_FIX_ASPECT_RATIO +
                    CALIB_ZERO_TANGENT_DIST +
                    CALIB_USE_INTRINSIC_GUESS +
                    CALIB_SAME_FOCAL_LENGTH +
                    CALIB_RATIONAL_MODEL +
                    CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                    TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

    cout << "RMS reprojection error: " << rms << endl;

    // Compute epipolar error (OpenCV official method)
    cout << "\nComputing epipolar error..." << endl;
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];

    for (size_t i = 0; i < objectPoints.size(); i++) {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];

        for (int k = 0; k < 2; k++) {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
        }

        for (int j = 0; j < npt; j++) {
            double errij = fabs(imagePoints[0][i][j].x * lines[1][j][0] +
                               imagePoints[0][i][j].y * lines[1][j][1] + lines[1][j][2]) +
                          fabs(imagePoints[1][i][j].x * lines[0][j][0] +
                               imagePoints[1][i][j].y * lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    cout << "Average epipolar error: " << err / npoints << " px" << endl;

    // Compute baseline
    double baseline = norm(T);
    cout << "Baseline: " << baseline << " mm" << endl;

    // Compute rectification
    cout << "\nComputing rectification with alpha=0..." << endl;
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 0.0, imageSize, &validRoi[0], &validRoi[1]);

    cout << "Valid ROI Left: " << validRoi[0] << endl;
    cout << "Valid ROI Right: " << validRoi[1] << endl;

    // Test rectification on first image pair
    cout << "\nTesting rectification on first image..." << endl;
    Mat imgL = imread(leftFiles[0]);
    Mat imgR = imread(rightFiles[0]);

    Mat map1L, map2L, map1R, map2R;
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, map1L, map2L);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, map1R, map2R);

    Mat rectL, rectR;
    remap(imgL, rectL, map1L, map2L, INTER_LINEAR);
    remap(imgR, rectR, map1R, map2R, INTER_LINEAR);

    imwrite("opencv_official_rect_left.png", rectL);
    imwrite("opencv_official_rect_right.png", rectR);

    cout << "Saved rectified images to opencv_official_rect_*.png" << endl;
    cout << "\n✓ OpenCV official calibration completed" << endl;

    return 0;
}
