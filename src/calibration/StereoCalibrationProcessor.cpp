#include "unlook/calibration/StereoCalibrationProcessor.hpp"
#include "unlook/calibration/CalibrationValidator.hpp"
#include "unlook/calibration/PatternDetector.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace fs = std::filesystem;

namespace unlook {
namespace calibration {

// Static helper functions defined first

// Helper function for getting current timestamp
static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");

    return ss.str();
}

// Helper function for pattern type parsing
static PatternType parsePatternType(const std::string& typeStr) {
    std::string lower = typeStr;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower == "checkerboard") {
        return PatternType::CHECKERBOARD;
    } else if (lower == "charuco") {
        return PatternType::CHARUCO;
    } else if (lower == "circle_grid" || lower == "circlegrid") {
        return PatternType::CIRCLE_GRID;
    }

    // Default to ChArUco
    return PatternType::CHARUCO;
}

// Helper function for pattern type to string
static std::string patternTypeToString(PatternType type) {
    switch (type) {
        case PatternType::CHECKERBOARD:
            return "checkerboard";
        case PatternType::CHARUCO:
            return "charuco";
        case PatternType::CIRCLE_GRID:
            return "circle_grid";
        default:
            return "unknown";
    }
}

// Helper function to generate object points
static std::vector<cv::Point3f> generateObjectPoints(const PatternConfig& config) {
    std::vector<cv::Point3f> objPoints;

    int rows = config.rows;
    int cols = config.cols;

    // For checkerboard, we need inner corners
    if (config.type == PatternType::CHECKERBOARD) {
        rows -= 1;
        cols -= 1;
    }
    // For ChArUco, we also use inner corners
    else if (config.type == PatternType::CHARUCO) {
        rows -= 1;
        cols -= 1;
    }

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            objPoints.push_back(cv::Point3f(
                col * config.squareSizeMM,
                row * config.squareSizeMM,
                0.0f
            ));
        }
    }

    return objPoints;
}

// Helper function to save rectification maps
static void saveRectificationMaps(const CalibrationResult& result,
                                 const std::string& basePath) {
    // Save rectification maps as binary files for faster loading
    std::string map1LeftPath = basePath + "-map-left-x.bin";
    std::string map2LeftPath = basePath + "-map-left-y.bin";
    std::string map1RightPath = basePath + "-map-right-x.bin";
    std::string map2RightPath = basePath + "-map-right-y.bin";

    auto saveMap = [](const cv::Mat& map, const std::string& path) {
        // Comprehensive error handling for binary map file writes
        std::ofstream file(path, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file for writing: " + path +
                                   " (check disk space and permissions)");
        }

        // Write binary data
        file.write(reinterpret_cast<const char*>(map.data),
                  map.total() * map.elemSize());

        // Check for write errors (disk full, I/O error)
        if (!file.good()) {
            file.close();
            throw std::runtime_error("Write error for file: " + path +
                                   " (disk may be full or I/O error occurred)");
        }

        file.close();

        // Verify file was written
        if (!file.good()) {
            throw std::runtime_error("Error closing file: " + path);
        }
    };

    saveMap(result.map1Left, map1LeftPath);
    saveMap(result.map2Left, map2LeftPath);
    saveMap(result.map1Right, map1RightPath);
    saveMap(result.map2Right, map2RightPath);

    core::Logger::getInstance().debug("Rectification maps saved to: " + basePath + "-map-*.bin");
}

// Helper function for computing epipolar errors (moved here before first use)
// FIX: EXACT copy of OpenCV official sample algorithm (lines 241-264)
void computeEpipolarErrors(
    const CalibrationResult& result,
    const std::vector<std::vector<cv::Point2f>>& leftPoints,
    const std::vector<std::vector<cv::Point2f>>& rightPoints,
    double& meanError,
    double& maxError) {

    meanError = 0.0;
    maxError = 0.0;
    int totalPoints = 0;

    std::vector<cv::Vec3f> lines[2];

    for (size_t i = 0; i < leftPoints.size() && i < rightPoints.size(); ++i) {
        int npt = (int)leftPoints[i].size();
        if (npt != (int)rightPoints[i].size()) continue;

        cv::Mat imgpt[2];
        for (int k = 0; k < 2; k++) {
            const auto& points = (k == 0) ? leftPoints[i] : rightPoints[i];
            const auto& camMat = (k == 0) ? result.cameraMatrixLeft : result.cameraMatrixRight;
            const auto& distCoeffs = (k == 0) ? result.distCoeffsLeft : result.distCoeffsRight;

            imgpt[k] = cv::Mat(points);
            cv::undistortPoints(imgpt[k], imgpt[k], camMat, distCoeffs, cv::Mat(), camMat);
            cv::computeCorrespondEpilines(imgpt[k], k + 1, result.F, lines[k]);
        }

        for (int j = 0; j < npt; j++) {
            // FIX: Use UNDISTORTED points (imgpt) for distance calculation
            // The epipolar lines were computed from undistorted points (line 165),
            // so we must use the same undistorted points to measure distances.
            // Mixing distorted points with lines from undistorted points is geometrically wrong!
            cv::Point2f ptLeft(imgpt[0].at<float>(j, 0), imgpt[0].at<float>(j, 1));
            cv::Point2f ptRight(imgpt[1].at<float>(j, 0), imgpt[1].at<float>(j, 1));

            double errij = std::abs(ptLeft.x * lines[1][j][0] +
                                   ptLeft.y * lines[1][j][1] + lines[1][j][2]) +
                          std::abs(ptRight.x * lines[0][j][0] +
                                   ptRight.y * lines[0][j][1] + lines[0][j][2]);
            meanError += errij;
            maxError = std::max(maxError, errij);
        }
        totalPoints += npt;
    }

    if (totalPoints > 0) {
        meanError /= totalPoints;
    }
}

StereoCalibrationProcessor::StereoCalibrationProcessor() {
    core::Logger::getInstance().info("StereoCalibrationProcessor initialized");
}

StereoCalibrationProcessor::~StereoCalibrationProcessor() {
    // Destructor
}

CalibrationResult StereoCalibrationProcessor::calibrateFromDataset(const std::string& datasetPath) {
    auto startTime = std::chrono::steady_clock::now();

    reportProgress("Starting calibration...");

    CalibrationResult result;
    result.datasetPath = datasetPath;
    result.calibrationDate = getCurrentTimestamp();

    // 1. Load dataset info JSON
    reportProgress("Step 1/9: Loading dataset info...");
    core::Logger::getInstance().info("Loading dataset from: " + datasetPath);
    auto datasetInfo = loadDatasetInfo(datasetPath);

    result.datasetTimestamp = datasetInfo["dataset_info"]["timestamp"].get<std::string>();

    // Parse pattern configuration
    auto& patternConfig = datasetInfo["pattern_config"];
    std::string patternTypeStr = patternConfig["type"].get<std::string>();

    core::Logger::getInstance().info("Pattern type from JSON: '" + patternTypeStr + "'");
    result.patternConfig.type = parsePatternType(patternTypeStr);

    // Log the parsed type
    std::string parsedTypeName;
    switch (result.patternConfig.type) {
        case PatternType::CHECKERBOARD:
            parsedTypeName = "CHECKERBOARD";
            break;
        case PatternType::CHARUCO:
            parsedTypeName = "CHARUCO";
            break;
        case PatternType::CIRCLE_GRID:
            parsedTypeName = "CIRCLE_GRID";
            break;
        default:
            parsedTypeName = "UNKNOWN";
    }
    core::Logger::getInstance().info("Parsed pattern type: " + parsedTypeName);

    result.patternConfig.rows = patternConfig["rows"].get<int>();
    result.patternConfig.cols = patternConfig["cols"].get<int>();
    result.patternConfig.squareSizeMM = patternConfig["square_size_mm"].get<float>();

    core::Logger::getInstance().info("Pattern config: " + std::to_string(result.patternConfig.rows) + "x" +
                                    std::to_string(result.patternConfig.cols) + ", square=" +
                                    std::to_string(result.patternConfig.squareSizeMM) + "mm");

    if (patternConfig.contains("aruco_marker_size_mm")) {
        result.patternConfig.arucoMarkerSizeMM = patternConfig["aruco_marker_size_mm"].get<float>();
    }
    if (patternConfig.contains("aruco_dictionary")) {
        // Default to DICT_4X4_250 for now
        result.patternConfig.arucoDict = cv::aruco::DICT_4X4_250;
    }

    // 2. Load all image pairs
    reportProgress("Step 2/9: Loading image pairs...");
    core::Logger::getInstance().info("Loading image pairs...");
    std::vector<cv::Mat> leftImages, rightImages;
    if (!loadDatasetImages(datasetPath, leftImages, rightImages)) {
        result.errors.push_back("Failed to load dataset images");
        result.qualityPassed = false;
        return result;
    }

    result.numImagePairs = leftImages.size();
    if (result.numImagePairs > 0) {
        result.imageSize = leftImages[0].size();
    }

    reportProgress("Loaded " + std::to_string(result.numImagePairs) + " image pairs (" +
                  std::to_string(result.imageSize.width) + "x" + std::to_string(result.imageSize.height) + ")");
    core::Logger::getInstance().info("Loaded " + std::to_string(result.numImagePairs) +
                                    " image pairs at " + std::to_string(result.imageSize.width) +
                                    "x" + std::to_string(result.imageSize.height));

    // 3. Detect patterns in all images
    reportProgress("Step 3/9: Detecting patterns (this may take a few minutes)...");
    core::Logger::getInstance().info("Detecting calibration patterns...");
    std::vector<std::vector<cv::Point2f>> leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    PatternDetector detector(result.patternConfig);
    int validPairs = 0;
    int skippedBlurLeft = 0, skippedBlurRight = 0;
    int skippedCornerMismatch = 0, skippedDetectionFailed = 0;

    // Expected corner count for perfect detection (ChArUco 7x10 = 6x9 inner corners = 54)
    int expectedCorners = (result.patternConfig.rows - 1) * (result.patternConfig.cols - 1);

    for (size_t i = 0; i < leftImages.size(); ++i) {
        // Report progress every 5 images
        if (i % 5 == 0) {
            reportProgress("Analyzing image " + std::to_string(i+1) + "/" + std::to_string(leftImages.size()) +
                          " (valid: " + std::to_string(validPairs) + ")");
        }
        std::vector<cv::Point2f> cornersLeft, cornersRight;
        cv::Mat overlayLeft, overlayRight;

        // Convert to grayscale for blur detection
        cv::Mat grayLeft, grayRight;
        cv::cvtColor(leftImages[i], grayLeft, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightImages[i], grayRight, cv::COLOR_BGR2GRAY);

        // Blur detection using Laplacian variance
        cv::Mat laplacianLeft, laplacianRight;
        cv::Laplacian(grayLeft, laplacianLeft, CV_64F);
        cv::Laplacian(grayRight, laplacianRight, CV_64F);

        cv::Scalar meanLeft, stddevLeft, meanRight, stddevRight;
        cv::meanStdDev(laplacianLeft, meanLeft, stddevLeft);
        cv::meanStdDev(laplacianRight, laplacianRight, stddevRight);

        double blurLeft = stddevLeft[0] * stddevLeft[0];  // Variance
        double blurRight = stddevRight[0] * stddevRight[0];

        // Blur threshold (lower = more blurry)
        // ADJUSTED for HD images: 100.0 was TOO HIGH, rejecting all images
        // Typical Laplacian variance for HD: 20-100 (good), 10-20 (acceptable), <10 (blurry)
        const double BLUR_THRESHOLD = 15.0;

        // Check for blur
        bool leftBlurry = (blurLeft < BLUR_THRESHOLD);
        bool rightBlurry = (blurRight < BLUR_THRESHOLD);

        if (leftBlurry || rightBlurry) {
            std::string reason = "Blurry image - ";
            if (leftBlurry && rightBlurry) {
                reason += "BOTH cameras (L:" + std::to_string(int(blurLeft)) +
                         " R:" + std::to_string(int(blurRight)) + ")";
                skippedBlurLeft++;
                skippedBlurRight++;
            } else if (leftBlurry) {
                reason += "LEFT camera (" + std::to_string(int(blurLeft)) + ")";
                skippedBlurLeft++;
            } else {
                reason += "RIGHT camera (" + std::to_string(int(blurRight)) + ")";
                skippedBlurRight++;
            }
            core::Logger::getInstance().warning("SKIP Pair " + std::to_string(i) + ": " + reason);
            result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
            continue;
        }

        // Pattern detection
        bool leftDetected = detector.detect(leftImages[i], cornersLeft, overlayLeft);
        bool rightDetected = detector.detect(rightImages[i], cornersRight, overlayRight);

        // FIX #1: Refine corners with cornerSubPix for checkerboard (OpenCV official sample)
        if (leftDetected && result.patternConfig.type == PatternType::CHECKERBOARD) {
            cv::cornerSubPix(grayLeft, cornersLeft, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        }
        if (rightDetected && result.patternConfig.type == PatternType::CHECKERBOARD) {
            cv::cornerSubPix(grayRight, cornersRight, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
        }

        // Quality validation
        if (leftDetected && rightDetected) {
            // Check corner count match with tolerance for ChArUco partial detection
            // Allow ±6 corners difference (~11% of 54) for robustness
            // (48-54 corners range = 6 corner tolerance)
            int cornerDiff = std::abs(static_cast<int>(cornersLeft.size()) - static_cast<int>(cornersRight.size()));
            if (cornerDiff > 6) {
                std::string reason = "Corner count mismatch (L:" +
                    std::to_string(cornersLeft.size()) + " vs R:" +
                    std::to_string(cornersRight.size()) +
                    " - diff:" + std::to_string(cornerDiff) +
                    " - expected: " + std::to_string(expectedCorners) + ")";
                core::Logger::getInstance().warning("SKIP Pair " + std::to_string(i) + ": " + reason);
                result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
                skippedCornerMismatch++;
                continue;
            } else if (cornerDiff > 0) {
                // Log warning but ACCEPT image with small mismatch
                core::Logger::getInstance().warning("Pair " + std::to_string(i) +
                    " - Small corner mismatch (L:" + std::to_string(cornersLeft.size()) +
                    " vs R:" + std::to_string(cornersRight.size()) + ") - ACCEPTED");
            }

            // Check minimum corners
            if (cornersLeft.size() < 4) {
                std::string reason = "Too few corners (" + std::to_string(cornersLeft.size()) + ")";
                core::Logger::getInstance().warning("SKIP Pair " + std::to_string(i) + ": " + reason);
                result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
                skippedDetectionFailed++;
                continue;
            }

            // Check if significantly fewer corners than expected (potential focus/angle issue)
            double cornerRatio = (double)cornersLeft.size() / expectedCorners;
            if (cornerRatio < 0.8) {  // Less than 80% of expected corners
                std::string reason = "Incomplete pattern (" +
                    std::to_string(cornersLeft.size()) + "/" +
                    std::to_string(expectedCorners) + " corners = " +
                    std::to_string(int(cornerRatio * 100)) + "%) - check focus/angle";
                core::Logger::getInstance().warning("SKIP Pair " + std::to_string(i) + ": " + reason);
                result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
                skippedDetectionFailed++;
                continue;
            }

            // All checks passed - add to calibration set
            leftImagePoints.push_back(cornersLeft);
            rightImagePoints.push_back(cornersRight);
            objectPoints.push_back(generateObjectPoints(result.patternConfig));

            validPairs++;

            std::string qualityInfo = "VALID Pair " + std::to_string(i) +
                ": " + std::to_string(cornersLeft.size()) + " corners " +
                "(" + std::to_string(int(cornerRatio * 100)) + "% complete) " +
                "blur(L:" + std::to_string(int(blurLeft)) + " R:" + std::to_string(int(blurRight)) + ")";
            core::Logger::getInstance().debug(qualityInfo);

        } else {
            std::string reason = "Pattern detection failed - ";
            if (!leftDetected && !rightDetected) {
                reason += "BOTH cameras";
            } else if (!leftDetected) {
                reason += "LEFT camera";
            } else {
                reason += "RIGHT camera";
            }
            core::Logger::getInstance().warning("SKIP Pair " + std::to_string(i) + ": " + reason);
            result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
            skippedDetectionFailed++;
        }
    }

    // Summary logging
    reportProgress("Pattern detection complete: " + std::to_string(validPairs) + "/" +
                  std::to_string(leftImages.size()) + " pairs valid");
    core::Logger::getInstance().info("========== QUALITY FILTER SUMMARY ==========");
    core::Logger::getInstance().info("Total image pairs: " + std::to_string(leftImages.size()));
    core::Logger::getInstance().info("Valid pairs (PASSED): " + std::to_string(validPairs));
    core::Logger::getInstance().info("Skipped - Blur LEFT: " + std::to_string(skippedBlurLeft));
    core::Logger::getInstance().info("Skipped - Blur RIGHT: " + std::to_string(skippedBlurRight));
    core::Logger::getInstance().info("Skipped - Corner mismatch: " + std::to_string(skippedCornerMismatch));
    core::Logger::getInstance().info("Skipped - Detection failed: " + std::to_string(skippedDetectionFailed));
    core::Logger::getInstance().info("===========================================");

    result.validImagePairs = validPairs;

    // Check minimum valid pairs
    if (validPairs < 10) {
        result.errors.push_back("Insufficient valid image pairs: " + std::to_string(validPairs) +
                               " (minimum 10 required)");
        result.qualityPassed = false;
        auto endTime = std::chrono::steady_clock::now();
        result.calibrationDurationSeconds =
            std::chrono::duration<double>(endTime - startTime).count();
        return result;
    }

    core::Logger::getInstance().info("Valid image pairs: " + std::to_string(validPairs) +
                                    "/" + std::to_string(result.numImagePairs));

    // FIX #2 & #3: Initialize camera matrices with initCameraMatrix2D (OpenCV official method)
    // Instead of separate calibrateCamera calls, use initial guess for stereoCalibrate
    reportProgress("Step 4/9: Initializing camera matrices...");
    core::Logger::getInstance().info("Initializing camera intrinsics with initCameraMatrix2D...");

    result.cameraMatrixLeft = cv::initCameraMatrix2D(objectPoints, leftImagePoints, result.imageSize, 0);
    result.cameraMatrixRight = cv::initCameraMatrix2D(objectPoints, rightImagePoints, result.imageSize, 0);
    result.distCoeffsLeft = cv::Mat::zeros(5, 1, CV_64F);
    result.distCoeffsRight = cv::Mat::zeros(5, 1, CV_64F);

    core::Logger::getInstance().info("Camera matrices initialized");
    reportProgress("Camera matrices initialized with 2D projection");

    // 6. Stereo calibration
    reportProgress("Step 6/9: Performing STEREO calibration (computing baseline)...");
    core::Logger::getInstance().info("Performing stereo calibration...");
    bool stereoCalibSuccess = calibrateStereo(leftImagePoints, rightImagePoints, objectPoints,
                                              result.imageSize, result);
    if (!stereoCalibSuccess) {
        result.warnings.push_back("Stereo calibration RMS is high - results may be less accurate");
        result.qualityPassed = false;
        core::Logger::getInstance().warning("Stereo calibration RMS > 2.0 px - quality may be poor");
    }
    reportProgress("Stereo calibration complete");

    // 7. Compute rectification
    reportProgress("Step 7/9: Computing rectification transforms...");
    core::Logger::getInstance().info("Computing rectification transforms...");
    // Use alpha=1 as per OpenCV official sample (exact match for comparison)
    cv::stereoRectify(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.cameraMatrixRight, result.distCoeffsRight,
        result.imageSize, result.R, result.T,
        result.R1, result.R2, result.P1, result.P2, result.Q,
        cv::CALIB_ZERO_DISPARITY, 1, result.imageSize  // alpha=1 (OpenCV sample)
    );
    reportProgress("Rectification transforms computed");

    // 8. Compute rectification maps
    reportProgress("Step 8/9: Computing rectification maps...");
    core::Logger::getInstance().info("Computing rectification maps...");
    // FIX: Use CV_16SC2 format as per OpenCV official sample (more efficient)
    cv::initUndistortRectifyMap(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.R1, result.P1, result.imageSize, CV_16SC2,
        result.map1Left, result.map2Left
    );
    cv::initUndistortRectifyMap(
        result.cameraMatrixRight, result.distCoeffsRight,
        result.R2, result.P2, result.imageSize, CV_16SC2,
        result.map1Right, result.map2Right
    );
    reportProgress("Rectification maps computed");

    // 9. Compute baseline
    core::Logger::getInstance().debug("Computing baseline from T vector...");
    core::Logger::getInstance().debug("T dimensions: " +
        std::to_string(result.T.rows) + "x" + std::to_string(result.T.cols));

    if (result.T.empty()) {
        core::Logger::getInstance().error("ERROR: Translation vector T is EMPTY! Cannot compute baseline!");
        result.errors.push_back("Translation vector T is empty - baseline cannot be computed");
        result.baselineMM = 0.0;
    } else {
        double tx = result.T.at<double>(0, 0);
        double ty = result.T.at<double>(1, 0);
        double tz = result.T.at<double>(2, 0);

        core::Logger::getInstance().debug("T = [" + std::to_string(tx) + ", " +
            std::to_string(ty) + ", " + std::to_string(tz) + "]");

        result.baselineMM = std::abs(tx);
        core::Logger::getInstance().info("Computed baseline from T(X): " + std::to_string(result.baselineMM) + " mm");
    }

    result.baselineExpectedMM = 70.0;  // Expected baseline for Unlook scanner
    result.baselineErrorMM = std::abs(result.baselineMM - result.baselineExpectedMM);
    result.baselineErrorPercent = (result.baselineErrorMM / result.baselineExpectedMM) * 100.0;
    reportProgress("Baseline: " + std::to_string(result.baselineMM) + "mm (expected: 70mm)");

    // 10. Validate calibration and compute quality metrics
    reportProgress("Step 9/9: Validating calibration quality...");
    core::Logger::getInstance().info("Validating calibration quality...");
    validateCalibration(result);

    // Compute epipolar errors separately
    computeEpipolarErrors(result, leftImagePoints, rightImagePoints,
                         result.meanEpipolarError, result.maxEpipolarError);

    // 11. Run comprehensive validation
    CalibrationValidator validator;
    CalibrationValidator::ValidationCriteria criteria;
    validator.validate(result, criteria);

    // Timing
    auto endTime = std::chrono::steady_clock::now();
    result.calibrationDurationSeconds =
        std::chrono::duration<double>(endTime - startTime).count();

    // Final report
    std::string qualityStatus = result.qualityPassed ? "✓ PASSED" : "✗ FAILED";
    reportProgress("Calibration complete in " + std::to_string(int(result.calibrationDurationSeconds)) +
                  "s - Quality: " + qualityStatus);
    reportProgress("RMS: " + std::to_string(result.rmsReprojectionError) + " px | Baseline: " +
                  std::to_string(result.baselineMM) + "mm");

    core::Logger::getInstance().info("Calibration completed in " +
                                    std::to_string(result.calibrationDurationSeconds) + " seconds");
    core::Logger::getInstance().info("Quality: " +
                                    (result.qualityPassed ? std::string("PASSED") : std::string("FAILED")));
    core::Logger::getInstance().info("RMS Error: " +
                                    std::to_string(result.rmsReprojectionError) + " pixels");
    core::Logger::getInstance().info("Baseline: " +
                                    std::to_string(result.baselineMM) + "mm (error: " +
                                    std::to_string(result.baselineErrorMM) + "mm)");

    return result;
}

nlohmann::json StereoCalibrationProcessor::loadDatasetInfo(const std::string& datasetPath) {
    std::string jsonPath = datasetPath + "/dataset_info.json";
    std::ifstream file(jsonPath);

    if (!file.is_open()) {
        throw std::runtime_error("Cannot open dataset info file: " + jsonPath);
    }

    nlohmann::json datasetInfo;
    file >> datasetInfo;
    return datasetInfo;
}

bool StereoCalibrationProcessor::loadDatasetImages(const std::string& datasetPath,
                                                  std::vector<cv::Mat>& leftImages,
                                                  std::vector<cv::Mat>& rightImages) {
    std::string leftDir = datasetPath + "/left";
    std::string rightDir = datasetPath + "/right";

    // Get sorted list of image files
    std::vector<std::string> leftFiles, rightFiles;

    for (const auto& entry : fs::directory_iterator(leftDir)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            leftFiles.push_back(entry.path().string());
        }
    }

    for (const auto& entry : fs::directory_iterator(rightDir)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            rightFiles.push_back(entry.path().string());
        }
    }

    std::sort(leftFiles.begin(), leftFiles.end());
    std::sort(rightFiles.begin(), rightFiles.end());

    if (leftFiles.size() != rightFiles.size()) {
        core::Logger::getInstance().error("Mismatch in number of left/right images");
        return false;
    }

    // Load images
    for (size_t i = 0; i < leftFiles.size(); ++i) {
        cv::Mat leftImg = cv::imread(leftFiles[i], cv::IMREAD_COLOR);
        cv::Mat rightImg = cv::imread(rightFiles[i], cv::IMREAD_COLOR);

        if (leftImg.empty() || rightImg.empty()) {
            core::Logger::getInstance().error("Failed to load image pair " + std::to_string(i));
            continue;
        }

        leftImages.push_back(leftImg);
        rightImages.push_back(rightImg);
    }

    return !leftImages.empty();
}


bool StereoCalibrationProcessor::calibrateIntrinsics(
    const std::vector<std::vector<cv::Point2f>>& imagePoints,
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const cv::Size& imageSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs) {

    core::Logger::getInstance().debug("Starting intrinsic calibration with " +
                                     std::to_string(imagePoints.size()) + " image views");

    std::vector<double> perImageErrors;  // Local variable for now

    // Initialize camera matrix with reasonable guess
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1200.0;  // fx estimate
    cameraMatrix.at<double>(1, 1) = 1200.0;  // fy estimate
    cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;   // cx
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;  // cy

    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs, tvecs;

    // Calibrate with minimal constraints (like MATLAB default)
    // REMOVED excessive flags that were causing 10x focal length error:
    // - CALIB_FIX_PRINCIPAL_POINT: Let OpenCV optimize cx,cy
    // - CALIB_FIX_ASPECT_RATIO: Allow fx != fy (real cameras have slight difference)
    // - CALIB_ZERO_TANGENT_DIST: Real lenses have tangential distortion
    int flags = 0;  // Use OpenCV defaults

    core::Logger::getInstance().debug("Running cv::calibrateCamera with " +
                                     std::to_string(imagePoints.size()) + " views (this may take 1-2 minutes)...");
    double rms = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize,
        cameraMatrix, distCoeffs,
        rvecs, tvecs, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-4)
    );
    core::Logger::getInstance().debug("cv::calibrateCamera completed with RMS: " + std::to_string(rms));

    // Compute per-image errors
    perImageErrors.clear();
    for (size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints[i], rvecs[i], tvecs[i],
                         cameraMatrix, distCoeffs, projectedPoints);

        double error = 0.0;
        for (size_t j = 0; j < imagePoints[i].size(); ++j) {
            double dx = imagePoints[i][j].x - projectedPoints[j].x;
            double dy = imagePoints[i][j].y - projectedPoints[j].y;
            error += std::sqrt(dx * dx + dy * dy);
        }
        perImageErrors.push_back(error / imagePoints[i].size());
    }

    core::Logger::getInstance().info("Intrinsic calibration RMS: " + std::to_string(rms));

    return rms < 5.0;  // Reasonable threshold
}

bool StereoCalibrationProcessor::calibrateStereo(
    const std::vector<std::vector<cv::Point2f>>& leftPoints,
    const std::vector<std::vector<cv::Point2f>>& rightPoints,
    const std::vector<std::vector<cv::Point3f>>& objectPoints,
    const cv::Size& imageSize,
    CalibrationResult& result) {

    core::Logger::getInstance().debug("Starting stereo calibration with " +
                                     std::to_string(objectPoints.size()) + " views");

    // CRITICAL FIX: Simplified distortion model for M12 6mm lenses (48° FOV)
    // RESEARCH FINDINGS (CALIBRATION_RESEARCH_FINDINGS.md):
    // - M12 6mm 1/2.7" has 48° diagonal FOV (wide-angle but NOT fisheye)
    // - CALIB_RATIONAL_MODEL (14 params: k1-k6, p1-p2) causes OVERFITTING
    // - OpenCV doesn't enforce monotonicity → converges to non-physical params
    // - Works perfect at center (r small) but diverges at edges (r large)
    // - GitHub Issue #15992: "reprojection error low at center, artifacts at edges"
    //
    // SOLUTION: Use polynomial model (8 params: k1-k3, p1-p2)
    // - k1, k2, k3: radial distortion (polynomial, always monotonic)
    // - p1, p2: tangential distortion (ESSENTIAL for M12 manufacturing tolerances)
    // - NO k4-k6 (rational model): prevents overfitting + non-monotonic convergence
    //
    // Expected improvement: 58px ORB error → 2-5px with 100 frames + edge coverage
    int flags = cv::CALIB_FIX_ASPECT_RATIO +
                cv::CALIB_USE_INTRINSIC_GUESS +
                cv::CALIB_SAME_FOCAL_LENGTH +
                cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5 + cv::CALIB_FIX_K6;
                // REMOVED: cv::CALIB_RATIONAL_MODEL (was causing overfitting)
                // REMOVED: cv::CALIB_ZERO_TANGENT_DIST (enables p1, p2 for M12 tolerances)
                // REMOVED: cv::CALIB_FIX_K3 (enables k3 for wide-angle 48° FOV)
                // This gives 8 parameters (k1-k3, p1-p2) vs 14 (rational model)

    core::Logger::getInstance().debug("Running cv::stereoCalibrate with " +
                                     std::to_string(objectPoints.size()) + " views (this may take 2-3 minutes)...");

    // Use same iteration count as OpenCV official sample
    double rms = cv::stereoCalibrate(
        objectPoints, leftPoints, rightPoints,
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.cameraMatrixRight, result.distCoeffsRight,
        imageSize, result.R, result.T, result.E, result.F,
        flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
    );
    core::Logger::getInstance().debug("cv::stereoCalibrate completed with RMS: " + std::to_string(rms));

    // DEBUG: Check translation vector T
    core::Logger::getInstance().debug("Translation vector T dimensions: " +
        std::to_string(result.T.rows) + "x" + std::to_string(result.T.cols));
    if (!result.T.empty()) {
        core::Logger::getInstance().debug("T values: [" +
            std::to_string(result.T.at<double>(0, 0)) + ", " +
            std::to_string(result.T.at<double>(1, 0)) + ", " +
            std::to_string(result.T.at<double>(2, 0)) + "]");
        core::Logger::getInstance().debug("T(X) = " + std::to_string(result.T.at<double>(0, 0)) + " mm (baseline)");
    } else {
        core::Logger::getInstance().error("Translation vector T is EMPTY!");
    }

    result.rmsReprojectionError = rms;

    // For now, use RMS as mean error (simplified)
    result.meanReprojectionErrorLeft = rms;
    result.meanReprojectionErrorRight = rms;
    result.maxReprojectionError = rms * 2.0;  // Estimate

    core::Logger::getInstance().info("Stereo calibration RMS: " + std::to_string(rms));

    return rms < 2.0;  // Reasonable threshold
}

void StereoCalibrationProcessor::validateCalibration(CalibrationResult& result) {
    // Basic validation of calibration parameters

    // Check if essential and fundamental matrices are valid
    if (result.E.empty() || result.F.empty()) {
        result.errors.push_back("Essential or Fundamental matrix is empty");
    }

    // Check if translation vector is reasonable (baseline)
    if (result.T.empty() || std::abs(result.T.at<double>(0, 0)) < 10.0) {
        result.warnings.push_back("Baseline seems too small");
    }

    core::Logger::getInstance().info("Calibration validation completed");
}

bool StereoCalibrationProcessor::saveCalibration(const CalibrationResult& result,
                                                const std::string& outputPath) {
    try {
        // Ensure output directory exists
        std::filesystem::path outputFilePath(outputPath);
        std::filesystem::path outputDir = outputFilePath.parent_path();

        if (!outputDir.empty() && !std::filesystem::exists(outputDir)) {
            core::Logger::getInstance().info("Creating calibration directory: " + outputDir.string());
            std::error_code ec;
            if (!std::filesystem::create_directories(outputDir, ec)) {
                throw std::runtime_error("Failed to create output directory: " + outputDir.string() +
                                       " (error: " + ec.message() + ")");
            }
        }

        // Verify directory is writable
        if (!outputDir.empty()) {
            std::filesystem::path testFile = outputDir / ".write_test";
            std::ofstream testStream(testFile);
            if (!testStream.good()) {
                throw std::runtime_error("Output directory is not writable: " + outputDir.string());
            }
            testStream.close();
            std::filesystem::remove(testFile);
        }

        saveCalibrationYAML(result, outputPath);

        // Also save rectification maps as binary files
        std::string basePath = outputPath.substr(0, outputPath.find_last_of('.'));
        saveRectificationMaps(result, basePath);

        core::Logger::getInstance().info("Calibration saved to: " + outputPath);
        return true;
    } catch (const std::exception& e) {
        core::Logger::getInstance().error("Failed to save calibration: " + std::string(e.what()));
        return false;
    }
}

void StereoCalibrationProcessor::saveCalibrationYAML(const CalibrationResult& result,
                                                    const std::string& outputPath) {
    cv::FileStorage fs(outputPath, cv::FileStorage::WRITE);

    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open file for writing: " + outputPath);
    }

    // Metadata
    fs << "calibration_date" << result.calibrationDate;
    fs << "dataset_timestamp" << result.datasetTimestamp;
    fs << "dataset_path" << result.datasetPath;

    // Pattern info
    fs << "pattern_type" << patternTypeToString(result.patternConfig.type);
    fs << "pattern_rows" << result.patternConfig.rows;
    fs << "pattern_cols" << result.patternConfig.cols;
    fs << "square_size_mm" << result.patternConfig.squareSizeMM;
    fs << "aruco_marker_size_mm" << result.patternConfig.arucoMarkerSizeMM;

    // Image info
    fs << "image_width" << result.imageSize.width;
    fs << "image_height" << result.imageSize.height;
    fs << "num_image_pairs" << result.numImagePairs;
    fs << "valid_image_pairs" << result.validImagePairs;

    // Camera matrices and distortion
    fs << "camera_matrix_left" << result.cameraMatrixLeft;
    fs << "distortion_coeffs_left" << result.distCoeffsLeft;
    fs << "camera_matrix_right" << result.cameraMatrixRight;
    fs << "distortion_coeffs_right" << result.distCoeffsRight;

    // Stereo extrinsics
    fs << "rotation_matrix" << result.R;
    fs << "translation_vector" << result.T;
    fs << "essential_matrix" << result.E;
    fs << "fundamental_matrix" << result.F;

    // Rectification
    fs << "rectification_transform_left" << result.R1;
    fs << "rectification_transform_right" << result.R2;
    fs << "projection_matrix_left" << result.P1;
    fs << "projection_matrix_right" << result.P2;
    fs << "disparity_to_depth_matrix" << result.Q;

    // Quality metrics
    fs << "rms_reprojection_error" << result.rmsReprojectionError;
    fs << "mean_reprojection_error_left" << result.meanReprojectionErrorLeft;
    fs << "mean_reprojection_error_right" << result.meanReprojectionErrorRight;
    fs << "max_reprojection_error" << result.maxReprojectionError;

    fs << "mean_epipolar_error" << result.meanEpipolarError;
    fs << "max_epipolar_error" << result.maxEpipolarError;

    fs << "baseline_mm" << result.baselineMM;
    fs << "baseline_expected_mm" << result.baselineExpectedMM;
    fs << "baseline_error_mm" << result.baselineErrorMM;
    fs << "baseline_error_percent" << result.baselineErrorPercent;

    // Validation
    fs << "quality_passed" << result.qualityPassed;
    fs << "rms_check" << result.rmsCheck;
    fs << "baseline_check" << result.baselineCheck;
    fs << "epipolar_check" << result.epipolarCheck;

    // System info
    fs << "opencv_version" << CV_VERSION;
    fs << "calibration_method" << "opencv_stereo_calibrate";
    fs << "calibration_duration_seconds" << result.calibrationDurationSeconds;

    fs.release();
}


CalibrationResult StereoCalibrationProcessor::loadCalibration(const std::string& calibPath) {
    return loadCalibrationYAML(calibPath);
}

CalibrationResult StereoCalibrationProcessor::loadCalibrationYAML(const std::string& calibPath) {
    CalibrationResult result;

    cv::FileStorage fs(calibPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open calibration file: " + calibPath);
    }

    // Load all parameters
    fs["calibration_date"] >> result.calibrationDate;
    fs["dataset_timestamp"] >> result.datasetTimestamp;
    fs["dataset_path"] >> result.datasetPath;

    std::string patternType;
    fs["pattern_type"] >> patternType;
    result.patternConfig.type = parsePatternType(patternType);
    fs["pattern_rows"] >> result.patternConfig.rows;
    fs["pattern_cols"] >> result.patternConfig.cols;
    fs["square_size_mm"] >> result.patternConfig.squareSizeMM;
    fs["aruco_marker_size_mm"] >> result.patternConfig.arucoMarkerSizeMM;

    int width, height;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    result.imageSize = cv::Size(width, height);
    fs["num_image_pairs"] >> result.numImagePairs;
    fs["valid_image_pairs"] >> result.validImagePairs;

    fs["camera_matrix_left"] >> result.cameraMatrixLeft;
    fs["distortion_coeffs_left"] >> result.distCoeffsLeft;
    fs["camera_matrix_right"] >> result.cameraMatrixRight;
    fs["distortion_coeffs_right"] >> result.distCoeffsRight;

    fs["rotation_matrix"] >> result.R;
    fs["translation_vector"] >> result.T;
    fs["essential_matrix"] >> result.E;
    fs["fundamental_matrix"] >> result.F;

    fs["rectification_transform_left"] >> result.R1;
    fs["rectification_transform_right"] >> result.R2;
    fs["projection_matrix_left"] >> result.P1;
    fs["projection_matrix_right"] >> result.P2;
    fs["disparity_to_depth_matrix"] >> result.Q;

    fs["rms_reprojection_error"] >> result.rmsReprojectionError;
    fs["mean_reprojection_error_left"] >> result.meanReprojectionErrorLeft;
    fs["mean_reprojection_error_right"] >> result.meanReprojectionErrorRight;
    fs["max_reprojection_error"] >> result.maxReprojectionError;

    fs["mean_epipolar_error"] >> result.meanEpipolarError;
    fs["max_epipolar_error"] >> result.maxEpipolarError;

    fs["baseline_mm"] >> result.baselineMM;
    fs["baseline_expected_mm"] >> result.baselineExpectedMM;
    fs["baseline_error_mm"] >> result.baselineErrorMM;
    fs["baseline_error_percent"] >> result.baselineErrorPercent;

    fs["quality_passed"] >> result.qualityPassed;
    fs["rms_check"] >> result.rmsCheck;
    fs["baseline_check"] >> result.baselineCheck;
    fs["epipolar_check"] >> result.epipolarCheck;

    fs["calibration_duration_seconds"] >> result.calibrationDurationSeconds;

    fs.release();

    // Regenerate rectification maps with CV_16SC2 format
    cv::initUndistortRectifyMap(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.R1, result.P1, result.imageSize, CV_16SC2,
        result.map1Left, result.map2Left
    );
    cv::initUndistortRectifyMap(
        result.cameraMatrixRight, result.distCoeffsRight,
        result.R2, result.P2, result.imageSize, CV_16SC2,
        result.map1Right, result.map2Right
    );

    core::Logger::getInstance().info("Calibration loaded from: " + calibPath);

    return result;
}

bool StereoCalibrationProcessor::setAsSystemDefault(const std::string& calibPath) {
    try {
        // Create symlink to default location
        std::string defaultPath = "/unlook_calib/default.yaml";

        // Remove existing symlink if it exists
        if (fs::exists(defaultPath)) {
            fs::remove(defaultPath);
        }

        // Create new symlink
        fs::create_symlink(calibPath, defaultPath);

        core::Logger::getInstance().info("Set system default calibration to: " + calibPath);
        return true;
    } catch (const std::exception& e) {
        core::Logger::getInstance().error("Failed to set default calibration: " + std::string(e.what()));
        return false;
    }
}

void StereoCalibrationProcessor::setProgressCallback(std::function<void(const std::string&)> callback) {
    progressCallback_ = callback;
}

void StereoCalibrationProcessor::reportProgress(const std::string& message) {
    if (progressCallback_) {
        progressCallback_(message);
    }
    core::Logger::getInstance().info(message);

    // Force flush to ensure log is written immediately
    std::cout << "[PROGRESS] " << message << std::endl;
    std::cout.flush();
}


} // namespace calibration
} // namespace unlook