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
void computeEpipolarErrors(
    const CalibrationResult& result,
    const std::vector<std::vector<cv::Point2f>>& leftPoints,
    const std::vector<std::vector<cv::Point2f>>& rightPoints,
    double& meanError,
    double& maxError) {

    meanError = 0.0;
    maxError = 0.0;
    int totalPoints = 0;

    for (size_t i = 0; i < leftPoints.size() && i < rightPoints.size(); ++i) {
        if (leftPoints[i].size() != rightPoints[i].size()) continue;

        // Sample subset of points for efficiency
        size_t step = std::max(size_t(1), leftPoints[i].size() / 10);

        for (size_t j = 0; j < leftPoints[i].size(); j += step) {
            // Convert to homogeneous coordinates
            cv::Mat pt1 = (cv::Mat_<double>(3, 1) <<
                          leftPoints[i][j].x, leftPoints[i][j].y, 1.0);
            cv::Mat pt2 = (cv::Mat_<double>(3, 1) <<
                          rightPoints[i][j].x, rightPoints[i][j].y, 1.0);

            // Compute epipolar line in second image
            cv::Mat epiline2 = result.F * pt1;

            // Distance from point to epipolar line
            double a = epiline2.at<double>(0, 0);
            double b = epiline2.at<double>(1, 0);
            double c = epiline2.at<double>(2, 0);

            double distance = std::abs(a * rightPoints[i][j].x + b * rightPoints[i][j].y + c) /
                            std::sqrt(a * a + b * b);

            meanError += distance;
            maxError = std::max(maxError, distance);
            totalPoints++;
        }
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

    CalibrationResult result;
    result.datasetPath = datasetPath;
    result.calibrationDate = getCurrentTimestamp();

    // 1. Load dataset info JSON
    core::Logger::getInstance().info("Loading dataset from: " + datasetPath);
    auto datasetInfo = loadDatasetInfo(datasetPath);

    result.datasetTimestamp = datasetInfo["dataset_info"]["timestamp"].get<std::string>();

    // Parse pattern configuration
    auto& patternConfig = datasetInfo["pattern_config"];
    result.patternConfig.type = parsePatternType(patternConfig["type"].get<std::string>());
    result.patternConfig.rows = patternConfig["rows"].get<int>();
    result.patternConfig.cols = patternConfig["cols"].get<int>();
    result.patternConfig.squareSizeMM = patternConfig["square_size_mm"].get<float>();

    if (patternConfig.contains("aruco_marker_size_mm")) {
        result.patternConfig.arucoMarkerSizeMM = patternConfig["aruco_marker_size_mm"].get<float>();
    }
    if (patternConfig.contains("aruco_dictionary")) {
        // Default to DICT_4X4_250 for now
        result.patternConfig.arucoDict = cv::aruco::DICT_4X4_250;
    }

    // 2. Load all image pairs
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

    core::Logger::getInstance().info("Loaded " + std::to_string(result.numImagePairs) +
                                    " image pairs at " + std::to_string(result.imageSize.width) +
                                    "x" + std::to_string(result.imageSize.height));

    // 3. Detect patterns in all images
    core::Logger::getInstance().info("Detecting calibration patterns...");
    std::vector<std::vector<cv::Point2f>> leftImagePoints;
    std::vector<std::vector<cv::Point2f>> rightImagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    PatternDetector detector(result.patternConfig);
    int validPairs = 0;

    for (size_t i = 0; i < leftImages.size(); ++i) {
        std::vector<cv::Point2f> cornersLeft, cornersRight;
        cv::Mat overlayLeft, overlayRight;

        bool leftDetected = detector.detect(leftImages[i], cornersLeft, overlayLeft);
        bool rightDetected = detector.detect(rightImages[i], cornersRight, overlayRight);

        if (leftDetected && rightDetected &&
            cornersLeft.size() == cornersRight.size() &&
            cornersLeft.size() >= 4) {

            leftImagePoints.push_back(cornersLeft);
            rightImagePoints.push_back(cornersRight);
            objectPoints.push_back(generateObjectPoints(result.patternConfig));

            validPairs++;
            core::Logger::getInstance().debug("Pair " + std::to_string(i) +
                                            ": Pattern detected (" +
                                            std::to_string(cornersLeft.size()) + " corners)");
        } else {
            std::string reason = "Unknown";
            if (!leftDetected) reason = "Left pattern not detected";
            else if (!rightDetected) reason = "Right pattern not detected";
            else if (cornersLeft.size() != cornersRight.size()) reason = "Corner count mismatch";
            else if (cornersLeft.size() < 4) reason = "Too few corners";

            core::Logger::getInstance().warning("Pair " + std::to_string(i) + ": " + reason);
            result.warnings.push_back("Image pair " + std::to_string(i) + ": " + reason);
        }
    }

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

    // 4. Calibrate left camera intrinsics
    core::Logger::getInstance().info("Calibrating left camera intrinsics...");
    if (!calibrateIntrinsics(leftImagePoints, objectPoints, result.imageSize,
                            result.cameraMatrixLeft, result.distCoeffsLeft)) {
        result.errors.push_back("Left camera intrinsic calibration failed");
        result.qualityPassed = false;
        return result;
    }

    // 5. Calibrate right camera intrinsics
    core::Logger::getInstance().info("Calibrating right camera intrinsics...");
    if (!calibrateIntrinsics(rightImagePoints, objectPoints, result.imageSize,
                            result.cameraMatrixRight, result.distCoeffsRight)) {
        result.errors.push_back("Right camera intrinsic calibration failed");
        result.qualityPassed = false;
        return result;
    }

    // 6. Stereo calibration
    core::Logger::getInstance().info("Performing stereo calibration...");
    if (!calibrateStereo(leftImagePoints, rightImagePoints, objectPoints,
                        result.imageSize, result)) {
        result.errors.push_back("Stereo calibration failed");
        result.qualityPassed = false;
        return result;
    }

    // 7. Compute rectification
    core::Logger::getInstance().info("Computing rectification transforms...");
    cv::stereoRectify(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.cameraMatrixRight, result.distCoeffsRight,
        result.imageSize, result.R, result.T,
        result.R1, result.R2, result.P1, result.P2, result.Q,
        cv::CALIB_ZERO_DISPARITY, -1, result.imageSize
    );

    // 8. Compute rectification maps
    core::Logger::getInstance().info("Computing rectification maps...");
    cv::initUndistortRectifyMap(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.R1, result.P1, result.imageSize, CV_32FC1,
        result.map1Left, result.map2Left
    );
    cv::initUndistortRectifyMap(
        result.cameraMatrixRight, result.distCoeffsRight,
        result.R2, result.P2, result.imageSize, CV_32FC1,
        result.map1Right, result.map2Right
    );

    // 9. Compute baseline
    result.baselineMM = std::abs(result.T.at<double>(0, 0));
    result.baselineExpectedMM = 70.0;  // Expected baseline for Unlook scanner
    result.baselineErrorMM = std::abs(result.baselineMM - result.baselineExpectedMM);
    result.baselineErrorPercent = (result.baselineErrorMM / result.baselineExpectedMM) * 100.0;

    // 10. Validate calibration and compute quality metrics
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

    std::vector<double> perImageErrors;  // Local variable for now

    // Initialize camera matrix with reasonable guess
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0, 0) = 1200.0;  // fx estimate
    cameraMatrix.at<double>(1, 1) = 1200.0;  // fy estimate
    cameraMatrix.at<double>(0, 2) = imageSize.width / 2.0;   // cx
    cameraMatrix.at<double>(1, 2) = imageSize.height / 2.0;  // cy

    distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

    std::vector<cv::Mat> rvecs, tvecs;

    // Calibrate with recommended flags
    int flags = cv::CALIB_FIX_ASPECT_RATIO |
                cv::CALIB_FIX_PRINCIPAL_POINT |
                cv::CALIB_ZERO_TANGENT_DIST;

    double rms = cv::calibrateCamera(
        objectPoints, imagePoints, imageSize,
        cameraMatrix, distCoeffs,
        rvecs, tvecs, flags
    );

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

    // Stereo calibration with fixed intrinsics
    int flags = cv::CALIB_FIX_INTRINSIC;

    double rms = cv::stereoCalibrate(
        objectPoints, leftPoints, rightPoints,
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.cameraMatrixRight, result.distCoeffsRight,
        imageSize, result.R, result.T, result.E, result.F,
        flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-5)
    );

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

    // Regenerate rectification maps
    cv::initUndistortRectifyMap(
        result.cameraMatrixLeft, result.distCoeffsLeft,
        result.R1, result.P1, result.imageSize, CV_32FC1,
        result.map1Left, result.map2Left
    );
    cv::initUndistortRectifyMap(
        result.cameraMatrixRight, result.distCoeffsRight,
        result.R2, result.P2, result.imageSize, CV_32FC1,
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
}


} // namespace calibration
} // namespace unlook