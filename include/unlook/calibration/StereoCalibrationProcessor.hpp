#pragma once

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <nlohmann/json.hpp>

namespace unlook {
namespace calibration {

/**
 * @brief Pattern type for calibration
 */
enum class PatternType {
    CHECKERBOARD,
    CHARUCO,
    CIRCLE_GRID
};

/**
 * @brief Pattern configuration for calibration target
 */
struct PatternConfig {
    PatternType type;
    int rows;
    int cols;
    float squareSizeMM;
    float arucoMarkerSizeMM;  // For ChArUco only
    cv::aruco::PredefinedDictionaryType arucoDict;  // For ChArUco only (OpenCV 4.x API)

    // Default constructor - ChArUco 7x10 pattern, 24mm squares
    PatternConfig()
        : type(PatternType::CHARUCO)
        , rows(7)
        , cols(10)
        , squareSizeMM(24.0f)
        , arucoMarkerSizeMM(17.0f)
        , arucoDict(cv::aruco::DICT_4X4_250)
    {}

    // Convert to/from string
    std::string typeToString() const;
    static PatternType stringToType(const std::string& str);
};

/**
 * @brief Calibration result containing all parameters and quality metrics
 */
struct CalibrationResult {
    // Timestamps
    std::string calibrationDate;
    std::string datasetTimestamp;
    std::string datasetPath;

    // Pattern info
    PatternConfig patternConfig;

    // Image info
    cv::Size imageSize;
    int numImagePairs;
    int validImagePairs;

    // Camera intrinsics
    cv::Mat cameraMatrixLeft;
    cv::Mat distCoeffsLeft;
    cv::Mat cameraMatrixRight;
    cv::Mat distCoeffsRight;

    // Stereo extrinsics
    cv::Mat R;  // Rotation matrix
    cv::Mat T;  // Translation vector (baseline!)
    cv::Mat E;  // Essential matrix
    cv::Mat F;  // Fundamental matrix

    // Rectification
    cv::Mat R1, R2;  // Rectification transforms
    cv::Mat P1, P2;  // Projection matrices
    cv::Mat Q;       // Disparity-to-depth matrix

    // Rectification maps
    cv::Mat map1Left, map2Left;
    cv::Mat map1Right, map2Right;

    // Quality metrics
    double rmsReprojectionError;
    double meanReprojectionErrorLeft;
    double meanReprojectionErrorRight;
    double maxReprojectionError;

    double meanEpipolarError;
    double maxEpipolarError;

    // Baseline validation
    double baselineMM;
    double baselineExpectedMM;
    double baselineErrorMM;
    double baselineErrorPercent;

    // Validation flags
    bool qualityPassed;
    std::string rmsCheck;       // "PASS" | "WARNING" | "FAIL"
    std::string baselineCheck;
    std::string epipolarCheck;

    std::vector<std::string> warnings;
    std::vector<std::string> errors;

    // Timing
    double calibrationDurationSeconds;

    // System info
    std::string opencvVersion;
    std::string calibrationMethod;

    CalibrationResult()
        : numImagePairs(0)
        , validImagePairs(0)
        , rmsReprojectionError(0.0)
        , meanReprojectionErrorLeft(0.0)
        , meanReprojectionErrorRight(0.0)
        , maxReprojectionError(0.0)
        , meanEpipolarError(0.0)
        , maxEpipolarError(0.0)
        , baselineMM(0.0)
        , baselineExpectedMM(70.0)
        , baselineErrorMM(0.0)
        , baselineErrorPercent(0.0)
        , qualityPassed(false)
        , calibrationDurationSeconds(0.0)
        , opencvVersion(CV_VERSION)
        , calibrationMethod("opencv_stereo_calibrate")
    {}
};

/**
 * @brief Stereo calibration processor - main calibration backend
 *
 * Performs complete stereo calibration from captured dataset:
 * 1. Load dataset images
 * 2. Detect calibration pattern
 * 3. Calibrate intrinsics (left and right)
 * 4. Calibrate stereo extrinsics
 * 5. Compute rectification transforms
 * 6. Validate calibration quality
 * 7. Save results to YAML
 */
class StereoCalibrationProcessor {
public:
    StereoCalibrationProcessor();
    ~StereoCalibrationProcessor();

    /**
     * @brief Main calibration function - process entire dataset
     * @param datasetPath Path to dataset directory
     * @return CalibrationResult with all parameters and quality metrics
     */
    CalibrationResult calibrateFromDataset(const std::string& datasetPath);

    /**
     * @brief Save calibration to YAML file
     * @param result Calibration result to save
     * @param outputPath Output YAML file path
     * @return true if saved successfully
     */
    bool saveCalibration(const CalibrationResult& result, const std::string& outputPath);

    /**
     * @brief Load calibration from YAML file
     * @param calibPath Path to calibration YAML file
     * @return CalibrationResult loaded from file
     */
    CalibrationResult loadCalibration(const std::string& calibPath);

    /**
     * @brief Set calibration as system default
     * @param calibPath Path to calibration file
     * @return true if set successfully
     */
    bool setAsSystemDefault(const std::string& calibPath);

    /**
     * @brief Set progress callback for UI updates
     * @param callback Function to call with progress messages
     */
    void setProgressCallback(std::function<void(const std::string&)> callback);

private:
    // Dataset loading
    bool loadDatasetImages(const std::string& datasetPath,
                          std::vector<cv::Mat>& leftImages,
                          std::vector<cv::Mat>& rightImages);

    nlohmann::json loadDatasetInfo(const std::string& datasetPath);

    // Pattern detection
    bool detectPattern(const cv::Mat& image,
                      const PatternConfig& config,
                      std::vector<cv::Point2f>& corners);

    bool detectCheckerboard(const cv::Mat& image,
                           const PatternConfig& config,
                           std::vector<cv::Point2f>& corners);

    bool detectCharuco(const cv::Mat& image,
                      const PatternConfig& config,
                      std::vector<cv::Point2f>& corners);

    // Calibration steps
    bool calibrateIntrinsics(const std::vector<std::vector<cv::Point2f>>& imagePoints,
                            const std::vector<std::vector<cv::Point3f>>& objectPoints,
                            const cv::Size& imageSize,
                            cv::Mat& cameraMatrix,
                            cv::Mat& distCoeffs);

    bool calibrateStereo(const std::vector<std::vector<cv::Point2f>>& leftPoints,
                        const std::vector<std::vector<cv::Point2f>>& rightPoints,
                        const std::vector<std::vector<cv::Point3f>>& objectPoints,
                        const cv::Size& imageSize,
                        CalibrationResult& result);

    // Validation
    void validateCalibration(CalibrationResult& result);

    double computeEpipolarError(const CalibrationResult& result,
                               const std::vector<cv::Point2f>& leftPoints,
                               const std::vector<cv::Point2f>& rightPoints);

    // YAML I/O
    void saveCalibrationYAML(const CalibrationResult& result,
                            const std::string& outputPath);

    CalibrationResult loadCalibrationYAML(const std::string& calibPath);

    // Progress reporting
    void reportProgress(const std::string& message);

    // Members
    std::function<void(const std::string&)> progressCallback_;
};

} // namespace calibration
} // namespace unlook
