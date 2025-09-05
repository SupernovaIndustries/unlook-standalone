/**
 * @file types.h
 * @brief Common type definitions for Unlook 3D Scanner
 * 
 * This file contains fundamental type definitions, enums, and structures
 * used throughout the Unlook library.
 */

#ifndef UNLOOK_CORE_TYPES_H
#define UNLOOK_CORE_TYPES_H

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>

namespace unlook {
namespace core {

/**
 * @brief Logging levels for the library
 */
enum class LogLevel {
    DEBUG = 0,
    INFO = 1,
    WARNING = 2,
    ERROR = 3,
    FATAL = 4
};

/**
 * @brief Camera identifiers
 */
enum class CameraId {
    LEFT = 0,   ///< Left camera (Camera 1 in hardware - MASTER)
    RIGHT = 1,  ///< Right camera (Camera 0 in hardware - SLAVE)
    UNKNOWN = -1
};

/**
 * @brief Calibration pattern types
 */
enum class PatternType {
    CHECKERBOARD,
    CHARUCO_BOARD,
    CIRCLES_GRID,
    ASYMMETRIC_CIRCLES_GRID
};

/**
 * @brief Stereo matching algorithms
 */
enum class StereoAlgorithm {
    SGBM,           ///< Semi-Global Block Matching
    BM,             ///< Block Matching
    SGBM_3WAY,      ///< 3-way SGBM
    ELAS,           ///< Efficient Large-scale Stereo
    LIBELAS         ///< libELAS implementation
};

/**
 * @brief Hardware synchronization modes
 */
enum class SyncMode {
    SOFTWARE,       ///< Software synchronization
    HARDWARE_XVS,   ///< Hardware XVS synchronization
    HARDWARE_XHS,   ///< Hardware XHS synchronization
    HARDWARE_BOTH   ///< Both XVS and XHS synchronization
};

/**
 * @brief Point cloud formats for export
 */
enum class PointCloudFormat {
    PLY,
    PCD,
    XYZ,
    OBJ
};

/**
 * @brief Time measurement using high-resolution clock
 */
using Timestamp = std::chrono::high_resolution_clock::time_point;
using Duration = std::chrono::high_resolution_clock::duration;

/**
 * @brief 2D point with sub-pixel precision
 */
using Point2f = cv::Point2f;
using Point2d = cv::Point2d;

/**
 * @brief 3D point for world coordinates
 */
using Point3f = cv::Point3f;
using Point3d = cv::Point3d;

/**
 * @brief Image size structure
 */
using Size = cv::Size;

/**
 * @brief Rectangle structure
 */
using Rect = cv::Rect;

/**
 * @brief Matrix types for transformations
 */
using Matrix3d = cv::Mat;  ///< 3x3 double matrix
using Matrix4d = cv::Mat;  ///< 4x4 double matrix
using Vector3d = cv::Mat;  ///< 3x1 double vector

/**
 * @brief Camera intrinsic parameters
 */
struct CameraIntrinsics {
    cv::Mat camera_matrix;      ///< 3x3 camera matrix
    cv::Mat distortion_coeffs;  ///< Distortion coefficients
    cv::Size image_size;        ///< Image dimensions
    double focal_length_mm;     ///< Physical focal length in mm
    double pixel_size_um;       ///< Pixel size in micrometers
    
    CameraIntrinsics() : focal_length_mm(6.0), pixel_size_um(3.45) {}
};

/**
 * @brief Stereo camera parameters
 */
struct StereoParameters {
    CameraIntrinsics left_camera;
    CameraIntrinsics right_camera;
    cv::Mat R;          ///< Rotation matrix between cameras
    cv::Mat T;          ///< Translation vector between cameras
    cv::Mat E;          ///< Essential matrix
    cv::Mat F;          ///< Fundamental matrix
    cv::Mat R1, R2;     ///< Rectification rotation matrices
    cv::Mat P1, P2;     ///< Rectification projection matrices
    cv::Mat Q;          ///< Disparity-to-depth mapping matrix
    double baseline_mm; ///< Baseline distance in millimeters
    double rms_error;   ///< RMS reprojection error
    
    StereoParameters() : baseline_mm(70.017), rms_error(0.0) {}
};

/**
 * @brief Calibration target configuration
 */
struct CalibrationTarget {
    PatternType pattern_type;
    cv::Size pattern_size;      ///< Number of inner corners/circles
    double square_size_mm;      ///< Size of squares/circles in mm
    double marker_size_mm;      ///< ArUco marker size (for ChArUco)
    int dictionary_id;          ///< ArUco dictionary ID
    
    CalibrationTarget() 
        : pattern_type(PatternType::CHECKERBOARD)
        , pattern_size(7, 10)
        , square_size_mm(24.0)
        , marker_size_mm(17.0)
        , dictionary_id(cv::aruco::DICT_4X4_250) {}
};

/**
 * @brief Image capture configuration
 */
struct CaptureConfig {
    cv::Size resolution;        ///< Image resolution
    int format;                 ///< Pixel format (SBGGR10, etc.)
    int framerate;              ///< Target framerate
    SyncMode sync_mode;         ///< Synchronization mode
    bool auto_exposure;         ///< Auto exposure enabled
    bool auto_gain;             ///< Auto gain enabled
    double exposure_us;         ///< Manual exposure in microseconds
    double gain_db;             ///< Manual gain in dB
    
    CaptureConfig()
        : resolution(1456, 1088)
        , format(CV_8UC1)  // Will be converted from SBGGR10
        , framerate(30)
        , sync_mode(SyncMode::HARDWARE_XVS)
        , auto_exposure(true)
        , auto_gain(true)
        , exposure_us(33000)
        , gain_db(0.0) {}
};

/**
 * @brief Stereo matching parameters
 */
struct StereoMatchingParams {
    StereoAlgorithm algorithm;
    int min_disparity;
    int max_disparity;
    int block_size;
    int p1, p2;                 ///< SGBM penalty parameters
    int disp_12_max_diff;
    int pre_filter_cap;
    int uniqueness_ratio;
    int speckle_window_size;
    int speckle_range;
    bool use_mode_HH;           ///< SGBM mode
    
    StereoMatchingParams()
        : algorithm(StereoAlgorithm::SGBM)
        , min_disparity(0)
        , max_disparity(128)
        , block_size(5)
        , p1(8 * 3 * 5 * 5)
        , p2(32 * 3 * 5 * 5)
        , disp_12_max_diff(1)
        , pre_filter_cap(63)
        , uniqueness_ratio(10)
        , speckle_window_size(100)
        , speckle_range(32)
        , use_mode_HH(false) {}
};

/**
 * @brief 3D point with confidence/quality measure
 */
struct Point3DWithQuality {
    Point3f point;
    float confidence;       ///< Quality/confidence [0.0, 1.0]
    uint8_t intensity;      ///< Pixel intensity
    
    Point3DWithQuality() : confidence(0.0f), intensity(0) {}
    Point3DWithQuality(const Point3f& p, float conf = 1.0f, uint8_t intens = 255)
        : point(p), confidence(conf), intensity(intens) {}
};

/**
 * @brief Point cloud data structure
 */
struct PointCloud {
    std::vector<Point3DWithQuality> points;
    cv::Mat colors;             ///< Optional color information (CV_8UC3)
    Timestamp timestamp;        ///< Capture timestamp
    StereoParameters stereo_params; ///< Parameters used for generation
    
    PointCloud() {
        timestamp = std::chrono::high_resolution_clock::now();
    }
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
    void clear() { points.clear(); colors.release(); }
};

/**
 * @brief Hardware LED configuration
 */
struct LEDConfig {
    uint8_t i2c_bus;            ///< I2C bus number
    uint8_t i2c_address;        ///< I2C device address
    uint8_t strobe_gpio;        ///< GPIO pin for strobe control
    double max_current_ma;      ///< Maximum current in mA
    double pulse_duration_us;   ///< Pulse duration in microseconds
    
    LEDConfig()
        : i2c_bus(4)
        , i2c_address(0x30)
        , strobe_gpio(27)
        , max_current_ma(1000.0)
        , pulse_duration_us(1000.0) {}
};

/**
 * @brief Callback function types
 */
using ImageCallback = std::function<void(const cv::Mat&, CameraId, Timestamp)>;
using StereoCallback = std::function<void(const cv::Mat&, const cv::Mat&, Timestamp)>;
using PointCloudCallback = std::function<void(const PointCloud&)>;
using ErrorCallback = std::function<void(const std::string&)>;

/**
 * @brief Result status for operations
 */
enum class ResultStatus {
    SUCCESS = 0,
    ERROR_INVALID_PARAMETER,
    ERROR_HARDWARE_FAILURE,
    ERROR_FILE_IO,
    ERROR_CALIBRATION_FAILED,
    ERROR_CAMERA_NOT_FOUND,
    ERROR_INSUFFICIENT_MEMORY,
    ERROR_TIMEOUT,
    ERROR_UNKNOWN
};

/**
 * @brief Operation result with status and optional message
 */
template<typename T = void>
struct Result {
    ResultStatus status;
    std::string message;
    T data;
    
    Result(ResultStatus s = ResultStatus::SUCCESS, const std::string& msg = "")
        : status(s), message(msg) {}
    
    Result(const T& d, ResultStatus s = ResultStatus::SUCCESS, const std::string& msg = "")
        : status(s), message(msg), data(d) {}
    
    bool isSuccess() const { return status == ResultStatus::SUCCESS; }
    bool hasError() const { return status != ResultStatus::SUCCESS; }
    
    operator bool() const { return isSuccess(); }
};

} // namespace core
} // namespace unlook

#endif // UNLOOK_CORE_TYPES_H