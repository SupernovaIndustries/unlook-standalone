#pragma once

#include <opencv2/opencv.hpp>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <chrono>

namespace unlook {
namespace core {

/**
 * @brief Core data types and structures used throughout the Unlook scanner system
 */

// Forward declarations
struct CameraConfig;
struct StereoConfig;
struct DepthResult;

/// Result codes for API operations
enum class ResultCode : int32_t {
    SUCCESS = 0,
    ERROR_GENERIC = -1,
    ERROR_INVALID_PARAMETER = -2,
    ERROR_NOT_INITIALIZED = -3,
    ERROR_ALREADY_INITIALIZED = -4,
    ERROR_HARDWARE_FAILURE = -5,
    ERROR_CALIBRATION_INVALID = -6,
    ERROR_CAMERA_NOT_FOUND = -7,
    ERROR_CAMERA_ACCESS_DENIED = -8,
    ERROR_MEMORY_ALLOCATION = -9,
    ERROR_FILE_NOT_FOUND = -10,
    ERROR_FILE_IO = -11,
    ERROR_TIMEOUT = -12,
    ERROR_THREAD_FAILURE = -13,
    ERROR_SYNC_FAILURE = -14
};

/// Scanner operation modes
enum class ScannerMode {
    STANDALONE,   ///< Direct GUI operation on Raspberry Pi
    COMPANION     ///< External PC control via shared library API
};

/// Hardware synchronization status
enum class SyncStatus {
    NOT_INITIALIZED,
    SYNCHRONIZING,
    SYNCHRONIZED,
    SYNC_FAILED
};

// Camera identifiers
enum class CameraId {
    LEFT = 0,   // Camera 1 - Master camera
    RIGHT = 1   // Camera 0 - Slave camera
};

// Camera states
enum class CameraState {
    NOT_INITIALIZED,
    DISCONNECTED,
    INITIALIZING,
    READY,
    CAPTURING,
    ERROR
};

// Processing algorithms
enum class StereoAlgorithm {
    SGBM_OPENCV,
    BOOFCV_BASIC,
    BOOFCV_PRECISE
};

// Depth map quality levels
enum class DepthQuality {
    FAST_LOW,
    BALANCED,
    SLOW_HIGH
};

/**
 * @brief Camera configuration parameters
 */
struct CameraConfig {
    // Resolution
    int width = 1456;
    int height = 1088;
    
    // Exposure settings
    bool auto_exposure = true;
    double exposure_time_us = 10000.0;  // microseconds
    
    // Gain settings
    bool auto_gain = true;
    double gain = 1.0;
    
    // Frame rate
    double fps = 30.0;
    
    // Hardware sync
    bool hardware_sync = true;
    bool is_master = false; // LEFT camera is master
};

/**
 * @brief Stereo processing configuration
 */
struct StereoConfig {
    StereoAlgorithm algorithm = StereoAlgorithm::SGBM_OPENCV;
    DepthQuality quality = DepthQuality::BALANCED;
    
    // SGBM parameters
    int min_disparity = 0;
    int num_disparities = 128;
    int block_size = 5;
    int p1 = 8 * 3 * 5 * 5;  // 8*number_of_image_channels*block_size^2
    int p2 = 32 * 3 * 5 * 5; // 32*number_of_image_channels*block_size^2
    int disp12_max_diff = 1;
    int pre_filter_cap = 63;
    int uniqueness_ratio = 10;
    int speckle_window_size = 100;
    int speckle_range = 32;
    int mode = cv::StereoSGBM::MODE_SGBM;
};

/**
 * @brief Depth processing result
 */
struct DepthResult {
    cv::Mat depth_map;
    cv::Mat disparity_map;
    cv::Mat confidence_map;
    
    // Quality metrics
    double mean_depth = 0.0;
    double std_depth = 0.0;
    double coverage_ratio = 0.0;  // Percentage of valid depth pixels
    
    // Processing time
    double processing_time_ms = 0.0;
    
    // Error information
    bool success = false;
    std::string error_message;
};

/**
 * @brief Camera frame data
 */
struct CameraFrame {
    cv::Mat image;
    uint64_t timestamp_ns;
    CameraId camera_id;
    bool valid = false;
};

/**
 * @brief Stereo frame pair
 */
struct StereoFramePair {
    CameraFrame left_frame;
    CameraFrame right_frame;
    double sync_error_ms = 0.0;  // Synchronization error between frames
    bool synchronized = false;
};

// Callback types
using CameraFrameCallback = std::function<void(const CameraFrame&)>;
using StereoFrameCallback = std::function<void(const StereoFramePair&)>;
using DepthResultCallback = std::function<void(const DepthResult&)>;
using ErrorCallback = std::function<void(const std::string& error)>;

// Smart pointer aliases
template<typename T>
using UniquePtr = std::unique_ptr<T>;

template<typename T>
using SharedPtr = std::shared_ptr<T>;

template<typename T>
using WeakPtr = std::weak_ptr<T>;

/// Version information
struct Version {
    uint8_t major;
    uint8_t minor; 
    uint8_t patch;
    std::string build;
    
    std::string toString() const {
        return std::to_string(major) + "." + 
               std::to_string(minor) + "." + 
               std::to_string(patch) + 
               (build.empty() ? "" : "-" + build);
    }
};

/// Current API version
const Version API_VERSION{1, 0, 0, "dev"};

/// Scanner status information
struct ScannerStatus {
    bool initialized;
    ScannerMode mode;
    SyncStatus sync_status;
    bool calibration_loaded;
    std::chrono::system_clock::time_point last_update;
    std::string error_message;
};

/// Calibration quality metrics
struct CalibrationQuality {
    double rms_error;           ///< RMS reprojection error in pixels
    double baseline_mm;         ///< Stereo baseline in millimeters
    double precision_mm;        ///< Theoretical precision at 100mm
    bool is_valid;              ///< Overall calibration validity
};

/// Point cloud export formats
enum class ExportFormat {
    PLY_ASCII,
    PLY_BINARY,
    OBJ,
    XYZ
};

// Helper function to convert ResultCode to string
std::string resultCodeToString(ResultCode code);

} // namespace core
} // namespace unlook