#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

namespace unlook {
namespace face {

/**
 * @brief High-performance face detection configuration
 */
struct FaceDetectionConfig {
    // Detection parameters
    float scale_factor = 1.1f;        ///< Scale factor for multi-scale detection
    int min_neighbors = 3;            ///< Minimum neighbor detections required
    cv::Size min_face_size{50, 50};   ///< Minimum face size in pixels
    cv::Size max_face_size{800, 800}; ///< Maximum face size in pixels

    // Quality filtering
    float confidence_threshold = 0.7f; ///< Minimum detection confidence
    bool enable_pose_filtering = true; ///< Filter faces with extreme poses
    float max_yaw_angle = 30.0f;      ///< Maximum yaw angle (degrees)
    float max_pitch_angle = 20.0f;    ///< Maximum pitch angle (degrees)
    float max_roll_angle = 15.0f;     ///< Maximum roll angle (degrees)

    // Image preprocessing
    bool enable_histogram_equalization = true; ///< Apply histogram equalization
    bool enable_gaussian_blur = false;         ///< Apply Gaussian blur
    int blur_kernel_size = 3;                  ///< Blur kernel size

    // Performance optimization
    bool enable_multithreading = true;         ///< Use multi-threaded detection
    int max_threads = 4;                      ///< Maximum threads for detection
    bool enable_gpu_acceleration = false;     ///< Use GPU acceleration (if available)

    // Banking-grade requirements
    bool banking_mode = false;        ///< Enable banking-grade detection
    float banking_confidence_threshold = 0.9f; ///< Banking grade confidence threshold
    bool require_frontal_face = true; ///< Require frontal face orientation

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Advanced face detection with pose estimation and quality assessment
 *
 * High-precision face detector optimized for banking-grade facial recognition.
 * Provides robust detection with pose estimation, quality assessment, and
 * multi-modal input support (RGB + depth).
 *
 * Thread-safe implementation with ARM64 NEON optimizations.
 */
class FaceDetector {
public:
    /**
     * @brief Constructor with default configuration
     */
    FaceDetector();

    /**
     * @brief Constructor with custom configuration
     * @param config Detection configuration
     */
    explicit FaceDetector(const FaceDetectionConfig& config);

    /**
     * @brief Destructor
     */
    ~FaceDetector();

    /**
     * @brief Initialize face detector with model files
     * @param model_path Path to face detection model
     * @param pose_model_path Path to pose estimation model (optional)
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const std::string& model_path,
                             const std::string& pose_model_path = "");

    /**
     * @brief Check if detector is initialized
     * @return True if detector is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Detect faces in RGB image
     * @param image Input RGB image (CV_8UC3)
     * @param result Output detection result
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectFaces(const cv::Mat& image, FaceDetectionResult& result);

    /**
     * @brief Detect faces with depth information
     * @param rgb_image Input RGB image (CV_8UC3)
     * @param depth_map Input depth map in millimeters (CV_32F)
     * @param result Output detection result with depth-enhanced quality
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectFacesWithDepth(const cv::Mat& rgb_image,
                                       const cv::Mat& depth_map,
                                       FaceDetectionResult& result);

    /**
     * @brief Detect single face (optimized for authentication scenarios)
     * @param image Input RGB image
     * @param face_box Output single best face detection
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode detectSingleFace(const cv::Mat& image, FaceBoundingBox& face_box);

    /**
     * @brief Asynchronous face detection
     * @param image Input RGB image
     * @param callback Callback function for result
     * @return True if detection task was queued successfully
     */
    bool detectFacesAsync(const cv::Mat& image, FaceDetectionCallback callback);

    /**
     * @brief Batch face detection for multiple images
     * @param images Vector of input images
     * @param results Vector of output results
     * @param progress_callback Optional progress callback (0-100)
     * @return Number of successfully processed images
     */
    size_t detectFacesBatch(const std::vector<cv::Mat>& images,
                           std::vector<FaceDetectionResult>& results,
                           std::function<void(float)> progress_callback = nullptr);

    /**
     * @brief Set detection configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const FaceDetectionConfig& config);

    /**
     * @brief Get current configuration
     * @return Current detection configuration
     */
    FaceDetectionConfig getConfiguration() const;

    /**
     * @brief Set region of interest for detection
     * @param roi Region of interest in image coordinates
     */
    void setROI(const cv::Rect& roi);

    /**
     * @brief Clear region of interest (detect in full image)
     */
    void clearROI();

    /**
     * @brief Enable/disable specific detection features
     * @param enable_pose Enable pose estimation
     * @param enable_quality Enable quality assessment
     * @param enable_depth Enable depth processing
     */
    void setFeatures(bool enable_pose, bool enable_quality, bool enable_depth);

    /**
     * @brief Get detection statistics
     * @param avg_detection_time Average detection time in milliseconds
     * @param total_detections Total number of detections performed
     * @param success_rate Success rate (0-1)
     */
    void getStatistics(double& avg_detection_time,
                      size_t& total_detections,
                      float& success_rate) const;

    /**
     * @brief Reset detection statistics
     */
    void resetStatistics();

    /**
     * @brief Cancel ongoing asynchronous operations
     */
    void cancelAsync();

    /**
     * @brief Get last error message
     * @return Human-readable error message
     */
    std::string getLastError() const;

    /**
     * @brief Validate face detection model
     * @param model_path Path to model file
     * @return True if model is valid and compatible
     */
    static bool validateModel(const std::string& model_path);

    /**
     * @brief Get supported image formats
     * @return Vector of supported OpenCV image types
     */
    static std::vector<int> getSupportedImageTypes();

    /**
     * @brief Get optimal input size for detection model
     * @return Recommended input image size for best performance
     */
    cv::Size getOptimalInputSize() const;

    /**
     * @brief Enable banking-grade detection mode
     * @param enable Enable/disable banking mode
     * @param strict_pose Require strict frontal pose
     * @param min_confidence Minimum confidence for banking grade
     */
    void setBankingMode(bool enable, bool strict_pose = true, float min_confidence = 0.95f);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_detections_{0};
    std::atomic<size_t> successful_detections_{0};
    std::atomic<double> total_detection_time_{0.0};

    /**
     * @brief Internal face detection implementation
     * @param image Input image
     * @param depth_map Optional depth map
     * @param result Output result
     * @return FaceResultCode
     */
    FaceResultCode detectFacesInternal(const cv::Mat& image,
                                      const cv::Mat* depth_map,
                                      FaceDetectionResult& result);

    /**
     * @brief Estimate face pose from landmarks or detection
     * @param face_box Face bounding box
     * @param image Input image
     * @param pose_angles Output pose angles (yaw, pitch, roll)
     */
    void estimateFacePose(const FaceBoundingBox& face_box,
                         const cv::Mat& image,
                         cv::Vec3f& pose_angles);

    /**
     * @brief Assess face quality for banking compliance
     * @param face_box Face bounding box
     * @param image Input image
     * @param depth_map Optional depth map
     * @return Face quality level
     */
    FaceQuality assessFaceQuality(const FaceBoundingBox& face_box,
                                 const cv::Mat& image,
                                 const cv::Mat* depth_map = nullptr);

    /**
     * @brief Filter detections based on quality and pose
     * @param raw_detections Input raw detections
     * @param image Input image
     * @param filtered_detections Output filtered detections
     */
    void filterDetections(const std::vector<cv::Rect>& raw_detections,
                         const cv::Mat& image,
                         std::vector<FaceBoundingBox>& filtered_detections);

    /**
     * @brief Preprocess image for optimal detection
     * @param input Input image
     * @param output Output preprocessed image
     */
    void preprocessImage(const cv::Mat& input, cv::Mat& output);

    /**
     * @brief Non-maximum suppression for face detections
     * @param faces Input face detections
     * @param overlap_threshold Overlap threshold for suppression
     * @return Filtered face detections
     */
    std::vector<FaceBoundingBox> nonMaximumSuppression(
        const std::vector<FaceBoundingBox>& faces,
        float overlap_threshold = 0.3f) const;

    /**
     * @brief Calculate intersection over union for two boxes
     * @param box1 First bounding box
     * @param box2 Second bounding box
     * @return IoU value (0-1)
     */
    static float calculateIoU(const cv::Rect2f& box1, const cv::Rect2f& box2);

    /**
     * @brief Update performance statistics
     * @param detection_time Time taken for detection
     * @param success Whether detection was successful
     */
    void updateStatistics(double detection_time, bool success);

    // Disable copy constructor and assignment
    FaceDetector(const FaceDetector&) = delete;
    FaceDetector& operator=(const FaceDetector&) = delete;

    // Enable move semantics
    FaceDetector(FaceDetector&&) noexcept;
    FaceDetector& operator=(FaceDetector&&) noexcept;
};

/**
 * @brief Factory class for creating face detectors with different models
 */
class FaceDetectorFactory {
public:
    /**
     * @brief Create OpenCV Haar Cascade detector
     * @param cascade_path Path to Haar cascade file
     * @param config Detection configuration
     * @return Unique pointer to face detector
     */
    static std::unique_ptr<FaceDetector> createHaarCascadeDetector(
        const std::string& cascade_path,
        const FaceDetectionConfig& config = FaceDetectionConfig());

    /**
     * @brief Create OpenCV DNN detector
     * @param model_path Path to DNN model
     * @param config_path Path to model configuration
     * @param config Detection configuration
     * @return Unique pointer to face detector
     */
    static std::unique_ptr<FaceDetector> createDNNDetector(
        const std::string& model_path,
        const std::string& config_path,
        const FaceDetectionConfig& config = FaceDetectionConfig());

    /**
     * @brief Create MediaPipe-based detector
     * @param model_path Path to MediaPipe model
     * @param config Detection configuration
     * @return Unique pointer to face detector
     */
    static std::unique_ptr<FaceDetector> createMediaPipeDetector(
        const std::string& model_path,
        const FaceDetectionConfig& config = FaceDetectionConfig());

    /**
     * @brief Create banking-grade detector with optimal settings
     * @param model_path Path to high-precision model
     * @return Unique pointer to banking-grade face detector
     */
    static std::unique_ptr<FaceDetector> createBankingGradeDetector(
        const std::string& model_path);

    /**
     * @brief Get default model paths for different detector types
     * @param detector_type Type of detector ("haar", "dnn", "mediapipe")
     * @return Default model path
     */
    static std::string getDefaultModelPath(const std::string& detector_type);
};

} // namespace face
} // namespace unlook