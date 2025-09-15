#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

namespace unlook {
namespace face {

/**
 * @brief Facial landmark extraction configuration
 */
struct LandmarkExtractionConfig {
    // Model configuration
    LandmarkModel model_type = LandmarkModel::LANDMARKS_68; ///< Landmark model type
    std::string model_path;                                ///< Path to landmark model

    // Detection parameters
    float confidence_threshold = 0.5f;     ///< Minimum landmark confidence
    bool enable_3d_estimation = true;      ///< Enable 3D landmark estimation
    bool enable_pose_estimation = true;    ///< Enable head pose estimation

    // Quality filtering
    float min_landmark_visibility = 0.7f;  ///< Minimum visible landmark ratio
    bool enable_geometric_validation = true; ///< Validate landmark geometry
    float max_landmark_deviation = 2.0f;   ///< Maximum deviation from mean (std devs)

    // 3D estimation parameters
    float depth_weight = 0.3f;             ///< Weight for depth in 3D estimation
    float stereo_confidence_threshold = 0.6f; ///< Minimum stereo confidence
    bool use_stereo_refinement = true;     ///< Refine landmarks using stereo data

    // Performance settings
    bool enable_multithreading = true;     ///< Use multi-threaded processing
    int max_threads = 2;                   ///< Maximum threads for landmark extraction
    bool enable_temporal_smoothing = false; ///< Enable temporal landmark smoothing
    float temporal_alpha = 0.3f;           ///< Temporal smoothing factor

    // Banking-grade requirements
    bool banking_mode = false;             ///< Enable banking-grade extraction
    float banking_confidence_threshold = 0.9f; ///< Banking grade confidence
    bool require_all_landmarks = true;     ///< Require detection of all landmarks

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief High-precision facial landmark extractor with 3D support
 *
 * Advanced facial landmark extraction system designed for banking-grade
 * facial recognition. Supports 68/468 point models with 3D coordinate
 * estimation using stereo vision integration.
 *
 * Features:
 * - Multi-model support (68/468 points)
 * - 3D landmark estimation from stereo data
 * - Geometric validation and quality assessment
 * - Temporal smoothing for video sequences
 * - Banking-grade precision and reliability
 *
 * Thread-safe implementation with ARM64 optimizations.
 */
class LandmarkExtractor {
public:
    /**
     * @brief Constructor with default configuration
     */
    LandmarkExtractor();

    /**
     * @brief Constructor with custom configuration
     * @param config Landmark extraction configuration
     */
    explicit LandmarkExtractor(const LandmarkExtractionConfig& config);

    /**
     * @brief Destructor
     */
    ~LandmarkExtractor();

    /**
     * @brief Initialize landmark extractor with model
     * @param model_path Path to landmark detection model
     * @param model_type Type of landmark model
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(const std::string& model_path,
                             LandmarkModel model_type = LandmarkModel::LANDMARKS_68);

    /**
     * @brief Check if extractor is initialized
     * @return True if extractor is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Extract 2D facial landmarks from face region
     * @param image Input RGB image (CV_8UC3)
     * @param face_box Face bounding box
     * @param landmarks Output facial landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode extractLandmarks2D(const cv::Mat& image,
                                     const FaceBoundingBox& face_box,
                                     FacialLandmarks& landmarks);

    /**
     * @brief Extract 3D facial landmarks using stereo depth data
     * @param rgb_image Input RGB image (CV_8UC3)
     * @param depth_map Input depth map in millimeters (CV_32F)
     * @param face_box Face bounding box
     * @param landmarks Output 3D facial landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode extractLandmarks3D(const cv::Mat& rgb_image,
                                     const cv::Mat& depth_map,
                                     const FaceBoundingBox& face_box,
                                     FacialLandmarks& landmarks);

    /**
     * @brief Extract landmarks from multiple faces
     * @param image Input RGB image
     * @param face_boxes Vector of face detections
     * @param all_landmarks Output vector of facial landmarks
     * @return Number of successfully processed faces
     */
    size_t extractMultipleLandmarks(const cv::Mat& image,
                                   const std::vector<FaceBoundingBox>& face_boxes,
                                   std::vector<FacialLandmarks>& all_landmarks);

    /**
     * @brief Asynchronous landmark extraction
     * @param image Input RGB image
     * @param face_box Face bounding box
     * @param callback Callback function for result
     * @return True if extraction task was queued successfully
     */
    bool extractLandmarksAsync(const cv::Mat& image,
                              const FaceBoundingBox& face_box,
                              LandmarkDetectionCallback callback);

    /**
     * @brief Refine landmarks using stereo depth information
     * @param landmarks Input 2D landmarks
     * @param depth_map Input depth map
     * @param confidence_map Optional depth confidence map
     * @param refined_landmarks Output refined 3D landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode refineLandmarksWithDepth(const FacialLandmarks& landmarks,
                                           const cv::Mat& depth_map,
                                           const cv::Mat& confidence_map,
                                           FacialLandmarks& refined_landmarks);

    /**
     * @brief Estimate head pose from landmarks
     * @param landmarks Input facial landmarks
     * @param camera_matrix Camera intrinsic matrix
     * @param pose_angles Output pose angles (yaw, pitch, roll) in degrees
     * @param translation_vector Output translation vector
     * @param rotation_matrix Output rotation matrix
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode estimateHeadPose(const FacialLandmarks& landmarks,
                                   const cv::Mat& camera_matrix,
                                   cv::Vec3f& pose_angles,
                                   cv::Vec3f& translation_vector,
                                   cv::Mat& rotation_matrix);

    /**
     * @brief Validate landmark quality and geometry
     * @param landmarks Input landmarks to validate
     * @param image Optional source image for context
     * @param quality_score Output quality score (0-1)
     * @param error_flags Output validation error flags
     * @return True if landmarks pass validation
     */
    bool validateLandmarks(const FacialLandmarks& landmarks,
                          const cv::Mat& image,
                          float& quality_score,
                          std::vector<std::string>& error_flags);

    /**
     * @brief Apply temporal smoothing to landmark sequence
     * @param new_landmarks Current frame landmarks
     * @param smoothed_landmarks Output smoothed landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode applySmoothingTemporalFilter(const FacialLandmarks& new_landmarks,
                                               FacialLandmarks& smoothed_landmarks);

    /**
     * @brief Set extraction configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const LandmarkExtractionConfig& config);

    /**
     * @brief Get current configuration
     * @return Current extraction configuration
     */
    LandmarkExtractionConfig getConfiguration() const;

    /**
     * @brief Set camera calibration data for 3D estimation
     * @param camera_matrix Camera intrinsic matrix
     * @param dist_coeffs Distortion coefficients
     */
    void setCameraCalibration(const cv::Mat& camera_matrix,
                             const cv::Mat& dist_coeffs);

    /**
     * @brief Enable/disable specific extraction features
     * @param enable_3d Enable 3D landmark estimation
     * @param enable_pose Enable head pose estimation
     * @param enable_validation Enable geometric validation
     */
    void setFeatures(bool enable_3d, bool enable_pose, bool enable_validation);

    /**
     * @brief Get extraction statistics
     * @param avg_extraction_time Average extraction time in milliseconds
     * @param total_extractions Total number of extractions performed
     * @param success_rate Success rate (0-1)
     * @param avg_landmark_confidence Average landmark confidence
     */
    void getStatistics(double& avg_extraction_time,
                      size_t& total_extractions,
                      float& success_rate,
                      float& avg_landmark_confidence) const;

    /**
     * @brief Reset extraction statistics
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
     * @brief Get supported landmark models
     * @return Vector of supported landmark model types
     */
    static std::vector<LandmarkModel> getSupportedModels();

    /**
     * @brief Get landmark point names for specific model
     * @param model Landmark model type
     * @return Vector of landmark point names
     */
    static std::vector<std::string> getLandmarkNames(LandmarkModel model);

    /**
     * @brief Get semantic landmark groups (eyes, nose, mouth, etc.)
     * @param model Landmark model type
     * @return Map of semantic group names to landmark indices
     */
    static std::map<std::string, std::vector<int>> getLandmarkGroups(LandmarkModel model);

    /**
     * @brief Validate landmark model file
     * @param model_path Path to model file
     * @param model_type Expected model type
     * @return True if model is valid and compatible
     */
    static bool validateModel(const std::string& model_path,
                             LandmarkModel model_type);

    /**
     * @brief Enable banking-grade extraction mode
     * @param enable Enable/disable banking mode
     * @param require_all_landmarks Require detection of all landmarks
     * @param min_confidence Minimum confidence for banking grade
     */
    void setBankingMode(bool enable,
                       bool require_all_landmarks = true,
                       float min_confidence = 0.95f);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_extractions_{0};
    std::atomic<size_t> successful_extractions_{0};
    std::atomic<double> total_extraction_time_{0.0};
    std::atomic<double> total_confidence_{0.0};

    // Temporal smoothing state
    std::mutex temporal_mutex_;
    FacialLandmarks previous_landmarks_;
    bool has_previous_landmarks_{false};

    /**
     * @brief Internal landmark extraction implementation
     * @param image Input image
     * @param depth_map Optional depth map
     * @param face_box Face bounding box
     * @param landmarks Output landmarks
     * @return FaceResultCode
     */
    FaceResultCode extractLandmarksInternal(const cv::Mat& image,
                                           const cv::Mat* depth_map,
                                           const FaceBoundingBox& face_box,
                                           FacialLandmarks& landmarks);

    /**
     * @brief Convert 2D landmarks to 3D using depth information
     * @param landmarks_2d Input 2D landmarks
     * @param depth_map Depth map in millimeters
     * @param camera_matrix Camera intrinsic matrix
     * @param landmarks_3d Output 3D landmarks
     * @return Number of successful 3D conversions
     */
    size_t convert2DTo3D(const std::vector<LandmarkPoint2D>& landmarks_2d,
                        const cv::Mat& depth_map,
                        const cv::Mat& camera_matrix,
                        std::vector<LandmarkPoint3D>& landmarks_3d);

    /**
     * @brief Validate landmark geometry using statistical models
     * @param landmarks Input landmarks
     * @param image Source image
     * @return Quality score (0-1)
     */
    float validateGeometry(const FacialLandmarks& landmarks,
                          const cv::Mat& image);

    /**
     * @brief Apply outlier detection to landmarks
     * @param landmarks Input/output landmarks
     * @return Number of outliers detected and corrected
     */
    int removeOutliers(FacialLandmarks& landmarks);

    /**
     * @brief Compute inter-landmark distances for validation
     * @param landmarks Input landmarks
     * @param distances Output distance vector
     */
    void computeLandmarkDistances(const FacialLandmarks& landmarks,
                                 std::vector<float>& distances);

    /**
     * @brief Update performance statistics
     * @param extraction_time Time taken for extraction
     * @param success Whether extraction was successful
     * @param confidence Average landmark confidence
     */
    void updateStatistics(double extraction_time,
                         bool success,
                         float confidence);

    // Disable copy constructor and assignment
    LandmarkExtractor(const LandmarkExtractor&) = delete;
    LandmarkExtractor& operator=(const LandmarkExtractor&) = delete;

    // Enable move semantics
    LandmarkExtractor(LandmarkExtractor&&) noexcept;
    LandmarkExtractor& operator=(LandmarkExtractor&&) noexcept;
};

/**
 * @brief Utility functions for landmark processing
 */
class LandmarkUtils {
public:
    /**
     * @brief Draw landmarks on image for visualization
     * @param image Input/output image
     * @param landmarks Landmarks to draw
     * @param color Drawing color
     * @param radius Point radius
     * @param draw_connections Draw landmark connections
     */
    static void drawLandmarks(cv::Mat& image,
                             const FacialLandmarks& landmarks,
                             const cv::Scalar& color = cv::Scalar(0, 255, 0),
                             int radius = 2,
                             bool draw_connections = true);

    /**
     * @brief Compute facial measurements from landmarks
     * @param landmarks Input landmarks
     * @param interpupillary_distance Output IPD in pixels/mm
     * @param face_width Output face width
     * @param face_height Output face height
     * @param use_3d Use 3D coordinates if available
     */
    static void computeFacialMeasurements(const FacialLandmarks& landmarks,
                                         float& interpupillary_distance,
                                         float& face_width,
                                         float& face_height,
                                         bool use_3d = true);

    /**
     * @brief Extract facial region using landmarks
     * @param image Input image
     * @param landmarks Facial landmarks
     * @param margin Margin around face region
     * @param normalized_size Output size for normalized face
     * @param normalized_face Output normalized face image
     * @param transform Output transformation matrix
     */
    static bool extractNormalizedFace(const cv::Mat& image,
                                     const FacialLandmarks& landmarks,
                                     float margin,
                                     const cv::Size& normalized_size,
                                     cv::Mat& normalized_face,
                                     cv::Mat& transform);

    /**
     * @brief Align face based on landmarks
     * @param image Input image
     * @param landmarks Input landmarks
     * @param reference_landmarks Reference landmarks for alignment
     * @param aligned_image Output aligned image
     * @param aligned_landmarks Output aligned landmarks
     */
    static bool alignFace(const cv::Mat& image,
                         const FacialLandmarks& landmarks,
                         const FacialLandmarks& reference_landmarks,
                         cv::Mat& aligned_image,
                         FacialLandmarks& aligned_landmarks);
};

} // namespace face
} // namespace unlook