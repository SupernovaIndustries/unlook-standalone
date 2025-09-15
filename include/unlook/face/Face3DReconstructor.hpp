#pragma once

#include "unlook/face/FaceTypes.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <mutex>
#include <atomic>

namespace unlook {
namespace face {

/**
 * @brief 3D face reconstruction configuration
 */
struct Face3DReconstructionConfig {
    // Reconstruction method
    enum Method {
        STEREO_DENSE,           ///< Dense stereo reconstruction
        LANDMARK_BASED,         ///< Landmark-based morphable model
        HYBRID                  ///< Hybrid stereo + morphable model
    } method = HYBRID;

    // Stereo reconstruction parameters
    float min_depth_mm = 300.0f;       ///< Minimum valid depth
    float max_depth_mm = 600.0f;       ///< Maximum valid depth
    float depth_confidence_threshold = 0.7f; ///< Minimum depth confidence
    bool enable_depth_filtering = true;  ///< Apply depth map filtering
    int median_filter_size = 5;         ///< Median filter kernel size

    // Mesh generation parameters
    float voxel_size_mm = 0.5f;         ///< Voxel size for mesh generation
    int max_vertices = 50000;           ///< Maximum mesh vertices
    int max_triangles = 100000;         ///< Maximum mesh triangles
    bool enable_mesh_smoothing = true;  ///< Apply mesh smoothing
    int smoothing_iterations = 3;       ///< Smoothing iterations

    // Morphable model parameters
    std::string morphable_model_path;   ///< Path to 3D morphable model
    float landmark_weight = 0.7f;       ///< Weight for landmark fitting
    float regularization_weight = 0.1f; ///< Regularization weight
    int max_iterations = 100;           ///< Maximum fitting iterations

    // Quality requirements
    float min_mesh_completeness = 0.8f; ///< Minimum mesh completeness
    float min_surface_smoothness = 0.7f; ///< Minimum surface smoothness
    float min_reconstruction_quality = 0.8f; ///< Minimum overall quality

    // Performance settings
    bool enable_gpu_acceleration = false; ///< Use GPU acceleration
    bool enable_multithreading = true;   ///< Use multi-threaded processing
    int max_threads = 4;                 ///< Maximum reconstruction threads

    // Banking-grade requirements
    bool banking_mode = false;           ///< Enable banking-grade reconstruction
    float banking_quality_threshold = 0.95f; ///< Banking quality requirement
    bool require_texture_mapping = true; ///< Require high-quality texture

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief High-precision 3D facial reconstruction system
 *
 * Advanced 3D face reconstruction engine that combines stereo vision,
 * landmark-based morphable models, and advanced mesh processing to create
 * banking-grade 3D facial models with sub-millimeter precision.
 *
 * Key features:
 * - Dense stereo reconstruction with 0.5mm precision
 * - 3D morphable model fitting with landmark constraints
 * - Hybrid reconstruction combining stereo + morphable models
 * - High-quality texture mapping and mesh optimization
 * - Banking-grade quality validation and compliance
 * - ARM64/CM4 optimized for real-time performance
 *
 * Integrates seamlessly with Unlook stereo system and depth processor.
 */
class Face3DReconstructor {
public:
    /**
     * @brief Constructor with default configuration
     */
    Face3DReconstructor();

    /**
     * @brief Constructor with custom configuration
     * @param config Reconstruction configuration
     */
    explicit Face3DReconstructor(const Face3DReconstructionConfig& config);

    /**
     * @brief Destructor
     */
    ~Face3DReconstructor();

    /**
     * @brief Initialize reconstructor with calibration and depth processor
     * @param calibration_manager Stereo calibration manager
     * @param depth_processor Depth processing system
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode initialize(
        std::shared_ptr<calibration::CalibrationManager> calibration_manager,
        std::shared_ptr<stereo::DepthProcessor> depth_processor);

    /**
     * @brief Check if reconstructor is initialized
     * @return True if reconstructor is ready for use
     */
    bool isInitialized() const;

    /**
     * @brief Reconstruct 3D face from stereo images and landmarks
     * @param left_image Left stereo image (CV_8UC3)
     * @param right_image Right stereo image (CV_8UC3)
     * @param landmarks Detected facial landmarks
     * @param face_model Output 3D face model
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode reconstructFace(const cv::Mat& left_image,
                                  const cv::Mat& right_image,
                                  const FacialLandmarks& landmarks,
                                  Face3DModel& face_model);

    /**
     * @brief Reconstruct 3D face from RGB image and depth map
     * @param rgb_image RGB image (CV_8UC3)
     * @param depth_map Depth map in millimeters (CV_32F)
     * @param landmarks Detected facial landmarks
     * @param face_model Output 3D face model
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode reconstructFromDepth(const cv::Mat& rgb_image,
                                       const cv::Mat& depth_map,
                                       const FacialLandmarks& landmarks,
                                       Face3DModel& face_model);

    /**
     * @brief Asynchronous face reconstruction
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param landmarks Facial landmarks
     * @param callback Callback function for result
     * @return True if reconstruction task was queued successfully
     */
    bool reconstructFaceAsync(const cv::Mat& left_image,
                             const cv::Mat& right_image,
                             const FacialLandmarks& landmarks,
                             Face3DReconstructionCallback callback);

    /**
     * @brief Refine 3D model using additional views
     * @param face_model Input/output 3D face model
     * @param additional_images Vector of additional RGB images
     * @param additional_landmarks Vector of corresponding landmarks
     * @param camera_poses Camera poses for additional views
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode refineWithMultipleViews(
        Face3DModel& face_model,
        const std::vector<cv::Mat>& additional_images,
        const std::vector<FacialLandmarks>& additional_landmarks,
        const std::vector<cv::Mat>& camera_poses);

    /**
     * @brief Apply texture mapping to 3D face model
     * @param face_model Input/output 3D face model
     * @param texture_image Source texture image
     * @param landmarks Corresponding landmarks
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode applyTextureMapping(Face3DModel& face_model,
                                      const cv::Mat& texture_image,
                                      const FacialLandmarks& landmarks);

    /**
     * @brief Optimize mesh quality and topology
     * @param face_model Input/output 3D face model
     * @param target_vertices Target number of vertices
     * @param preserve_features Preserve facial features during optimization
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode optimizeMesh(Face3DModel& face_model,
                               int target_vertices = 10000,
                               bool preserve_features = true);

    /**
     * @brief Validate 3D face model quality
     * @param face_model Input 3D face model
     * @param quality_score Output overall quality score (0-1)
     * @param detailed_metrics Output detailed quality metrics
     * @return True if model meets quality requirements
     */
    bool validateModelQuality(const Face3DModel& face_model,
                             float& quality_score,
                             std::map<std::string, float>& detailed_metrics);

    /**
     * @brief Export 3D face model to file
     * @param face_model Input 3D face model
     * @param filename Output filename
     * @param format Export format ("ply", "obj", "stl", "x3d")
     * @param include_texture Include texture data in export
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode exportModel(const Face3DModel& face_model,
                              const std::string& filename,
                              const std::string& format = "ply",
                              bool include_texture = true);

    /**
     * @brief Import 3D face model from file
     * @param filename Input filename
     * @param face_model Output 3D face model
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode importModel(const std::string& filename,
                              Face3DModel& face_model);

    /**
     * @brief Set reconstruction configuration
     * @param config New configuration
     * @return FaceResultCode indicating success or failure
     */
    FaceResultCode setConfiguration(const Face3DReconstructionConfig& config);

    /**
     * @brief Get current configuration
     * @return Current reconstruction configuration
     */
    Face3DReconstructionConfig getConfiguration() const;

    /**
     * @brief Set progress callback for long operations
     * @param callback Progress callback function (0-100)
     */
    void setProgressCallback(std::function<void(float)> callback);

    /**
     * @brief Enable/disable specific reconstruction features
     * @param enable_stereo Enable stereo reconstruction
     * @param enable_morphable Enable morphable model fitting
     * @param enable_texture Enable texture mapping
     */
    void setFeatures(bool enable_stereo,
                    bool enable_morphable,
                    bool enable_texture);

    /**
     * @brief Get reconstruction statistics
     * @param avg_reconstruction_time Average reconstruction time in ms
     * @param total_reconstructions Total number of reconstructions
     * @param success_rate Success rate (0-1)
     * @param avg_quality_score Average quality score
     */
    void getStatistics(double& avg_reconstruction_time,
                      size_t& total_reconstructions,
                      float& success_rate,
                      float& avg_quality_score) const;

    /**
     * @brief Reset reconstruction statistics
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
     * @brief Get supported export formats
     * @return Vector of supported 3D model formats
     */
    static std::vector<std::string> getSupportedFormats();

    /**
     * @brief Validate morphable model file
     * @param model_path Path to morphable model
     * @return True if model is valid and compatible
     */
    static bool validateMorphableModel(const std::string& model_path);

    /**
     * @brief Enable banking-grade reconstruction mode
     * @param enable Enable/disable banking mode
     * @param require_texture Require high-quality texture mapping
     * @param min_quality Minimum quality for banking grade
     */
    void setBankingMode(bool enable,
                       bool require_texture = true,
                       float min_quality = 0.95f);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Integration with existing Unlook systems
    std::shared_ptr<calibration::CalibrationManager> calibration_manager_;
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // Performance monitoring
    mutable std::mutex stats_mutex_;
    std::atomic<size_t> total_reconstructions_{0};
    std::atomic<size_t> successful_reconstructions_{0};
    std::atomic<double> total_reconstruction_time_{0.0};
    std::atomic<double> total_quality_score_{0.0};

    // Progress callback
    std::mutex callback_mutex_;
    std::function<void(float)> progress_callback_;

    /**
     * @brief Internal reconstruction implementation
     * @param rgb_image RGB image
     * @param depth_map Depth map (can be nullptr)
     * @param landmarks Facial landmarks
     * @param face_model Output model
     * @return FaceResultCode
     */
    FaceResultCode reconstructInternal(const cv::Mat& rgb_image,
                                      const cv::Mat* depth_map,
                                      const FacialLandmarks& landmarks,
                                      Face3DModel& face_model);

    /**
     * @brief Generate dense point cloud from stereo
     * @param left_image Left stereo image
     * @param right_image Right stereo image
     * @param face_region Face region of interest
     * @param point_cloud Output point cloud
     * @return FaceResultCode
     */
    FaceResultCode generateDensePointCloud(const cv::Mat& left_image,
                                          const cv::Mat& right_image,
                                          const cv::Rect& face_region,
                                          stereo::PointCloud& point_cloud);

    /**
     * @brief Fit morphable model to landmarks and depth
     * @param landmarks Input landmarks
     * @param depth_points Optional depth points
     * @param face_mesh Output face mesh
     * @return FaceResultCode
     */
    FaceResultCode fitMorphableModel(const FacialLandmarks& landmarks,
                                    const std::vector<cv::Point3f>* depth_points,
                                    Face3DMesh& face_mesh);

    /**
     * @brief Generate mesh from point cloud
     * @param point_cloud Input point cloud
     * @param face_mesh Output mesh
     * @return FaceResultCode
     */
    FaceResultCode generateMeshFromPointCloud(const stereo::PointCloud& point_cloud,
                                             Face3DMesh& face_mesh);

    /**
     * @brief Apply Poisson surface reconstruction
     * @param point_cloud Input point cloud with normals
     * @param face_mesh Output mesh
     * @return FaceResultCode
     */
    FaceResultCode poissonReconstruction(const stereo::PointCloud& point_cloud,
                                        Face3DMesh& face_mesh);

    /**
     * @brief Compute mesh normals and curvature
     * @param face_mesh Input/output mesh
     */
    void computeMeshNormals(Face3DMesh& face_mesh);

    /**
     * @brief Apply Laplacian smoothing to mesh
     * @param face_mesh Input/output mesh
     * @param iterations Number of smoothing iterations
     */
    void applySmoothingLaplacian(Face3DMesh& face_mesh, int iterations);

    /**
     * @brief Compute geometric measurements from 3D model
     * @param face_model Input/output face model
     */
    void computeGeometricMeasurements(Face3DModel& face_model);

    /**
     * @brief Assess reconstruction quality
     * @param face_model Input face model
     * @param quality_score Output quality score
     * @return Detailed quality assessment
     */
    std::map<std::string, float> assessQuality(const Face3DModel& face_model,
                                              float& quality_score);

    /**
     * @brief Update progress during reconstruction
     * @param progress Progress value (0-100)
     */
    void updateProgress(float progress);

    /**
     * @brief Update performance statistics
     * @param reconstruction_time Time taken for reconstruction
     * @param success Whether reconstruction was successful
     * @param quality_score Reconstruction quality score
     */
    void updateStatistics(double reconstruction_time,
                         bool success,
                         float quality_score);

    // Disable copy constructor and assignment
    Face3DReconstructor(const Face3DReconstructor&) = delete;
    Face3DReconstructor& operator=(const Face3DReconstructor&) = delete;

    // Enable move semantics
    Face3DReconstructor(Face3DReconstructor&&) noexcept;
    Face3DReconstructor& operator=(Face3DReconstructor&&) noexcept;
};

/**
 * @brief Utility functions for 3D face processing
 */
class Face3DUtils {
public:
    /**
     * @brief Compute face centroid from 3D mesh
     * @param face_mesh Input face mesh
     * @return Face centroid in 3D space
     */
    static cv::Point3f computeCentroid(const Face3DMesh& face_mesh);

    /**
     * @brief Compute face bounding box in 3D
     * @param face_mesh Input face mesh
     * @param min_point Output minimum point
     * @param max_point Output maximum point
     */
    static void computeBoundingBox(const Face3DMesh& face_mesh,
                                  cv::Point3f& min_point,
                                  cv::Point3f& max_point);

    /**
     * @brief Align 3D face model to canonical pose
     * @param face_model Input/output face model
     * @param reference_landmarks Reference canonical landmarks
     * @param transformation_matrix Output transformation applied
     */
    static bool alignToCanonicalPose(Face3DModel& face_model,
                                    const FacialLandmarks& reference_landmarks,
                                    cv::Mat& transformation_matrix);

    /**
     * @brief Compute mesh surface area
     * @param face_mesh Input face mesh
     * @return Total surface area in mm²
     */
    static float computeSurfaceArea(const Face3DMesh& face_mesh);

    /**
     * @brief Compute mesh volume
     * @param face_mesh Input face mesh
     * @return Mesh volume in mm³
     */
    static float computeVolume(const Face3DMesh& face_mesh);

    /**
     * @brief Extract facial profile curves
     * @param face_mesh Input face mesh
     * @param profile_curves Output profile curves
     * @param num_profiles Number of profile slices
     */
    static void extractProfileCurves(const Face3DMesh& face_mesh,
                                    std::vector<std::vector<cv::Point3f>>& profile_curves,
                                    int num_profiles = 9);

    /**
     * @brief Render 3D face model from specific viewpoint
     * @param face_model Input 3D face model
     * @param camera_matrix Camera intrinsic matrix
     * @param pose_matrix Camera pose matrix
     * @param image_size Output image size
     * @param rendered_image Output rendered image
     * @param rendered_depth Output depth buffer
     */
    static bool renderFaceModel(const Face3DModel& face_model,
                               const cv::Mat& camera_matrix,
                               const cv::Mat& pose_matrix,
                               const cv::Size& image_size,
                               cv::Mat& rendered_image,
                               cv::Mat& rendered_depth);
};

} // namespace face
} // namespace unlook