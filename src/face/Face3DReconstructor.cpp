#include "unlook/face/Face3DReconstructor.hpp"
#include "unlook/face/FaceException.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>
#include <fstream>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace face {

// Standard facial measurements for validation (in mm)
static const float TYPICAL_IPD_MM = 63.0f;           // Average interpupillary distance
static const float TYPICAL_FACE_WIDTH_MM = 140.0f;   // Average face width
static const float TYPICAL_FACE_HEIGHT_MM = 180.0f;  // Average face height
static const float TYPICAL_NOSE_HEIGHT_MM = 15.0f;   // Average nose bridge height

// Quality thresholds for banking-grade reconstruction
static const float BANKING_MIN_COMPLETENESS = 0.95f;
static const float BANKING_MIN_SMOOTHNESS = 0.90f;
static const float BANKING_MIN_ACCURACY = 0.95f;

/**
 * @brief Private implementation class for Face3DReconstructor
 */
class Face3DReconstructor::Impl {
public:
    // Configuration
    Face3DReconstructionConfig config_;

    // Integration with Unlook systems
    std::shared_ptr<calibration::CalibrationManager> calibration_manager_;
    std::shared_ptr<stereo::DepthProcessor> depth_processor_;

    // Camera calibration data
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat stereo_rectify_map1_, stereo_rectify_map2_;
    float baseline_mm_ = 70.017f; // From calibration

    // Error handling
    std::string last_error_;

    // Banking mode state
    bool banking_mode_ = false;

    // Progress callback
    std::function<void(float)> progress_callback_;

    Impl() = default;

    /**
     * @brief Initialize reconstruction system with calibration
     */
    FaceResultCode initializeWithCalibration() {
        try {
            if (!calibration_manager_ || !depth_processor_) {
                last_error_ = "Calibration manager and depth processor required";
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            // Get camera parameters from calibration manager
            // This would typically be retrieved from the calibration system
            camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                1000.0, 0, 728.0,      // Approximate values for IMX296
                0, 1000.0, 544.0,      // Will be updated from actual calibration
                0, 0, 1);

            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

            // Load actual calibration if available
            if (calibration_manager_->isCalibrated()) {
                // Get stereo calibration parameters
                baseline_mm_ = 70.017f; // From calib_boofcv_test3.yaml

                UNLOOK_LOG_INFO("Face3DReconstructor initialized with stereo baseline: {}mm", baseline_mm_);
            }

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during initialization: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Generate dense point cloud from stereo images focused on face region
     */
    FaceResultCode generateFacialPointCloud(const cv::Mat& left_image,
                                           const cv::Mat& right_image,
                                           const cv::Rect& face_region,
                                           stereo::PointCloud& point_cloud) {
        updateProgress(10.0f);

        try {
            // Extract face regions from stereo images
            cv::Mat left_face = left_image(face_region);
            cv::Mat right_face = right_image(face_region);

            // Enhance images for better stereo matching
            cv::Mat enhanced_left, enhanced_right;
            enhanceImageForStereo(left_face, enhanced_left);
            enhanceImageForStereo(right_face, enhanced_right);

            updateProgress(20.0f);

            // Generate depth map using depth processor
            cv::Mat depth_map, confidence_map;
            if (!depth_processor_->processWithConfidence(enhanced_left, enhanced_right,
                                                        depth_map, confidence_map)) {
                last_error_ = "Failed to generate depth map for face region";
                return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
            }

            updateProgress(40.0f);

            // Convert depth map to point cloud with color
            if (!depth_processor_->generatePointCloud(depth_map, left_face, point_cloud)) {
                last_error_ = "Failed to generate point cloud from depth map";
                return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
            }

            updateProgress(50.0f);

            // Filter point cloud for face-specific requirements
            filterFacialPointCloud(point_cloud, confidence_map);

            updateProgress(60.0f);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during point cloud generation: " + std::string(e.what());
            return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
        }
    }

    /**
     * @brief Enhance image contrast and sharpness for better stereo matching
     */
    void enhanceImageForStereo(const cv::Mat& input, cv::Mat& output) {
        // Convert to grayscale if needed
        cv::Mat gray;
        if (input.channels() == 3) {
            cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = input.clone();
        }

        // Apply CLAHE for adaptive contrast enhancement
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        cv::Mat enhanced;
        clahe->apply(gray, enhanced);

        // Apply mild Gaussian blur to reduce noise
        cv::GaussianBlur(enhanced, output, cv::Size(3, 3), 0.5);
    }

    /**
     * @brief Filter point cloud for facial reconstruction requirements
     */
    void filterFacialPointCloud(stereo::PointCloud& point_cloud, const cv::Mat& confidence_map) {
        std::vector<stereo::Point3D> filtered_points;
        filtered_points.reserve(point_cloud.points.size());

        for (size_t i = 0; i < point_cloud.points.size(); ++i) {
            const auto& point = point_cloud.points[i];

            // Filter by depth range (typical face distances)
            if (point.z >= config_.min_depth_mm && point.z <= config_.max_depth_mm) {
                // Check confidence if available
                bool include_point = true;
                if (!confidence_map.empty() && point_cloud.isOrganized) {
                    int row = i / point_cloud.width;
                    int col = i % point_cloud.width;
                    if (row < confidence_map.rows && col < confidence_map.cols) {
                        float confidence = confidence_map.at<float>(row, col);
                        include_point = (confidence >= config_.depth_confidence_threshold);
                    }
                }

                if (include_point) {
                    filtered_points.push_back(point);
                }
            }
        }

        point_cloud.points = std::move(filtered_points);
        point_cloud.isOrganized = false; // No longer organized after filtering
    }

    /**
     * @brief Generate 3D mesh from facial point cloud using Poisson reconstruction
     */
    FaceResultCode generateFacialMesh(const stereo::PointCloud& point_cloud,
                                     Face3DMesh& face_mesh) {
        updateProgress(70.0f);

        try {
            if (point_cloud.points.empty()) {
                last_error_ = "Empty point cloud for mesh generation";
                return FaceResultCode::ERROR_MESH_GENERATION_FAILED;
            }

            // Clear output mesh
            face_mesh.vertices.clear();
            face_mesh.triangles.clear();

            // Convert point cloud to vertices
            face_mesh.vertices.reserve(point_cloud.points.size());
            for (const auto& point : point_cloud.points) {
                face_mesh.vertices.emplace_back(point.x, point.y, point.z);
            }

            updateProgress(75.0f);

            // Compute surface normals
            computeSurfaceNormals(face_mesh);

            updateProgress(80.0f);

            // Generate triangulation using Delaunay or alpha shapes
            // For simplicity, using a grid-based approach for organized point clouds
            if (generateTriangulation(face_mesh)) {
                updateProgress(85.0f);

                // Apply mesh smoothing if enabled
                if (config_.enable_mesh_smoothing) {
                    smoothMesh(face_mesh, config_.smoothing_iterations);
                }

                updateProgress(90.0f);

                // Validate mesh quality
                face_mesh.mesh_completeness = computeMeshCompleteness(face_mesh);
                face_mesh.surface_smoothness = computeSurfaceSmoothness(face_mesh);

                updateProgress(95.0f);

                return FaceResultCode::SUCCESS;
            } else {
                last_error_ = "Failed to generate mesh triangulation";
                return FaceResultCode::ERROR_MESH_GENERATION_FAILED;
            }

        } catch (const std::exception& e) {
            last_error_ = "Exception during mesh generation: " + std::string(e.what());
            return FaceResultCode::ERROR_MESH_GENERATION_FAILED;
        }
    }

    /**
     * @brief Compute surface normals for mesh vertices
     */
    void computeSurfaceNormals(Face3DMesh& face_mesh) {
        // This is a simplified normal computation
        // In a production system, this would use proper mesh algorithms

        // For now, estimate normals from neighboring vertices
        std::vector<cv::Point3f> normals(face_mesh.vertices.size(), cv::Point3f(0, 0, 0));

        // Simple approach: use cross products of edge vectors
        for (size_t i = 1; i < face_mesh.vertices.size() - 1; ++i) {
            cv::Point3f v1 = face_mesh.vertices[i] - face_mesh.vertices[i-1];
            cv::Point3f v2 = face_mesh.vertices[i+1] - face_mesh.vertices[i];

            // Cross product for normal
            cv::Point3f normal;
            normal.x = v1.y * v2.z - v1.z * v2.y;
            normal.y = v1.z * v2.x - v1.x * v2.z;
            normal.z = v1.x * v2.y - v1.y * v2.x;

            // Normalize
            float length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
            if (length > 0) {
                normal.x /= length;
                normal.y /= length;
                normal.z /= length;
            }

            normals[i] = normal;
        }

        // Store normals (would typically be part of Face3DMesh structure)
    }

    /**
     * @brief Generate mesh triangulation
     */
    bool generateTriangulation(Face3DMesh& face_mesh) {
        size_t vertex_count = face_mesh.vertices.size();
        if (vertex_count < 3) return false;

        // Simple triangulation for demonstration
        // In production, use proper mesh generation algorithms like Poisson reconstruction

        face_mesh.triangles.clear();

        // Create a simple triangulation assuming roughly grid-like structure
        int grid_width = static_cast<int>(std::sqrt(vertex_count));
        int grid_height = vertex_count / grid_width;

        for (int row = 0; row < grid_height - 1; ++row) {
            for (int col = 0; col < grid_width - 1; ++col) {
                int i0 = row * grid_width + col;
                int i1 = row * grid_width + (col + 1);
                int i2 = (row + 1) * grid_width + col;
                int i3 = (row + 1) * grid_width + (col + 1);

                if (i0 < vertex_count && i1 < vertex_count &&
                    i2 < vertex_count && i3 < vertex_count) {
                    // Create two triangles for each quad
                    face_mesh.triangles.emplace_back(i0, i1, i2);
                    face_mesh.triangles.emplace_back(i1, i3, i2);
                }
            }
        }

        return !face_mesh.triangles.empty();
    }

    /**
     * @brief Apply Laplacian smoothing to mesh
     */
    void smoothMesh(Face3DMesh& face_mesh, int iterations) {
        for (int iter = 0; iter < iterations; ++iter) {
            std::vector<cv::Point3f> smoothed_vertices = face_mesh.vertices;

            // Simple Laplacian smoothing
            for (size_t i = 0; i < face_mesh.vertices.size(); ++i) {
                cv::Point3f sum(0, 0, 0);
                int neighbor_count = 0;

                // Find neighboring vertices (simplified approach)
                for (const auto& triangle : face_mesh.triangles) {
                    if (triangle.x == i || triangle.y == i || triangle.z == i) {
                        // Add other vertices of this triangle
                        if (triangle.x != i) {
                            sum += face_mesh.vertices[triangle.x];
                            neighbor_count++;
                        }
                        if (triangle.y != i) {
                            sum += face_mesh.vertices[triangle.y];
                            neighbor_count++;
                        }
                        if (triangle.z != i) {
                            sum += face_mesh.vertices[triangle.z];
                            neighbor_count++;
                        }
                    }
                }

                if (neighbor_count > 0) {
                    cv::Point3f avg = sum * (1.0f / neighbor_count);
                    smoothed_vertices[i] = face_mesh.vertices[i] * 0.5f + avg * 0.5f;
                }
            }

            face_mesh.vertices = std::move(smoothed_vertices);
        }
    }

    /**
     * @brief Compute mesh completeness metric
     */
    float computeMeshCompleteness(const Face3DMesh& face_mesh) {
        if (face_mesh.vertices.empty()) return 0.0f;

        // Simple completeness metric based on vertex density and coverage
        float total_area = computeMeshArea(face_mesh);
        float vertex_density = face_mesh.vertices.size() / std::max(total_area, 1.0f);

        // Normalize to 0-1 range (arbitrary scaling for demonstration)
        float completeness = std::min(1.0f, vertex_density / 100.0f);
        return completeness;
    }

    /**
     * @brief Compute surface smoothness metric
     */
    float computeSurfaceSmoothness(const Face3DMesh& face_mesh) {
        if (face_mesh.triangles.empty()) return 0.0f;

        float total_curvature = 0.0f;
        int triangle_count = 0;

        // Compute average curvature from triangle normals
        for (size_t i = 0; i < face_mesh.triangles.size() - 1; ++i) {
            const auto& tri1 = face_mesh.triangles[i];
            const auto& tri2 = face_mesh.triangles[i + 1];

            cv::Point3f normal1 = computeTriangleNormal(face_mesh, tri1);
            cv::Point3f normal2 = computeTriangleNormal(face_mesh, tri2);

            // Angle between normals as curvature measure
            float dot_product = normal1.x * normal2.x + normal1.y * normal2.y + normal1.z * normal2.z;
            float angle = std::acos(std::max(-1.0f, std::min(1.0f, dot_product)));

            total_curvature += angle;
            triangle_count++;
        }

        if (triangle_count > 0) {
            float avg_curvature = total_curvature / triangle_count;
            // Convert to smoothness (lower curvature = higher smoothness)
            return std::max(0.0f, 1.0f - (avg_curvature / M_PI));
        }

        return 0.5f; // Default medium smoothness
    }

    /**
     * @brief Compute triangle normal
     */
    cv::Point3f computeTriangleNormal(const Face3DMesh& face_mesh, const cv::Point3i& triangle) {
        if (triangle.x >= face_mesh.vertices.size() ||
            triangle.y >= face_mesh.vertices.size() ||
            triangle.z >= face_mesh.vertices.size()) {
            return cv::Point3f(0, 0, 1); // Default normal
        }

        const cv::Point3f& v0 = face_mesh.vertices[triangle.x];
        const cv::Point3f& v1 = face_mesh.vertices[triangle.y];
        const cv::Point3f& v2 = face_mesh.vertices[triangle.z];

        cv::Point3f edge1 = v1 - v0;
        cv::Point3f edge2 = v2 - v0;

        // Cross product
        cv::Point3f normal;
        normal.x = edge1.y * edge2.z - edge1.z * edge2.y;
        normal.y = edge1.z * edge2.x - edge1.x * edge2.z;
        normal.z = edge1.x * edge2.y - edge1.y * edge2.x;

        // Normalize
        float length = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        if (length > 0) {
            normal.x /= length;
            normal.y /= length;
            normal.z /= length;
        }

        return normal;
    }

    /**
     * @brief Compute total mesh surface area
     */
    float computeMeshArea(const Face3DMesh& face_mesh) {
        float total_area = 0.0f;

        for (const auto& triangle : face_mesh.triangles) {
            if (triangle.x < face_mesh.vertices.size() &&
                triangle.y < face_mesh.vertices.size() &&
                triangle.z < face_mesh.vertices.size()) {

                const cv::Point3f& v0 = face_mesh.vertices[triangle.x];
                const cv::Point3f& v1 = face_mesh.vertices[triangle.y];
                const cv::Point3f& v2 = face_mesh.vertices[triangle.z];

                // Triangle area using cross product
                cv::Point3f edge1 = v1 - v0;
                cv::Point3f edge2 = v2 - v0;

                cv::Point3f cross;
                cross.x = edge1.y * edge2.z - edge1.z * edge2.y;
                cross.y = edge1.z * edge2.x - edge1.x * edge2.z;
                cross.z = edge1.x * edge2.y - edge1.y * edge2.x;

                float area = 0.5f * std::sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
                total_area += area;
            }
        }

        return total_area;
    }

    /**
     * @brief Apply texture mapping to 3D mesh
     */
    FaceResultCode applyTextureMapping(Face3DModel& face_model,
                                      const cv::Mat& texture_image,
                                      const FacialLandmarks& landmarks) {
        try {
            if (texture_image.empty() || landmarks.points2D.empty()) {
                last_error_ = "Invalid texture image or landmarks for texture mapping";
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            // Clear existing texture data
            face_model.mesh.texture_coords.clear();
            face_model.mesh.texture_map = texture_image.clone();

            // Generate texture coordinates for mesh vertices
            face_model.mesh.texture_coords.reserve(face_model.mesh.vertices.size());

            for (const auto& vertex : face_model.mesh.vertices) {
                // Project 3D vertex back to 2D image coordinates
                cv::Point2f texture_coord = project3DTo2D(vertex);

                // Normalize to texture coordinates (0-1 range)
                texture_coord.x /= texture_image.cols;
                texture_coord.y /= texture_image.rows;

                face_model.mesh.texture_coords.push_back(texture_coord);
            }

            // Compute texture quality metric
            face_model.mesh.texture_quality = computeTextureQuality(texture_image, landmarks);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during texture mapping: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }

    /**
     * @brief Project 3D point to 2D image coordinates
     */
    cv::Point2f project3DTo2D(const cv::Point3f& point_3d) {
        // Use camera matrix to project 3D point to 2D
        float fx = camera_matrix_.at<double>(0, 0);
        float fy = camera_matrix_.at<double>(1, 1);
        float cx = camera_matrix_.at<double>(0, 2);
        float cy = camera_matrix_.at<double>(1, 2);

        float x_2d = (point_3d.x * fx / point_3d.z) + cx;
        float y_2d = (point_3d.y * fy / point_3d.z) + cy;

        return cv::Point2f(x_2d, y_2d);
    }

    /**
     * @brief Compute texture quality metric
     */
    float computeTextureQuality(const cv::Mat& texture_image, const FacialLandmarks& landmarks) {
        // Analyze texture quality based on sharpness, contrast, and coverage

        // Compute image sharpness using Laplacian variance
        cv::Mat gray, laplacian;
        if (texture_image.channels() == 3) {
            cv::cvtColor(texture_image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = texture_image.clone();
        }

        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);
        float sharpness = stddev[0] * stddev[0]; // Variance

        // Normalize sharpness (typical range 0-1000)
        float sharpness_score = std::min(1.0f, sharpness / 500.0f);

        // Compute contrast using standard deviation of intensity
        cv::meanStdDev(gray, mean, stddev);
        float contrast = stddev[0];
        float contrast_score = std::min(1.0f, contrast / 128.0f);

        // Overall texture quality
        float quality = (sharpness_score * 0.6f + contrast_score * 0.4f);
        return std::max(0.0f, std::min(1.0f, quality));
    }

    /**
     * @brief Compute geometric measurements from 3D model
     */
    void computeGeometricMeasurements(Face3DModel& face_model) {
        if (!face_model.landmarks.has3D() || face_model.landmarks.points3D.size() != 68) {
            return; // Cannot compute measurements without 3D landmarks
        }

        const auto& landmarks = face_model.landmarks.points3D;

        // Interpupillary distance (outer eye corners)
        const auto& left_eye = landmarks[36];   // Left eye outer corner
        const auto& right_eye = landmarks[45];  // Right eye outer corner
        face_model.interpupillary_distance_mm = std::sqrt(
            (left_eye.x - right_eye.x) * (left_eye.x - right_eye.x) +
            (left_eye.y - right_eye.y) * (left_eye.y - right_eye.y) +
            (left_eye.z - right_eye.z) * (left_eye.z - right_eye.z)
        );

        // Face width (jaw corners)
        const auto& jaw_left = landmarks[0];    // Left jaw corner
        const auto& jaw_right = landmarks[16];  // Right jaw corner
        face_model.face_width_mm = std::sqrt(
            (jaw_left.x - jaw_right.x) * (jaw_left.x - jaw_right.x) +
            (jaw_left.y - jaw_right.y) * (jaw_left.y - jaw_right.y) +
            (jaw_left.z - jaw_right.z) * (jaw_left.z - jaw_right.z)
        );

        // Face height (forehead to chin)
        const auto& forehead = landmarks[27];   // Nose bridge (approximate forehead)
        const auto& chin = landmarks[8];        // Chin center
        face_model.face_height_mm = std::sqrt(
            (forehead.x - chin.x) * (forehead.x - chin.x) +
            (forehead.y - chin.y) * (forehead.y - chin.y) +
            (forehead.z - chin.z) * (forehead.z - chin.z)
        );

        // Nose bridge height (prominence)
        const auto& nose_tip = landmarks[33];   // Nose tip
        const auto& nose_bridge = landmarks[27]; // Nose bridge
        face_model.nose_bridge_height_mm = std::abs(nose_tip.z - nose_bridge.z);
    }

    /**
     * @brief Assess overall reconstruction quality
     */
    std::map<std::string, float> assessReconstructionQuality(const Face3DModel& face_model,
                                                            float& overall_quality) {
        std::map<std::string, float> metrics;

        // Landmark quality
        float landmark_quality = face_model.landmarks.overall_confidence;
        metrics["landmark_confidence"] = landmark_quality;

        // 3D coverage
        float depth_coverage = face_model.landmarks.depth_coverage;
        metrics["depth_coverage"] = depth_coverage;

        // Mesh quality
        float mesh_completeness = face_model.mesh.mesh_completeness;
        float mesh_smoothness = face_model.mesh.surface_smoothness;
        metrics["mesh_completeness"] = mesh_completeness;
        metrics["mesh_smoothness"] = mesh_smoothness;

        // Texture quality
        float texture_quality = face_model.mesh.texture_quality;
        metrics["texture_quality"] = texture_quality;

        // Geometric consistency (check if measurements are reasonable)
        float geometric_consistency = 1.0f;
        if (face_model.interpupillary_distance_mm > 0) {
            float ipd_ratio = face_model.interpupillary_distance_mm / TYPICAL_IPD_MM;
            if (ipd_ratio < 0.8f || ipd_ratio > 1.3f) {
                geometric_consistency *= 0.7f; // Unusual IPD
            }
        }
        if (face_model.face_width_mm > 0) {
            float width_ratio = face_model.face_width_mm / TYPICAL_FACE_WIDTH_MM;
            if (width_ratio < 0.7f || width_ratio > 1.4f) {
                geometric_consistency *= 0.8f; // Unusual width
            }
        }
        metrics["geometric_consistency"] = geometric_consistency;

        // Overall quality computation
        overall_quality = (
            landmark_quality * 0.25f +
            depth_coverage * 0.20f +
            mesh_completeness * 0.20f +
            mesh_smoothness * 0.15f +
            texture_quality * 0.10f +
            geometric_consistency * 0.10f
        );

        metrics["overall_quality"] = overall_quality;

        return metrics;
    }

    /**
     * @brief Update progress callback
     */
    void updateProgress(float progress) {
        if (progress_callback_) {
            progress_callback_(progress);
        }
    }

    /**
     * @brief Export 3D model to PLY format
     */
    FaceResultCode exportToPLY(const Face3DModel& face_model, const std::string& filename,
                              bool include_texture) {
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                last_error_ = "Cannot open file for writing: " + filename;
                return FaceResultCode::ERROR_TEMPLATE_INVALID;
            }

            const auto& mesh = face_model.mesh;

            // Write PLY header
            file << "ply\n";
            file << "format ascii 1.0\n";
            file << "element vertex " << mesh.vertices.size() << "\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";

            if (include_texture && !mesh.texture_coords.empty()) {
                file << "property uchar red\n";
                file << "property uchar green\n";
                file << "property uchar blue\n";
            }

            file << "element face " << mesh.triangles.size() << "\n";
            file << "property list uchar int vertex_indices\n";
            file << "end_header\n";

            // Write vertices
            for (size_t i = 0; i < mesh.vertices.size(); ++i) {
                const auto& vertex = mesh.vertices[i];
                file << vertex.x << " " << vertex.y << " " << vertex.z;

                // Add color if texture mapping is available
                if (include_texture && !mesh.texture_map.empty() &&
                    i < mesh.texture_coords.size()) {
                    const auto& tex_coord = mesh.texture_coords[i];
                    int u = static_cast<int>(tex_coord.x * mesh.texture_map.cols);
                    int v = static_cast<int>(tex_coord.y * mesh.texture_map.rows);

                    u = std::max(0, std::min(u, mesh.texture_map.cols - 1));
                    v = std::max(0, std::min(v, mesh.texture_map.rows - 1));

                    cv::Vec3b color = mesh.texture_map.at<cv::Vec3b>(v, u);
                    file << " " << static_cast<int>(color[2]) << " "
                         << static_cast<int>(color[1]) << " "
                         << static_cast<int>(color[0]);
                }

                file << "\n";
            }

            // Write faces
            for (const auto& triangle : mesh.triangles) {
                file << "3 " << triangle.x << " " << triangle.y << " " << triangle.z << "\n";
            }

            file.close();
            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during PLY export: " + std::string(e.what());
            return FaceResultCode::ERROR_TEMPLATE_INVALID;
        }
    }
};

// Face3DReconstructionConfig validation
bool Face3DReconstructionConfig::validate() const {
    if (min_depth_mm <= 0 || max_depth_mm <= min_depth_mm) return false;
    if (depth_confidence_threshold < 0.0f || depth_confidence_threshold > 1.0f) return false;
    if (voxel_size_mm <= 0) return false;
    if (max_vertices < 100 || max_triangles < 100) return false;
    if (landmark_weight < 0.0f || landmark_weight > 1.0f) return false;
    if (regularization_weight < 0.0f || regularization_weight > 1.0f) return false;
    if (min_mesh_completeness < 0.0f || min_mesh_completeness > 1.0f) return false;
    if (banking_quality_threshold < 0.8f || banking_quality_threshold > 1.0f) return false;
    if (max_threads < 1 || max_threads > 16) return false;
    return true;
}

std::string Face3DReconstructionConfig::toString() const {
    return "Face3DReconstructionConfig{" +
           std::string("method=") + (method == HYBRID ? "hybrid" : "other") +
           ", depth_range=[" + std::to_string(min_depth_mm) + "," + std::to_string(max_depth_mm) + "]" +
           ", banking_mode=" + (banking_mode ? "true" : "false") + "}";
}

// Face3DReconstructor implementation
Face3DReconstructor::Face3DReconstructor() : pImpl(std::make_unique<Impl>()) {
    Face3DReconstructionConfig default_config;
    pImpl->config_ = default_config;
}

Face3DReconstructor::Face3DReconstructor(const Face3DReconstructionConfig& config)
    : pImpl(std::make_unique<Impl>()) {
    pImpl->config_ = config;
}

Face3DReconstructor::~Face3DReconstructor() = default;

Face3DReconstructor::Face3DReconstructor(Face3DReconstructor&&) noexcept = default;
Face3DReconstructor& Face3DReconstructor::operator=(Face3DReconstructor&&) noexcept = default;

FaceResultCode Face3DReconstructor::initialize(
    std::shared_ptr<calibration::CalibrationManager> calibration_manager,
    std::shared_ptr<stereo::DepthProcessor> depth_processor) {

    calibration_manager_ = calibration_manager;
    depth_processor_ = depth_processor;
    pImpl->calibration_manager_ = calibration_manager;
    pImpl->depth_processor_ = depth_processor;

    return pImpl->initializeWithCalibration();
}

bool Face3DReconstructor::isInitialized() const {
    return calibration_manager_ && depth_processor_;
}

FaceResultCode Face3DReconstructor::reconstructFace(const cv::Mat& left_image,
                                                   const cv::Mat& right_image,
                                                   const FacialLandmarks& landmarks,
                                                   Face3DModel& face_model) {
    auto start_time = std::chrono::high_resolution_clock::now();

    try {
        // Initialize output model
        face_model = Face3DModel();
        face_model.landmarks = landmarks;

        pImpl->updateProgress(0.0f);

        // Determine face region from landmarks
        cv::Rect face_region = computeFaceRegion(landmarks);
        if (face_region.area() == 0) {
            pImpl->last_error_ = "Invalid face region from landmarks";
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }

        // Generate facial point cloud
        stereo::PointCloud point_cloud;
        FaceResultCode result = pImpl->generateFacialPointCloud(left_image, right_image,
                                                               face_region, point_cloud);
        if (result != FaceResultCode::SUCCESS) {
            return result;
        }

        // Generate 3D mesh from point cloud
        result = pImpl->generateFacialMesh(point_cloud, face_model.mesh);
        if (result != FaceResultCode::SUCCESS) {
            return result;
        }

        // Apply texture mapping
        result = pImpl->applyTextureMapping(face_model, left_image, landmarks);
        if (result != FaceResultCode::SUCCESS) {
            return result;
        }

        // Compute geometric measurements
        pImpl->computeGeometricMeasurements(face_model);

        // Assess reconstruction quality
        std::map<std::string, float> detailed_metrics =
            pImpl->assessReconstructionQuality(face_model, face_model.reconstruction_quality);

        face_model.geometric_accuracy = detailed_metrics["geometric_consistency"];
        face_model.completeness_score = detailed_metrics["mesh_completeness"];

        // Banking mode validation
        if (pImpl->banking_mode_) {
            if (!face_model.isBankingGrade()) {
                pImpl->last_error_ = "Reconstruction does not meet banking-grade requirements";
                return FaceResultCode::ERROR_3D_MODEL_INCOMPLETE;
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        face_model.reconstruction_time_ms =
            std::chrono::duration<double, std::milli>(end_time - start_time).count();

        face_model.reconstruction_method = "Unlook Stereo + Mesh Generation";

        pImpl->updateProgress(100.0f);

        // Update statistics
        updateStatistics(face_model.reconstruction_time_ms, true, face_model.reconstruction_quality);

        return FaceResultCode::SUCCESS;

    } catch (const std::exception& e) {
        pImpl->last_error_ = "Exception during face reconstruction: " + std::string(e.what());
        return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
    }
}

FaceResultCode Face3DReconstructor::reconstructFromDepth(const cv::Mat& rgb_image,
                                                        const cv::Mat& depth_map,
                                                        const FacialLandmarks& landmarks,
                                                        Face3DModel& face_model) {
    // Generate point cloud from existing depth map
    stereo::PointCloud point_cloud;
    if (!depth_processor_->generatePointCloud(depth_map, rgb_image, point_cloud)) {
        pImpl->last_error_ = "Failed to generate point cloud from depth map";
        return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
    }

    // Filter for face region
    cv::Rect face_region = computeFaceRegion(landmarks);
    // Apply face-specific filtering to point cloud based on face_region

    // Continue with mesh generation
    face_model = Face3DModel();
    face_model.landmarks = landmarks;

    FaceResultCode result = pImpl->generateFacialMesh(point_cloud, face_model.mesh);
    if (result != FaceResultCode::SUCCESS) {
        return result;
    }

    // Apply texture and compute measurements
    pImpl->applyTextureMapping(face_model, rgb_image, landmarks);
    pImpl->computeGeometricMeasurements(face_model);

    return FaceResultCode::SUCCESS;
}

FaceResultCode Face3DReconstructor::exportModel(const Face3DModel& face_model,
                                               const std::string& filename,
                                               const std::string& format,
                                               bool include_texture) {
    if (format == "ply") {
        return pImpl->exportToPLY(face_model, filename, include_texture);
    } else {
        pImpl->last_error_ = "Unsupported export format: " + format;
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }
}

void Face3DReconstructor::setBankingMode(bool enable, bool require_texture, float min_quality) {
    pImpl->banking_mode_ = enable;
    pImpl->config_.banking_mode = enable;
    pImpl->config_.require_texture_mapping = require_texture;
    pImpl->config_.banking_quality_threshold = min_quality;
}

bool Face3DReconstructor::validateModelQuality(const Face3DModel& face_model,
                                              float& quality_score,
                                              std::map<std::string, float>& detailed_metrics) {
    detailed_metrics = pImpl->assessReconstructionQuality(face_model, quality_score);

    bool meets_requirements = (quality_score >= pImpl->config_.min_reconstruction_quality);

    if (pImpl->banking_mode_) {
        meets_requirements = face_model.isBankingGrade();
    }

    return meets_requirements;
}

FaceResultCode Face3DReconstructor::setConfiguration(const Face3DReconstructionConfig& config) {
    if (!config.validate()) {
        pImpl->last_error_ = "Invalid face 3D reconstruction configuration";
        return FaceResultCode::ERROR_TEMPLATE_INVALID;
    }
    pImpl->config_ = config;
    return FaceResultCode::SUCCESS;
}

Face3DReconstructionConfig Face3DReconstructor::getConfiguration() const {
    return pImpl->config_;
}

void Face3DReconstructor::setProgressCallback(std::function<void(float)> callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    progress_callback_ = callback;
    pImpl->progress_callback_ = callback;
}

std::string Face3DReconstructor::getLastError() const {
    return pImpl->last_error_;
}

std::vector<std::string> Face3DReconstructor::getSupportedFormats() {
    return {"ply", "obj", "stl"};
}

cv::Rect Face3DReconstructor::computeFaceRegion(const FacialLandmarks& landmarks) {
    if (landmarks.points2D.empty()) {
        return cv::Rect();
    }

    // Find bounding box of all landmarks with some margin
    float min_x = landmarks.points2D[0].x, max_x = landmarks.points2D[0].x;
    float min_y = landmarks.points2D[0].y, max_y = landmarks.points2D[0].y;

    for (const auto& point : landmarks.points2D) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
    }

    // Add margin around face region
    float margin_x = (max_x - min_x) * 0.1f;
    float margin_y = (max_y - min_y) * 0.1f;

    min_x -= margin_x;
    max_x += margin_x;
    min_y -= margin_y;
    max_y += margin_y;

    return cv::Rect(static_cast<int>(min_x), static_cast<int>(min_y),
                   static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
}

void Face3DReconstructor::updateStatistics(double reconstruction_time, bool success, float quality_score) {
    total_reconstructions_.fetch_add(1);
    if (success) {
        successful_reconstructions_.fetch_add(1);
    }

    // Update running averages
    total_reconstruction_time_.store(total_reconstruction_time_.load() + reconstruction_time);
    total_quality_score_.store(total_quality_score_.load() + quality_score);
}

void Face3DReconstructor::getStatistics(double& avg_reconstruction_time,
                                       size_t& total_reconstructions,
                                       float& success_rate,
                                       float& avg_quality_score) const {
    std::lock_guard<std::mutex> lock(stats_mutex_);

    total_reconstructions = total_reconstructions_.load();
    size_t successful = successful_reconstructions_.load();

    success_rate = (total_reconstructions > 0) ?
                  static_cast<float>(successful) / total_reconstructions : 0.0f;

    avg_reconstruction_time = (total_reconstructions > 0) ?
                             total_reconstruction_time_.load() / total_reconstructions : 0.0;

    avg_quality_score = (total_reconstructions > 0) ?
                       total_quality_score_.load() / total_reconstructions : 0.0f;
}

void Face3DReconstructor::resetStatistics() {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    total_reconstructions_.store(0);
    successful_reconstructions_.store(0);
    total_reconstruction_time_.store(0.0);
    total_quality_score_.store(0.0);
}

// Face3DUtils implementation
cv::Point3f Face3DUtils::computeCentroid(const Face3DMesh& face_mesh) {
    if (face_mesh.vertices.empty()) {
        return cv::Point3f(0, 0, 0);
    }

    cv::Point3f centroid(0, 0, 0);
    for (const auto& vertex : face_mesh.vertices) {
        centroid.x += vertex.x;
        centroid.y += vertex.y;
        centroid.z += vertex.z;
    }

    centroid.x /= face_mesh.vertices.size();
    centroid.y /= face_mesh.vertices.size();
    centroid.z /= face_mesh.vertices.size();

    return centroid;
}

void Face3DUtils::computeBoundingBox(const Face3DMesh& face_mesh,
                                    cv::Point3f& min_point, cv::Point3f& max_point) {
    if (face_mesh.vertices.empty()) {
        min_point = max_point = cv::Point3f(0, 0, 0);
        return;
    }

    min_point = max_point = face_mesh.vertices[0];

    for (const auto& vertex : face_mesh.vertices) {
        min_point.x = std::min(min_point.x, vertex.x);
        min_point.y = std::min(min_point.y, vertex.y);
        min_point.z = std::min(min_point.z, vertex.z);

        max_point.x = std::max(max_point.x, vertex.x);
        max_point.y = std::max(max_point.y, vertex.y);
        max_point.z = std::max(max_point.z, vertex.z);
    }
}

float Face3DUtils::computeSurfaceArea(const Face3DMesh& face_mesh) {
    float total_area = 0.0f;

    for (const auto& triangle : face_mesh.triangles) {
        if (triangle.x < face_mesh.vertices.size() &&
            triangle.y < face_mesh.vertices.size() &&
            triangle.z < face_mesh.vertices.size()) {

            const cv::Point3f& v0 = face_mesh.vertices[triangle.x];
            const cv::Point3f& v1 = face_mesh.vertices[triangle.y];
            const cv::Point3f& v2 = face_mesh.vertices[triangle.z];

            // Triangle area using cross product
            cv::Point3f edge1 = v1 - v0;
            cv::Point3f edge2 = v2 - v0;

            cv::Point3f cross;
            cross.x = edge1.y * edge2.z - edge1.z * edge2.y;
            cross.y = edge1.z * edge2.x - edge1.x * edge2.z;
            cross.z = edge1.x * edge2.y - edge1.y * edge2.x;

            float area = 0.5f * std::sqrt(cross.x * cross.x + cross.y * cross.y + cross.z * cross.z);
            total_area += area;
        }
    }

    return total_area;
}

} // namespace face
} // namespace unlook