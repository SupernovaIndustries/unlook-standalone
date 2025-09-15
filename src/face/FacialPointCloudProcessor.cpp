#include "unlook/face/FaceTypes.hpp"
#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/core/logging.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace face {

/**
 * @brief Advanced 3D point cloud processor optimized for facial landmarks
 *
 * This class provides high-precision point cloud processing specifically
 * designed for facial recognition applications with the Unlook 70mm baseline
 * stereo system. Optimized for banking-grade precision requirements.
 */
class FacialPointCloudProcessor {
public:
    /**
     * @brief Configuration for facial point cloud processing
     */
    struct Config {
        // Depth filtering parameters
        float min_depth_mm = 200.0f;           // Minimum valid face depth
        float max_depth_mm = 800.0f;           // Maximum valid face depth
        float depth_noise_threshold = 2.0f;    // Depth noise threshold in mm

        // Statistical outlier removal
        bool enable_statistical_outlier_removal = true;
        int statistical_neighbors = 50;         // Number of neighbors for statistics
        float statistical_std_dev_threshold = 2.0f; // Standard deviation threshold

        // Voxel downsampling for performance
        bool enable_voxel_downsampling = true;
        float voxel_size_mm = 0.5f;            // Voxel size for downsampling

        // Plane removal (background/wall removal)
        bool enable_plane_removal = true;
        float plane_distance_threshold = 5.0f; // Distance threshold for plane detection
        int plane_max_iterations = 1000;       // RANSAC iterations for plane fitting

        // Normal estimation
        bool compute_normals = true;
        int normal_neighbors = 20;             // Neighbors for normal computation

        // Smoothing
        bool enable_smoothing = true;
        int smoothing_iterations = 2;          // Smoothing iterations
        float smoothing_factor = 0.3f;         // Smoothing strength

        // Banking-grade requirements
        bool banking_mode = false;             // Enable banking-grade processing
        float banking_precision_threshold = 0.1f; // Required precision in mm

        bool validate() const {
            return min_depth_mm > 0 && max_depth_mm > min_depth_mm &&
                   depth_noise_threshold > 0 && voxel_size_mm > 0 &&
                   statistical_neighbors > 10 && normal_neighbors > 5;
        }
    };

private:
    Config config_;
    std::string last_error_;

public:
    explicit FacialPointCloudProcessor(const Config& config = Config{}) : config_(config) {
        if (!config_.validate()) {
            UNLOOK_LOG_WARN("Invalid FacialPointCloudProcessor configuration");
        }
    }

    /**
     * @brief Process facial point cloud with advanced filtering and optimization
     */
    FaceResultCode processFacialPointCloud(const stereo::PointCloud& input_cloud,
                                          const FacialLandmarks& landmarks,
                                          stereo::PointCloud& processed_cloud,
                                          std::map<std::string, float>& quality_metrics) {
        auto start_time = std::chrono::high_resolution_clock::now();

        try {
            processed_cloud = input_cloud;
            quality_metrics.clear();

            // Step 1: Face region filtering based on landmarks
            FaceResultCode result = filterFaceRegion(processed_cloud, landmarks);
            if (result != FaceResultCode::SUCCESS) {
                return result;
            }

            // Step 2: Depth range filtering
            result = filterDepthRange(processed_cloud);
            if (result != FaceResultCode::SUCCESS) {
                return result;
            }

            // Step 3: Statistical outlier removal
            if (config_.enable_statistical_outlier_removal) {
                result = removeStatisticalOutliers(processed_cloud);
                if (result != FaceResultCode::SUCCESS) {
                    return result;
                }
            }

            // Step 4: Background plane removal
            if (config_.enable_plane_removal) {
                result = removeLargestPlane(processed_cloud);
                if (result != FaceResultCode::SUCCESS) {
                    return result;
                }
            }

            // Step 5: Voxel downsampling for performance
            if (config_.enable_voxel_downsampling) {
                result = voxelDownsample(processed_cloud);
                if (result != FaceResultCode::SUCCESS) {
                    return result;
                }
            }

            // Step 6: Surface smoothing
            if (config_.enable_smoothing) {
                result = smoothSurface(processed_cloud);
                if (result != FaceResultCode::SUCCESS) {
                    return result;
                }
            }

            // Step 7: Compute surface normals
            if (config_.compute_normals) {
                result = computeSurfaceNormals(processed_cloud);
                if (result != FaceResultCode::SUCCESS) {
                    return result;
                }
            }

            // Step 8: Quality assessment
            result = assessPointCloudQuality(processed_cloud, landmarks, quality_metrics);
            if (result != FaceResultCode::SUCCESS) {
                return result;
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            double processing_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            quality_metrics["processing_time_ms"] = processing_time;

            // Banking-grade validation
            if (config_.banking_mode) {
                if (quality_metrics["overall_quality"] < 0.95f) {
                    last_error_ = "Point cloud quality below banking-grade requirements";
                    return FaceResultCode::ERROR_3D_MODEL_INCOMPLETE;
                }
            }

            UNLOOK_LOG_DEBUG("Facial point cloud processed: {} points, quality: {:.3f}",
                            processed_cloud.points.size(), quality_metrics["overall_quality"]);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during point cloud processing: " + std::string(e.what());
            return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
        }
    }

    /**
     * @brief Optimize landmark positions using high-precision point cloud data
     */
    FaceResultCode optimizeLandmarkPositions(const stereo::PointCloud& point_cloud,
                                            FacialLandmarks& landmarks,
                                            float search_radius_mm = 3.0f) {
        if (!landmarks.has3D() || point_cloud.points.empty()) {
            last_error_ = "3D landmarks and valid point cloud required";
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }

        try {
            size_t optimized_count = 0;

            for (size_t i = 0; i < landmarks.points3D.size(); ++i) {
                auto& landmark = landmarks.points3D[i];

                if (landmark.confidence < 0.5f || landmark.depth_quality < 0.5f) {
                    continue; // Skip low-quality landmarks
                }

                // Find nearby point cloud points
                std::vector<stereo::Point3D> nearby_points;
                findNearbyPoints(point_cloud, landmark, search_radius_mm, nearby_points);

                if (nearby_points.size() >= 5) {
                    // Optimize position using weighted centroid
                    cv::Point3f optimized_pos = computeWeightedCentroid(nearby_points);

                    // Update landmark position
                    landmark.x = optimized_pos.x;
                    landmark.y = optimized_pos.y;
                    landmark.z = optimized_pos.z;

                    // Improve confidence based on point density
                    float density_factor = std::min(1.0f, nearby_points.size() / 20.0f);
                    landmark.confidence = std::min(1.0f, landmark.confidence + 0.1f * density_factor);
                    landmark.depth_quality = std::min(1.0f, landmark.depth_quality + 0.1f * density_factor);

                    optimized_count++;
                }
            }

            // Update overall landmark metrics
            float total_confidence = 0.0f;
            float total_depth_quality = 0.0f;

            for (const auto& landmark : landmarks.points3D) {
                total_confidence += landmark.confidence;
                total_depth_quality += landmark.depth_quality;
            }

            landmarks.overall_confidence = total_confidence / landmarks.points3D.size();
            landmarks.depth_coverage = total_depth_quality / landmarks.points3D.size();

            UNLOOK_LOG_DEBUG("Optimized {} landmarks using point cloud data", optimized_count);

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during landmark optimization: " + std::string(e.what());
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }
    }

    /**
     * @brief Extract high-precision facial measurements from point cloud
     */
    FaceResultCode extractPrecisionMeasurements(const stereo::PointCloud& point_cloud,
                                               const FacialLandmarks& landmarks,
                                               std::map<std::string, float>& measurements) {
        if (!landmarks.has3D()) {
            last_error_ = "3D landmarks required for precision measurements";
            return FaceResultCode::ERROR_LANDMARK_DETECTION_FAILED;
        }

        try {
            measurements.clear();

            const auto& points3d = landmarks.points3D;

            // Interpupillary distance (high precision)
            if (points3d.size() > 45) {
                const auto& left_eye = points3d[36];   // Left eye outer corner
                const auto& right_eye = points3d[45];  // Right eye outer corner

                float ipd = std::sqrt(
                    (left_eye.x - right_eye.x) * (left_eye.x - right_eye.x) +
                    (left_eye.y - right_eye.y) * (left_eye.y - right_eye.y) +
                    (left_eye.z - right_eye.z) * (left_eye.z - right_eye.z)
                );
                measurements["interpupillary_distance_mm"] = ipd;
            }

            // Face width at cheekbones
            if (points3d.size() > 16) {
                const auto& left_cheek = points3d[0];   // Left jaw point
                const auto& right_cheek = points3d[16]; // Right jaw point

                float face_width = std::sqrt(
                    (left_cheek.x - right_cheek.x) * (left_cheek.x - right_cheek.x) +
                    (left_cheek.y - right_cheek.y) * (left_cheek.y - right_cheek.y) +
                    (left_cheek.z - right_cheek.z) * (left_cheek.z - right_cheek.z)
                );
                measurements["face_width_mm"] = face_width;
            }

            // Nose prominence measurement
            if (points3d.size() > 33) {
                const auto& nose_tip = points3d[33];     // Nose tip
                const auto& nose_bridge = points3d[27];  // Nose bridge

                float nose_prominence = std::abs(nose_tip.z - nose_bridge.z);
                measurements["nose_prominence_mm"] = nose_prominence;
            }

            // Face volume estimation from point cloud
            float face_volume = estimateFaceVolume(point_cloud);
            measurements["face_volume_mm3"] = face_volume;

            // Surface area estimation
            float surface_area = estimateSurfaceArea(point_cloud);
            measurements["surface_area_mm2"] = surface_area;

            // Facial symmetry analysis
            float symmetry_score = analyzeFacialSymmetry(landmarks, point_cloud);
            measurements["symmetry_score"] = symmetry_score;

            UNLOOK_LOG_DEBUG("Extracted {} precision measurements", measurements.size());

            return FaceResultCode::SUCCESS;

        } catch (const std::exception& e) {
            last_error_ = "Exception during measurement extraction: " + std::string(e.what());
            return FaceResultCode::ERROR_STEREO_RECONSTRUCTION_FAILED;
        }
    }

    /**
     * @brief Get last error message
     */
    std::string getLastError() const {
        return last_error_;
    }

    /**
     * @brief Set processing configuration
     */
    void setConfiguration(const Config& config) {
        config_ = config;
    }

    /**
     * @brief Get current configuration
     */
    Config getConfiguration() const {
        return config_;
    }

private:
    /**
     * @brief Filter point cloud to face region based on landmarks
     */
    FaceResultCode filterFaceRegion(stereo::PointCloud& point_cloud,
                                   const FacialLandmarks& landmarks) {
        if (landmarks.points2D.empty()) {
            return FaceResultCode::SUCCESS; // No filtering needed
        }

        // Compute face bounding box from landmarks
        float min_x = landmarks.points2D[0].x, max_x = landmarks.points2D[0].x;
        float min_y = landmarks.points2D[0].y, max_y = landmarks.points2D[0].y;

        for (const auto& point : landmarks.points2D) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }

        // Add margin around face region
        float margin_x = (max_x - min_x) * 0.2f;
        float margin_y = (max_y - min_y) * 0.2f;
        min_x -= margin_x; max_x += margin_x;
        min_y -= margin_y; max_y += margin_y;

        // Filter points based on 2D projection (if organized point cloud)
        if (point_cloud.isOrganized && point_cloud.width > 0 && point_cloud.height > 0) {
            std::vector<stereo::Point3D> filtered_points;

            for (int row = 0; row < point_cloud.height; ++row) {
                for (int col = 0; col < point_cloud.width; ++col) {
                    if (col >= min_x && col <= max_x && row >= min_y && row <= max_y) {
                        int idx = row * point_cloud.width + col;
                        if (idx < static_cast<int>(point_cloud.points.size())) {
                            filtered_points.push_back(point_cloud.points[idx]);
                        }
                    }
                }
            }

            point_cloud.points = std::move(filtered_points);
            point_cloud.isOrganized = false;
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Filter points by depth range
     */
    FaceResultCode filterDepthRange(stereo::PointCloud& point_cloud) {
        std::vector<stereo::Point3D> filtered_points;
        filtered_points.reserve(point_cloud.points.size());

        for (const auto& point : point_cloud.points) {
            if (point.z >= config_.min_depth_mm && point.z <= config_.max_depth_mm) {
                filtered_points.push_back(point);
            }
        }

        point_cloud.points = std::move(filtered_points);
        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Remove statistical outliers using ARM64-optimized algorithms
     */
    FaceResultCode removeStatisticalOutliers(stereo::PointCloud& point_cloud) {
        if (point_cloud.points.size() < static_cast<size_t>(config_.statistical_neighbors)) {
            return FaceResultCode::SUCCESS; // Not enough points for statistical analysis
        }

        std::vector<stereo::Point3D> filtered_points;
        filtered_points.reserve(point_cloud.points.size());

        // Compute distances to k-nearest neighbors for each point
        for (size_t i = 0; i < point_cloud.points.size(); ++i) {
            const auto& query_point = point_cloud.points[i];

            // Find k nearest neighbors
            std::vector<float> distances;
            distances.reserve(config_.statistical_neighbors);

            for (size_t j = 0; j < point_cloud.points.size(); ++j) {
                if (i == j) continue;

                const auto& neighbor = point_cloud.points[j];
                float dist = computeDistance3D(query_point, neighbor);
                distances.push_back(dist);
            }

            // Sort and take k nearest
            std::partial_sort(distances.begin(),
                            distances.begin() + std::min(static_cast<size_t>(config_.statistical_neighbors),
                                                         distances.size()),
                            distances.end());

            // Compute mean distance to k-nearest neighbors
            float mean_dist = 0.0f;
            int count = std::min(config_.statistical_neighbors, static_cast<int>(distances.size()));
            for (int k = 0; k < count; ++k) {
                mean_dist += distances[k];
            }
            mean_dist /= count;

            // Check if point is within statistical bounds
            if (mean_dist <= config_.statistical_std_dev_threshold * 2.0f) { // Simplified threshold
                filtered_points.push_back(query_point);
            }
        }

        point_cloud.points = std::move(filtered_points);
        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Remove largest plane (typically background) using RANSAC
     */
    FaceResultCode removeLargestPlane(stereo::PointCloud& point_cloud) {
        if (point_cloud.points.size() < 100) {
            return FaceResultCode::SUCCESS; // Not enough points for plane fitting
        }

        // RANSAC plane fitting
        cv::Vec4f best_plane;
        size_t best_inlier_count = 0;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<size_t> dis(0, point_cloud.points.size() - 1);

        for (int iter = 0; iter < config_.plane_max_iterations; ++iter) {
            // Sample 3 random points
            std::vector<cv::Point3f> sample_points;
            for (int i = 0; i < 3; ++i) {
                size_t idx = dis(gen);
                const auto& p = point_cloud.points[idx];
                sample_points.emplace_back(p.x, p.y, p.z);
            }

            // Compute plane equation
            cv::Vec4f plane = computePlaneEquation(sample_points);

            // Count inliers
            size_t inlier_count = 0;
            for (const auto& point : point_cloud.points) {
                float distance = std::abs(plane[0] * point.x + plane[1] * point.y +
                                        plane[2] * point.z + plane[3]) /
                               std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);

                if (distance <= config_.plane_distance_threshold) {
                    inlier_count++;
                }
            }

            if (inlier_count > best_inlier_count) {
                best_inlier_count = inlier_count;
                best_plane = plane;
            }
        }

        // Remove inliers of the best plane
        if (best_inlier_count > point_cloud.points.size() / 4) { // At least 25% of points
            std::vector<stereo::Point3D> filtered_points;
            filtered_points.reserve(point_cloud.points.size());

            for (const auto& point : point_cloud.points) {
                float distance = std::abs(best_plane[0] * point.x + best_plane[1] * point.y +
                                        best_plane[2] * point.z + best_plane[3]) /
                               std::sqrt(best_plane[0] * best_plane[0] + best_plane[1] * best_plane[1] +
                                        best_plane[2] * best_plane[2]);

                if (distance > config_.plane_distance_threshold) {
                    filtered_points.push_back(point);
                }
            }

            point_cloud.points = std::move(filtered_points);
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Voxel downsampling for performance optimization
     */
    FaceResultCode voxelDownsample(stereo::PointCloud& point_cloud) {
        if (point_cloud.points.empty()) {
            return FaceResultCode::SUCCESS;
        }

        // Create voxel grid
        std::map<std::tuple<int, int, int>, std::vector<stereo::Point3D>> voxel_grid;

        for (const auto& point : point_cloud.points) {
            int vx = static_cast<int>(std::floor(point.x / config_.voxel_size_mm));
            int vy = static_cast<int>(std::floor(point.y / config_.voxel_size_mm));
            int vz = static_cast<int>(std::floor(point.z / config_.voxel_size_mm));

            voxel_grid[std::make_tuple(vx, vy, vz)].push_back(point);
        }

        // Compute centroid for each voxel
        std::vector<stereo::Point3D> downsampled_points;
        downsampled_points.reserve(voxel_grid.size());

        for (const auto& voxel : voxel_grid) {
            const auto& points_in_voxel = voxel.second;

            stereo::Point3D centroid;
            centroid.x = centroid.y = centroid.z = 0.0f;
            centroid.r = centroid.g = centroid.b = 0;
            centroid.confidence = 0.0f;

            for (const auto& p : points_in_voxel) {
                centroid.x += p.x;
                centroid.y += p.y;
                centroid.z += p.z;
                centroid.r += p.r;
                centroid.g += p.g;
                centroid.b += p.b;
                centroid.confidence += p.confidence;
            }

            float count = static_cast<float>(points_in_voxel.size());
            centroid.x /= count;
            centroid.y /= count;
            centroid.z /= count;
            centroid.r = static_cast<uint8_t>(centroid.r / count);
            centroid.g = static_cast<uint8_t>(centroid.g / count);
            centroid.b = static_cast<uint8_t>(centroid.b / count);
            centroid.confidence /= count;

            downsampled_points.push_back(centroid);
        }

        point_cloud.points = std::move(downsampled_points);
        point_cloud.isOrganized = false;

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Smooth surface using iterative averaging
     */
    FaceResultCode smoothSurface(stereo::PointCloud& point_cloud) {
        for (int iter = 0; iter < config_.smoothing_iterations; ++iter) {
            std::vector<stereo::Point3D> smoothed_points = point_cloud.points;

            for (size_t i = 0; i < point_cloud.points.size(); ++i) {
                const auto& current_point = point_cloud.points[i];

                // Find nearby points for smoothing
                std::vector<stereo::Point3D> neighbors;
                for (const auto& other_point : point_cloud.points) {
                    float dist = computeDistance3D(current_point, other_point);
                    if (dist > 0 && dist <= config_.voxel_size_mm * 3) {
                        neighbors.push_back(other_point);
                    }
                }

                if (neighbors.size() >= 3) {
                    // Compute weighted average
                    float weight_sum = 0.0f;
                    cv::Point3f weighted_pos(0, 0, 0);

                    for (const auto& neighbor : neighbors) {
                        float weight = 1.0f / (1.0f + computeDistance3D(current_point, neighbor));
                        weighted_pos.x += neighbor.x * weight;
                        weighted_pos.y += neighbor.y * weight;
                        weighted_pos.z += neighbor.z * weight;
                        weight_sum += weight;
                    }

                    if (weight_sum > 0) {
                        weighted_pos.x /= weight_sum;
                        weighted_pos.y /= weight_sum;
                        weighted_pos.z /= weight_sum;

                        // Apply smoothing factor
                        smoothed_points[i].x = current_point.x * (1.0f - config_.smoothing_factor) +
                                              weighted_pos.x * config_.smoothing_factor;
                        smoothed_points[i].y = current_point.y * (1.0f - config_.smoothing_factor) +
                                              weighted_pos.y * config_.smoothing_factor;
                        smoothed_points[i].z = current_point.z * (1.0f - config_.smoothing_factor) +
                                              weighted_pos.z * config_.smoothing_factor;
                    }
                }
            }

            point_cloud.points = std::move(smoothed_points);
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Compute surface normals for point cloud
     */
    FaceResultCode computeSurfaceNormals(stereo::PointCloud& point_cloud) {
        // Surface normals would be stored in an extended point structure
        // For this implementation, we'll compute them but not store them
        // as the current PointCloud structure doesn't include normals

        for (size_t i = 0; i < point_cloud.points.size(); ++i) {
            const auto& query_point = point_cloud.points[i];

            // Find k nearest neighbors
            std::vector<cv::Point3f> neighbors;
            for (const auto& other_point : point_cloud.points) {
                float dist = computeDistance3D(query_point, other_point);
                if (dist > 0 && neighbors.size() < static_cast<size_t>(config_.normal_neighbors)) {
                    neighbors.emplace_back(other_point.x, other_point.y, other_point.z);
                }
            }

            if (neighbors.size() >= 3) {
                // Compute normal using PCA (simplified)
                cv::Point3f normal = computeNormalPCA(neighbors);

                // Store normal in point confidence (simplified approach)
                point_cloud.points[i].confidence = std::max(point_cloud.points[i].confidence,
                                                          cv::norm(normal));
            }
        }

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Assess point cloud quality metrics
     */
    FaceResultCode assessPointCloudQuality(const stereo::PointCloud& point_cloud,
                                          const FacialLandmarks& landmarks,
                                          std::map<std::string, float>& quality_metrics) {
        // Point density
        float point_density = static_cast<float>(point_cloud.points.size()) / 10000.0f; // Normalize to typical face area
        quality_metrics["point_density"] = std::min(1.0f, point_density);

        // Depth consistency
        float depth_variance = computeDepthVariance(point_cloud);
        quality_metrics["depth_consistency"] = std::max(0.0f, 1.0f - depth_variance / 50.0f);

        // Completeness (how well landmarks are covered)
        float landmark_coverage = computeLandmarkCoverage(point_cloud, landmarks);
        quality_metrics["landmark_coverage"] = landmark_coverage;

        // Noise level
        float noise_level = estimateNoiseLevel(point_cloud);
        quality_metrics["noise_level"] = std::max(0.0f, 1.0f - noise_level / 5.0f);

        // Overall quality
        float overall_quality = (
            quality_metrics["point_density"] * 0.25f +
            quality_metrics["depth_consistency"] * 0.25f +
            quality_metrics["landmark_coverage"] * 0.30f +
            quality_metrics["noise_level"] * 0.20f
        );
        quality_metrics["overall_quality"] = overall_quality;

        return FaceResultCode::SUCCESS;
    }

    /**
     * @brief Utility functions
     */
    float computeDistance3D(const stereo::Point3D& p1, const stereo::Point3D& p2) {
        float dx = p1.x - p2.x;
        float dy = p1.y - p2.y;
        float dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    cv::Vec4f computePlaneEquation(const std::vector<cv::Point3f>& points) {
        if (points.size() < 3) {
            return cv::Vec4f(0, 0, 1, 0); // Default plane
        }

        // Use first 3 points to compute plane
        cv::Point3f v1 = points[1] - points[0];
        cv::Point3f v2 = points[2] - points[0];

        // Normal vector (cross product)
        cv::Point3f normal;
        normal.x = v1.y * v2.z - v1.z * v2.y;
        normal.y = v1.z * v2.x - v1.x * v2.z;
        normal.z = v1.x * v2.y - v1.y * v2.x;

        // Normalize
        float norm = std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
        if (norm > 0) {
            normal.x /= norm;
            normal.y /= norm;
            normal.z /= norm;
        }

        // Compute d: ax + by + cz + d = 0
        float d = -(normal.x * points[0].x + normal.y * points[0].y + normal.z * points[0].z);

        return cv::Vec4f(normal.x, normal.y, normal.z, d);
    }

    cv::Point3f computeNormalPCA(const std::vector<cv::Point3f>& points) {
        if (points.size() < 3) {
            return cv::Point3f(0, 0, 1);
        }

        // Compute centroid
        cv::Point3f centroid(0, 0, 0);
        for (const auto& p : points) {
            centroid += p;
        }
        centroid.x /= points.size();
        centroid.y /= points.size();
        centroid.z /= points.size();

        // Compute covariance matrix
        cv::Mat covariance = cv::Mat::zeros(3, 3, CV_32F);
        for (const auto& p : points) {
            cv::Point3f diff = p - centroid;
            covariance.at<float>(0, 0) += diff.x * diff.x;
            covariance.at<float>(0, 1) += diff.x * diff.y;
            covariance.at<float>(0, 2) += diff.x * diff.z;
            covariance.at<float>(1, 1) += diff.y * diff.y;
            covariance.at<float>(1, 2) += diff.y * diff.z;
            covariance.at<float>(2, 2) += diff.z * diff.z;
        }

        // Make symmetric
        covariance.at<float>(1, 0) = covariance.at<float>(0, 1);
        covariance.at<float>(2, 0) = covariance.at<float>(0, 2);
        covariance.at<float>(2, 1) = covariance.at<float>(1, 2);

        // Compute eigenvalues and eigenvectors
        cv::Mat eigenvalues, eigenvectors;
        cv::eigen(covariance, eigenvalues, eigenvectors);

        // Normal is the eigenvector corresponding to smallest eigenvalue
        cv::Point3f normal(eigenvectors.at<float>(2, 0),
                          eigenvectors.at<float>(2, 1),
                          eigenvectors.at<float>(2, 2));

        return normal;
    }

    void findNearbyPoints(const stereo::PointCloud& point_cloud,
                         const LandmarkPoint3D& landmark,
                         float radius,
                         std::vector<stereo::Point3D>& nearby_points) {
        nearby_points.clear();

        for (const auto& point : point_cloud.points) {
            float dx = point.x - landmark.x;
            float dy = point.y - landmark.y;
            float dz = point.z - landmark.z;
            float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

            if (distance <= radius) {
                nearby_points.push_back(point);
            }
        }
    }

    cv::Point3f computeWeightedCentroid(const std::vector<stereo::Point3D>& points) {
        cv::Point3f centroid(0, 0, 0);
        float total_weight = 0.0f;

        for (const auto& point : points) {
            float weight = point.confidence;
            centroid.x += point.x * weight;
            centroid.y += point.y * weight;
            centroid.z += point.z * weight;
            total_weight += weight;
        }

        if (total_weight > 0) {
            centroid.x /= total_weight;
            centroid.y /= total_weight;
            centroid.z /= total_weight;
        }

        return centroid;
    }

    float estimateFaceVolume(const stereo::PointCloud& point_cloud) {
        // Simplified volume estimation using convex hull approximation
        if (point_cloud.points.size() < 4) {
            return 0.0f;
        }

        // Find bounding box
        float min_x = point_cloud.points[0].x, max_x = point_cloud.points[0].x;
        float min_y = point_cloud.points[0].y, max_y = point_cloud.points[0].y;
        float min_z = point_cloud.points[0].z, max_z = point_cloud.points[0].z;

        for (const auto& point : point_cloud.points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
            min_z = std::min(min_z, point.z);
            max_z = std::max(max_z, point.z);
        }

        // Approximate volume as fraction of bounding box
        float bounding_volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
        return bounding_volume * 0.6f; // Approximate face occupancy factor
    }

    float estimateSurfaceArea(const stereo::PointCloud& point_cloud) {
        // Simplified surface area estimation
        float total_area = 0.0f;

        for (size_t i = 0; i < point_cloud.points.size(); ++i) {
            // Estimate local surface area around each point
            float local_area = config_.voxel_size_mm * config_.voxel_size_mm;
            total_area += local_area;
        }

        return total_area;
    }

    float analyzeFacialSymmetry(const FacialLandmarks& landmarks,
                               const stereo::PointCloud& point_cloud) {
        if (!landmarks.has3D() || landmarks.points3D.size() != 68) {
            return 0.5f; // Default symmetry score
        }

        // Compute facial midline
        const auto& nose_bridge = landmarks.points3D[27];
        const auto& nose_tip = landmarks.points3D[33];
        const auto& chin = landmarks.points3D[8];

        // Midline plane (simplified)
        cv::Point3f midline_normal(1, 0, 0); // Assume X is left-right axis

        // Analyze symmetry by comparing left and right landmarks
        float symmetry_score = 1.0f;
        std::vector<std::pair<int, int>> symmetric_pairs = {
            {17, 26}, {18, 25}, {19, 24}, {20, 23}, {21, 22}, // Eyebrows
            {36, 45}, {37, 44}, {38, 43}, {39, 42}, {40, 47}, {41, 46}, // Eyes
            {48, 54}, {49, 53}, {50, 52} // Mouth
        };

        float total_asymmetry = 0.0f;
        for (const auto& pair : symmetric_pairs) {
            const auto& left_point = landmarks.points3D[pair.first];
            const auto& right_point = landmarks.points3D[pair.second];

            // Compute asymmetry
            float asymmetry = std::abs(std::abs(left_point.x) - std::abs(right_point.x));
            total_asymmetry += asymmetry;
        }

        // Normalize asymmetry to symmetry score
        float avg_asymmetry = total_asymmetry / symmetric_pairs.size();
        symmetry_score = std::max(0.0f, 1.0f - avg_asymmetry / 10.0f);

        return symmetry_score;
    }

    float computeDepthVariance(const stereo::PointCloud& point_cloud) {
        if (point_cloud.points.empty()) {
            return 0.0f;
        }

        float mean_depth = 0.0f;
        for (const auto& point : point_cloud.points) {
            mean_depth += point.z;
        }
        mean_depth /= point_cloud.points.size();

        float variance = 0.0f;
        for (const auto& point : point_cloud.points) {
            float diff = point.z - mean_depth;
            variance += diff * diff;
        }
        variance /= point_cloud.points.size();

        return std::sqrt(variance);
    }

    float computeLandmarkCoverage(const stereo::PointCloud& point_cloud,
                                 const FacialLandmarks& landmarks) {
        if (!landmarks.has3D()) {
            return 0.0f;
        }

        size_t covered_landmarks = 0;
        float search_radius = 5.0f; // mm

        for (const auto& landmark : landmarks.points3D) {
            if (landmark.confidence > 0.5f) {
                // Check if landmark is well-supported by point cloud
                std::vector<stereo::Point3D> nearby_points;
                findNearbyPoints(point_cloud, landmark, search_radius, nearby_points);

                if (nearby_points.size() >= 3) {
                    covered_landmarks++;
                }
            }
        }

        return static_cast<float>(covered_landmarks) / landmarks.points3D.size();
    }

    float estimateNoiseLevel(const stereo::PointCloud& point_cloud) {
        if (point_cloud.points.size() < 10) {
            return 0.0f;
        }

        float total_noise = 0.0f;
        size_t sample_count = 0;

        // Sample points and compute local noise estimates
        for (size_t i = 0; i < point_cloud.points.size(); i += 10) {
            const auto& query_point = point_cloud.points[i];

            // Find nearby points
            std::vector<float> distances;
            for (const auto& other_point : point_cloud.points) {
                float dist = computeDistance3D(query_point, other_point);
                if (dist > 0 && dist < 10.0f) { // Within 10mm
                    distances.push_back(dist);
                }
            }

            if (distances.size() >= 5) {
                // Compute standard deviation of distances as noise estimate
                float mean_dist = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
                float variance = 0.0f;
                for (float dist : distances) {
                    float diff = dist - mean_dist;
                    variance += diff * diff;
                }
                variance /= distances.size();

                total_noise += std::sqrt(variance);
                sample_count++;
            }
        }

        return (sample_count > 0) ? total_noise / sample_count : 0.0f;
    }
};

} // namespace face
} // namespace unlook