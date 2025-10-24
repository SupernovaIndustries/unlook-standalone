#include "unlook/mesh/PoissonReconstructor.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <stdexcept>

// Open3D includes
#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/geometry/KDTreeFlann.h>
#include <open3d/utility/Logging.h>
#endif

// Logging macros (compatible with Unlook logging system)
#define UNLOOK_LOG_DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#define UNLOOK_LOG_INFO(msg) std::cout << "[INFO] " << msg << std::endl
#define UNLOOK_LOG_WARN(msg) std::cerr << "[WARN] " << msg << std::endl
#define UNLOOK_LOG_ERROR(msg) std::cerr << "[ERROR] " << msg << std::endl

namespace unlook {
namespace mesh {

// ============================================================================
// PoissonSettings Implementation
// ============================================================================

std::string PoissonSettings::toString() const {
    std::stringstream ss;
    ss << "PoissonSettings:\n";
    ss << "  Octree Depth: " << octree_depth << "\n";
    ss << "  Scale Factor: " << scale_factor << "\n";
    ss << "  Quadratic Fit: " << (use_quadratic_fit ? "Yes" : "No") << "\n";
    ss << "  Density Threshold: " << density_threshold << "\n";
    ss << "  Quality Mode: ";
    switch (mode) {
        case QualityMode::SHARP: ss << "SHARP\n"; break;
        case QualityMode::SMOOTH: ss << "SMOOTH\n"; break;
        case QualityMode::ADAPTIVE: ss << "ADAPTIVE\n"; break;
    }
    ss << "  Point Weight: " << point_weight << "\n";
    ss << "  Solver Divide: " << solver_divide << "\n";
    ss << "  Iso Divide: " << iso_divide << "\n";
    ss << "  Num Threads: " << (num_threads == 0 ? "auto" : std::to_string(num_threads)) << "\n";
    return ss.str();
}

// ============================================================================
// ReconstructionStats Implementation
// ============================================================================

std::string ReconstructionStats::toString() const {
    std::stringstream ss;
    ss << "Reconstruction Statistics:\n";
    ss << "  Input Points: " << input_points << "\n";
    ss << "  Output Vertices: " << output_vertices << "\n";
    ss << "  Output Triangles: " << output_triangles << "\n";
    ss << "  Removed Low-Density Vertices: " << removed_low_density_vertices << "\n";
    ss << "  \n";
    ss << "  Quality Metrics:\n";
    ss << "    Watertight: " << (is_watertight ? "YES" : "NO") << "\n";
    ss << "    Manifold: " << (is_manifold ? "YES" : "NO") << "\n";
    ss << "    Average Triangle Quality: " << std::fixed << std::setprecision(3)
       << average_triangle_quality << "\n";
    ss << "    Min Triangle Quality: " << std::fixed << std::setprecision(3)
       << min_triangle_quality << "\n";
    ss << "    Degenerate Triangles: " << degenerate_triangles << "\n";
    ss << "  \n";
    ss << "  Performance:\n";
    ss << "    Total Time: " << std::fixed << std::setprecision(2)
       << reconstruction_time_ms << " ms\n";
    ss << "    Octree Build: " << std::fixed << std::setprecision(2)
       << octree_build_time_ms << " ms\n";
    ss << "    Solver: " << std::fixed << std::setprecision(2)
       << solver_time_ms << " ms\n";
    ss << "    Mesh Extraction: " << std::fixed << std::setprecision(2)
       << mesh_extraction_time_ms << " ms\n";
    ss << "    Post-processing: " << std::fixed << std::setprecision(2)
       << postprocessing_time_ms << " ms\n";
    ss << "    Memory Usage: " << memory_usage_mb << " MB\n";
    ss << "  \n";
    ss << "  Surface Characteristics:\n";
    ss << "    Surface Area: " << std::fixed << std::setprecision(2)
       << surface_area << " mm²\n";
    ss << "    Bounding Box Volume: " << std::fixed << std::setprecision(2)
       << bounding_box_volume << " mm³\n";
    ss << "    Bounding Box: [" << bounding_box_min[0] << ", "
       << bounding_box_min[1] << ", " << bounding_box_min[2] << "] to ["
       << bounding_box_max[0] << ", " << bounding_box_max[1] << ", "
       << bounding_box_max[2] << "]\n";
    return ss.str();
}

// ============================================================================
// PoissonReconstructor Implementation
// ============================================================================

PoissonReconstructor::PoissonReconstructor() {
#ifdef __aarch64__
    arm64_optimized_ = true;
    UNLOOK_LOG_INFO("PoissonReconstructor: ARM64 optimizations enabled");
#endif

#ifdef OPEN3D_ENABLED
    // Set Open3D logging level to warnings only
    open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
#endif
}

PoissonReconstructor::~PoissonReconstructor() {
    // Ensure no reconstruction is in progress
    if (is_processing_.load()) {
        UNLOOK_LOG_WARN("PoissonReconstructor destroyed while processing");
    }
}

#ifdef OPEN3D_ENABLED

bool PoissonReconstructor::validateInputCloud(const open3d::geometry::PointCloud& cloud) {
    // Check cloud has points
    if (cloud.points_.empty()) {
        last_error_ = "Input point cloud is empty";
        UNLOOK_LOG_ERROR(last_error_);
        return false;
    }

    // Check minimum point count (Poisson needs sufficient density)
    const size_t MIN_POINTS = 100;
    if (cloud.points_.size() < MIN_POINTS) {
        std::stringstream ss;
        ss << "Input point cloud has too few points: " << cloud.points_.size()
           << " (minimum: " << MIN_POINTS << ")";
        last_error_ = ss.str();
        UNLOOK_LOG_ERROR(last_error_);
        return false;
    }

    // Check for NaN/Inf values
    size_t invalid_points = 0;
    for (const auto& point : cloud.points_) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
            invalid_points++;
        }
    }

    if (invalid_points > 0) {
        std::stringstream ss;
        ss << "Input point cloud contains " << invalid_points << " invalid points (NaN/Inf)";
        last_error_ = ss.str();
        UNLOOK_LOG_WARN(last_error_);
        // Allow processing but warn user
    }

    // Check normals are present
    if (cloud.normals_.empty()) {
        last_error_ = "Input point cloud missing normals (required for Poisson)";
        UNLOOK_LOG_ERROR(last_error_);
        return false;
    }

    if (cloud.normals_.size() != cloud.points_.size()) {
        last_error_ = "Point cloud normals size mismatch";
        UNLOOK_LOG_ERROR(last_error_);
        return false;
    }

    return true;
}

bool PoissonReconstructor::ensureNormals(open3d::geometry::PointCloud& cloud) {
    if (cloud.HasNormals() && cloud.normals_.size() == cloud.points_.size()) {
        return true;
    }

    UNLOOK_LOG_INFO("Computing point cloud normals...");

    // Estimate normals using KNN search
    const int knn = 30;  // Number of neighbors for normal estimation
    cloud.EstimateNormals(
        open3d::geometry::KDTreeSearchParamKNN(knn),
        false  // Don't use Covariance method
    );

    // Orient normals consistently
    cloud.OrientNormalsConsistentTangentPlane(knn);

    if (!cloud.HasNormals()) {
        last_error_ = "Failed to compute point cloud normals";
        UNLOOK_LOG_ERROR(last_error_);
        return false;
    }

    return true;
}

void PoissonReconstructor::applyDensityFiltering(
    open3d::geometry::TriangleMesh& mesh,
    const std::vector<double>& densities,
    double threshold) {

    if (densities.empty()) {
        UNLOOK_LOG_WARN("No density information available for filtering");
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Find vertices to remove (low density)
    std::vector<size_t> vertices_to_remove;
    for (size_t i = 0; i < densities.size(); ++i) {
        if (densities[i] < threshold) {
            vertices_to_remove.push_back(i);
        }
    }

    last_stats_.removed_low_density_vertices = vertices_to_remove.size();

    if (vertices_to_remove.empty()) {
        UNLOOK_LOG_INFO("No low-density vertices to remove");
        return;
    }

    UNLOOK_LOG_INFO("Removing " + std::to_string(vertices_to_remove.size()) +
                    " low-density vertices");

    // Remove vertices and update triangles
    mesh.RemoveVerticesByIndex(vertices_to_remove);
    mesh.RemoveDegenerateTriangles();
    mesh.RemoveUnreferencedVertices();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    UNLOOK_LOG_DEBUG("Density filtering took " + std::to_string(duration.count()) + " ms");
}

void PoissonReconstructor::computeMeshStatistics(
    const open3d::geometry::TriangleMesh& mesh) {

    auto start = std::chrono::high_resolution_clock::now();

    // Basic counts
    last_stats_.output_vertices = mesh.vertices_.size();
    last_stats_.output_triangles = mesh.triangles_.size();

    // Topology checks
    last_stats_.is_watertight = mesh.IsWatertight();
    last_stats_.is_manifold = mesh.IsEdgeManifold();

    // Triangle quality assessment
    if (!mesh.triangles_.empty()) {
        std::vector<double> qualities;
        qualities.reserve(mesh.triangles_.size());

        size_t degenerate_count = 0;
        const double DEGENERATE_THRESHOLD = 1e-10;

        for (const auto& triangle : mesh.triangles_) {
            // Get triangle vertices
            const auto& v0 = mesh.vertices_[triangle(0)];
            const auto& v1 = mesh.vertices_[triangle(1)];
            const auto& v2 = mesh.vertices_[triangle(2)];

            // Compute edge lengths
            double a = (v1 - v0).norm();
            double b = (v2 - v1).norm();
            double c = (v0 - v2).norm();

            // Check for degenerate triangle
            if (a < DEGENERATE_THRESHOLD || b < DEGENERATE_THRESHOLD || c < DEGENERATE_THRESHOLD) {
                degenerate_count++;
                qualities.push_back(0.0);
                continue;
            }

            // Compute triangle quality (normalized area / circumradius)
            // Quality score ranges from 0 (degenerate) to 1 (equilateral)
            double s = (a + b + c) / 2.0;  // Semi-perimeter
            double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));
            double circumradius = (a * b * c) / (4.0 * area + 1e-10);
            double quality = (area * std::sqrt(3.0)) / (1.5 * circumradius * circumradius + 1e-10);
            quality = std::clamp(quality, 0.0, 1.0);

            qualities.push_back(quality);
        }

        last_stats_.degenerate_triangles = degenerate_count;

        if (!qualities.empty()) {
            last_stats_.average_triangle_quality =
                std::accumulate(qualities.begin(), qualities.end(), 0.0) / qualities.size();
            last_stats_.min_triangle_quality =
                *std::min_element(qualities.begin(), qualities.end());
        }
    }

    // Surface area and bounding box
    last_stats_.surface_area = mesh.GetSurfaceArea() * 1e6;  // Convert m² to mm²

    auto bbox = mesh.GetAxisAlignedBoundingBox();
    auto extent = bbox.GetExtent();
    last_stats_.bounding_box_volume = (extent.x() * extent.y() * extent.z()) * 1e9;  // m³ to mm³

    auto min_bound = bbox.GetMinBound();
    auto max_bound = bbox.GetMaxBound();
    last_stats_.bounding_box_min = cv::Vec3f(
        min_bound.x() * 1000.0,  // m to mm
        min_bound.y() * 1000.0,
        min_bound.z() * 1000.0
    );
    last_stats_.bounding_box_max = cv::Vec3f(
        max_bound.x() * 1000.0,
        max_bound.y() * 1000.0,
        max_bound.z() * 1000.0
    );

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    UNLOOK_LOG_DEBUG("Statistics computation took " + std::to_string(duration.count()) + " ms");
}

void PoissonReconstructor::updateProgress(int progress) {
    if (progress_callback_) {
        try {
            progress_callback_(std::clamp(progress, 0, 100));
        } catch (const std::exception& e) {
            UNLOOK_LOG_WARN("Progress callback exception: " + std::string(e.what()));
        }
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh>
PoissonReconstructor::reconstruct(
    const open3d::geometry::PointCloud& cloud,
    const PoissonSettings& settings) {

    // Thread safety: Lock for entire operation
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);

    // Check if already processing (should never happen with lock, but be defensive)
    if (is_processing_.load()) {
        last_error_ = "Reconstruction already in progress";
        UNLOOK_LOG_ERROR(last_error_);
        return nullptr;
    }

    // Set processing flag
    is_processing_.store(true);

    // Clear previous state
    last_stats_ = ReconstructionStats();
    last_error_.clear();

    auto total_start = std::chrono::high_resolution_clock::now();

    try {
        // Validate settings
        if (!settings.validate()) {
            last_error_ = "Invalid reconstruction settings";
            UNLOOK_LOG_ERROR(last_error_);
            is_processing_.store(false);
            return nullptr;
        }

        UNLOOK_LOG_INFO("Starting Poisson reconstruction");
        UNLOOK_LOG_INFO(settings.toString());

        updateProgress(5);

        // Create mutable copy of input cloud for processing
        auto working_cloud = std::make_shared<open3d::geometry::PointCloud>(cloud);
        last_stats_.input_points = working_cloud->points_.size();

        // Validate input
        if (!validateInputCloud(*working_cloud)) {
            is_processing_.store(false);
            return nullptr;
        }

        updateProgress(10);

        // Ensure normals are present and consistent
        if (!ensureNormals(*working_cloud)) {
            is_processing_.store(false);
            return nullptr;
        }

        updateProgress(20);

        UNLOOK_LOG_INFO("Input validation complete: " +
                       std::to_string(last_stats_.input_points) + " points");

        // Configure Open3D verbosity for Poisson
        if (settings.verbose) {
            open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
        }

        // Build octree and solve Poisson equation
        auto octree_start = std::chrono::high_resolution_clock::now();

        UNLOOK_LOG_INFO("Building adaptive octree (depth=" +
                       std::to_string(settings.octree_depth) + ")...");

        updateProgress(30);

        // Execute Poisson reconstruction
        // This is the core operation that:
        // 1. Builds adaptive octree
        // 2. Solves Poisson equation
        // 3. Extracts isosurface
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
        std::vector<double> densities;

        try {
            auto result = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
                *working_cloud,
                settings.octree_depth,
                settings.point_weight,
                settings.scale_factor,
                !settings.use_quadratic_fit  // linear_fit parameter (inverted)
            );

            mesh = std::get<0>(result);
            densities = std::get<1>(result);

        } catch (const std::exception& e) {
            std::stringstream ss;
            ss << "Poisson reconstruction failed: " << e.what();
            last_error_ = ss.str();
            UNLOOK_LOG_ERROR(last_error_);
            is_processing_.store(false);
            return nullptr;
        }

        auto octree_end = std::chrono::high_resolution_clock::now();
        last_stats_.octree_build_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                octree_end - octree_start).count();

        UNLOOK_LOG_INFO("Poisson reconstruction complete: " +
                       std::to_string(mesh->vertices_.size()) + " vertices, " +
                       std::to_string(mesh->triangles_.size()) + " triangles");

        updateProgress(70);

        // Post-processing
        auto postproc_start = std::chrono::high_resolution_clock::now();

        // Apply density-based filtering to remove low-confidence boundary vertices
        if (settings.density_threshold > 0.0 && !densities.empty()) {
            UNLOOK_LOG_INFO("Applying density filtering (threshold=" +
                           std::to_string(settings.density_threshold) + ")...");
            applyDensityFiltering(*mesh, densities, settings.density_threshold);
        }

        updateProgress(80);

        // Remove any remaining degenerate triangles
        mesh->RemoveDegenerateTriangles();
        mesh->RemoveDuplicatedTriangles();
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveUnreferencedVertices();

        updateProgress(85);

        // Ensure consistent triangle orientation
        mesh->ComputeTriangleNormals(true);  // normalized=true
        mesh->ComputeVertexNormals(true);    // normalized=true

        updateProgress(90);

        auto postproc_end = std::chrono::high_resolution_clock::now();
        last_stats_.postprocessing_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                postproc_end - postproc_start).count();

        // Compute comprehensive statistics
        UNLOOK_LOG_INFO("Computing mesh statistics...");
        computeMeshStatistics(*mesh);

        updateProgress(95);

        // Total time
        auto total_end = std::chrono::high_resolution_clock::now();
        last_stats_.reconstruction_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                total_end - total_start).count();

        updateProgress(100);

        // Log final statistics
        UNLOOK_LOG_INFO("Reconstruction completed successfully");
        UNLOOK_LOG_INFO(last_stats_.toString());

        // Validate quality
        if (!last_stats_.is_watertight) {
            UNLOOK_LOG_WARN("Generated mesh is NOT watertight");
        }
        if (!last_stats_.is_manifold) {
            UNLOOK_LOG_WARN("Generated mesh is NOT manifold");
        }

        // Reset verbosity
        if (settings.verbose) {
            open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Warning);
        }

        is_processing_.store(false);
        return mesh;

    } catch (const std::exception& e) {
        std::stringstream ss;
        ss << "Reconstruction exception: " << e.what();
        last_error_ = ss.str();
        UNLOOK_LOG_ERROR(last_error_);
        is_processing_.store(false);
        return nullptr;
    }
}

bool PoissonReconstructor::validateMeshQuality(
    const open3d::geometry::TriangleMesh& mesh,
    bool check_self_intersection) {

    std::lock_guard<std::mutex> lock(reconstruction_mutex_);

    UNLOOK_LOG_INFO("Validating mesh quality...");

    // Basic topology checks
    bool is_watertight = mesh.IsWatertight();
    bool is_manifold = mesh.IsEdgeManifold();
    bool is_vertex_manifold = mesh.IsVertexManifold();
    bool is_orientable = mesh.IsOrientable();

    UNLOOK_LOG_INFO("  Watertight: " + std::string(is_watertight ? "YES" : "NO"));
    UNLOOK_LOG_INFO("  Edge Manifold: " + std::string(is_manifold ? "YES" : "NO"));
    UNLOOK_LOG_INFO("  Vertex Manifold: " + std::string(is_vertex_manifold ? "YES" : "NO"));
    UNLOOK_LOG_INFO("  Orientable: " + std::string(is_orientable ? "YES" : "NO"));

    // Self-intersection check (expensive, optional)
    bool has_self_intersections = false;
    if (check_self_intersection) {
        UNLOOK_LOG_INFO("Checking for self-intersections (this may take a while)...");
        has_self_intersections = mesh.IsSelfIntersecting();
        UNLOOK_LOG_INFO("  Self-intersecting: " + std::string(has_self_intersections ? "YES" : "NO"));
    }

    // Quality is acceptable if mesh is watertight, manifold, and orientable
    bool quality_ok = is_watertight && is_manifold && is_orientable && !has_self_intersections;

    if (quality_ok) {
        UNLOOK_LOG_INFO("Mesh quality validation PASSED");
    } else {
        UNLOOK_LOG_WARN("Mesh quality validation FAILED");
    }

    return quality_ok;
}

#endif  // OPEN3D_ENABLED

// ============================================================================
// Public Methods (Always Available)
// ============================================================================

ReconstructionStats PoissonReconstructor::getLastStats() const {
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);
    return last_stats_;
}

std::string PoissonReconstructor::getLastError() const {
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);
    return last_error_;
}

void PoissonReconstructor::enableARM64Optimizations(bool enable) {
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);
    arm64_optimized_ = enable;
    if (enable) {
        UNLOOK_LOG_INFO("ARM64 optimizations enabled");
    } else {
        UNLOOK_LOG_INFO("ARM64 optimizations disabled");
    }
}

void PoissonReconstructor::setProgressCallback(std::function<void(int)> callback) {
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);
    progress_callback_ = callback;
}

void PoissonReconstructor::clearState() {
    std::lock_guard<std::mutex> lock(reconstruction_mutex_);
    last_stats_ = ReconstructionStats();
    last_error_.clear();
}

} // namespace mesh
} // namespace unlook
