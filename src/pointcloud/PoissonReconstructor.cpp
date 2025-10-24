#include "unlook/pointcloud/PoissonReconstructor.hpp"
#include "unlook/pointcloud/PointCloudProcessor.hpp"  // For OutlierRemovalSettings/Mode definitions
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#endif

namespace unlook {
namespace pointcloud {

// PoissonSettings implementation
bool PoissonSettings::validate() const {
    if (octreeDepth < 5 || octreeDepth > 14) return false;
    if (widthFactor <= 0.0) return false;
    if (scale <= 0.0) return false;
    if (densityQuantile < 0.0 || densityQuantile > 1.0) return false;
    return true;
}

std::string PoissonSettings::toString() const {
    std::stringstream ss;
    ss << "Poisson Reconstruction Settings:\n";
    ss << "  Mode: ";
    switch (mode) {
        case QualityMode::FAST: ss << "FAST"; break;
        case QualityMode::BALANCED: ss << "BALANCED"; break;
        case QualityMode::SHARP: ss << "SHARP"; break;
        case QualityMode::ULTRA: ss << "ULTRA"; break;
    }
    ss << "\n  Octree depth: " << octreeDepth;
    ss << "\n  Width factor: " << widthFactor;
    ss << "\n  Scale: " << scale;
    ss << "\n  Linear fit: " << (linearFit ? "Yes" : "No");
    ss << "\n  Density quantile: " << densityQuantile;
    ss << "\n  Crop low density: " << (cropLowDensity ? "Yes" : "No");
    return ss.str();
}

PoissonSettings PoissonSettings::forFastPreview() {
    PoissonSettings settings;
    settings.mode = QualityMode::FAST;
    settings.octreeDepth = 7;
    settings.linearFit = false;
    settings.densityQuantile = 0.05;
    return settings;
}

PoissonSettings PoissonSettings::forBalancedQuality() {
    PoissonSettings settings;
    settings.mode = QualityMode::BALANCED;
    settings.octreeDepth = 9;
    settings.linearFit = false;
    settings.densityQuantile = 0.01;
    return settings;
}

PoissonSettings PoissonSettings::forSharpFeatures() {
    PoissonSettings settings;
    settings.mode = QualityMode::SHARP;
    settings.octreeDepth = 10;
    settings.linearFit = false;
    settings.densityQuantile = 0.005;
    return settings;
}

PoissonSettings PoissonSettings::forUltraQuality() {
    PoissonSettings settings;
    settings.mode = QualityMode::ULTRA;
    settings.octreeDepth = 11;
    settings.linearFit = true;
    settings.densityQuantile = 0.001;
    return settings;
}

// PoissonResult implementation
std::string PoissonResult::toString() const {
    std::stringstream ss;
    ss << "Poisson Reconstruction Result:\n";
    ss << "  Success: " << (success ? "Yes" : "No") << "\n";
    if (success) {
        ss << "  Input points: " << inputPoints << "\n";
        ss << "  Output vertices: " << outputVertices << "\n";
        ss << "  Output triangles: " << outputTriangles << "\n";
        ss << "  Reconstruction time: " << std::fixed << std::setprecision(2) << reconstructionTimeMs << " ms\n";
        ss << "  Memory usage: " << std::fixed << std::setprecision(1) << memoryUsageMB << " MB\n";
        ss << "  Watertight: " << (isWatertight ? "Yes" : "No") << "\n";
        ss << "  Manifold: " << (isManifold ? "Yes" : "No") << "\n";
        ss << "  Surface deviation: " << std::fixed << std::setprecision(4) << surfaceDeviation << " mm\n";
    } else {
        ss << "  Error: " << errorMessage << "\n";
    }
    return ss.str();
}

// PoissonReconstructor::Impl
class PoissonReconstructor::Impl {
public:
    std::string lastError;
    PoissonResult lastResult;
    std::function<void(int)> progressCallback;
    bool arm64Optimized = false;

    Impl() {
#ifdef __aarch64__
        arm64Optimized = true;
#endif
    }

    void updateProgress(int progress) {
        if (progressCallback) {
            progressCallback(progress);
        }
    }
};

PoissonReconstructor::PoissonReconstructor() : pImpl(std::make_unique<Impl>()) {}
PoissonReconstructor::~PoissonReconstructor() = default;

#ifdef OPEN3D_ENABLED
std::shared_ptr<open3d::geometry::TriangleMesh> PoissonReconstructor::reconstruct(
    const open3d::geometry::PointCloud& pointCloud,
    const PoissonSettings& settings) {

    PoissonResult result;
    return reconstructWithMetrics(pointCloud, settings, result);
}

std::shared_ptr<open3d::geometry::TriangleMesh> PoissonReconstructor::reconstructWithMetrics(
    const open3d::geometry::PointCloud& pointCloud,
    const PoissonSettings& settings,
    PoissonResult& result) {

    if (!settings.validate()) {
        pImpl->lastError = "Invalid Poisson settings";
        result.success = false;
        result.errorMessage = pImpl->lastError;
        return nullptr;
    }

    if (pointCloud.points_.empty()) {
        pImpl->lastError = "Empty point cloud";
        result.success = false;
        result.errorMessage = pImpl->lastError;
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.inputPoints = pointCloud.points_.size();

    try {
        pImpl->updateProgress(10);

        // Copy point cloud to ensure we don't modify input
        auto workingCloud = std::make_shared<open3d::geometry::PointCloud>(pointCloud);

        // Ensure normals are computed
        if (!workingCloud->HasNormals() || settings.computeNormals) {
            std::cout << "[PoissonReconstructor] Computing normals..." << std::endl;
            workingCloud->EstimateNormals(
                open3d::geometry::KDTreeSearchParamHybrid(settings.widthFactor * 10.0, 30)
            );
            pImpl->updateProgress(25);
        }

        // Orient normals consistently
        if (settings.orientNormals) {
            std::cout << "[PoissonReconstructor] Orienting normals..." << std::endl;
            workingCloud->OrientNormalsConsistentTangentPlane(100);
            pImpl->updateProgress(40);
        }

        // Perform Poisson reconstruction
        std::cout << "[PoissonReconstructor] Reconstructing surface (depth=" << settings.octreeDepth << ")..." << std::endl;
        auto [mesh, densities] = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
            *workingCloud,
            settings.octreeDepth,
            settings.widthFactor,
            settings.scale,
            settings.linearFit
        );

        if (!mesh || mesh->vertices_.empty()) {
            pImpl->lastError = "Poisson reconstruction failed to produce mesh";
            result.success = false;
            result.errorMessage = pImpl->lastError;
            return nullptr;
        }

        pImpl->updateProgress(70);

        // Crop low-density regions
        if (settings.cropLowDensity && !densities.empty()) {
            std::cout << "[PoissonReconstructor] Cropping low-density regions..." << std::endl;

            // Calculate density threshold
            std::vector<double> density_vec(densities.begin(), densities.end());
            std::sort(density_vec.begin(), density_vec.end());
            size_t threshold_idx = static_cast<size_t>(density_vec.size() * settings.densityQuantile);
            double density_threshold = density_vec[threshold_idx];

            // Create vertices to remove list
            std::vector<size_t> vertices_to_remove;
            for (size_t i = 0; i < densities.size(); ++i) {
                if (density_vec[i] < density_threshold) {
                    vertices_to_remove.push_back(i);
                }
            }

            // Remove low-density vertices
            mesh = mesh->SelectByIndex(vertices_to_remove, true);
            pImpl->updateProgress(85);
        }

        // Clean up mesh
        mesh->RemoveUnreferencedVertices();
        mesh->RemoveDuplicatedVertices();
        mesh->RemoveDegenerateTriangles();
        mesh->ComputeVertexNormals();

        pImpl->updateProgress(95);

        // Compute result metrics
        result.outputVertices = mesh->vertices_.size();
        result.outputTriangles = mesh->triangles_.size();
        result.isWatertight = mesh->IsWatertight();
        result.isManifold = mesh->IsVertexManifold() && mesh->IsEdgeManifold();

        // Estimate memory usage
        result.memoryUsageMB = (result.outputVertices * sizeof(Eigen::Vector3d) +
                               result.outputTriangles * sizeof(Eigen::Vector3i)) / (1024.0 * 1024.0);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.reconstructionTimeMs = duration.count();

        result.success = true;
        pImpl->lastResult = result;

        std::cout << "[PoissonReconstructor] Reconstruction complete: "
                  << result.outputVertices << " vertices, "
                  << result.outputTriangles << " triangles ("
                  << result.reconstructionTimeMs << " ms)" << std::endl;

        pImpl->updateProgress(100);

        return mesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Poisson reconstruction exception: " + std::string(e.what());
        result.success = false;
        result.errorMessage = pImpl->lastError;
        pImpl->lastResult = result;
        return nullptr;
    }
}

size_t PoissonReconstructor::filterOutliers(
    open3d::geometry::PointCloud& pointCloud,
    const OutlierRemovalSettings& settings) {

    if (!settings.validate()) {
        pImpl->lastError = "Invalid outlier removal settings";
        return 0;
    }

    size_t originalSize = pointCloud.points_.size();

    try {
        if (settings.mode == OutlierRemovalMode::STATISTICAL ||
            settings.mode == OutlierRemovalMode::HYBRID) {
            pImpl->updateProgress(10);

            std::cout << "[PoissonReconstructor] Statistical outlier removal..." << std::endl;
            auto [filtered, indices] = pointCloud.RemoveStatisticalOutliers(
                settings.nb_neighbors,
                settings.std_ratio
            );
            pointCloud = *filtered;
        }

        if (settings.mode == OutlierRemovalMode::RADIUS ||
            settings.mode == OutlierRemovalMode::HYBRID) {
            pImpl->updateProgress(50);

            std::cout << "[PoissonReconstructor] Radius outlier removal..." << std::endl;
            auto [filtered, indices] = pointCloud.RemoveRadiusOutliers(
                settings.min_neighbors,
                settings.radius / 1000.0  // Convert mm to meters
            );
            pointCloud = *filtered;
        }

        pImpl->updateProgress(100);

        size_t removed = originalSize - pointCloud.points_.size();
        std::cout << "[PoissonReconstructor] Removed " << removed << " outlier points" << std::endl;
        return removed;

    } catch (const std::exception& e) {
        pImpl->lastError = "Outlier removal failed: " + std::string(e.what());
        return 0;
    }
}

bool PoissonReconstructor::estimateAndOrientNormals(
    open3d::geometry::PointCloud& pointCloud,
    double searchRadius,
    int maxNeighbors) {

    try {
        pImpl->updateProgress(20);

        // Estimate normals
        std::cout << "[PoissonReconstructor] Estimating normals (radius=" << searchRadius << "mm)..." << std::endl;
        pointCloud.EstimateNormals(
            open3d::geometry::KDTreeSearchParamHybrid(searchRadius / 1000.0, maxNeighbors)
        );

        pImpl->updateProgress(60);

        // Orient normals consistently
        std::cout << "[PoissonReconstructor] Orienting normals..." << std::endl;
        pointCloud.OrientNormalsConsistentTangentPlane(maxNeighbors);

        pImpl->updateProgress(100);

        std::cout << "[PoissonReconstructor] Normal estimation complete" << std::endl;
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Normal estimation failed: " + std::string(e.what());
        return false;
    }
}
#endif

bool PoissonReconstructor::reconstructFallback(
    const std::vector<cv::Vec3f>& points,
    const std::vector<cv::Vec3f>& normals,
    std::vector<cv::Vec3f>& vertices,
    std::vector<cv::Vec3i>& faces,
    const PoissonSettings& settings) {

    pImpl->lastError = "Poisson reconstruction requires Open3D - fallback not implemented";
    return false;
}

void PoissonReconstructor::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string PoissonReconstructor::getLastError() const {
    return pImpl->lastError;
}

PoissonResult PoissonReconstructor::getLastResult() const {
    return pImpl->lastResult;
}

void PoissonReconstructor::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

} // namespace pointcloud
} // namespace unlook
