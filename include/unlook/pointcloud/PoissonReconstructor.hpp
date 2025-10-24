#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>

// Forward declarations for Open3D types
#ifdef OPEN3D_ENABLED
namespace open3d {
namespace geometry {
class PointCloud;
class TriangleMesh;
}
}
#endif

namespace unlook {
namespace pointcloud {

/**
 * @brief Poisson surface reconstruction configuration
 *
 * Implements industry-standard Poisson reconstruction for creating
 * watertight, manifold meshes from oriented point clouds.
 * Based on Artec Studio's reconstruction pipeline.
 */
struct PoissonSettings {
    enum class QualityMode {
        FAST,       // Quick reconstruction, octree depth 7
        BALANCED,   // Good quality, octree depth 9 (default)
        SHARP,      // High quality with sharp features, depth 10
        ULTRA       // Maximum quality, octree depth 11+
    };

    QualityMode mode = QualityMode::BALANCED;
    int octreeDepth = 9;                    // Octree depth (7-12, higher = more detail)
    double widthFactor = 1.1;               // Octree width factor
    double scale = 1.1;                     // Scale factor for reconstruction
    bool linearFit = false;                 // Use linear interpolation
    int minVerticesPerOctreeNode = 1;       // Minimum vertices per node
    double densityQuantile = 0.01;          // Density quantile for cropping
    bool cropLowDensity = true;             // Remove low-density vertices
    bool computeNormals = true;             // Ensure normals are computed
    bool orientNormals = true;              // Orient normals consistently

    bool validate() const;
    std::string toString() const;

    // Factory methods for common use cases
    static PoissonSettings forFastPreview();
    static PoissonSettings forBalancedQuality();
    static PoissonSettings forSharpFeatures();
    static PoissonSettings forUltraQuality();
};

// Forward declaration - full definition in PointCloudProcessor.hpp
struct OutlierRemovalSettings;

/**
 * @brief Poisson reconstruction result metrics
 */
struct PoissonResult {
    bool success = false;
    size_t inputPoints = 0;
    size_t outputVertices = 0;
    size_t outputTriangles = 0;
    double reconstructionTimeMs = 0.0;
    double memoryUsageMB = 0.0;
    bool isWatertight = false;
    bool isManifold = false;
    double surfaceDeviation = 0.0;          // Average distance from points to mesh (mm)
    std::string errorMessage;

    std::string toString() const;
};

/**
 * @brief High-precision Poisson surface reconstructor
 *
 * Professional Poisson reconstruction implementation matching Artec Studio quality.
 * Produces watertight, manifold meshes suitable for industrial applications.
 *
 * Key features:
 * - Adaptive octree-based reconstruction
 * - Normal orientation and consistency
 * - Low-density region cropping
 * - Statistical validation
 * - Industrial precision (0.005mm target)
 */
class PoissonReconstructor {
public:
    PoissonReconstructor();
    ~PoissonReconstructor();

#ifdef OPEN3D_ENABLED
    /**
     * @brief Reconstruct mesh from oriented point cloud
     * @param pointCloud Input point cloud with normals
     * @param settings Poisson reconstruction settings
     * @return Reconstructed triangle mesh (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> reconstruct(
        const open3d::geometry::PointCloud& pointCloud,
        const PoissonSettings& settings = {});

    /**
     * @brief Reconstruct with detailed result metrics
     * @param pointCloud Input point cloud with normals
     * @param settings Poisson reconstruction settings
     * @param result Output reconstruction metrics
     * @return Reconstructed triangle mesh (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> reconstructWithMetrics(
        const open3d::geometry::PointCloud& pointCloud,
        const PoissonSettings& settings,
        PoissonResult& result);

    /**
     * @brief Filter outliers from point cloud
     * @param pointCloud Point cloud to filter (modified in-place)
     * @param settings Outlier removal settings
     * @return Number of points removed
     */
    size_t filterOutliers(
        open3d::geometry::PointCloud& pointCloud,
        const OutlierRemovalSettings& settings);

    /**
     * @brief Estimate and orient normals for point cloud
     * @param pointCloud Point cloud to process (modified in-place)
     * @param searchRadius Search radius for normal estimation (mm)
     * @param maxNeighbors Maximum neighbors for estimation
     * @return true if successful
     */
    bool estimateAndOrientNormals(
        open3d::geometry::PointCloud& pointCloud,
        double searchRadius = 10.0,
        int maxNeighbors = 30);
#endif

    /**
     * @brief OpenCV-based reconstruction (fallback without Open3D)
     * @param points Input points as OpenCV vectors
     * @param normals Input normals
     * @param vertices Output mesh vertices
     * @param faces Output mesh faces
     * @param settings Reconstruction settings
     * @return true if successful
     */
    bool reconstructFallback(
        const std::vector<cv::Vec3f>& points,
        const std::vector<cv::Vec3f>& normals,
        std::vector<cv::Vec3f>& vertices,
        std::vector<cv::Vec3i>& faces,
        const PoissonSettings& settings = {});

    /**
     * @brief Set progress callback for long operations
     * @param callback Function called with progress (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    /**
     * @brief Get last reconstruction result
     * @return Result metrics
     */
    PoissonResult getLastResult() const;

    /**
     * @brief Enable ARM64 optimizations
     * @param enable Enable ARM64 specific optimizations
     */
    void enableARM64Optimizations(bool enable = true);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Disable copy construction and assignment
    PoissonReconstructor(const PoissonReconstructor&) = delete;
    PoissonReconstructor& operator=(const PoissonReconstructor&) = delete;
};

} // namespace pointcloud
} // namespace unlook
