#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>

// Forward declarations for Open3D types
#ifdef OPEN3D_ENABLED
namespace open3d {
namespace geometry {
class TriangleMesh;
}
}
#endif

namespace unlook {
namespace pointcloud {

/**
 * @brief Mesh cleaning configuration
 *
 * Professional mesh cleanup tools for removing artifacts,
 * small objects, and improving mesh quality for industrial use.
 */
struct MeshCleanerSettings {
    enum class FilterMode {
        KEEP_LARGEST,       // Keep only largest connected component (Artec standard)
        SIZE_THRESHOLD,     // Remove components smaller than threshold
        MANUAL_SELECT       // Manual component selection
    };

    FilterMode mode = FilterMode::KEEP_LARGEST;
    size_t minComponentTriangles = 100;     // Minimum triangles for SIZE_THRESHOLD
    double minComponentArea = 10.0;         // Minimum area in mm² for SIZE_THRESHOLD
    bool removeIsolatedVertices = true;     // Remove unreferenced vertices
    bool removeDuplicateVertices = true;    // Merge duplicate vertices
    bool removeDegenerateTriangles = true;  // Remove zero-area triangles
    bool fixNonManifold = true;             // Fix non-manifold geometry
    double degenerateThreshold = 1e-6;      // Degenerate triangle area threshold

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Mesh simplification configuration
 */
struct SimplificationSettings {
    enum class Mode {
        TRIANGLE_COUNT,     // Target specific triangle count
        REDUCTION_RATIO,    // Reduce by percentage
        ACCURACY            // Simplify while maintaining geometric accuracy
    };

    Mode mode = Mode::ACCURACY;
    size_t targetTriangles = 100000;        // Target triangle count
    double reductionRatio = 0.5;            // Reduction ratio (0.0-1.0)
    double max_geometric_error = 0.01;      // Maximum allowed error in mm (for ACCURACY mode)
    bool preserveBoundaries = true;         // Preserve mesh boundaries
    bool preserveTopology = true;           // Preserve mesh topology
    bool optimizeVertexCache = true;        // Optimize for GPU rendering

    bool validate() const;
    std::string toString() const;

    // Factory methods
    static SimplificationSettings forTargetTriangles(size_t triangles);
    static SimplificationSettings forReductionRatio(double ratio);
    static SimplificationSettings forGeometricAccuracy(double maxErrorMm);
};

/**
 * @brief Mesh cleaning result
 */
struct MeshCleaningResult {
    bool success = false;
    size_t inputVertices = 0;
    size_t inputTriangles = 0;
    size_t outputVertices = 0;
    size_t outputTriangles = 0;
    size_t componentsRemoved = 0;
    size_t verticesRemoved = 0;
    size_t trianglesRemoved = 0;
    double processingTimeMs = 0.0;
    double sizeReduction = 0.0;             // Reduction as percentage
    std::string errorMessage;

    std::string toString() const;
};

/**
 * @brief Professional mesh cleaning and optimization
 *
 * Artec-grade mesh cleanup tools for removing scan artifacts, simplifying meshes,
 * and ensuring production-ready quality for industrial applications.
 *
 * Key features:
 * - Connected component analysis and filtering
 * - Quadric edge collapse simplification
 * - Non-manifold geometry repair
 * - Degenerate triangle removal
 * - Industrial precision maintenance (0.005mm target)
 */
class MeshCleaner {
public:
    MeshCleaner();
    ~MeshCleaner();

#ifdef OPEN3D_ENABLED
    /**
     * @brief Remove small disconnected objects from mesh
     * @param mesh Input mesh (modified in-place)
     * @param settings Cleaning settings
     * @return Cleaning result with metrics
     */
    MeshCleaningResult removeSmallObjects(
        open3d::geometry::TriangleMesh& mesh,
        const MeshCleanerSettings& settings = {});

    /**
     * @brief Simplify mesh while preserving quality
     * @param mesh Input mesh
     * @param settings Simplification settings
     * @return Simplified mesh (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> simplify(
        const open3d::geometry::TriangleMesh& mesh,
        const SimplificationSettings& settings = {});

    /**
     * @brief Clean and optimize mesh for production
     * @param mesh Input mesh (modified in-place)
     * @param cleanSettings Cleaning settings
     * @param simplifySettings Optional simplification settings (nullptr to skip)
     * @return Cleaning result with metrics
     */
    MeshCleaningResult cleanAndOptimize(
        open3d::geometry::TriangleMesh& mesh,
        const MeshCleanerSettings& cleanSettings = {},
        const SimplificationSettings* simplifySettings = nullptr);

    /**
     * @brief Analyze mesh connectivity
     * @param mesh Input mesh
     * @param componentSizes Output: size of each connected component
     * @return Number of connected components
     */
    int analyzeConnectedComponents(
        const open3d::geometry::TriangleMesh& mesh,
        std::vector<size_t>& componentSizes);

    /**
     * @brief Select specific connected component
     * @param mesh Input mesh
     * @param componentIndex Component index (0 = largest)
     * @return Mesh containing only selected component (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> selectComponent(
        const open3d::geometry::TriangleMesh& mesh,
        size_t componentIndex);

    /**
     * @brief Repair non-manifold geometry
     * @param mesh Input mesh (modified in-place)
     * @return true if repair successful
     */
    bool repairNonManifold(open3d::geometry::TriangleMesh& mesh);

    /**
     * @brief Validate mesh quality after cleaning
     * @param mesh Input mesh
     * @param result Output validation results
     * @return true if mesh passes validation
     */
    bool validateMeshQuality(
        const open3d::geometry::TriangleMesh& mesh,
        MeshCleaningResult& result);
#endif

    /**
     * @brief OpenCV-based cleaning (fallback without Open3D)
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param settings Cleaning settings
     * @return Cleaning result
     */
    MeshCleaningResult cleanFallback(
        std::vector<cv::Vec3f>& vertices,
        std::vector<cv::Vec3i>& faces,
        const MeshCleanerSettings& settings = {});

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
     * @brief Get last cleaning result
     * @return Result metrics
     */
    MeshCleaningResult getLastResult() const;

    /**
     * @brief Enable ARM64 optimizations
     * @param enable Enable ARM64 specific optimizations
     */
    void enableARM64Optimizations(bool enable = true);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Helper methods
    void buildConnectivityGraph(
        const std::vector<cv::Vec3i>& faces,
        size_t numVertices,
        std::vector<std::vector<int>>& adjacency);

    int findConnectedComponents(
        const std::vector<cv::Vec3i>& faces,
        size_t numVertices,
        std::vector<int>& componentLabels);

    // Disable copy construction and assignment
    MeshCleaner(const MeshCleaner&) = delete;
    MeshCleaner& operator=(const MeshCleaner&) = delete;
};

} // namespace pointcloud
} // namespace unlook
