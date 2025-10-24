#pragma once

#include "unlook/core/types.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

// Forward declarations for Open3D types
#ifdef OPEN3D_ENABLED
namespace open3d {
namespace geometry {
class TriangleMesh;
}
}
#endif

namespace unlook {
namespace mesh {

/**
 * @brief Mesh smoothing configuration
 */
struct MeshSmoothingConfig {
    enum class Algorithm {
        LAPLACIAN,          // Simple Laplacian smoothing
        TAUBIN,            // Taubin smoothing (inflation + deflation)
        BILATERAL,         // Bilateral smoothing (preserves features)
        ANISOTROPIC        // Anisotropic smoothing
    };

    Algorithm algorithm = Algorithm::TAUBIN;
    int iterations = 10;                    // Number of smoothing iterations
    double lambda = 0.5;                    // Smoothing strength [0,1]
    double mu = -0.53;                      // Inflation factor (Taubin only)
    bool preserveBoundaries = true;         // Preserve boundary vertices
    bool preserveFeatures = true;           // Preserve sharp features
    double featureThreshold = 60.0;         // Feature angle threshold (degrees)

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Mesh decimation configuration with advanced simplification modes
 *
 * Matching Artec Studio professional decimation quality with multiple algorithms:
 * - QUADRIC_ERROR: Best quality, preserves geometric features (default)
 * - VERTEX_CLUSTERING: Fast mode, 2x faster (Artec fast mode)
 * - ADAPTIVE: Curvature-based, preserves high-detail areas
 */
struct MeshDecimationConfig {
    enum class Algorithm {
        EDGE_COLLAPSE,      // Basic edge collapse (deprecated, use QUADRIC_ERROR)
        VERTEX_CLUSTERING,  // Fast vertex clustering (Artec fast mode)
        PROGRESSIVE_MESH,   // Progressive mesh decimation
        QUADRIC_ERROR,      // Quadric Error Metric decimation (best quality)
        ADAPTIVE            // Adaptive curvature-based decimation
    };

    /**
     * @brief Simplification modes for different quality/speed tradeoffs
     */
    enum class SimplifyMode {
        TRIANGLE_COUNT,     // Target specific triangle count
        ACCURACY,           // Maintain geometric error < threshold (recommended)
        ADAPTIVE,           // Curvature-based preservation (preserve detail)
        FAST                // Fast vertex clustering (2x faster, Artec fast mode)
    };

    Algorithm algorithm = Algorithm::QUADRIC_ERROR;  // Best quality by default
    SimplifyMode mode = SimplifyMode::ACCURACY;      // Accuracy-driven by default

    // Triangle count targeting
    double targetReduction = 0.5;           // Target reduction ratio [0,1]
    size_t targetTriangles = 0;             // Target triangle count (0 = use ratio)

    // Accuracy control
    double maxError = 0.01;                 // Maximum geometric error (mm) - CRITICAL
    double maxGeometricError = 0.01;        // Alternative name for clarity

    // Quality preservation
    bool preserveBoundaries = true;         // Preserve boundary edges
    bool preserveTopology = true;           // Preserve mesh topology
    double qualityThreshold = 0.3;          // Minimum triangle quality [0,1]
    bool preserveFeatures = true;           // Preserve sharp features
    double featureAngle = 60.0;             // Feature angle threshold (degrees)

    // Adaptive mode parameters
    double curvatureThreshold = 0.1;        // Curvature threshold for adaptive mode
    bool adaptiveSampling = true;           // Enable adaptive sampling

    // Performance options
    bool parallel = true;                   // Enable parallel processing (ARM64)

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Mesh repair configuration
 */
struct MeshRepairConfig {
    bool fillHoles = true;                  // Fill mesh holes
    double maxHoleSize = 50.0;              // Maximum hole diameter (mm)
    bool removeIsolatedComponents = true;   // Remove small isolated components
    size_t minComponentSize = 100;          // Minimum component size (triangles)
    bool fixNonManifoldEdges = true;        // Fix non-manifold edges
    bool fixNonManifoldVertices = true;     // Fix non-manifold vertices
    bool removeDegenerateTriangles = true;  // Remove degenerate triangles
    double degenerateThreshold = 1e-6;      // Degenerate area threshold
    bool improveMeshQuality = true;         // Apply quality improvement
    bool makeWatertight = true;             // Ensure watertight mesh

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Mesh optimization results
 */
struct MeshOptimizationResult {
    bool success = false;
    std::string operation;

    // Before/after metrics
    MeshQualityMetrics beforeMetrics;
    MeshQualityMetrics afterMetrics;

    // Operation statistics
    double processingTimeMs = 0.0;
    size_t memoryUsageMB = 0;
    double qualityImprovement = 0.0;
    double sizeReduction = 0.0;

    std::string toString() const;
};

/**
 * @brief Advanced mesh optimizer for industrial applications
 *
 * Provides comprehensive mesh optimization including smoothing, decimation,
 * repair, and quality improvement for manufacturing-ready meshes.
 * Optimized for 0.005mm precision requirements on ARM64 hardware.
 */
class MeshOptimizer {
public:
    MeshOptimizer();
    ~MeshOptimizer();

    /**
     * @brief Smooth mesh to reduce noise while preserving features
     * @param vertices Input mesh vertices (modified in-place)
     * @param faces Mesh faces
     * @param normals Optional vertex normals (updated if provided)
     * @param config Smoothing configuration
     * @param result Output optimization result
     * @return true if smoothing successful
     */
    bool smoothMesh(std::vector<cv::Vec3f>& vertices,
                   const std::vector<cv::Vec3i>& faces,
                   std::vector<cv::Vec3f>& normals,
                   const MeshSmoothingConfig& config,
                   MeshOptimizationResult& result);

    /**
     * @brief Decimate mesh to reduce triangle count while preserving quality
     * @param vertices Input mesh vertices
     * @param faces Input mesh faces
     * @param normals Input vertex normals
     * @param decimatedVertices Output decimated vertices
     * @param decimatedFaces Output decimated faces
     * @param decimatedNormals Output decimated normals
     * @param config Decimation configuration
     * @param result Output optimization result
     * @return true if decimation successful
     */
    bool decimateMesh(const std::vector<cv::Vec3f>& vertices,
                     const std::vector<cv::Vec3i>& faces,
                     const std::vector<cv::Vec3f>& normals,
                     std::vector<cv::Vec3f>& decimatedVertices,
                     std::vector<cv::Vec3i>& decimatedFaces,
                     std::vector<cv::Vec3f>& decimatedNormals,
                     const MeshDecimationConfig& config,
                     MeshOptimizationResult& result);

    /**
     * @brief Repair mesh to fix topology and quality issues
     * @param vertices Input mesh vertices (modified in-place)
     * @param faces Input mesh faces (modified in-place)
     * @param normals Input vertex normals (updated if provided)
     * @param config Repair configuration
     * @param result Output optimization result
     * @return true if repair successful
     */
    bool repairMesh(std::vector<cv::Vec3f>& vertices,
                   std::vector<cv::Vec3i>& faces,
                   std::vector<cv::Vec3f>& normals,
                   const MeshRepairConfig& config,
                   MeshOptimizationResult& result);

#ifdef OPEN3D_ENABLED
    /**
     * @brief Smooth Open3D mesh with advanced algorithms
     * @param mesh Open3D triangle mesh (modified in-place)
     * @param config Smoothing configuration
     * @param result Output optimization result
     * @return true if smoothing successful
     */
    bool smoothOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                         const MeshSmoothingConfig& config,
                         MeshOptimizationResult& result);

    /**
     * @brief Decimate Open3D mesh with quadric edge collapse
     * @param mesh Input Open3D mesh
     * @param config Decimation configuration
     * @param result Output optimization result
     * @return Decimated mesh (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> decimateOpen3DMesh(
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
        const MeshDecimationConfig& config,
        MeshOptimizationResult& result);

    /**
     * @brief Advanced simplification with multiple quality modes (PRODUCTION)
     *
     * Professional mesh simplification matching Artec Studio quality:
     * - TRIANGLE_COUNT: Target specific triangle count
     * - ACCURACY: Maintain geometric error < threshold (recommended)
     * - ADAPTIVE: Curvature-based preservation (preserve detail)
     * - FAST: Vertex clustering (2x faster, Artec fast mode)
     *
     * @param mesh Input Open3D mesh
     * @param config Simplification configuration with mode selection
     * @param result Output optimization result
     * @return Simplified mesh (nullptr if failed)
     *
     * @note This is the RECOMMENDED method for investor demos
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> simplifyMesh(
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
        const MeshDecimationConfig& config,
        MeshOptimizationResult& result);

    /**
     * @brief Repair Open3D mesh using advanced algorithms
     * @param mesh Open3D triangle mesh (modified in-place)
     * @param config Repair configuration
     * @param result Output optimization result
     * @return true if repair successful
     */
    bool repairOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                         const MeshRepairConfig& config,
                         MeshOptimizationResult& result);
#endif

    /**
     * @brief Optimize mesh for 3D printing applications
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param normals Vertex normals (updated if provided)
     * @param targetPrecision Target precision in millimeters
     * @param result Output optimization result
     * @return true if optimization successful
     */
    bool optimizeFor3DPrinting(std::vector<cv::Vec3f>& vertices,
                              std::vector<cv::Vec3i>& faces,
                              std::vector<cv::Vec3f>& normals,
                              double targetPrecision,
                              MeshOptimizationResult& result);

    /**
     * @brief Optimize mesh for manufacturing applications
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param normals Vertex normals (updated if provided)
     * @param tolerances Manufacturing tolerances
     * @param result Output optimization result
     * @return true if optimization successful
     */
    bool optimizeForManufacturing(std::vector<cv::Vec3f>& vertices,
                                 std::vector<cv::Vec3i>& faces,
                                 std::vector<cv::Vec3f>& normals,
                                 const std::map<std::string, double>& tolerances,
                                 MeshOptimizationResult& result);

    /**
     * @brief Fill holes in mesh using various algorithms
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param maxHoleSize Maximum hole diameter to fill (mm)
     * @param filledHoles Output number of holes filled
     * @return true if hole filling successful
     */
    bool fillHoles(std::vector<cv::Vec3f>& vertices,
                  std::vector<cv::Vec3i>& faces,
                  double maxHoleSize,
                  int& filledHoles);

    /**
     * @brief Remove isolated mesh components
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param minComponentSize Minimum component size (triangles)
     * @param removedComponents Output number of components removed
     * @return true if removal successful
     */
    bool removeIsolatedComponents(std::vector<cv::Vec3f>& vertices,
                                 std::vector<cv::Vec3i>& faces,
                                 size_t minComponentSize,
                                 int& removedComponents);

    /**
     * @brief Fix non-manifold edges and vertices
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param fixedEdges Output number of edges fixed
     * @param fixedVertices Output number of vertices fixed
     * @return true if repair successful
     */
    bool fixNonManifoldGeometry(std::vector<cv::Vec3f>& vertices,
                               std::vector<cv::Vec3i>& faces,
                               int& fixedEdges,
                               int& fixedVertices);

    /**
     * @brief Improve triangle quality through local operations
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param targetQuality Target triangle quality threshold
     * @param improvedTriangles Output number of triangles improved
     * @return true if improvement successful
     */
    bool improveTriangleQuality(std::vector<cv::Vec3f>& vertices,
                               std::vector<cv::Vec3i>& faces,
                               double targetQuality,
                               int& improvedTriangles);

    /**
     * @brief Adaptive mesh refinement for precision requirements
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param normals Vertex normals (updated if provided)
     * @param pointCloud Reference point cloud for precision assessment
     * @param targetPrecision Target precision in millimeters
     * @param result Output optimization result
     * @return true if refinement successful
     */
    bool adaptiveRefinement(std::vector<cv::Vec3f>& vertices,
                           std::vector<cv::Vec3i>& faces,
                           std::vector<cv::Vec3f>& normals,
                           const std::vector<cv::Vec3f>& pointCloud,
                           double targetPrecision,
                           MeshOptimizationResult& result);

    /**
     * @brief Set optimization progress callback
     * @param callback Function called with progress (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    /**
     * @brief Enable ARM64 optimizations
     * @param enable Enable ARM64 specific optimizations
     */
    void enableARM64Optimizations(bool enable = true);

    /**
     * @brief Get optimization performance statistics
     * @return Map of operation name to execution time (ms)
     */
    std::map<std::string, double> getPerformanceStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Helper methods
    bool validateMeshInput(const std::vector<cv::Vec3f>& vertices,
                          const std::vector<cv::Vec3i>& faces);

    void computeMeshQuality(const std::vector<cv::Vec3f>& vertices,
                           const std::vector<cv::Vec3i>& faces,
                           MeshQualityMetrics& metrics);

    // Smoothing algorithms
    bool laplacianSmoothing(std::vector<cv::Vec3f>& vertices,
                           const std::vector<cv::Vec3i>& faces,
                           const MeshSmoothingConfig& config);

    bool taubinSmoothing(std::vector<cv::Vec3f>& vertices,
                        const std::vector<cv::Vec3i>& faces,
                        const MeshSmoothingConfig& config);

    bool bilateralSmoothing(std::vector<cv::Vec3f>& vertices,
                           const std::vector<cv::Vec3i>& faces,
                           const std::vector<cv::Vec3f>& normals,
                           const MeshSmoothingConfig& config);

    // Decimation algorithms
    bool edgeCollapseDecimation(const std::vector<cv::Vec3f>& vertices,
                               const std::vector<cv::Vec3i>& faces,
                               std::vector<cv::Vec3f>& decimatedVertices,
                               std::vector<cv::Vec3i>& decimatedFaces,
                               const MeshDecimationConfig& config);

    bool vertexClusteringDecimation(const std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   std::vector<cv::Vec3f>& decimatedVertices,
                                   std::vector<cv::Vec3i>& decimatedFaces,
                                   const MeshDecimationConfig& config);

    // Hole filling algorithms
    bool fillHoleDelaunay(std::vector<cv::Vec3f>& vertices,
                         std::vector<cv::Vec3i>& faces,
                         const std::vector<int>& holeLoop);

    bool fillHoleAdvancingFront(std::vector<cv::Vec3f>& vertices,
                               std::vector<cv::Vec3i>& faces,
                               const std::vector<int>& holeLoop);

    // Utility methods
    void buildVertexAdjacency(const std::vector<cv::Vec3i>& faces,
                             size_t numVertices,
                             std::vector<std::vector<int>>& adjacency);

    double computeEdgeCollapseCost(const cv::Vec3f& v1, const cv::Vec3f& v2,
                                  const std::vector<cv::Vec3f>& vertices,
                                  const std::vector<cv::Vec3i>& faces);

    cv::Vec3f computeLaplacianCoordinate(int vertexIndex,
                                        const std::vector<cv::Vec3f>& vertices,
                                        const std::vector<std::vector<int>>& adjacency);

    // Disable copy construction and assignment
    MeshOptimizer(const MeshOptimizer&) = delete;
    MeshOptimizer& operator=(const MeshOptimizer&) = delete;
};

/**
 * @brief Utility functions for mesh optimization
 */
namespace optimization {

/**
 * @brief Compute optimal decimation parameters for target precision
 * @param vertices Input mesh vertices
 * @param faces Input mesh faces
 * @param targetPrecision Target precision in millimeters
 * @param targetReduction Output optimal reduction ratio
 * @param maxError Output maximum allowed error
 * @return true if computation successful
 */
bool computeOptimalDecimationParams(const std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   double targetPrecision,
                                   double& targetReduction,
                                   double& maxError);

/**
 * @brief Estimate processing time for mesh operation
 * @param numVertices Number of vertices
 * @param numFaces Number of faces
 * @param operation Operation type
 * @param isARM64 Whether running on ARM64
 * @return Estimated time in milliseconds
 */
double estimateProcessingTime(size_t numVertices, size_t numFaces,
                             const std::string& operation, bool isARM64);

/**
 * @brief Check if mesh optimization is recommended
 * @param metrics Current mesh quality metrics
 * @param targetPrecision Target precision requirements
 * @return true if optimization recommended
 */
bool isOptimizationRecommended(const MeshQualityMetrics& metrics,
                              double targetPrecision);

} // namespace optimization

} // namespace mesh
} // namespace unlook