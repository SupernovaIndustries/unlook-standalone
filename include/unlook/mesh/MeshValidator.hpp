#pragma once

#include "unlook/core/types.hpp"
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
 * @brief Comprehensive mesh quality metrics for industrial applications
 */
struct MeshQualityMetrics {
    // Basic topology
    size_t numVertices = 0;
    size_t numFaces = 0;
    size_t numEdges = 0;

    // Geometric properties
    double surfaceArea = 0.0;                   // Square millimeters
    double volume = 0.0;                        // Cubic millimeters
    cv::Vec3f boundingBoxMin = {0, 0, 0};
    cv::Vec3f boundingBoxMax = {0, 0, 0};
    cv::Vec3f centroid = {0, 0, 0};

    // Manufacturing readiness
    bool isWatertight = false;
    bool isManifold = false;
    bool isOrientable = false;
    bool hasSelfIntersections = false;

    // Quality assessment
    double aspectRatioMean = 0.0;               // Triangle aspect ratio statistics
    double aspectRatioStd = 0.0;
    double aspectRatioMin = 0.0;
    double aspectRatioMax = 0.0;

    double edgeLengthMean = 0.0;                // Edge length statistics (mm)
    double edgeLengthStd = 0.0;
    double edgeLengthMin = 0.0;
    double edgeLengthMax = 0.0;

    double triangleQualityMean = 0.0;           // Overall triangle quality [0,1]
    double triangleQualityStd = 0.0;

    // Precision assessment
    double surfaceDeviation = 0.0;             // Deviation from input point cloud (mm)
    double precisionScore = 0.0;               // Overall precision score [0,1]

    // Topology analysis
    int numBoundaryEdges = 0;
    int numNonManifoldEdges = 0;
    int numNonManifoldVertices = 0;
    int numIsolatedVertices = 0;
    int numHoles = 0;

    // Connected components
    int numConnectedComponents = 0;
    std::vector<size_t> componentSizes;

    // Performance metrics
    double processingTimeMs = 0.0;
    size_t memoryUsageMB = 0;

    std::string toString() const;
    std::map<std::string, double> toMap() const;
    bool isManufacturingReady() const;
    double getOverallQualityScore() const;
};

/**
 * @brief Mesh validation configuration
 */
struct MeshValidationConfig {
    // Quality thresholds
    double minTriangleQuality = 0.3;           // Minimum acceptable triangle quality
    double maxAspectRatio = 10.0;              // Maximum acceptable aspect ratio
    double minEdgeLength = 0.01;               // Minimum edge length (mm)
    double maxEdgeLength = 100.0;              // Maximum edge length (mm)

    // Precision requirements
    double maxSurfaceDeviation = 0.01;         // Maximum deviation from input (mm)
    double targetPrecision = 0.005;            // Target precision (mm)

    // Manufacturing constraints
    bool requireWatertight = true;             // Require watertight mesh
    bool requireManifold = true;               // Require manifold mesh
    bool allowSelfIntersections = false;       // Allow self-intersections

    // Performance settings
    bool enableDetailedAnalysis = true;        // Enable detailed topology analysis
    bool enablePrecisionCheck = true;          // Enable precision assessment
    bool enablePerformanceMonitoring = true;  // Enable performance monitoring

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Advanced mesh validator for industrial quality control
 *
 * Provides comprehensive mesh quality assessment for manufacturing applications,
 * including watertight validation, topology analysis, and precision measurement.
 * Optimized for 0.005mm precision requirements on ARM64 hardware.
 */
class MeshValidator {
public:
    MeshValidator();
    ~MeshValidator();

    /**
     * @brief Validate mesh quality with comprehensive metrics
     * @param vertices Mesh vertices in millimeters
     * @param faces Triangle faces (vertex indices)
     * @param normals Optional vertex normals
     * @param config Validation configuration
     * @param metrics Output quality metrics
     * @return true if validation successful
     */
    bool validateMesh(const std::vector<cv::Vec3f>& vertices,
                     const std::vector<cv::Vec3i>& faces,
                     const std::vector<cv::Vec3f>& normals,
                     const MeshValidationConfig& config,
                     MeshQualityMetrics& metrics);

#ifdef OPEN3D_ENABLED
    /**
     * @brief Validate Open3D mesh with advanced algorithms
     * @param mesh Open3D triangle mesh
     * @param config Validation configuration
     * @param metrics Output quality metrics
     * @return true if validation successful
     */
    bool validateOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                           const MeshValidationConfig& config,
                           MeshQualityMetrics& metrics);
#endif

    /**
     * @brief Check if mesh is watertight
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @return true if mesh is watertight
     */
    bool isWatertight(const std::vector<cv::Vec3f>& vertices,
                     const std::vector<cv::Vec3i>& faces);

    /**
     * @brief Check if mesh is manifold
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @return true if mesh is manifold
     */
    bool isManifold(const std::vector<cv::Vec3f>& vertices,
                   const std::vector<cv::Vec3i>& faces);

    /**
     * @brief Detect self-intersections in mesh
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param intersectionFaces Output indices of intersecting faces
     * @return true if self-intersections found
     */
    bool detectSelfIntersections(const std::vector<cv::Vec3f>& vertices,
                                const std::vector<cv::Vec3i>& faces,
                                std::vector<int>& intersectionFaces);

    /**
     * @brief Compute triangle quality metrics
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param triangleQualities Output per-triangle quality scores [0,1]
     * @param aspectRatios Output per-triangle aspect ratios
     * @return true if computation successful
     */
    bool computeTriangleQuality(const std::vector<cv::Vec3f>& vertices,
                               const std::vector<cv::Vec3i>& faces,
                               std::vector<double>& triangleQualities,
                               std::vector<double>& aspectRatios);

    /**
     * @brief Find mesh boundaries and holes
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param boundaryEdges Output boundary edge indices
     * @param holeLoops Output hole boundary loops
     * @return Number of holes found
     */
    int findBoundariesAndHoles(const std::vector<cv::Vec3f>& vertices,
                              const std::vector<cv::Vec3i>& faces,
                              std::vector<std::pair<int, int>>& boundaryEdges,
                              std::vector<std::vector<int>>& holeLoops);

    /**
     * @brief Analyze mesh connectivity
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param connectedComponents Output connected component assignments
     * @return Number of connected components
     */
    int analyzeConnectivity(const std::vector<cv::Vec3f>& vertices,
                           const std::vector<cv::Vec3i>& faces,
                           std::vector<int>& connectedComponents);

    /**
     * @brief Assess mesh precision against input point cloud
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param pointCloud Reference point cloud
     * @param maxDeviation Output maximum deviation (mm)
     * @param meanDeviation Output mean deviation (mm)
     * @param stdDeviation Output standard deviation (mm)
     * @return true if assessment successful
     */
    bool assessPrecision(const std::vector<cv::Vec3f>& vertices,
                        const std::vector<cv::Vec3i>& faces,
                        const std::vector<cv::Vec3f>& pointCloud,
                        double& maxDeviation,
                        double& meanDeviation,
                        double& stdDeviation);

    /**
     * @brief Generate detailed validation report
     * @param metrics Quality metrics
     * @param config Validation configuration
     * @return Formatted validation report
     */
    std::string generateValidationReport(const MeshQualityMetrics& metrics,
                                        const MeshValidationConfig& config);

    /**
     * @brief Set validation progress callback
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

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Helper methods
    double computeTriangleArea(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);
    double computeTriangleAspectRatio(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);
    double computeTriangleQuality(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);
    bool areTrianglesIntersecting(const cv::Vec3f& a0, const cv::Vec3f& a1, const cv::Vec3f& a2,
                                 const cv::Vec3f& b0, const cv::Vec3f& b1, const cv::Vec3f& b2);
    double pointToTriangleDistance(const cv::Vec3f& point,
                                  const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);

    // Disable copy construction and assignment
    MeshValidator(const MeshValidator&) = delete;
    MeshValidator& operator=(const MeshValidator&) = delete;
};

/**
 * @brief Utility functions for mesh validation
 */
namespace validation {

/**
 * @brief Check if triangle is degenerate
 * @param v0, v1, v2 Triangle vertices
 * @param threshold Area threshold for degeneracy
 * @return true if triangle is degenerate
 */
bool isDegenerateTriangle(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2,
                         double threshold = 1e-6);

/**
 * @brief Compute triangle normal
 * @param v0, v1, v2 Triangle vertices
 * @return Normalized triangle normal
 */
cv::Vec3f computeTriangleNormal(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2);

/**
 * @brief Check if mesh satisfies manufacturing constraints
 * @param metrics Quality metrics
 * @param config Validation configuration
 * @return true if mesh is manufacturing-ready
 */
bool isManufacturingReady(const MeshQualityMetrics& metrics,
                         const MeshValidationConfig& config);

/**
 * @brief Generate quality score for industrial applications
 * @param metrics Quality metrics
 * @return Overall quality score [0,1]
 */
double computeIndustrialQualityScore(const MeshQualityMetrics& metrics);

} // namespace validation

} // namespace mesh
} // namespace unlook