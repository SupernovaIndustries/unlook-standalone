#pragma once

#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/core/types.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <mutex>
#include <atomic>
#include <map>

// Forward declarations for mesh classes
namespace unlook {
namespace mesh {
class MeshValidator;
class MeshOptimizer;
class IndustrialMeshExporter;
struct MeshQualityMetrics;
struct MeshValidationConfig;
struct MeshSmoothingConfig;
struct MeshDecimationConfig;
struct MeshRepairConfig;
struct IndustrialExportConfig;
}
}

// Forward declarations for Open3D types to avoid heavy includes
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
 * @brief Point cloud filtering configuration
 */
struct PointCloudFilterConfig {
    // Statistical outlier removal
    bool enableStatisticalFilter = true;
    int statisticalNeighbors = 20;              // Number of neighbors for statistical analysis
    double statisticalStdRatio = 2.0;           // Standard deviation ratio threshold

    // Voxel downsampling
    bool enableVoxelDownsampling = false;
    double voxelSize = 1.0;                     // Voxel size in millimeters

    // Radius outlier removal
    bool enableRadiusFilter = false;
    double radiusThreshold = 5.0;               // Radius in millimeters
    int minNeighbors = 10;                      // Minimum neighbors within radius

    // Plane removal/segmentation
    bool enablePlaneRemoval = false;
    double planeDistanceThreshold = 1.0;        // Distance threshold in mm
    int ransacIterations = 1000;                // RANSAC iterations
    int ransacMinPoints = 3;                    // Minimum points for plane

    // Normal estimation
    bool computeNormals = true;
    int normalNeighbors = 30;                   // Neighbors for normal computation
    double normalRadius = 10.0;                 // Radius for normal computation (mm)

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Mesh generation configuration
 */
struct MeshGenerationConfig {
    enum class Algorithm {
        POISSON,
        BALL_PIVOTING,
        ALPHA_SHAPES
    };

    Algorithm algorithm = Algorithm::POISSON;

    // Poisson reconstruction parameters
    int poissonDepth = 9;                       // Octree depth for Poisson
    double poissonWidthFactor = 1.1;            // Width factor
    double poissonScale = 1.1;                  // Scale factor
    bool poissonLinearFit = false;              // Use linear fit

    // Ball pivoting parameters
    std::vector<double> ballRadii = {2.0, 4.0, 8.0};  // Ball radii in mm

    // Alpha shapes parameter
    double alphaValue = 0.03;                   // Alpha value

    // Post-processing
    bool removeIsolatedVertices = true;
    bool removeDuplicatedVertices = true;
    bool removeNonManifoldEdges = false;
    bool orientNormals = true;

    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Point cloud quality metrics
 */
struct PointCloudQuality {
    size_t totalPoints = 0;
    size_t validPoints = 0;
    double validRatio = 0.0;
    double density = 0.0;                       // Points per cubic millimeter
    double meanNearestNeighborDistance = 0.0;
    double stdNearestNeighborDistance = 0.0;
    double completeness = 0.0;                  // Coverage ratio
    double outlierRatio = 0.0;
    cv::Vec3f boundingBoxMin = {0, 0, 0};
    cv::Vec3f boundingBoxMax = {0, 0, 0};
    cv::Vec3f centroid = {0, 0, 0};

    std::string toString() const;
};

/**
 * @brief Mesh quality metrics
 */
struct MeshQuality {
    size_t numVertices = 0;
    size_t numFaces = 0;
    size_t numEdges = 0;
    double surfaceArea = 0.0;                   // Square millimeters
    double volume = 0.0;                        // Cubic millimeters
    bool isWatertight = false;
    bool isManifold = false;
    double meanEdgeLength = 0.0;
    double stdEdgeLength = 0.0;
    int numBoundaryEdges = 0;
    int numIsolatedVertices = 0;

    std::string toString() const;
};

/**
 * @brief Export format specification
 */
struct ExportFormat {
    enum class Format {
        PLY_ASCII,
        PLY_BINARY,
        PCD_ASCII,
        PCD_BINARY,
        OBJ,
        XYZ
    };

    Format format = Format::PLY_BINARY;
    bool includeNormals = true;
    bool includeColors = true;
    bool includeConfidence = false;
    bool compressData = true;

    // Metadata
    std::string scannerInfo = "Unlook 3D Scanner";
    std::string timestamp;
    double precisionMm = 0.005;
    std::string calibrationFile;

    std::string getFileExtension() const;
    std::string toString() const;
};

/**
 * @brief Advanced point cloud processor with Open3D integration
 *
 * Extends the basic DepthProcessor with comprehensive point cloud processing,
 * advanced filtering, mesh generation, and multi-format export capabilities.
 * Optimized for industrial precision (0.005mm target) on ARM64 hardware.
 */
class PointCloudProcessor {
public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    /**
     * @brief Initialize with depth processor
     * @param depthProcessor Shared pointer to depth processor
     * @return true if initialization successful
     */
    bool initialize(std::shared_ptr<stereo::DepthProcessor> depthProcessor);

    /**
     * @brief Generate point cloud from depth map with advanced processing
     * @param depthMap Input depth map in millimeters
     * @param colorImage Optional color image for RGB values
     * @param pointCloud Output point cloud (unlook format)
     * @param filterConfig Filtering configuration
     * @return true if generation successful
     */
    bool generatePointCloud(const cv::Mat& depthMap,
                           const cv::Mat& colorImage,
                           stereo::PointCloud& pointCloud,
                           const PointCloudFilterConfig& filterConfig = {});

    /**
     * @brief Generate filtered point cloud with Open3D
     * @param depthMap Input depth map
     * @param colorImage Optional color image
     * @param filterConfig Filtering configuration
     * @return Open3D point cloud (nullptr if failed)
     */
#ifdef OPEN3D_ENABLED
    std::shared_ptr<open3d::geometry::PointCloud> generateOpen3DPointCloud(
        const cv::Mat& depthMap,
        const cv::Mat& colorImage,
        const PointCloudFilterConfig& filterConfig = {});
#endif

    /**
     * @brief Apply advanced filtering to point cloud
     * @param pointCloud Point cloud to filter (modified in-place)
     * @param filterConfig Filtering configuration
     * @return true if filtering successful
     */
    bool applyAdvancedFiltering(stereo::PointCloud& pointCloud,
                               const PointCloudFilterConfig& filterConfig);

#ifdef OPEN3D_ENABLED
    /**
     * @brief Apply Open3D filtering pipeline
     * @param pointCloud Open3D point cloud to filter
     * @param filterConfig Filtering configuration
     * @return true if filtering successful
     */
    bool applyOpen3DFiltering(std::shared_ptr<open3d::geometry::PointCloud> pointCloud,
                             const PointCloudFilterConfig& filterConfig);
#endif

    /**
     * @brief Generate mesh from point cloud
     * @param pointCloud Input point cloud
     * @param meshConfig Mesh generation configuration
     * @param mesh Output triangle mesh data
     * @return true if mesh generation successful
     */
    bool generateMesh(const stereo::PointCloud& pointCloud,
                     const MeshGenerationConfig& meshConfig,
                     std::vector<cv::Vec3f>& vertices,
                     std::vector<cv::Vec3i>& faces,
                     std::vector<cv::Vec3f>& normals);

    // Overload without normals parameter
    bool generateMesh(const stereo::PointCloud& pointCloud,
                     const MeshGenerationConfig& meshConfig,
                     std::vector<cv::Vec3f>& vertices,
                     std::vector<cv::Vec3i>& faces);

#ifdef OPEN3D_ENABLED
    /**
     * @brief Generate mesh using Open3D algorithms
     * @param pointCloud Input Open3D point cloud
     * @param meshConfig Mesh generation configuration
     * @return Open3D triangle mesh (nullptr if failed)
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> generateOpen3DMesh(
        std::shared_ptr<open3d::geometry::PointCloud> pointCloud,
        const MeshGenerationConfig& meshConfig);
#endif

    /**
     * @brief Export point cloud with comprehensive metadata
     * @param pointCloud Point cloud to export
     * @param filename Output filename
     * @param exportFormat Export format specification
     * @return true if export successful
     */
    bool exportPointCloud(const stereo::PointCloud& pointCloud,
                         const std::string& filename,
                         const ExportFormat& exportFormat = {});

    /**
     * @brief Export mesh with industrial precision metadata
     * @param vertices Mesh vertices
     * @param faces Mesh faces
     * @param normals Optional vertex normals
     * @param filename Output filename
     * @param exportFormat Export format specification
     * @return true if export successful
     */
    bool exportMesh(const std::vector<cv::Vec3f>& vertices,
                   const std::vector<cv::Vec3i>& faces,
                   const std::vector<cv::Vec3f>& normals,
                   const std::string& filename,
                   const ExportFormat& exportFormat = {});

    /**
     * @brief Generate optimized mesh for industrial applications
     * @param pointCloud Input point cloud
     * @param meshConfig Mesh generation configuration
     * @param optimizeForManufacturing Apply manufacturing optimizations
     * @param targetPrecision Target precision in millimeters
     * @param vertices Output mesh vertices
     * @param faces Output mesh faces
     * @param normals Output vertex normals
     * @return true if generation successful
     */
    bool generateOptimizedMesh(const stereo::PointCloud& pointCloud,
                              const MeshGenerationConfig& meshConfig,
                              bool optimizeForManufacturing,
                              double targetPrecision,
                              std::vector<cv::Vec3f>& vertices,
                              std::vector<cv::Vec3i>& faces,
                              std::vector<cv::Vec3f>& normals);

    /**
     * @brief Export mesh in industrial formats (STL, OBJ with materials)
     * @param vertices Mesh vertices
     * @param faces Mesh faces
     * @param normals Vertex normals
     * @param filename Output filename
     * @param format Industrial export format
     * @param manufacturingMetadata Additional manufacturing metadata
     * @return true if export successful
     */
    bool exportIndustrialMesh(const std::vector<cv::Vec3f>& vertices,
                             const std::vector<cv::Vec3i>& faces,
                             const std::vector<cv::Vec3f>& normals,
                             const std::string& filename,
                             const std::string& format = "STL_BINARY",
                             const std::map<std::string, std::string>& manufacturingMetadata = {});

    /**
     * @brief Validate mesh quality for manufacturing applications
     * @param vertices Mesh vertices
     * @param faces Mesh faces
     * @param normals Vertex normals
     * @param targetPrecision Target precision requirements
     * @param qualityReport Output detailed quality report
     * @return true if mesh passes validation
     */
    bool validateMeshQuality(const std::vector<cv::Vec3f>& vertices,
                            const std::vector<cv::Vec3i>& faces,
                            const std::vector<cv::Vec3f>& normals,
                            double targetPrecision,
                            std::string& qualityReport);

    /**
     * @brief Optimize mesh for 3D printing applications
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param normals Vertex normals (updated)
     * @param targetPrecision Target precision for printing
     * @param optimizationReport Output optimization report
     * @return true if optimization successful
     */
    bool optimizeFor3DPrinting(std::vector<cv::Vec3f>& vertices,
                              std::vector<cv::Vec3i>& faces,
                              std::vector<cv::Vec3f>& normals,
                              double targetPrecision,
                              std::string& optimizationReport);

    /**
     * @brief Generate manufacturing report with estimates
     * @param vertices Mesh vertices
     * @param faces Mesh faces
     * @param normals Vertex normals
     * @param material Material type (PLA, ABS, PETG)
     * @param printSettings Print settings map
     * @return Comprehensive manufacturing report
     */
    std::string generateManufacturingReport(const std::vector<cv::Vec3f>& vertices,
                                           const std::vector<cv::Vec3i>& faces,
                                           const std::vector<cv::Vec3f>& normals,
                                           const std::string& material = "PLA",
                                           const std::map<std::string, double>& printSettings = {});

    /**
     * @brief Compute comprehensive point cloud quality metrics
     * @param pointCloud Input point cloud
     * @param quality Output quality metrics
     * @return true if computation successful
     */
    bool assessPointCloudQuality(const stereo::PointCloud& pointCloud,
                                PointCloudQuality& quality);

    /**
     * @brief Compute mesh quality metrics
     * @param vertices Mesh vertices
     * @param faces Mesh faces
     * @param quality Output quality metrics
     * @return true if computation successful
     */
    bool assessMeshQuality(const std::vector<cv::Vec3f>& vertices,
                          const std::vector<cv::Vec3i>& faces,
                          MeshQuality& quality);

    /**
     * @brief Coordinate transformation from camera space to world space
     * @param pointCloud Point cloud in camera coordinates
     * @param transformMatrix 4x4 transformation matrix
     * @return true if transformation successful
     */
    bool transformPointCloud(stereo::PointCloud& pointCloud,
                           const cv::Mat& transformMatrix);

    /**
     * @brief Estimate surface normals for point cloud
     * @param pointCloud Point cloud (modified with normals)
     * @param config Configuration for normal estimation
     * @return true if normal estimation successful
     */
    bool estimateNormals(stereo::PointCloud& pointCloud,
                        const PointCloudFilterConfig& config);

    /**
     * @brief Set progress callback for long operations
     * @param callback Function called with progress (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

    /**
     * @brief Cancel ongoing processing
     */
    void cancelProcessing();

    /**
     * @brief Check if processing is in progress
     * @return true if currently processing
     */
    bool isProcessing() const;

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    /**
     * @brief Get processing performance statistics
     * @return Map of operation name to execution time (ms)
     */
    std::map<std::string, double> getPerformanceStats() const;

    /**
     * @brief Enable/disable ARM64 optimizations
     * @param enable Enable ARM64 specific optimizations
     */
    void enableARM64Optimizations(bool enable = true);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Private helper methods
    bool exportPLY(const stereo::PointCloud& pointCloud,
                  const std::string& filename,
                  const ExportFormat& exportFormat);
    bool exportPCD(const stereo::PointCloud& pointCloud,
                  const std::string& filename,
                  const ExportFormat& exportFormat);
    bool exportOBJ(const stereo::PointCloud& pointCloud,
                  const std::string& filename,
                  const ExportFormat& exportFormat);
    bool exportXYZ(const stereo::PointCloud& pointCloud,
                  const std::string& filename,
                  const ExportFormat& exportFormat);

    bool applyFallbackFiltering(stereo::PointCloud& pointCloud,
                               const PointCloudFilterConfig& filterConfig);

    bool generateFallbackMesh(const stereo::PointCloud& pointCloud,
                             const MeshGenerationConfig& meshConfig,
                             std::vector<cv::Vec3f>& vertices,
                             std::vector<cv::Vec3i>& faces,
                             std::vector<cv::Vec3f>& normals);

    void computeNearestNeighborStatistics(const std::vector<cv::Vec3f>& points,
                                         double& meanDistance,
                                         double& stdDistance);

    // Disable copy construction and assignment
    PointCloudProcessor(const PointCloudProcessor&) = delete;
    PointCloudProcessor& operator=(const PointCloudProcessor&) = delete;
};

/**
 * @brief Utility functions for coordinate transformations
 */
namespace transforms {

/**
 * @brief Create transformation matrix from camera to world coordinates
 * @param rotation 3x3 rotation matrix
 * @param translation 3x1 translation vector
 * @return 4x4 transformation matrix
 */
cv::Mat createTransformMatrix(const cv::Mat& rotation, const cv::Mat& translation);

/**
 * @brief Create transformation from Euler angles
 * @param rx, ry, rz Rotation angles in radians
 * @param tx, ty, tz Translation in millimeters
 * @return 4x4 transformation matrix
 */
cv::Mat createTransformFromEuler(double rx, double ry, double rz,
                                double tx, double ty, double tz);

/**
 * @brief Invert transformation matrix
 * @param transform Input transformation matrix
 * @return Inverted transformation matrix
 */
cv::Mat invertTransform(const cv::Mat& transform);

} // namespace transforms

} // namespace pointcloud
} // namespace unlook