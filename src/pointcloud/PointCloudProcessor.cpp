#include "unlook/pointcloud/PointCloudProcessor.hpp"
#include "unlook/pointcloud/PoissonReconstructor.hpp"
#include "unlook/pointcloud/MeshCleaner.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
// Temporarily disabled mesh dependencies due to compilation issues
// #include "unlook/mesh/MeshValidator.hpp"
// #include "unlook/mesh/MeshOptimizer.hpp"
// #include "unlook/mesh/IndustrialMeshExporter.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

// Conditional Open3D includes
#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include <open3d/camera/PinholeCameraIntrinsic.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/io/TriangleMeshIO.h>
#endif

namespace unlook {
namespace pointcloud {

// PointCloudProcessor implementation with Open3D integration
class PointCloudProcessor::Impl {
public:
    std::shared_ptr<stereo::DepthProcessor> depthProcessor;
    std::string lastError;
    std::atomic<bool> processing{false};
    std::atomic<bool> cancelRequested{false};
    std::function<void(int)> progressCallback;
    mutable std::mutex processingMutex;
    std::map<std::string, double> performanceStats;
    bool arm64Optimized = false;

    // Advanced mesh processing components (temporarily disabled)
    // std::unique_ptr<mesh::MeshValidator> meshValidator;
    // std::unique_ptr<mesh::MeshOptimizer> meshOptimizer;
    // std::unique_ptr<mesh::IndustrialMeshExporter> meshExporter;

    Impl() {
        // Enable ARM64 optimizations if available
#ifdef OPEN3D_ARM64_OPTIMIZED
        arm64Optimized = true;
        std::cout << "[PointCloudProcessor] ARM64 optimizations enabled" << std::endl;
#endif

        // Initialize advanced mesh processing components (temporarily disabled)
        // meshValidator = std::make_unique<mesh::MeshValidator>();
        // meshOptimizer = std::make_unique<mesh::MeshOptimizer>();
        // meshExporter = std::make_unique<mesh::IndustrialMeshExporter>();

        // Enable ARM64 optimizations for mesh components (temporarily disabled)
        // if (arm64Optimized) {
        //     meshValidator->enableARM64Optimizations(true);
        //     meshOptimizer->enableARM64Optimizations(true);
        // }
    }

    void updatePerformanceStats(const std::string& operation, double timeMs) {
        std::lock_guard<std::mutex> lock(processingMutex);
        performanceStats[operation] = timeMs;
    }

#ifdef OPEN3D_ENABLED
    std::shared_ptr<open3d::geometry::PointCloud> convertToOpen3D(
        const stereo::PointCloud& unlookPointCloud) {

        auto open3dCloud = std::make_shared<open3d::geometry::PointCloud>();

        // Reserve space for efficiency
        open3dCloud->points_.reserve(unlookPointCloud.points.size());
        open3dCloud->colors_.reserve(unlookPointCloud.points.size());

        // Convert points
        for (const auto& point : unlookPointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                // Convert mm to meters for Open3D (standard units)
                open3dCloud->points_.emplace_back(
                    point.x / 1000.0,
                    point.y / 1000.0,
                    point.z / 1000.0
                );

                // Convert RGB to [0,1] range
                open3dCloud->colors_.emplace_back(
                    point.r / 255.0,
                    point.g / 255.0,
                    point.b / 255.0
                );
            }
        }

        return open3dCloud;
    }

    stereo::PointCloud convertFromOpen3D(
        std::shared_ptr<open3d::geometry::PointCloud> open3dCloud) {

        stereo::PointCloud unlookCloud;

        if (!open3dCloud || open3dCloud->points_.empty()) {
            return unlookCloud;
        }

        unlookCloud.points.reserve(open3dCloud->points_.size());

        bool hasColors = !open3dCloud->colors_.empty();

        for (size_t i = 0; i < open3dCloud->points_.size(); ++i) {
            stereo::Point3D point;

            // Convert meters back to millimeters
            const auto& p3d = open3dCloud->points_[i];
            point.x = static_cast<float>(p3d.x() * 1000.0);
            point.y = static_cast<float>(p3d.y() * 1000.0);
            point.z = static_cast<float>(p3d.z() * 1000.0);

            // Convert colors
            if (hasColors && i < open3dCloud->colors_.size()) {
                const auto& color = open3dCloud->colors_[i];
                point.r = static_cast<uint8_t>(color.x() * 255.0);
                point.g = static_cast<uint8_t>(color.y() * 255.0);
                point.b = static_cast<uint8_t>(color.z() * 255.0);
            } else {
                point.r = point.g = point.b = 255;
            }

            point.confidence = 1.0f;
            unlookCloud.points.push_back(point);
        }

        return unlookCloud;
    }
#endif
};

PointCloudProcessor::PointCloudProcessor() : pImpl(std::make_unique<Impl>()) {}
PointCloudProcessor::~PointCloudProcessor() = default;

bool PointCloudProcessor::initialize(std::shared_ptr<stereo::DepthProcessor> depthProcessor) {
    pImpl->depthProcessor = depthProcessor;

    if (!depthProcessor) {
        pImpl->lastError = "DepthProcessor is null";
        return false;
    }

#ifdef OPEN3D_ENABLED
    std::cout << "[PointCloudProcessor] Initialized with Open3D support" << std::endl;
#else
    std::cout << "[PointCloudProcessor] Initialized with fallback implementation (no Open3D)" << std::endl;
#endif

    return true;
}

bool PointCloudProcessor::generatePointCloud(const cv::Mat& depthMap,
                                            const cv::Mat& colorImage,
                                            stereo::PointCloud& pointCloud,
                                            const PointCloudFilterConfig& filterConfig) {
    if (depthMap.empty()) {
        pImpl->lastError = "Empty depth map";
        return false;
    }

    if (!pImpl->depthProcessor) {
        pImpl->lastError = "DepthProcessor not initialized";
        return false;
    }

    pImpl->processing = true;
    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Generate basic point cloud using DepthProcessor
        if (!pImpl->depthProcessor->generatePointCloud(depthMap, colorImage, pointCloud)) {
            pImpl->lastError = "Failed to generate basic point cloud: " +
                              pImpl->depthProcessor->getLastError();
            pImpl->processing = false;
            return false;
        }

        // Apply advanced filtering if requested
        if (filterConfig.enableStatisticalFilter ||
            filterConfig.enableVoxelDownsampling ||
            filterConfig.enableRadiusFilter ||
            filterConfig.enablePlaneRemoval) {

            if (!applyAdvancedFiltering(pointCloud, filterConfig)) {
                pImpl->lastError = "Advanced filtering failed";
                pImpl->processing = false;
                return false;
            }
        }

        // Estimate normals if requested
        if (filterConfig.computeNormals) {
            estimateNormals(pointCloud, filterConfig);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("generatePointCloud", duration.count());

        pImpl->processing = false;
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Point cloud generation failed: " + std::string(e.what());
        pImpl->processing = false;
        return false;
    }
}

#ifdef OPEN3D_ENABLED
std::shared_ptr<open3d::geometry::PointCloud> PointCloudProcessor::generateOpen3DPointCloud(
    const cv::Mat& depthMap,
    const cv::Mat& colorImage,
    const PointCloudFilterConfig& filterConfig) {

    stereo::PointCloud unlookCloud;
    if (!generatePointCloud(depthMap, colorImage, unlookCloud, filterConfig)) {
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    auto open3dCloud = pImpl->convertToOpen3D(unlookCloud);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    pImpl->updatePerformanceStats("convertToOpen3D", duration.count());

    return open3dCloud;
}
#endif

bool PointCloudProcessor::applyAdvancedFiltering(stereo::PointCloud& pointCloud,
                                                const PointCloudFilterConfig& filterConfig) {
#ifdef OPEN3D_ENABLED
    // Use Open3D for advanced filtering
    auto open3dCloud = pImpl->convertToOpen3D(pointCloud);
    if (!applyOpen3DFiltering(open3dCloud, filterConfig)) {
        return false;
    }

    pointCloud = pImpl->convertFromOpen3D(open3dCloud);
    return true;

#else
    // Fallback implementation using OpenCV and custom algorithms
    return applyFallbackFiltering(pointCloud, filterConfig);
#endif
}

#ifdef OPEN3D_ENABLED
bool PointCloudProcessor::applyOpen3DFiltering(
    std::shared_ptr<open3d::geometry::PointCloud> pointCloud,
    const PointCloudFilterConfig& filterConfig) {

    if (!pointCloud || pointCloud->points_.empty()) {
        pImpl->lastError = "Empty point cloud for filtering";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Statistical outlier removal
        if (filterConfig.enableStatisticalFilter) {
            // DIAGNOSTIC: Count points before filtering
            size_t points_before = pointCloud->points_.size();
            std::cout << "[PointCloudProcessor] BEFORE statistical filter: " << points_before << " points" << std::endl;
            std::cout << "[PointCloudProcessor] Filter parameters: "
                      << filterConfig.statisticalNeighbors << " neighbors, "
                      << filterConfig.statisticalStdRatio << " std ratio" << std::endl;

            auto [filtered, indices] = pointCloud->RemoveStatisticalOutliers(
                filterConfig.statisticalNeighbors,
                filterConfig.statisticalStdRatio);

            // DIAGNOSTIC: Count points after filtering and calculate removal rate
            size_t points_after = filtered->points_.size();
            size_t removed = points_before - points_after;
            double removal_rate = (points_before > 0) ? (100.0 * removed / points_before) : 0.0;

            std::cout << "[PointCloudProcessor] AFTER statistical filter: " << points_after << " points" << std::endl;
            std::cout << "[PointCloudProcessor] Removed: " << removed << " points ("
                      << std::fixed << std::setprecision(3) << removal_rate << "% removal rate)" << std::endl;

            // SAFETY CHECK: Warn if removal rate is dangerously high
            if (removal_rate > 50.0) {
                std::cout << "[PointCloudProcessor] WARNING: Statistical filter removed >"
                          << std::fixed << std::setprecision(1) << removal_rate
                          << "% of points! Filter may be too aggressive." << std::endl;
                std::cout << "[PointCloudProcessor] Consider using more lenient parameters (50 neighbors, 3.0 std ratio)" << std::endl;
            }

            *pointCloud = *filtered;

            if (pImpl->progressCallback) {
                pImpl->progressCallback(25);
            }
        }

        // Radius outlier removal
        if (filterConfig.enableRadiusFilter) {
            auto [filtered, indices] = pointCloud->RemoveRadiusOutliers(
                filterConfig.minNeighbors,
                filterConfig.radiusThreshold / 1000.0);  // Convert mm to meters
            *pointCloud = *filtered;

            if (pImpl->progressCallback) {
                pImpl->progressCallback(50);
            }
        }

        // Voxel downsampling
        if (filterConfig.enableVoxelDownsampling) {
            auto downsampled = pointCloud->VoxelDownSample(
                filterConfig.voxelSize / 1000.0);  // Convert mm to meters
            *pointCloud = *downsampled;

            if (pImpl->progressCallback) {
                pImpl->progressCallback(75);
            }
        }

        // Plane segmentation and removal
        if (filterConfig.enablePlaneRemoval) {
            auto [plane_model, inliers] = pointCloud->SegmentPlane(
                filterConfig.planeDistanceThreshold / 1000.0,  // Convert mm to meters
                filterConfig.ransacMinPoints,
                filterConfig.ransacIterations);

            // Remove plane points
            auto filtered = pointCloud->SelectByIndex(inliers, true);  // true = invert selection
            *pointCloud = *filtered;
        }

        // Estimate normals if needed
        if (filterConfig.computeNormals) {
            pointCloud->EstimateNormals(open3d::geometry::KDTreeSearchParamRadius(
                filterConfig.normalRadius / 1000.0));  // Convert mm to meters
        }

        if (pImpl->progressCallback) {
            pImpl->progressCallback(100);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("open3DFiltering", duration.count());

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D filtering failed: " + std::string(e.what());
        return false;
    }
}
#endif

// Overload without normals parameter
bool PointCloudProcessor::generateMesh(const stereo::PointCloud& pointCloud,
                                      const MeshGenerationConfig& meshConfig,
                                      std::vector<cv::Vec3f>& vertices,
                                      std::vector<cv::Vec3i>& faces) {
    std::vector<cv::Vec3f> normals;
    return generateMesh(pointCloud, meshConfig, vertices, faces, normals);
}

bool PointCloudProcessor::generateMesh(const stereo::PointCloud& pointCloud,
                                      const MeshGenerationConfig& meshConfig,
                                      std::vector<cv::Vec3f>& vertices,
                                      std::vector<cv::Vec3i>& faces,
                                      std::vector<cv::Vec3f>& normals) {
#ifdef OPEN3D_ENABLED
    auto open3dCloud = pImpl->convertToOpen3D(pointCloud);
    auto mesh = generateOpen3DMesh(open3dCloud, meshConfig);

    if (!mesh) {
        return false;
    }

    // Convert Open3D mesh back to OpenCV format
    vertices.clear();
    faces.clear();
    normals.clear();

    vertices.reserve(mesh->vertices_.size());
    for (const auto& vertex : mesh->vertices_) {
        vertices.emplace_back(
            static_cast<float>(vertex.x() * 1000.0),  // Convert to mm
            static_cast<float>(vertex.y() * 1000.0),
            static_cast<float>(vertex.z() * 1000.0)
        );
    }

    faces.reserve(mesh->triangles_.size());
    for (const auto& triangle : mesh->triangles_) {
        faces.emplace_back(
            static_cast<int>(triangle.x()),
            static_cast<int>(triangle.y()),
            static_cast<int>(triangle.z())
        );
    }

    if (mesh->HasVertexNormals()) {
        normals.reserve(mesh->vertex_normals_.size());
        for (const auto& normal : mesh->vertex_normals_) {
            normals.emplace_back(
                static_cast<float>(normal.x()),
                static_cast<float>(normal.y()),
                static_cast<float>(normal.z())
            );
        }
    }

    return true;

#else
    // Fallback: Basic triangulation (Delaunay or similar)
    pImpl->lastError = "Mesh generation requires Open3D - using fallback implementation";
    return generateFallbackMesh(pointCloud, meshConfig, vertices, faces, normals);
#endif
}

#ifdef OPEN3D_ENABLED
std::shared_ptr<open3d::geometry::TriangleMesh> PointCloudProcessor::generateOpen3DMesh(
    std::shared_ptr<open3d::geometry::PointCloud> pointCloud,
    const MeshGenerationConfig& meshConfig) {

    if (!pointCloud || pointCloud->points_.empty()) {
        pImpl->lastError = "Empty point cloud for mesh generation";
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Ensure normals are computed
        if (!pointCloud->HasNormals()) {
            pointCloud->EstimateNormals();
        }

        // Orient normals consistently
        if (meshConfig.orientNormals) {
            pointCloud->OrientNormalsConsistentTangentPlane(100);
        }

        std::shared_ptr<open3d::geometry::TriangleMesh> mesh;

        switch (meshConfig.algorithm) {
            case MeshGenerationConfig::Algorithm::POISSON: {
                auto [poisson_mesh, densities] = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(
                    *pointCloud,
                    meshConfig.poissonDepth,
                    meshConfig.poissonWidthFactor,
                    meshConfig.poissonScale,
                    meshConfig.poissonLinearFit);
                mesh = std::make_shared<open3d::geometry::TriangleMesh>(*poisson_mesh);
                break;
            }

            case MeshGenerationConfig::Algorithm::BALL_PIVOTING: {
                // Convert radii from mm to meters
                std::vector<double> radii_meters;
                for (double radius : meshConfig.ballRadii) {
                    radii_meters.push_back(radius / 1000.0);
                }

                auto ball_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(
                    *pointCloud, radii_meters);
                mesh = std::make_shared<open3d::geometry::TriangleMesh>(*ball_mesh);
                break;
            }

            case MeshGenerationConfig::Algorithm::ALPHA_SHAPES: {
                auto alpha_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(
                    *pointCloud, meshConfig.alphaValue);
                mesh = std::make_shared<open3d::geometry::TriangleMesh>(*alpha_mesh);
                break;
            }
        }

        if (!mesh || mesh->vertices_.empty()) {
            pImpl->lastError = "Mesh generation algorithm failed";
            return nullptr;
        }

        // Post-processing
        if (meshConfig.removeIsolatedVertices) {
            mesh->RemoveUnreferencedVertices();
        }

        if (meshConfig.removeDuplicatedVertices) {
            mesh->RemoveDuplicatedVertices();
        }

        if (meshConfig.removeNonManifoldEdges) {
            mesh->RemoveNonManifoldEdges();
        }

        // Compute vertex normals if not present
        if (!mesh->HasVertexNormals()) {
            mesh->ComputeVertexNormals();
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("meshGeneration", duration.count());

        return mesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D mesh generation failed: " + std::string(e.what());
        return nullptr;
    }
}
#endif

bool PointCloudProcessor::exportPointCloud(const stereo::PointCloud& pointCloud,
                                          const std::string& filename,
                                          const ExportFormat& exportFormat) {
    // CRITICAL DEBUG: Analyze point cloud before export
    std::cout << "[PointCloudProcessor] Export requested: pointCloud.size()=" << pointCloud.points.size() << std::endl;

    if (pointCloud.empty()) {
        std::cout << "[PointCloudProcessor] ERROR: Point cloud is EMPTY (size=0)" << std::endl;
        pImpl->lastError = "Empty point cloud";
        return false;
    }

    // Count valid vs invalid points
    // FIXED: Only count points with positive depth as invalid, allow optical axis points
    int validCount = 0, nanCount = 0, zeroDepthCount = 0;
    for (const auto& pt : pointCloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
            if (pt.z > 0) {  // Valid point with positive depth
                validCount++;
            } else {
                zeroDepthCount++;  // Invalid: zero or negative depth
            }
        } else {
            nanCount++;  // Invalid: NaN coordinates
        }
    }

    std::cout << "[PointCloudProcessor] Point cloud analysis: total=" << pointCloud.points.size()
              << ", valid=" << validCount << ", NaN=" << nanCount << ", zeroDepth=" << zeroDepthCount
              << " (" << (100.0 * validCount / pointCloud.points.size()) << "% valid)" << std::endl;

    if (validCount == 0) {
        std::cout << "[PointCloudProcessor] ERROR: No valid points in point cloud!" << std::endl;
        pImpl->lastError = "Point cloud contains no valid 3D points";
        return false;
    }

    [[maybe_unused]] auto startTime = std::chrono::high_resolution_clock::now();

    try {
        switch (exportFormat.format) {
            case ExportFormat::Format::PLY_ASCII:
            case ExportFormat::Format::PLY_BINARY:
                return exportPLY(pointCloud, filename, exportFormat);

            case ExportFormat::Format::PCD_ASCII:
            case ExportFormat::Format::PCD_BINARY:
                return exportPCD(pointCloud, filename, exportFormat);

            case ExportFormat::Format::OBJ:
                return exportOBJ(pointCloud, filename, exportFormat);

            case ExportFormat::Format::XYZ:
                return exportXYZ(pointCloud, filename, exportFormat);

            default:
                pImpl->lastError = "Unsupported export format";
                return false;
        }

    } catch (const std::exception& e) {
        pImpl->lastError = "Export failed: " + std::string(e.what());
        return false;
    }
}

bool PointCloudProcessor::assessPointCloudQuality(const stereo::PointCloud& pointCloud,
                                                 PointCloudQuality& quality) {
    if (pointCloud.empty()) {
        quality = {};
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        quality.totalPoints = pointCloud.points.size();

        // Count valid points and compute statistics
        std::vector<cv::Vec3f> validPoints;
        validPoints.reserve(pointCloud.points.size());

        cv::Vec3f minBounds(FLT_MAX, FLT_MAX, FLT_MAX);
        cv::Vec3f maxBounds(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        cv::Vec3f centroidSum(0, 0, 0);

        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.z > 0) {  // FIXED: Allow optical axis points, only require positive depth

                validPoints.emplace_back(point.x, point.y, point.z);

                // Update bounds
                minBounds[0] = std::min(minBounds[0], point.x);
                minBounds[1] = std::min(minBounds[1], point.y);
                minBounds[2] = std::min(minBounds[2], point.z);

                maxBounds[0] = std::max(maxBounds[0], point.x);
                maxBounds[1] = std::max(maxBounds[1], point.y);
                maxBounds[2] = std::max(maxBounds[2], point.z);

                centroidSum[0] += point.x;
                centroidSum[1] += point.y;
                centroidSum[2] += point.z;
            }
        }

        quality.validPoints = validPoints.size();
        quality.validRatio = static_cast<double>(quality.validPoints) / quality.totalPoints;
        quality.boundingBoxMin = minBounds;
        quality.boundingBoxMax = maxBounds;

        if (quality.validPoints > 0) {
            quality.centroid[0] = centroidSum[0] / quality.validPoints;
            quality.centroid[1] = centroidSum[1] / quality.validPoints;
            quality.centroid[2] = centroidSum[2] / quality.validPoints;

            // Calculate volume and density
            double volume = (maxBounds[0] - minBounds[0]) *
                           (maxBounds[1] - minBounds[1]) *
                           (maxBounds[2] - minBounds[2]);
            quality.density = quality.validPoints / volume;

            // Calculate mean nearest neighbor distance
            computeNearestNeighborStatistics(validPoints,
                                           quality.meanNearestNeighborDistance,
                                           quality.stdNearestNeighborDistance);
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("qualityAssessment", duration.count());

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Quality assessment failed: " + std::string(e.what());
        return false;
    }
}

void PointCloudProcessor::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

void PointCloudProcessor::cancelProcessing() {
    pImpl->cancelRequested = true;
    pImpl->processing = false;
}

bool PointCloudProcessor::isProcessing() const {
    return pImpl->processing;
}

std::string PointCloudProcessor::getLastError() const {
    return pImpl->lastError;
}

std::map<std::string, double> PointCloudProcessor::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(pImpl->processingMutex);
    return pImpl->performanceStats;
}

void PointCloudProcessor::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

// Helper function implementations
bool PointCloudProcessor::exportPLY(const stereo::PointCloud& pointCloud,
                                   const std::string& filename,
                                   const ExportFormat& exportFormat) {
    std::cout << "[PointCloudProcessor] PLY export to: " << filename << std::endl;

    std::ofstream file(filename);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open PLY file for writing: " + filename;
        return false;
    }

    // Count valid points (CRITICAL: must match what we write)
    // FIXED: Only reject points with invalid Z depth, NOT points near optical axis with X,Y ≈ 0
    size_t validPoints = 0;
    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            point.z > 0) {  // Only require valid positive depth, allow X,Y=0 (optical axis points)
            validPoints++;
        }
    }

    std::cout << "[PointCloudProcessor] PLY export: " << validPoints << " valid points out of "
              << pointCloud.points.size() << " total (" << (100.0 * validPoints / pointCloud.points.size())
              << "%)" << std::endl;

    if (validPoints == 0) {
        file.close();
        std::cout << "[PointCloudProcessor] ERROR: No valid points to export to PLY" << std::endl;
        pImpl->lastError = "No valid points to export";
        return false;
    }

    // Write PLY header
    file << "ply\n";
    file << "format " << (exportFormat.format == ExportFormat::Format::PLY_ASCII ? "ascii" : "binary_little_endian") << " 1.0\n";

    // Add metadata comments
    if (!exportFormat.scannerInfo.empty()) {
        file << "comment Generated by " << exportFormat.scannerInfo << "\n";
    }
    if (!exportFormat.timestamp.empty()) {
        file << "comment Timestamp: " << exportFormat.timestamp << "\n";
    }
    file << "comment Precision: " << exportFormat.precisionMm << "mm\n";
    if (!exportFormat.calibrationFile.empty()) {
        file << "comment Calibration: " << exportFormat.calibrationFile << "\n";
    }

    file << "element vertex " << validPoints << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";

    if (exportFormat.includeColors) {
        file << "property uchar red\n";
        file << "property uchar green\n";
        file << "property uchar blue\n";
    }

    if (exportFormat.includeConfidence) {
        file << "property float confidence\n";
    }

    file << "end_header\n";

    // Write point data (CRITICAL: must write exactly 'validPoints' points)
    size_t pointsWritten = 0;
    if (exportFormat.format == ExportFormat::Format::PLY_ASCII) {
        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.z > 0) {  // FIXED: Allow optical axis points with X,Y ≈ 0
                file << point.x << " " << point.y << " " << point.z;
                pointsWritten++;

                if (exportFormat.includeColors) {
                    file << " " << (int)point.r << " " << (int)point.g << " " << (int)point.b;
                }

                if (exportFormat.includeConfidence) {
                    file << " " << point.confidence;
                }

                file << "\n";
            }
        }
    } else {
        // Binary format
        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.z > 0) {  // FIXED: Allow optical axis points with X,Y ≈ 0
                file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
                pointsWritten++;

                if (exportFormat.includeColors) {
                    file.write(reinterpret_cast<const char*>(&point.r), sizeof(uint8_t));
                    file.write(reinterpret_cast<const char*>(&point.g), sizeof(uint8_t));
                    file.write(reinterpret_cast<const char*>(&point.b), sizeof(uint8_t));
                }

                if (exportFormat.includeConfidence) {
                    file.write(reinterpret_cast<const char*>(&point.confidence), sizeof(float));
                }
            }
        }
    }

    file.close();

    // CRITICAL VERIFICATION: Ensure we wrote the exact number of points declared in header
    if (pointsWritten != validPoints) {
        std::cout << "[PointCloudProcessor] ERROR: PLY header/data mismatch! Header declared "
                  << validPoints << " points but wrote " << pointsWritten << " points" << std::endl;
        pImpl->lastError = "PLY export failed: header/data mismatch";
        return false;
    }

    std::cout << "[PointCloudProcessor] PLY export successful: " << pointsWritten << " points written" << std::endl;
    return true;
}

bool PointCloudProcessor::exportPCD(const stereo::PointCloud& pointCloud,
                                   const std::string& filename,
                                   const ExportFormat& exportFormat) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open PCD file for writing: " + filename;
        return false;
    }

    // Count valid points (allow optical axis points with X,Y ≈ 0)
    size_t validPoints = 0;
    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            point.z > 0) {
            validPoints++;
        }
    }

    // Write PCD header
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";

    if (exportFormat.includeColors) {
        file << "FIELDS x y z rgb\n";
        file << "SIZE 4 4 4 4\n";
        file << "TYPE F F F U\n";
        file << "COUNT 1 1 1 1\n";
    } else {
        file << "FIELDS x y z\n";
        file << "SIZE 4 4 4\n";
        file << "TYPE F F F\n";
        file << "COUNT 1 1 1\n";
    }

    file << "WIDTH " << validPoints << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << validPoints << "\n";
    file << "DATA " << (exportFormat.format == ExportFormat::Format::PCD_ASCII ? "ascii" : "binary") << "\n";

    // Write point data
    if (exportFormat.format == ExportFormat::Format::PCD_ASCII) {
        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.z > 0) {
                file << point.x << " " << point.y << " " << point.z;

                if (exportFormat.includeColors) {
                    uint32_t rgb = ((uint32_t)point.r << 16) | ((uint32_t)point.g << 8) | point.b;
                    file << " " << rgb;
                }

                file << "\n";
            }
        }
    } else {
        // Binary format
        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
                point.z > 0) {
                file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
                file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
                file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));

                if (exportFormat.includeColors) {
                    uint32_t rgb = ((uint32_t)point.r << 16) | ((uint32_t)point.g << 8) | point.b;
                    file.write(reinterpret_cast<const char*>(&rgb), sizeof(uint32_t));
                }
            }
        }
    }

    file.close();
    return true;
}

bool PointCloudProcessor::exportOBJ(const stereo::PointCloud& pointCloud,
                                   const std::string& filename,
                                   const ExportFormat& exportFormat) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open OBJ file for writing: " + filename;
        return false;
    }

    // Write OBJ header
    file << "# Generated by " << exportFormat.scannerInfo << "\n";
    file << "# Precision: " << exportFormat.precisionMm << "mm\n";

    // Write vertices (allow optical axis points)
    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            point.z > 0) {
            file << "v " << point.x << " " << point.y << " " << point.z << "\n";
        }
    }

    file.close();
    return true;
}

bool PointCloudProcessor::exportXYZ(const stereo::PointCloud& pointCloud,
                                   const std::string& filename,
                                   const ExportFormat& exportFormat) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open XYZ file for writing: " + filename;
        return false;
    }

    // Write simple XYZ format (allow optical axis points)
    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            point.z > 0) {
            file << point.x << " " << point.y << " " << point.z;

            if (exportFormat.includeColors) {
                file << " " << (int)point.r << " " << (int)point.g << " " << (int)point.b;
            }

            file << "\n";
        }
    }

    file.close();
    return true;
}

bool PointCloudProcessor::applyFallbackFiltering(stereo::PointCloud& pointCloud,
                                                const PointCloudFilterConfig& filterConfig) {
    // Fallback implementation using OpenCV and custom algorithms

    if (filterConfig.enableStatisticalFilter) {
        // Simple statistical outlier removal
        std::vector<stereo::Point3D> filteredPoints;
        filteredPoints.reserve(pointCloud.points.size());

        // Calculate mean distance for each point to its neighbors
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            const auto& point = pointCloud.points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }

            std::vector<double> distances;
            distances.reserve(filterConfig.statisticalNeighbors);

            // Find nearest neighbors
            for (size_t j = 0; j < pointCloud.points.size() && distances.size() < filterConfig.statisticalNeighbors; ++j) {
                if (i == j) continue;

                const auto& neighbor = pointCloud.points[j];
                if (!std::isfinite(neighbor.x) || !std::isfinite(neighbor.y) || !std::isfinite(neighbor.z)) {
                    continue;
                }

                double dist = std::sqrt(
                    std::pow(point.x - neighbor.x, 2) +
                    std::pow(point.y - neighbor.y, 2) +
                    std::pow(point.z - neighbor.z, 2)
                );
                distances.push_back(dist);
            }

            if (distances.size() >= 3) {
                std::sort(distances.begin(), distances.end());
                double meanDist = std::accumulate(distances.begin(),
                                                 distances.begin() + std::min((int)distances.size(), filterConfig.statisticalNeighbors),
                                                 0.0) / std::min((int)distances.size(), filterConfig.statisticalNeighbors);

                // Simple threshold test
                if (meanDist < filterConfig.statisticalStdRatio * 10.0) {  // Simple heuristic
                    filteredPoints.push_back(point);
                }
            }
        }

        pointCloud.points = std::move(filteredPoints);
    }

    // Voxel downsampling (simple grid-based)
    if (filterConfig.enableVoxelDownsampling) {
        std::map<std::tuple<int, int, int>, std::vector<stereo::Point3D>> voxelGrid;

        // Group points by voxel
        for (const auto& point : pointCloud.points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                int vx = static_cast<int>(std::floor(point.x / filterConfig.voxelSize));
                int vy = static_cast<int>(std::floor(point.y / filterConfig.voxelSize));
                int vz = static_cast<int>(std::floor(point.z / filterConfig.voxelSize));

                voxelGrid[{vx, vy, vz}].push_back(point);
            }
        }

        // Average points in each voxel
        std::vector<stereo::Point3D> downsampledPoints;
        downsampledPoints.reserve(voxelGrid.size());

        for (const auto& [voxel, points] : voxelGrid) {
            if (points.empty()) continue;

            stereo::Point3D avgPoint;
            float sumX = 0, sumY = 0, sumZ = 0;
            int sumR = 0, sumG = 0, sumB = 0;
            float sumConf = 0;

            for (const auto& p : points) {
                sumX += p.x; sumY += p.y; sumZ += p.z;
                sumR += p.r; sumG += p.g; sumB += p.b;
                sumConf += p.confidence;
            }

            avgPoint.x = sumX / points.size();
            avgPoint.y = sumY / points.size();
            avgPoint.z = sumZ / points.size();
            avgPoint.r = static_cast<uint8_t>(sumR / points.size());
            avgPoint.g = static_cast<uint8_t>(sumG / points.size());
            avgPoint.b = static_cast<uint8_t>(sumB / points.size());
            avgPoint.confidence = sumConf / points.size();

            downsampledPoints.push_back(avgPoint);
        }

        pointCloud.points = std::move(downsampledPoints);
    }

    return true;
}

bool PointCloudProcessor::generateFallbackMesh(const stereo::PointCloud& pointCloud,
                                             const MeshGenerationConfig& [[maybe_unused]] meshConfig,
                                             std::vector<cv::Vec3f>& vertices,
                                             std::vector<cv::Vec3i>& faces,
                                             std::vector<cv::Vec3f>& normals) {
    // Simple fallback mesh generation (basic triangulation)
    pImpl->lastError = "Mesh generation requires Open3D - fallback implementation is limited";

    // Convert point cloud to vertices
    vertices.clear();
    vertices.reserve(pointCloud.points.size());

    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
            vertices.emplace_back(point.x, point.y, point.z);
        }
    }

    // Very basic triangulation (this is a placeholder - real implementation would be much more complex)
    faces.clear();
    normals.clear();

    // For now, just return vertices without faces (point cloud mode)
    return true;
}

void PointCloudProcessor::computeNearestNeighborStatistics(const std::vector<cv::Vec3f>& points,
                                                          double& meanDistance,
                                                          double& stdDistance) {
    if (points.size() < 2) {
        meanDistance = stdDistance = 0.0;
        return;
    }

    std::vector<double> nearestDistances;
    nearestDistances.reserve(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        double minDist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j) continue;

            double dist = cv::norm(points[i] - points[j]);
            if (dist < minDist) {
                minDist = dist;
            }
        }

        if (minDist != std::numeric_limits<double>::max()) {
            nearestDistances.push_back(minDist);
        }
    }

    if (!nearestDistances.empty()) {
        meanDistance = std::accumulate(nearestDistances.begin(), nearestDistances.end(), 0.0)
                       / nearestDistances.size();

        double variance = 0.0;
        for (double dist : nearestDistances) {
            variance += std::pow(dist - meanDistance, 2);
        }
        stdDistance = std::sqrt(variance / nearestDistances.size());
    }
}

bool PointCloudProcessor::estimateNormals(stereo::PointCloud& pointCloud,
                                         const PointCloudFilterConfig& config) {
    // Simple normal estimation using local plane fitting
    // This is a fallback implementation - Open3D provides much better algorithms

    for (size_t i = 0; i < pointCloud.points.size(); ++i) {
        auto& point = pointCloud.points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // Find neighbors within radius
        std::vector<cv::Vec3f> neighbors;
        neighbors.reserve(config.normalNeighbors);

        for (size_t j = 0; j < pointCloud.points.size(); ++j) {
            if (i == j) continue;

            const auto& neighbor = pointCloud.points[j];
            if (!std::isfinite(neighbor.x) || !std::isfinite(neighbor.y) || !std::isfinite(neighbor.z)) {
                continue;
            }

            double dist = std::sqrt(
                std::pow(point.x - neighbor.x, 2) +
                std::pow(point.y - neighbor.y, 2) +
                std::pow(point.z - neighbor.z, 2)
            );

            if (dist <= config.normalRadius && neighbors.size() < config.normalNeighbors) {
                neighbors.emplace_back(neighbor.x, neighbor.y, neighbor.z);
            }
        }

        // Simple normal estimation using cross product of first two vectors
        if (neighbors.size() >= 2) {
            cv::Vec3f center(point.x, point.y, point.z);
            cv::Vec3f v1 = neighbors[0] - center;
            cv::Vec3f v2 = neighbors[1] - center;

            cv::Vec3f normal = v1.cross(v2);
            double norm = cv::norm(normal);
            if (norm > 1e-6) {
                normal /= norm;
                // Store normal in confidence field for now (extend Point3D if needed)
                // This is a limitation of the current Point3D structure
            }
        }
    }

    return true;
}

// Artec-grade statistical outlier removal implementation
bool PointCloudProcessor::filterOutliers(stereo::PointCloud& pointCloud,
                                        const OutlierRemovalSettings& settings) {
    if (pointCloud.empty()) {
        pImpl->lastError = "Empty point cloud for outlier removal";
        return false;
    }

    if (!settings.validate()) {
        pImpl->lastError = "Invalid outlier removal settings";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        std::cout << "[PointCloudProcessor] Artec-grade outlier removal: "
                  << pointCloud.points.size() << " points" << std::endl;
        std::cout << settings.toString() << std::endl;

#ifdef OPEN3D_ENABLED
        // Use Open3D for high-performance filtering
        auto open3dCloud = pImpl->convertToOpen3D(pointCloud);
        if (!open3dCloud || open3dCloud->points_.empty()) {
            pImpl->lastError = "Failed to convert point cloud to Open3D format";
            return false;
        }

        size_t points_before = open3dCloud->points_.size();

        // Adaptive parameter adjustment based on point density
        int nb_neighbors = settings.nb_neighbors;
        double std_ratio = settings.std_ratio;

        if (settings.adaptive) {
            double density = estimatePointDensity(open3dCloud);
            std::cout << "[PointCloudProcessor] Point density: " << std::fixed
                      << std::setprecision(2) << density << " points/m³" << std::endl;

            // Adjust parameters based on density
            // High density (>1000 pts/m³) → more neighbors for robust statistics
            // Low density (<500 pts/m³) → fewer neighbors, more lenient threshold
            if (density > 1000.0) {
                nb_neighbors = std::max(30, settings.nb_neighbors);
                std_ratio = settings.std_ratio;  // Keep standard ratio
            } else if (density < 500.0) {
                nb_neighbors = std::max(15, settings.nb_neighbors);
                std_ratio = std::max(2.5, settings.std_ratio);  // More lenient
            }

            std::cout << "[PointCloudProcessor] Adaptive parameters: "
                      << nb_neighbors << " neighbors, "
                      << std::fixed << std::setprecision(1) << std_ratio << " std ratio" << std::endl;
        }

        std::shared_ptr<open3d::geometry::PointCloud> filtered;
        std::vector<size_t> indices;

        switch (settings.mode) {
            case OutlierRemovalMode::STATISTICAL: {
                // Artec-grade statistical outlier removal (K-NN method)
                std::tie(filtered, indices) = open3dCloud->RemoveStatisticalOutliers(
                    nb_neighbors,
                    std_ratio
                );
                break;
            }

            case OutlierRemovalMode::RADIUS: {
                // Radius-based outlier removal
                double radius_meters = settings.radius / 1000.0;  // Convert mm to meters
                std::tie(filtered, indices) = open3dCloud->RemoveRadiusOutliers(
                    settings.min_neighbors,
                    radius_meters
                );
                break;
            }

            case OutlierRemovalMode::HYBRID: {
                // Apply statistical first, then radius
                std::tie(filtered, indices) = open3dCloud->RemoveStatisticalOutliers(
                    nb_neighbors,
                    std_ratio
                );

                // Apply radius filter to the result
                double radius_meters = settings.radius / 1000.0;
                std::tie(filtered, indices) = filtered->RemoveRadiusOutliers(
                    settings.min_neighbors,
                    radius_meters
                );
                break;
            }

            default:
                pImpl->lastError = "Invalid outlier removal mode";
                return false;
        }

        // Statistics and validation
        size_t points_after = filtered->points_.size();
        size_t removed = points_before - points_after;
        double removal_rate = (points_before > 0) ? (100.0 * removed / points_before) : 0.0;

        std::cout << "[PointCloudProcessor] Outlier removal results:" << std::endl;
        std::cout << "  Before: " << points_before << " points" << std::endl;
        std::cout << "  After: " << points_after << " points" << std::endl;
        std::cout << "  Removed: " << removed << " outliers ("
                  << std::fixed << std::setprecision(3) << removal_rate << "%)" << std::endl;

        // Safety check: warn if removal rate is too aggressive
        if (removal_rate > 50.0) {
            std::cout << "[PointCloudProcessor] WARNING: Removed >"
                      << std::fixed << std::setprecision(1) << removal_rate
                      << "% of points! Filter may be too aggressive." << std::endl;
            std::cout << "[PointCloudProcessor] Consider more lenient parameters "
                      << "(nb_neighbors=50, std_ratio=3.0)" << std::endl;
        }

        // Quality check: ensure we didn't remove too much
        if (removal_rate > 95.0) {
            pImpl->lastError = "Outlier removal too aggressive: removed >" +
                              std::to_string(static_cast<int>(removal_rate)) + "% of points";
            return false;
        }

        // Convert back to unlook format
        pointCloud = pImpl->convertFromOpen3D(filtered);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("filterOutliers", duration.count());

        std::cout << "[PointCloudProcessor] Outlier removal completed in "
                  << duration.count() << " ms" << std::endl;

        return true;

#else
        // Fallback implementation without Open3D
        std::cout << "[PointCloudProcessor] WARNING: Open3D not available, using fallback implementation" << std::endl;

        std::vector<stereo::Point3D> filteredPoints;
        filteredPoints.reserve(pointCloud.points.size());

        size_t points_before = pointCloud.points.size();

        // Simple statistical outlier removal (K-NN based)
        for (size_t i = 0; i < pointCloud.points.size(); ++i) {
            const auto& point = pointCloud.points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
                point.z <= 0) {
                continue;
            }

            std::vector<double> distances;
            distances.reserve(settings.nb_neighbors);

            // Find K nearest neighbors
            for (size_t j = 0; j < pointCloud.points.size() && distances.size() < static_cast<size_t>(settings.nb_neighbors); ++j) {
                if (i == j) continue;

                const auto& neighbor = pointCloud.points[j];
                if (!std::isfinite(neighbor.x) || !std::isfinite(neighbor.y) || !std::isfinite(neighbor.z)) {
                    continue;
                }

                double dist = std::sqrt(
                    std::pow(point.x - neighbor.x, 2) +
                    std::pow(point.y - neighbor.y, 2) +
                    std::pow(point.z - neighbor.z, 2)
                );
                distances.push_back(dist);
            }

            if (distances.size() >= 3) {
                std::sort(distances.begin(), distances.end());
                double meanDist = std::accumulate(distances.begin(),
                                                 distances.begin() + std::min(static_cast<int>(distances.size()),
                                                                            settings.nb_neighbors),
                                                 0.0) / std::min(static_cast<int>(distances.size()),
                                                               settings.nb_neighbors);

                // Simple threshold test (heuristic: mean distance < ratio * typical spacing)
                if (meanDist < settings.std_ratio * 10.0) {  // 10mm is typical point spacing
                    filteredPoints.push_back(point);
                }
            }
        }

        size_t points_after = filteredPoints.size();
        size_t removed = points_before - points_after;
        double removal_rate = (points_before > 0) ? (100.0 * removed / points_before) : 0.0;

        std::cout << "[PointCloudProcessor] Fallback outlier removal results:" << std::endl;
        std::cout << "  Before: " << points_before << " points" << std::endl;
        std::cout << "  After: " << points_after << " points" << std::endl;
        std::cout << "  Removed: " << removed << " outliers ("
                  << std::fixed << std::setprecision(3) << removal_rate << "%)" << std::endl;

        pointCloud.points = std::move(filteredPoints);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("filterOutliers_fallback", duration.count());

        return true;
#endif

    } catch (const std::exception& e) {
        pImpl->lastError = "Outlier removal failed: " + std::string(e.what());
        std::cout << "[PointCloudProcessor] ERROR: " << pImpl->lastError << std::endl;
        return false;
    }
}

// Point density estimation implementations
double PointCloudProcessor::estimatePointDensity(const stereo::PointCloud& pointCloud) {
    if (pointCloud.points.size() < 10) {
        return 0.0;
    }

    // Count valid points and compute bounding box
    std::vector<cv::Vec3f> validPoints;
    validPoints.reserve(pointCloud.points.size());

    cv::Vec3f minBounds(FLT_MAX, FLT_MAX, FLT_MAX);
    cv::Vec3f maxBounds(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& point : pointCloud.points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
            point.z > 0) {
            validPoints.emplace_back(point.x, point.y, point.z);

            minBounds[0] = std::min(minBounds[0], point.x);
            minBounds[1] = std::min(minBounds[1], point.y);
            minBounds[2] = std::min(minBounds[2], point.z);

            maxBounds[0] = std::max(maxBounds[0], point.x);
            maxBounds[1] = std::max(maxBounds[1], point.y);
            maxBounds[2] = std::max(maxBounds[2], point.z);
        }
    }

    if (validPoints.size() < 10) {
        return 0.0;
    }

    // Calculate volume (in cubic millimeters)
    double volume = (maxBounds[0] - minBounds[0]) *
                   (maxBounds[1] - minBounds[1]) *
                   (maxBounds[2] - minBounds[2]);

    if (volume <= 0.0) {
        return 0.0;
    }

    // Density = points per cubic millimeter
    return static_cast<double>(validPoints.size()) / volume;
}

#ifdef OPEN3D_ENABLED
double PointCloudProcessor::estimatePointDensity(std::shared_ptr<open3d::geometry::PointCloud> pointCloud) {
    if (!pointCloud || pointCloud->points_.empty() || pointCloud->points_.size() < 10) {
        return 0.0;
    }

    // Compute bounding box
    auto bbox = pointCloud->GetAxisAlignedBoundingBox();
    auto extent = bbox.GetExtent();

    // Volume in cubic meters (Open3D uses meters)
    double volume = extent.x() * extent.y() * extent.z();

    if (volume <= 0.0) {
        return 0.0;
    }

    // Density = points per cubic meter
    return static_cast<double>(pointCloud->points_.size()) / volume;
}
#endif

// OutlierRemovalSettings implementation
std::string OutlierRemovalSettings::toString() const {
    std::stringstream ss;
    ss << "Outlier Removal Settings:\n";
    ss << "  Mode: ";
    switch (mode) {
        case OutlierRemovalMode::STATISTICAL:
            ss << "Statistical (Artec K-NN)";
            break;
        case OutlierRemovalMode::RADIUS:
            ss << "Radius-based";
            break;
        case OutlierRemovalMode::HYBRID:
            ss << "Hybrid (Statistical + Radius)";
            break;
    }
    ss << "\n";
    ss << "  K Neighbors: " << nb_neighbors << "\n";
    ss << "  Std Ratio: " << std_ratio << "\n";
    ss << "  Radius: " << radius << " mm\n";
    ss << "  Min Neighbors (radius): " << min_neighbors << "\n";
    ss << "  Adaptive: " << (adaptive ? "Yes" : "No") << "\n";
    return ss.str();
}

// Configuration validation implementations
bool PointCloudFilterConfig::validate() const {
    if (statisticalNeighbors <= 0 || statisticalStdRatio <= 0) return false;
    if (voxelSize <= 0) return false;
    if (radiusThreshold <= 0 || minNeighbors <= 0) return false;
    if (planeDistanceThreshold <= 0 || ransacIterations <= 0) return false;
    if (normalNeighbors <= 0 || normalRadius <= 0) return false;
    return true;
}

std::string PointCloudFilterConfig::toString() const {
    std::stringstream ss;
    ss << "Point Cloud Filter Configuration:\n";
    ss << "  Statistical Filter: " << (enableStatisticalFilter ? "Yes" : "No");
    if (enableStatisticalFilter) {
        ss << " (neighbors=" << statisticalNeighbors
           << ", std_ratio=" << statisticalStdRatio << ")";
    }
    ss << "\n";
    ss << "  Voxel Downsampling: " << (enableVoxelDownsampling ? "Yes" : "No");
    if (enableVoxelDownsampling) {
        ss << " (voxel_size=" << voxelSize << "mm)";
    }
    ss << "\n";
    ss << "  Radius Filter: " << (enableRadiusFilter ? "Yes" : "No");
    if (enableRadiusFilter) {
        ss << " (radius=" << radiusThreshold << "mm, min_neighbors=" << minNeighbors << ")";
    }
    ss << "\n";
    ss << "  Plane Removal: " << (enablePlaneRemoval ? "Yes" : "No");
    if (enablePlaneRemoval) {
        ss << " (threshold=" << planeDistanceThreshold << "mm)";
    }
    ss << "\n";
    ss << "  Compute Normals: " << (computeNormals ? "Yes" : "No");
    if (computeNormals) {
        ss << " (neighbors=" << normalNeighbors
           << ", radius=" << normalRadius << "mm)";
    }
    ss << "\n";
    return ss.str();
}

bool MeshGenerationConfig::validate() const {
    if (poissonDepth <= 0 || poissonWidthFactor <= 0 || poissonScale <= 0) return false;
    if (ballRadii.empty()) return false;
    for (double radius : ballRadii) {
        if (radius <= 0) return false;
    }
    if (alphaValue <= 0) return false;
    return true;
}

std::string MeshGenerationConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Generation Configuration:\n";
    ss << "  Algorithm: ";
    switch (algorithm) {
        case Algorithm::POISSON: ss << "Poisson Surface Reconstruction"; break;
        case Algorithm::BALL_PIVOTING: ss << "Ball Pivoting"; break;
        case Algorithm::ALPHA_SHAPES: ss << "Alpha Shapes"; break;
    }
    ss << "\n";
    return ss.str();
}

std::string ExportFormat::getFileExtension() const {
    switch (format) {
        case Format::PLY_ASCII:
        case Format::PLY_BINARY: return ".ply";
        case Format::PCD_ASCII:
        case Format::PCD_BINARY: return ".pcd";
        case Format::OBJ: return ".obj";
        case Format::XYZ: return ".xyz";
        default: return ".dat";
    }
}

// Namespace transforms implementation
namespace transforms {

cv::Mat createTransformMatrix(const cv::Mat& rotation, const cv::Mat& translation) {
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);

    // Copy rotation (3x3 -> top-left of 4x4)
    rotation.copyTo(transform(cv::Rect(0, 0, 3, 3)));

    // Copy translation (3x1 -> top-right of 4x4)
    translation.copyTo(transform(cv::Rect(3, 0, 1, 3)));

    return transform;
}

cv::Mat createTransformFromEuler(double rx, double ry, double rz,
                                double tx, double ty, double tz) {
    // Create rotation matrix from Euler angles
    cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(rx), -sin(rx),
        0, sin(rx), cos(rx));

    cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
        cos(ry), 0, sin(ry),
        0, 1, 0,
        -sin(ry), 0, cos(ry));

    cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
        cos(rz), -sin(rz), 0,
        sin(rz), cos(rz), 0,
        0, 0, 1);

    cv::Mat rotation = R_z * R_y * R_x;
    cv::Mat translation = (cv::Mat_<double>(3, 1) << tx, ty, tz);

    return createTransformMatrix(rotation, translation);
}

cv::Mat invertTransform(const cv::Mat& transform) {
    cv::Mat invTransform = cv::Mat::eye(4, 4, CV_64F);

    // R^T
    cv::Mat R = transform(cv::Rect(0, 0, 3, 3));
    cv::Mat R_inv;
    cv::transpose(R, R_inv);
    R_inv.copyTo(invTransform(cv::Rect(0, 0, 3, 3)));

    // -R^T * t
    cv::Mat t = transform(cv::Rect(3, 0, 1, 3));
    cv::Mat t_inv = -R_inv * t;
    t_inv.copyTo(invTransform(cv::Rect(3, 0, 1, 3)));

    return invTransform;
}

} // namespace transforms

// Advanced mesh generation and export implementations
bool PointCloudProcessor::generateOptimizedMesh(const stereo::PointCloud& pointCloud,
                                               const MeshGenerationConfig& meshConfig,
                                               bool optimizeForManufacturing,
                                               double [[maybe_unused]] targetPrecision,
                                               std::vector<cv::Vec3f>& vertices,
                                               std::vector<cv::Vec3i>& faces,
                                               std::vector<cv::Vec3f>& normals) {
    if (pointCloud.empty()) {
        pImpl->lastError = "Empty point cloud for mesh generation";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Generate basic mesh
        if (!generateMesh(pointCloud, meshConfig, vertices, faces, normals)) {
            pImpl->lastError = "Failed to generate basic mesh: " + pImpl->lastError;
            return false;
        }

        if (optimizeForManufacturing) {
            // Manufacturing optimization temporarily disabled
            pImpl->lastError = "Manufacturing optimization temporarily disabled - mesh module not built";
            return false;

            // // Apply manufacturing optimizations
            // mesh::MeshOptimizationResult result;
            // if (!pImpl->meshOptimizer->optimizeForManufacturing(vertices, faces, normals,
            //                                                     {{"precision", targetPrecision}}, result)) {
            //     pImpl->lastError = "Manufacturing optimization failed: " + pImpl->meshOptimizer->getLastError();
            //     return false;
            // }
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        pImpl->updatePerformanceStats("generateOptimizedMesh", duration.count());

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Optimized mesh generation failed: " + std::string(e.what());
        return false;
    }
}

bool PointCloudProcessor::exportIndustrialMesh(const std::vector<cv::Vec3f>& vertices,
                                              const std::vector<cv::Vec3i>& faces,
                                              const std::vector<cv::Vec3f>& normals,
                                              const std::string& filename,
                                              const std::string& format,
                                              const std::map<std::string, std::string>& manufacturingMetadata) {
    if (vertices.empty() || faces.empty()) {
        pImpl->lastError = "Empty mesh data for industrial export";
        return false;
    }

    try {
        // Mesh export temporarily disabled
        pImpl->lastError = "Industrial mesh export temporarily disabled - mesh module not built";
        return false;

        /* ALL MESH CODE TEMPORARILY DISABLED
        // mesh::IndustrialExportConfig config;

        // // Set format
        if (format == "STL_ASCII") {
            config.format = mesh::IndustrialExportConfig::Format::STL_ASCII;
        } else if (format == "STL_BINARY") {
            config.format = mesh::IndustrialExportConfig::Format::STL_BINARY;
        } else if (format == "OBJ_WITH_MATERIALS") {
            config.format = mesh::IndustrialExportConfig::Format::OBJ_WITH_MATERIALS;
        } else if (format == "PLY_INDUSTRIAL") {
            config.format = mesh::IndustrialExportConfig::Format::PLY_INDUSTRIAL;
        } else {
            config.format = mesh::IndustrialExportConfig::Format::STL_BINARY; // Default
        }

        // Apply manufacturing metadata
        auto it = manufacturingMetadata.find("partName");
        if (it != manufacturingMetadata.end()) config.partName = it->second;

        it = manufacturingMetadata.find("material");
        if (it != manufacturingMetadata.end()) config.material = it->second;

        it = manufacturingMetadata.find("operator");
        if (it != manufacturingMetadata.end()) config.operator_ = it->second;

        // Set current timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        config.scanDate = ss.str();

        // Set high precision requirements
        config.coordinatePrecision = 0.005; // 5 micron precision
        config.requireWatertight = true;
        config.validateBeforeExport = true;

        // Mesh export temporarily disabled
        pImpl->lastError = "Industrial mesh export temporarily disabled - mesh module not built";
        return false;

        // mesh::ExportResult result;
        // if (!pImpl->meshExporter->exportMesh(vertices, faces, normals, filename, config, result)) {
        //     pImpl->lastError = "Industrial mesh export failed: " + pImpl->meshExporter->getLastError();
        //     return false;
        // }
        */ // END OF DISABLED MESH CODE

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Industrial export failed: " + std::string(e.what());
        return false;
    }
}

bool PointCloudProcessor::validateMeshQuality(const std::vector<cv::Vec3f>& vertices,
                                             const std::vector<cv::Vec3i>& faces,
                                             const std::vector<cv::Vec3f>& normals,
                                             double targetPrecision,
                                             std::string& qualityReport) {
    if (vertices.empty() || faces.empty()) {
        pImpl->lastError = "Empty mesh data for validation";
        return false;
    }

    try {
        // mesh::MeshValidationConfig config;
        // config.targetPrecision = targetPrecision;
        // config.requireWatertight = true;
        // config.requireManifold = true;
        // config.minTriangleQuality = 0.3;

        // Mesh validation temporarily disabled
        qualityReport = "Mesh validation temporarily disabled - mesh module not built";

        // mesh::MeshQualityMetrics metrics;
        // bool isValid = pImpl->meshValidator->validateMesh(vertices, faces, normals, config, metrics);
        // qualityReport = pImpl->meshValidator->generateValidationReport(metrics, config);

        bool isValid = false;

        return isValid;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh validation failed: " + std::string(e.what());
        qualityReport = "Validation failed: " + std::string(e.what());
        return false;
    }
}

bool PointCloudProcessor::optimizeFor3DPrinting(std::vector<cv::Vec3f>& vertices,
                                               std::vector<cv::Vec3i>& faces,
                                               std::vector<cv::Vec3f>& normals,
                                               double targetPrecision,
                                               std::string& optimizationReport) {
    if (vertices.empty() || faces.empty()) {
        pImpl->lastError = "Empty mesh data for 3D printing optimization";
        return false;
    }

    try {
        // 3D printing optimization temporarily disabled
        pImpl->lastError = "3D printing optimization temporarily disabled - mesh module not built";
        optimizationReport = "Optimization temporarily disabled - mesh module not built";
        return false;

        // mesh::MeshOptimizationResult result;
        // if (!pImpl->meshOptimizer->optimizeFor3DPrinting(vertices, faces, normals, targetPrecision, result)) {
        //     pImpl->lastError = "3D printing optimization failed: " + pImpl->meshOptimizer->getLastError();
        //     optimizationReport = "Optimization failed: " + pImpl->meshOptimizer->getLastError();
        //     return false;
        // }

        // optimizationReport = result.toString();
        // return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "3D printing optimization failed: " + std::string(e.what());
        optimizationReport = "Optimization failed: " + std::string(e.what());
        return false;
    }
}

std::string PointCloudProcessor::generateManufacturingReport(const std::vector<cv::Vec3f>& vertices,
                                                            const std::vector<cv::Vec3i>& faces,
                                                            const std::vector<cv::Vec3f>& normals,
                                                            const std::string& material,
                                                            const std::map<std::string, double>& printSettings) {
    if (vertices.empty() || faces.empty()) {
        return "ERROR: Empty mesh data for manufacturing report";
    }

    try {
        // Manufacturing report temporarily disabled
        return "Manufacturing report temporarily disabled - mesh module not built";

        /* ALL MESH CODE TEMPORARILY DISABLED
        // mesh::IndustrialExportConfig config;
        // config.material = material;

        // Apply print settings
        auto it = printSettings.find("layerHeight");
        if (it != printSettings.end()) {
            // Store layer height for reporting (config doesn't have this field)
        }

        config.partName = "Scanned Part";
        config.operator_ = "Unlook Scanner";

        // Set current timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
        config.scanDate = ss.str();

        // Manufacturing report temporarily disabled
        return "Manufacturing report temporarily disabled - mesh module not built";

        // return pImpl->meshExporter->generateManufacturingReport(vertices, faces, normals, config);
        */ // END OF DISABLED MESH CODE

    } catch (const std::exception& e) {
        return "ERROR: Manufacturing report generation failed: " + std::string(e.what());
    }
}

// Temporary stub implementations for mesh functions (mesh module disabled)
bool PointCloudProcessor::assessMeshQuality(const std::vector<cv::Vec3f>& vertices,
                                           const std::vector<cv::Vec3i>& faces,
                                           MeshQuality& quality) {
    // Mesh module temporarily disabled - return basic quality assessment
    pImpl->lastError = "Mesh assessment temporarily disabled - mesh module not built";

    quality.isWatertight = false;
    quality.isManifold = false;
    quality.numVertices = vertices.size();
    quality.numFaces = faces.size();
    quality.numEdges = 0;
    quality.surfaceArea = 0.0;
    quality.volume = 0.0;
    quality.meanEdgeLength = 0.0;
    quality.stdEdgeLength = 0.0;
    quality.numBoundaryEdges = 0;

    return false; // Indicate that full assessment is not available
}

bool PointCloudProcessor::exportMesh(const std::vector<cv::Vec3f>& [[maybe_unused]] vertices,
                                    const std::vector<cv::Vec3i>& [[maybe_unused]] faces,
                                    const std::vector<cv::Vec3f>& [[maybe_unused]] normals,
                                    const std::string& [[maybe_unused]] filename,
                                    const ExportFormat& [[maybe_unused]] exportFormat) {
    // Mesh module temporarily disabled - cannot export mesh
    pImpl->lastError = "Mesh export temporarily disabled - mesh module not built";
    return false;
}

#ifdef OPEN3D_ENABLED
// Complete Artec-grade processing pipeline implementation
std::shared_ptr<open3d::geometry::TriangleMesh> PointCloudProcessor::processCompletePipeline(
    const open3d::geometry::PointCloud& pointCloud) {

    // Use Artec standard settings
    OutlierRemovalSettings outlierSettings;
    outlierSettings.mode = OutlierRemovalMode::STATISTICAL;
    outlierSettings.nb_neighbors = 20;
    outlierSettings.std_ratio = 2.0;

    PoissonSettings poissonSettings = PoissonSettings::forSharpFeatures();

    MeshCleanerSettings cleanSettings;
    cleanSettings.mode = MeshCleanerSettings::FilterMode::KEEP_LARGEST;

    SimplificationSettings simplifySettings = SimplificationSettings::forGeometricAccuracy(0.01);

    return processCompletePipeline(pointCloud, outlierSettings, poissonSettings, cleanSettings, &simplifySettings);
}

std::shared_ptr<open3d::geometry::TriangleMesh> PointCloudProcessor::processCompletePipeline(
    const open3d::geometry::PointCloud& pointCloud,
    const OutlierRemovalSettings& outlierSettings,
    const PoissonSettings& poissonSettings,
    const MeshCleanerSettings& cleanSettings,
    const SimplificationSettings* simplifySettings) {

    if (pointCloud.points_.empty()) {
        pImpl->lastError = "Empty input point cloud for pipeline";
        return nullptr;
    }

    auto overallStartTime = std::chrono::high_resolution_clock::now();

    try {
        std::cout << "\n========== ARTEC-GRADE PROCESSING PIPELINE ==========\n" << std::endl;
        std::cout << "[Pipeline] Input: " << pointCloud.points_.size() << " points" << std::endl;

        // STEP 1: Statistical Outlier Removal (Artec standard)
        std::cout << "\n[Pipeline] STEP 1: Statistical outlier removal..." << std::endl;
        if (pImpl->progressCallback) pImpl->progressCallback(5);

        auto workingCloud = std::make_shared<open3d::geometry::PointCloud>(pointCloud);

        PoissonReconstructor poissonReconstructor;
        poissonReconstructor.setProgressCallback(pImpl->progressCallback);

        size_t removed = poissonReconstructor.filterOutliers(*workingCloud, outlierSettings);
        std::cout << "[Pipeline] Removed " << removed << " outlier points ("
                  << (100.0 * removed / pointCloud.points_.size()) << "%)" << std::endl;

        if (pImpl->progressCallback) pImpl->progressCallback(15);

        // STEP 2: Poisson Surface Reconstruction (Artec-grade)
        std::cout << "\n[Pipeline] STEP 2: Poisson surface reconstruction..." << std::endl;
        if (pImpl->progressCallback) pImpl->progressCallback(20);

        auto mesh = poissonReconstructor.reconstruct(*workingCloud, poissonSettings);
        if (!mesh) {
            pImpl->lastError = "Poisson reconstruction failed: " + poissonReconstructor.getLastError();
            return nullptr;
        }

        PoissonResult poissonResult = poissonReconstructor.getLastResult();
        std::cout << "[Pipeline] Reconstructed: " << poissonResult.outputVertices << " vertices, "
                  << poissonResult.outputTriangles << " triangles" << std::endl;
        std::cout << "[Pipeline] Watertight: " << (poissonResult.isWatertight ? "Yes" : "No")
                  << ", Manifold: " << (poissonResult.isManifold ? "Yes" : "No") << std::endl;

        if (pImpl->progressCallback) pImpl->progressCallback(50);

        // STEP 3: Remove Small Objects (Artec cleanup)
        std::cout << "\n[Pipeline] STEP 3: Mesh cleaning..." << std::endl;
        if (pImpl->progressCallback) pImpl->progressCallback(55);

        MeshCleaner meshCleaner;
        meshCleaner.setProgressCallback(pImpl->progressCallback);

        auto cleanResult = meshCleaner.removeSmallObjects(*mesh, cleanSettings);
        if (!cleanResult.success) {
            pImpl->lastError = "Mesh cleaning failed: " + cleanResult.errorMessage;
            return nullptr;
        }

        std::cout << "[Pipeline] Cleaned: removed " << cleanResult.componentsRemoved << " components, "
                  << cleanResult.trianglesRemoved << " triangles ("
                  << cleanResult.sizeReduction << "% reduction)" << std::endl;

        if (pImpl->progressCallback) pImpl->progressCallback(70);

        // STEP 4: Mesh Simplification (Artec optimization, optional)
        if (simplifySettings != nullptr) {
            std::cout << "\n[Pipeline] STEP 4: Mesh simplification..." << std::endl;
            if (pImpl->progressCallback) pImpl->progressCallback(75);

            size_t trianglesBefore = mesh->triangles_.size();
            auto simplified = meshCleaner.simplify(*mesh, *simplifySettings);
            if (simplified) {
                mesh = simplified;
                size_t trianglesAfter = mesh->triangles_.size();
                double reduction = 100.0 * (trianglesBefore - trianglesAfter) / trianglesBefore;
                std::cout << "[Pipeline] Simplified: " << trianglesBefore << " -> " << trianglesAfter
                          << " triangles (" << std::fixed << std::setprecision(1) << reduction << "% reduction)" << std::endl;
            } else {
                std::cout << "[Pipeline] Warning: Simplification failed, using unsimplified mesh" << std::endl;
            }
        } else {
            std::cout << "\n[Pipeline] STEP 4: Mesh simplification (skipped)" << std::endl;
        }

        if (pImpl->progressCallback) pImpl->progressCallback(90);

        // STEP 5: Final Quality Validation
        std::cout << "\n[Pipeline] STEP 5: Quality validation..." << std::endl;
        if (pImpl->progressCallback) pImpl->progressCallback(95);

        bool isWatertight = mesh->IsWatertight();
        bool isManifold = mesh->IsVertexManifold() && mesh->IsEdgeManifold();

        std::cout << "[Pipeline] Final mesh quality:" << std::endl;
        std::cout << "  Vertices: " << mesh->vertices_.size() << std::endl;
        std::cout << "  Triangles: " << mesh->triangles_.size() << std::endl;
        std::cout << "  Watertight: " << (isWatertight ? "Yes" : "No") << std::endl;
        std::cout << "  Manifold: " << (isManifold ? "Yes" : "No") << std::endl;

        if (!isWatertight || !isManifold) {
            std::cout << "[Pipeline] Warning: Mesh quality issues detected, may require repair" << std::endl;
        }

        auto overallEndTime = std::chrono::high_resolution_clock::now();
        auto overallDuration = std::chrono::duration_cast<std::chrono::milliseconds>(overallEndTime - overallStartTime);

        std::cout << "\n[Pipeline] COMPLETE: Total processing time: "
                  << overallDuration.count() << " ms" << std::endl;
        std::cout << "\n====================================================\n" << std::endl;

        pImpl->updatePerformanceStats("completePipeline", overallDuration.count());
        if (pImpl->progressCallback) pImpl->progressCallback(100);

        return mesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Pipeline exception: " + std::string(e.what());
        std::cout << "[Pipeline] ERROR: " << pImpl->lastError << std::endl;
        return nullptr;
    }
}
#endif

} // namespace pointcloud
} // namespace unlook