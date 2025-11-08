#include "unlook/stereo/PointCloudGenerator.hpp"
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <ctime>

#ifdef _OPENMP
#include <omp.h>
#endif

// Conditional Open3D includes
#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#endif

namespace unlook {
namespace stereo {

// =====================================================
// Configuration validation implementations
// =====================================================

bool PointCloudGenerationConfig::validate() const {
    if (minDepthMm <= 0 || maxDepthMm <= minDepthMm) {
        std::cerr << "[PointCloudGenerator] Invalid depth range: " << minDepthMm
                  << " - " << maxDepthMm << std::endl;
        return false;
    }

    if (statisticalNeighbors <= 0 || statisticalStdRatio <= 0) {
        std::cerr << "[PointCloudGenerator] Invalid statistical filter parameters" << std::endl;
        return false;
    }

    if (normalNeighborRadius <= 0) {
        std::cerr << "[PointCloudGenerator] Invalid normal neighbor radius" << std::endl;
        return false;
    }

    return true;
}

std::string PointCloudGenerationConfig::toString() const {
    std::stringstream ss;
    ss << "Point Cloud Generation Config:\n";
    ss << "  Depth range: " << minDepthMm << " - " << maxDepthMm << " mm\n";
    ss << "  Statistical filtering: " << (enableStatisticalFiltering ? "ON" : "OFF");
    if (enableStatisticalFiltering) {
        ss << " (neighbors=" << statisticalNeighbors
           << ", std_ratio=" << statisticalStdRatio << ")";
    }
    ss << "\n";
    ss << "  Radius filtering: " << (enableRadiusFiltering ? "ON" : "OFF");
    if (enableRadiusFiltering) {
        ss << " (radius=" << radiusThresholdMm << "mm, min_neighbors=" << minRadiusNeighbors << ")";
    }
    ss << "\n";
    ss << "  Voxel downsampling: " << (enableVoxelDownsampling ? "ON" : "OFF");
    if (enableVoxelDownsampling) {
        ss << " (voxel_size=" << voxelSizeMm << "mm)";
    }
    ss << "\n";
    ss << "  Compute normals: " << (computeNormals ? "YES" : "NO") << "\n";
    ss << "  Color mapping: " << (enableColorMapping ? "YES" : "NO") << "\n";
    return ss.str();
}

bool PLYExportConfig::validate() const {
    if (precisionMm <= 0) {
        std::cerr << "[PointCloudGenerator] Invalid precision: " << precisionMm << std::endl;
        return false;
    }
    return true;
}

std::string PLYExportConfig::toString() const {
    std::stringstream ss;
    ss << "PLY Export Config:\n";
    ss << "  Format: ";
    switch (format) {
        case Format::ASCII: ss << "ASCII"; break;
        case Format::BINARY_LE: ss << "Binary Little-Endian"; break;
        case Format::BINARY_BE: ss << "Binary Big-Endian"; break;
    }
    ss << "\n";
    ss << "  Include normals: " << (includeNormals ? "YES" : "NO") << "\n";
    ss << "  Include colors: " << (includeColors ? "YES" : "NO") << "\n";
    ss << "  Precision: " << precisionMm << " mm\n";
    return ss.str();
}

std::string PointCloudQualityMetrics::toString() const {
    std::stringstream ss;
    ss << "Point Cloud Quality Metrics:\n";
    ss << "  Total points: " << totalPoints << "\n";
    ss << "  Valid points: " << validPoints << " (" << std::fixed << std::setprecision(1)
       << (validRatio * 100.0) << "%)\n";
    ss << "  Point density: " << std::scientific << std::setprecision(2)
       << meanPointDensity << " points/mm³\n";
    ss << "  Point spacing: " << std::fixed << std::setprecision(3)
       << pointSpacing << " mm\n";
    ss << "  Noise level: " << noiseLevel << " mm\n";
    ss << "  Outliers: " << outlierCount << " (" << std::fixed << std::setprecision(1)
       << (outlierRatio * 100.0) << "%)\n";
    ss << "  Coverage: " << std::fixed << std::setprecision(1)
       << coveragePercentage << "%\n";
    ss << "  Bounds: [" << minBounds[0] << ", " << minBounds[1] << ", " << minBounds[2] << "] to ["
       << maxBounds[0] << ", " << maxBounds[1] << ", " << maxBounds[2] << "] mm\n";
    ss << "  Centroid: [" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << "] mm\n";
    ss << "  Processing time: " << totalTime.count() << " ms "
       << "(conversion=" << conversionTime.count() << "ms, filtering=" << filteringTime.count() << "ms)\n";
    return ss.str();
}

// =====================================================
// PointCloudGenerator::Impl - Private implementation
// =====================================================

class PointCloudGenerator::Impl {
public:
    std::shared_ptr<calibration::CalibrationManager> calibManager;
    std::string lastError;
    PointCloudQualityMetrics lastMetrics;
    std::function<void(int)> progressCallback;
    bool arm64Optimized = false;
    bool parallelProcessing = true;
    int numThreads = 0;

    Impl() {
#ifdef __aarch64__
        arm64Optimized = true;
        std::cout << "[PointCloudGenerator] ARM64 optimizations enabled" << std::endl;
#endif

#ifdef _OPENMP
        parallelProcessing = true;
        std::cout << "[PointCloudGenerator] OpenMP parallel processing enabled" << std::endl;
#endif
    }

    void updateProgress(int progress) {
        if (progressCallback) {
            progressCallback(progress);
        }
    }

#ifdef OPEN3D_ENABLED
    // Convert unlook PointCloud to Open3D format
    std::shared_ptr<open3d::geometry::PointCloud> convertToOpen3D(const PointCloud& unlookCloud) {
        auto o3dCloud = std::make_shared<open3d::geometry::PointCloud>();

        o3dCloud->points_.reserve(unlookCloud.points.size());
        o3dCloud->colors_.reserve(unlookCloud.points.size());

        for (const auto& pt : unlookCloud.points) {
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) && pt.z > 0) {
                // Convert mm to meters for Open3D
                o3dCloud->points_.emplace_back(pt.x / 1000.0, pt.y / 1000.0, pt.z / 1000.0);
                o3dCloud->colors_.emplace_back(pt.r / 255.0, pt.g / 255.0, pt.b / 255.0);
            }
        }

        return o3dCloud;
    }

    // Convert Open3D PointCloud back to unlook format
    PointCloud convertFromOpen3D(std::shared_ptr<open3d::geometry::PointCloud> o3dCloud) {
        PointCloud unlookCloud;

        if (!o3dCloud || o3dCloud->points_.empty()) {
            return unlookCloud;
        }

        unlookCloud.points.reserve(o3dCloud->points_.size());
        bool hasColors = !o3dCloud->colors_.empty();

        for (size_t i = 0; i < o3dCloud->points_.size(); ++i) {
            Point3D pt;
            const auto& p3d = o3dCloud->points_[i];

            // Convert meters back to mm
            pt.x = static_cast<float>(p3d.x() * 1000.0);
            pt.y = static_cast<float>(p3d.y() * 1000.0);
            pt.z = static_cast<float>(p3d.z() * 1000.0);

            if (hasColors && i < o3dCloud->colors_.size()) {
                const auto& color = o3dCloud->colors_[i];
                pt.r = static_cast<uint8_t>(std::min(255.0, color.x() * 255.0));
                pt.g = static_cast<uint8_t>(std::min(255.0, color.y() * 255.0));
                pt.b = static_cast<uint8_t>(std::min(255.0, color.z() * 255.0));
            } else {
                pt.r = pt.g = pt.b = 255;
            }

            pt.confidence = 1.0f;
            unlookCloud.points.push_back(pt);
        }

        return unlookCloud;
    }
#endif
};

// =====================================================
// PointCloudGenerator public methods
// =====================================================

PointCloudGenerator::PointCloudGenerator()
    : pImpl(std::make_unique<Impl>()) {
}

PointCloudGenerator::~PointCloudGenerator() = default;

bool PointCloudGenerator::initialize(std::shared_ptr<calibration::CalibrationManager> calibrationManager) {
    if (!calibrationManager) {
        pImpl->lastError = "Calibration manager is null";
        return false;
    }

    if (!calibrationManager->isCalibrationValid()) {
        pImpl->lastError = "Calibration is not valid";
        return false;
    }

    pImpl->calibManager = calibrationManager;
    std::cout << "[PointCloudGenerator] Initialized with calibration baseline="
              << calibrationManager->getBaselineMm() << "mm" << std::endl;

    return true;
}

bool PointCloudGenerator::validateInputs(const cv::Mat& depthMap, const cv::Mat& colorImage) const {
    if (depthMap.empty()) {
        pImpl->lastError = "Empty depth map";
        return false;
    }

    if (depthMap.type() != CV_32F) {
        pImpl->lastError = "Depth map must be CV_32F";
        return false;
    }

    if (!colorImage.empty()) {
        if (colorImage.size() != depthMap.size()) {
            pImpl->lastError = "Color image size does not match depth map size";
            return false;
        }

        if (colorImage.type() != CV_8UC3) {
            pImpl->lastError = "Color image must be CV_8UC3 (BGR)";
            return false;
        }
    }

    return true;
}

bool PointCloudGenerator::filterDepthMap(cv::Mat& depthMap, const PointCloudGenerationConfig& config) {
    if (!config.filterDepthBeforeConversion) {
        return true;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    // Filter invalid depth values
    cv::Mat mask = (depthMap >= config.minDepthMm) & (depthMap <= config.maxDepthMm);

    // Filter depth discontinuities (optional)
    if (config.maxDepthChange > 0) {
        cv::Mat tempDepth = depthMap.clone();

        #pragma omp parallel for if(pImpl->parallelProcessing) schedule(dynamic)
        for (int y = 1; y < depthMap.rows - 1; ++y) {
            for (int x = 1; x < depthMap.cols - 1; ++x) {
                float centerDepth = tempDepth.at<float>(y, x);
                if (centerDepth <= 0 || !std::isfinite(centerDepth)) {
                    continue;
                }

                // Check neighbors for large depth changes
                bool validNeighborhood = true;
                for (int dy = -1; dy <= 1 && validNeighborhood; ++dy) {
                    for (int dx = -1; dx <= 1 && validNeighborhood; ++dx) {
                        if (dx == 0 && dy == 0) continue;

                        float neighborDepth = tempDepth.at<float>(y + dy, x + dx);
                        if (neighborDepth > 0 && std::isfinite(neighborDepth)) {
                            float depthDiff = std::abs(centerDepth - neighborDepth);
                            if (depthDiff > config.maxDepthChange) {
                                validNeighborhood = false;
                            }
                        }
                    }
                }

                if (!validNeighborhood) {
                    mask.at<uint8_t>(y, x) = 0;
                }
            }
        }
    }

    // Apply mask
    depthMap.setTo(std::numeric_limits<float>::quiet_NaN(), ~mask);

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "[PointCloudGenerator] Depth filtering completed in " << duration.count() << "ms" << std::endl;

    return true;
}

bool PointCloudGenerator::convertDepthToPoints(const cv::Mat& depthMap,
                                              const cv::Mat& colorImage,
                                              PointCloud& pointCloud,
                                              const PointCloudGenerationConfig& config) {
    if (!pImpl->calibManager) {
        pImpl->lastError = "Calibration manager not initialized";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    const auto& calibData = pImpl->calibManager->getCalibrationData();
    const cv::Mat& Q = calibData.Q;

    if (Q.empty() || Q.rows != 4 || Q.cols != 4) {
        pImpl->lastError = "Invalid Q matrix in calibration";
        return false;
    }

    pointCloud.clear();
    pointCloud.width = depthMap.cols;
    pointCloud.height = depthMap.rows;
    pointCloud.isOrganized = true;
    pointCloud.points.reserve(depthMap.cols * depthMap.rows);

    bool hasColor = !colorImage.empty() && config.enableColorMapping;

    // Extract Q matrix parameters for efficient computation
    // Q matrix structure:
    // [1  0    0      -cx]
    // [0  1    0      -cy]
    // [0  0    0       f ]
    // [0  0  -1/Tx  (cx-cx')/Tx]
    double fx = calibData.cameraMatrixLeft.at<double>(0, 0);
    double fy = calibData.cameraMatrixLeft.at<double>(1, 1);
    double cx = calibData.cameraMatrixLeft.at<double>(0, 2);
    double cy = calibData.cameraMatrixLeft.at<double>(1, 2);
    double baseline = pImpl->calibManager->getBaselineMm();

    std::cout << "[PointCloudGenerator] Using camera parameters: fx=" << fx
              << ", fy=" << fy << ", cx=" << cx << ", cy=" << cy
              << ", baseline=" << baseline << "mm" << std::endl;

    // Parallel point cloud generation
    std::vector<Point3D> threadLocalPoints;

    #pragma omp parallel if(pImpl->parallelProcessing) private(threadLocalPoints)
    {
        threadLocalPoints.reserve((depthMap.cols * depthMap.rows) / omp_get_num_threads());

        #pragma omp for schedule(dynamic, 32) nowait
        for (int y = 0; y < depthMap.rows; ++y) {
            for (int x = 0; x < depthMap.cols; ++x) {
                float depth = depthMap.at<float>(y, x);

                // Skip invalid depths
                if (!std::isfinite(depth) || depth <= config.minDepthMm || depth > config.maxDepthMm) {
                    if (!config.skipInvalidPoints) {
                        // Add NaN point to maintain organized structure
                        Point3D pt;
                        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                        threadLocalPoints.push_back(pt);
                    }
                    continue;
                }

                // Compute 3D coordinates
                // Standard stereo formula: Z = f * baseline / disparity
                // Here we work directly with depth from depthMap
                Point3D pt;
                pt.z = depth;  // Depth in mm
                pt.x = ((x - cx) * depth) / fx;
                pt.y = ((y - cy) * depth) / fy;

                // Map color if available
                if (hasColor) {
                    const cv::Vec3b& bgr = colorImage.at<cv::Vec3b>(y, x);
                    pt.b = bgr[0];
                    pt.g = bgr[1];
                    pt.r = bgr[2];
                } else {
                    pt.r = config.defaultColor[2];
                    pt.g = config.defaultColor[1];
                    pt.b = config.defaultColor[0];
                }

                pt.confidence = 1.0f;

                threadLocalPoints.push_back(pt);
            }
        }

        // Merge thread-local results
        #pragma omp critical
        {
            pointCloud.points.insert(pointCloud.points.end(),
                                    threadLocalPoints.begin(),
                                    threadLocalPoints.end());
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "[PointCloudGenerator] Converted " << pointCloud.points.size()
              << " points in " << duration.count() << "ms" << std::endl;

    return !pointCloud.points.empty();
}

bool PointCloudGenerator::generatePointCloud(const cv::Mat& depthMap,
                                            const cv::Mat& colorImage,
                                            PointCloud& pointCloud,
                                            const PointCloudGenerationConfig& config) {
    PointCloudQualityMetrics metrics;
    return generatePointCloudWithMetrics(depthMap, colorImage, pointCloud, metrics, config);
}

bool PointCloudGenerator::generatePointCloudWithMetrics(const cv::Mat& depthMap,
                                                       const cv::Mat& colorImage,
                                                       PointCloud& pointCloud,
                                                       PointCloudQualityMetrics& metrics,
                                                       const PointCloudGenerationConfig& config) {
    auto overallStartTime = std::chrono::high_resolution_clock::now();

    if (!config.validate()) {
        pImpl->lastError = "Invalid configuration";
        return false;
    }

    if (!validateInputs(depthMap, colorImage)) {
        return false;
    }

    pImpl->updateProgress(5);

    // Step 1: Filter depth map
    cv::Mat filteredDepth = depthMap.clone();
    if (!filterDepthMap(filteredDepth, config)) {
        return false;
    }

    pImpl->updateProgress(15);

    // Step 2: Convert to 3D points
    auto conversionStartTime = std::chrono::high_resolution_clock::now();

    if (!convertDepthToPoints(filteredDepth, colorImage, pointCloud, config)) {
        pImpl->lastError = "Failed to convert depth to points";
        return false;
    }

    auto conversionEndTime = std::chrono::high_resolution_clock::now();
    metrics.conversionTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        conversionEndTime - conversionStartTime);

    pImpl->updateProgress(40);

    // Step 3: Apply filtering pipeline
    auto filteringStartTime = std::chrono::high_resolution_clock::now();

    if (config.enableStatisticalFiltering || config.enableRadiusFiltering ||
        config.enableVoxelDownsampling || config.enablePlaneRemoval) {

        if (!applyFilteringPipeline(pointCloud, config)) {
            std::cerr << "[PointCloudGenerator] Warning: Filtering failed, using unfiltered cloud" << std::endl;
        }
    }

    auto filteringEndTime = std::chrono::high_resolution_clock::now();
    metrics.filteringTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        filteringEndTime - filteringStartTime);

    pImpl->updateProgress(70);

    // Step 4: Estimate normals if requested
    if (config.computeNormals) {
        estimateNormals(pointCloud, config.normalNeighborRadius);
    }

    pImpl->updateProgress(85);

    // Step 5: Assess quality metrics
    assessQuality(pointCloud, metrics);

    pImpl->updateProgress(95);

    auto overallEndTime = std::chrono::high_resolution_clock::now();
    metrics.totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        overallEndTime - overallStartTime);

    pImpl->lastMetrics = metrics;

    std::cout << "[PointCloudGenerator] Generation complete:\n" << metrics.toString() << std::endl;

    pImpl->updateProgress(100);

    return true;
}

#ifdef OPEN3D_ENABLED
std::shared_ptr<open3d::geometry::PointCloud> PointCloudGenerator::generateOpen3DPointCloud(
    const cv::Mat& depthMap,
    const cv::Mat& colorImage,
    const PointCloudGenerationConfig& config) {

    PointCloud unlookCloud;
    PointCloudQualityMetrics metrics;

    if (!generatePointCloudWithMetrics(depthMap, colorImage, unlookCloud, metrics, config)) {
        return nullptr;
    }

    return pImpl->convertToOpen3D(unlookCloud);
}
#endif

bool PointCloudGenerator::applyFilteringPipeline(PointCloud& pointCloud,
                                                const PointCloudGenerationConfig& config) {
#ifdef OPEN3D_ENABLED
    std::cout << "[PointCloudGenerator] Applying Open3D filtering pipeline..." << std::endl;

    auto o3dCloud = pImpl->convertToOpen3D(pointCloud);
    if (!o3dCloud || o3dCloud->points_.empty()) {
        pImpl->lastError = "Failed to convert to Open3D format";
        return false;
    }

    size_t initialPoints = o3dCloud->points_.size();

    // Statistical outlier removal
    if (config.enableStatisticalFiltering) {
        std::cout << "[PointCloudGenerator] Statistical outlier removal (neighbors="
                  << config.statisticalNeighbors << ", std_ratio=" << config.statisticalStdRatio << ")..." << std::endl;

        auto [filtered, indices] = o3dCloud->RemoveStatisticalOutliers(
            config.statisticalNeighbors,
            config.statisticalStdRatio);

        size_t removed = initialPoints - filtered->points_.size();
        std::cout << "[PointCloudGenerator] Removed " << removed << " outliers ("
                  << (100.0 * removed / initialPoints) << "%)" << std::endl;

        o3dCloud = filtered;
    }

    // Radius outlier removal
    if (config.enableRadiusFiltering) {
        std::cout << "[PointCloudGenerator] Radius outlier removal (radius="
                  << config.radiusThresholdMm << "mm, min_neighbors="
                  << config.minRadiusNeighbors << ")..." << std::endl;

        auto [filtered, indices] = o3dCloud->RemoveRadiusOutliers(
            config.minRadiusNeighbors,
            config.radiusThresholdMm / 1000.0);  // Convert mm to meters

        o3dCloud = filtered;
    }

    // Voxel downsampling
    if (config.enableVoxelDownsampling) {
        std::cout << "[PointCloudGenerator] Voxel downsampling (voxel_size="
                  << config.voxelSizeMm << "mm)..." << std::endl;

        size_t beforeDownsample = o3dCloud->points_.size();
        auto downsampled = o3dCloud->VoxelDownSample(config.voxelSizeMm / 1000.0);

        std::cout << "[PointCloudGenerator] Downsampled: " << beforeDownsample
                  << " -> " << downsampled->points_.size() << " points ("
                  << (100.0 * downsampled->points_.size() / beforeDownsample) << "% retained)" << std::endl;

        o3dCloud = downsampled;
    }

    // Plane removal (RANSAC-based)
    if (config.enablePlaneRemoval) {
        std::cout << "[PointCloudGenerator] Plane removal (threshold="
                  << config.planeDistanceThresholdMm << "mm)..." << std::endl;

        auto [plane_model, inliers] = o3dCloud->SegmentPlane(
            config.planeDistanceThresholdMm / 1000.0,  // Convert mm to meters
            3,  // min points
            config.planeRansacIterations);

        if (inliers.size() >= static_cast<size_t>(config.planeMinPoints)) {
            auto filtered = o3dCloud->SelectByIndex(inliers, true);  // Invert selection

            std::cout << "[PointCloudGenerator] Removed plane with " << inliers.size()
                      << " inlier points" << std::endl;

            o3dCloud = filtered;
        } else {
            std::cout << "[PointCloudGenerator] No significant plane detected" << std::endl;
        }
    }

    // Convert back to unlook format
    pointCloud = pImpl->convertFromOpen3D(o3dCloud);

    std::cout << "[PointCloudGenerator] Filtering complete: " << initialPoints
              << " -> " << pointCloud.points.size() << " points" << std::endl;

    return true;

#else
    std::cout << "[PointCloudGenerator] Open3D not available, skipping advanced filtering" << std::endl;
    pImpl->lastError = "Open3D filtering not available in this build";
    return false;
#endif
}

bool PointCloudGenerator::estimateNormals(PointCloud& pointCloud, int neighborRadius) {
    // Simple normal estimation using local plane fitting
    // This is a basic implementation - Open3D provides superior algorithms

    std::cout << "[PointCloudGenerator] Estimating normals (radius=" << neighborRadius << ")..." << std::endl;

    auto startTime = std::chrono::high_resolution_clock::now();

    // Note: Current Point3D structure doesn't have normal fields
    // This is a placeholder implementation that would require extending Point3D

    // TODO: Extend Point3D to include normal vectors (nx, ny, nz)
    // For now, log a warning

    std::cout << "[PointCloudGenerator] Normal estimation requires Point3D extension (nx, ny, nz fields)" << std::endl;
    std::cout << "[PointCloudGenerator] Skipping normal estimation - use Open3D integration for normals" << std::endl;

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "[PointCloudGenerator] Normal estimation completed in " << duration.count() << "ms" << std::endl;

    return true;
}

void PointCloudGenerator::computeBoundingBox(const PointCloud& pointCloud,
                                            cv::Vec3f& minBounds,
                                            cv::Vec3f& maxBounds,
                                            cv::Vec3f& centroid) const {
    minBounds = cv::Vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    maxBounds = cv::Vec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    centroid = cv::Vec3f(0, 0, 0);

    size_t validCount = 0;

    for (const auto& pt : pointCloud.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
            continue;
        }

        minBounds[0] = std::min(minBounds[0], pt.x);
        minBounds[1] = std::min(minBounds[1], pt.y);
        minBounds[2] = std::min(minBounds[2], pt.z);

        maxBounds[0] = std::max(maxBounds[0], pt.x);
        maxBounds[1] = std::max(maxBounds[1], pt.y);
        maxBounds[2] = std::max(maxBounds[2], pt.z);

        centroid[0] += pt.x;
        centroid[1] += pt.y;
        centroid[2] += pt.z;

        validCount++;
    }

    if (validCount > 0) {
        centroid[0] /= validCount;
        centroid[1] /= validCount;
        centroid[2] /= validCount;
    }
}

double PointCloudGenerator::estimateNoiseLevel(const PointCloud& pointCloud) const {
    // Estimate noise as standard deviation of point-to-local-plane distances
    // Simplified implementation: use nearest neighbor distances

    if (pointCloud.points.size() < 10) {
        return 0.0;
    }

    std::vector<double> nearestDistances;
    nearestDistances.reserve(std::min(pointCloud.points.size(), size_t(1000)));

    // Sample subset of points for efficiency
    size_t step = std::max(size_t(1), pointCloud.points.size() / 1000);

    for (size_t i = 0; i < pointCloud.points.size(); i += step) {
        const auto& pt = pointCloud.points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
            continue;
        }

        double minDist = DBL_MAX;

        // Find nearest neighbor
        for (size_t j = 0; j < pointCloud.points.size(); j += step) {
            if (i == j) continue;

            const auto& neighbor = pointCloud.points[j];
            if (!std::isfinite(neighbor.x) || !std::isfinite(neighbor.y) ||
                !std::isfinite(neighbor.z) || neighbor.z <= 0) {
                continue;
            }

            double dist = std::sqrt(
                std::pow(pt.x - neighbor.x, 2) +
                std::pow(pt.y - neighbor.y, 2) +
                std::pow(pt.z - neighbor.z, 2));

            if (dist < minDist) {
                minDist = dist;
            }
        }

        if (minDist != DBL_MAX) {
            nearestDistances.push_back(minDist);
        }
    }

    if (nearestDistances.empty()) {
        return 0.0;
    }

    // Compute standard deviation
    double mean = std::accumulate(nearestDistances.begin(), nearestDistances.end(), 0.0)
                  / nearestDistances.size();

    double variance = 0.0;
    for (double dist : nearestDistances) {
        variance += std::pow(dist - mean, 2);
    }
    variance /= nearestDistances.size();

    return std::sqrt(variance);
}

double PointCloudGenerator::estimatePointDensity(const PointCloud& pointCloud,
                                                const cv::Vec3f& minBounds,
                                                const cv::Vec3f& maxBounds) const {
    double volume = (maxBounds[0] - minBounds[0]) *
                   (maxBounds[1] - minBounds[1]) *
                   (maxBounds[2] - minBounds[2]);

    if (volume <= 0) {
        return 0.0;
    }

    size_t validPoints = 0;
    for (const auto& pt : pointCloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) && pt.z > 0) {
            validPoints++;
        }
    }

    return static_cast<double>(validPoints) / volume;
}

bool PointCloudGenerator::assessQuality(const PointCloud& pointCloud,
                                       PointCloudQualityMetrics& metrics) const {
    auto startTime = std::chrono::high_resolution_clock::now();

    metrics.totalPoints = pointCloud.points.size();
    metrics.validPoints = 0;

    for (const auto& pt : pointCloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) && pt.z > 0) {
            metrics.validPoints++;
        }
    }

    metrics.validRatio = static_cast<double>(metrics.validPoints) / metrics.totalPoints;

    if (metrics.validPoints > 0) {
        computeBoundingBox(pointCloud, metrics.minBounds, metrics.maxBounds, metrics.centroid);
        metrics.meanPointDensity = estimatePointDensity(pointCloud, metrics.minBounds, metrics.maxBounds);
        metrics.noiseLevel = estimateNoiseLevel(pointCloud);

        // Estimate point spacing (average nearest neighbor distance)
        // Approximation: cube root of (volume / num_points)
        double volume = (metrics.maxBounds[0] - metrics.minBounds[0]) *
                       (metrics.maxBounds[1] - metrics.minBounds[1]) *
                       (metrics.maxBounds[2] - metrics.minBounds[2]);

        if (volume > 0 && metrics.validPoints > 0) {
            metrics.pointSpacing = std::cbrt(volume / metrics.validPoints);
        }

        metrics.coveragePercentage = metrics.validRatio * 100.0;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "[PointCloudGenerator] Quality assessment completed in " << duration.count() << "ms" << std::endl;

    return true;
}

bool PointCloudGenerator::exportToPLY(const PointCloud& pointCloud,
                                     const std::string& filename,
                                     const PLYExportConfig& config) {
    if (!config.validate()) {
        pImpl->lastError = "Invalid PLY export configuration";
        return false;
    }

    std::cout << "[PointCloudGenerator] Exporting to PLY: " << filename << std::endl;

    // Count valid points
    size_t validPoints = 0;
    for (const auto& pt : pointCloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) && pt.z > 0) {
            validPoints++;
        }
    }

    if (validPoints == 0) {
        pImpl->lastError = "No valid points to export";
        return false;
    }

    std::ofstream file(filename, config.format == PLYExportConfig::Format::ASCII ?
                                 std::ios::out : std::ios::out | std::ios::binary);

    if (!file.is_open()) {
        pImpl->lastError = "Failed to open file for writing: " + filename;
        return false;
    }

    // Write PLY header
    file << "ply\n";

    switch (config.format) {
        case PLYExportConfig::Format::ASCII:
            file << "format ascii 1.0\n";
            break;
        case PLYExportConfig::Format::BINARY_LE:
            file << "format binary_little_endian 1.0\n";
            break;
        case PLYExportConfig::Format::BINARY_BE:
            file << "format binary_big_endian 1.0\n";
            break;
    }

    // Write metadata comments
    file << "comment " << config.comment << "\n";

    if (!config.timestamp.empty()) {
        file << "comment timestamp: " << config.timestamp << "\n";
    } else {
        // Auto-generate timestamp
        std::time_t now = std::time(nullptr);
        char timeStr[100];
        std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", std::localtime(&now));
        file << "comment timestamp: " << timeStr << "\n";
    }

    file << "comment precision: " << config.precisionMm << " mm\n";

    if (!config.calibrationFile.empty()) {
        file << "comment calibration: " << config.calibrationFile << "\n";
    }

    if (pImpl->calibManager) {
        file << "comment baseline: " << pImpl->calibManager->getBaselineMm() << " mm\n";
    }

    if (config.includeManufacturingMetadata) {
        if (!config.partName.empty()) {
            file << "comment part_name: " << config.partName << "\n";
        }
        if (!config.scanOperator.empty()) {
            file << "comment operator: " << config.scanOperator << "\n";
        }
        if (!config.material.empty()) {
            file << "comment material: " << config.material << "\n";
        }
    }

    // Write element vertex
    file << "element vertex " << validPoints << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";

    if (config.includeNormals) {
        file << "property float nx\n";
        file << "property float ny\n";
        file << "property float nz\n";
    }

    if (config.includeColors) {
        file << "property uchar red\n";
        file << "property uchar green\n";
        file << "property uchar blue\n";
    }

    if (config.includeConfidence) {
        file << "property float confidence\n";
    }

    file << "end_header\n";

    // Write point data
    size_t pointsWritten = 0;

    if (config.format == PLYExportConfig::Format::ASCII) {
        // ASCII format
        for (const auto& pt : pointCloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
                continue;
            }

            file << pt.x << " " << pt.y << " " << pt.z;

            if (config.includeNormals) {
                file << " 0 0 1";  // Default normals (TODO: use actual normals when available)
            }

            if (config.includeColors) {
                file << " " << static_cast<int>(pt.r)
                     << " " << static_cast<int>(pt.g)
                     << " " << static_cast<int>(pt.b);
            }

            if (config.includeConfidence) {
                file << " " << pt.confidence;
            }

            file << "\n";
            pointsWritten++;
        }
    } else {
        // Binary format
        for (const auto& pt : pointCloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
                continue;
            }

            file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));

            if (config.includeNormals) {
                float nx = 0.0f, ny = 0.0f, nz = 1.0f;
                file.write(reinterpret_cast<const char*>(&nx), sizeof(float));
                file.write(reinterpret_cast<const char*>(&ny), sizeof(float));
                file.write(reinterpret_cast<const char*>(&nz), sizeof(float));
            }

            if (config.includeColors) {
                file.write(reinterpret_cast<const char*>(&pt.r), sizeof(uint8_t));
                file.write(reinterpret_cast<const char*>(&pt.g), sizeof(uint8_t));
                file.write(reinterpret_cast<const char*>(&pt.b), sizeof(uint8_t));
            }

            if (config.includeConfidence) {
                file.write(reinterpret_cast<const char*>(&pt.confidence), sizeof(float));
            }

            pointsWritten++;
        }
    }

    file.close();

    if (pointsWritten != validPoints) {
        pImpl->lastError = "PLY export point count mismatch";
        return false;
    }

    std::cout << "[PointCloudGenerator] PLY export successful: " << pointsWritten
              << " points written to " << filename << std::endl;

    return true;
}

bool PointCloudGenerator::exportToPCD(const PointCloud& pointCloud,
                                     const std::string& filename,
                                     bool binaryFormat) {
    // Count valid points
    size_t validPoints = 0;
    for (const auto& pt : pointCloud.points) {
        if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z) && pt.z > 0) {
            validPoints++;
        }
    }

    if (validPoints == 0) {
        pImpl->lastError = "No valid points to export";
        return false;
    }

    std::ofstream file(filename, binaryFormat ? std::ios::out | std::ios::binary : std::ios::out);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open file for writing: " + filename;
        return false;
    }

    // Write PCD header
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z rgb\n";
    file << "SIZE 4 4 4 4\n";
    file << "TYPE F F F U\n";
    file << "COUNT 1 1 1 1\n";
    file << "WIDTH " << validPoints << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << validPoints << "\n";
    file << "DATA " << (binaryFormat ? "binary" : "ascii") << "\n";

    // Write point data
    if (binaryFormat) {
        for (const auto& pt : pointCloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
                continue;
            }

            file.write(reinterpret_cast<const char*>(&pt.x), sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.y), sizeof(float));
            file.write(reinterpret_cast<const char*>(&pt.z), sizeof(float));

            uint32_t rgb = (static_cast<uint32_t>(pt.r) << 16) |
                          (static_cast<uint32_t>(pt.g) << 8) |
                          static_cast<uint32_t>(pt.b);
            file.write(reinterpret_cast<const char*>(&rgb), sizeof(uint32_t));
        }
    } else {
        for (const auto& pt : pointCloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
                continue;
            }

            uint32_t rgb = (static_cast<uint32_t>(pt.r) << 16) |
                          (static_cast<uint32_t>(pt.g) << 8) |
                          static_cast<uint32_t>(pt.b);

            file << pt.x << " " << pt.y << " " << pt.z << " " << rgb << "\n";
        }
    }

    file.close();

    std::cout << "[PointCloudGenerator] PCD export successful: " << validPoints
              << " points written to " << filename << std::endl;

    return true;
}

bool PointCloudGenerator::exportToOBJ(const PointCloud& pointCloud,
                                     const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open file for writing: " + filename;
        return false;
    }

    // Write OBJ header
    file << "# Generated by Unlook 3D Scanner\n";
    file << "# Point cloud export (vertices only)\n";

    size_t validPoints = 0;

    // Write vertices
    for (const auto& pt : pointCloud.points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z) || pt.z <= 0) {
            continue;
        }

        file << "v " << pt.x << " " << pt.y << " " << pt.z << "\n";
        validPoints++;
    }

    file.close();

    std::cout << "[PointCloudGenerator] OBJ export successful: " << validPoints
              << " vertices written to " << filename << std::endl;

    return true;
}

void PointCloudGenerator::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string PointCloudGenerator::getLastError() const {
    return pImpl->lastError;
}

PointCloudQualityMetrics PointCloudGenerator::getLastMetrics() const {
    return pImpl->lastMetrics;
}

void PointCloudGenerator::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
    std::cout << "[PointCloudGenerator] ARM64 optimizations "
              << (enable ? "enabled" : "disabled") << std::endl;
}

void PointCloudGenerator::enableParallelProcessing(bool enable, int numThreads) {
    pImpl->parallelProcessing = enable;
    pImpl->numThreads = numThreads;

#ifdef _OPENMP
    if (enable) {
        if (numThreads > 0) {
            omp_set_num_threads(numThreads);
        }
        std::cout << "[PointCloudGenerator] Parallel processing enabled with "
                  << (numThreads > 0 ? numThreads : omp_get_max_threads()) << " threads" << std::endl;
    } else {
        std::cout << "[PointCloudGenerator] Parallel processing disabled" << std::endl;
    }
#else
    if (enable) {
        std::cout << "[PointCloudGenerator] Warning: OpenMP not available, parallel processing disabled" << std::endl;
        pImpl->parallelProcessing = false;
    }
#endif
}

} // namespace stereo
} // namespace unlook
