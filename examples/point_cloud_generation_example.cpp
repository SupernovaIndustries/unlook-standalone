/**
 * @file point_cloud_generation_example.cpp
 * @brief Example demonstrating point cloud generation from depth maps
 *
 * This example shows how to use the PointCloudGenerator to convert
 * depth maps to high-quality 3D point clouds with filtering and export.
 *
 * Usage:
 *   ./point_cloud_generation_example <depth_map.tiff> <color_image.png> [output.ply]
 *
 * Features demonstrated:
 * - Depth map loading and validation
 * - Point cloud generation with color mapping
 * - Statistical outlier removal
 * - Quality assessment
 * - PLY/PCD export with metadata
 */

#include "unlook/stereo/PointCloudGenerator.hpp"
#include "unlook/calibration/CalibrationManager.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <iomanip>
#include <chrono>

using namespace unlook;

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <depth_map> <color_image> [output_ply]\n";
    std::cout << "\n";
    std::cout << "Arguments:\n";
    std::cout << "  depth_map     - Input depth map (TIFF/PNG, CV_32F millimeters)\n";
    std::cout << "  color_image   - Input color image (PNG/JPG, CV_8UC3)\n";
    std::cout << "  output_ply    - Output PLY file (default: point_cloud.ply)\n";
    std::cout << "\n";
    std::cout << "Example:\n";
    std::cout << "  " << programName << " depth.tiff color.png output.ply\n";
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "Unlook Point Cloud Generation Example\n";
    std::cout << "========================================\n\n";

    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    std::string depthMapPath = argv[1];
    std::string colorImagePath = argv[2];
    std::string outputPath = (argc >= 4) ? argv[3] : "point_cloud.ply";

    // ===============================================
    // Step 1: Load calibration
    // ===============================================
    std::cout << "[1/6] Loading calibration...\n";

    auto calibManager = std::make_shared<calibration::CalibrationManager>();

    // Try to load calibration from standard locations
    std::vector<std::string> calibPaths = {
        "/unlook_calib/calib-latest.yaml",
        "calibration/calib_boofcv_test3.yaml",
        "../calibration/calib_boofcv_test3.yaml"
    };

    bool calibLoaded = false;
    for (const auto& path : calibPaths) {
        if (calibManager->loadCalibration(path)) {
            std::cout << "  Calibration loaded from: " << path << "\n";
            std::cout << "  Baseline: " << calibManager->getBaselineMm() << " mm\n";
            std::cout << "  RMS error: " << calibManager->getRmsError() << " pixels\n";
            calibLoaded = true;
            break;
        }
    }

    if (!calibLoaded) {
        std::cerr << "ERROR: Failed to load calibration from standard paths\n";
        std::cerr << "Tried:\n";
        for (const auto& path : calibPaths) {
            std::cerr << "  - " << path << "\n";
        }
        return 1;
    }

    // ===============================================
    // Step 2: Load depth map and color image
    // ===============================================
    std::cout << "\n[2/6] Loading input images...\n";

    cv::Mat depthMap = cv::imread(depthMapPath, cv::IMREAD_UNCHANGED);
    if (depthMap.empty()) {
        std::cerr << "ERROR: Failed to load depth map: " << depthMapPath << "\n";
        return 1;
    }

    // Convert depth map to CV_32F if needed
    if (depthMap.type() != CV_32F) {
        if (depthMap.type() == CV_16U) {
            // Assume 16-bit depth map with millimeter encoding
            depthMap.convertTo(depthMap, CV_32F);
        } else if (depthMap.type() == CV_8U) {
            // Assume 8-bit disparity map (needs conversion)
            depthMap.convertTo(depthMap, CV_32F);
            // TODO: Convert disparity to depth using calibration
            std::cerr << "WARNING: 8-bit depth map requires disparity-to-depth conversion\n";
        } else {
            std::cerr << "ERROR: Unsupported depth map type: " << depthMap.type() << "\n";
            return 1;
        }
    }

    std::cout << "  Depth map: " << depthMap.cols << "x" << depthMap.rows
              << " (type=" << depthMap.type() << ")\n";

    cv::Mat colorImage = cv::imread(colorImagePath, cv::IMREAD_COLOR);
    if (colorImage.empty()) {
        std::cerr << "WARNING: Failed to load color image: " << colorImagePath << "\n";
        std::cerr << "Continuing without color mapping...\n";
    } else {
        std::cout << "  Color image: " << colorImage.cols << "x" << colorImage.rows << "\n";

        // Resize color image if it doesn't match depth map
        if (colorImage.size() != depthMap.size()) {
            std::cout << "  Resizing color image to match depth map...\n";
            cv::resize(colorImage, colorImage, depthMap.size(), 0, 0, cv::INTER_LINEAR);
        }
    }

    // ===============================================
    // Step 3: Initialize PointCloudGenerator
    // ===============================================
    std::cout << "\n[3/6] Initializing PointCloudGenerator...\n";

    stereo::PointCloudGenerator pcGenerator;

    if (!pcGenerator.initialize(calibManager)) {
        std::cerr << "ERROR: Failed to initialize PointCloudGenerator: "
                  << pcGenerator.getLastError() << "\n";
        return 1;
    }

    // Enable optimizations
    pcGenerator.enableARM64Optimizations(true);
    pcGenerator.enableParallelProcessing(true, 4);

    // Set progress callback
    pcGenerator.setProgressCallback([](int progress) {
        std::cout << "\r  Progress: " << progress << "%" << std::flush;
    });

    // ===============================================
    // Step 4: Configure point cloud generation
    // ===============================================
    std::cout << "\n[4/6] Configuring point cloud generation...\n";

    stereo::PointCloudGenerationConfig config;

    // Depth filtering
    config.filterDepthBeforeConversion = true;
    config.minDepthMm = 200.0f;           // 3x baseline minimum
    config.maxDepthMm = 3500.0f;          // Extended range
    config.maxDepthChange = 50.0f;        // Filter depth discontinuities

    // Point cloud generation
    config.skipInvalidPoints = true;
    config.computeNormals = true;
    config.normalNeighborRadius = 5;

    // Color mapping
    config.enableColorMapping = !colorImage.empty();
    config.defaultColor = {255, 255, 255};

    // Statistical outlier removal (Open3D-based)
    config.enableStatisticalFiltering = true;
    config.statisticalNeighbors = 20;     // Unlook standard
    config.statisticalStdRatio = 2.0;     // Unlook standard

    // Voxel downsampling (optional, for large point clouds)
    config.enableVoxelDownsampling = false;
    config.voxelSizeMm = 1.0;

    // Plane removal (optional, for removing background)
    config.enablePlaneRemoval = false;

    // Parallel processing
    config.useParallelProcessing = true;
    config.numThreads = 4;

    std::cout << config.toString() << "\n";

    // ===============================================
    // Step 5: Generate point cloud with metrics
    // ===============================================
    std::cout << "\n[5/6] Generating point cloud...\n";

    auto startTime = std::chrono::high_resolution_clock::now();

    stereo::PointCloud pointCloud;
    stereo::PointCloudQualityMetrics metrics;

    if (!pcGenerator.generatePointCloudWithMetrics(
            depthMap, colorImage, pointCloud, metrics, config)) {
        std::cerr << "ERROR: Point cloud generation failed: "
                  << pcGenerator.getLastError() << "\n";
        return 1;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    std::cout << "\n\n  Point cloud generated successfully!\n";
    std::cout << "\n" << metrics.toString() << "\n";

    // ===============================================
    // Step 6: Export to PLY format
    // ===============================================
    std::cout << "\n[6/6] Exporting to PLY format...\n";

    stereo::PLYExportConfig plyConfig;
    plyConfig.format = stereo::PLYExportConfig::Format::BINARY_LE;
    plyConfig.includeNormals = true;
    plyConfig.includeColors = !colorImage.empty();
    plyConfig.includeConfidence = false;

    // Metadata
    plyConfig.comment = "Generated by Unlook 3D Scanner - Point Cloud Example";
    plyConfig.precisionMm = 0.005;
    plyConfig.calibrationFile = "calib_boofcv_test3.yaml";

    // Manufacturing metadata (optional)
    plyConfig.includeManufacturingMetadata = true;
    plyConfig.partName = "Example Scan";
    plyConfig.scanOperator = "Unlook Example";

    std::cout << plyConfig.toString() << "\n";

    if (!pcGenerator.exportToPLY(pointCloud, outputPath, plyConfig)) {
        std::cerr << "ERROR: PLY export failed: " << pcGenerator.getLastError() << "\n";
        return 1;
    }

    // Also export to PCD format
    std::string pcdPath = outputPath.substr(0, outputPath.find_last_of('.')) + ".pcd";
    if (pcGenerator.exportToPCD(pointCloud, pcdPath, true)) {
        std::cout << "  Also exported to PCD: " << pcdPath << "\n";
    }

    // ===============================================
    // Summary
    // ===============================================
    std::cout << "\n========================================\n";
    std::cout << "Point Cloud Generation Summary\n";
    std::cout << "========================================\n";
    std::cout << "Input:\n";
    std::cout << "  Depth map: " << depthMapPath << "\n";
    std::cout << "  Color image: " << colorImagePath << "\n";
    std::cout << "\nOutput:\n";
    std::cout << "  PLY file: " << outputPath << "\n";
    std::cout << "  PCD file: " << pcdPath << "\n";
    std::cout << "\nPoint Cloud Quality:\n";
    std::cout << "  Total points: " << metrics.totalPoints << "\n";
    std::cout << "  Valid points: " << metrics.validPoints << " ("
              << std::fixed << std::setprecision(1) << (metrics.validRatio * 100.0) << "%)\n";
    std::cout << "  Point density: " << std::scientific << std::setprecision(2)
              << metrics.meanPointDensity << " points/mm³\n";
    std::cout << "  Noise level: " << std::fixed << std::setprecision(3)
              << metrics.noiseLevel << " mm\n";
    std::cout << "\nProcessing Time:\n";
    std::cout << "  Conversion: " << metrics.conversionTime.count() << " ms\n";
    std::cout << "  Filtering: " << metrics.filteringTime.count() << " ms\n";
    std::cout << "  Total: " << totalDuration.count() << " ms\n";
    std::cout << "\nPerformance:\n";

    if (totalDuration.count() > 0) {
        double fps = 1000.0 / totalDuration.count();
        std::cout << "  Effective FPS: " << std::fixed << std::setprecision(2) << fps << " Hz\n";
    }

    std::cout << "\nVisualization:\n";
    std::cout << "  meshlab " << outputPath << "\n";
    std::cout << "  cloudcompare " << outputPath << "\n";
    std::cout << "========================================\n";

    return 0;
}
