#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/PointCloud.h>
#include <open3d/geometry/TriangleMesh.h>
#include "unlook/pointcloud/PointCloudProcessor.hpp"
#include "unlook/pointcloud/PoissonReconstructor.hpp"
#include "unlook/pointcloud/MeshCleaner.hpp"

using namespace unlook::pointcloud;

/**
 * @brief Generate synthetic test point cloud (sphere with noise)
 */
std::shared_ptr<open3d::geometry::PointCloud> generateTestPointCloud(int numPoints = 10000) {
    auto cloud = std::make_shared<open3d::geometry::PointCloud>();

    // Generate sphere with uniform distribution
    for (int i = 0; i < numPoints; ++i) {
        // Fibonacci sphere for uniform distribution
        double phi = M_PI * (3.0 - std::sqrt(5.0));  // Golden angle
        double y = 1.0 - (i / double(numPoints - 1)) * 2.0;  // y goes from 1 to -1
        double radius = std::sqrt(1.0 - y * y);  // radius at y

        double theta = phi * i;  // golden angle increment

        double x = cos(theta) * radius;
        double z = sin(theta) * radius;

        // Scale to 50mm radius (100mm diameter object)
        double scale = 50.0 / 1000.0;  // Convert to meters for Open3D
        cloud->points_.push_back(Eigen::Vector3d(x * scale, y * scale, z * scale));

        // Add color (optional)
        cloud->colors_.push_back(Eigen::Vector3d(0.5, 0.7, 0.9));
    }

    std::cout << "[TestSetup] Generated synthetic sphere: " << cloud->points_.size() << " points" << std::endl;
    return cloud;
}

/**
 * @brief Test 1: Basic Poisson Reconstruction
 */
bool testPoissonReconstruction() {
    std::cout << "\n========== TEST 1: Basic Poisson Reconstruction ==========\n" << std::endl;

    try {
        // Generate test data
        auto cloud = generateTestPointCloud(5000);

        // Estimate normals
        PoissonReconstructor reconstructor;
        if (!reconstructor.estimateAndOrientNormals(*cloud, 10.0, 30)) {
            std::cout << "[TEST FAILED] Normal estimation failed: " << reconstructor.getLastError() << std::endl;
            return false;
        }

        // Perform reconstruction
        PoissonSettings settings = PoissonSettings::forBalancedQuality();
        auto mesh = reconstructor.reconstruct(*cloud, settings);

        if (!mesh) {
            std::cout << "[TEST FAILED] Reconstruction failed: " << reconstructor.getLastError() << std::endl;
            return false;
        }

        // Validate results
        PoissonResult result = reconstructor.getLastResult();

        std::cout << "[TEST RESULT] " << result.toString() << std::endl;

        // Check success criteria
        bool success = result.success &&
                      result.outputVertices > 0 &&
                      result.outputTriangles > 0 &&
                      result.isWatertight &&
                      result.isManifold;

        if (success) {
            std::cout << "[TEST PASSED] Poisson reconstruction successful" << std::endl;
        } else {
            std::cout << "[TEST FAILED] Quality checks failed" << std::endl;
        }

        return success;

    } catch (const std::exception& e) {
        std::cout << "[TEST FAILED] Exception: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Test 2: Mesh Cleaning
 */
bool testMeshCleaning() {
    std::cout << "\n========== TEST 2: Mesh Cleaning ==========\n" << std::endl;

    try {
        // Generate and reconstruct
        auto cloud = generateTestPointCloud(5000);

        PoissonReconstructor reconstructor;
        reconstructor.estimateAndOrientNormals(*cloud);

        auto mesh = reconstructor.reconstruct(*cloud);
        if (!mesh) {
            std::cout << "[TEST FAILED] Reconstruction failed" << std::endl;
            return false;
        }

        size_t trianglesBefore = mesh->triangles_.size();

        // Clean mesh
        MeshCleaner cleaner;
        MeshCleanerSettings settings;
        settings.mode = MeshCleanerSettings::FilterMode::KEEP_LARGEST;

        auto cleanResult = cleaner.removeSmallObjects(*mesh, settings);

        std::cout << "[TEST RESULT] " << cleanResult.toString() << std::endl;

        bool success = cleanResult.success &&
                      cleanResult.outputTriangles > 0;

        if (success) {
            std::cout << "[TEST PASSED] Mesh cleaning successful" << std::endl;
        } else {
            std::cout << "[TEST FAILED] Cleaning failed" << std::endl;
        }

        return success;

    } catch (const std::exception& e) {
        std::cout << "[TEST FAILED] Exception: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Test 3: Complete Pipeline Integration
 */
bool testCompletePipeline() {
    std::cout << "\n========== TEST 3: Complete Pipeline Integration ==========\n" << std::endl;

    try {
        // Generate test data
        auto cloud = generateTestPointCloud(10000);

        // Estimate normals first
        PoissonReconstructor reconstructor;
        if (!reconstructor.estimateAndOrientNormals(*cloud, 10.0, 30)) {
            std::cout << "[TEST FAILED] Normal estimation failed" << std::endl;
            return false;
        }

        // Run complete pipeline
        PointCloudProcessor processor;
        auto mesh = processor.processCompletePipeline(*cloud);

        if (!mesh) {
            std::cout << "[TEST FAILED] Pipeline failed: " << processor.getLastError() << std::endl;
            return false;
        }

        // Validate final mesh quality
        bool isWatertight = mesh->IsWatertight();
        bool isManifold = mesh->IsVertexManifold() && mesh->IsEdgeManifold();

        size_t vertices = mesh->vertices_.size();
        size_t triangles = mesh->triangles_.size();

        std::cout << "\n[TEST RESULT] Final Mesh Quality:" << std::endl;
        std::cout << "  Vertices: " << vertices << std::endl;
        std::cout << "  Triangles: " << triangles << std::endl;
        std::cout << "  Watertight: " << (isWatertight ? "Yes" : "No") << std::endl;
        std::cout << "  Manifold: " << (isManifold ? "Yes" : "No") << std::endl;

        // Success criteria (matching user requirements from prompt)
        bool success = mesh != nullptr &&
                      isWatertight &&
                      isManifold &&
                      triangles < 200000 &&   // Optimized (not too many)
                      triangles > 50000;      // Not over-simplified

        if (success) {
            std::cout << "\n[TEST PASSED] Complete pipeline successful - production ready!" << std::endl;
        } else {
            std::cout << "\n[TEST FAILED] Quality criteria not met" << std::endl;
            if (!isWatertight) std::cout << "  - Not watertight" << std::endl;
            if (!isManifold) std::cout << "  - Not manifold" << std::endl;
            if (triangles >= 200000) std::cout << "  - Too many triangles (not optimized)" << std::endl;
            if (triangles <= 50000) std::cout << "  - Too few triangles (over-simplified)" << std::endl;
        }

        // Get performance stats
        auto stats = processor.getPerformanceStats();
        if (stats.find("completePipeline") != stats.end()) {
            std::cout << "\n[PERFORMANCE] Complete pipeline time: "
                      << stats["completePipeline"] << " ms" << std::endl;

            // Check performance requirement (<15s for typical point cloud)
            if (stats["completePipeline"] < 15000.0) {
                std::cout << "[PERFORMANCE PASSED] Processing time < 15s" << std::endl;
            } else {
                std::cout << "[PERFORMANCE WARNING] Processing time > 15s" << std::endl;
            }
        }

        return success;

    } catch (const std::exception& e) {
        std::cout << "[TEST FAILED] Exception: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Test 4: Mesh Simplification
 */
bool testMeshSimplification() {
    std::cout << "\n========== TEST 4: Mesh Simplification ==========\n" << std::endl;

    try {
        // Generate and reconstruct
        auto cloud = generateTestPointCloud(5000);

        PoissonReconstructor reconstructor;
        reconstructor.estimateAndOrientNormals(*cloud);

        auto mesh = reconstructor.reconstruct(*cloud);
        if (!mesh) {
            std::cout << "[TEST FAILED] Reconstruction failed" << std::endl;
            return false;
        }

        size_t trianglesBefore = mesh->triangles_.size();
        std::cout << "[TEST] Triangles before simplification: " << trianglesBefore << std::endl;

        // Simplify mesh with geometric accuracy requirement
        MeshCleaner cleaner;
        SimplificationSettings simplifySettings = SimplificationSettings::forGeometricAccuracy(0.01);  // 10 microns

        auto simplified = cleaner.simplify(*mesh, simplifySettings);

        if (!simplified) {
            std::cout << "[TEST FAILED] Simplification failed: " << cleaner.getLastError() << std::endl;
            return false;
        }

        size_t trianglesAfter = simplified->triangles_.size();
        std::cout << "[TEST] Triangles after simplification: " << trianglesAfter << std::endl;

        double reduction = 100.0 * (trianglesBefore - trianglesAfter) / trianglesBefore;
        std::cout << "[TEST] Reduction: " << std::fixed << std::setprecision(1) << reduction << "%" << std::endl;

        bool success = trianglesAfter < trianglesBefore &&
                      trianglesAfter > (trianglesBefore / 10);  // Not over-simplified

        if (success) {
            std::cout << "[TEST PASSED] Simplification successful" << std::endl;
        } else {
            std::cout << "[TEST FAILED] Simplification criteria not met" << std::endl;
        }

        return success;

    } catch (const std::exception& e) {
        std::cout << "[TEST FAILED] Exception: " << e.what() << std::endl;
        return false;
    }
}

int main() {
    std::cout << "\n" << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << "  UNLOOK 3D SCANNER - ARTEC-GRADE PIPELINE TEST SUITE" << std::endl;
    std::cout << "  Testing complete processing pipeline for investor demo" << std::endl;
    std::cout << "==========================================================" << std::endl;

    int totalTests = 0;
    int passedTests = 0;

    // Test 1: Poisson Reconstruction
    totalTests++;
    if (testPoissonReconstruction()) {
        passedTests++;
    }

    // Test 2: Mesh Cleaning
    totalTests++;
    if (testMeshCleaning()) {
        passedTests++;
    }

    // Test 3: Complete Pipeline (CRITICAL)
    totalTests++;
    if (testCompletePipeline()) {
        passedTests++;
    }

    // Test 4: Mesh Simplification
    totalTests++;
    if (testMeshSimplification()) {
        passedTests++;
    }

    // Final summary
    std::cout << "\n==========================================================" << std::endl;
    std::cout << "  TEST SUITE SUMMARY" << std::endl;
    std::cout << "==========================================================" << std::endl;
    std::cout << "Tests passed: " << passedTests << "/" << totalTests << std::endl;

    if (passedTests == totalTests) {
        std::cout << "\n  STATUS: ALL TESTS PASSED - PRODUCTION READY!" << std::endl;
        std::cout << "  Pipeline validated for investor demo" << std::endl;
        std::cout << "=========================================================\n" << std::endl;
        return 0;
    } else {
        std::cout << "\n  STATUS: SOME TESTS FAILED - REVIEW REQUIRED" << std::endl;
        std::cout << "=========================================================\n" << std::endl;
        return 1;
    }
}

#else

int main() {
    std::cerr << "ERROR: Tests require Open3D support (OPEN3D_ENABLED)" << std::endl;
    std::cerr << "Please rebuild with Open3D enabled" << std::endl;
    return 1;
}

#endif
