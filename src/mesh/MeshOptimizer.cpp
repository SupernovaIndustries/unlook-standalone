#include "unlook/mesh/MeshOptimizer.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>
#include <set>
#include <queue>
#include <unordered_map>
#include <unordered_set>

// Conditional Open3D includes
#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#endif

namespace unlook {
namespace mesh {

// Edge structure for decimation
struct Edge {
    int v1, v2;
    double cost;
    cv::Vec3f optimalPosition;

    bool operator>(const Edge& other) const {
        return cost > other.cost;
    }
};

// MeshOptimizer implementation
class MeshOptimizer::Impl {
public:
    std::string lastError;
    std::function<void(int)> progressCallback;
    std::map<std::string, double> performanceStats;
    mutable std::mutex statsMutex;
    bool arm64Optimized = false;
    MeshValidator validator;
    std::vector<std::vector<int>> vertexAdjacency;

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

    void updatePerformanceStats(const std::string& operation, double timeMs) {
        std::lock_guard<std::mutex> lock(statsMutex);
        performanceStats[operation] = timeMs;
    }
};

MeshOptimizer::MeshOptimizer() : pImpl(std::make_unique<Impl>()) {}
MeshOptimizer::~MeshOptimizer() = default;

bool MeshOptimizer::smoothMesh(std::vector<cv::Vec3f>& vertices,
                              const std::vector<cv::Vec3i>& faces,
                              std::vector<cv::Vec3f>& normals,
                              const MeshSmoothingConfig& config,
                              MeshOptimizationResult& result) {
    if (!validateMeshInput(vertices, faces)) {
        pImpl->lastError = "Invalid mesh input for smoothing";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid smoothing configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Mesh Smoothing";

    try {
        // Compute before metrics
        computeMeshQuality(vertices, faces, result.beforeMetrics);

        pImpl->updateProgress(10);

        // Apply smoothing algorithm
        bool success = false;
        switch (config.algorithm) {
            case MeshSmoothingConfig::Algorithm::LAPLACIAN:
                success = laplacianSmoothing(vertices, faces, config);
                break;
            case MeshSmoothingConfig::Algorithm::TAUBIN:
                success = taubinSmoothing(vertices, faces, config);
                break;
            case MeshSmoothingConfig::Algorithm::BILATERAL:
                success = bilateralSmoothing(vertices, faces, normals, config);
                break;
            case MeshSmoothingConfig::Algorithm::ANISOTROPIC:
                // Fallback to Taubin for now
                success = taubinSmoothing(vertices, faces, config);
                break;
        }

        if (!success) {
            pImpl->lastError = "Smoothing algorithm failed";
            return false;
        }

        pImpl->updateProgress(80);

        // Recompute normals if provided
        if (!normals.empty() && normals.size() == vertices.size()) {
            // Simple normal recomputation
            std::fill(normals.begin(), normals.end(), cv::Vec3f(0, 0, 0));

            for (const auto& face : faces) {
                if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                    cv::Vec3f normal = validation::computeTriangleNormal(
                        vertices[face[0]], vertices[face[1]], vertices[face[2]]);

                    normals[face[0]] += normal;
                    normals[face[1]] += normal;
                    normals[face[2]] += normal;
                }
            }

            // Normalize
            for (auto& normal : normals) {
                double length = cv::norm(normal);
                if (length > 1e-6) {
                    normal /= length;
                }
            }
        }

        pImpl->updateProgress(90);

        // Compute after metrics
        computeMeshQuality(vertices, faces, result.afterMetrics);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();

        result.qualityImprovement = result.afterMetrics.triangleQualityMean - result.beforeMetrics.triangleQualityMean;
        result.success = true;

        pImpl->updatePerformanceStats("smoothing", result.processingTimeMs);
        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh smoothing failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::decimateMesh(const std::vector<cv::Vec3f>& vertices,
                                const std::vector<cv::Vec3i>& faces,
                                const std::vector<cv::Vec3f>& normals,
                                std::vector<cv::Vec3f>& decimatedVertices,
                                std::vector<cv::Vec3i>& decimatedFaces,
                                std::vector<cv::Vec3f>& decimatedNormals,
                                const MeshDecimationConfig& config,
                                MeshOptimizationResult& result) {
    if (!validateMeshInput(vertices, faces)) {
        pImpl->lastError = "Invalid mesh input for decimation";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid decimation configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Mesh Decimation";

    try {
        // Compute before metrics
        computeMeshQuality(vertices, faces, result.beforeMetrics);

        pImpl->updateProgress(10);

        // Apply decimation algorithm
        bool success = false;
        switch (config.algorithm) {
            case MeshDecimationConfig::Algorithm::EDGE_COLLAPSE:
                success = edgeCollapseDecimation(vertices, faces, decimatedVertices, decimatedFaces, config);
                break;
            case MeshDecimationConfig::Algorithm::VERTEX_CLUSTERING:
                success = vertexClusteringDecimation(vertices, faces, decimatedVertices, decimatedFaces, config);
                break;
            case MeshDecimationConfig::Algorithm::PROGRESSIVE_MESH:
                // Fallback to edge collapse for now
                success = edgeCollapseDecimation(vertices, faces, decimatedVertices, decimatedFaces, config);
                break;
        }

        if (!success) {
            pImpl->lastError = "Decimation algorithm failed";
            return false;
        }

        pImpl->updateProgress(80);

        // Transfer normals if provided
        if (!normals.empty() && normals.size() == vertices.size()) {
            decimatedNormals.resize(decimatedVertices.size());
            // Simple normal transfer (could be improved with interpolation)
            for (size_t i = 0; i < decimatedVertices.size() && i < normals.size(); ++i) {
                decimatedNormals[i] = normals[i];
            }
        }

        pImpl->updateProgress(90);

        // Compute after metrics
        computeMeshQuality(decimatedVertices, decimatedFaces, result.afterMetrics);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();

        result.sizeReduction = 1.0 - (double)result.afterMetrics.numFaces / result.beforeMetrics.numFaces;
        result.success = true;

        pImpl->updatePerformanceStats("decimation", result.processingTimeMs);
        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh decimation failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::repairMesh(std::vector<cv::Vec3f>& vertices,
                              std::vector<cv::Vec3i>& faces,
                              std::vector<cv::Vec3f>& normals,
                              const MeshRepairConfig& config,
                              MeshOptimizationResult& result) {
    if (!validateMeshInput(vertices, faces)) {
        pImpl->lastError = "Invalid mesh input for repair";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid repair configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Mesh Repair";

    try {
        // Compute before metrics
        computeMeshQuality(vertices, faces, result.beforeMetrics);

        pImpl->updateProgress(10);

        int totalOperations = 0;
        int currentOperation = 0;

        // Count enabled operations
        if (config.removeDegenerateTriangles) totalOperations++;
        if (config.removeIsolatedComponents) totalOperations++;
        if (config.fixNonManifoldEdges || config.fixNonManifoldVertices) totalOperations++;
        if (config.fillHoles) totalOperations++;
        if (config.improveMeshQuality) totalOperations++;

        // Remove degenerate triangles
        if (config.removeDegenerateTriangles) {
            std::vector<cv::Vec3i> validFaces;
            validFaces.reserve(faces.size());

            for (const auto& face : faces) {
                if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                    if (!validation::isDegenerateTriangle(vertices[face[0]], vertices[face[1]], vertices[face[2]],
                                                         config.degenerateThreshold)) {
                        validFaces.push_back(face);
                    }
                }
            }

            faces = std::move(validFaces);
            currentOperation++;
            pImpl->updateProgress(10 + (currentOperation * 70) / totalOperations);
        }

        // Remove isolated components
        if (config.removeIsolatedComponents) {
            int removedComponents = 0;
            if (removeIsolatedComponents(vertices, faces, config.minComponentSize, removedComponents)) {
                currentOperation++;
                pImpl->updateProgress(10 + (currentOperation * 70) / totalOperations);
            }
        }

        // Fix non-manifold geometry
        if (config.fixNonManifoldEdges || config.fixNonManifoldVertices) {
            int fixedEdges = 0, fixedVertices = 0;
            if (fixNonManifoldGeometry(vertices, faces, fixedEdges, fixedVertices)) {
                currentOperation++;
                pImpl->updateProgress(10 + (currentOperation * 70) / totalOperations);
            }
        }

        // Fill holes
        if (config.fillHoles) {
            int filledHoles = 0;
            if (fillHoles(vertices, faces, config.maxHoleSize, filledHoles)) {
                currentOperation++;
                pImpl->updateProgress(10 + (currentOperation * 70) / totalOperations);
            }
        }

        // Improve mesh quality
        if (config.improveMeshQuality) {
            int improvedTriangles = 0;
            if (improveTriangleQuality(vertices, faces, 0.3, improvedTriangles)) {
                currentOperation++;
                pImpl->updateProgress(10 + (currentOperation * 70) / totalOperations);
            }
        }

        pImpl->updateProgress(80);

        // Recompute normals if provided
        if (!normals.empty()) {
            normals.resize(vertices.size());
            std::fill(normals.begin(), normals.end(), cv::Vec3f(0, 0, 0));

            for (const auto& face : faces) {
                if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                    cv::Vec3f normal = validation::computeTriangleNormal(
                        vertices[face[0]], vertices[face[1]], vertices[face[2]]);

                    normals[face[0]] += normal;
                    normals[face[1]] += normal;
                    normals[face[2]] += normal;
                }
            }

            for (auto& normal : normals) {
                double length = cv::norm(normal);
                if (length > 1e-6) {
                    normal /= length;
                }
            }
        }

        pImpl->updateProgress(90);

        // Compute after metrics
        computeMeshQuality(vertices, faces, result.afterMetrics);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();

        result.qualityImprovement = result.afterMetrics.getOverallQualityScore() - result.beforeMetrics.getOverallQualityScore();
        result.success = true;

        pImpl->updatePerformanceStats("repair", result.processingTimeMs);
        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh repair failed: " + std::string(e.what());
        return false;
    }
}

#ifdef OPEN3D_ENABLED
bool MeshOptimizer::smoothOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                    const MeshSmoothingConfig& config,
                                    MeshOptimizationResult& result) {
    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for smoothing";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Open3D Mesh Smoothing";

    try {
        // Apply Open3D smoothing
        switch (config.algorithm) {
            case MeshSmoothingConfig::Algorithm::LAPLACIAN:
                *mesh = *mesh->FilterSmoothLaplacian(config.iterations, config.lambda);
                break;
            case MeshSmoothingConfig::Algorithm::TAUBIN:
                *mesh = *mesh->FilterSmoothTaubin(config.iterations, config.lambda, config.mu);
                break;
            default:
                // Fallback to Taubin
                *mesh = *mesh->FilterSmoothTaubin(config.iterations, config.lambda, config.mu);
                break;
        }

        // Recompute normals
        mesh->ComputeVertexNormals();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("open3d_smoothing", result.processingTimeMs);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D mesh smoothing failed: " + std::string(e.what());
        return false;
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshOptimizer::decimateOpen3DMesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const MeshDecimationConfig& config,
    MeshOptimizationResult& result) {

    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for decimation";
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Open3D Mesh Decimation";

    try {
        size_t targetTriangles = config.targetTriangles;
        if (targetTriangles == 0) {
            targetTriangles = static_cast<size_t>(mesh->triangles_.size() * (1.0 - config.targetReduction));
        }

        auto decimatedMesh = mesh->SimplifyQuadricDecimation(
            static_cast<int>(targetTriangles),
            config.maxGeometricError,
            1.0  // boundary_weight (preserve boundaries)
        );

        if (!decimatedMesh || decimatedMesh->vertices_.empty()) {
            pImpl->lastError = "Open3D decimation produced empty mesh";
            return nullptr;
        }

        // Recompute normals
        decimatedMesh->ComputeVertexNormals();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        result.sizeReduction = 1.0 - (double)decimatedMesh->triangles_.size() / mesh->triangles_.size();

        pImpl->updatePerformanceStats("open3d_decimation", result.processingTimeMs);

        return decimatedMesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D mesh decimation failed: " + std::string(e.what());
        return nullptr;
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshOptimizer::simplifyMesh(
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
    const MeshDecimationConfig& config,
    MeshOptimizationResult& result) {

    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for simplification";
        return nullptr;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid simplification configuration";
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Advanced Mesh Simplification";

    try {
        // Store before metrics
        result.beforeMetrics.numVertices = mesh->vertices_.size();
        result.beforeMetrics.numFaces = mesh->triangles_.size();
        result.beforeMetrics.surfaceArea = mesh->GetSurfaceArea();

        pImpl->updateProgress(10);

        std::shared_ptr<open3d::geometry::TriangleMesh> simplifiedMesh;

        // Select algorithm based on mode
        switch (config.mode) {
            case MeshDecimationConfig::SimplifyMode::TRIANGLE_COUNT: {
                // Target specific triangle count
                size_t targetTriangles = config.targetTriangles;
                if (targetTriangles == 0) {
                    targetTriangles = static_cast<size_t>(mesh->triangles_.size() * (1.0 - config.targetReduction));
                }

                pImpl->updateProgress(30);

                if (config.algorithm == MeshDecimationConfig::Algorithm::QUADRIC_ERROR ||
                    config.algorithm == MeshDecimationConfig::Algorithm::EDGE_COLLAPSE) {
                    // Quadric Error Metric decimation (best quality)
                    simplifiedMesh = mesh->SimplifyQuadricDecimation(
                        static_cast<int>(targetTriangles),
                        config.maxGeometricError,
                        1.0
                    );
                } else {
                    // Fallback to quadric
                    simplifiedMesh = mesh->SimplifyQuadricDecimation(
                        static_cast<int>(targetTriangles),
                        config.maxGeometricError,
                        1.0
                    );
                }

                result.operation = "Quadric Decimation (Triangle Count)";
                break;
            }

            case MeshDecimationConfig::SimplifyMode::ACCURACY: {
                // Maintain geometric error < threshold (RECOMMENDED)
                // Use quadric decimation with maximum error constraint

                // Calculate target triangles based on error tolerance
                // For high precision (0.01mm), use conservative reduction
                double error_factor = config.maxGeometricError / 0.01;  // Normalize to 0.01mm baseline
                double adaptive_reduction = std::min(config.targetReduction, 0.5 + error_factor * 0.3);

                size_t targetTriangles = static_cast<size_t>(mesh->triangles_.size() * (1.0 - adaptive_reduction));
                targetTriangles = std::max(targetTriangles, static_cast<size_t>(1000));  // Minimum 1000 triangles

                pImpl->updateProgress(30);

                // Quadric decimation with error control
                simplifiedMesh = mesh->SimplifyQuadricDecimation(
                    static_cast<int>(targetTriangles),
                    config.maxGeometricError,  // Maximum allowed error
                    1.0  // boundary_weight
                );

                result.operation = "Quadric Decimation (Accuracy Mode, max_error=" +
                                 std::to_string(config.maxGeometricError) + "mm)";
                break;
            }

            case MeshDecimationConfig::SimplifyMode::ADAPTIVE: {
                // Curvature-based preservation (preserve detail in high-curvature areas)
                // This would require custom curvature analysis in production
                // For now, use quadric with conservative settings

                size_t targetTriangles = static_cast<size_t>(mesh->triangles_.size() * (1.0 - config.targetReduction * 0.7));

                pImpl->updateProgress(30);

                simplifiedMesh = mesh->SimplifyQuadricDecimation(
                    static_cast<int>(targetTriangles),
                    config.maxGeometricError * 0.5,  // Stricter error for adaptive mode
                    1.0  // boundary_weight
                );

                result.operation = "Adaptive Quadric Decimation";
                break;
            }

            case MeshDecimationConfig::SimplifyMode::FAST: {
                // Fast vertex clustering (Unlook fast mode, 2x faster)
                // Compute voxel size based on max error
                double voxel_size = config.maxGeometricError * 2.0;

                pImpl->updateProgress(30);

                simplifiedMesh = mesh->SimplifyVertexClustering(voxel_size);

                result.operation = "Fast Vertex Clustering (voxel=" +
                                 std::to_string(voxel_size) + "mm)";
                break;
            }

            default: {
                // Fallback to quadric decimation
                size_t targetTriangles = config.targetTriangles;
                if (targetTriangles == 0) {
                    targetTriangles = static_cast<size_t>(mesh->triangles_.size() * (1.0 - config.targetReduction));
                }
                simplifiedMesh = mesh->SimplifyQuadricDecimation(
                    static_cast<int>(targetTriangles),
                    config.maxGeometricError,
                    1.0
                );
                result.operation = "Quadric Decimation (Default)";
                break;
            }
        }

        pImpl->updateProgress(70);

        // Validation
        if (!simplifiedMesh || simplifiedMesh->vertices_.empty() || simplifiedMesh->triangles_.empty()) {
            pImpl->lastError = "Simplification produced empty or invalid mesh";
            return nullptr;
        }

        pImpl->updateProgress(80);

        // Clean up simplified mesh
        simplifiedMesh->RemoveDuplicatedVertices();
        simplifiedMesh->RemoveUnreferencedVertices();
        simplifiedMesh->RemoveDegenerateTriangles();

        pImpl->updateProgress(90);

        // Recompute normals
        simplifiedMesh->ComputeVertexNormals();

        // Compute after metrics
        result.afterMetrics.numVertices = simplifiedMesh->vertices_.size();
        result.afterMetrics.numFaces = simplifiedMesh->triangles_.size();
        result.afterMetrics.surfaceArea = simplifiedMesh->GetSurfaceArea();

        // Compute statistics
        result.sizeReduction = 1.0 - (double)result.afterMetrics.numFaces / (double)result.beforeMetrics.numFaces;

        // Performance metrics
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("mesh_simplification", result.processingTimeMs);
        pImpl->updateProgress(100);

        return simplifiedMesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh simplification failed: " + std::string(e.what());
        return nullptr;
    }
}

bool MeshOptimizer::repairOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                     const MeshRepairConfig& config,
                                     MeshOptimizationResult& result) {
    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for repair";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Open3D Mesh Repair";

    try {
        // Remove duplicated and unreferenced vertices
        if (config.removeDegenerateTriangles) {
            mesh->RemoveDuplicatedVertices();
            mesh->RemoveUnreferencedVertices();
            mesh->RemoveDegenerateTriangles();
        }

        // Remove non-manifold edges
        if (config.fixNonManifoldEdges) {
            mesh->RemoveNonManifoldEdges();
        }

        // Recompute normals
        mesh->ComputeVertexNormals();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("open3d_repair", result.processingTimeMs);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D mesh repair failed: " + std::string(e.what());
        return false;
    }
}
#endif

bool MeshOptimizer::optimizeFor3DPrinting(std::vector<cv::Vec3f>& vertices,
                                          std::vector<cv::Vec3i>& faces,
                                          std::vector<cv::Vec3f>& normals,
                                          double targetPrecision,
                                          MeshOptimizationResult& result) {
    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "3D Printing Optimization";

    try {
        // Step 1: Repair mesh to ensure it's watertight
        MeshRepairConfig repairConfig;
        repairConfig.makeWatertight = true;
        repairConfig.fillHoles = true;
        repairConfig.maxHoleSize = targetPrecision * 10.0; // Fill small holes
        repairConfig.removeIsolatedComponents = true;
        repairConfig.minComponentSize = 50;

        MeshOptimizationResult repairResult;
        if (!repairMesh(vertices, faces, normals, repairConfig, repairResult)) {
            pImpl->lastError = "Failed to repair mesh for 3D printing";
            return false;
        }

        pImpl->updateProgress(50);

        // Step 2: Smooth to improve surface quality
        MeshSmoothingConfig smoothConfig;
        smoothConfig.algorithm = MeshSmoothingConfig::Algorithm::TAUBIN;
        smoothConfig.iterations = 5; // Conservative smoothing
        smoothConfig.lambda = 0.3;
        smoothConfig.preserveBoundaries = true;

        MeshOptimizationResult smoothResult;
        if (!smoothMesh(vertices, faces, normals, smoothConfig, smoothResult)) {
            pImpl->lastError = "Failed to smooth mesh for 3D printing";
            return false;
        }

        pImpl->updateProgress(80);

        // Step 3: Final quality check
        MeshQualityMetrics finalMetrics;
        computeMeshQuality(vertices, faces, finalMetrics);

        if (!finalMetrics.isWatertight) {
            pImpl->lastError = "Mesh is not watertight after optimization";
            return false;
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        pImpl->updateProgress(100);
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "3D printing optimization failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::optimizeForManufacturing(std::vector<cv::Vec3f>& vertices,
                                             std::vector<cv::Vec3i>& faces,
                                             std::vector<cv::Vec3f>& normals,
                                             const std::map<std::string, double>& tolerances,
                                             MeshOptimizationResult& result) {
    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Manufacturing Optimization";

    try {
        double precision = 0.005; // Default 5 micron precision
        auto it = tolerances.find("precision");
        if (it != tolerances.end()) {
            precision = it->second;
        }

        // Manufacturing-specific optimization
        MeshRepairConfig repairConfig;
        repairConfig.makeWatertight = true;
        repairConfig.fillHoles = true;
        repairConfig.maxHoleSize = precision * 2.0; // Very small holes only
        repairConfig.removeIsolatedComponents = true;
        repairConfig.improveMeshQuality = true;

        MeshOptimizationResult repairResult;
        if (!repairMesh(vertices, faces, normals, repairConfig, repairResult)) {
            pImpl->lastError = "Failed to repair mesh for manufacturing";
            return false;
        }

        pImpl->updateProgress(70);

        // Minimal smoothing to preserve precision
        MeshSmoothingConfig smoothConfig;
        smoothConfig.algorithm = MeshSmoothingConfig::Algorithm::BILATERAL;
        smoothConfig.iterations = 2; // Very conservative
        smoothConfig.lambda = 0.1;
        smoothConfig.preserveFeatures = true;

        MeshOptimizationResult smoothResult;
        if (!smoothMesh(vertices, faces, normals, smoothConfig, smoothResult)) {
            // Continue even if smoothing fails for manufacturing
        }

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();
        result.success = true;

        pImpl->updateProgress(100);
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Manufacturing optimization failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::fillHoles(std::vector<cv::Vec3f>& vertices,
                             std::vector<cv::Vec3i>& faces,
                             double maxHoleSize,
                             int& filledHoles) {
    filledHoles = 0;

    try {
        // Find boundary edges and holes
        std::vector<std::pair<int, int>> boundaryEdges;
        std::vector<std::vector<int>> holeLoops;

        MeshValidator validator;
        int numHoles = validator.findBoundariesAndHoles(vertices, faces, boundaryEdges, holeLoops);

        for (const auto& holeLoop : holeLoops) {
            if (holeLoop.size() < 3) continue;

            // Compute hole size
            double holeSize = 0.0;
            for (size_t i = 0; i < holeLoop.size(); ++i) {
                int v1 = holeLoop[i];
                int v2 = holeLoop[(i + 1) % holeLoop.size()];
                if (v1 < vertices.size() && v2 < vertices.size()) {
                    holeSize += cv::norm(vertices[v1] - vertices[v2]);
                }
            }

            if (holeSize <= maxHoleSize) {
                if (fillHoleDelaunay(vertices, faces, holeLoop)) {
                    filledHoles++;
                }
            }
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Hole filling failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::removeIsolatedComponents(std::vector<cv::Vec3f>& vertices,
                                            std::vector<cv::Vec3i>& faces,
                                            size_t minComponentSize,
                                            int& removedComponents) {
    removedComponents = 0;

    try {
        MeshValidator validator;
        std::vector<int> connectedComponents;
        int numComponents = validator.analyzeConnectivity(vertices, faces, connectedComponents);

        // Count component sizes
        std::map<int, size_t> componentSizes;
        for (int comp : connectedComponents) {
            componentSizes[comp]++;
        }

        // Identify components to remove
        std::set<int> componentsToRemove;
        for (const auto& [comp, size] : componentSizes) {
            if (size < minComponentSize) {
                componentsToRemove.insert(comp);
                removedComponents++;
            }
        }

        if (componentsToRemove.empty()) {
            return true;
        }

        // Remove faces from small components
        std::vector<cv::Vec3i> filteredFaces;
        filteredFaces.reserve(faces.size());

        for (const auto& face : faces) {
            bool keepFace = true;
            for (int i = 0; i < 3; ++i) {
                if (face[i] < connectedComponents.size()) {
                    if (componentsToRemove.count(connectedComponents[face[i]])) {
                        keepFace = false;
                        break;
                    }
                }
            }
            if (keepFace) {
                filteredFaces.push_back(face);
            }
        }

        faces = std::move(filteredFaces);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Component removal failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::fixNonManifoldGeometry(std::vector<cv::Vec3f>& vertices,
                                           std::vector<cv::Vec3i>& faces,
                                           int& fixedEdges,
                                           int& fixedVertices) {
    fixedEdges = 0;
    fixedVertices = 0;

    try {
        // Simple non-manifold edge removal
        std::map<std::pair<int, int>, std::vector<int>> edgeToFaces;

        // Build edge-to-face mapping
        for (int faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
            const auto& face = faces[faceIdx];
            for (int i = 0; i < 3; ++i) {
                int v1 = face[i];
                int v2 = face[(i + 1) % 3];
                if (v1 > v2) std::swap(v1, v2);
                edgeToFaces[{v1, v2}].push_back(faceIdx);
            }
        }

        // Find and remove faces with non-manifold edges
        std::set<int> facesToRemove;
        for (const auto& [edge, faceList] : edgeToFaces) {
            if (faceList.size() > 2) {
                // Non-manifold edge - remove excess faces
                for (size_t i = 2; i < faceList.size(); ++i) {
                    facesToRemove.insert(faceList[i]);
                    fixedEdges++;
                }
            }
        }

        // Remove marked faces
        std::vector<cv::Vec3i> filteredFaces;
        filteredFaces.reserve(faces.size());

        for (int i = 0; i < faces.size(); ++i) {
            if (facesToRemove.find(i) == facesToRemove.end()) {
                filteredFaces.push_back(faces[i]);
            }
        }

        faces = std::move(filteredFaces);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Non-manifold geometry fix failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::improveTriangleQuality(std::vector<cv::Vec3f>& vertices,
                                           std::vector<cv::Vec3i>& faces,
                                           double targetQuality,
                                           int& improvedTriangles) {
    improvedTriangles = 0;

    try {
        // Simple quality improvement through edge flipping
        std::vector<std::vector<int>> vertexToFaces(vertices.size());

        // Build vertex-to-faces mapping
        for (int faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
            const auto& face = faces[faceIdx];
            for (int i = 0; i < 3; ++i) {
                if (face[i] < vertices.size()) {
                    vertexToFaces[face[i]].push_back(faceIdx);
                }
            }
        }

        // Simple vertex smoothing for quality improvement
        std::vector<cv::Vec3f> smoothedVertices = vertices;

        for (int vertexIdx = 0; vertexIdx < vertices.size(); ++vertexIdx) {
            const auto& adjacentFaces = vertexToFaces[vertexIdx];

            if (adjacentFaces.size() < 3) continue;

            // Check if any adjacent triangle has poor quality
            bool hasLowQuality = false;
            for (int faceIdx : adjacentFaces) {
                const auto& face = faces[faceIdx];
                if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                    double quality = pImpl->validator.computeTriangleQuality(
                        vertices[face[0]], vertices[face[1]], vertices[face[2]]);
                    if (quality < targetQuality) {
                        hasLowQuality = true;
                        break;
                    }
                }
            }

            if (hasLowQuality) {
                // Apply simple Laplacian smoothing to this vertex
                cv::Vec3f sum(0, 0, 0);
                int count = 0;

                for (int faceIdx : adjacentFaces) {
                    const auto& face = faces[faceIdx];
                    for (int i = 0; i < 3; ++i) {
                        if (face[i] != vertexIdx && face[i] < vertices.size()) {
                            sum += vertices[face[i]];
                            count++;
                        }
                    }
                }

                if (count > 0) {
                    cv::Vec3f centroid = sum / static_cast<float>(count);
                    smoothedVertices[vertexIdx] = vertices[vertexIdx] * 0.8f + centroid * 0.2f;
                    improvedTriangles++;
                }
            }
        }

        vertices = std::move(smoothedVertices);
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Triangle quality improvement failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::adaptiveRefinement(std::vector<cv::Vec3f>& vertices,
                                       std::vector<cv::Vec3i>& faces,
                                       std::vector<cv::Vec3f>& normals,
                                       const std::vector<cv::Vec3f>& pointCloud,
                                       double targetPrecision,
                                       MeshOptimizationResult& result) {
    // This is a placeholder for adaptive mesh refinement
    // A full implementation would require complex algorithms like:
    // - Point-to-mesh distance computation
    // - Adaptive subdivision
    // - Local remeshing

    pImpl->lastError = "Adaptive refinement not fully implemented";
    return false;
}

void MeshOptimizer::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string MeshOptimizer::getLastError() const {
    return pImpl->lastError;
}

void MeshOptimizer::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

std::map<std::string, double> MeshOptimizer::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(pImpl->statsMutex);
    return pImpl->performanceStats;
}

// Helper method implementations
bool MeshOptimizer::validateMeshInput(const std::vector<cv::Vec3f>& vertices,
                                     const std::vector<cv::Vec3i>& faces) {
    if (vertices.empty() || faces.empty()) {
        return false;
    }

    // Check face indices
    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            if (face[i] < 0 || face[i] >= vertices.size()) {
                return false;
            }
        }
    }

    return true;
}

void MeshOptimizer::computeMeshQuality(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      MeshQualityMetrics& metrics) {
    MeshValidationConfig config;
    pImpl->validator.validateMesh(vertices, faces, {}, config, metrics);
}

bool MeshOptimizer::laplacianSmoothing(std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const MeshSmoothingConfig& config) {
    try {
        buildVertexAdjacency(faces, vertices.size(), pImpl->vertexAdjacency);

        for (int iter = 0; iter < config.iterations; ++iter) {
            std::vector<cv::Vec3f> smoothedVertices = vertices;

            for (size_t i = 0; i < vertices.size(); ++i) {
                if (pImpl->vertexAdjacency[i].empty()) continue;

                cv::Vec3f laplacian = computeLaplacianCoordinate(i, vertices, pImpl->vertexAdjacency);
                smoothedVertices[i] = vertices[i] + laplacian * config.lambda;
            }

            vertices = std::move(smoothedVertices);
            pImpl->updateProgress(20 + (iter * 50) / config.iterations);
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Laplacian smoothing failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::taubinSmoothing(std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   const MeshSmoothingConfig& config) {
    try {
        std::vector<std::vector<int>> adjacency;
        buildVertexAdjacency(faces, vertices.size(), adjacency);

        for (int iter = 0; iter < config.iterations; ++iter) {
            std::vector<cv::Vec3f> tempVertices = vertices;

            // Inflation step
            for (size_t i = 0; i < vertices.size(); ++i) {
                if (adjacency[i].empty()) continue;

                cv::Vec3f laplacian = computeLaplacianCoordinate(i, vertices, adjacency);
                tempVertices[i] = vertices[i] + laplacian * config.lambda;
            }

            // Deflation step
            std::vector<cv::Vec3f> smoothedVertices = tempVertices;
            for (size_t i = 0; i < vertices.size(); ++i) {
                if (adjacency[i].empty()) continue;

                cv::Vec3f laplacian = computeLaplacianCoordinate(i, tempVertices, adjacency);
                smoothedVertices[i] = tempVertices[i] + laplacian * config.mu;
            }

            vertices = std::move(smoothedVertices);
            pImpl->updateProgress(20 + (iter * 50) / config.iterations);
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Taubin smoothing failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::bilateralSmoothing(std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const std::vector<cv::Vec3f>& normals,
                                      const MeshSmoothingConfig& config) {
    // Simplified bilateral smoothing
    return taubinSmoothing(vertices, faces, config);
}

bool MeshOptimizer::edgeCollapseDecimation(const std::vector<cv::Vec3f>& vertices,
                                          const std::vector<cv::Vec3i>& faces,
                                          std::vector<cv::Vec3f>& decimatedVertices,
                                          std::vector<cv::Vec3i>& decimatedFaces,
                                          const MeshDecimationConfig& config) {
    try {
        // Simplified edge collapse decimation
        // A full implementation would use quadric error metrics

        size_t targetFaces = config.targetTriangles;
        if (targetFaces == 0) {
            targetFaces = static_cast<size_t>(faces.size() * (1.0 - config.targetReduction));
        }

        // Simple vertex clustering approach for now
        return vertexClusteringDecimation(vertices, faces, decimatedVertices, decimatedFaces, config);

    } catch (const std::exception& e) {
        pImpl->lastError = "Edge collapse decimation failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::vertexClusteringDecimation(const std::vector<cv::Vec3f>& vertices,
                                              const std::vector<cv::Vec3i>& faces,
                                              std::vector<cv::Vec3f>& decimatedVertices,
                                              std::vector<cv::Vec3i>& decimatedFaces,
                                              const MeshDecimationConfig& config) {
    try {
        // Compute bounding box
        cv::Vec3f minBounds(FLT_MAX, FLT_MAX, FLT_MAX);
        cv::Vec3f maxBounds(-FLT_MAX, -FLT_MAX, -FLT_MAX);

        for (const auto& vertex : vertices) {
            minBounds[0] = std::min(minBounds[0], vertex[0]);
            minBounds[1] = std::min(minBounds[1], vertex[1]);
            minBounds[2] = std::min(minBounds[2], vertex[2]);

            maxBounds[0] = std::max(maxBounds[0], vertex[0]);
            maxBounds[1] = std::max(maxBounds[1], vertex[1]);
            maxBounds[2] = std::max(maxBounds[2], vertex[2]);
        }

        // Compute grid size based on target reduction
        double gridSize = std::pow(config.targetReduction, 1.0/3.0) *
                         std::min({maxBounds[0] - minBounds[0],
                                  maxBounds[1] - minBounds[1],
                                  maxBounds[2] - minBounds[2]}) / 10.0;

        // Group vertices into clusters
        std::map<std::tuple<int, int, int>, std::vector<int>> clusters;

        for (int i = 0; i < vertices.size(); ++i) {
            const auto& vertex = vertices[i];
            int gx = static_cast<int>((vertex[0] - minBounds[0]) / gridSize);
            int gy = static_cast<int>((vertex[1] - minBounds[1]) / gridSize);
            int gz = static_cast<int>((vertex[2] - minBounds[2]) / gridSize);

            clusters[{gx, gy, gz}].push_back(i);
        }

        // Create representative vertices for each cluster
        decimatedVertices.clear();
        decimatedVertices.reserve(clusters.size());

        std::map<int, int> oldToNewVertex;

        for (const auto& [grid, vertexIndices] : clusters) {
            // Compute centroid
            cv::Vec3f centroid(0, 0, 0);
            for (int idx : vertexIndices) {
                centroid += vertices[idx];
            }
            centroid /= static_cast<float>(vertexIndices.size());

            int newVertexIndex = decimatedVertices.size();
            decimatedVertices.push_back(centroid);

            // Map all old vertices to new vertex
            for (int idx : vertexIndices) {
                oldToNewVertex[idx] = newVertexIndex;
            }
        }

        // Update faces
        decimatedFaces.clear();
        decimatedFaces.reserve(faces.size());

        for (const auto& face : faces) {
            cv::Vec3i newFace(
                oldToNewVertex[face[0]],
                oldToNewVertex[face[1]],
                oldToNewVertex[face[2]]
            );

            // Skip degenerate faces
            if (newFace[0] != newFace[1] && newFace[1] != newFace[2] && newFace[0] != newFace[2]) {
                decimatedFaces.push_back(newFace);
            }
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Vertex clustering decimation failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::fillHoleDelaunay(std::vector<cv::Vec3f>& vertices,
                                     std::vector<cv::Vec3i>& faces,
                                     const std::vector<int>& holeLoop) {
    if (holeLoop.size() < 3) return false;

    try {
        // Simple fan triangulation for now
        // A full implementation would use constrained Delaunay triangulation

        int centerIdx = holeLoop[0];
        for (size_t i = 1; i < holeLoop.size() - 1; ++i) {
            cv::Vec3i newFace(centerIdx, holeLoop[i], holeLoop[i + 1]);
            faces.push_back(newFace);
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Hole filling failed: " + std::string(e.what());
        return false;
    }
}

bool MeshOptimizer::fillHoleAdvancingFront(std::vector<cv::Vec3f>& vertices,
                                           std::vector<cv::Vec3i>& faces,
                                           const std::vector<int>& holeLoop) {
    // Placeholder for advancing front algorithm
    return fillHoleDelaunay(vertices, faces, holeLoop);
}

void MeshOptimizer::buildVertexAdjacency(const std::vector<cv::Vec3i>& faces,
                                        size_t numVertices,
                                        std::vector<std::vector<int>>& adjacency) {
    adjacency.clear();
    adjacency.resize(numVertices);

    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % 3];

            if (v1 < numVertices && v2 < numVertices) {
                adjacency[v1].push_back(v2);
                adjacency[v2].push_back(v1);
            }
        }
    }

    // Remove duplicates and sort
    for (auto& adj : adjacency) {
        std::sort(adj.begin(), adj.end());
        adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
    }
}

double MeshOptimizer::computeEdgeCollapseCost(const cv::Vec3f& v1, const cv::Vec3f& v2,
                                             const std::vector<cv::Vec3f>& vertices,
                                             const std::vector<cv::Vec3i>& faces) {
    // Simplified cost computation (edge length)
    return cv::norm(v1 - v2);
}

cv::Vec3f MeshOptimizer::computeLaplacianCoordinate(int vertexIndex,
                                                   const std::vector<cv::Vec3f>& vertices,
                                                   const std::vector<std::vector<int>>& adjacency) {
    if (vertexIndex >= adjacency.size() || adjacency[vertexIndex].empty()) {
        return cv::Vec3f(0, 0, 0);
    }

    cv::Vec3f sum(0, 0, 0);
    for (int neighbor : adjacency[vertexIndex]) {
        if (neighbor < vertices.size()) {
            sum += vertices[neighbor];
        }
    }

    cv::Vec3f centroid = sum / static_cast<float>(adjacency[vertexIndex].size());
    return centroid - vertices[vertexIndex];
}

// Configuration validation implementations
bool MeshSmoothingConfig::validate() const {
    if (iterations <= 0) return false;
    if (lambda < 0.0 || lambda > 1.0) return false;
    if (mu >= 0.0) return false; // mu should be negative for Taubin
    if (featureThreshold < 0.0 || featureThreshold > 180.0) return false;
    return true;
}

std::string MeshSmoothingConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Smoothing Configuration:\n";
    ss << "  Algorithm: ";
    switch (algorithm) {
        case Algorithm::LAPLACIAN: ss << "Laplacian"; break;
        case Algorithm::TAUBIN: ss << "Taubin"; break;
        case Algorithm::BILATERAL: ss << "Bilateral"; break;
        case Algorithm::ANISOTROPIC: ss << "Anisotropic"; break;
    }
    ss << "\n  Iterations: " << iterations;
    ss << "\n  Lambda: " << lambda;
    ss << "\n  Mu: " << mu;
    ss << "\n  Preserve boundaries: " << (preserveBoundaries ? "Yes" : "No");
    ss << "\n  Preserve features: " << (preserveFeatures ? "Yes" : "No");
    return ss.str();
}

bool MeshDecimationConfig::validate() const {
    if (targetReduction < 0.0 || targetReduction > 1.0) return false;
    if (maxError <= 0.0) return false;
    if (maxGeometricError <= 0.0) return false;
    if (qualityThreshold < 0.0 || qualityThreshold > 1.0) return false;
    if (featureAngle < 0.0 || featureAngle > 180.0) return false;
    if (curvatureThreshold < 0.0) return false;
    return true;
}

std::string MeshDecimationConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Decimation/Simplification Configuration:\n";
    ss << "  Algorithm: ";
    switch (algorithm) {
        case Algorithm::EDGE_COLLAPSE: ss << "Edge Collapse"; break;
        case Algorithm::VERTEX_CLUSTERING: ss << "Vertex Clustering"; break;
        case Algorithm::PROGRESSIVE_MESH: ss << "Progressive Mesh"; break;
        case Algorithm::QUADRIC_ERROR: ss << "Quadric Error Metric"; break;
        case Algorithm::ADAPTIVE: ss << "Adaptive Curvature-based"; break;
    }
    ss << "\n  Mode: ";
    switch (mode) {
        case SimplifyMode::TRIANGLE_COUNT: ss << "Triangle Count Target"; break;
        case SimplifyMode::ACCURACY: ss << "Accuracy Preservation"; break;
        case SimplifyMode::ADAPTIVE: ss << "Adaptive Feature Preservation"; break;
        case SimplifyMode::FAST: ss << "Fast Vertex Clustering"; break;
    }
    ss << "\n  Target reduction: " << std::fixed << std::setprecision(1) << (targetReduction * 100.0) << "%";
    if (targetTriangles > 0) {
        ss << "\n  Target triangles: " << targetTriangles;
    }
    ss << "\n  Max geometric error: " << std::fixed << std::setprecision(4) << maxGeometricError << " mm";
    ss << "\n  Preserve boundaries: " << (preserveBoundaries ? "Yes" : "No");
    ss << "\n  Preserve topology: " << (preserveTopology ? "Yes" : "No");
    ss << "\n  Quality threshold: " << std::fixed << std::setprecision(2) << qualityThreshold;
    if (preserveFeatures) {
        ss << "\n  Feature angle: " << std::fixed << std::setprecision(1) << featureAngle << "°";
    }
    ss << "\n  Parallel processing: " << (parallel ? "Enabled" : "Disabled");
    return ss.str();
}

bool MeshRepairConfig::validate() const {
    if (maxHoleSize <= 0.0) return false;
    if (degenerateThreshold <= 0.0) return false;
    return true;
}

std::string MeshRepairConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Repair Configuration:\n";
    ss << "  Fill holes: " << (fillHoles ? "Yes" : "No");
    if (fillHoles) {
        ss << " (max size: " << maxHoleSize << " mm)";
    }
    ss << "\n  Remove isolated components: " << (removeIsolatedComponents ? "Yes" : "No");
    ss << "\n  Fix non-manifold edges: " << (fixNonManifoldEdges ? "Yes" : "No");
    ss << "\n  Make watertight: " << (makeWatertight ? "Yes" : "No");
    return ss.str();
}

std::string MeshOptimizationResult::toString() const {
    std::stringstream ss;
    ss << "Mesh Optimization Result: " << operation << "\n";
    ss << "  Success: " << (success ? "Yes" : "No") << "\n";
    ss << "  Processing time: " << processingTimeMs << " ms\n";
    ss << "  Quality improvement: " << std::fixed << std::setprecision(3) << qualityImprovement << "\n";
    ss << "  Size reduction: " << std::fixed << std::setprecision(1) << (sizeReduction * 100.0) << "%\n";
    return ss.str();
}

// Namespace optimization implementations
namespace optimization {

bool computeOptimalDecimationParams(const std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   double targetPrecision,
                                   double& targetReduction,
                                   double& maxError) {
    // Heuristic computation based on mesh complexity and precision requirements
    targetReduction = std::min(0.8, targetPrecision / 0.001); // More reduction for higher precision
    maxError = targetPrecision * 0.5; // Error should be half of target precision
    return true;
}

double estimateProcessingTime(size_t numVertices, size_t numFaces,
                             const std::string& operation, bool isARM64) {
    double baseTime = 0.0;

    if (operation == "smoothing") {
        baseTime = numVertices * 0.001; // 1 microsecond per vertex
    } else if (operation == "decimation") {
        baseTime = numFaces * 0.01; // 10 microseconds per face
    } else if (operation == "repair") {
        baseTime = numFaces * 0.005; // 5 microseconds per face
    }

    // ARM64 performance factor (CM5 is 2-3x faster than CM4)
    if (isARM64) {
        baseTime *= 0.4; // 2.5x speedup
    }

    return baseTime;
}

bool isOptimizationRecommended(const MeshQualityMetrics& metrics,
                              double targetPrecision) {
    // Recommend optimization if:
    // - Not watertight and precision is important
    // - Poor triangle quality
    // - High surface deviation
    return !metrics.isWatertight ||
           metrics.triangleQualityMean < 0.5 ||
           metrics.surfaceDeviation > targetPrecision;
}

} // namespace optimization

} // namespace mesh
} // namespace unlook