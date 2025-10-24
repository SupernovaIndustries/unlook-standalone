#include "unlook/pointcloud/MeshCleaner.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <set>
#include <map>
#include <queue>

#ifdef OPEN3D_ENABLED
#include <open3d/Open3D.h>
#include <open3d/geometry/TriangleMesh.h>
#endif

namespace unlook {
namespace pointcloud {

// MeshCleanerSettings implementation
bool MeshCleanerSettings::validate() const {
    if (minComponentTriangles <= 0) return false;
    if (minComponentArea <= 0.0) return false;
    if (degenerateThreshold <= 0.0) return false;
    return true;
}

std::string MeshCleanerSettings::toString() const {
    std::stringstream ss;
    ss << "Mesh Cleaner Settings:\n";
    ss << "  Mode: ";
    switch (mode) {
        case FilterMode::KEEP_LARGEST: ss << "KEEP_LARGEST"; break;
        case FilterMode::SIZE_THRESHOLD: ss << "SIZE_THRESHOLD"; break;
        case FilterMode::MANUAL_SELECT: ss << "MANUAL_SELECT"; break;
    }
    ss << "\n  Min component triangles: " << minComponentTriangles;
    ss << "\n  Min component area: " << minComponentArea << " mm²";
    ss << "\n  Remove isolated vertices: " << (removeIsolatedVertices ? "Yes" : "No");
    ss << "\n  Remove duplicate vertices: " << (removeDuplicateVertices ? "Yes" : "No");
    ss << "\n  Remove degenerate triangles: " << (removeDegenerateTriangles ? "Yes" : "No");
    ss << "\n  Fix non-manifold: " << (fixNonManifold ? "Yes" : "No");
    return ss.str();
}

// SimplificationSettings implementation
bool SimplificationSettings::validate() const {
    if (mode == Mode::TRIANGLE_COUNT && targetTriangles <= 0) return false;
    if (mode == Mode::REDUCTION_RATIO && (reductionRatio <= 0.0 || reductionRatio >= 1.0)) return false;
    if (mode == Mode::ACCURACY && max_geometric_error <= 0.0) return false;
    return true;
}

std::string SimplificationSettings::toString() const {
    std::stringstream ss;
    ss << "Mesh Simplification Settings:\n";
    ss << "  Mode: ";
    switch (mode) {
        case Mode::TRIANGLE_COUNT: ss << "TRIANGLE_COUNT"; break;
        case Mode::REDUCTION_RATIO: ss << "REDUCTION_RATIO"; break;
        case Mode::ACCURACY: ss << "ACCURACY"; break;
    }
    if (mode == Mode::TRIANGLE_COUNT) {
        ss << "\n  Target triangles: " << targetTriangles;
    } else if (mode == Mode::REDUCTION_RATIO) {
        ss << "\n  Reduction ratio: " << (reductionRatio * 100.0) << "%";
    } else if (mode == Mode::ACCURACY) {
        ss << "\n  Max geometric error: " << max_geometric_error << " mm";
    }
    ss << "\n  Preserve boundaries: " << (preserveBoundaries ? "Yes" : "No");
    ss << "\n  Preserve topology: " << (preserveTopology ? "Yes" : "No");
    return ss.str();
}

SimplificationSettings SimplificationSettings::forTargetTriangles(size_t triangles) {
    SimplificationSettings settings;
    settings.mode = Mode::TRIANGLE_COUNT;
    settings.targetTriangles = triangles;
    return settings;
}

SimplificationSettings SimplificationSettings::forReductionRatio(double ratio) {
    SimplificationSettings settings;
    settings.mode = Mode::REDUCTION_RATIO;
    settings.reductionRatio = ratio;
    return settings;
}

SimplificationSettings SimplificationSettings::forGeometricAccuracy(double maxErrorMm) {
    SimplificationSettings settings;
    settings.mode = Mode::ACCURACY;
    settings.max_geometric_error = maxErrorMm;
    return settings;
}

// MeshCleaningResult implementation
std::string MeshCleaningResult::toString() const {
    std::stringstream ss;
    ss << "Mesh Cleaning Result:\n";
    ss << "  Success: " << (success ? "Yes" : "No") << "\n";
    if (success) {
        ss << "  Input: " << inputVertices << " vertices, " << inputTriangles << " triangles\n";
        ss << "  Output: " << outputVertices << " vertices, " << outputTriangles << " triangles\n";
        ss << "  Components removed: " << componentsRemoved << "\n";
        ss << "  Vertices removed: " << verticesRemoved << "\n";
        ss << "  Triangles removed: " << trianglesRemoved << "\n";
        ss << "  Size reduction: " << std::fixed << std::setprecision(1) << sizeReduction << "%\n";
        ss << "  Processing time: " << std::fixed << std::setprecision(2) << processingTimeMs << " ms\n";
    } else {
        ss << "  Error: " << errorMessage << "\n";
    }
    return ss.str();
}

// MeshCleaner::Impl
class MeshCleaner::Impl {
public:
    std::string lastError;
    MeshCleaningResult lastResult;
    std::function<void(int)> progressCallback;
    bool arm64Optimized = false;

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
};

MeshCleaner::MeshCleaner() : pImpl(std::make_unique<Impl>()) {}
MeshCleaner::~MeshCleaner() = default;

#ifdef OPEN3D_ENABLED
MeshCleaningResult MeshCleaner::removeSmallObjects(
    open3d::geometry::TriangleMesh& mesh,
    const MeshCleanerSettings& settings) {

    if (!settings.validate()) {
        MeshCleaningResult result;
        result.success = false;
        result.errorMessage = "Invalid mesh cleaner settings";
        pImpl->lastError = result.errorMessage;
        return result;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    MeshCleaningResult result;
    result.inputVertices = mesh.vertices_.size();
    result.inputTriangles = mesh.triangles_.size();

    try {
        pImpl->updateProgress(10);

        // Find connected components
        std::vector<size_t> componentSizes;
        int numComponents = analyzeConnectedComponents(mesh, componentSizes);

        std::cout << "[MeshCleaner] Found " << numComponents << " connected components" << std::endl;

        pImpl->updateProgress(30);

        if (settings.mode == MeshCleanerSettings::FilterMode::KEEP_LARGEST) {
            // Keep only the largest component
            if (numComponents > 1) {
                size_t largestIdx = std::distance(
                    componentSizes.begin(),
                    std::max_element(componentSizes.begin(), componentSizes.end())
                );

                auto filteredMesh = selectComponent(mesh, largestIdx);
                if (filteredMesh) {
                    mesh = *filteredMesh;
                    result.componentsRemoved = numComponents - 1;
                    std::cout << "[MeshCleaner] Kept largest component (" << componentSizes[largestIdx] << " triangles)" << std::endl;
                }
            }
        } else if (settings.mode == MeshCleanerSettings::FilterMode::SIZE_THRESHOLD) {
            // Remove components smaller than threshold
            auto cluster_result = mesh.ClusterConnectedTriangles();
            auto& cluster_indices = std::get<0>(cluster_result);
            auto& triangles_per_cluster = std::get<1>(cluster_result);

            // Collect triangle indices to keep
            std::vector<size_t> triangles_to_keep;
            for (size_t i = 0; i < cluster_indices.size(); ++i) {
                int cluster_id = cluster_indices[i];
                if (cluster_id >= 0 && static_cast<size_t>(cluster_id) < triangles_per_cluster.size()) {
                    if (triangles_per_cluster[cluster_id] >= settings.minComponentTriangles) {
                        triangles_to_keep.push_back(i);
                    }
                }
            }

            // Create filtered mesh
            mesh = *mesh.SelectByIndex(triangles_to_keep);
            result.componentsRemoved = numComponents - static_cast<int>(triangles_to_keep.size());

            std::cout << "[MeshCleaner] Removed " << result.componentsRemoved << " small components" << std::endl;
        }

        pImpl->updateProgress(50);

        // Remove isolated vertices
        if (settings.removeIsolatedVertices) {
            mesh.RemoveUnreferencedVertices();
        }

        pImpl->updateProgress(60);

        // Remove duplicate vertices
        if (settings.removeDuplicateVertices) {
            mesh.RemoveDuplicatedVertices();
        }

        pImpl->updateProgress(70);

        // Remove degenerate triangles
        if (settings.removeDegenerateTriangles) {
            mesh.RemoveDegenerateTriangles();
        }

        pImpl->updateProgress(80);

        // Fix non-manifold geometry
        if (settings.fixNonManifold) {
            mesh.RemoveNonManifoldEdges();
        }

        pImpl->updateProgress(90);

        // Recompute normals
        mesh.ComputeVertexNormals();

        // Compute result metrics
        result.outputVertices = mesh.vertices_.size();
        result.outputTriangles = mesh.triangles_.size();
        result.verticesRemoved = result.inputVertices - result.outputVertices;
        result.trianglesRemoved = result.inputTriangles - result.outputTriangles;
        result.sizeReduction = 100.0 * (double)result.trianglesRemoved / result.inputTriangles;

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processingTimeMs = duration.count();

        result.success = true;
        pImpl->lastResult = result;

        std::cout << "[MeshCleaner] Cleaning complete: "
                  << result.verticesRemoved << " vertices removed, "
                  << result.trianglesRemoved << " triangles removed ("
                  << result.processingTimeMs << " ms)" << std::endl;

        pImpl->updateProgress(100);

        return result;

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "Mesh cleaning exception: " + std::string(e.what());
        pImpl->lastError = result.errorMessage;
        pImpl->lastResult = result;
        return result;
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshCleaner::simplify(
    const open3d::geometry::TriangleMesh& mesh,
    const SimplificationSettings& settings) {

    if (!settings.validate()) {
        pImpl->lastError = "Invalid simplification settings";
        return nullptr;
    }

    try {
        pImpl->updateProgress(10);

        size_t targetTriangles = settings.targetTriangles;

        if (settings.mode == SimplificationSettings::Mode::REDUCTION_RATIO) {
            targetTriangles = static_cast<size_t>(mesh.triangles_.size() * (1.0 - settings.reductionRatio));
        } else if (settings.mode == SimplificationSettings::Mode::ACCURACY) {
            // Estimate target based on geometric error
            // This is a heuristic - in practice, you'd need iterative simplification
            double errorRatio = settings.max_geometric_error / 0.01; // Normalized to 0.01mm baseline
            targetTriangles = static_cast<size_t>(mesh.triangles_.size() * std::max(0.1, std::min(0.9, errorRatio)));
        }

        std::cout << "[MeshCleaner] Simplifying from " << mesh.triangles_.size()
                  << " to " << targetTriangles << " triangles..." << std::endl;

        pImpl->updateProgress(30);

        // Perform quadric decimation
        // Open3D SimplifyQuadricDecimation signature: (target_number_of_triangles, max_error=1e-10, boundary_weight=1.0)
        auto simplified = mesh.SimplifyQuadricDecimation(
            static_cast<int>(targetTriangles),
            settings.max_geometric_error / 1000.0,  // Convert mm to meters
            1.0  // boundary weight
        );

        if (!simplified || simplified->vertices_.empty()) {
            pImpl->lastError = "Simplification produced empty mesh";
            return nullptr;
        }

        pImpl->updateProgress(80);

        // Recompute normals
        simplified->ComputeVertexNormals();

        std::cout << "[MeshCleaner] Simplified to " << simplified->triangles_.size() << " triangles" << std::endl;

        pImpl->updateProgress(100);

        return simplified;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh simplification failed: " + std::string(e.what());
        return nullptr;
    }
}

MeshCleaningResult MeshCleaner::cleanAndOptimize(
    open3d::geometry::TriangleMesh& mesh,
    const MeshCleanerSettings& cleanSettings,
    const SimplificationSettings* simplifySettings) {

    auto startTime = std::chrono::high_resolution_clock::now();

    // Step 1: Clean mesh
    auto cleanResult = removeSmallObjects(mesh, cleanSettings);

    if (!cleanResult.success) {
        return cleanResult;
    }

    // Step 2: Optional simplification
    if (simplifySettings != nullptr) {
        auto simplified = simplify(mesh, *simplifySettings);
        if (simplified) {
            mesh = *simplified;
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    cleanResult.processingTimeMs = duration.count();

    return cleanResult;
}

int MeshCleaner::analyzeConnectedComponents(
    const open3d::geometry::TriangleMesh& mesh,
    std::vector<size_t>& componentSizes) {

    componentSizes.clear();

    try {
        auto [cluster_indices, triangles_per_cluster, vertex_per_cluster] = mesh.ClusterConnectedTriangles();

        componentSizes.resize(triangles_per_cluster.size());
        for (size_t i = 0; i < triangles_per_cluster.size(); ++i) {
            componentSizes[i] = triangles_per_cluster[i];
        }

        return triangles_per_cluster.size();

    } catch (const std::exception& e) {
        pImpl->lastError = "Component analysis failed: " + std::string(e.what());
        return 0;
    }
}

std::shared_ptr<open3d::geometry::TriangleMesh> MeshCleaner::selectComponent(
    const open3d::geometry::TriangleMesh& mesh,
    size_t componentIndex) {

    try {
        auto [cluster_indices, triangles_per_cluster, vertex_per_cluster] = mesh.ClusterConnectedTriangles();

        if (componentIndex >= triangles_per_cluster.size()) {
            pImpl->lastError = "Component index out of range";
            return nullptr;
        }

        // Collect triangle indices for selected component
        std::vector<size_t> selected_triangles;
        for (size_t i = 0; i < cluster_indices.size(); ++i) {
            if (static_cast<size_t>(cluster_indices[i]) == componentIndex) {
                selected_triangles.push_back(i);
            }
        }

        if (selected_triangles.empty()) {
            pImpl->lastError = "No triangles in selected component";
            return nullptr;
        }

        return mesh.SelectByIndex(selected_triangles);

    } catch (const std::exception& e) {
        pImpl->lastError = "Component selection failed: " + std::string(e.what());
        return nullptr;
    }
}

bool MeshCleaner::repairNonManifold(open3d::geometry::TriangleMesh& mesh) {
    try {
        mesh.RemoveNonManifoldEdges();
        mesh.RemoveDuplicatedVertices();
        mesh.RemoveUnreferencedVertices();
        mesh.RemoveDegenerateTriangles();
        mesh.ComputeVertexNormals();
        return true;
    } catch (const std::exception& e) {
        pImpl->lastError = "Non-manifold repair failed: " + std::string(e.what());
        return false;
    }
}

bool MeshCleaner::validateMeshQuality(
    const open3d::geometry::TriangleMesh& mesh,
    MeshCleaningResult& result) {

    result = {};

    try {
        result.outputVertices = mesh.vertices_.size();
        result.outputTriangles = mesh.triangles_.size();

        // Check if watertight
        bool isWatertight = mesh.IsWatertight();

        // Check if manifold
        bool isManifold = mesh.IsVertexManifold() && mesh.IsEdgeManifold();

        result.success = isWatertight && isManifold;

        return result.success;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh validation failed: " + std::string(e.what());
        result.success = false;
        result.errorMessage = pImpl->lastError;
        return false;
    }
}
#endif

MeshCleaningResult MeshCleaner::cleanFallback(
    std::vector<cv::Vec3f>& vertices,
    std::vector<cv::Vec3i>& faces,
    const MeshCleanerSettings& settings) {

    MeshCleaningResult result;
    result.success = false;
    result.errorMessage = "Mesh cleaning requires Open3D - fallback not implemented";
    pImpl->lastError = result.errorMessage;
    return result;
}

void MeshCleaner::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string MeshCleaner::getLastError() const {
    return pImpl->lastError;
}

MeshCleaningResult MeshCleaner::getLastResult() const {
    return pImpl->lastResult;
}

void MeshCleaner::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

// Helper methods
void MeshCleaner::buildConnectivityGraph(
    const std::vector<cv::Vec3i>& faces,
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

    // Remove duplicates
    for (auto& adj : adjacency) {
        std::sort(adj.begin(), adj.end());
        adj.erase(std::unique(adj.begin(), adj.end()), adj.end());
    }
}

int MeshCleaner::findConnectedComponents(
    const std::vector<cv::Vec3i>& faces,
    size_t numVertices,
    std::vector<int>& componentLabels) {

    std::vector<std::vector<int>> adjacency;
    buildConnectivityGraph(faces, numVertices, adjacency);

    componentLabels.assign(numVertices, -1);
    int numComponents = 0;

    for (size_t i = 0; i < numVertices; ++i) {
        if (componentLabels[i] == -1) {
            // BFS to label component
            std::queue<int> queue;
            queue.push(i);
            componentLabels[i] = numComponents;

            while (!queue.empty()) {
                int v = queue.front();
                queue.pop();

                for (int neighbor : adjacency[v]) {
                    if (componentLabels[neighbor] == -1) {
                        componentLabels[neighbor] = numComponents;
                        queue.push(neighbor);
                    }
                }
            }

            numComponents++;
        }
    }

    return numComponents;
}

} // namespace pointcloud
} // namespace unlook
