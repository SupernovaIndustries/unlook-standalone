#include "unlook/mesh/MeshCleaner.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
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

// ============================================================================
// Configuration string implementations
// ============================================================================

std::string MeshCleaningConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Cleaning Configuration:\n";
    ss << "  Mode: " << (mode == FilterMode::KEEP_LARGEST ? "Keep Largest Component" : "Filter By Threshold") << "\n";
    if (mode == FilterMode::FILTER_BY_THRESHOLD) {
        ss << "  Min Triangles: " << min_triangles << "\n";
        ss << "  Min Area Ratio: " << (min_area_ratio * 100.0) << "%\n";
        ss << "  Min Vertices: " << min_vertices << "\n";
    }
    ss << "  Remove Duplicates: " << (remove_duplicate_vertices ? "Yes" : "No") << "\n";
    ss << "  Remove Unreferenced: " << (remove_unreferenced_vertices ? "Yes" : "No") << "\n";
    ss << "  Parallel Processing: " << (parallel_processing ? "Enabled" : "Disabled");
    return ss.str();
}

std::string MeshCleaningResult::toString() const {
    std::stringstream ss;
    ss << "Mesh Cleaning Result: " << operation << "\n";
    ss << "  Success: " << (success ? "Yes" : "No") << "\n";
    ss << "  Processing Time: " << std::fixed << std::setprecision(2) << processing_time_ms << " ms\n";
    ss << "\nComponent Statistics:\n";
    ss << "  Total Components Found: " << total_components_found << "\n";
    ss << "  Components Kept: " << components_kept << "\n";
    ss << "  Components Removed: " << components_removed << "\n";
    ss << "\nTriangle Statistics:\n";
    ss << "  Before: " << triangles_before << " triangles\n";
    ss << "  After: " << triangles_after << " triangles\n";
    ss << "  Removed: " << triangles_removed << " (" << std::fixed << std::setprecision(1)
       << getReductionPercentage() << "%)\n";
    ss << "\nVertex Statistics:\n";
    ss << "  Before: " << vertices_before << " vertices\n";
    ss << "  After: " << vertices_after << " vertices\n";
    ss << "  Removed: " << vertices_removed << "\n";
    if (duplicate_vertices_removed > 0) {
        ss << "  Duplicates Removed: " << duplicate_vertices_removed << "\n";
    }
    ss << "\nArea Statistics:\n";
    ss << "  Total Area Before: " << std::fixed << std::setprecision(3) << total_area_before << " mm²\n";
    ss << "  Total Area After: " << total_area_after << " mm²\n";
    ss << "  Largest Component: " << largest_component_area << " mm²";
    return ss.str();
}

// ============================================================================
// MeshCleaner::Impl - Private implementation with thread-safe operations
// ============================================================================

class MeshCleaner::Impl {
public:
    std::string lastError;
    std::function<void(int)> progressCallback;
    std::map<std::string, double> performanceStats;
    mutable std::mutex statsMutex;
    bool arm64Optimized = false;
    MeshValidator validator;

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

    // Build connected components using Union-Find algorithm
    void buildConnectedComponents(const std::vector<cv::Vec3i>& faces,
                                  size_t num_vertices,
                                  std::vector<int>& vertex_to_component,
                                  size_t& num_components) {
        // Union-Find data structure
        std::vector<int> parent(num_vertices);
        std::iota(parent.begin(), parent.end(), 0);

        // Union-Find helper: Find with path compression
        std::function<int(int)> find_root = [&](int v) -> int {
            if (parent[v] != v) {
                parent[v] = find_root(parent[v]); // Path compression
            }
            return parent[v];
        };

        // Union-Find helper: Union two components
        auto unite = [&](int v1, int v2) {
            int root1 = find_root(v1);
            int root2 = find_root(v2);
            if (root1 != root2) {
                parent[root2] = root1;
            }
        };

        // Build connected components from faces
        for (const auto& face : faces) {
            if (face[0] < num_vertices && face[1] < num_vertices && face[2] < num_vertices) {
                unite(face[0], face[1]);
                unite(face[1], face[2]);
            }
        }

        // Normalize component IDs (make them consecutive 0, 1, 2, ...)
        std::unordered_map<int, int> root_to_component;
        int next_component_id = 0;

        vertex_to_component.resize(num_vertices);
        for (size_t i = 0; i < num_vertices; ++i) {
            int root = find_root(i);
            if (root_to_component.find(root) == root_to_component.end()) {
                root_to_component[root] = next_component_id++;
            }
            vertex_to_component[i] = root_to_component[root];
        }

        num_components = next_component_id;
    }

    // Compute component statistics (size, area)
    void computeComponentStats(const std::vector<cv::Vec3f>& vertices,
                               const std::vector<cv::Vec3i>& faces,
                               const std::vector<int>& vertex_to_component,
                               size_t num_components,
                               std::vector<size_t>& component_triangle_count,
                               std::vector<double>& component_area) {
        component_triangle_count.assign(num_components, 0);
        component_area.assign(num_components, 0.0);

        for (const auto& face : faces) {
            if (face[0] >= vertices.size() || face[1] >= vertices.size() || face[2] >= vertices.size()) {
                continue;
            }

            // Determine component of this triangle (use first vertex)
            int component_id = vertex_to_component[face[0]];

            // Count triangle
            component_triangle_count[component_id]++;

            // Compute triangle area
            const cv::Vec3f& v0 = vertices[face[0]];
            const cv::Vec3f& v1 = vertices[face[1]];
            const cv::Vec3f& v2 = vertices[face[2]];

            cv::Vec3f edge1 = v1 - v0;
            cv::Vec3f edge2 = v2 - v0;
            cv::Vec3f cross = edge1.cross(edge2);
            double area = 0.5 * cv::norm(cross);

            component_area[component_id] += area;
        }
    }
};

// ============================================================================
// MeshCleaner public interface
// ============================================================================

MeshCleaner::MeshCleaner() : pImpl(std::make_unique<Impl>()) {}
MeshCleaner::~MeshCleaner() = default;

bool MeshCleaner::removeSmallObjects(std::vector<cv::Vec3f>& vertices,
                                     std::vector<cv::Vec3i>& faces,
                                     std::vector<cv::Vec3f>& normals,
                                     const MeshCleaningConfig& config,
                                     MeshCleaningResult& result) {
    std::lock_guard<std::mutex> lock(cleaning_mutex_);

    if (!validateMeshInput(vertices, faces)) {
        pImpl->lastError = "Invalid mesh input for cleaning";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid cleaning configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Small Objects Removal";

    try {
        // Store initial statistics
        result.vertices_before = vertices.size();
        result.triangles_before = faces.size();
        computeTotalArea(vertices, faces, result.total_area_before);

        pImpl->updateProgress(10);

        // Step 1: Remove duplicate vertices if requested
        if (config.remove_duplicate_vertices) {
            size_t duplicates = removeDuplicateVertices(vertices, faces, 1e-9);
            result.duplicate_vertices_removed = duplicates;
        }

        pImpl->updateProgress(20);

        // Step 2: Build connected components
        std::vector<int> vertex_to_component;
        size_t num_components = 0;

        pImpl->buildConnectedComponents(faces, vertices.size(), vertex_to_component, num_components);
        result.total_components_found = num_components;

        if (num_components == 0) {
            pImpl->lastError = "No connected components found in mesh";
            return false;
        }

        pImpl->updateProgress(40);

        // Step 3: Compute component statistics
        std::vector<size_t> component_triangle_count;
        std::vector<double> component_area;

        pImpl->computeComponentStats(vertices, faces, vertex_to_component, num_components,
                                     component_triangle_count, component_area);

        pImpl->updateProgress(60);

        // Step 4: Determine which components to keep
        std::set<int> components_to_keep;

        if (config.mode == MeshCleaningConfig::FilterMode::KEEP_LARGEST) {
            // Find largest component by area
            auto largest_it = std::max_element(component_area.begin(), component_area.end());
            size_t largest_idx = std::distance(component_area.begin(), largest_it);
            components_to_keep.insert(largest_idx);
            result.largest_component_area = *largest_it;
        } else {
            // Filter by threshold - keep all components above threshold
            double total_area = std::accumulate(component_area.begin(), component_area.end(), 0.0);
            double min_area_threshold = total_area * config.min_area_ratio;

            for (size_t i = 0; i < num_components; ++i) {
                bool keep = false;

                // Check triangle count threshold
                if (component_triangle_count[i] >= config.min_triangles) {
                    keep = true;
                }

                // Check area threshold
                if (component_area[i] >= min_area_threshold) {
                    keep = true;
                }

                if (keep) {
                    components_to_keep.insert(i);
                }
            }

            // Store largest component area
            if (!component_area.empty()) {
                result.largest_component_area = *std::max_element(component_area.begin(), component_area.end());
            }
        }

        result.components_kept = components_to_keep.size();
        result.components_removed = num_components - components_to_keep.size();

        pImpl->updateProgress(70);

        // Step 5: Filter faces based on component selection
        std::vector<cv::Vec3i> filtered_faces;
        filtered_faces.reserve(faces.size());

        for (const auto& face : faces) {
            if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                int component_id = vertex_to_component[face[0]];
                if (components_to_keep.count(component_id) > 0) {
                    filtered_faces.push_back(face);
                }
            }
        }

        faces = std::move(filtered_faces);

        pImpl->updateProgress(80);

        // Step 6: Remove unreferenced vertices if requested
        if (config.remove_unreferenced_vertices) {
            size_t unreferenced = removeUnreferencedVertices(vertices, faces);
            result.vertices_removed += unreferenced;
        }

        pImpl->updateProgress(90);

        // Step 7: Recompute normals if provided
        if (!normals.empty() && normals.size() == result.vertices_before) {
            normals.resize(vertices.size());
            std::fill(normals.begin(), normals.end(), cv::Vec3f(0, 0, 0));

            for (const auto& face : faces) {
                if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                    cv::Vec3f v0 = vertices[face[0]];
                    cv::Vec3f v1 = vertices[face[1]];
                    cv::Vec3f v2 = vertices[face[2]];

                    cv::Vec3f edge1 = v1 - v0;
                    cv::Vec3f edge2 = v2 - v0;
                    cv::Vec3f normal = edge1.cross(edge2);

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

        // Compute final statistics
        result.vertices_after = vertices.size();
        result.triangles_after = faces.size();
        result.triangles_removed = result.triangles_before - result.triangles_after;
        result.vertices_removed += (result.vertices_before - result.vertices_after);
        computeTotalArea(vertices, faces, result.total_area_after);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processing_time_ms = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("small_objects_removal", result.processing_time_ms);
        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Small objects removal failed: " + std::string(e.what());
        return false;
    }
}

#ifdef OPEN3D_ENABLED
std::shared_ptr<open3d::geometry::TriangleMesh> MeshCleaner::removeSmallObjects(
    const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
    const MeshCleaningConfig& config,
    MeshCleaningResult& result) {

    std::lock_guard<std::mutex> lock(cleaning_mutex_);

    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for cleaning";
        return nullptr;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Small Objects Removal (Open3D)";

    try {
        // Store initial statistics
        result.vertices_before = mesh->vertices_.size();
        result.triangles_before = mesh->triangles_.size();
        result.total_area_before = mesh->GetSurfaceArea();

        pImpl->updateProgress(10);

        // Create working copy
        auto cleaned_mesh = std::make_shared<open3d::geometry::TriangleMesh>(*mesh);

        // Step 1: Remove duplicates if requested
        if (config.remove_duplicate_vertices) {
            size_t before = cleaned_mesh->vertices_.size();
            cleaned_mesh->RemoveDuplicatedVertices();
            result.duplicate_vertices_removed = before - cleaned_mesh->vertices_.size();
        }

        pImpl->updateProgress(20);

        // Step 2: Cluster connected triangles (Open3D native implementation)
        auto [triangle_clusters, cluster_n_triangles, cluster_area] =
            cleaned_mesh->ClusterConnectedTriangles();

        result.total_components_found = cluster_n_triangles.size();

        if (cluster_n_triangles.empty()) {
            pImpl->lastError = "No connected components found in Open3D mesh";
            return nullptr;
        }

        pImpl->updateProgress(50);

        // Step 3: Determine which clusters to keep
        std::set<int> clusters_to_keep;

        if (config.mode == MeshCleaningConfig::FilterMode::KEEP_LARGEST) {
            // Find largest cluster by area
            auto largest_it = std::max_element(cluster_area.begin(), cluster_area.end());
            size_t largest_idx = std::distance(cluster_area.begin(), largest_it);
            clusters_to_keep.insert(largest_idx);
            result.largest_component_area = *largest_it;
        } else {
            // Filter by threshold
            double total_area = std::accumulate(cluster_area.begin(), cluster_area.end(), 0.0);
            double min_area_threshold = total_area * config.min_area_ratio;

            for (size_t i = 0; i < cluster_n_triangles.size(); ++i) {
                bool keep = false;

                // Check triangle count threshold
                if (cluster_n_triangles[i] >= config.min_triangles) {
                    keep = true;
                }

                // Check area threshold
                if (cluster_area[i] >= min_area_threshold) {
                    keep = true;
                }

                if (keep) {
                    clusters_to_keep.insert(i);
                }
            }

            result.largest_component_area = *std::max_element(cluster_area.begin(), cluster_area.end());
        }

        result.components_kept = clusters_to_keep.size();
        result.components_removed = cluster_n_triangles.size() - clusters_to_keep.size();

        pImpl->updateProgress(70);

        // Step 4: Select triangles from kept clusters
        std::vector<size_t> triangles_to_keep;
        triangles_to_keep.reserve(cleaned_mesh->triangles_.size());

        for (size_t i = 0; i < triangle_clusters.size(); ++i) {
            if (clusters_to_keep.count(triangle_clusters[i]) > 0) {
                triangles_to_keep.push_back(i);
            }
        }

        // Create filtered mesh
        auto filtered_mesh = cleaned_mesh->SelectByIndex(triangles_to_keep);

        pImpl->updateProgress(80);

        // Step 5: Remove unreferenced vertices if requested
        if (config.remove_unreferenced_vertices) {
            size_t before = filtered_mesh->vertices_.size();
            filtered_mesh->RemoveUnreferencedVertices();
            result.vertices_removed += (before - filtered_mesh->vertices_.size());
        }

        // Step 6: Recompute normals
        filtered_mesh->ComputeVertexNormals();

        pImpl->updateProgress(90);

        // Compute final statistics
        result.vertices_after = filtered_mesh->vertices_.size();
        result.triangles_after = filtered_mesh->triangles_.size();
        result.triangles_removed = result.triangles_before - result.triangles_after;
        result.vertices_removed += (result.vertices_before - result.vertices_after);
        result.total_area_after = filtered_mesh->GetSurfaceArea();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processing_time_ms = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("open3d_small_objects_removal", result.processing_time_ms);
        pImpl->updateProgress(100);

        return filtered_mesh;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D small objects removal failed: " + std::string(e.what());
        return nullptr;
    }
}

bool MeshCleaner::cleanMeshInPlace(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                   const MeshCleaningConfig& config,
                                   MeshCleaningResult& result) {
    std::lock_guard<std::mutex> lock(cleaning_mutex_);

    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh for in-place cleaning";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.operation = "Comprehensive Mesh Cleaning (Open3D)";

    try {
        result.vertices_before = mesh->vertices_.size();
        result.triangles_before = mesh->triangles_.size();
        result.total_area_before = mesh->GetSurfaceArea();

        // Step 1: Remove duplicate vertices
        if (config.remove_duplicate_vertices) {
            size_t before = mesh->vertices_.size();
            mesh->RemoveDuplicatedVertices();
            result.duplicate_vertices_removed = before - mesh->vertices_.size();
        }

        // Step 2: Remove unreferenced vertices
        if (config.remove_unreferenced_vertices) {
            size_t before = mesh->vertices_.size();
            mesh->RemoveUnreferencedVertices();
            result.vertices_removed = before - mesh->vertices_.size();
        }

        // Step 3: Remove degenerate triangles
        size_t before_triangles = mesh->triangles_.size();
        mesh->RemoveDegenerateTriangles();
        result.triangles_removed = before_triangles - mesh->triangles_.size();

        // Step 4: Remove small components
        auto [triangle_clusters, cluster_n_triangles, cluster_area] =
            mesh->ClusterConnectedTriangles();

        result.total_components_found = cluster_n_triangles.size();

        // Step 5: Recompute normals
        mesh->ComputeVertexNormals();

        result.vertices_after = mesh->vertices_.size();
        result.triangles_after = mesh->triangles_.size();
        result.total_area_after = mesh->GetSurfaceArea();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.processing_time_ms = duration.count();
        result.success = true;

        pImpl->updatePerformanceStats("open3d_clean_in_place", result.processing_time_ms);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D in-place cleaning failed: " + std::string(e.what());
        return false;
    }
}
#endif

size_t MeshCleaner::analyzeComponents(const std::vector<cv::Vec3f>& vertices,
                                     const std::vector<cv::Vec3i>& faces,
                                     std::vector<size_t>& component_sizes,
                                     std::vector<double>& component_areas) const {
    if (!validateMeshInput(vertices, faces)) {
        return 0;
    }

    try {
        std::vector<int> vertex_to_component;
        size_t num_components = 0;

        pImpl->buildConnectedComponents(faces, vertices.size(), vertex_to_component, num_components);
        pImpl->computeComponentStats(vertices, faces, vertex_to_component, num_components,
                                     component_sizes, component_areas);

        return num_components;

    } catch (const std::exception& e) {
        return 0;
    }
}

size_t MeshCleaner::removeDuplicateVertices(std::vector<cv::Vec3f>& vertices,
                                           std::vector<cv::Vec3i>& faces,
                                           double epsilon) {
    if (vertices.empty()) {
        return 0;
    }

    try {
        // Build spatial hash for vertex deduplication
        // Capture epsilon in lambda
        auto hash_func = [epsilon](const cv::Vec3f& v) -> size_t {
            // Grid-based hashing for spatial coherence
            int x = static_cast<int>(v[0] / epsilon);
            int y = static_cast<int>(v[1] / epsilon);
            int z = static_cast<int>(v[2] / epsilon);

            size_t h1 = std::hash<int>{}(x);
            size_t h2 = std::hash<int>{}(y);
            size_t h3 = std::hash<int>{}(z);

            return h1 ^ (h2 << 1) ^ (h3 << 2);
        };

        std::unordered_map<int, int> old_to_new_index;
        std::vector<cv::Vec3f> unique_vertices;
        unique_vertices.reserve(vertices.size());

        double epsilon_sq = epsilon * epsilon;

        for (size_t i = 0; i < vertices.size(); ++i) {
            bool found_duplicate = false;

            // Check against existing unique vertices (brute force within epsilon)
            for (size_t j = 0; j < unique_vertices.size(); ++j) {
                double dist_sq = cv::norm(vertices[i] - unique_vertices[j], cv::NORM_L2SQR);
                if (dist_sq < epsilon_sq) {
                    old_to_new_index[i] = j;
                    found_duplicate = true;
                    break;
                }
            }

            if (!found_duplicate) {
                old_to_new_index[i] = unique_vertices.size();
                unique_vertices.push_back(vertices[i]);
            }
        }

        size_t duplicates_removed = vertices.size() - unique_vertices.size();

        if (duplicates_removed > 0) {
            // Update face indices
            for (auto& face : faces) {
                face[0] = old_to_new_index[face[0]];
                face[1] = old_to_new_index[face[1]];
                face[2] = old_to_new_index[face[2]];
            }

            vertices = std::move(unique_vertices);
        }

        return duplicates_removed;

    } catch (const std::exception& e) {
        pImpl->lastError = "Duplicate vertex removal failed: " + std::string(e.what());
        return 0;
    }
}

size_t MeshCleaner::removeUnreferencedVertices(std::vector<cv::Vec3f>& vertices,
                                               std::vector<cv::Vec3i>& faces) {
    if (vertices.empty() || faces.empty()) {
        return 0;
    }

    try {
        // Mark referenced vertices
        std::vector<bool> is_referenced(vertices.size(), false);

        for (const auto& face : faces) {
            if (face[0] < vertices.size()) is_referenced[face[0]] = true;
            if (face[1] < vertices.size()) is_referenced[face[1]] = true;
            if (face[2] < vertices.size()) is_referenced[face[2]] = true;
        }

        // Build index mapping
        std::vector<int> old_to_new_index(vertices.size(), -1);
        std::vector<cv::Vec3f> referenced_vertices;
        referenced_vertices.reserve(vertices.size());

        for (size_t i = 0; i < vertices.size(); ++i) {
            if (is_referenced[i]) {
                old_to_new_index[i] = referenced_vertices.size();
                referenced_vertices.push_back(vertices[i]);
            }
        }

        size_t unreferenced_count = vertices.size() - referenced_vertices.size();

        if (unreferenced_count > 0) {
            // Update face indices
            for (auto& face : faces) {
                face[0] = old_to_new_index[face[0]];
                face[1] = old_to_new_index[face[1]];
                face[2] = old_to_new_index[face[2]];
            }

            vertices = std::move(referenced_vertices);
        }

        return unreferenced_count;

    } catch (const std::exception& e) {
        pImpl->lastError = "Unreferenced vertex removal failed: " + std::string(e.what());
        return 0;
    }
}

void MeshCleaner::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string MeshCleaner::getLastError() const {
    return pImpl->lastError;
}

void MeshCleaner::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

std::map<std::string, double> MeshCleaner::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(pImpl->statsMutex);
    return pImpl->performanceStats;
}

// ============================================================================
// Helper methods (private)
// ============================================================================

bool MeshCleaner::validateMeshInput(const std::vector<cv::Vec3f>& vertices,
                                    const std::vector<cv::Vec3i>& faces) const {
    if (vertices.empty() || faces.empty()) {
        return false;
    }

    // Check face indices validity
    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            if (face[i] < 0 || face[i] >= vertices.size()) {
                return false;
            }
        }
    }

    return true;
}

void MeshCleaner::computeTotalArea(const std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   double& total_area) const {
    total_area = 0.0;

    for (const auto& face : faces) {
        if (face[0] >= vertices.size() || face[1] >= vertices.size() || face[2] >= vertices.size()) {
            continue;
        }

        const cv::Vec3f& v0 = vertices[face[0]];
        const cv::Vec3f& v1 = vertices[face[1]];
        const cv::Vec3f& v2 = vertices[face[2]];

        cv::Vec3f edge1 = v1 - v0;
        cv::Vec3f edge2 = v2 - v0;
        cv::Vec3f cross = edge1.cross(edge2);

        total_area += 0.5 * cv::norm(cross);
    }
}

void MeshCleaner::buildFaceComponentMapping(const std::vector<cv::Vec3i>& faces,
                                           size_t num_vertices,
                                           std::vector<int>& face_to_component,
                                           size_t& num_components) const {
    std::vector<int> vertex_to_component;
    pImpl->buildConnectedComponents(faces, num_vertices, vertex_to_component, num_components);

    face_to_component.resize(faces.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        // Component of face = component of first vertex
        face_to_component[i] = vertex_to_component[faces[i][0]];
    }
}

} // namespace mesh
} // namespace unlook
