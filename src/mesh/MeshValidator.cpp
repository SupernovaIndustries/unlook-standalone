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

// MeshValidator implementation
class MeshValidator::Impl {
public:
    std::string lastError;
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

MeshValidator::MeshValidator() : pImpl(std::make_unique<Impl>()) {}
MeshValidator::~MeshValidator() = default;

bool MeshValidator::validateMesh(const std::vector<cv::Vec3f>& vertices,
                                const std::vector<cv::Vec3i>& faces,
                                const std::vector<cv::Vec3f>& normals,
                                const MeshValidationConfig& config,
                                MeshQualityMetrics& metrics) {
    if (vertices.empty() || faces.empty()) {
        pImpl->lastError = "Empty mesh data";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid validation configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    metrics = {}; // Reset metrics

    try {
        // Basic topology
        metrics.numVertices = vertices.size();
        metrics.numFaces = faces.size();
        metrics.numEdges = faces.size() * 3 / 2; // Rough estimate

        pImpl->updateProgress(10);

        // Compute bounding box and centroid
        if (!vertices.empty()) {
            cv::Vec3f minBounds(FLT_MAX, FLT_MAX, FLT_MAX);
            cv::Vec3f maxBounds(-FLT_MAX, -FLT_MAX, -FLT_MAX);
            cv::Vec3f centroidSum(0, 0, 0);

            for (const auto& vertex : vertices) {
                minBounds[0] = std::min(minBounds[0], vertex[0]);
                minBounds[1] = std::min(minBounds[1], vertex[1]);
                minBounds[2] = std::min(minBounds[2], vertex[2]);

                maxBounds[0] = std::max(maxBounds[0], vertex[0]);
                maxBounds[1] = std::max(maxBounds[1], vertex[1]);
                maxBounds[2] = std::max(maxBounds[2], vertex[2]);

                centroidSum[0] += vertex[0];
                centroidSum[1] += vertex[1];
                centroidSum[2] += vertex[2];
            }

            metrics.boundingBoxMin = minBounds;
            metrics.boundingBoxMax = maxBounds;
            metrics.centroid[0] = centroidSum[0] / vertices.size();
            metrics.centroid[1] = centroidSum[1] / vertices.size();
            metrics.centroid[2] = centroidSum[2] / vertices.size();
        }

        pImpl->updateProgress(20);

        // Manufacturing readiness checks
        metrics.isWatertight = isWatertight(vertices, faces);
        metrics.isManifold = isManifold(vertices, faces);

        pImpl->updateProgress(30);

        // Self-intersection detection
        std::vector<int> intersectionFaces;
        metrics.hasSelfIntersections = detectSelfIntersections(vertices, faces, intersectionFaces);

        pImpl->updateProgress(40);

        // Triangle quality analysis
        std::vector<double> triangleQualities, aspectRatios;
        if (computeTriangleQuality(vertices, faces, triangleQualities, aspectRatios)) {
            if (!triangleQualities.empty()) {
                metrics.triangleQualityMean = std::accumulate(triangleQualities.begin(), triangleQualities.end(), 0.0) / triangleQualities.size();

                double variance = 0.0;
                for (double quality : triangleQualities) {
                    variance += std::pow(quality - metrics.triangleQualityMean, 2);
                }
                metrics.triangleQualityStd = std::sqrt(variance / triangleQualities.size());
            }

            if (!aspectRatios.empty()) {
                metrics.aspectRatioMean = std::accumulate(aspectRatios.begin(), aspectRatios.end(), 0.0) / aspectRatios.size();
                metrics.aspectRatioMin = *std::min_element(aspectRatios.begin(), aspectRatios.end());
                metrics.aspectRatioMax = *std::max_element(aspectRatios.begin(), aspectRatios.end());

                double variance = 0.0;
                for (double ratio : aspectRatios) {
                    variance += std::pow(ratio - metrics.aspectRatioMean, 2);
                }
                metrics.aspectRatioStd = std::sqrt(variance / aspectRatios.size());
            }
        }

        pImpl->updateProgress(50);

        // Edge length analysis
        std::vector<double> edgeLengths;
        edgeLengths.reserve(faces.size() * 3);

        for (const auto& face : faces) {
            for (int i = 0; i < 3; ++i) {
                int v1 = face[i];
                int v2 = face[(i + 1) % 3];

                if (v1 >= 0 && v1 < vertices.size() && v2 >= 0 && v2 < vertices.size()) {
                    double length = cv::norm(vertices[v1] - vertices[v2]);
                    edgeLengths.push_back(length);
                }
            }
        }

        if (!edgeLengths.empty()) {
            metrics.edgeLengthMean = std::accumulate(edgeLengths.begin(), edgeLengths.end(), 0.0) / edgeLengths.size();
            metrics.edgeLengthMin = *std::min_element(edgeLengths.begin(), edgeLengths.end());
            metrics.edgeLengthMax = *std::max_element(edgeLengths.begin(), edgeLengths.end());

            double variance = 0.0;
            for (double length : edgeLengths) {
                variance += std::pow(length - metrics.edgeLengthMean, 2);
            }
            metrics.edgeLengthStd = std::sqrt(variance / edgeLengths.size());
        }

        pImpl->updateProgress(60);

        // Surface area and volume calculation
        double totalArea = 0.0;
        double totalVolume = 0.0;

        for (const auto& face : faces) {
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {

                const cv::Vec3f& v0 = vertices[face[0]];
                const cv::Vec3f& v1 = vertices[face[1]];
                const cv::Vec3f& v2 = vertices[face[2]];

                // Triangle area
                double area = computeTriangleArea(v0, v1, v2);
                totalArea += area;

                // Volume contribution (signed volume to origin)
                double volume = (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                               v0[1] * (v1[2] * v2[0] - v1[0] * v2[2]) +
                               v0[2] * (v1[0] * v2[1] - v1[1] * v2[0])) / 6.0;
                totalVolume += volume;
            }
        }

        metrics.surfaceArea = totalArea;
        metrics.volume = std::abs(totalVolume);

        pImpl->updateProgress(70);

        // Detailed topology analysis if enabled
        if (config.enableDetailedAnalysis) {
            std::vector<std::pair<int, int>> boundaryEdges;
            std::vector<std::vector<int>> holeLoops;
            metrics.numHoles = findBoundariesAndHoles(vertices, faces, boundaryEdges, holeLoops);
            metrics.numBoundaryEdges = boundaryEdges.size();

            std::vector<int> connectedComponents;
            metrics.numConnectedComponents = analyzeConnectivity(vertices, faces, connectedComponents);

            // Count component sizes
            std::map<int, size_t> componentSizeMap;
            for (int comp : connectedComponents) {
                componentSizeMap[comp]++;
            }
            for (const auto& [comp, size] : componentSizeMap) {
                metrics.componentSizes.push_back(size);
            }
            std::sort(metrics.componentSizes.rbegin(), metrics.componentSizes.rend());
        }

        pImpl->updateProgress(80);

        // Overall quality assessment
        metrics.precisionScore = std::min(1.0, config.targetPrecision / std::max(metrics.surfaceDeviation, 0.001));

        pImpl->updateProgress(90);

        // Performance metrics
        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        metrics.processingTimeMs = duration.count();

        // Estimate memory usage (rough approximation)
        metrics.memoryUsageMB = (vertices.size() * sizeof(cv::Vec3f) +
                                faces.size() * sizeof(cv::Vec3i) +
                                normals.size() * sizeof(cv::Vec3f)) / (1024 * 1024);

        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh validation failed: " + std::string(e.what());
        return false;
    }
}

#ifdef OPEN3D_ENABLED
bool MeshValidator::validateOpen3DMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                      const MeshValidationConfig& config,
                                      MeshQualityMetrics& metrics) {
    if (!mesh || mesh->vertices_.empty() || mesh->triangles_.empty()) {
        pImpl->lastError = "Empty Open3D mesh";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    try {
        // Convert to OpenCV format for validation
        std::vector<cv::Vec3f> vertices;
        std::vector<cv::Vec3i> faces;
        std::vector<cv::Vec3f> normals;

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

        // Use Open3D built-in validation functions where available
        metrics.isWatertight = mesh->IsWatertight();
        metrics.isManifold = mesh->IsEdgeManifold() && mesh->IsVertexManifold();
        metrics.isOrientable = mesh->IsOrientable();
        metrics.hasSelfIntersections = mesh->IsSelfIntersecting();

        // Use standard validation for the rest
        bool result = validateMesh(vertices, faces, normals, config, metrics);

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        metrics.processingTimeMs = duration.count();

        return result;

    } catch (const std::exception& e) {
        pImpl->lastError = "Open3D mesh validation failed: " + std::string(e.what());
        return false;
    }
}
#endif

bool MeshValidator::isWatertight(const std::vector<cv::Vec3f>& vertices,
                                const std::vector<cv::Vec3i>& faces) {
    // Build edge map to check if every edge has exactly two adjacent faces
    std::map<std::pair<int, int>, int> edgeCount;

    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % 3];

            // Ensure consistent edge ordering
            if (v1 > v2) std::swap(v1, v2);

            edgeCount[{v1, v2}]++;
        }
    }

    // Check if every edge has exactly 2 adjacent faces
    for (const auto& [edge, count] : edgeCount) {
        if (count != 2) {
            return false;
        }
    }

    return true;
}

bool MeshValidator::isManifold(const std::vector<cv::Vec3f>& vertices,
                              const std::vector<cv::Vec3i>& faces) {
    // Build vertex-to-faces mapping
    std::unordered_map<int, std::vector<int>> vertexToFaces;

    for (int faceIdx = 0; faceIdx < faces.size(); ++faceIdx) {
        const auto& face = faces[faceIdx];
        for (int i = 0; i < 3; ++i) {
            vertexToFaces[face[i]].push_back(faceIdx);
        }
    }

    // Check if the faces around each vertex form a proper fan
    for (const auto& [vertex, adjacentFaces] : vertexToFaces) {
        if (adjacentFaces.size() < 3) continue;

        // Build edge connectivity around this vertex
        std::set<std::pair<int, int>> edges;
        for (int faceIdx : adjacentFaces) {
            const auto& face = faces[faceIdx];

            // Find the vertex position in the face
            int pos = -1;
            for (int i = 0; i < 3; ++i) {
                if (face[i] == vertex) {
                    pos = i;
                    break;
                }
            }

            if (pos >= 0) {
                int prev = face[(pos + 2) % 3];
                int next = face[(pos + 1) % 3];
                edges.insert({prev, next});
            }
        }

        // Check if edges form a proper cycle
        if (edges.size() != adjacentFaces.size()) {
            return false;
        }
    }

    return true;
}

bool MeshValidator::detectSelfIntersections(const std::vector<cv::Vec3f>& vertices,
                                           const std::vector<cv::Vec3i>& faces,
                                           std::vector<int>& intersectionFaces) {
    intersectionFaces.clear();

    // Simple O(n²) approach for small meshes
    // For large meshes, spatial data structures like octrees would be more efficient
    for (int i = 0; i < faces.size(); ++i) {
        const auto& face1 = faces[i];

        if (face1[0] >= vertices.size() || face1[1] >= vertices.size() || face1[2] >= vertices.size()) {
            continue;
        }

        const cv::Vec3f& a0 = vertices[face1[0]];
        const cv::Vec3f& a1 = vertices[face1[1]];
        const cv::Vec3f& a2 = vertices[face1[2]];

        for (int j = i + 1; j < faces.size(); ++j) {
            const auto& face2 = faces[j];

            if (face2[0] >= vertices.size() || face2[1] >= vertices.size() || face2[2] >= vertices.size()) {
                continue;
            }

            // Skip if faces share vertices
            bool shareVertex = false;
            for (int vi = 0; vi < 3; ++vi) {
                for (int vj = 0; vj < 3; ++vj) {
                    if (face1[vi] == face2[vj]) {
                        shareVertex = true;
                        break;
                    }
                }
                if (shareVertex) break;
            }
            if (shareVertex) continue;

            const cv::Vec3f& b0 = vertices[face2[0]];
            const cv::Vec3f& b1 = vertices[face2[1]];
            const cv::Vec3f& b2 = vertices[face2[2]];

            if (areTrianglesIntersecting(a0, a1, a2, b0, b1, b2)) {
                intersectionFaces.push_back(i);
                intersectionFaces.push_back(j);
            }
        }
    }

    return !intersectionFaces.empty();
}

bool MeshValidator::computeTriangleQuality(const std::vector<cv::Vec3f>& vertices,
                                          const std::vector<cv::Vec3i>& faces,
                                          std::vector<double>& triangleQualities,
                                          std::vector<double>& aspectRatios) {
    triangleQualities.clear();
    aspectRatios.clear();

    triangleQualities.reserve(faces.size());
    aspectRatios.reserve(faces.size());

    for (const auto& face : faces) {
        if (face[0] >= 0 && face[0] < vertices.size() &&
            face[1] >= 0 && face[1] < vertices.size() &&
            face[2] >= 0 && face[2] < vertices.size()) {

            const cv::Vec3f& v0 = vertices[face[0]];
            const cv::Vec3f& v1 = vertices[face[1]];
            const cv::Vec3f& v2 = vertices[face[2]];

            double quality = computeTriangleQuality(v0, v1, v2);
            double aspectRatio = computeTriangleAspectRatio(v0, v1, v2);

            triangleQualities.push_back(quality);
            aspectRatios.push_back(aspectRatio);
        }
    }

    return true;
}

int MeshValidator::findBoundariesAndHoles(const std::vector<cv::Vec3f>& vertices,
                                         const std::vector<cv::Vec3i>& faces,
                                         std::vector<std::pair<int, int>>& boundaryEdges,
                                         std::vector<std::vector<int>>& holeLoops) {
    boundaryEdges.clear();
    holeLoops.clear();

    // Find boundary edges (edges with only one adjacent face)
    std::map<std::pair<int, int>, int> edgeCount;

    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % 3];

            if (v1 > v2) std::swap(v1, v2);
            edgeCount[{v1, v2}]++;
        }
    }

    for (const auto& [edge, count] : edgeCount) {
        if (count == 1) {
            boundaryEdges.push_back(edge);
        }
    }

    // Trace boundary loops to identify holes
    std::set<std::pair<int, int>> unusedBoundaryEdges(boundaryEdges.begin(), boundaryEdges.end());

    while (!unusedBoundaryEdges.empty()) {
        std::vector<int> loop;
        auto startEdge = *unusedBoundaryEdges.begin();
        unusedBoundaryEdges.erase(unusedBoundaryEdges.begin());

        int currentVertex = startEdge.first;
        int nextVertex = startEdge.second;
        loop.push_back(currentVertex);

        while (nextVertex != startEdge.first) {
            loop.push_back(nextVertex);

            // Find next edge
            bool found = false;
            for (auto it = unusedBoundaryEdges.begin(); it != unusedBoundaryEdges.end(); ++it) {
                if (it->first == nextVertex) {
                    currentVertex = nextVertex;
                    nextVertex = it->second;
                    unusedBoundaryEdges.erase(it);
                    found = true;
                    break;
                } else if (it->second == nextVertex) {
                    currentVertex = nextVertex;
                    nextVertex = it->first;
                    unusedBoundaryEdges.erase(it);
                    found = true;
                    break;
                }
            }

            if (!found) break;
        }

        if (loop.size() >= 3) {
            holeLoops.push_back(loop);
        }
    }

    return holeLoops.size();
}

int MeshValidator::analyzeConnectivity(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      std::vector<int>& connectedComponents) {
    connectedComponents.assign(vertices.size(), -1);

    // Build adjacency list
    std::vector<std::set<int>> adjacency(vertices.size());

    for (const auto& face : faces) {
        for (int i = 0; i < 3; ++i) {
            int v1 = face[i];
            int v2 = face[(i + 1) % 3];

            if (v1 < vertices.size() && v2 < vertices.size()) {
                adjacency[v1].insert(v2);
                adjacency[v2].insert(v1);
            }
        }
    }

    // Find connected components using DFS
    int componentId = 0;

    for (int i = 0; i < vertices.size(); ++i) {
        if (connectedComponents[i] == -1) {
            // Start new component
            std::queue<int> queue;
            queue.push(i);
            connectedComponents[i] = componentId;

            while (!queue.empty()) {
                int current = queue.front();
                queue.pop();

                for (int neighbor : adjacency[current]) {
                    if (connectedComponents[neighbor] == -1) {
                        connectedComponents[neighbor] = componentId;
                        queue.push(neighbor);
                    }
                }
            }

            componentId++;
        }
    }

    return componentId;
}

bool MeshValidator::assessPrecision(const std::vector<cv::Vec3f>& vertices,
                                   const std::vector<cv::Vec3i>& faces,
                                   const std::vector<cv::Vec3f>& pointCloud,
                                   double& maxDeviation,
                                   double& meanDeviation,
                                   double& stdDeviation) {
    if (pointCloud.empty() || faces.empty()) {
        maxDeviation = meanDeviation = stdDeviation = 0.0;
        return false;
    }

    std::vector<double> distances;
    distances.reserve(pointCloud.size());

    // For each point in the point cloud, find distance to nearest triangle
    for (const auto& point : pointCloud) {
        double minDistance = std::numeric_limits<double>::max();

        for (const auto& face : faces) {
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {

                const cv::Vec3f& v0 = vertices[face[0]];
                const cv::Vec3f& v1 = vertices[face[1]];
                const cv::Vec3f& v2 = vertices[face[2]];

                double distance = pointToTriangleDistance(point, v0, v1, v2);
                minDistance = std::min(minDistance, distance);
            }
        }

        if (minDistance != std::numeric_limits<double>::max()) {
            distances.push_back(minDistance);
        }
    }

    if (distances.empty()) {
        maxDeviation = meanDeviation = stdDeviation = 0.0;
        return false;
    }

    maxDeviation = *std::max_element(distances.begin(), distances.end());
    meanDeviation = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();

    double variance = 0.0;
    for (double dist : distances) {
        variance += std::pow(dist - meanDeviation, 2);
    }
    stdDeviation = std::sqrt(variance / distances.size());

    return true;
}

std::string MeshValidator::generateValidationReport(const MeshQualityMetrics& metrics,
                                                   const MeshValidationConfig& config) {
    std::stringstream report;

    report << "=== MESH VALIDATION REPORT ===\n\n";

    // Basic topology
    report << "TOPOLOGY:\n";
    report << "  Vertices: " << metrics.numVertices << "\n";
    report << "  Faces: " << metrics.numFaces << "\n";
    report << "  Edges: " << metrics.numEdges << "\n\n";

    // Manufacturing readiness
    report << "MANUFACTURING READINESS:\n";
    report << "  Watertight: " << (metrics.isWatertight ? "YES" : "NO") << "\n";
    report << "  Manifold: " << (metrics.isManifold ? "YES" : "NO") << "\n";
    report << "  Self-intersections: " << (metrics.hasSelfIntersections ? "YES" : "NO") << "\n";
    report << "  Manufacturing ready: " << (metrics.isManufacturingReady() ? "YES" : "NO") << "\n\n";

    // Geometric properties
    report << "GEOMETRIC PROPERTIES:\n";
    report << "  Surface area: " << std::fixed << std::setprecision(3) << metrics.surfaceArea << " mm²\n";
    report << "  Volume: " << std::fixed << std::setprecision(3) << metrics.volume << " mm³\n";
    report << "  Bounding box: [" << metrics.boundingBoxMin[0] << ", " << metrics.boundingBoxMin[1] << ", " << metrics.boundingBoxMin[2] << "] to ";
    report << "[" << metrics.boundingBoxMax[0] << ", " << metrics.boundingBoxMax[1] << ", " << metrics.boundingBoxMax[2] << "]\n\n";

    // Quality metrics
    report << "QUALITY METRICS:\n";
    report << "  Triangle quality: " << std::fixed << std::setprecision(3) << metrics.triangleQualityMean << " ± " << metrics.triangleQualityStd << "\n";
    report << "  Aspect ratio: " << std::fixed << std::setprecision(3) << metrics.aspectRatioMean << " ± " << metrics.aspectRatioStd;
    report << " (range: " << metrics.aspectRatioMin << " - " << metrics.aspectRatioMax << ")\n";
    report << "  Edge length: " << std::fixed << std::setprecision(3) << metrics.edgeLengthMean << " ± " << metrics.edgeLengthStd << " mm";
    report << " (range: " << metrics.edgeLengthMin << " - " << metrics.edgeLengthMax << ")\n";
    report << "  Surface deviation: " << std::fixed << std::setprecision(4) << metrics.surfaceDeviation << " mm\n";
    report << "  Precision score: " << std::fixed << std::setprecision(3) << metrics.precisionScore << "\n\n";

    // Topology details
    if (metrics.numBoundaryEdges > 0 || metrics.numHoles > 0) {
        report << "TOPOLOGY ISSUES:\n";
        report << "  Boundary edges: " << metrics.numBoundaryEdges << "\n";
        report << "  Holes: " << metrics.numHoles << "\n";
        report << "  Connected components: " << metrics.numConnectedComponents << "\n\n";
    }

    // Overall assessment
    report << "OVERALL ASSESSMENT:\n";
    report << "  Quality score: " << std::fixed << std::setprecision(3) << metrics.getOverallQualityScore() << " / 1.0\n";
    report << "  Processing time: " << metrics.processingTimeMs << " ms\n";
    report << "  Memory usage: " << metrics.memoryUsageMB << " MB\n\n";

    // Recommendations
    report << "RECOMMENDATIONS:\n";
    if (!metrics.isWatertight) {
        report << "  - Fix mesh holes to make it watertight\n";
    }
    if (!metrics.isManifold) {
        report << "  - Repair non-manifold geometry\n";
    }
    if (metrics.hasSelfIntersections) {
        report << "  - Remove self-intersections\n";
    }
    if (metrics.triangleQualityMean < config.minTriangleQuality) {
        report << "  - Improve triangle quality through remeshing\n";
    }
    if (metrics.aspectRatioMax > config.maxAspectRatio) {
        report << "  - Reduce high aspect ratio triangles\n";
    }
    if (metrics.surfaceDeviation > config.maxSurfaceDeviation) {
        report << "  - Improve surface accuracy to meet precision requirements\n";
    }

    return report.str();
}

void MeshValidator::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string MeshValidator::getLastError() const {
    return pImpl->lastError;
}

void MeshValidator::enableARM64Optimizations(bool enable) {
    pImpl->arm64Optimized = enable;
}

// Helper method implementations
double MeshValidator::computeTriangleArea(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2) {
    cv::Vec3f edge1 = v1 - v0;
    cv::Vec3f edge2 = v2 - v0;
    cv::Vec3f cross = edge1.cross(edge2);
    return cv::norm(cross) * 0.5;
}

double MeshValidator::computeTriangleAspectRatio(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2) {
    double a = cv::norm(v1 - v0);
    double b = cv::norm(v2 - v1);
    double c = cv::norm(v0 - v2);

    double maxEdge = std::max({a, b, c});
    double minEdge = std::min({a, b, c});

    return (minEdge > 1e-6) ? maxEdge / minEdge : std::numeric_limits<double>::max();
}

double MeshValidator::computeTriangleQuality(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2) {
    double area = computeTriangleArea(v0, v1, v2);

    double a = cv::norm(v1 - v0);
    double b = cv::norm(v2 - v1);
    double c = cv::norm(v0 - v2);

    double perimeter = a + b + c;

    if (perimeter < 1e-6) return 0.0;

    // Quality metric: 4 * sqrt(3) * area / (perimeter²)
    // This gives 1.0 for equilateral triangles and approaches 0 for degenerate triangles
    return (4.0 * std::sqrt(3.0) * area) / (perimeter * perimeter);
}

bool MeshValidator::areTrianglesIntersecting(const cv::Vec3f& a0, const cv::Vec3f& a1, const cv::Vec3f& a2,
                                            const cv::Vec3f& b0, const cv::Vec3f& b1, const cv::Vec3f& b2) {
    // Simplified triangle-triangle intersection test
    // This is a basic implementation - more sophisticated algorithms exist

    // Check if any vertex of triangle A is inside triangle B, and vice versa
    // This is not a complete test but catches many common cases

    auto pointInTriangle = [](const cv::Vec3f& p, const cv::Vec3f& a, const cv::Vec3f& b, const cv::Vec3f& c) -> bool {
        cv::Vec3f v0 = c - a;
        cv::Vec3f v1 = b - a;
        cv::Vec3f v2 = p - a;

        double dot00 = v0.dot(v0);
        double dot01 = v0.dot(v1);
        double dot02 = v0.dot(v2);
        double dot11 = v1.dot(v1);
        double dot12 = v1.dot(v2);

        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        return (u >= 0) && (v >= 0) && (u + v <= 1);
    };

    // Simple check: if any vertex of one triangle is inside the other
    if (pointInTriangle(a0, b0, b1, b2) || pointInTriangle(a1, b0, b1, b2) || pointInTriangle(a2, b0, b1, b2) ||
        pointInTriangle(b0, a0, a1, a2) || pointInTriangle(b1, a0, a1, a2) || pointInTriangle(b2, a0, a1, a2)) {
        return true;
    }

    return false;
}

double MeshValidator::pointToTriangleDistance(const cv::Vec3f& point,
                                             const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2) {
    // Project point onto triangle plane and find closest point
    cv::Vec3f edge0 = v1 - v0;
    cv::Vec3f edge1 = v2 - v0;
    cv::Vec3f v0_to_point = point - v0;

    double a = edge0.dot(edge0);
    double b = edge0.dot(edge1);
    double c = edge1.dot(edge1);
    double d = edge0.dot(v0_to_point);
    double e = edge1.dot(v0_to_point);

    double det = a * c - b * b;
    double s = b * e - c * d;
    double t = b * d - a * e;

    if (s + t < det) {
        if (s < 0.0) {
            if (t < 0.0) {
                // Region 4
                s = std::max(0.0, -d / a);
                t = 0.0;
            } else {
                // Region 3
                s = 0.0;
                t = std::max(0.0, std::min(1.0, e / c));
            }
        } else if (t < 0.0) {
            // Region 5
            t = 0.0;
            s = std::max(0.0, std::min(1.0, d / a));
        } else {
            // Region 0
            double invDet = 1.0 / det;
            s *= invDet;
            t *= invDet;
        }
    } else {
        if (s < 0.0) {
            // Region 2
            double tmp0 = b + d;
            double tmp1 = c + e;
            if (tmp1 > tmp0) {
                double numer = tmp1 - tmp0;
                double denom = a - 2.0 * b + c;
                s = std::max(0.0, std::min(1.0, numer / denom));
                t = 1.0 - s;
            } else {
                t = std::max(0.0, std::min(1.0, e / c));
                s = 0.0;
            }
        } else if (t < 0.0) {
            // Region 6
            double tmp0 = b + e;
            double tmp1 = a + d;
            if (tmp1 > tmp0) {
                double numer = tmp1 - tmp0;
                double denom = a - 2.0 * b + c;
                t = std::max(0.0, std::min(1.0, numer / denom));
                s = 1.0 - t;
            } else {
                s = std::max(0.0, std::min(1.0, d / a));
                t = 0.0;
            }
        } else {
            // Region 1
            double numer = c + e - b - d;
            if (numer <= 0.0) {
                s = 0.0;
            } else {
                double denom = a - 2.0 * b + c;
                s = std::max(0.0, std::min(1.0, numer / denom));
            }
            t = 1.0 - s;
        }
    }

    cv::Vec3f closest = v0 + s * edge0 + t * edge1;
    return cv::norm(point - closest);
}

// Configuration validation implementations
bool MeshValidationConfig::validate() const {
    if (minTriangleQuality < 0.0 || minTriangleQuality > 1.0) return false;
    if (maxAspectRatio <= 1.0) return false;
    if (minEdgeLength <= 0.0 || maxEdgeLength <= minEdgeLength) return false;
    if (maxSurfaceDeviation <= 0.0) return false;
    if (targetPrecision <= 0.0) return false;
    return true;
}

std::string MeshValidationConfig::toString() const {
    std::stringstream ss;
    ss << "Mesh Validation Configuration:\n";
    ss << "  Min triangle quality: " << minTriangleQuality << "\n";
    ss << "  Max aspect ratio: " << maxAspectRatio << "\n";
    ss << "  Edge length range: " << minEdgeLength << " - " << maxEdgeLength << " mm\n";
    ss << "  Max surface deviation: " << maxSurfaceDeviation << " mm\n";
    ss << "  Target precision: " << targetPrecision << " mm\n";
    ss << "  Require watertight: " << (requireWatertight ? "Yes" : "No") << "\n";
    ss << "  Require manifold: " << (requireManifold ? "Yes" : "No") << "\n";
    return ss.str();
}

// MeshQualityMetrics implementations
std::string MeshQualityMetrics::toString() const {
    std::stringstream ss;
    ss << "Mesh Quality Metrics:\n";
    ss << "  Topology: " << numVertices << " vertices, " << numFaces << " faces\n";
    ss << "  Manufacturing ready: " << (isManufacturingReady() ? "Yes" : "No") << "\n";
    ss << "  Surface area: " << std::fixed << std::setprecision(3) << surfaceArea << " mm²\n";
    ss << "  Volume: " << std::fixed << std::setprecision(3) << volume << " mm³\n";
    ss << "  Triangle quality: " << std::fixed << std::setprecision(3) << triangleQualityMean << " ± " << triangleQualityStd << "\n";
    ss << "  Overall quality score: " << std::fixed << std::setprecision(3) << getOverallQualityScore() << "\n";
    return ss.str();
}

std::map<std::string, double> MeshQualityMetrics::toMap() const {
    std::map<std::string, double> map;
    map["numVertices"] = static_cast<double>(numVertices);
    map["numFaces"] = static_cast<double>(numFaces);
    map["surfaceArea"] = surfaceArea;
    map["volume"] = volume;
    map["isWatertight"] = isWatertight ? 1.0 : 0.0;
    map["isManifold"] = isManifold ? 1.0 : 0.0;
    map["triangleQualityMean"] = triangleQualityMean;
    map["aspectRatioMean"] = aspectRatioMean;
    map["edgeLengthMean"] = edgeLengthMean;
    map["surfaceDeviation"] = surfaceDeviation;
    map["precisionScore"] = precisionScore;
    map["overallQualityScore"] = getOverallQualityScore();
    return map;
}

bool MeshQualityMetrics::isManufacturingReady() const {
    return isWatertight && isManifold && !hasSelfIntersections &&
           triangleQualityMean > 0.3 && aspectRatioMean < 10.0;
}

double MeshQualityMetrics::getOverallQualityScore() const {
    double score = 0.0;

    // Manufacturing readiness (40% weight)
    if (isWatertight) score += 0.15;
    if (isManifold) score += 0.15;
    if (!hasSelfIntersections) score += 0.10;

    // Triangle quality (30% weight)
    score += std::min(0.30, triangleQualityMean * 0.30);

    // Precision (30% weight)
    score += std::min(0.30, precisionScore * 0.30);

    return std::min(1.0, score);
}

// Namespace validation implementations
namespace validation {

bool isDegenerateTriangle(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2, double threshold) {
    cv::Vec3f edge1 = v1 - v0;
    cv::Vec3f edge2 = v2 - v0;
    cv::Vec3f cross = edge1.cross(edge2);
    double area = cv::norm(cross) * 0.5;
    return area < threshold;
}

cv::Vec3f computeTriangleNormal(const cv::Vec3f& v0, const cv::Vec3f& v1, const cv::Vec3f& v2) {
    cv::Vec3f edge1 = v1 - v0;
    cv::Vec3f edge2 = v2 - v0;
    cv::Vec3f normal = edge1.cross(edge2);
    double length = cv::norm(normal);
    if (length > 1e-6) {
        normal /= length;
    }
    return normal;
}

bool isManufacturingReady(const MeshQualityMetrics& metrics, const MeshValidationConfig& config) {
    return metrics.isWatertight &&
           metrics.isManifold &&
           !metrics.hasSelfIntersections &&
           metrics.triangleQualityMean >= config.minTriangleQuality &&
           metrics.aspectRatioMean <= config.maxAspectRatio &&
           metrics.surfaceDeviation <= config.maxSurfaceDeviation;
}

double computeIndustrialQualityScore(const MeshQualityMetrics& metrics) {
    return metrics.getOverallQualityScore();
}

} // namespace validation

} // namespace mesh
} // namespace unlook