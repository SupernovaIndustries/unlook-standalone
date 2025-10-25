#pragma once

#include "unlook/core/types.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <functional>

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
 * @brief Mesh cleaning configuration matching Unlook Studio algorithms
 *
 * Implements industrial-grade mesh cleaning for removing small artifacts,
 * isolated components, and noise from 3D scans. Critical for investor demos
 * where professional mesh quality is required.
 */
struct MeshCleaningConfig {
    /**
     * @brief Filtering modes matching Unlook Studio behavior
     */
    enum class FilterMode {
        KEEP_LARGEST,         // Keep only the largest connected component (Unlook: LeaveBiggestObject)
        FILTER_BY_THRESHOLD   // Remove components below size threshold (Unlook: FilterByThreshold)
    };

    FilterMode mode = FilterMode::KEEP_LARGEST;

    // Threshold parameters for FILTER_BY_THRESHOLD mode
    size_t min_triangles = 100;        // Minimum triangles to keep component
    double min_area_ratio = 0.01;      // Minimum area ratio (1% of total area)
    size_t min_vertices = 50;          // Minimum vertices to keep component

    // Advanced filtering options
    bool remove_duplicate_vertices = true;   // Remove duplicate vertices
    bool remove_unreferenced_vertices = true; // Remove unreferenced vertices
    bool merge_close_vertices = false;        // Merge vertices within threshold
    double merge_distance = 0.001;            // Merge distance threshold (mm)

    // Performance options
    bool parallel_processing = true;          // Use parallel algorithms on ARM64

    /**
     * @brief Validate configuration parameters
     */
    bool validate() const {
        if (min_triangles == 0 && mode == FilterMode::FILTER_BY_THRESHOLD) {
            return false;
        }
        if (min_area_ratio < 0.0 || min_area_ratio > 1.0) {
            return false;
        }
        if (merge_distance < 0.0) {
            return false;
        }
        return true;
    }

    /**
     * @brief Get human-readable configuration string
     */
    std::string toString() const;
};

/**
 * @brief Mesh cleaning result with detailed statistics
 */
struct MeshCleaningResult {
    bool success = false;
    std::string operation;

    // Component statistics
    size_t total_components_found = 0;
    size_t components_removed = 0;
    size_t components_kept = 0;

    // Triangle statistics
    size_t triangles_before = 0;
    size_t triangles_after = 0;
    size_t triangles_removed = 0;

    // Vertex statistics
    size_t vertices_before = 0;
    size_t vertices_after = 0;
    size_t vertices_removed = 0;
    size_t duplicate_vertices_removed = 0;

    // Area statistics
    double total_area_before = 0.0;     // mm²
    double total_area_after = 0.0;      // mm²
    double largest_component_area = 0.0; // mm²

    // Performance metrics
    double processing_time_ms = 0.0;
    size_t memory_usage_mb = 0;

    /**
     * @brief Get human-readable result string
     */
    std::string toString() const;

    /**
     * @brief Get reduction percentage
     */
    double getReductionPercentage() const {
        if (triangles_before == 0) return 0.0;
        return 100.0 * static_cast<double>(triangles_removed) / static_cast<double>(triangles_before);
    }
};

/**
 * @brief Advanced mesh cleaning for industrial 3D scanning applications
 *
 * Provides professional-grade mesh cleaning matching Unlook Studio quality:
 * - Small object filtering (isolated fragments removal)
 * - Connected component analysis
 * - Duplicate vertex removal
 * - Unreferenced geometry cleanup
 *
 * Thread-safe implementation with comprehensive error handling.
 * Optimized for ARM64 (Raspberry Pi CM4/CM5) with parallel processing.
 */
class MeshCleaner {
public:
    MeshCleaner();
    ~MeshCleaner();

    /**
     * @brief Remove small isolated objects from mesh
     *
     * Analyzes connected components and removes small artifacts based on
     * configuration. Matching Unlook Studio's Small Objects Filter.
     *
     * @param vertices Input mesh vertices (modified in-place)
     * @param faces Input mesh faces (modified in-place)
     * @param normals Optional vertex normals (updated if provided)
     * @param config Cleaning configuration
     * @param result Output cleaning statistics
     * @return true if cleaning successful
     *
     * @note Thread-safe operation with internal mutex protection
     */
    bool removeSmallObjects(std::vector<cv::Vec3f>& vertices,
                           std::vector<cv::Vec3i>& faces,
                           std::vector<cv::Vec3f>& normals,
                           const MeshCleaningConfig& config,
                           MeshCleaningResult& result);

#ifdef OPEN3D_ENABLED
    /**
     * @brief Remove small isolated objects from Open3D mesh
     *
     * High-performance implementation using Open3D's native clustering.
     * Recommended for best quality and performance.
     *
     * @param mesh Input mesh (creates new cleaned mesh)
     * @param config Cleaning configuration
     * @param result Output cleaning statistics
     * @return Cleaned mesh (nullptr if failed)
     *
     * @note Returns new mesh, original unchanged
     */
    std::shared_ptr<open3d::geometry::TriangleMesh> removeSmallObjects(
        const std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
        const MeshCleaningConfig& config,
        MeshCleaningResult& result);

    /**
     * @brief Clean Open3D mesh in-place with all filters
     *
     * Comprehensive cleaning pipeline:
     * 1. Remove duplicate vertices
     * 2. Remove unreferenced vertices
     * 3. Remove degenerate triangles
     * 4. Filter small components
     * 5. Recompute normals
     *
     * @param mesh Input mesh (modified in-place)
     * @param config Cleaning configuration
     * @param result Output cleaning statistics
     * @return true if cleaning successful
     */
    bool cleanMeshInPlace(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                         const MeshCleaningConfig& config,
                         MeshCleaningResult& result);
#endif

    /**
     * @brief Analyze connected components without modification
     *
     * Useful for preview/analysis before cleaning.
     *
     * @param vertices Input mesh vertices
     * @param faces Input mesh faces
     * @param component_sizes Output vector of component sizes (triangles)
     * @param component_areas Output vector of component areas (mm²)
     * @return Number of connected components found
     */
    size_t analyzeComponents(const std::vector<cv::Vec3f>& vertices,
                            const std::vector<cv::Vec3i>& faces,
                            std::vector<size_t>& component_sizes,
                            std::vector<double>& component_areas) const;

    /**
     * @brief Remove duplicate vertices from mesh
     *
     * Merges vertices at same position (within epsilon threshold).
     * Updates face indices accordingly.
     *
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @param epsilon Distance threshold for duplicate detection (default 1e-9)
     * @return Number of duplicate vertices removed
     */
    size_t removeDuplicateVertices(std::vector<cv::Vec3f>& vertices,
                                   std::vector<cv::Vec3i>& faces,
                                   double epsilon = 1e-9);

    /**
     * @brief Remove unreferenced vertices from mesh
     *
     * Removes vertices not used by any triangle.
     * Updates face indices accordingly.
     *
     * @param vertices Mesh vertices (modified in-place)
     * @param faces Mesh faces (modified in-place)
     * @return Number of unreferenced vertices removed
     */
    size_t removeUnreferencedVertices(std::vector<cv::Vec3f>& vertices,
                                      std::vector<cv::Vec3i>& faces);

    /**
     * @brief Set cleaning progress callback
     * @param callback Function called with progress percentage (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

    /**
     * @brief Get last error message
     * @return Error description string
     */
    std::string getLastError() const;

    /**
     * @brief Enable ARM64 NEON optimizations
     * @param enable Enable ARM64-specific optimizations (default true)
     */
    void enableARM64Optimizations(bool enable = true);

    /**
     * @brief Get cleaning performance statistics
     * @return Map of operation name to execution time (ms)
     */
    std::map<std::string, double> getPerformanceStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Internal cleaning mutex for thread safety
    mutable std::mutex cleaning_mutex_;

    // Helper methods
    bool validateMeshInput(const std::vector<cv::Vec3f>& vertices,
                          const std::vector<cv::Vec3i>& faces) const;

    void computeTotalArea(const std::vector<cv::Vec3f>& vertices,
                         const std::vector<cv::Vec3i>& faces,
                         double& total_area) const;

    void buildFaceComponentMapping(const std::vector<cv::Vec3i>& faces,
                                   size_t num_vertices,
                                   std::vector<int>& face_to_component,
                                   size_t& num_components) const;

    // Disable copy construction and assignment
    MeshCleaner(const MeshCleaner&) = delete;
    MeshCleaner& operator=(const MeshCleaner&) = delete;
};

} // namespace mesh
} // namespace unlook
