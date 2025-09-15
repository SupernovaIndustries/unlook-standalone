#pragma once

#include "unlook/core/types.hpp"
#include "unlook/mesh/MeshValidator.hpp"
#include <opencv2/core.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

namespace unlook {
namespace mesh {

/**
 * @brief Industrial mesh export configuration
 */
struct IndustrialExportConfig {
    enum class Format {
        STL_ASCII,              // ASCII STL for 3D printing
        STL_BINARY,             // Binary STL for 3D printing
        OBJ_WITH_MATERIALS,     // OBJ with MTL file
        PLY_INDUSTRIAL,         // PLY with extended metadata
        X3D_MANUFACTURING,      // X3D for CAD integration
        GLTF_PRECISION         // glTF for visualization
    };

    Format format = Format::STL_BINARY;

    // Precision settings
    double coordinatePrecision = 0.005;         // Coordinate precision (mm)
    int decimalPlaces = 6;                      // Decimal places in ASCII formats
    bool highPrecisionMode = true;              // Use double precision

    // Unit conversion
    enum class Units {
        MILLIMETERS,
        METERS,
        INCHES
    };
    Units units = Units::MILLIMETERS;

    // Manufacturing metadata
    std::string partName;
    std::string partNumber;
    std::string material = "PLA";               // Default 3D printing material
    std::string manufacturer = "Unlook Scanner";
    std::string scanDate;
    std::string operator_;
    double tolerance = 0.1;                     // Manufacturing tolerance (mm)

    // Quality requirements
    bool requireWatertight = true;              // Require watertight mesh
    bool validateBeforeExport = true;           // Validate mesh quality
    double minTriangleQuality = 0.3;            // Minimum triangle quality

    // STL specific settings
    std::string stlHeader = "Unlook 3D Scanner - Industrial Mesh";
    bool includeNormals = true;                 // Include face normals in STL

    // OBJ specific settings
    bool generateMTL = true;                    // Generate MTL material file
    bool includeTexCoords = false;              // Include texture coordinates
    bool includeVertexNormals = true;           // Include vertex normals
    std::string materialName = "DefaultMaterial";

    // PLY specific settings
    bool includeQualityMetrics = true;          // Include quality metrics as comments
    bool includeCalibrationInfo = true;         // Include calibration information
    bool compressBinary = true;                 // Compress binary data

    bool validate() const;
    std::string toString() const;
    std::string getFileExtension() const;
    double getScaleFactor() const;              // Get scale factor for unit conversion
};

/**
 * @brief Export result with comprehensive information
 */
struct ExportResult {
    bool success = false;
    std::string format;
    std::string filePath;

    // File statistics
    size_t fileSizeBytes = 0;
    double exportTimeMs = 0.0;

    // Mesh statistics
    size_t verticesExported = 0;
    size_t facesExported = 0;
    size_t normalsExported = 0;

    // Quality assessment
    MeshQualityMetrics meshQuality;
    bool qualityPassed = false;
    std::vector<std::string> qualityWarnings;

    // Manufacturing readiness
    bool manufacturingReady = false;
    std::vector<std::string> manufacturingIssues;

    std::string errorMessage;
    std::string toString() const;
};

/**
 * @brief Advanced industrial mesh exporter
 *
 * Provides high-precision mesh export for manufacturing and industrial applications.
 * Supports STL, OBJ, PLY, and other formats with comprehensive metadata and validation.
 * Optimized for 0.005mm precision requirements.
 */
class IndustrialMeshExporter {
public:
    IndustrialMeshExporter();
    ~IndustrialMeshExporter();

    /**
     * @brief Export mesh with industrial precision and validation
     * @param vertices Mesh vertices in millimeters
     * @param faces Triangle faces
     * @param normals Vertex normals (optional)
     * @param filePath Output file path
     * @param config Export configuration
     * @param result Output export result
     * @return true if export successful
     */
    bool exportMesh(const std::vector<cv::Vec3f>& vertices,
                   const std::vector<cv::Vec3i>& faces,
                   const std::vector<cv::Vec3f>& normals,
                   const std::string& filePath,
                   const IndustrialExportConfig& config,
                   ExportResult& result);

    /**
     * @brief Export STL file optimized for 3D printing
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param filePath Output STL file path
     * @param config Export configuration
     * @param result Output export result
     * @return true if export successful
     */
    bool exportSTL(const std::vector<cv::Vec3f>& vertices,
                  const std::vector<cv::Vec3i>& faces,
                  const std::string& filePath,
                  const IndustrialExportConfig& config,
                  ExportResult& result);

    /**
     * @brief Export OBJ file with materials for CAD applications
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param normals Vertex normals
     * @param filePath Output OBJ file path
     * @param config Export configuration
     * @param result Output export result
     * @return true if export successful
     */
    bool exportOBJ(const std::vector<cv::Vec3f>& vertices,
                  const std::vector<cv::Vec3i>& faces,
                  const std::vector<cv::Vec3f>& normals,
                  const std::string& filePath,
                  const IndustrialExportConfig& config,
                  ExportResult& result);

    /**
     * @brief Export PLY file with extended industrial metadata
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param normals Vertex normals
     * @param filePath Output PLY file path
     * @param config Export configuration
     * @param result Output export result
     * @return true if export successful
     */
    bool exportPLY(const std::vector<cv::Vec3f>& vertices,
                  const std::vector<cv::Vec3i>& faces,
                  const std::vector<cv::Vec3f>& normals,
                  const std::string& filePath,
                  const IndustrialExportConfig& config,
                  ExportResult& result);

    /**
     * @brief Export X3D file for CAD integration
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param normals Vertex normals
     * @param filePath Output X3D file path
     * @param config Export configuration
     * @param result Output export result
     * @return true if export successful
     */
    bool exportX3D(const std::vector<cv::Vec3f>& vertices,
                  const std::vector<cv::Vec3i>& faces,
                  const std::vector<cv::Vec3f>& normals,
                  const std::string& filePath,
                  const IndustrialExportConfig& config,
                  ExportResult& result);

    /**
     * @brief Validate mesh for manufacturing before export
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param config Export configuration
     * @param warnings Output validation warnings
     * @param errors Output validation errors
     * @return true if mesh passes validation
     */
    bool validateForManufacturing(const std::vector<cv::Vec3f>& vertices,
                                 const std::vector<cv::Vec3i>& faces,
                                 const IndustrialExportConfig& config,
                                 std::vector<std::string>& warnings,
                                 std::vector<std::string>& errors);

    /**
     * @brief Generate manufacturing report with mesh statistics
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param normals Vertex normals
     * @param config Export configuration
     * @return Manufacturing report string
     */
    std::string generateManufacturingReport(const std::vector<cv::Vec3f>& vertices,
                                           const std::vector<cv::Vec3i>& faces,
                                           const std::vector<cv::Vec3f>& normals,
                                           const IndustrialExportConfig& config);

    /**
     * @brief Estimate 3D printing time and material usage
     * @param vertices Mesh vertices
     * @param faces Triangle faces
     * @param layerHeight Layer height in mm
     * @param infillDensity Infill density [0,1]
     * @param printSpeed Print speed in mm/s
     * @param printTimeMinutes Output estimated print time
     * @param materialGrams Output estimated material usage
     * @return true if estimation successful
     */
    bool estimate3DPrintingParameters(const std::vector<cv::Vec3f>& vertices,
                                     const std::vector<cv::Vec3i>& faces,
                                     double layerHeight,
                                     double infillDensity,
                                     double printSpeed,
                                     double& printTimeMinutes,
                                     double& materialGrams);

    /**
     * @brief Set export progress callback
     * @param callback Function called with progress (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);

    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    /**
     * @brief Get export performance statistics
     * @return Map of operation name to execution time (ms)
     */
    std::map<std::string, double> getPerformanceStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    // Helper methods
    bool writeSTLAscii(const std::vector<cv::Vec3f>& vertices,
                      const std::vector<cv::Vec3i>& faces,
                      const std::string& filePath,
                      const IndustrialExportConfig& config);

    bool writeSTLBinary(const std::vector<cv::Vec3f>& vertices,
                       const std::vector<cv::Vec3i>& faces,
                       const std::string& filePath,
                       const IndustrialExportConfig& config);

    bool writeMTLFile(const std::string& mtlPath,
                     const IndustrialExportConfig& config);

    void scaleVertices(std::vector<cv::Vec3f>& vertices,
                      double scaleFactor);

    cv::Vec3f computeFaceNormal(const cv::Vec3f& v0,
                               const cv::Vec3f& v1,
                               const cv::Vec3f& v2);

    std::string formatCoordinate(double value, int precision);
    std::string getCurrentTimestamp();
    size_t calculateFileSize(const std::string& filePath);

    // Disable copy construction and assignment
    IndustrialMeshExporter(const IndustrialMeshExporter&) = delete;
    IndustrialMeshExporter& operator=(const IndustrialMeshExporter&) = delete;
};

/**
 * @brief Utility functions for industrial export
 */
namespace industrial_export {

/**
 * @brief Convert mesh units
 * @param vertices Mesh vertices (modified in-place)
 * @param fromUnits Source units
 * @param toUnits Target units
 * @return Conversion factor applied
 */
double convertUnits(std::vector<cv::Vec3f>& vertices,
                   IndustrialExportConfig::Units fromUnits,
                   IndustrialExportConfig::Units toUnits);

/**
 * @brief Generate material properties for common 3D printing materials
 * @param materialName Material name (e.g., "PLA", "ABS", "PETG")
 * @return Material properties map
 */
std::map<std::string, std::string> getMaterialProperties(const std::string& materialName);

/**
 * @brief Validate file path and create directories if needed
 * @param filePath File path to validate
 * @param extension Required file extension
 * @return true if path is valid
 */
bool validateFilePath(const std::string& filePath, const std::string& extension);

/**
 * @brief Compute mesh bounding box for manufacturing constraints
 * @param vertices Mesh vertices
 * @param minBounds Output minimum bounds
 * @param maxBounds Output maximum bounds
 * @param dimensions Output dimensions (width, height, depth)
 */
void computeMeshBounds(const std::vector<cv::Vec3f>& vertices,
                      cv::Vec3f& minBounds,
                      cv::Vec3f& maxBounds,
                      cv::Vec3f& dimensions);

/**
 * @brief Check if mesh fits within manufacturing constraints
 * @param vertices Mesh vertices
 * @param maxDimensions Maximum allowed dimensions
 * @param maxVolume Maximum allowed volume
 * @return true if mesh fits constraints
 */
bool checkManufacturingConstraints(const std::vector<cv::Vec3f>& vertices,
                                  const std::vector<cv::Vec3i>& faces,
                                  const cv::Vec3f& maxDimensions,
                                  double maxVolume);

} // namespace industrial_export

} // namespace mesh
} // namespace unlook