#include "unlook/mesh/IndustrialMeshExporter.hpp"
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
#include <filesystem>

namespace unlook {
namespace mesh {

// IndustrialMeshExporter implementation
class IndustrialMeshExporter::Impl {
public:
    std::string lastError;
    std::function<void(int)> progressCallback;
    std::map<std::string, double> performanceStats;
    mutable std::mutex statsMutex;
    MeshValidator validator;

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

IndustrialMeshExporter::IndustrialMeshExporter() : pImpl(std::make_unique<Impl>()) {}
IndustrialMeshExporter::~IndustrialMeshExporter() = default;

bool IndustrialMeshExporter::exportMesh(const std::vector<cv::Vec3f>& vertices,
                                       const std::vector<cv::Vec3i>& faces,
                                       const std::vector<cv::Vec3f>& normals,
                                       const std::string& filePath,
                                       const IndustrialExportConfig& config,
                                       ExportResult& result) {
    if (vertices.empty() || faces.empty()) {
        pImpl->lastError = "Empty mesh data for export";
        return false;
    }

    if (!config.validate()) {
        pImpl->lastError = "Invalid export configuration";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();
    result = {};
    result.filePath = filePath;
    result.format = config.toString();

    try {
        pImpl->updateProgress(10);

        // Validate mesh if required
        if (config.validateBeforeExport) {
            std::vector<std::string> warnings, errors;
            result.qualityPassed = validateForManufacturing(vertices, faces, config, warnings, errors);
            result.qualityWarnings = warnings;
            result.manufacturingIssues = errors;

            if (!result.qualityPassed && config.requireWatertight) {
                pImpl->lastError = "Mesh failed manufacturing validation";
                return false;
            }
        }

        pImpl->updateProgress(20);

        // Create a copy of vertices for unit conversion
        std::vector<cv::Vec3f> scaledVertices = vertices;
        double scaleFactor = config.getScaleFactor();
        if (scaleFactor != 1.0) {
            scaleVertices(scaledVertices, scaleFactor);
        }

        pImpl->updateProgress(30);

        // Export based on format
        bool exportSuccess = false;
        switch (config.format) {
            case IndustrialExportConfig::Format::STL_ASCII:
            case IndustrialExportConfig::Format::STL_BINARY:
                exportSuccess = exportSTL(scaledVertices, faces, filePath, config, result);
                break;

            case IndustrialExportConfig::Format::OBJ_WITH_MATERIALS:
                exportSuccess = exportOBJ(scaledVertices, faces, normals, filePath, config, result);
                break;

            case IndustrialExportConfig::Format::PLY_INDUSTRIAL:
                exportSuccess = exportPLY(scaledVertices, faces, normals, filePath, config, result);
                break;

            case IndustrialExportConfig::Format::X3D_MANUFACTURING:
                exportSuccess = exportX3D(scaledVertices, faces, normals, filePath, config, result);
                break;

            default:
                pImpl->lastError = "Unsupported export format";
                return false;
        }

        if (!exportSuccess) {
            return false;
        }

        pImpl->updateProgress(90);

        // Compute export statistics
        result.verticesExported = vertices.size();
        result.facesExported = faces.size();
        result.normalsExported = normals.size();
        result.fileSizeBytes = calculateFileSize(filePath);

        // Assess manufacturing readiness
        MeshValidationConfig validationConfig;
        pImpl->validator.validateMesh(vertices, faces, normals, validationConfig, result.meshQuality);
        result.manufacturingReady = result.meshQuality.isManufacturingReady();

        auto endTime = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        result.exportTimeMs = duration.count();

        result.success = true;
        pImpl->updatePerformanceStats("export_" + std::to_string(static_cast<int>(config.format)), result.exportTimeMs);
        pImpl->updateProgress(100);

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "Mesh export failed: " + std::string(e.what());
        result.errorMessage = pImpl->lastError;
        return false;
    }
}

bool IndustrialMeshExporter::exportSTL(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const std::string& filePath,
                                      const IndustrialExportConfig& config,
                                      ExportResult& result) {
    try {
        bool success = false;
        if (config.format == IndustrialExportConfig::Format::STL_ASCII) {
            success = writeSTLAscii(vertices, faces, filePath, config);
        } else {
            success = writeSTLBinary(vertices, faces, filePath, config);
        }

        if (!success) {
            pImpl->lastError = "Failed to write STL file";
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "STL export failed: " + std::string(e.what());
        return false;
    }
}

bool IndustrialMeshExporter::exportOBJ(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const std::vector<cv::Vec3f>& normals,
                                      const std::string& filePath,
                                      const IndustrialExportConfig& config,
                                      ExportResult& result) {
    try {
        std::ofstream file(filePath);
        if (!file.is_open()) {
            pImpl->lastError = "Failed to open OBJ file for writing: " + filePath;
            return false;
        }

        // Write header
        file << "# OBJ file generated by " << config.manufacturer << "\n";
        file << "# Part: " << config.partName << "\n";
        file << "# Part Number: " << config.partNumber << "\n";
        file << "# Material: " << config.material << "\n";
        file << "# Scan Date: " << config.scanDate << "\n";
        file << "# Precision: " << config.coordinatePrecision << " mm\n";
        file << "# Tolerance: " << config.tolerance << " mm\n";

        if (config.generateMTL) {
            std::string mtlFile = filePath.substr(0, filePath.find_last_of(".")) + ".mtl";
            file << "mtllib " << std::filesystem::path(mtlFile).filename().string() << "\n";
            writeMTLFile(mtlFile, config);
        }

        file << "\n";

        // Write vertices
        file << std::fixed << std::setprecision(config.decimalPlaces);
        for (const auto& vertex : vertices) {
            file << "v " << formatCoordinate(vertex[0], config.decimalPlaces)
                 << " " << formatCoordinate(vertex[1], config.decimalPlaces)
                 << " " << formatCoordinate(vertex[2], config.decimalPlaces) << "\n";
        }

        pImpl->updateProgress(60);

        // Write vertex normals
        if (config.includeVertexNormals && !normals.empty()) {
            for (const auto& normal : normals) {
                file << "vn " << formatCoordinate(normal[0], config.decimalPlaces)
                     << " " << formatCoordinate(normal[1], config.decimalPlaces)
                     << " " << formatCoordinate(normal[2], config.decimalPlaces) << "\n";
            }
        }

        pImpl->updateProgress(70);

        // Write material group
        if (config.generateMTL) {
            file << "\nusemtl " << config.materialName << "\n";
        }

        // Write faces
        bool hasNormals = config.includeVertexNormals && !normals.empty() && normals.size() == vertices.size();

        for (const auto& face : faces) {
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {

                file << "f ";
                for (int i = 0; i < 3; ++i) {
                    file << (face[i] + 1); // OBJ indices are 1-based
                    if (hasNormals) {
                        file << "//" << (face[i] + 1);
                    }
                    if (i < 2) file << " ";
                }
                file << "\n";
            }
        }

        file.close();
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "OBJ export failed: " + std::string(e.what());
        return false;
    }
}

bool IndustrialMeshExporter::exportPLY(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const std::vector<cv::Vec3f>& normals,
                                      const std::string& filePath,
                                      const IndustrialExportConfig& config,
                                      ExportResult& result) {
    try {
        std::ofstream file(filePath);
        if (!file.is_open()) {
            pImpl->lastError = "Failed to open PLY file for writing: " + filePath;
            return false;
        }

        // Count valid vertices and faces
        size_t validVertices = 0;
        size_t validFaces = 0;

        for (const auto& vertex : vertices) {
            if (std::isfinite(vertex[0]) && std::isfinite(vertex[1]) && std::isfinite(vertex[2])) {
                validVertices++;
            }
        }

        for (const auto& face : faces) {
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {
                validFaces++;
            }
        }

        // Write PLY header
        file << "ply\n";
        file << "format ascii 1.0\n";

        // Industrial metadata comments
        if (config.includeQualityMetrics) {
            file << "comment Generated by " << config.manufacturer << "\n";
            file << "comment Part: " << config.partName << "\n";
            file << "comment Part Number: " << config.partNumber << "\n";
            file << "comment Material: " << config.material << "\n";
            file << "comment Scan Date: " << config.scanDate << "\n";
            file << "comment Operator: " << config.operator_ << "\n";
            file << "comment Precision: " << config.coordinatePrecision << " mm\n";
            file << "comment Tolerance: " << config.tolerance << " mm\n";
            file << "comment Units: ";
            switch (config.units) {
                case IndustrialExportConfig::Units::MILLIMETERS: file << "millimeters\n"; break;
                case IndustrialExportConfig::Units::METERS: file << "meters\n"; break;
                case IndustrialExportConfig::Units::INCHES: file << "inches\n"; break;
            }
        }

        // Element definitions
        file << "element vertex " << validVertices << "\n";
        file << "property float x\n";
        file << "property float y\n";
        file << "property float z\n";

        if (config.includeVertexNormals && !normals.empty()) {
            file << "property float nx\n";
            file << "property float ny\n";
            file << "property float nz\n";
        }

        file << "element face " << validFaces << "\n";
        file << "property list uchar int vertex_indices\n";

        file << "end_header\n";

        // Write vertex data
        file << std::fixed << std::setprecision(config.decimalPlaces);
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& vertex = vertices[i];
            if (std::isfinite(vertex[0]) && std::isfinite(vertex[1]) && std::isfinite(vertex[2])) {
                file << formatCoordinate(vertex[0], config.decimalPlaces) << " "
                     << formatCoordinate(vertex[1], config.decimalPlaces) << " "
                     << formatCoordinate(vertex[2], config.decimalPlaces);

                if (config.includeVertexNormals && !normals.empty() && i < normals.size()) {
                    const auto& normal = normals[i];
                    file << " " << formatCoordinate(normal[0], config.decimalPlaces)
                         << " " << formatCoordinate(normal[1], config.decimalPlaces)
                         << " " << formatCoordinate(normal[2], config.decimalPlaces);
                }

                file << "\n";
            }
        }

        pImpl->updateProgress(80);

        // Write face data
        for (const auto& face : faces) {
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {
                file << "3 " << face[0] << " " << face[1] << " " << face[2] << "\n";
            }
        }

        file.close();
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "PLY export failed: " + std::string(e.what());
        return false;
    }
}

bool IndustrialMeshExporter::exportX3D(const std::vector<cv::Vec3f>& vertices,
                                      const std::vector<cv::Vec3i>& faces,
                                      const std::vector<cv::Vec3f>& normals,
                                      const std::string& filePath,
                                      const IndustrialExportConfig& config,
                                      ExportResult& result) {
    try {
        std::ofstream file(filePath);
        if (!file.is_open()) {
            pImpl->lastError = "Failed to open X3D file for writing: " + filePath;
            return false;
        }

        // Write X3D header
        file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        file << "<!DOCTYPE X3D PUBLIC \"ISO//Web3D//DTD X3D 3.3//EN\" \"http://www.web3d.org/specifications/x3d-3.3.dtd\">\n";
        file << "<X3D profile=\"Interchange\" version=\"3.3\">\n";
        file << "  <head>\n";
        file << "    <meta name=\"title\" content=\"" << config.partName << "\"/>\n";
        file << "    <meta name=\"creator\" content=\"" << config.manufacturer << "\"/>\n";
        file << "    <meta name=\"created\" content=\"" << config.scanDate << "\"/>\n";
        file << "    <meta name=\"precision\" content=\"" << config.coordinatePrecision << " mm\"/>\n";
        file << "  </head>\n";
        file << "  <Scene>\n";
        file << "    <Shape>\n";
        file << "      <IndexedFaceSet coordIndex=\"";

        // Write face indices
        for (size_t i = 0; i < faces.size(); ++i) {
            const auto& face = faces[i];
            if (face[0] >= 0 && face[0] < vertices.size() &&
                face[1] >= 0 && face[1] < vertices.size() &&
                face[2] >= 0 && face[2] < vertices.size()) {
                file << face[0] << " " << face[1] << " " << face[2] << " -1";
                if (i < faces.size() - 1) file << " ";
            }
        }

        file << "\">\n";
        file << "        <Coordinate point=\"";

        // Write vertices
        file << std::fixed << std::setprecision(config.decimalPlaces);
        for (size_t i = 0; i < vertices.size(); ++i) {
            const auto& vertex = vertices[i];
            file << formatCoordinate(vertex[0], config.decimalPlaces) << " "
                 << formatCoordinate(vertex[1], config.decimalPlaces) << " "
                 << formatCoordinate(vertex[2], config.decimalPlaces);
            if (i < vertices.size() - 1) file << " ";
        }

        file << "\"/>\n";
        file << "      </IndexedFaceSet>\n";
        file << "      <Appearance>\n";
        file << "        <Material diffuseColor=\"0.8 0.8 0.8\" specularColor=\"0.2 0.2 0.2\"/>\n";
        file << "      </Appearance>\n";
        file << "    </Shape>\n";
        file << "  </Scene>\n";
        file << "</X3D>\n";

        file.close();
        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "X3D export failed: " + std::string(e.what());
        return false;
    }
}

bool IndustrialMeshExporter::validateForManufacturing(const std::vector<cv::Vec3f>& vertices,
                                                     const std::vector<cv::Vec3i>& faces,
                                                     const IndustrialExportConfig& config,
                                                     std::vector<std::string>& warnings,
                                                     std::vector<std::string>& errors) {
    warnings.clear();
    errors.clear();

    try {
        MeshValidationConfig validationConfig;
        validationConfig.requireWatertight = config.requireWatertight;
        validationConfig.minTriangleQuality = config.minTriangleQuality;
        validationConfig.targetPrecision = config.coordinatePrecision;

        MeshQualityMetrics metrics;
        if (!pImpl->validator.validateMesh(vertices, faces, {}, validationConfig, metrics)) {
            errors.push_back("Mesh validation failed");
            return false;
        }

        // Check manufacturing readiness
        if (config.requireWatertight && !metrics.isWatertight) {
            errors.push_back("Mesh is not watertight - required for 3D printing");
        }

        if (!metrics.isManifold) {
            warnings.push_back("Mesh has non-manifold geometry");
        }

        if (metrics.hasSelfIntersections) {
            warnings.push_back("Mesh has self-intersections");
        }

        if (metrics.triangleQualityMean < config.minTriangleQuality) {
            warnings.push_back("Low triangle quality detected (mean: " +
                              std::to_string(metrics.triangleQualityMean) + ")");
        }

        if (metrics.numHoles > 0) {
            warnings.push_back("Mesh has " + std::to_string(metrics.numHoles) + " holes");
        }

        // Check dimensions for manufacturing constraints
        cv::Vec3f minBounds, maxBounds, dimensions;
        industrial_export::computeMeshBounds(vertices, minBounds, maxBounds, dimensions);

        if (dimensions[0] > 300.0 || dimensions[1] > 300.0 || dimensions[2] > 300.0) {
            warnings.push_back("Large mesh dimensions may exceed printer build volume");
        }

        if (dimensions[0] < 1.0 || dimensions[1] < 1.0 || dimensions[2] < 1.0) {
            warnings.push_back("Very small dimensions may be difficult to print accurately");
        }

        return errors.empty();

    } catch (const std::exception& e) {
        errors.push_back("Validation failed: " + std::string(e.what()));
        return false;
    }
}

std::string IndustrialMeshExporter::generateManufacturingReport(const std::vector<cv::Vec3f>& vertices,
                                                               const std::vector<cv::Vec3i>& faces,
                                                               const std::vector<cv::Vec3f>& normals,
                                                               const IndustrialExportConfig& config) {
    std::stringstream report;

    report << "=== MANUFACTURING READINESS REPORT ===\n\n";

    // Part information
    report << "PART INFORMATION:\n";
    report << "  Part Name: " << config.partName << "\n";
    report << "  Part Number: " << config.partNumber << "\n";
    report << "  Material: " << config.material << "\n";
    report << "  Scan Date: " << config.scanDate << "\n";
    report << "  Operator: " << config.operator_ << "\n";
    report << "  Tolerance: " << config.tolerance << " mm\n\n";

    // Mesh statistics
    report << "MESH STATISTICS:\n";
    report << "  Vertices: " << vertices.size() << "\n";
    report << "  Faces: " << faces.size() << "\n";
    report << "  Normals: " << normals.size() << "\n\n";

    // Dimensions and volume
    cv::Vec3f minBounds, maxBounds, dimensions;
    industrial_export::computeMeshBounds(vertices, minBounds, maxBounds, dimensions);

    report << "DIMENSIONS:\n";
    report << "  Width (X): " << std::fixed << std::setprecision(3) << dimensions[0] << " mm\n";
    report << "  Height (Y): " << std::fixed << std::setprecision(3) << dimensions[1] << " mm\n";
    report << "  Depth (Z): " << std::fixed << std::setprecision(3) << dimensions[2] << " mm\n";

    // Calculate volume
    double volume = 0.0;
    for (const auto& face : faces) {
        if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
            const cv::Vec3f& v0 = vertices[face[0]];
            const cv::Vec3f& v1 = vertices[face[1]];
            const cv::Vec3f& v2 = vertices[face[2]];

            volume += (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                      v0[1] * (v1[2] * v2[0] - v1[0] * v2[2]) +
                      v0[2] * (v1[0] * v2[1] - v1[1] * v2[0])) / 6.0;
        }
    }
    volume = std::abs(volume) / 1000.0; // Convert mm³ to cm³

    report << "  Volume: " << std::fixed << std::setprecision(3) << volume << " cm³\n\n";

    // Quality assessment
    MeshValidationConfig validationConfig;
    MeshQualityMetrics metrics;
    pImpl->validator.validateMesh(vertices, faces, normals, validationConfig, metrics);

    report << "QUALITY ASSESSMENT:\n";
    report << "  Watertight: " << (metrics.isWatertight ? "YES" : "NO") << "\n";
    report << "  Manifold: " << (metrics.isManifold ? "YES" : "NO") << "\n";
    report << "  Self-intersections: " << (metrics.hasSelfIntersections ? "YES" : "NO") << "\n";
    report << "  Triangle quality: " << std::fixed << std::setprecision(3) << metrics.triangleQualityMean << "\n";
    report << "  Manufacturing ready: " << (metrics.isManufacturingReady() ? "YES" : "NO") << "\n\n";

    // 3D printing estimates
    if (config.material == "PLA" || config.material == "ABS" || config.material == "PETG") {
        double printTime, materialGrams;
        if (estimate3DPrintingParameters(vertices, faces, 0.2, 0.2, 50.0, printTime, materialGrams)) {
            report << "3D PRINTING ESTIMATES:\n";
            report << "  Print time: " << std::fixed << std::setprecision(1) << printTime << " minutes\n";
            report << "  Material usage: " << std::fixed << std::setprecision(1) << materialGrams << " grams\n\n";
        }
    }

    return report.str();
}

bool IndustrialMeshExporter::estimate3DPrintingParameters(const std::vector<cv::Vec3f>& vertices,
                                                         const std::vector<cv::Vec3i>& faces,
                                                         double layerHeight,
                                                         double infillDensity,
                                                         double printSpeed,
                                                         double& printTimeMinutes,
                                                         double& materialGrams) {
    try {
        // Calculate volume
        double volume = 0.0;
        for (const auto& face : faces) {
            if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                const cv::Vec3f& v0 = vertices[face[0]];
                const cv::Vec3f& v1 = vertices[face[1]];
                const cv::Vec3f& v2 = vertices[face[2]];

                volume += (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                          v0[1] * (v1[2] * v2[0] - v1[0] * v2[2]) +
                          v0[2] * (v1[0] * v2[1] - v1[1] * v2[0])) / 6.0;
            }
        }
        volume = std::abs(volume); // mm³

        // Calculate surface area
        double surfaceArea = 0.0;
        for (const auto& face : faces) {
            if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                const cv::Vec3f& v0 = vertices[face[0]];
                const cv::Vec3f& v1 = vertices[face[1]];
                const cv::Vec3f& v2 = vertices[face[2]];

                cv::Vec3f edge1 = v1 - v0;
                cv::Vec3f edge2 = v2 - v0;
                cv::Vec3f cross = edge1.cross(edge2);
                surfaceArea += cv::norm(cross) * 0.5;
            }
        }

        // Get mesh bounds
        cv::Vec3f minBounds, maxBounds, dimensions;
        industrial_export::computeMeshBounds(vertices, minBounds, maxBounds, dimensions);

        // Estimate number of layers
        double height = dimensions[2];
        int numLayers = static_cast<int>(std::ceil(height / layerHeight));

        // Estimate print time (simplified)
        double perimeterLength = std::sqrt(surfaceArea) * 4.0; // Rough estimate
        double infillVolume = volume * infillDensity;
        double totalExtrusionLength = perimeterLength + infillVolume / (layerHeight * 0.4); // 0.4mm line width

        printTimeMinutes = totalExtrusionLength / printSpeed / 60.0; // Convert to minutes

        // Estimate material usage (PLA density ~1.24 g/cm³)
        double materialVolume = totalExtrusionLength * 0.4 * layerHeight; // mm³
        materialGrams = (materialVolume / 1000.0) * 1.24; // Convert to grams

        return true;

    } catch (const std::exception& e) {
        pImpl->lastError = "3D printing estimation failed: " + std::string(e.what());
        return false;
    }
}

void IndustrialMeshExporter::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

std::string IndustrialMeshExporter::getLastError() const {
    return pImpl->lastError;
}

std::map<std::string, double> IndustrialMeshExporter::getPerformanceStats() const {
    std::lock_guard<std::mutex> lock(pImpl->statsMutex);
    return pImpl->performanceStats;
}

// Helper method implementations
bool IndustrialMeshExporter::writeSTLAscii(const std::vector<cv::Vec3f>& vertices,
                                          const std::vector<cv::Vec3i>& faces,
                                          const std::string& filePath,
                                          const IndustrialExportConfig& config) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open STL file for writing: " + filePath;
        return false;
    }

    // Write header
    file << "solid " << config.stlHeader << "\n";

    // Write triangles
    file << std::fixed << std::setprecision(config.decimalPlaces);
    for (const auto& face : faces) {
        if (face[0] >= 0 && face[0] < vertices.size() &&
            face[1] >= 0 && face[1] < vertices.size() &&
            face[2] >= 0 && face[2] < vertices.size()) {

            const cv::Vec3f& v0 = vertices[face[0]];
            const cv::Vec3f& v1 = vertices[face[1]];
            const cv::Vec3f& v2 = vertices[face[2]];

            // Compute face normal
            cv::Vec3f normal = computeFaceNormal(v0, v1, v2);

            file << "  facet normal " << formatCoordinate(normal[0], config.decimalPlaces)
                 << " " << formatCoordinate(normal[1], config.decimalPlaces)
                 << " " << formatCoordinate(normal[2], config.decimalPlaces) << "\n";
            file << "    outer loop\n";
            file << "      vertex " << formatCoordinate(v0[0], config.decimalPlaces)
                 << " " << formatCoordinate(v0[1], config.decimalPlaces)
                 << " " << formatCoordinate(v0[2], config.decimalPlaces) << "\n";
            file << "      vertex " << formatCoordinate(v1[0], config.decimalPlaces)
                 << " " << formatCoordinate(v1[1], config.decimalPlaces)
                 << " " << formatCoordinate(v1[2], config.decimalPlaces) << "\n";
            file << "      vertex " << formatCoordinate(v2[0], config.decimalPlaces)
                 << " " << formatCoordinate(v2[1], config.decimalPlaces)
                 << " " << formatCoordinate(v2[2], config.decimalPlaces) << "\n";
            file << "    endloop\n";
            file << "  endfacet\n";
        }
    }

    file << "endsolid " << config.stlHeader << "\n";
    file.close();
    return true;
}

bool IndustrialMeshExporter::writeSTLBinary(const std::vector<cv::Vec3f>& vertices,
                                           const std::vector<cv::Vec3i>& faces,
                                           const std::string& filePath,
                                           const IndustrialExportConfig& config) {
    std::ofstream file(filePath, std::ios::binary);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open STL file for writing: " + filePath;
        return false;
    }

    // Count valid faces
    uint32_t numTriangles = 0;
    for (const auto& face : faces) {
        if (face[0] >= 0 && face[0] < vertices.size() &&
            face[1] >= 0 && face[1] < vertices.size() &&
            face[2] >= 0 && face[2] < vertices.size()) {
            numTriangles++;
        }
    }

    // Write header (80 bytes)
    std::string header = config.stlHeader;
    header.resize(80, '\0');
    file.write(header.c_str(), 80);

    // Write number of triangles
    file.write(reinterpret_cast<const char*>(&numTriangles), sizeof(uint32_t));

    // Write triangles
    for (const auto& face : faces) {
        if (face[0] >= 0 && face[0] < vertices.size() &&
            face[1] >= 0 && face[1] < vertices.size() &&
            face[2] >= 0 && face[2] < vertices.size()) {

            const cv::Vec3f& v0 = vertices[face[0]];
            const cv::Vec3f& v1 = vertices[face[1]];
            const cv::Vec3f& v2 = vertices[face[2]];

            // Compute and write face normal
            cv::Vec3f normal = computeFaceNormal(v0, v1, v2);
            float normalFloat[3] = {
                static_cast<float>(normal[0]),
                static_cast<float>(normal[1]),
                static_cast<float>(normal[2])
            };
            file.write(reinterpret_cast<const char*>(normalFloat), 3 * sizeof(float));

            // Write vertices
            float vertices_data[9] = {
                static_cast<float>(v0[0]), static_cast<float>(v0[1]), static_cast<float>(v0[2]),
                static_cast<float>(v1[0]), static_cast<float>(v1[1]), static_cast<float>(v1[2]),
                static_cast<float>(v2[0]), static_cast<float>(v2[1]), static_cast<float>(v2[2])
            };
            file.write(reinterpret_cast<const char*>(vertices_data), 9 * sizeof(float));

            // Write attribute byte count (usually 0)
            uint16_t attributeByteCount = 0;
            file.write(reinterpret_cast<const char*>(&attributeByteCount), sizeof(uint16_t));
        }
    }

    file.close();
    return true;
}

bool IndustrialMeshExporter::writeMTLFile(const std::string& mtlPath,
                                         const IndustrialExportConfig& config) {
    std::ofstream file(mtlPath);
    if (!file.is_open()) {
        pImpl->lastError = "Failed to open MTL file for writing: " + mtlPath;
        return false;
    }

    file << "# MTL file generated by " << config.manufacturer << "\n";
    file << "# Material: " << config.material << "\n\n";

    file << "newmtl " << config.materialName << "\n";

    // Get material properties
    auto materialProps = industrial_export::getMaterialProperties(config.material);

    file << "Ka 0.2 0.2 0.2\n";  // Ambient color
    file << "Kd 0.8 0.8 0.8\n";  // Diffuse color
    file << "Ks 0.1 0.1 0.1\n";  // Specular color
    file << "Ns 10.0\n";         // Specular exponent
    file << "d 1.0\n";           // Transparency (1.0 = opaque)
    file << "illum 2\n";         // Illumination model

    file.close();
    return true;
}

void IndustrialMeshExporter::scaleVertices(std::vector<cv::Vec3f>& vertices, double scaleFactor) {
    for (auto& vertex : vertices) {
        vertex[0] *= scaleFactor;
        vertex[1] *= scaleFactor;
        vertex[2] *= scaleFactor;
    }
}

cv::Vec3f IndustrialMeshExporter::computeFaceNormal(const cv::Vec3f& v0,
                                                   const cv::Vec3f& v1,
                                                   const cv::Vec3f& v2) {
    cv::Vec3f edge1 = v1 - v0;
    cv::Vec3f edge2 = v2 - v0;
    cv::Vec3f normal = edge1.cross(edge2);

    double length = cv::norm(normal);
    if (length > 1e-6) {
        normal /= length;
    }

    return normal;
}

std::string IndustrialMeshExporter::formatCoordinate(double value, int precision) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

std::string IndustrialMeshExporter::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

size_t IndustrialMeshExporter::calculateFileSize(const std::string& filePath) {
    try {
        return std::filesystem::file_size(filePath);
    } catch (const std::exception&) {
        return 0;
    }
}

// Configuration validation implementations
bool IndustrialExportConfig::validate() const {
    if (coordinatePrecision <= 0.0) return false;
    if (decimalPlaces < 1 || decimalPlaces > 15) return false;
    if (tolerance <= 0.0) return false;
    if (minTriangleQuality < 0.0 || minTriangleQuality > 1.0) return false;
    return true;
}

std::string IndustrialExportConfig::toString() const {
    std::stringstream ss;
    ss << "Industrial Export Configuration:\n";
    ss << "  Format: ";
    switch (format) {
        case Format::STL_ASCII: ss << "STL ASCII"; break;
        case Format::STL_BINARY: ss << "STL Binary"; break;
        case Format::OBJ_WITH_MATERIALS: ss << "OBJ with Materials"; break;
        case Format::PLY_INDUSTRIAL: ss << "PLY Industrial"; break;
        case Format::X3D_MANUFACTURING: ss << "X3D Manufacturing"; break;
        case Format::GLTF_PRECISION: ss << "glTF Precision"; break;
    }
    ss << "\n  Precision: " << coordinatePrecision << " mm";
    ss << "\n  Units: ";
    switch (units) {
        case Units::MILLIMETERS: ss << "Millimeters"; break;
        case Units::METERS: ss << "Meters"; break;
        case Units::INCHES: ss << "Inches"; break;
    }
    ss << "\n  Material: " << material;
    return ss.str();
}

std::string IndustrialExportConfig::getFileExtension() const {
    switch (format) {
        case Format::STL_ASCII:
        case Format::STL_BINARY: return ".stl";
        case Format::OBJ_WITH_MATERIALS: return ".obj";
        case Format::PLY_INDUSTRIAL: return ".ply";
        case Format::X3D_MANUFACTURING: return ".x3d";
        case Format::GLTF_PRECISION: return ".gltf";
        default: return ".dat";
    }
}

double IndustrialExportConfig::getScaleFactor() const {
    switch (units) {
        case Units::MILLIMETERS: return 1.0;
        case Units::METERS: return 0.001;
        case Units::INCHES: return 1.0 / 25.4;
        default: return 1.0;
    }
}

std::string ExportResult::toString() const {
    std::stringstream ss;
    ss << "Export Result:\n";
    ss << "  Success: " << (success ? "Yes" : "No") << "\n";
    ss << "  Format: " << format << "\n";
    ss << "  File: " << filePath << "\n";
    ss << "  File size: " << fileSizeBytes << " bytes\n";
    ss << "  Export time: " << exportTimeMs << " ms\n";
    ss << "  Vertices exported: " << verticesExported << "\n";
    ss << "  Faces exported: " << facesExported << "\n";
    ss << "  Manufacturing ready: " << (manufacturingReady ? "Yes" : "No") << "\n";
    if (!errorMessage.empty()) {
        ss << "  Error: " << errorMessage << "\n";
    }
    return ss.str();
}

// Namespace industrial_export implementations
namespace industrial_export {

double convertUnits(std::vector<cv::Vec3f>& vertices,
                   IndustrialExportConfig::Units fromUnits,
                   IndustrialExportConfig::Units toUnits) {
    if (fromUnits == toUnits) return 1.0;

    double factor = 1.0;

    // Convert to millimeters first
    switch (fromUnits) {
        case IndustrialExportConfig::Units::METERS:
            factor = 1000.0;
            break;
        case IndustrialExportConfig::Units::INCHES:
            factor = 25.4;
            break;
        default:
            factor = 1.0;
            break;
    }

    // Convert from millimeters to target
    switch (toUnits) {
        case IndustrialExportConfig::Units::METERS:
            factor /= 1000.0;
            break;
        case IndustrialExportConfig::Units::INCHES:
            factor /= 25.4;
            break;
        default:
            break;
    }

    // Apply conversion
    for (auto& vertex : vertices) {
        vertex[0] *= factor;
        vertex[1] *= factor;
        vertex[2] *= factor;
    }

    return factor;
}

std::map<std::string, std::string> getMaterialProperties(const std::string& materialName) {
    std::map<std::string, std::string> props;

    if (materialName == "PLA") {
        props["density"] = "1.24";
        props["melting_temp"] = "180-220";
        props["bed_temp"] = "60";
        props["color"] = "white";
    } else if (materialName == "ABS") {
        props["density"] = "1.04";
        props["melting_temp"] = "220-250";
        props["bed_temp"] = "80-100";
        props["color"] = "white";
    } else if (materialName == "PETG") {
        props["density"] = "1.27";
        props["melting_temp"] = "220-250";
        props["bed_temp"] = "70-80";
        props["color"] = "clear";
    } else {
        props["density"] = "1.0";
        props["color"] = "white";
    }

    return props;
}

bool validateFilePath(const std::string& filePath, const std::string& extension) {
    if (filePath.empty()) return false;

    // Check extension
    if (!filePath.ends_with(extension)) {
        return false;
    }

    // Create directory if it doesn't exist
    try {
        std::filesystem::path path(filePath);
        std::filesystem::create_directories(path.parent_path());
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

void computeMeshBounds(const std::vector<cv::Vec3f>& vertices,
                      cv::Vec3f& minBounds,
                      cv::Vec3f& maxBounds,
                      cv::Vec3f& dimensions) {
    if (vertices.empty()) {
        minBounds = maxBounds = dimensions = cv::Vec3f(0, 0, 0);
        return;
    }

    minBounds = cv::Vec3f(FLT_MAX, FLT_MAX, FLT_MAX);
    maxBounds = cv::Vec3f(-FLT_MAX, -FLT_MAX, -FLT_MAX);

    for (const auto& vertex : vertices) {
        minBounds[0] = std::min(minBounds[0], vertex[0]);
        minBounds[1] = std::min(minBounds[1], vertex[1]);
        minBounds[2] = std::min(minBounds[2], vertex[2]);

        maxBounds[0] = std::max(maxBounds[0], vertex[0]);
        maxBounds[1] = std::max(maxBounds[1], vertex[1]);
        maxBounds[2] = std::max(maxBounds[2], vertex[2]);
    }

    dimensions[0] = maxBounds[0] - minBounds[0];
    dimensions[1] = maxBounds[1] - minBounds[1];
    dimensions[2] = maxBounds[2] - minBounds[2];
}

bool checkManufacturingConstraints(const std::vector<cv::Vec3f>& vertices,
                                  const std::vector<cv::Vec3i>& faces,
                                  const cv::Vec3f& maxDimensions,
                                  double maxVolume) {
    cv::Vec3f minBounds, maxBounds, dimensions;
    computeMeshBounds(vertices, minBounds, maxBounds, dimensions);

    // Check dimensions
    if (dimensions[0] > maxDimensions[0] ||
        dimensions[1] > maxDimensions[1] ||
        dimensions[2] > maxDimensions[2]) {
        return false;
    }

    // Check volume if specified
    if (maxVolume > 0.0) {
        double volume = 0.0;
        for (const auto& face : faces) {
            if (face[0] < vertices.size() && face[1] < vertices.size() && face[2] < vertices.size()) {
                const cv::Vec3f& v0 = vertices[face[0]];
                const cv::Vec3f& v1 = vertices[face[1]];
                const cv::Vec3f& v2 = vertices[face[2]];

                volume += (v0[0] * (v1[1] * v2[2] - v1[2] * v2[1]) +
                          v0[1] * (v1[2] * v2[0] - v1[0] * v2[2]) +
                          v0[2] * (v1[0] * v2[1] - v1[1] * v2[0])) / 6.0;
            }
        }
        volume = std::abs(volume);

        if (volume > maxVolume) {
            return false;
        }
    }

    return true;
}

} // namespace industrial_export

} // namespace mesh
} // namespace unlook