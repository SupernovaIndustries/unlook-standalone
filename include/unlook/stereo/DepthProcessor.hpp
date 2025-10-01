#pragma once

#include "unlook/calibration/CalibrationManager.hpp"
#include "unlook/stereo/StereoMatcher.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <memory>
#include <functional>
#include <thread>
#include <atomic>

namespace unlook {
namespace stereo {

/**
 * @brief Depth processing configuration
 */
struct DepthProcessingConfig {
    // INDUSTRIAL STANDARDS COMPLIANT: Depth range for 70mm baseline stereo system
    // Based on IEEE/ISO stereo vision standards and actual measurement capabilities
    float minDepthMm = 200.0f;      // Minimum: 3x baseline (210mm) with safety margin
    float maxDepthMm = 3500.0f;     // Maximum: Extended range for industrial scanning
    
    // Processing options
    bool computePointCloud = true;      // Generate 3D point cloud (ENABLED for investor demo)
    bool computeNormals = false;        // Compute surface normals
    bool applyMedianFilter = true;      // Apply median filter to depth
    int medianKernelSize = 5;          // Median filter kernel size
    
    // Bilateral filter parameters
    bool applyBilateralFilter = true;   // Apply bilateral filter
    int bilateralD = 9;                // Diameter of pixel neighborhood
    double bilateralSigmaColor = 75.0; // Filter sigma in color space
    double bilateralSigmaSpace = 75.0; // Filter sigma in coordinate space
    
    // Hole filling
    bool fillHoles = true;              // Fill holes in depth map
    int maxHoleSize = 100;             // Maximum hole size to fill
    
    // Temporal filtering (for sequences)
    bool enableTemporalFilter = false;  // Enable temporal smoothing
    float temporalAlpha = 0.4f;        // Temporal filter weight
    
    // Point cloud parameters
    float pointCloudScale = 1.0f;      // Scale factor for point cloud
    bool colorizePointCloud = true;    // Add color to point cloud
    
    // Validation
    bool validateDepth = true;         // Validate depth values
    float maxDepthChange = 10.0f;      // Maximum depth change between neighbors
    
    bool validate() const;
    std::string toString() const;
};

/**
 * @brief Depth map statistics
 */
struct DepthStatistics {
    float minDepth;           // Minimum depth value
    float maxDepth;           // Maximum depth value
    float meanDepth;          // Mean depth value
    float stdDepth;           // Standard deviation of depth
    int validPixels;          // Number of valid depth pixels
    int totalPixels;          // Total number of pixels
    float validRatio;         // Ratio of valid pixels
    float processingTimeMs;   // Processing time in milliseconds
    
    std::string toString() const;
};

/**
 * @brief 3D point with color
 */
struct Point3D {
    float x, y, z;        // 3D coordinates in mm
    uint8_t r, g, b;      // RGB color
    float confidence;     // Point confidence (0-1)
    
    Point3D() : x(0), y(0), z(0), r(0), g(0), b(0), confidence(0) {}
    Point3D(float x_, float y_, float z_) : x(x_), y(y_), z(z_), r(255), g(255), b(255), confidence(1.0f) {}
};

/**
 * @brief Point cloud data structure
 */
struct PointCloud {
    std::vector<Point3D> points;
    int width;              // Organized point cloud width
    int height;             // Organized point cloud height
    bool isOrganized;       // True if point cloud is organized
    
    void clear() {
        points.clear();
        width = height = 0;
        isOrganized = false;
    }
    
    size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }
};

/**
 * @brief High-precision depth processor for stereo vision
 * 
 * Processes stereo images to generate depth maps and point clouds
 * with industrial-grade precision (0.005mm repeatability target).
 */
class DepthProcessor {
public:
    DepthProcessor();
    ~DepthProcessor();
    
    /**
     * @brief Initialize with calibration manager
     * @param calibrationManager Shared pointer to calibration manager
     * @return true if initialization successful
     */
    bool initialize(std::shared_ptr<calibration::CalibrationManager> calibrationManager);
    
    /**
     * @brief Set stereo matcher algorithm
     * @param algorithm Algorithm type to use
     * @return true if matcher set successfully
     */
    bool setStereoMatcher(StereoAlgorithm algorithm);
    
    /**
     * @brief Get current stereo matcher
     * @return Pointer to current stereo matcher
     */
    StereoMatcher* getStereoMatcher() const;
    
    /**
     * @brief Process stereo pair to generate depth map
     * @param leftImage Left camera image
     * @param rightImage Right camera image
     * @param depthMap Output depth map in millimeters (CV_32F)
     * @return true if processing successful
     */
    bool processStereoPair(const cv::Mat& leftImage,
                          const cv::Mat& rightImage,
                          cv::Mat& depthMap);
    
    /**
     * @brief Process with confidence map
     * @param leftImage Left camera image
     * @param rightImage Right camera image
     * @param depthMap Output depth map in millimeters
     * @param confidenceMap Output confidence map (0-1)
     * @return true if processing successful
     */
    bool processWithConfidence(const cv::Mat& leftImage,
                              const cv::Mat& rightImage,
                              cv::Mat& depthMap,
                              cv::Mat& confidenceMap,
                              cv::Mat* disparityMap = nullptr);
    
    /**
     * @brief Generate point cloud from depth map
     * @param depthMap Input depth map
     * @param colorImage Optional color image for colorization
     * @param pointCloud Output point cloud
     * @return true if generation successful
     */
    bool generatePointCloud(const cv::Mat& depthMap,
                          const cv::Mat& colorImage,
                          PointCloud& pointCloud);
    
    /**
     * @brief Compute depth statistics
     * @param depthMap Input depth map
     * @param stats Output statistics
     * @return true if computation successful
     */
    bool computeDepthStatistics(const cv::Mat& depthMap,
                               DepthStatistics& stats) const;
    
    /**
     * @brief Apply depth filtering
     * @param depthMap Input/output depth map
     * @return true if filtering successful
     */
    bool applyDepthFiltering(cv::Mat& depthMap);
    
    /**
     * @brief Set processing configuration
     * @param config Processing configuration
     * @return true if configuration valid and set
     */
    bool setConfiguration(const DepthProcessingConfig& config);
    
    /**
     * @brief Get current configuration
     * @return Current processing configuration
     */
    DepthProcessingConfig getConfiguration() const;
    
    /**
     * @brief Set stereo matching parameters
     * @param params Stereo matching parameters
     * @return true if parameters set successfully
     */
    bool setStereoParameters(const StereoMatchingParams& params);
    
    /**
     * @brief Get stereo matching parameters
     * @return Current stereo matching parameters
     */
    StereoMatchingParams getStereoParameters() const;
    
    /**
     * @brief Validate depth map quality
     * @param depthMap Depth map to validate
     * @param validMask Output mask of valid pixels
     * @return Quality score (0-1)
     */
    double validateDepthMap(const cv::Mat& depthMap,
                           cv::Mat& validMask) const;
    
    /**
     * @brief Export depth map to file
     * @param depthMap Depth map to export
     * @param filename Output filename
     * @param format Format (e.g., "pfm", "exr", "png16")
     * @return true if export successful
     */
    bool exportDepthMap(const cv::Mat& depthMap,
                       const std::string& filename,
                       const std::string& format = "pfm") const;
    
    /**
     * @brief Export point cloud to file
     * @param pointCloud Point cloud to export
     * @param filename Output filename
     * @param format Format (e.g., "ply", "pcd", "xyz")
     * @return true if export successful
     */
    bool exportPointCloud(const PointCloud& pointCloud,
                         const std::string& filename,
                         const std::string& format = "ply") const;
    
    /**
     * @brief Set progress callback
     * @param callback Function called with progress (0-100)
     */
    void setProgressCallback(std::function<void(int)> callback);
    
    /**
     * @brief Cancel ongoing processing
     */
    void cancelProcessing();
    
    /**
     * @brief Check if processing is in progress
     * @return true if currently processing
     */
    bool isProcessing() const;
    
    /**
     * @brief Get last error message
     * @return Error message string
     */
    std::string getLastError() const;

    /**
     * @brief Get last generated point cloud from direct disparity conversion
     *
     * INVESTOR DEMO FIX: Retrieves the point cloud generated by
     * generatePointCloudFromDisparity() during the last processWithConfidence() call.
     * This contains ~1M points directly converted from disparity without lossy depth map conversion.
     *
     * @return Const reference to last generated point cloud (may be empty if not yet generated)
     */
    const PointCloud& getLastGeneratedPointCloud() const;

    /**
     * @brief Get path to last exported point cloud file
     *
     * MEMORY-SAFE FIX: Returns file path instead of copying 1M points.
     * GUI loads from file to avoid memory copy failures.
     *
     * @return Path to PLY file with point cloud (empty if not generated)
     */
    std::string getLastPointCloudPath() const;

    /**
     * @brief Compute depth precision at given distance
     * @param depthMm Depth in millimeters
     * @return Expected precision in millimeters
     */
    double computeDepthPrecision(double depthMm) const;
    
    /**
     * @brief Visualize depth map
     * @param depthMap Input depth map
     * @param colorized Output colorized visualization
     * @param colormap OpenCV colormap type
     */
    static void visualizeDepthMap(const cv::Mat& depthMap,
                                 cv::Mat& colorized,
                                 int colormap = cv::COLORMAP_JET);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl;

    /**
     * @brief Update depth range parameters based on calibration data
     *
     * Calculates optimal depth range using industrial stereo vision standards:
     * - Minimum depth: 3x baseline (IEEE/ISO standard) + safety margin
     * - Maximum depth: Based on disparity precision limits
     *
     * This ensures industrial standards compliance and prevents data loss
     * due to arbitrary range limitations.
     */
    void updateDepthRangeFromCalibration();

    /**
     * @brief Create LIDAR-like continuous depth map by filling holes and smoothing
     * @param inputDepth Input depth map with potential holes
     * @param outputDepth Output continuous depth map
     */
    void createLidarLikeDepthMap(const cv::Mat& inputDepth, cv::Mat& outputDepth) const;

#ifdef OPEN3D_ENABLED
    /**
     * @brief Professional Open3D-based point cloud generation
     * @param depthMap Input depth map
     * @param colorImage Optional color image
     * @param pointCloud Output point cloud
     * @param fx Focal length X
     * @param fy Focal length Y
     * @param cx Principal point X
     * @param cy Principal point Y
     * @return true if generation successful
     */
    bool generatePointCloudOpen3D(const cv::Mat& depthMap,
                                  const cv::Mat& colorImage,
                                  PointCloud& pointCloud,
                                  double fx, double fy, double cx, double cy);
#endif

    /**
     * @brief Professional pinhole camera model point cloud generation (fallback)
     * @param depthMap Input depth map
     * @param colorImage Optional color image
     * @param pointCloud Output point cloud
     * @param fx Focal length X
     * @param fy Focal length Y
     * @param cx Principal point X
     * @param cy Principal point Y
     * @return true if generation successful
     */
    bool generatePointCloudPinhole(const cv::Mat& depthMap,
                                  const cv::Mat& colorImage,
                                  PointCloud& pointCloud,
                                  double fx, double fy, double cx, double cy);

    /**
     * @brief Direct disparity-to-3D point cloud conversion (bypasses depth map)
     *
     * This method directly converts disparity values to 3D points without the
     * intermediate depth map conversion, preserving maximum precision and avoiding
     * data loss from filtering stages.
     *
     * @param disparity Input disparity map (CV_16S or CV_32F from SGBM)
     * @param leftRectified Rectified left image for color extraction
     * @param pointCloud Output point cloud with 3D points and colors
     * @param calibData Calibration data containing intrinsics and baseline
     * @return true if conversion successful
     */
    bool generatePointCloudFromDisparity(const cv::Mat& disparity,
                                         const cv::Mat& leftRectified,
                                         PointCloud& pointCloud,
                                         const calibration::StereoCalibrationData& calibData);

    // Disable copy
    DepthProcessor(const DepthProcessor&) = delete;
    DepthProcessor& operator=(const DepthProcessor&) = delete;
};

/**
 * @brief Depth map file I/O utilities
 */
class DepthIO {
public:
    /**
     * @brief Save depth map in PFM format
     * @param filename Output filename
     * @param depthMap Depth map to save
     * @return true if saved successfully
     */
    static bool savePFM(const std::string& filename, const cv::Mat& depthMap);
    
    /**
     * @brief Load depth map from PFM format
     * @param filename Input filename
     * @param depthMap Output depth map
     * @return true if loaded successfully
     */
    static bool loadPFM(const std::string& filename, cv::Mat& depthMap);
    
    /**
     * @brief Save depth map in OpenEXR format
     * @param filename Output filename
     * @param depthMap Depth map to save
     * @return true if saved successfully
     */
    static bool saveEXR(const std::string& filename, const cv::Mat& depthMap);
    
    /**
     * @brief Save depth map as 16-bit PNG
     * @param filename Output filename
     * @param depthMap Depth map to save
     * @param scale Scale factor for conversion
     * @return true if saved successfully
     */
    static bool savePNG16(const std::string& filename, 
                         const cv::Mat& depthMap,
                         float scale = 1.0f);
};

} // namespace stereo
} // namespace unlook