#include "unlook/stereo/DepthProcessor.hpp"
#include "unlook/stereo/StereoMatcher.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

namespace unlook {
namespace stereo {

// DepthProcessor implementation with full functionality
class DepthProcessor::Impl {
public:
    std::shared_ptr<calibration::CalibrationManager> calibManager;
    std::unique_ptr<StereoMatcher> stereoMatcher;
    DepthProcessingConfig config;
    std::string lastError;
    std::atomic<bool> processing{false};
    std::atomic<bool> cancelRequested{false};
    std::function<void(int)> progressCallback;
    mutable std::mutex processingMutex;
    
    // WLS filter for post-processing
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsFilter;
    cv::Ptr<cv::StereoMatcher> rightMatcher;
    
    // Temporal filtering state
    cv::Mat previousDepth;
    
    Impl() {
        // OPTIMIZED CONFIGURATION FOR 70MM BASELINE
        // Target: 0.005mm precision at 100-800mm range
        config.minDepthMm = 100.0f;      // Minimum depth for 70mm baseline
        config.maxDepthMm = 800.0f;      // Maximum practical depth

        // Filtering parameters - less aggressive to preserve precision
        config.medianKernelSize = 3;     // Smaller kernel (was 5)
        config.applyMedianFilter = true; // Only for outliers
        config.applyBilateralFilter = true;
        config.bilateralD = 5;           // Smaller window (was 9)
        config.bilateralSigmaColor = 50.0; // Reduced (was 75)
        config.bilateralSigmaSpace = 50.0; // Reduced (was 75)

        // Hole filling - more conservative
        config.fillHoles = true;
        config.maxHoleSize = 10;         // Much smaller (was 100)

        // Validation parameters
        config.validateDepth = true;
        config.maxDepthChange = 20.0f;   // Allow larger changes (was 10)

        // Temporal filtering disabled by default
        config.enableTemporalFilter = false;
        config.temporalAlpha = 0.2f;     // Lower weight if enabled
    }
};

DepthProcessor::DepthProcessor() : pImpl(std::make_unique<Impl>()) {}
DepthProcessor::~DepthProcessor() = default;

bool DepthProcessor::initialize(std::shared_ptr<calibration::CalibrationManager> calibrationManager) {
    pImpl->calibManager = calibrationManager;
    return calibrationManager && calibrationManager->isCalibrationValid();
}

bool DepthProcessor::setStereoMatcher(StereoAlgorithm algorithm) {
    pImpl->stereoMatcher = StereoMatcher::create(algorithm);
    return pImpl->stereoMatcher != nullptr;
}

StereoMatcher* DepthProcessor::getStereoMatcher() const {
    return pImpl->stereoMatcher.get();
}

bool DepthProcessor::processStereoPair(const cv::Mat& leftImage,
                                      const cv::Mat& rightImage,
                                      cv::Mat& depthMap) {
    if (!pImpl->calibManager || !pImpl->stereoMatcher) {
        pImpl->lastError = "Not initialized";
        return false;
    }
    
    pImpl->processing = true;
    
    // Rectify images
    cv::Mat leftRect, rightRect;
    if (!pImpl->calibManager->rectifyImages(leftImage, rightImage, leftRect, rightRect)) {
        pImpl->lastError = "Rectification failed";
        pImpl->processing = false;
        return false;
    }
    
    // Compute disparity
    cv::Mat disparity;
    if (!pImpl->stereoMatcher->computeDisparity(leftRect, rightRect, disparity)) {
        pImpl->lastError = "Disparity computation failed";
        pImpl->processing = false;
        return false;
    }
    
    // Convert to depth
    auto& calibData = pImpl->calibManager->getCalibrationData();
    if (!StereoMatcher::disparityToDepth(disparity, calibData.Q, depthMap, true)) {
        pImpl->lastError = "Depth conversion failed";
        pImpl->processing = false;
        return false;
    }
    
    // Apply filtering if configured
    if (pImpl->config.applyMedianFilter || pImpl->config.applyBilateralFilter) {
        applyDepthFiltering(depthMap);
    }
    
    pImpl->processing = false;
    return true;
}

bool DepthProcessor::processWithConfidence(const cv::Mat& leftImage,
                                          const cv::Mat& rightImage,
                                          cv::Mat& depthMap,
                                          cv::Mat& confidenceMap,
                                          cv::Mat* disparityMap) {
    if (!pImpl->calibManager || !pImpl->stereoMatcher) {
        if (!pImpl->calibManager) {
            std::cout << "[Core DepthProcessor] ERROR: calibManager is null!" << std::endl;
        }
        if (!pImpl->stereoMatcher) {
            std::cout << "[Core DepthProcessor] ERROR: stereoMatcher is null!" << std::endl;
        }
        pImpl->lastError = "Not initialized";
        return false;
    }
    
    pImpl->processing = true;
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Rectify images
    cv::Mat leftRect, rightRect;
    if (!pImpl->calibManager->rectifyImages(leftImage, rightImage, leftRect, rightRect)) {
        pImpl->lastError = "Rectification failed";
        pImpl->processing = false;
        return false;
    }
    
    // Compute disparity with confidence
    cv::Mat disparity;
    if (!pImpl->stereoMatcher->computeDisparity(leftRect, rightRect, disparity)) {
        pImpl->lastError = "Disparity computation failed";
        pImpl->processing = false;
        return false;
    }
    
    // COMPREHENSIVE DEBUG: Save disparity at each step
    std::cout << "[Core DepthProcessor] RAW Disparity computed - Size: " << disparity.cols << "x" << disparity.rows 
              << ", Type: " << disparity.type() << ", Channels: " << disparity.channels() << std::endl;
    
    // Analyze raw disparity values
    double minDisp, maxDisp;
    cv::minMaxLoc(disparity, &minDisp, &maxDisp);
    cv::Scalar meanDisp = cv::mean(disparity, disparity > 0);  // Only valid pixels
    int validPixels = cv::countNonZero(disparity);
    int totalPixels = disparity.rows * disparity.cols;
    double validRatio = double(validPixels) / totalPixels * 100.0;
    
    std::cout << "[Core DepthProcessor] RAW Disparity stats: Min=" << minDisp << ", Max=" << maxDisp 
              << ", Mean=" << meanDisp[0] << ", Valid=" << validPixels << "/" << totalPixels 
              << " (" << validRatio << "%)" << std::endl;
    
    // Convert to depth
    auto& calibData = pImpl->calibManager->getCalibrationData();
    if (!StereoMatcher::disparityToDepth(disparity, calibData.Q, depthMap, true)) {
        pImpl->lastError = "Depth conversion failed";
        pImpl->processing = false;
        return false;
    }
    
    // OPTIMIZED POST-PROCESSING: Preserve depth precision while handling invalid pixels
    if (pImpl->config.fillHoles) {
        cv::Mat depthOptimized;
        createLidarLikeDepthMap(depthMap, depthOptimized);
        depthOptimized.copyTo(depthMap);
        std::cout << "[Core DepthProcessor] Precision-preserving post-processing applied" << std::endl;
    } else {
        std::cout << "[Core DepthProcessor] Post-processing skipped (fillHoles=false)" << std::endl;
    }
    
    // CRITICAL FIX: Copy disparity map to output parameter if requested
    if (disparityMap != nullptr) {
        disparity.copyTo(*disparityMap);
        std::cout << "[Core DepthProcessor] Disparity map copied to output (size: " 
                  << disparityMap->cols << "x" << disparityMap->rows << ")" << std::endl;
    }
    
    // Generate confidence map based on disparity validity and consistency
    confidenceMap = cv::Mat::zeros(disparity.size(), CV_32F);
    
    // Calculate confidence based on:
    // 1. Valid disparity values (non-zero)
    // 2. Local consistency (low variance in neighborhood)
    // 3. Depth range validity
    
    for (int y = 1; y < disparity.rows - 1; ++y) {
        for (int x = 1; x < disparity.cols - 1; ++x) {
            float d = disparity.at<float>(y, x);
            float depth = depthMap.at<float>(y, x);
            
            if (d > 0 && depth > pImpl->config.minDepthMm && depth < pImpl->config.maxDepthMm) {
                // Calculate local variance
                float variance = 0;
                int count = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        float neighbor = disparity.at<float>(y + dy, x + dx);
                        if (neighbor > 0) {
                            variance += std::abs(neighbor - d);
                            count++;
                        }
                    }
                }
                
                if (count > 0) {
                    variance /= count;
                    // Convert variance to confidence (lower variance = higher confidence)
                    float confidence = std::exp(-variance / 10.0f);
                    
                    // Apply depth-based confidence weighting
                    // Closer objects have higher confidence due to higher disparity precision
                    float depthWeight = 1.0f - (depth - pImpl->config.minDepthMm) / 
                                              (pImpl->config.maxDepthMm - pImpl->config.minDepthMm);
                    depthWeight = std::max(0.3f, depthWeight);  // Minimum 30% weight
                    
                    confidenceMap.at<float>(y, x) = confidence * depthWeight;
                } else {
                    confidenceMap.at<float>(y, x) = 0.0f;
                }
            }
        }
    }
    
    // Apply filtering if configured
    if (pImpl->config.applyMedianFilter || pImpl->config.applyBilateralFilter) {
        applyDepthFiltering(depthMap);
        
        // Also filter confidence map
        if (pImpl->config.applyMedianFilter) {
            cv::medianBlur(confidenceMap, confidenceMap, pImpl->config.medianKernelSize);
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    // Update processing stats if needed
    if (pImpl->progressCallback) {
        pImpl->progressCallback(100);
    }
    
    pImpl->processing = false;
    return true;
}

bool DepthProcessor::generatePointCloud(const cv::Mat& depthMap,
                                       const cv::Mat& colorImage,
                                       PointCloud& pointCloud) {
    if (depthMap.empty()) {
        pImpl->lastError = "Empty depth map";
        return false;
    }
    
    if (!pImpl->calibManager || !pImpl->calibManager->isCalibrationValid()) {
        pImpl->lastError = "No valid calibration available";
        return false;
    }
    
    try {
        pointCloud.clear();
        pointCloud.width = depthMap.cols;
        pointCloud.height = depthMap.rows;
        pointCloud.isOrganized = true;
        pointCloud.points.reserve(depthMap.rows * depthMap.cols);
        
        // Get camera intrinsics from calibration
        auto& calibData = pImpl->calibManager->getCalibrationData();
        cv::Mat Q = calibData.Q;  // Disparity-to-depth mapping matrix
        
        if (Q.empty()) {
            // If Q matrix not available, use simple pinhole model
            float fx = calibData.cameraMatrixLeft.at<double>(0, 0);
            float fy = calibData.cameraMatrixLeft.at<double>(1, 1);
            float cx = calibData.cameraMatrixLeft.at<double>(0, 2);
            float cy = calibData.cameraMatrixLeft.at<double>(1, 2);
            
            cv::Mat colorResized;
            if (!colorImage.empty() && colorImage.size() != depthMap.size()) {
                cv::resize(colorImage, colorResized, depthMap.size());
            } else {
                colorResized = colorImage;
            }
            
            for (int y = 0; y < depthMap.rows; ++y) {
                for (int x = 0; x < depthMap.cols; ++x) {
                    float depth = depthMap.at<float>(y, x);
                    
                    Point3D pt;
                    if (depth > pImpl->config.minDepthMm && depth < pImpl->config.maxDepthMm) {
                        // Back-project to 3D
                        pt.z = depth;
                        pt.x = (x - cx) * depth / fx;
                        pt.y = (y - cy) * depth / fy;
                        
                        // Add color if available
                        if (!colorResized.empty()) {
                            cv::Vec3b color = colorResized.at<cv::Vec3b>(y, x);
                            pt.b = color[0];
                            pt.g = color[1];
                            pt.r = color[2];
                        } else {
                            pt.r = pt.g = pt.b = 255;
                        }
                        
                        pt.confidence = 1.0f;  // Simple confidence for now
                    } else {
                        // Invalid point
                        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                        pt.r = pt.g = pt.b = 0;
                        pt.confidence = 0.0f;
                    }
                    
                    pointCloud.points.push_back(pt);
                }
                
                // Update progress if callback set
                if (pImpl->progressCallback && y % 10 == 0) {
                    int progress = (y * 100) / depthMap.rows;
                    pImpl->progressCallback(progress);
                }
                
                // Check for cancellation
                if (pImpl->cancelRequested) {
                    pointCloud.clear();
                    return false;
                }
            }
        } else {
            // Use Q matrix for reprojection
            cv::Mat points3D;
            cv::reprojectImageTo3D(depthMap, points3D, Q, true);
            
            cv::Mat colorResized;
            if (!colorImage.empty() && colorImage.size() != depthMap.size()) {
                cv::resize(colorImage, colorResized, depthMap.size());
            } else {
                colorResized = colorImage;
            }
            
            for (int y = 0; y < points3D.rows; ++y) {
                for (int x = 0; x < points3D.cols; ++x) {
                    cv::Vec3f point3d = points3D.at<cv::Vec3f>(y, x);
                    
                    Point3D pt;
                    pt.x = point3d[0] * pImpl->config.pointCloudScale;
                    pt.y = point3d[1] * pImpl->config.pointCloudScale;
                    pt.z = point3d[2] * pImpl->config.pointCloudScale;
                    
                    if (pt.z > pImpl->config.minDepthMm && pt.z < pImpl->config.maxDepthMm) {
                        // Add color if available
                        if (!colorResized.empty()) {
                            cv::Vec3b color = colorResized.at<cv::Vec3b>(y, x);
                            pt.b = color[0];
                            pt.g = color[1];
                            pt.r = color[2];
                        } else {
                            pt.r = pt.g = pt.b = 255;
                        }
                        pt.confidence = 1.0f;
                    } else {
                        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                        pt.r = pt.g = pt.b = 0;
                        pt.confidence = 0.0f;
                    }
                    
                    pointCloud.points.push_back(pt);
                }
            }
        }
        
        if (pImpl->progressCallback) {
            pImpl->progressCallback(100);
        }
        
        return true;
        
    } catch (const cv::Exception& e) {
        pImpl->lastError = "Point cloud generation failed: " + std::string(e.what());
        return false;
    }
}

bool DepthProcessor::computeDepthStatistics(const cv::Mat& depthMap,
                                           DepthStatistics& stats) const {
    if (depthMap.empty()) return false;
    
    cv::Mat validMask = depthMap > 0;
    cv::Scalar mean, stddev;
    cv::meanStdDev(depthMap, mean, stddev, validMask);
    
    double minVal, maxVal;
    cv::minMaxLoc(depthMap, &minVal, &maxVal, nullptr, nullptr, validMask);
    
    stats.minDepth = minVal;
    stats.maxDepth = maxVal;
    stats.meanDepth = mean[0];
    stats.stdDepth = stddev[0];
    stats.validPixels = cv::countNonZero(validMask);
    stats.totalPixels = depthMap.rows * depthMap.cols;
    stats.validRatio = (float)stats.validPixels / stats.totalPixels;
    
    return true;
}

bool DepthProcessor::applyDepthFiltering(cv::Mat& depthMap) {
    if (depthMap.empty()) return false;
    
    try {
        // Apply median filter for salt-and-pepper noise
        if (pImpl->config.applyMedianFilter && pImpl->config.medianKernelSize > 1) {
            cv::Mat temp;
            cv::medianBlur(depthMap, temp, pImpl->config.medianKernelSize);
            depthMap = temp;
        }
        
        // Apply bilateral filter for edge-preserving smoothing
        if (pImpl->config.applyBilateralFilter) {
            cv::Mat filtered;
            // Convert to 8-bit for bilateral filter if needed
            cv::Mat depthNorm;
            double minVal, maxVal;
            cv::minMaxLoc(depthMap, &minVal, &maxVal, nullptr, nullptr, depthMap > 0);
            
            if (maxVal > minVal) {
                depthMap.convertTo(depthNorm, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
                cv::bilateralFilter(depthNorm, filtered, 
                                  pImpl->config.bilateralD,
                                  pImpl->config.bilateralSigmaColor,
                                  pImpl->config.bilateralSigmaSpace);
                filtered.convertTo(depthMap, CV_32F, (maxVal - minVal) / 255.0, minVal);
            }
        }
        
        // Fill holes if requested
        if (pImpl->config.fillHoles && pImpl->config.maxHoleSize > 0) {
            // Simple hole filling using morphological operations
            cv::Mat mask = (depthMap == 0) | (depthMap != depthMap);  // Invalid pixels
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                       cv::Size(pImpl->config.maxHoleSize, pImpl->config.maxHoleSize));
            
            // Dilate valid regions to fill small holes
            cv::Mat validMask;
            cv::bitwise_not(mask, validMask);
            cv::Mat dilated;
            cv::dilate(depthMap, dilated, kernel);
            
            // Copy filled values only to hole regions
            dilated.copyTo(depthMap, mask);
        }
        
        // Temporal filtering if enabled and previous frame available
        if (pImpl->config.enableTemporalFilter && !pImpl->previousDepth.empty() && 
            pImpl->previousDepth.size() == depthMap.size()) {
            
            cv::Mat blended;
            cv::addWeighted(depthMap, 1.0f - pImpl->config.temporalAlpha,
                          pImpl->previousDepth, pImpl->config.temporalAlpha,
                          0, blended);
            
            // Only blend where both frames have valid depth
            cv::Mat validMask = (depthMap > 0) & (pImpl->previousDepth > 0);
            blended.copyTo(depthMap, validMask);
        }
        
        // Store current frame for next temporal filtering
        if (pImpl->config.enableTemporalFilter) {
            pImpl->previousDepth = depthMap.clone();
        }
        
        // Validate depth changes if requested
        if (pImpl->config.validateDepth) {
            // Remove outliers based on local depth changes
            cv::Mat gradX, gradY;
            cv::Sobel(depthMap, gradX, CV_32F, 1, 0, 3);
            cv::Sobel(depthMap, gradY, CV_32F, 0, 1, 3);
            cv::Mat gradient;
            cv::magnitude(gradX, gradY, gradient);
            
            // Mask out pixels with too large depth changes
            cv::Mat invalidMask = gradient > pImpl->config.maxDepthChange;
            depthMap.setTo(0, invalidMask);
        }
        
        return true;
        
    } catch (const cv::Exception& e) {
        pImpl->lastError = "Filtering failed: " + std::string(e.what());
        return false;
    }
}

bool DepthProcessor::setConfiguration(const DepthProcessingConfig& config) {
    if (!config.validate()) return false;
    pImpl->config = config;
    return true;
}

DepthProcessingConfig DepthProcessor::getConfiguration() const {
    return pImpl->config;
}

bool DepthProcessor::setStereoParameters(const StereoMatchingParams& params) {
    if (!pImpl->stereoMatcher) return false;
    return pImpl->stereoMatcher->setParameters(params);
}

StereoMatchingParams DepthProcessor::getStereoParameters() const {
    if (!pImpl->stereoMatcher) return StereoMatchingParams();
    return pImpl->stereoMatcher->getParameters();
}

double DepthProcessor::validateDepthMap(const cv::Mat& depthMap,
                                       cv::Mat& validMask) const {
    if (depthMap.empty()) return 0.0;
    
    validMask = (depthMap > pImpl->config.minDepthMm) & 
                (depthMap < pImpl->config.maxDepthMm);
    
    int validCount = cv::countNonZero(validMask);
    int totalCount = depthMap.rows * depthMap.cols;
    
    return (double)validCount / totalCount;
}

bool DepthProcessor::exportDepthMap(const cv::Mat& depthMap,
                                   const std::string& filename,
                                   const std::string& format) const {
    if (depthMap.empty()) {
        pImpl->lastError = "Empty depth map";
        return false;
    }
    
    try {
        if (format == "pfm" || format == "PFM") {
            return DepthIO::savePFM(filename, depthMap);
        } else if (format == "exr" || format == "EXR") {
            return DepthIO::saveEXR(filename, depthMap);
        } else if (format == "png16" || format == "PNG16") {
            return DepthIO::savePNG16(filename, depthMap, 1.0f);
        } else if (format == "png" || format == "PNG") {
            // Standard PNG save for visualization
            cv::Mat visualization;
            visualizeDepthMap(depthMap, visualization, cv::COLORMAP_JET);
            return cv::imwrite(filename, visualization);
        } else {
            pImpl->lastError = "Unsupported format: " + format;
            return false;
        }
    } catch (const std::exception& e) {
        pImpl->lastError = "Export failed: " + std::string(e.what());
        return false;
    }
}

bool DepthProcessor::exportPointCloud(const PointCloud& pointCloud,
                                     const std::string& filename,
                                     const std::string& format) const {
    if (pointCloud.empty()) {
        pImpl->lastError = "Empty point cloud";
        return false;
    }
    
    try {
        if (format == "ply" || format == "PLY") {
            std::ofstream file(filename);
            if (!file.is_open()) {
                pImpl->lastError = "Failed to open file: " + filename;
                return false;
            }
            
            // Write PLY header
            file << "ply\n";
            file << "format ascii 1.0\n";
            file << "element vertex " << pointCloud.points.size() << "\n";
            file << "property float x\n";
            file << "property float y\n";
            file << "property float z\n";
            file << "property uchar red\n";
            file << "property uchar green\n";
            file << "property uchar blue\n";
            file << "property float confidence\n";
            file << "end_header\n";
            
            // Write points
            for (const auto& point : pointCloud.points) {
                if (point.z > pImpl->config.minDepthMm && point.z < pImpl->config.maxDepthMm) {
                    file << point.x << " " << point.y << " " << point.z << " "
                         << (int)point.r << " " << (int)point.g << " " << (int)point.b << " "
                         << point.confidence << "\n";
                }
            }
            
            file.close();
            return true;
            
        } else if (format == "xyz" || format == "XYZ") {
            std::ofstream file(filename);
            if (!file.is_open()) {
                pImpl->lastError = "Failed to open file: " + filename;
                return false;
            }
            
            // Write simple XYZ format
            for (const auto& point : pointCloud.points) {
                if (point.z > pImpl->config.minDepthMm && point.z < pImpl->config.maxDepthMm) {
                    file << point.x << " " << point.y << " " << point.z << "\n";
                }
            }
            
            file.close();
            return true;
            
        } else if (format == "pcd" || format == "PCD") {
            // PCL PCD format
            std::ofstream file(filename);
            if (!file.is_open()) {
                pImpl->lastError = "Failed to open file: " + filename;
                return false;
            }
            
            // Write PCD header
            file << "# .PCD v0.7 - Point Cloud Data file format\n";
            file << "VERSION 0.7\n";
            file << "FIELDS x y z rgb\n";
            file << "SIZE 4 4 4 4\n";
            file << "TYPE F F F U\n";
            file << "COUNT 1 1 1 1\n";
            file << "WIDTH " << pointCloud.points.size() << "\n";
            file << "HEIGHT 1\n";
            file << "VIEWPOINT 0 0 0 1 0 0 0\n";
            file << "POINTS " << pointCloud.points.size() << "\n";
            file << "DATA ascii\n";
            
            // Write points with packed RGB
            for (const auto& point : pointCloud.points) {
                if (point.z > pImpl->config.minDepthMm && point.z < pImpl->config.maxDepthMm) {
                    uint32_t rgb = ((uint32_t)point.r << 16) | ((uint32_t)point.g << 8) | point.b;
                    file << point.x << " " << point.y << " " << point.z << " " << rgb << "\n";
                }
            }
            
            file.close();
            return true;
            
        } else {
            pImpl->lastError = "Unsupported format: " + format;
            return false;
        }
    } catch (const std::exception& e) {
        pImpl->lastError = "Export failed: " + std::string(e.what());
        return false;
    }
}

void DepthProcessor::setProgressCallback(std::function<void(int)> callback) {
    pImpl->progressCallback = callback;
}

void DepthProcessor::cancelProcessing() {
    pImpl->cancelRequested = true;
    pImpl->processing = false;
}

bool DepthProcessor::isProcessing() const {
    return pImpl->processing;
}

std::string DepthProcessor::getLastError() const {
    return pImpl->lastError;
}

double DepthProcessor::computeDepthPrecision(double depthMm) const {
    if (!pImpl->calibManager) return 0.0;
    
    double baseline = pImpl->calibManager->getBaselineMm();
    double focal = 1700;  // Approximate focal length in pixels
    double disparityError = 0.2;  // Sub-pixel error
    
    // Z = (baseline * focal) / disparity
    // dZ/dd = -(baseline * focal) / (disparity^2)
    // Error propagation: deltaZ = |dZ/dd| * deltaDisparity
    
    double disparity = (baseline * focal) / depthMm;
    double precision = (depthMm * depthMm * disparityError) / (baseline * focal);
    
    return precision;
}

void DepthProcessor::visualizeDepthMap(const cv::Mat& depthMap,
                                      cv::Mat& colorized,
                                      int colormap) {
    if (depthMap.empty()) return;

    // OPTIMIZED VISUALIZATION: Avoid quantization artifacts
    // Use robust statistics to handle outliers without creating artificial levels

    // Step 1: Calculate robust depth range (exclude outliers)
    cv::Mat validMask = (depthMap > 0) & (depthMap < 10000);  // Reasonable depth range
    if (cv::countNonZero(validMask) < 100) {
        // Not enough valid pixels
        colorized = cv::Mat::zeros(depthMap.size(), CV_8UC3);
        return;
    }

    // Calculate percentiles for robust range
    std::vector<float> validDepths;
    validDepths.reserve(depthMap.rows * depthMap.cols);

    for (int y = 0; y < depthMap.rows; ++y) {
        for (int x = 0; x < depthMap.cols; ++x) {
            float d = depthMap.at<float>(y, x);
            if (d > 0 && d < 10000) {
                validDepths.push_back(d);
            }
        }
    }

    if (validDepths.empty()) {
        colorized = cv::Mat::zeros(depthMap.size(), CV_8UC3);
        return;
    }

    // Use 2nd and 98th percentiles for robust range
    std::sort(validDepths.begin(), validDepths.end());
    float minVal = validDepths[validDepths.size() * 0.02];  // 2nd percentile
    float maxVal = validDepths[validDepths.size() * 0.98];  // 98th percentile

    // Ensure reasonable range
    if (maxVal - minVal < 10.0f) {
        // Range too small, expand it
        float center = (minVal + maxVal) / 2.0f;
        minVal = center - 50.0f;
        maxVal = center + 50.0f;
    }

    // Step 2: Normalize with higher precision (16-bit intermediate)
    cv::Mat normalized16;
    depthMap.convertTo(normalized16, CV_16U, 65535.0 / (maxVal - minVal),
                       -minVal * 65535.0 / (maxVal - minVal));

    // Set invalid pixels to a specific value (not black)
    normalized16.setTo(32768, ~validMask);  // Middle gray for invalid

    // Step 3: Apply colormap with dithering to reduce banding
    // First convert to 8-bit with dithering
    cv::Mat normalized8;
    normalized16.convertTo(normalized8, CV_8U, 1.0/256.0);

    // Apply Gaussian noise to reduce banding (very subtle)
    cv::Mat noise = cv::Mat(normalized8.size(), CV_8UC1);
    cv::randn(noise, 0, 1);  // Very small noise
    cv::add(normalized8, noise, normalized8, validMask);  // Only add to valid pixels

    // Apply colormap
    cv::applyColorMap(normalized8, colorized, colormap);

    // Set invalid pixels to dark blue instead of black (less jarring)
    cv::Vec3b invalidColor(64, 0, 0);  // Dark blue
    for (int y = 0; y < colorized.rows; ++y) {
        for (int x = 0; x < colorized.cols; ++x) {
            if (!validMask.at<uint8_t>(y, x)) {
                colorized.at<cv::Vec3b>(y, x) = invalidColor;
            }
        }
    }
}

// DepthProcessingConfig implementation
bool DepthProcessingConfig::validate() const {
    if (minDepthMm <= 0 || maxDepthMm <= minDepthMm) return false;
    if (medianKernelSize < 1 || medianKernelSize % 2 == 0) return false;
    if (bilateralD < 1) return false;
    if (bilateralSigmaColor < 0 || bilateralSigmaSpace < 0) return false;
    if (maxHoleSize < 0) return false;
    if (temporalAlpha < 0 || temporalAlpha > 1) return false;
    if (pointCloudScale <= 0) return false;
    if (maxDepthChange < 0) return false;
    return true;
}

std::string DepthProcessingConfig::toString() const {
    std::stringstream ss;
    ss << "Depth Processing Configuration:\n";
    ss << "  Depth Range: " << minDepthMm << " - " << maxDepthMm << " mm\n";
    ss << "  Median Filter: " << (applyMedianFilter ? "Yes" : "No");
    if (applyMedianFilter) ss << " (kernel: " << medianKernelSize << ")";
    ss << "\n";
    ss << "  Bilateral Filter: " << (applyBilateralFilter ? "Yes" : "No");
    if (applyBilateralFilter) {
        ss << " (d=" << bilateralD << ", sigmaColor=" << bilateralSigmaColor 
           << ", sigmaSpace=" << bilateralSigmaSpace << ")";
    }
    ss << "\n";
    ss << "  Fill Holes: " << (fillHoles ? "Yes" : "No");
    if (fillHoles) ss << " (max size: " << maxHoleSize << ")";
    ss << "\n";
    ss << "  Point Cloud: " << (computePointCloud ? "Yes" : "No");
    ss << "\n";
    return ss.str();
}

// DepthStatistics implementation
std::string DepthStatistics::toString() const {
    std::stringstream ss;
    ss << "Depth Statistics:\n";
    ss << "  Range: " << minDepth << " - " << maxDepth << " mm\n";
    ss << "  Mean: " << meanDepth << " mm\n";
    ss << "  Std Dev: " << stdDepth << " mm\n";
    ss << "  Valid Pixels: " << validPixels << "/" << totalPixels 
       << " (" << (validRatio * 100) << "%)\n";
    ss << "  Processing Time: " << processingTimeMs << " ms\n";
    return ss.str();
}

// DepthIO implementation for file I/O
bool DepthIO::savePFM(const std::string& filename, const cv::Mat& depthMap) {
    if (depthMap.empty() || depthMap.type() != CV_32F) {
        return false;
    }
    
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    // Write PFM header
    file << "Pf\n";
    file << depthMap.cols << " " << depthMap.rows << "\n";
    file << "-1.0\n";  // Little-endian
    
    // Write data (bottom to top for PFM format)
    for (int y = depthMap.rows - 1; y >= 0; --y) {
        const float* row = depthMap.ptr<float>(y);
        file.write(reinterpret_cast<const char*>(row), depthMap.cols * sizeof(float));
    }
    
    file.close();
    return true;
}

bool DepthIO::loadPFM(const std::string& filename, cv::Mat& depthMap) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    
    std::string header;
    int width, height;
    float scale;
    
    // Read header
    file >> header;
    if (header != "Pf" && header != "PF") {
        return false;
    }
    
    file >> width >> height >> scale;
    file.ignore(1);  // Skip newline
    
    // Allocate depth map
    depthMap = cv::Mat(height, width, CV_32F);
    
    // Read data (bottom to top for PFM format)
    bool littleEndian = (scale < 0);
    for (int y = height - 1; y >= 0; --y) {
        float* row = depthMap.ptr<float>(y);
        file.read(reinterpret_cast<char*>(row), width * sizeof(float));
        
        // Handle endianness if necessary
        if (!littleEndian) {
            for (int x = 0; x < width; ++x) {
                char* bytes = reinterpret_cast<char*>(&row[x]);
                std::swap(bytes[0], bytes[3]);
                std::swap(bytes[1], bytes[2]);
            }
        }
    }
    
    file.close();
    return true;
}

bool DepthIO::saveEXR(const std::string& filename, const cv::Mat& depthMap) {
    if (depthMap.empty()) {
        return false;
    }
    
    // Convert to 32-bit float if necessary
    cv::Mat floatDepth;
    if (depthMap.type() != CV_32F) {
        depthMap.convertTo(floatDepth, CV_32F);
    } else {
        floatDepth = depthMap;
    }
    
    // OpenCV doesn't have native EXR support, use imwrite with .exr extension
    // This will work if OpenCV is compiled with OpenEXR support
    try {
        return cv::imwrite(filename, floatDepth);
    } catch (const cv::Exception&) {
        // If EXR not supported, fall back to PFM
        std::string pfmFile = filename.substr(0, filename.find_last_of('.')) + ".pfm";
        return savePFM(pfmFile, floatDepth);
    }
}

bool DepthIO::savePNG16(const std::string& filename, const cv::Mat& depthMap, float scale) {
    if (depthMap.empty()) {
        return false;
    }
    
    // Convert to 16-bit unsigned integer
    cv::Mat depth16;
    double minVal, maxVal;
    cv::minMaxLoc(depthMap, &minVal, &maxVal, nullptr, nullptr, depthMap > 0);
    
    // Scale to 16-bit range preserving millimeter precision
    // Use scale factor to convert mm to 16-bit values
    depthMap.convertTo(depth16, CV_16U, scale);
    
    // Save as 16-bit PNG
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);  // Maximum compression
    
    return cv::imwrite(filename, depth16, compression_params);
}

void DepthProcessor::createLidarLikeDepthMap(const cv::Mat& inputDepth, cv::Mat& outputDepth) const {
    std::cout << "[Core DepthProcessor] Applying optimized depth post-processing for 70mm baseline..." << std::endl;

    // Copy input to output
    inputDepth.copyTo(outputDepth);

    // Step 1: Identify valid depth range optimized for 70mm baseline
    // For 70mm baseline with 6mm focal length: optimal range is 100-800mm
    cv::Mat mask = (inputDepth > pImpl->config.minDepthMm) & (inputDepth < pImpl->config.maxDepthMm);
    cv::Mat holes = ~mask;

    int holesToFill = cv::countNonZero(holes);
    int totalPixels = inputDepth.rows * inputDepth.cols;
    double holeRatio = double(holesToFill) / totalPixels * 100.0;

    std::cout << "[Core DepthProcessor] Invalid pixels: " << holesToFill << " (" << holeRatio << "%)" << std::endl;

    // Step 2: PRECISION-AWARE hole filling using weighted median instead of mean
    // This prevents artificial depth levels and preserves natural depth variation
    if (holesToFill > 0 && holesToFill < totalPixels * 0.5) {  // Only fill if <50% holes
        cv::Mat filledDepth = outputDepth.clone();

        // Single-pass weighted median filling for small holes only
        const int maxHoleRadius = 2;  // Much smaller radius to preserve detail

        for (int y = maxHoleRadius; y < outputDepth.rows - maxHoleRadius; ++y) {
            for (int x = maxHoleRadius; x < outputDepth.cols - maxHoleRadius; ++x) {
                if (filledDepth.at<float>(y, x) <= pImpl->config.minDepthMm) {

                    // Collect valid neighbor depths
                    std::vector<float> validDepths;
                    validDepths.reserve(25);

                    for (int dy = -maxHoleRadius; dy <= maxHoleRadius; ++dy) {
                        for (int dx = -maxHoleRadius; dx <= maxHoleRadius; ++dx) {
                            if (dx == 0 && dy == 0) continue;

                            float neighborDepth = filledDepth.at<float>(y + dy, x + dx);
                            if (neighborDepth > pImpl->config.minDepthMm &&
                                neighborDepth < pImpl->config.maxDepthMm) {
                                validDepths.push_back(neighborDepth);
                            }
                        }
                    }

                    // Use median instead of mean to avoid creating artificial levels
                    if (validDepths.size() >= 3) {  // Need at least 3 valid neighbors
                        std::nth_element(validDepths.begin(),
                                       validDepths.begin() + validDepths.size()/2,
                                       validDepths.end());
                        filledDepth.at<float>(y, x) = validDepths[validDepths.size()/2];
                    }
                }
            }
        }

        filledDepth.copyTo(outputDepth);
        std::cout << "[Core DepthProcessor] Precision hole filling completed" << std::endl;
    }

    // Step 3: EDGE-PRESERVING smoothing with WLS filter parameters optimized for 70mm baseline
    // Skip morphological operations - they create artificial depth levels
    if (pImpl->config.applyBilateralFilter && pImpl->config.bilateralD > 0) {
        cv::Mat smoothed;
        // Adaptive bilateral filter - preserve edges while smoothing
        // Reduced parameters to preserve fine detail at 0.005mm target precision
        cv::bilateralFilter(outputDepth, smoothed,
                          pImpl->config.bilateralD,  // Use config value (default 9)
                          pImpl->config.bilateralSigmaColor * 0.5,  // Reduce color sigma
                          pImpl->config.bilateralSigmaSpace * 0.5); // Reduce space sigma

        // Very conservative blending to preserve original depth precision
        cv::Mat validMask = outputDepth > pImpl->config.minDepthMm;
        cv::addWeighted(outputDepth, 0.95, smoothed, 0.05, 0, outputDepth);
    }

    // Step 4: OUTLIER removal with small median filter (not smoothing)
    if (pImpl->config.applyMedianFilter && pImpl->config.medianKernelSize > 1) {
        // Detect outliers using local statistics
        cv::Mat localMean, localStdDev;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT,
                                                  cv::Size(5, 5));

        // Calculate local statistics
        cv::Mat depthSquared;
        cv::multiply(outputDepth, outputDepth, depthSquared);
        cv::morphologyEx(outputDepth, localMean, cv::MORPH_DILATE, kernel);
        cv::morphologyEx(depthSquared, localStdDev, cv::MORPH_DILATE, kernel);

        // Identify outliers (> 3 sigma from local mean)
        cv::Mat variance = localStdDev - localMean.mul(localMean);
        cv::Mat stdDev;
        cv::sqrt(variance, stdDev);
        cv::Mat outlierMask = cv::abs(outputDepth - localMean) > 3 * stdDev;

        // Apply median filter ONLY to outliers
        cv::Mat medianFiltered;
        cv::medianBlur(outputDepth, medianFiltered, pImpl->config.medianKernelSize);
        medianFiltered.copyTo(outputDepth, outlierMask);
    }

    // Final validation - ensure no artificial quantization levels
    double finalMinVal, finalMaxVal;
    cv::minMaxLoc(outputDepth, &finalMinVal, &finalMaxVal, nullptr, nullptr, outputDepth > 0);
    cv::Scalar meanDepth = cv::mean(outputDepth, outputDepth > 0);
    int finalValidPixels = cv::countNonZero(outputDepth > 0);

    std::cout << "[Core DepthProcessor] Optimized depth map: "
              << "Range=" << finalMinVal << "-" << finalMaxVal << "mm, "
              << "Mean=" << meanDepth[0] << "mm, "
              << "Valid=" << finalValidPixels << "/" << totalPixels
              << " (" << (100.0 * finalValidPixels / totalPixels) << "%)" << std::endl;
}

} // namespace stereo
} // namespace unlook