#include "unlook/stereo/StereoMatcher.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include "unlook/stereo/CensusStereoMatcher.hpp"
#include "unlook/stereo/ProgressiveStereoMatcher.hpp"
#ifdef HAVE_BOOFCV
#include "unlook/stereo/BoofCVStereoMatcher.hpp"
#endif
#include <opencv2/imgproc.hpp>
#include <limits>
#include <cmath>

namespace unlook {
namespace stereo {

// Base class implementation
bool StereoMatcher::computeDisparityWithConfidence(const cv::Mat& leftRectified,
                                                   const cv::Mat& rightRectified,
                                                   cv::Mat& disparity,
                                                   cv::Mat& confidence) {
    // Default implementation - compute disparity and generate simple confidence
    if (!computeDisparity(leftRectified, rightRectified, disparity)) {
        return false;
    }
    
    // Simple confidence based on disparity validity
    confidence = cv::Mat::ones(disparity.size(), CV_32F);
    confidence.setTo(0, disparity <= 0);
    
    return true;
}

bool StereoMatcher::updateParameter(const std::string& name, double value) {
    auto params = getParameters();
    std::map<std::string, double> paramMap = params.toMap();
    paramMap[name] = value;
    params.fromMap(paramMap);
    return setParameters(params);
}

bool StereoMatcher::computeQualityMetrics(const cv::Mat& disparity,
                                         StereoQualityMetrics& metrics) const {
    if (disparity.empty()) return false;
    
    // Convert to float if needed
    cv::Mat disp32f;
    if (disparity.type() != CV_32F) {
        disparity.convertTo(disp32f, CV_32F);
    } else {
        disp32f = disparity;
    }
    
    // Compute statistics
    int totalPixels = disp32f.rows * disp32f.cols;
    metrics.invalidPixelCount = cv::countNonZero(disp32f <= 0);
    metrics.validPixelRatio = 1.0 - (double)metrics.invalidPixelCount / totalPixels;
    
    // Compute mean and std for valid pixels
    cv::Mat validMask = disp32f > 0;
    cv::Scalar mean, stddev;
    cv::meanStdDev(disp32f, mean, stddev, validMask);
    
    metrics.avgDisparity = mean[0];
    metrics.stdDisparity = stddev[0];
    
    // Placeholder for other metrics
    metrics.textureScore = 0.8;  // Placeholder
    metrics.confidenceScore = metrics.validPixelRatio;
    metrics.occludedPixelCount = metrics.invalidPixelCount / 2;  // Estimate
    
    return true;
}

bool StereoMatcher::applyPostProcessing(cv::Mat& disparity, const cv::Mat& leftImage) {
    // Default post-processing: median filter
    if (disparity.empty()) return false;
    
    cv::Mat filtered;
    cv::medianBlur(disparity, filtered, 5);
    disparity = filtered;
    
    return true;
}

double StereoMatcher::validateDisparity(const cv::Mat& disparity, cv::Mat& validMask) const {
    if (disparity.empty()) return 0.0;
    
    // Create valid mask
    validMask = disparity > 0;
    
    // Additional validation based on local consistency
    cv::Mat gradX, gradY;
    cv::Sobel(disparity, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(disparity, gradY, CV_32F, 0, 1, 3);
    
    cv::Mat gradMag;
    cv::magnitude(gradX, gradY, gradMag);
    
    // Mark pixels with too high gradient as invalid
    validMask.setTo(0, gradMag > params_.speckleRange);
    
    int validCount = cv::countNonZero(validMask);
    int totalCount = disparity.rows * disparity.cols;
    
    return (double)validCount / totalCount;
}

bool StereoMatcher::disparityToDepth(const cv::Mat& disparity,
                                    const cv::Mat& Q,
                                    cv::Mat& depth,
                                    bool handleMissingValues) {
    if (disparity.empty() || Q.empty()) return false;

    // CRITICAL FIX: Pre-filter disparity to avoid infinite depth values
    // Minimum disparity threshold to prevent divide-by-near-zero
    // With 70mm baseline, disparity < 1px would give depth > 122 meters (unrealistic)
    cv::Mat disparityFiltered;
    disparity.copyTo(disparityFiltered);

    // Set invalid disparities to negative (will be filtered later)
    cv::Mat invalidMask = (disparity <= 1.0f);  // Less than 1 pixel = invalid
    disparityFiltered.setTo(-1, invalidMask);

    // Convert disparity to 3D points
    cv::Mat points3D;
    cv::reprojectImageTo3D(disparityFiltered, points3D, Q, true);

    // Extract Z channel as depth
    std::vector<cv::Mat> channels(3);
    cv::split(points3D, channels);
    depth = channels[2];  // Z channel

    // AGGRESSIVE filtering to prevent OOM - synchronized with DepthProcessor config
    if (handleMissingValues) {
        // Filter: invalid disparity, negative depth, unrealistic depth (>4000mm), infinite/NaN
        // Synchronized with DepthProcessor maxDepthMm=4000mm
        cv::Mat mask = (disparity <= 1.0f) | (depth < 0) | (depth > 4000.0f) |
                       (cv::abs(depth) == std::numeric_limits<float>::infinity());

        // Additional safety: replace any remaining extreme values
        for (int y = 0; y < depth.rows; ++y) {
            for (int x = 0; x < depth.cols; ++x) {
                float d = depth.at<float>(y, x);
                if (!std::isfinite(d) || d < 0 || d > 4000.0f) {
                    depth.at<float>(y, x) = 0.0f;
                }
            }
        }
    }

    return true;
}

void StereoMatcher::normalizeDisparity(const cv::Mat& disparity, cv::Mat& normalized) const {
    double minVal, maxVal;
    cv::minMaxLoc(disparity, &minVal, &maxVal, nullptr, nullptr, disparity > 0);
    
    if (maxVal > minVal) {
        disparity.convertTo(normalized, CV_8U, 255.0 / (maxVal - minVal), 
                           -minVal * 255.0 / (maxVal - minVal));
    } else {
        normalized = cv::Mat::zeros(disparity.size(), CV_8U);
    }
}

bool StereoMatcher::validateStereoPair(const cv::Mat& left, const cv::Mat& right) const {
    if (left.empty() || right.empty()) return false;
    if (left.size() != right.size()) return false;
    if (left.type() != right.type()) return false;
    return true;
}

std::unique_ptr<StereoMatcher> StereoMatcher::create(StereoAlgorithm algorithm) {
    switch (algorithm) {
        // OpenCV algorithms
        case StereoAlgorithm::SGBM:
            return std::make_unique<SGBMStereoMatcher>();
        case StereoAlgorithm::BM:
            return std::make_unique<SGBMStereoMatcher>(); // TODO: Create BMStereoMatcher

        // Census Transform matcher (optimized for VCSEL dots)
        case StereoAlgorithm::CENSUS:
            return std::make_unique<CensusStereoMatcher>();

        // Custom progressive matcher
        case StereoAlgorithm::CUSTOM:
            return std::make_unique<ProgressiveStereoMatcher>();
            
#ifdef HAVE_BOOFCV
        // BoofCV algorithms - all use BoofCVStereoMatcher with different configurations
        case StereoAlgorithm::BOOFCV_SGBM:
        case StereoAlgorithm::BOOFCV_BM:
        {
            // Check if BoofCV is available before creating
            if (!BoofCVStereoMatcher::isAvailable()) {
                return nullptr;
            }
            
            // Create configuration based on algorithm
            BoofCVStereoConfig config;
            if (algorithm == StereoAlgorithm::BOOFCV_SGBM) {
                config.algorithm = BoofCVAlgorithm::DENSE_DISPARITY_SGM;
                config.subpixel_enabled = true;
            } else {
                config.algorithm = BoofCVAlgorithm::DENSE_DISPARITY_BM;
                config.subpixel_enabled = false;
            }
            
            return std::make_unique<BoofCVStereoMatcher>(config);
        }
#else
        // BoofCV algorithms not available - return null
        case StereoAlgorithm::BOOFCV_SGBM:
        case StereoAlgorithm::BOOFCV_BM:
            return nullptr;
#endif
            
        default:
            return nullptr;
    }
}

std::vector<StereoAlgorithm> StereoMatcher::getAvailableAlgorithms() {
    std::vector<StereoAlgorithm> algorithms = {
        StereoAlgorithm::SGBM,
        StereoAlgorithm::BM,
        StereoAlgorithm::CENSUS,  // Census Transform (VCSEL-optimized)
        StereoAlgorithm::CUSTOM   // Progressive matcher
    };
    
    // Add BoofCV algorithms if available
#ifdef HAVE_BOOFCV
    if (BoofCVStereoMatcher::isAvailable()) {
        algorithms.push_back(StereoAlgorithm::BOOFCV_SGBM);
        algorithms.push_back(StereoAlgorithm::BOOFCV_BM);
    }
#endif
    
    return algorithms;
}

// StereoQualityMetrics implementation
std::string StereoQualityMetrics::toString() const {
    std::stringstream ss;
    ss << "  Average Disparity: " << avgDisparity << "\n";
    ss << "  Std Disparity: " << stdDisparity << "\n";
    ss << "  Valid Pixel Ratio: " << (validPixelRatio * 100) << "%\n";
    ss << "  Invalid Pixels: " << invalidPixelCount << "\n";
    ss << "  Occluded Pixels: " << occludedPixelCount << "\n";
    ss << "  Texture Score: " << textureScore << "\n";
    ss << "  Confidence Score: " << confidenceScore << "\n";
    ss << "  Processing Time: " << processingTimeMs << " ms\n";
    return ss.str();
}

// Factory implementation
std::map<StereoAlgorithm, std::function<std::unique_ptr<StereoMatcher>()>> 
    StereoMatcherFactory::creators_;

void StereoMatcherFactory::registerMatcher(StereoAlgorithm algorithm,
                                          std::function<std::unique_ptr<StereoMatcher>()> creator) {
    creators_[algorithm] = creator;
}

std::unique_ptr<StereoMatcher> StereoMatcherFactory::create(StereoAlgorithm algorithm) {
    auto it = creators_.find(algorithm);
    if (it != creators_.end()) {
        return it->second();
    }
    return StereoMatcher::create(algorithm);
}

} // namespace stereo
} // namespace unlook