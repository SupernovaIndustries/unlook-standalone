#include "unlook/stereo/StereoMatcher.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include <opencv2/imgproc.hpp>

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
    
    // Convert disparity to 3D points
    cv::Mat points3D;
    cv::reprojectImageTo3D(disparity, points3D, Q, true);
    
    // Extract Z channel as depth
    std::vector<cv::Mat> channels(3);
    cv::split(points3D, channels);
    depth = channels[2];  // Z channel
    
    // Handle missing values if requested
    if (handleMissingValues) {
        cv::Mat mask = (disparity <= 0) | (depth > 10000) | (depth < 0);
        depth.setTo(0, mask);
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
        case StereoAlgorithm::SGBM:
            return std::make_unique<SGBMStereoMatcher>();
        default:
            return nullptr;
    }
}

std::vector<StereoAlgorithm> StereoMatcher::getAvailableAlgorithms() {
    return {StereoAlgorithm::SGBM, StereoAlgorithm::BM};
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