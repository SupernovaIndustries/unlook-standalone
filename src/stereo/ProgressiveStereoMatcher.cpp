#include "unlook/stereo/ProgressiveStereoMatcher.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <algorithm>
#include <numeric>

#ifdef __ARM_NEON
#include <arm_neon.h>
#endif

namespace unlook {
namespace stereo {

// ProgressiveStats implementation
std::string ProgressiveStats::toString() const {
    std::stringstream ss;
    ss << "Progressive Stereo Matching Statistics:\n";
    ss << "  Total Processing Time: " << totalProcessingTimeMs << " ms\n";
    ss << "  Layer Times: NEAR=" << perLayerTimeMs[0]
       << "ms, MID=" << perLayerTimeMs[1]
       << "ms, FAR=" << perLayerTimeMs[2] << "ms\n";
    ss << "  Valid Pixels: " << totalValidPixels << " total\n";
    ss << "  Layer Valid: NEAR=" << layerValidPixels[0]
       << ", MID=" << layerValidPixels[1]
       << ", FAR=" << layerValidPixels[2] << "\n";
    ss << "  Average Confidence: " << averageConfidence << "\n";
    ss << "  Early Terminations: " << earlyTerminationCount << "\n";
    ss << "  Background Filtered: " << backgroundFilteredPixels << " pixels\n";
    ss << "  Memory Usage: " << memoryUsageMB << " MB\n";
    return ss.str();
}

// ProgressiveStereoMatcher implementation
ProgressiveStereoMatcher::ProgressiveStereoMatcher() {
    // Initialize base SGBM matcher
    baseMatcher_ = std::make_unique<SGBMStereoMatcher>();

    // Set up default progressive configuration for 70mm baseline
    config_.baselineMm = 70.017f;
    config_.focalLength = 1456.0f;  // Approximate from IMX296 sensor

    // Setup default depth layers
    setupDefaultLayers();

    // Initialize buffers
    layerBuffers_.resize(3);
    confidenceBuffers_.resize(3);

    // Create WLS filter for post-processing
    if (params_.useWLSFilter) {
        auto sgbm = cv::StereoSGBM::create();
        rightMatcher_ = cv::ximgproc::createRightMatcher(sgbm);
        wlsFilter_ = cv::ximgproc::createDisparityWLSFilter(sgbm);
        wlsFilter_->setLambda(params_.wlsLambda);
        wlsFilter_->setSigmaColor(params_.wlsSigma);
    }

    std::cout << "[ProgressiveStereoMatcher] Initialized with 70mm baseline optimization\n";
    std::cout << "  Disparity ROI: " << roiMinDisparity_ << "-" << roiMaxDisparity_ << "\n";
    std::cout << "  Depth range: " << disparityToDepth(roiMaxDisparity_)
              << "-" << disparityToDepth(roiMinDisparity_) << " mm\n";
}

ProgressiveStereoMatcher::~ProgressiveStereoMatcher() = default;

void ProgressiveStereoMatcher::setupDefaultLayers() {
    layers_.clear();

    // Layer 1: NEAR objects (300-600mm range)
    // High disparity = close objects, need fine detail
    DepthLayer nearLayer;
    nearLayer.name = "NEAR";
    nearLayer.minDepthMm = 300.0f;
    nearLayer.maxDepthMm = 600.0f;
    nearLayer.maxDisparity = static_cast<int>(depthToDisparity(nearLayer.minDepthMm));
    nearLayer.minDisparity = static_cast<int>(depthToDisparity(nearLayer.maxDepthMm));
    nearLayer.numDisparities = ((nearLayer.maxDisparity - nearLayer.minDisparity + 15) / 16) * 16;
    nearLayer.confidenceThreshold = 0.9f;
    nearLayer.enabled = true;
    nearLayer.blockSize = 7;  // Smaller block for fine detail
    nearLayer.uniquenessRatio = 5;  // Strict for precision
    nearLayer.speckleWindowSize = 50;
    layers_.push_back(nearLayer);

    // Layer 2: MID objects (500-1000mm range)
    // Medium disparity = medium distance, balanced parameters
    DepthLayer midLayer;
    midLayer.name = "MID";
    midLayer.minDepthMm = 500.0f;
    midLayer.maxDepthMm = 1000.0f;
    midLayer.maxDisparity = static_cast<int>(depthToDisparity(midLayer.minDepthMm));
    midLayer.minDisparity = static_cast<int>(depthToDisparity(midLayer.maxDepthMm));
    midLayer.numDisparities = ((midLayer.maxDisparity - midLayer.minDisparity + 15) / 16) * 16;
    midLayer.confidenceThreshold = 0.85f;
    midLayer.enabled = true;
    midLayer.blockSize = 9;  // Medium block
    midLayer.uniquenessRatio = 10;
    midLayer.speckleWindowSize = 100;
    layers_.push_back(midLayer);

    // Layer 3: FAR objects (900-1500mm range)
    // Low disparity = far objects, larger blocks for stability
    DepthLayer farLayer;
    farLayer.name = "FAR";
    farLayer.minDepthMm = 900.0f;
    farLayer.maxDepthMm = 1500.0f;
    farLayer.maxDisparity = static_cast<int>(depthToDisparity(farLayer.minDepthMm));
    farLayer.minDisparity = static_cast<int>(depthToDisparity(farLayer.maxDepthMm));
    farLayer.numDisparities = ((farLayer.maxDisparity - farLayer.minDisparity + 15) / 16) * 16;
    farLayer.confidenceThreshold = 0.8f;
    farLayer.enabled = true;
    farLayer.blockSize = 11;  // Larger block for stability
    farLayer.uniquenessRatio = 15;
    farLayer.speckleWindowSize = 150;
    layers_.push_back(farLayer);

    std::cout << "[ProgressiveStereoMatcher] Default layers configured:\n";
    for (const auto& layer : layers_) {
        std::cout << "  " << layer.name << ": "
                  << layer.minDepthMm << "-" << layer.maxDepthMm << "mm, "
                  << "disparity " << layer.minDisparity << "-" << layer.maxDisparity
                  << " (" << layer.numDisparities << " levels)\n";
    }
}

void ProgressiveStereoMatcher::setDepthLayers(const std::vector<DepthLayer>& layers) {
    layers_ = layers;

    // Ensure disparity values are properly calculated
    for (auto& layer : layers_) {
        if (layer.minDisparity == 0 && layer.maxDisparity == 0) {
            layer.maxDisparity = static_cast<int>(depthToDisparity(layer.minDepthMm));
            layer.minDisparity = static_cast<int>(depthToDisparity(layer.maxDepthMm));
            layer.numDisparities = ((layer.maxDisparity - layer.minDisparity + 15) / 16) * 16;
        }
    }
}

void ProgressiveStereoMatcher::setDisparityROI(int minDisparity, int maxDisparity) {
    roiMinDisparity_ = minDisparity;
    roiMaxDisparity_ = maxDisparity;

    std::cout << "[ProgressiveStereoMatcher] Disparity ROI set to "
              << minDisparity << "-" << maxDisparity << "\n";
    std::cout << "  Depth range: " << disparityToDepth(maxDisparity)
              << "-" << disparityToDepth(minDisparity) << " mm\n";
}

bool ProgressiveStereoMatcher::computeDisparity(const cv::Mat& leftRectified,
                                               const cv::Mat& rightRectified,
                                               cv::Mat& disparity) {
    auto startTime = std::chrono::high_resolution_clock::now();

    // Validate input
    if (!validateStereoPair(leftRectified, rightRectified)) {
        std::cerr << "[ProgressiveStereoMatcher] Invalid stereo pair\n";
        return false;
    }

    // Reset statistics
    resetStatistics();

    // Initialize output
    disparity = cv::Mat::zeros(leftRectified.size(), CV_16S);
    cv::Mat confidence = cv::Mat::zeros(leftRectified.size(), CV_32F);

    if (!config_.enableProgressive) {
        // Fall back to standard SGBM processing
        params_.minDisparity = roiMinDisparity_;
        params_.numDisparities = roiMaxDisparity_ - roiMinDisparity_;
        baseMatcher_->setParameters(params_);
        bool result = baseMatcher_->computeDisparity(leftRectified, rightRectified, disparity);

        // Filter background if enabled
        if (config_.filterBackground && result) {
            filterBackground(disparity, confidence);
        }

        return result;
    }

    // Progressive layer processing
    std::vector<cv::Mat> layerDisparities;
    std::vector<cv::Mat> layerConfidences;

    int progress = 0;
    for (size_t i = 0; i < layers_.size(); ++i) {
        if (!layers_[i].enabled) continue;

        auto layerStart = std::chrono::high_resolution_clock::now();

        cv::Mat layerDisparity, layerConfidence;
        bool layerSuccess = processDepthLayer(leftRectified, rightRectified,
                                             layers_[i], layerDisparity, layerConfidence);

        if (layerSuccess) {
            layerDisparities.push_back(layerDisparity);
            layerConfidences.push_back(layerConfidence);

            // Update statistics
            auto layerEnd = std::chrono::high_resolution_clock::now();
            stats_.perLayerTimeMs[i] = std::chrono::duration<double, std::milli>(
                layerEnd - layerStart).count();

            // Check for early termination
            if (config_.earlyTermination && checkEarlyTermination(layerConfidence)) {
                stats_.earlyTerminationCount++;
                std::cout << "[ProgressiveStereoMatcher] Early termination after layer "
                          << layers_[i].name << "\n";
                break;
            }
        }

        // Update progress
        progress = static_cast<int>((i + 1) * 100 / layers_.size());
        if (progressCallback_) {
            progressCallback_(progress);
        }
    }

    // Merge layer results
    if (!layerDisparities.empty()) {
        mergeLayers(layerDisparities, layerConfidences, disparity, confidence);

        // Filter background if enabled
        if (config_.filterBackground) {
            filterBackground(disparity, confidence);
        }

        // Apply post-processing if enabled
        if (params_.useWLSFilter) {
            applyPostProcessing(disparity, leftRectified);
        }
    }

    // Calculate final statistics
    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.totalProcessingTimeMs = std::chrono::duration<double, std::milli>(
        endTime - startTime).count();

    // Count valid pixels
    stats_.totalValidPixels = cv::countNonZero(disparity > 0);

    // Calculate average confidence
    if (confidence.total() > 0) {
        cv::Scalar mean = cv::mean(confidence, disparity > 0);
        stats_.averageConfidence = static_cast<float>(mean[0]);
    }

    std::cout << "[ProgressiveStereoMatcher] Processing complete: "
              << stats_.totalProcessingTimeMs << " ms\n";

    return true;
}

bool ProgressiveStereoMatcher::computeDisparityWithConfidence(const cv::Mat& leftRectified,
                                                             const cv::Mat& rightRectified,
                                                             cv::Mat& disparity,
                                                             cv::Mat& confidence) {
    // Main processing
    bool result = computeDisparity(leftRectified, rightRectified, disparity);

    if (result) {
        // Generate confidence map if not already computed
        if (confidence.empty()) {
            confidence = cv::Mat::zeros(disparity.size(), CV_32F);

            // Simple confidence based on disparity validity and texture
            cv::Mat gray;
            cv::cvtColor(leftRectified, gray, cv::COLOR_BGR2GRAY);

            cv::Mat gradX, gradY;
            cv::Sobel(gray, gradX, CV_16S, 1, 0, 3);
            cv::Sobel(gray, gradY, CV_16S, 0, 1, 3);

            cv::Mat gradMag;
            cv::magnitude(gradX, gradY, gradMag);
            cv::normalize(gradMag, gradMag, 0, 1, cv::NORM_MINMAX, CV_32F);

            // Confidence = texture * disparity validity
            for (int y = 0; y < disparity.rows; ++y) {
                for (int x = 0; x < disparity.cols; ++x) {
                    if (disparity.at<int16_t>(y, x) > 0) {
                        confidence.at<float>(y, x) = gradMag.at<float>(y, x);
                    }
                }
            }
        }
    }

    return result;
}

bool ProgressiveStereoMatcher::processDepthLayer(const cv::Mat& left,
                                                const cv::Mat& right,
                                                const DepthLayer& layer,
                                                cv::Mat& layerDisparity,
                                                cv::Mat& layerConfidence) {
    std::cout << "[ProgressiveStereoMatcher] Processing layer " << layer.name
              << " (disparity " << layer.minDisparity << "-" << layer.maxDisparity << ")\n";

    // Configure SGBM parameters for this layer
    StereoMatchingParams layerParams = params_;
    layerParams.minDisparity = layer.minDisparity;
    layerParams.numDisparities = layer.numDisparities;
    layerParams.blockSize = layer.blockSize;
    layerParams.uniquenessRatio = layer.uniquenessRatio;
    layerParams.speckleWindowSize = layer.speckleWindowSize;

    // Adjust P1/P2 based on block size
    int cn = left.channels();
    layerParams.P1 = 8 * cn * layer.blockSize * layer.blockSize;
    layerParams.P2 = 32 * cn * layer.blockSize * layer.blockSize;

    // Set parameters in base matcher
    baseMatcher_->setParameters(layerParams);

    // Compute disparity for this layer
    bool result = baseMatcher_->computeDisparity(left, right, layerDisparity);

    if (result) {
        // Generate confidence map for this layer
        layerConfidence = cv::Mat::ones(layerDisparity.size(), CV_32F) * 0.5f;

        // Higher confidence for valid disparities in the expected range
        for (int y = 0; y < layerDisparity.rows; ++y) {
            for (int x = 0; x < layerDisparity.cols; ++x) {
                int16_t d = layerDisparity.at<int16_t>(y, x);
                if (d >= layer.minDisparity * 16 && d <= layer.maxDisparity * 16) {
                    layerConfidence.at<float>(y, x) = layer.confidenceThreshold;
                } else if (d > 0) {
                    layerConfidence.at<float>(y, x) = 0.3f;
                }
            }
        }

        // Count valid pixels
        int validCount = cv::countNonZero(layerDisparity > 0);
        std::cout << "  Valid pixels: " << validCount
                  << " (" << (100.0 * validCount / layerDisparity.total()) << "%)\n";
    }

    return result;
}

void ProgressiveStereoMatcher::mergeLayers(const std::vector<cv::Mat>& layers,
                                          const std::vector<cv::Mat>& confidences,
                                          cv::Mat& finalDisparity,
                                          cv::Mat& finalConfidence) {
    if (layers.empty()) return;

    // Initialize with first layer
    finalDisparity = layers[0].clone();
    finalConfidence = confidences[0].clone();

    // Merge subsequent layers
    for (size_t i = 1; i < layers.size(); ++i) {
        const cv::Mat& layerDisp = layers[i];
        const cv::Mat& layerConf = confidences[i];

        // Use higher confidence values
        for (int y = 0; y < finalDisparity.rows; ++y) {
            for (int x = 0; x < finalDisparity.cols; ++x) {
                float currConf = finalConfidence.at<float>(y, x);
                float newConf = layerConf.at<float>(y, x);

                // Replace if new confidence is higher or current is invalid
                if ((newConf > currConf) || (finalDisparity.at<int16_t>(y, x) <= 0)) {
                    finalDisparity.at<int16_t>(y, x) = layerDisp.at<int16_t>(y, x);
                    finalConfidence.at<float>(y, x) = newConf;
                }
            }
        }
    }

    // Apply confidence threshold
    for (int y = 0; y < finalDisparity.rows; ++y) {
        for (int x = 0; x < finalDisparity.cols; ++x) {
            if (finalConfidence.at<float>(y, x) < params_.confidenceThreshold) {
                finalDisparity.at<int16_t>(y, x) = 0;
            }
        }
    }
}

void ProgressiveStereoMatcher::filterBackground(cv::Mat& disparity, const cv::Mat& confidence) {
    if (!config_.filterBackground) return;

    // Calculate minimum disparity for background threshold
    float minDisparityForBackground = depthToDisparity(config_.backgroundThresholdMm);

    int filteredCount = 0;

    // Filter pixels beyond threshold
    for (int y = 0; y < disparity.rows; ++y) {
        for (int x = 0; x < disparity.cols; ++x) {
            int16_t d = disparity.at<int16_t>(y, x);
            // Convert from subpixel (16x) to pixel disparity
            float pixelDisparity = d / 16.0f;

            if (pixelDisparity > 0 && pixelDisparity < minDisparityForBackground) {
                disparity.at<int16_t>(y, x) = 0;
                filteredCount++;
            }
        }
    }

    stats_.backgroundFilteredPixels = filteredCount;

    if (filteredCount > 0) {
        std::cout << "[ProgressiveStereoMatcher] Filtered " << filteredCount
                  << " background pixels (beyond " << config_.backgroundThresholdMm << "mm)\n";
    }
}

bool ProgressiveStereoMatcher::checkEarlyTermination(const cv::Mat& confidence) const {
    if (!config_.earlyTermination) return false;

    // Count high-confidence pixels
    int highConfCount = 0;
    int totalValid = 0;

    for (int y = 0; y < confidence.rows; ++y) {
        for (int x = 0; x < confidence.cols; ++x) {
            float conf = confidence.at<float>(y, x);
            if (conf > 0) {
                totalValid++;
                if (conf >= config_.earlyTerminationThreshold) {
                    highConfCount++;
                }
            }
        }
    }

    // Terminate if enough pixels have high confidence
    float ratio = (totalValid > 0) ? (float)highConfCount / totalValid : 0;
    return ratio > 0.8f;  // 80% of valid pixels have high confidence
}

void ProgressiveStereoMatcher::propagateConfidence(const cv::Mat& prevConfidence,
                                                  cv::Mat& currConfidence) {
    if (!config_.propagateConfidence || prevConfidence.empty()) return;

    // Propagate high confidence regions to guide current layer
    for (int y = 0; y < currConfidence.rows; ++y) {
        for (int x = 0; x < currConfidence.cols; ++x) {
            float prev = prevConfidence.at<float>(y, x);
            float curr = currConfidence.at<float>(y, x);

            // Boost confidence in regions that were confident in previous layer
            if (prev > 0.9f && curr > 0) {
                currConfidence.at<float>(y, x) = std::min(1.0f, curr * 1.2f);
            }
        }
    }
}

float ProgressiveStereoMatcher::depthToDisparity(float depthMm) const {
    if (depthMm <= 0) return 0;
    return (config_.baselineMm * config_.focalLength) / depthMm;
}

float ProgressiveStereoMatcher::disparityToDepth(float disparity) const {
    if (disparity <= 0) return 0;
    return (config_.baselineMm * config_.focalLength) / disparity;
}

bool ProgressiveStereoMatcher::setParameters(const StereoMatchingParams& params) {
    params_ = params;

    // Update WLS filter if needed
    if (params_.useWLSFilter && wlsFilter_) {
        wlsFilter_->setLambda(params_.wlsLambda);
        wlsFilter_->setSigmaColor(params_.wlsSigma);
    }

    // Propagate to base matcher
    if (baseMatcher_) {
        baseMatcher_->setParameters(params);
    }

    return true;
}

StereoMatchingParams ProgressiveStereoMatcher::getParameters() const {
    return params_;
}

bool ProgressiveStereoMatcher::applyPostProcessing(cv::Mat& disparity,
                                                  const cv::Mat& leftImage) {
    if (!params_.useWLSFilter || !wlsFilter_) {
        return false;
    }

    try {
        // Apply WLS filter
        cv::Mat filteredDisparity;
        wlsFilter_->filter(disparity, leftImage, filteredDisparity);
        disparity = filteredDisparity;

        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[ProgressiveStereoMatcher] WLS filter error: " << e.what() << std::endl;
        return false;
    }
}

void ProgressiveStereoMatcher::setProgressiveConfig(const ProgressiveConfig& config) {
    config_ = config;
}

ProgressiveConfig ProgressiveStereoMatcher::getProgressiveConfig() const {
    return config_;
}

std::vector<DepthLayer> ProgressiveStereoMatcher::getDepthLayers() const {
    return layers_;
}

ProgressiveStats ProgressiveStereoMatcher::getStatistics() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    return stats_;
}

void ProgressiveStereoMatcher::resetStatistics() {
    std::lock_guard<std::mutex> lock(statsMutex_);
    stats_ = ProgressiveStats();
}

void ProgressiveStereoMatcher::setLayerEnabled(int layerIndex, bool enabled) {
    if (layerIndex >= 0 && layerIndex < static_cast<int>(layers_.size())) {
        layers_[layerIndex].enabled = enabled;
    }
}

void ProgressiveStereoMatcher::setProgressCallback(std::function<void(int)> callback) {
    progressCallback_ = callback;
}

// BackgroundFilter implementation

BackgroundFilter::BackgroundFilter(float maxDepthMm, float baselineMm, float focalLength)
    : maxDepthMm_(maxDepthMm), baselineMm_(baselineMm), focalLength_(focalLength) {
    // Calculate minimum disparity for maximum depth
    minDisparity_ = (baselineMm_ * focalLength_) / maxDepthMm_;

    std::cout << "[BackgroundFilter] Configured for max depth " << maxDepthMm_
              << "mm (min disparity: " << minDisparity_ << " pixels)\n";
}

int BackgroundFilter::filterDisparity(cv::Mat& disparity, const cv::Mat& confidence) {
    auto startTime = std::chrono::high_resolution_clock::now();

    int filteredCount = 0;
    stats_.totalPixels = disparity.total();

    // Create mask for background pixels
    cv::Mat mask;
    createMask(disparity, mask);

    // Apply mask to disparity
    for (int y = 0; y < disparity.rows; ++y) {
        for (int x = 0; x < disparity.cols; ++x) {
            if (mask.at<uint8_t>(y, x) == 0) {
                // Background pixel - set to invalid
                if (disparity.type() == CV_16S) {
                    disparity.at<int16_t>(y, x) = 0;
                } else if (disparity.type() == CV_32F) {
                    disparity.at<float>(y, x) = 0;
                }
                filteredCount++;
            }
        }
    }

    // Update statistics
    stats_.filteredPixels = filteredCount;
    stats_.filterRatio = static_cast<float>(filteredCount) / stats_.totalPixels;

    auto endTime = std::chrono::high_resolution_clock::now();
    stats_.processingTimeMs = std::chrono::duration<double, std::milli>(
        endTime - startTime).count();

    return filteredCount;
}

void BackgroundFilter::createMask(const cv::Mat& disparity, cv::Mat& mask) const {
    mask = cv::Mat::ones(disparity.size(), CV_8UC1) * 255;

    // Mark background pixels
    if (disparity.type() == CV_16S) {
        // 16-bit signed disparity (subpixel format from SGBM)
        for (int y = 0; y < disparity.rows; ++y) {
            for (int x = 0; x < disparity.cols; ++x) {
                int16_t d = disparity.at<int16_t>(y, x);
                float pixelDisparity = d / 16.0f;  // Convert from subpixel

                if (pixelDisparity > 0 && pixelDisparity < minDisparity_) {
                    mask.at<uint8_t>(y, x) = 0;  // Mark as background
                }
            }
        }
    } else if (disparity.type() == CV_32F) {
        // 32-bit float disparity
        for (int y = 0; y < disparity.rows; ++y) {
            for (int x = 0; x < disparity.cols; ++x) {
                float d = disparity.at<float>(y, x);

                if (d > 0 && d < minDisparity_) {
                    mask.at<uint8_t>(y, x) = 0;  // Mark as background
                }
            }
        }
    }
}

void BackgroundFilter::setMaxDepth(float maxDepthMm) {
    maxDepthMm_ = maxDepthMm;
    minDisparity_ = (baselineMm_ * focalLength_) / maxDepthMm_;

    std::cout << "[BackgroundFilter] Updated max depth to " << maxDepthMm_
              << "mm (min disparity: " << minDisparity_ << " pixels)\n";
}

} // namespace stereo
} // namespace unlook