/**
 * @file TemporalFilter.cpp
 * @brief Implementation of temporal filter for multi-frame smoothing
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md specification.
 */

#include "unlook/stereo/TemporalFilter.hpp"
#include "unlook/stereo/DebugOutputManager.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cmath>

namespace unlook {
namespace stereo {

using namespace std::chrono;

// ========== CONSTRUCTOR / DESTRUCTOR ==========

TemporalFilter::TemporalFilter(core::Logger* logger)
    : logger_(logger ? logger : &core::Logger::getInstance())
    , frameCounter_(0)
    , lastConsistency_(0)
{
    if (logger_) {
        logger_->info("TemporalFilter initialized");
    }
}

TemporalFilter::~TemporalFilter() = default;

// ========== CONFIGURATION ==========

void TemporalFilter::setConfig(const Config& config) {
    config_ = config;

    // Clear history if history size changed
    if (frameHistory_.size() > static_cast<size_t>(config_.historySize)) {
        while (frameHistory_.size() > static_cast<size_t>(config_.historySize)) {
            frameHistory_.pop_front();
        }
    }

    if (logger_) {
        logger_->info("TemporalFilter configured:");
        logger_->info("  Alpha: " + std::to_string(config_.alpha));
        logger_->info("  Delta threshold: " + std::to_string(config_.deltaThreshold) + " mm");
        logger_->info("  Persistence frames: " + std::to_string(config_.persistenceFrames));
        logger_->info("  History size: " + std::to_string(config_.historySize));
        logger_->info("  Adaptive alpha: " + std::string(config_.adaptiveAlpha ? "yes" : "no"));
    }
}

// ========== RESET ==========

void TemporalFilter::reset() {
    frameHistory_.clear();
    lastFiltered_ = cv::Mat();
    persistenceMap_ = cv::Mat();
    frameCounter_ = 0;
    lastConsistency_ = 0;

    if (logger_) {
        logger_->info("TemporalFilter reset - history cleared");
    }
}

// ========== MAIN FILTER FUNCTION ==========

TemporalFilter::Result TemporalFilter::filter(const cv::Mat& depthMap, const cv::Mat& confidence) {
    Result result;
    auto startTime = high_resolution_clock::now();

    if (depthMap.empty()) {
        result.success = false;
        result.errorMessage = "Empty depth map";
        return result;
    }

    if (depthMap.type() != CV_32F) {
        result.success = false;
        result.errorMessage = "Depth map must be CV_32F";
        return result;
    }

    try {
        frameCounter_++;

        // Initialize persistence map if needed
        if (persistenceMap_.empty()) {
            persistenceMap_ = cv::Mat::zeros(depthMap.size(), CV_8U);
        }

        // Create frame data
        FrameData currentFrame;
        currentFrame.depth = depthMap.clone();
        currentFrame.confidence = confidence.empty() ?
            cv::Mat::ones(depthMap.size(), CV_8U) * 255 : confidence.clone();
        currentFrame.timestamp = steady_clock::now();
        currentFrame.frameNumber = frameCounter_;

        // First frame - no temporal filtering
        if (lastFiltered_.empty()) {
            result.filtered = depthMap.clone();
            lastFiltered_ = result.filtered;
            result.motionMask = cv::Mat::zeros(depthMap.size(), CV_8U);
            result.motionPixels = 0;
            result.temporalConsistency = 1.0f;
        } else {
            // Detect motion between frames
            detectMotion(depthMap, lastFiltered_, result.motionMask, config_.deltaThreshold);
            result.motionPixels = cv::countNonZero(result.motionMask);

            // Apply IIR filtering
            result.filtered = cv::Mat(depthMap.size(), CV_32F);

            for (int y = 0; y < depthMap.rows; ++y) {
                for (int x = 0; x < depthMap.cols; ++x) {
                    float currentVal = depthMap.at<float>(y, x);
                    float previousVal = lastFiltered_.at<float>(y, x);

                    // Check for invalid values
                    if (std::isnan(currentVal) || currentVal <= 0) {
                        // Use persistence to maintain last valid value
                        if (persistenceMap_.at<uint8_t>(y, x) > 0) {
                            result.filtered.at<float>(y, x) = previousVal;
                            persistenceMap_.at<uint8_t>(y, x)--;
                        } else {
                            result.filtered.at<float>(y, x) = currentVal; // Keep as invalid
                        }
                        continue;
                    }

                    // Reset persistence for valid values
                    persistenceMap_.at<uint8_t>(y, x) = config_.persistenceFrames;

                    // Check for motion/jump
                    if (result.motionMask.at<uint8_t>(y, x) > 0) {
                        // Motion detected - reset to current value
                        result.filtered.at<float>(y, x) = currentVal;
                    } else {
                        // No motion - apply temporal smoothing
                        float alpha = config_.alpha;

                        // Adaptive alpha based on confidence
                        if (config_.adaptiveAlpha && !confidence.empty()) {
                            uint8_t conf = confidence.at<uint8_t>(y, x);
                            alpha = calculateAdaptiveAlpha(alpha, conf);
                        }

                        // IIR filter: output = alpha * previous + (1 - alpha) * current
                        if (!std::isnan(previousVal) && previousVal > 0) {
                            result.filtered.at<float>(y, x) = alpha * previousVal +
                                                             (1.0f - alpha) * currentVal;
                        } else {
                            // No valid previous value
                            result.filtered.at<float>(y, x) = currentVal;
                        }
                    }
                }
            }

            // Calculate temporal consistency
            result.temporalConsistency = calculateConsistency(depthMap, lastFiltered_);
            lastConsistency_ = result.temporalConsistency;

            // Update last filtered
            lastFiltered_ = result.filtered.clone();
        }

        // Add to history
        frameHistory_.push_back(currentFrame);

        // Limit history size
        while (frameHistory_.size() > static_cast<size_t>(config_.historySize)) {
            frameHistory_.pop_front();
        }

        // Count valid pixels
        result.validPixels = 0;
        for (int y = 0; y < result.filtered.rows; ++y) {
            for (int x = 0; x < result.filtered.cols; ++x) {
                float val = result.filtered.at<float>(y, x);
                if (!std::isnan(val) && val > 0) {
                    result.validPixels++;
                }
            }
        }

        result.processingTime = duration_cast<microseconds>(high_resolution_clock::now() - startTime);
        result.success = true;

        if (logger_) {
            logger_->info("Temporal filter applied (frame " + std::to_string(frameCounter_) + "):");
            logger_->info("  Motion pixels: " + std::to_string(result.motionPixels) +
                         " (" + std::to_string((100.0f * result.motionPixels) /
                         (depthMap.rows * depthMap.cols)) + "%)");
            logger_->info("  Temporal consistency: " + std::to_string(result.temporalConsistency));
            logger_->info("  Valid pixels: " + std::to_string(result.validPixels));
            logger_->info("  Processing time: " + std::to_string(result.processingTime.count()) + " µs");
        }

        // Save debug output
        if (DebugOutputManager::getInstance().isEnabled()) {
            DebugOutputManager::getInstance().saveDebugImage("temporal_motion_mask", result.motionMask);

            // Create temporal difference visualization
            cv::Mat diff;
            cv::absdiff(depthMap, result.filtered, diff);
            cv::normalize(diff, diff, 0, 255, cv::NORM_MINMAX);
            diff.convertTo(diff, CV_8U);
            cv::applyColorMap(diff, diff, cv::COLORMAP_JET);
            DebugOutputManager::getInstance().saveDebugImage("temporal_diff", diff);
        }

    } catch (const cv::Exception& e) {
        result.success = false;
        result.errorMessage = "OpenCV exception: " + std::string(e.what());
        if (logger_) logger_->error(result.errorMessage);
    }

    return result;
}

// ========== IIR FILTER ==========

void TemporalFilter::applyIIRFilter(const cv::Mat& current, const cv::Mat& previous,
                                    cv::Mat& output, float alpha) {
    output = cv::Mat(current.size(), current.type());

    for (int y = 0; y < current.rows; ++y) {
        const float* currRow = current.ptr<float>(y);
        const float* prevRow = previous.ptr<float>(y);
        float* outRow = output.ptr<float>(y);

        for (int x = 0; x < current.cols; ++x) {
            float currVal = currRow[x];
            float prevVal = prevRow[x];

            if (std::isnan(currVal) || currVal <= 0) {
                outRow[x] = currVal;
            } else if (std::isnan(prevVal) || prevVal <= 0) {
                outRow[x] = currVal;
            } else {
                outRow[x] = alpha * prevVal + (1.0f - alpha) * currVal;
            }
        }
    }
}

// ========== MOTION DETECTION ==========

void TemporalFilter::detectMotion(const cv::Mat& current, const cv::Mat& previous,
                                  cv::Mat& motionMask, float threshold) {
    motionMask = cv::Mat::zeros(current.size(), CV_8U);

    for (int y = 0; y < current.rows; ++y) {
        const float* currRow = current.ptr<float>(y);
        const float* prevRow = previous.ptr<float>(y);
        uint8_t* maskRow = motionMask.ptr<uint8_t>(y);

        for (int x = 0; x < current.cols; ++x) {
            float currVal = currRow[x];
            float prevVal = prevRow[x];

            // Check if both values are valid
            if (!std::isnan(currVal) && currVal > 0 &&
                !std::isnan(prevVal) && prevVal > 0) {

                float diff = std::abs(currVal - prevVal);

                // Check for motion/jump
                if (diff > threshold) {
                    maskRow[x] = 255;
                }
            }
        }
    }

    // Apply morphological operations to clean up motion mask
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(motionMask, motionMask, cv::MORPH_CLOSE, kernel);
}

// ========== PERSISTENCE UPDATE ==========

void TemporalFilter::updatePersistence(const cv::Mat& depthMap, const cv::Mat& validMask) {
    if (persistenceMap_.empty()) {
        persistenceMap_ = cv::Mat::zeros(depthMap.size(), CV_8U);
    }

    for (int y = 0; y < depthMap.rows; ++y) {
        const float* depthRow = depthMap.ptr<float>(y);
        uint8_t* persRow = persistenceMap_.ptr<uint8_t>(y);

        for (int x = 0; x < depthMap.cols; ++x) {
            if (!std::isnan(depthRow[x]) && depthRow[x] > 0) {
                // Reset persistence for valid values
                persRow[x] = config_.persistenceFrames;
            } else if (persRow[x] > 0) {
                // Decrement persistence for invalid values
                persRow[x]--;
            }
        }
    }
}

// ========== ADAPTIVE ALPHA ==========

float TemporalFilter::calculateAdaptiveAlpha(float baseAlpha, uint8_t confidence) {
    // Lower confidence -> less temporal smoothing (trust current frame more)
    // Higher confidence -> more temporal smoothing (trust history more)

    float confNorm = confidence / 255.0f;

    // Scale alpha based on confidence
    // Low confidence (0) -> alpha = baseAlpha * 0.5 (less smoothing)
    // High confidence (255) -> alpha = baseAlpha * 1.5 (more smoothing)

    float scaleFactor = 0.5f + confNorm;
    float adaptedAlpha = baseAlpha * scaleFactor;

    // Clamp to valid range
    return std::max(0.1f, std::min(0.9f, adaptedAlpha));
}

// ========== TEMPORAL CONSISTENCY ==========

float TemporalFilter::calculateConsistency(const cv::Mat& current, const cv::Mat& previous) {
    int consistentPixels = 0;
    int totalValidPixels = 0;

    for (int y = 0; y < current.rows; ++y) {
        const float* currRow = current.ptr<float>(y);
        const float* prevRow = previous.ptr<float>(y);

        for (int x = 0; x < current.cols; ++x) {
            float currVal = currRow[x];
            float prevVal = prevRow[x];

            if (!std::isnan(currVal) && currVal > 0 &&
                !std::isnan(prevVal) && prevVal > 0) {

                totalValidPixels++;

                float diff = std::abs(currVal - prevVal);
                float relDiff = diff / prevVal;

                // Consider consistent if relative difference < 5%
                if (relDiff < 0.05f) {
                    consistentPixels++;
                }
            }
        }
    }

    return (totalValidPixels > 0) ?
           static_cast<float>(consistentPixels) / totalValidPixels : 0;
}

} // namespace stereo
} // namespace unlook