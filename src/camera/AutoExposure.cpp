#include <unlook/camera/AutoExposure.hpp>
#include <unlook/core/Logger.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <iostream>

// Temporary simple logging macros for compilation
#define LOG_DEBUG(msg) std::cout << "[DEBUG] AutoExposure: " << msg << std::endl
#define LOG_INFO(msg) std::cout << "[INFO] AutoExposure: " << msg << std::endl
#define LOG_WARNING(msg) std::cout << "[WARN] AutoExposure: " << msg << std::endl
#define LOG_ERROR(msg) std::cerr << "[ERROR] AutoExposure: " << msg << std::endl

namespace unlook {
namespace camera {

AutoExposure::AutoExposure() {
    currentParams_.exposureUs = 5000.0;  // Start with 5ms
    currentParams_.analogGain = 1.5;     // Start with low gain for clean image
    currentParams_.digitalGain = 1.0;
    currentParams_.isConverged = false;
    currentParams_.currentBrightness = 0.0;
    currentParams_.targetBrightness = 80.0;  // REDUCED target for indoor
    
    LOG_DEBUG("AutoExposure constructor");
}

AutoExposure::~AutoExposure() {
    LOG_DEBUG("AutoExposure destructor");
    
    // Emergency cleanup only if shutdown wasn't called properly
    // This should normally not be needed with proper RAII
    if (initialized_.load() || running_.load()) {
        LOG_WARNING("AutoExposure destructor called without proper shutdown - performing emergency cleanup");
        shutdown();
    }
}

bool AutoExposure::initialize(const Config& config) {
    if (initialized_) {
        LOG_WARNING("AutoExposure already initialized");
        return true;
    }
    
    LOG_INFO("Initializing AutoExposure with algorithm: " + std::to_string(static_cast<int>(config.algorithm)));
    
    config_ = config;
    currentParams_.targetBrightness = config.targetBrightness;
    
    // Initialize brightness history for convergence tracking
    brightnessHistory_.reserve(config.convergenceFrames);
    
    initialized_ = true;
    LOG_INFO("AutoExposure initialized successfully");
    
    return true;
}

bool AutoExposure::initialize() {
    Config defaultConfig;  // Uses default member initializers
    return initialize(defaultConfig);
}

void AutoExposure::shutdown() {
    // Make shutdown idempotent and thread-safe
    bool expected = true;
    if (!initialized_.compare_exchange_strong(expected, false)) {
        // Already shut down
        return;
    }
    
    LOG_INFO("Shutting down AutoExposure");
    
    // Stop processing thread if running
    stop();
    
    LOG_INFO("AutoExposure shutdown complete");
}

bool AutoExposure::start() {
    if (!initialized_) {
        LOG_ERROR("AutoExposure not initialized");
        return false;
    }
    
    if (running_) {
        LOG_WARNING("AutoExposure already running");
        return true;
    }
    
    LOG_INFO("Starting AutoExposure thread");
    
    shouldStop_ = false;
    processingThread_ = std::thread(&AutoExposure::processingThread, this);
    running_ = true;
    
    LOG_INFO("AutoExposure thread started");
    return true;
}

void AutoExposure::stop() {
    // Make stop idempotent and thread-safe
    bool expected = true;
    if (!running_.compare_exchange_strong(expected, false)) {
        // Already stopped or stopping
        return;
    }
    
    LOG_INFO("Stopping AutoExposure thread");
    
    // Signal thread to stop
    shouldStop_ = true;
    frameCondition_.notify_all();
    
    // Safe thread joining - only join if this thread created it
    if (processingThread_.joinable()) {
        try {
            processingThread_.join();
            LOG_INFO("AutoExposure thread joined successfully");
        } catch (const std::system_error& e) {
            LOG_ERROR("Thread join failed: " + std::string(e.what()));
        }
    }
    
    LOG_INFO("AutoExposure thread stopped");
}

AutoExposure::ExposureParams AutoExposure::processFrame(const cv::Mat& frame) {
    if (!enabled_) {
        return currentParams_;
    }
    
    if (frame.empty() || frame.type() != CV_8UC1) {
        LOG_WARNING("Invalid frame for auto-exposure processing");
        return currentParams_;
    }
    
    // Calculate brightness based on selected algorithm
    double brightness = calculateBrightness(frame);
    
    // Update convergence tracking
    updateConvergence(brightness);
    
    // Adjust exposure parameters
    ExposureParams newParams = adjustExposure(brightness);
    
    // Update current parameters
    currentParams_ = newParams;
    
    // Increment frame counter
    framesProcessed_++;
    
    // Call update callback if set
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (updateCallback_) {
            updateCallback_(currentParams_);
        }
    }
    
    return currentParams_;
}

AutoExposure::ExposureParams AutoExposure::processStereo(const cv::Mat& leftFrame, 
                                                          const cv::Mat& rightFrame) {
    if (!enabled_) {
        return currentParams_;
    }
    
    // Calculate average brightness from both frames
    double leftBrightness = calculateBrightness(leftFrame);
    double rightBrightness = calculateBrightness(rightFrame);
    double avgBrightness = (leftBrightness + rightBrightness) / 2.0;
    
    // Use average for exposure adjustment
    currentParams_.currentBrightness = avgBrightness;
    
    // Update convergence tracking
    updateConvergence(avgBrightness);
    
    // Adjust exposure parameters
    ExposureParams newParams = adjustExposure(avgBrightness);
    
    // Update current parameters
    currentParams_ = newParams;
    
    // Call update callback if set
    {
        std::lock_guard<std::mutex> lock(callbackMutex_);
        if (updateCallback_) {
            updateCallback_(currentParams_);
        }
    }
    
    return currentParams_;
}

void AutoExposure::processingThread() {
    LOG_INFO("AutoExposure processing thread started");
    
    while (!shouldStop_) {
        std::unique_lock<std::mutex> lock(frameMutex_);
        
        // Wait for frame
        frameCondition_.wait(lock, [this] { 
            return frameAvailable_ || shouldStop_; 
        });
        
        if (shouldStop_) {
            break;
        }
        
        if (frameAvailable_ && !pendingFrame_.empty()) {
            cv::Mat frame = pendingFrame_.clone();
            frameAvailable_ = false;
            lock.unlock();
            
            // Process frame
            processFrame(frame);
        }
    }
    
    LOG_INFO("AutoExposure processing thread stopped");
}

double AutoExposure::calculateBrightness(const cv::Mat& frame) {
    switch (config_.algorithm) {
        case Algorithm::HISTOGRAM_MEAN:
            return calculateHistogramMean(frame);
            
        case Algorithm::HISTOGRAM_PERCENTILE:
            return calculateHistogramPercentile(frame, config_.targetPercentile);
            
        case Algorithm::SPOT_METERING:
            return calculateSpotMetering(frame);
            
        case Algorithm::MATRIX_METERING:
            return calculateMatrixMetering(frame);
            
        default:
            return calculateHistogramMean(frame);
    }
}

double AutoExposure::calculateHistogramMean(const cv::Mat& frame) {
    cv::Scalar meanValue = cv::mean(frame);
    return meanValue[0];
}

double AutoExposure::calculateHistogramPercentile(const cv::Mat& frame, int percentile) {
    // Calculate histogram
    cv::Mat hist;
    int histSize = 256;
    float range[] = {0, 256};
    const float* histRange = {range};
    
    cv::calcHist(&frame, 1, nullptr, cv::Mat(), hist, 1, &histSize, &histRange);
    
    // Find percentile value
    float totalPixels = frame.rows * frame.cols;
    float targetCount = totalPixels * (percentile / 100.0f);
    float cumSum = 0;
    
    for (int i = 0; i < histSize; i++) {
        cumSum += hist.at<float>(i);
        if (cumSum >= targetCount) {
            return static_cast<double>(i);
        }
    }
    
    return 255.0;
}

double AutoExposure::calculateSpotMetering(const cv::Mat& frame) {
    // Extract center region
    int centerX = static_cast<int>(frame.cols * config_.meteringRegion.x);
    int centerY = static_cast<int>(frame.rows * config_.meteringRegion.y);
    int width = static_cast<int>(frame.cols * config_.meteringRegion.width);
    int height = static_cast<int>(frame.rows * config_.meteringRegion.height);
    
    // Ensure region is within bounds
    centerX = std::max(0, std::min(centerX, frame.cols - width));
    centerY = std::max(0, std::min(centerY, frame.rows - height));
    
    cv::Rect roi(centerX, centerY, width, height);
    cv::Mat centerRegion = frame(roi);
    
    // Calculate mean of center region
    cv::Scalar meanValue = cv::mean(centerRegion);
    return meanValue[0];
}

double AutoExposure::calculateMatrixMetering(const cv::Mat& frame) {
    // Divide frame into zones (e.g., 3x3 grid)
    const int gridSize = 3;
    double totalBrightness = 0.0;
    double totalWeight = 0.0;
    
    int zoneWidth = frame.cols / gridSize;
    int zoneHeight = frame.rows / gridSize;
    
    for (int y = 0; y < gridSize; y++) {
        for (int x = 0; x < gridSize; x++) {
            cv::Rect zoneRect(x * zoneWidth, y * zoneHeight, zoneWidth, zoneHeight);
            cv::Mat zone = frame(zoneRect);
            
            double zoneBrightness = cv::mean(zone)[0];
            
            // Center zones have higher weight
            double weight = 1.0;
            if (x == 1 && y == 1) {
                weight = 2.0;  // Center zone
            }
            
            totalBrightness += zoneBrightness * weight;
            totalWeight += weight;
        }
    }
    
    return totalBrightness / totalWeight;
}

AutoExposure::ExposureParams AutoExposure::adjustExposure(double currentBrightness) {
    ExposureParams newParams = currentParams_;
    newParams.currentBrightness = currentBrightness;
    
    // Calculate brightness error
    double error = config_.targetBrightness - currentBrightness;
    
    // Skip adjustment if within threshold
    if (std::abs(error) < config_.convergenceThreshold) {
        newParams.isConverged = true;
        return newParams;
    }
    
    // Calculate adjustment factor - IMPROVED for faster convergence
    // Use larger factor when far from target, smaller when close
    double errorRatio = std::abs(error) / 255.0;
    double dynamicSpeed = config_.adjustmentSpeed * (1.0 + errorRatio);  // Adaptive speed
    double adjustmentFactor = 1.0 + (error / 255.0) * dynamicSpeed;
    
    // IMPROVED STRATEGY: Prioritize exposure time up to 10ms, then gain
    if (error > 0) {  // Too dark - need more light
        // First increase exposure up to 10ms for better SNR
        if (currentParams_.exposureUs < 10000) {
            double newExposure = currentParams_.exposureUs * adjustmentFactor;
            newParams.exposureUs = std::min(newExposure, 10000.0);
        }
        // Then increase gain up to 4x
        else if (currentParams_.analogGain < 4.0) {
            double newGain = currentParams_.analogGain * adjustmentFactor;
            newParams.analogGain = std::min(newGain, 4.0);
        }
        // Then continue with exposure up to max
        else if (currentParams_.exposureUs < config_.maxExposureUs) {
            double newExposure = currentParams_.exposureUs * adjustmentFactor;
            newParams.exposureUs = std::min(newExposure, config_.maxExposureUs);
        }
        // Finally increase gain to max if needed
        else if (currentParams_.analogGain < config_.maxGain) {
            double newGain = currentParams_.analogGain * (1.0 + (error / 255.0) * 0.1);  // Slower gain increase
            newParams.analogGain = std::min(newGain, config_.maxGain);
        }
    } else {  // Too bright - reduce light
        // First reduce gain if above 1.5x
        if (currentParams_.analogGain > 1.5) {
            double newGain = currentParams_.analogGain * adjustmentFactor;
            newParams.analogGain = std::max(newGain, 1.0);
        }
        // Then reduce exposure time
        else {
            double newExposure = currentParams_.exposureUs * adjustmentFactor;
            newParams.exposureUs = std::max(newExposure, config_.minExposureUs);
        }
    }
    
    return newParams;
}

double AutoExposure::smoothAdjustment(double current, double target, double speed) {
    // Exponential smoothing
    return current + (target - current) * speed;
}

void AutoExposure::updateConvergence(double brightness) {
    brightnessHistory_.push_back(brightness);
    
    if (brightnessHistory_.size() > static_cast<size_t>(config_.convergenceFrames)) {
        brightnessHistory_.erase(brightnessHistory_.begin());
    }
    
    if (brightnessHistory_.size() == static_cast<size_t>(config_.convergenceFrames)) {
        // Check if brightness is stable
        double minBrightness = *std::min_element(brightnessHistory_.begin(), 
                                                  brightnessHistory_.end());
        double maxBrightness = *std::max_element(brightnessHistory_.begin(), 
                                                  brightnessHistory_.end());
        
        if ((maxBrightness - minBrightness) < config_.convergenceThreshold) {
            convergenceCounter_++;
            if (convergenceCounter_ >= config_.convergenceFrames) {
                converged_ = true;
            }
        } else {
            convergenceCounter_ = 0;
            converged_ = false;
        }
    }
}

void AutoExposure::setUpdateCallback(UpdateCallback callback) {
    std::lock_guard<std::mutex> lock(callbackMutex_);
    updateCallback_ = callback;
}

AutoExposure::Config AutoExposure::getConfig() const {
    return config_;
}

void AutoExposure::setConfig(const Config& config) {
    config_ = config;
    currentParams_.targetBrightness = config.targetBrightness;
    resetConvergence();
}

AutoExposure::ExposureParams AutoExposure::getCurrentParams() const {
    return currentParams_;
}

void AutoExposure::resetConvergence() {
    brightnessHistory_.clear();
    convergenceCounter_ = 0;
    converged_ = false;
    currentParams_.isConverged = false;
}

void AutoExposure::setTargetBrightness(int brightness) {
    config_.targetBrightness = std::clamp(brightness, 0, 255);
    currentParams_.targetBrightness = config_.targetBrightness;
    resetConvergence();
}

void AutoExposure::setMeteringRegion(const cv::Rect2d& region) {
    config_.meteringRegion = region;
}

void AutoExposure::setExposureLimits(double minUs, double maxUs) {
    config_.minExposureUs = minUs;
    config_.maxExposureUs = maxUs;
}

void AutoExposure::setGainLimits(double minGain, double maxGain) {
    config_.minGain = minGain;
    config_.maxGain = maxGain;
}

void AutoExposure::setAdjustmentSpeed(double speed) {
    config_.adjustmentSpeed = std::clamp(speed, 0.0, 1.0);
}

} // namespace camera
} // namespace unlook