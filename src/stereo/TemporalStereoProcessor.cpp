#include "unlook/stereo/TemporalStereoProcessor.hpp"
#include "unlook/stereo/SGBMStereoMatcher.hpp"
#include "unlook/core/Logger.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/photo.hpp>
#include <filesystem>
#include <chrono>
#include <deque>
#include <cmath>

namespace unlook {
namespace stereo {

using namespace unlook::core;

TemporalStereoProcessor::TemporalStereoProcessor() {
    UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Creating temporal stereo processor for VCSEL-based depth sensing";
}

TemporalStereoProcessor::~TemporalStereoProcessor() {
    UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Destroying temporal stereo processor";
}

bool TemporalStereoProcessor::initialize(
    std::shared_ptr<hardware::AS1170DualVCSELController> dualVCSEL,
    std::shared_ptr<calibration::CalibrationManager> calibManager,
    const TemporalStereoConfig& config) {

    if (!dualVCSEL || !calibManager) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid dual VCSEL controller or calibration manager";
        return false;
    }

    if (!dualVCSEL->isInitialized()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Dual VCSEL controller not initialized";
        return false;
    }

    if (!calibManager->isCalibrationValid()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Calibration manager not calibrated";
        return false;
    }

    if (!validateConfig(config)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid configuration parameters";
        return false;
    }

    dualVCSEL_ = dualVCSEL;
    calibManager_ = calibManager;
    config_ = config;

    // Create stereo matcher with VCSEL-optimized parameters
    stereoMatcher_ = StereoMatcher::create(StereoAlgorithm::SGBM);
    if (!stereoMatcher_) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to create SGBM stereo matcher";
        return false;
    }

    // Set VCSEL-optimized parameters
    if (!stereoMatcher_->setParameters(config_.sgbmParams)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to set stereo matching parameters";
        return false;
    }

    // Create debug output directory if needed
    if (config_.saveIntermediateImages) {
        std::filesystem::create_directories(config_.debugOutputPath);
        debugMode_ = true;
        debugOutputPath_ = config_.debugOutputPath;
    }

    initialized_ = true;

    UNLOOK_LOG_INFO("TemporalStereoProcessor")
        << "Initialized with blockSize=" << config_.sgbmParams.blockSize
        << ", numDisparities=" << config_.sgbmParams.numDisparities
        << ", noiseThreshold=" << config_.isolationParams.noiseThreshold;

    return true;
}

bool TemporalStereoProcessor::processTemporalStereo(
    const hardware::AS1170DualVCSELController::TripleFrameCapture& frames,
    TemporalStereoResult& result) {

    if (!initialized_) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Processor not initialized";
        return false;
    }

    if (!frames.is_valid) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid triple frame capture";
        return false;
    }

    auto startTime = std::chrono::high_resolution_clock::now();

    // Step 1: Isolate patterns from both VCSELs
    auto isolationStart = std::chrono::high_resolution_clock::now();

    // Isolate VCSEL1 patterns (left perspective projection)
    float variance1Left = isolateVCSELPattern(
        frames.frame_vcsel1_left,
        frames.frame_ambient_left,
        result.vcsel1PatternLeft
    );

    float variance1Right = isolateVCSELPattern(
        frames.frame_vcsel1_right,
        frames.frame_ambient_right,
        result.vcsel1PatternRight
    );

    // Isolate VCSEL2 patterns (right perspective projection)
    float variance2Left = isolateVCSELPattern(
        frames.frame_vcsel2_left,
        frames.frame_ambient_left,
        result.vcsel2PatternLeft
    );

    float variance2Right = isolateVCSELPattern(
        frames.frame_vcsel2_right,
        frames.frame_ambient_right,
        result.vcsel2PatternRight
    );

    auto isolationEnd = std::chrono::high_resolution_clock::now();
    result.isolationTimeMs = std::chrono::duration<double, std::milli>(isolationEnd - isolationStart).count();

    // Step 2: Combine patterns from both VCSELs
    float coverageLeft = combinePatterns(
        result.vcsel1PatternLeft,
        result.vcsel2PatternLeft,
        result.isolatedPatternLeft
    );

    float coverageRight = combinePatterns(
        result.vcsel1PatternRight,
        result.vcsel2PatternRight,
        result.isolatedPatternRight
    );

    // Store quality metrics
    result.patternVariance = (variance1Left + variance1Right + variance2Left + variance2Right) / 4.0f;
    result.coverageRatio = (coverageLeft + coverageRight) / 2.0f;

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern metrics: variance=" << result.patternVariance << ", coverage=" << (result.coverageRatio * 100) << "%";

    // Validate pattern quality
    if (result.patternVariance < config_.isolationParams.minPatternVariance) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Pattern variance too low: " << result.patternVariance << " < " << config_.isolationParams.minPatternVariance;
    }

    if (result.coverageRatio < config_.isolationParams.minCoverage ||
        result.coverageRatio > config_.isolationParams.maxCoverage) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Coverage ratio out of range: " << (result.coverageRatio * 100) << "% not in [" << (config_.isolationParams.minCoverage * 100) << "%, " << (config_.isolationParams.maxCoverage * 100) << "%]";
    }

    // Step 3: Generate dual depth maps from individual VCSEL patterns
    auto matchingStart = std::chrono::high_resolution_clock::now();

    cv::Mat depth1, depth2, confidence1, confidence2;

    // Generate depth maps from both VCSEL patterns independently
    if (!generateDualDepthMaps(
            result.vcsel1PatternLeft, result.vcsel1PatternRight,
            result.vcsel2PatternLeft, result.vcsel2PatternRight,
            depth1, depth2, confidence1, confidence2)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to generate dual depth maps";
        return false;
    }

    // Step 4: Intelligently fuse the dual depth maps
    cv::Mat fusedDepth, fusedConfidence;
    if (!fuseDualDepthMaps(depth1, depth2, confidence1, confidence2,
                           fusedDepth, fusedConfidence)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to fuse dual depth maps";
        return false;
    }

    // Also compute depth from combined patterns for comparison
    cv::Mat rectifiedLeft, rectifiedRight;
    calibManager_->rectifyImages(result.isolatedPatternLeft, result.isolatedPatternRight,
                                  rectifiedLeft, rectifiedRight);

    // Compute disparity from combined patterns
    if (config_.computeConfidence) {
        if (!stereoMatcher_->computeDisparityWithConfidence(
                rectifiedLeft, rectifiedRight,
                result.disparityMap, result.confidenceMap)) {
            UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to compute disparity with confidence";
            return false;
        }

        // Use the better of fused or combined confidence
        cv::Mat combinedConfidence;
        cv::max(result.confidenceMap, fusedConfidence, combinedConfidence);
        result.confidenceMap = combinedConfidence;

        // Compute average confidence
        cv::Scalar meanConf = cv::mean(result.confidenceMap);
        result.avgConfidence = static_cast<float>(meanConf[0]);
    } else {
        if (!stereoMatcher_->computeDisparity(
                rectifiedLeft, rectifiedRight,
                result.disparityMap)) {
            UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to compute disparity";
            return false;
        }
    }

    // Apply WLS filtering if enabled
    if (config_.useWLSFilter) {
        stereoMatcher_->applyPostProcessing(result.disparityMap, rectifiedLeft);
    }

    auto matchingEnd = std::chrono::high_resolution_clock::now();
    result.matchingTimeMs = std::chrono::duration<double, std::milli>(matchingEnd - matchingStart).count();

    // Step 5: Convert disparity to depth and use the best result
    cv::Mat combinedDepth;
    if (!disparityToDepth(result.disparityMap, combinedDepth)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to convert disparity to depth";
        return false;
    }

    // Choose the best depth map based on confidence and coverage
    int validFused = cv::countNonZero(fusedDepth > 0);
    int validCombined = cv::countNonZero(combinedDepth > 0);

    if (validFused > validCombined) {
        result.depthMap = fusedDepth;
        UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Using fused dual depth (" << validFused << ">" << validCombined << " valid pixels)";
    } else {
        result.depthMap = combinedDepth;
        UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Using combined pattern depth (" << validCombined << ">" << validFused << " valid pixels)";
    }

    // Count valid pixels
    result.validPixelCount = cv::countNonZero(result.depthMap > 0);

    // Step 5: Apply temporal filtering if enabled
    if (config_.isolationParams.enableTemporalFilter) {
        applyTemporalFilter(result.depthMap);
    }

    // Step 6: Validate depth topologically if enabled
    if (config_.validateDepth) {
        result.isTopologicallyCorrect = validateDepthTopology(
            result.depthMap, result.validationMessage);
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    result.totalTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    // Update statistics
    updateStatistics(result);

    // Save debug images if enabled
    if (debugMode_) {
        saveDebugImages(result);
    }

    UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Processing complete: " << result.totalTimeMs << "ms total (isolation: " << result.isolationTimeMs << "ms, matching: " << result.matchingTimeMs << "ms)";

    return result.isValid();
}

bool TemporalStereoProcessor::captureAndProcess(TemporalStereoResult& result) {
    if (!initialized_) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Processor not initialized";
        return false;
    }

    // Capture triple frame sequence
    hardware::AS1170DualVCSELController::TripleFrameCapture frames;
    if (!dualVCSEL_->captureTemporalSequence(frames)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Failed to capture temporal sequence";
        return false;
    }

    // Process the captured frames
    return processTemporalStereo(frames, result);
}

float TemporalStereoProcessor::isolateVCSELPattern(
    const cv::Mat& vcselFrame,
    const cv::Mat& ambientFrame,
    cv::Mat& isolatedPattern) {

    if (vcselFrame.size() != ambientFrame.size() ||
        vcselFrame.type() != ambientFrame.type()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Frame size or type mismatch";
        return 0.0f;
    }

    // Convert to float for precise subtraction
    cv::Mat vcselFloat, ambientFloat;
    vcselFrame.convertTo(vcselFloat, CV_32F);
    ambientFrame.convertTo(ambientFloat, CV_32F);

    // Advanced pattern isolation: pure_pattern = vcsel - ambient
    cv::Mat patternFloat;
    cv::subtract(vcselFloat, ambientFloat, patternFloat);

    // Apply adaptive noise suppression based on ambient variance
    cv::Scalar ambientMean, ambientStdDev;
    cv::meanStdDev(ambientFloat, ambientMean, ambientStdDev);
    float adaptiveThreshold = std::max(
        static_cast<float>(config_.isolationParams.noiseThreshold),
        static_cast<float>(ambientStdDev[0] * 2.0)  // 2-sigma threshold
    );

    // Apply adaptive threshold to remove ambient noise
    cv::threshold(patternFloat, patternFloat,
                  adaptiveThreshold, 0, cv::THRESH_TOZERO);

    // Normalize pattern intensity for consistent processing
    double minVal, maxVal;
    cv::minMaxLoc(patternFloat, &minVal, &maxVal);
    if (maxVal > minVal) {
        patternFloat = (patternFloat - minVal) * (255.0 / (maxVal - minVal));
    }

    // Convert back to 8-bit
    patternFloat.convertTo(isolatedPattern, CV_8U);

    // Apply contrast enhancement if enabled
    if (config_.isolationParams.enhanceContrast) {
        enhancePatternContrast(isolatedPattern);
    }

    // Apply morphological cleaning if enabled
    if (config_.isolationParams.applyMorphology) {
        applyMorphologicalCleaning(isolatedPattern);
    }

    // Compute pattern quality metrics
    cv::Scalar mean, stddev;
    cv::meanStdDev(isolatedPattern, mean, stddev);
    float variance = static_cast<float>(stddev[0] * stddev[0]);

    // Compute pattern enhancement factor (pattern variance / ambient variance)
    float enhancementFactor = variance / (ambientStdDev[0] * ambientStdDev[0] + 1e-6f);

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern isolation: variance=" << variance << ", mean=" << mean[0] << ", enhancement=" << enhancementFactor << "x, adaptive_threshold=" << adaptiveThreshold;

    return variance;
}

float TemporalStereoProcessor::combinePatterns(
    const cv::Mat& pattern1,
    const cv::Mat& pattern2,
    cv::Mat& combined) {

    if (pattern1.size() != pattern2.size() ||
        pattern1.type() != pattern2.type()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Pattern size or type mismatch";
        return 0.0f;
    }

    // Geometric-aware pattern fusion based on VCSEL positions
    // VCSEL1 is 2cm from LEFT camera → better pattern on left side of image
    // VCSEL2 is 2cm from RIGHT camera → better pattern on right side of image

    int width = pattern1.cols;
    int height = pattern1.rows;

    // Create weight maps based on geometric configuration
    cv::Mat weight1(height, width, CV_32F);
    cv::Mat weight2(height, width, CV_32F);

    // Create horizontal gradient for geometric weighting
    // Left side favors VCSEL1, right side favors VCSEL2
    for (int x = 0; x < width; ++x) {
        float w1 = 1.0f - (static_cast<float>(x) / width);  // Decreases left to right
        float w2 = static_cast<float>(x) / width;           // Increases left to right

        // Apply smooth transition with sigmoid-like curve
        w1 = 0.5f * (1.0f + std::tanh(3.0f * (w1 - 0.5f)));
        w2 = 0.5f * (1.0f + std::tanh(3.0f * (w2 - 0.5f)));

        // Normalize weights
        float sum = w1 + w2;
        if (sum > 0) {
            w1 /= sum;
            w2 /= sum;
        }

        weight1.col(x) = w1 * config_.isolationParams.vcsel1Weight;
        weight2.col(x) = w2 * config_.isolationParams.vcsel2Weight;
    }

    // Convert patterns to float for weighted combination
    cv::Mat pattern1Float, pattern2Float, combinedFloat;
    pattern1.convertTo(pattern1Float, CV_32F);
    pattern2.convertTo(pattern2Float, CV_32F);

    // Apply geometric-aware weighted combination
    combinedFloat = cv::Mat::zeros(height, width, CV_32F);
    cv::multiply(pattern1Float, weight1, pattern1Float);
    cv::multiply(pattern2Float, weight2, pattern2Float);
    cv::add(pattern1Float, pattern2Float, combinedFloat);

    // Also create max fusion for areas where one pattern is much stronger
    cv::Mat maxPattern;
    cv::max(pattern1, pattern2, maxPattern);

    // Blend weighted and max fusion based on pattern strength difference
    cv::Mat diff;
    cv::absdiff(pattern1, pattern2, diff);
    cv::Scalar meanDiff = cv::mean(diff);
    float blendFactor = std::min(1.0f, static_cast<float>(meanDiff[0] / 50.0));  // More max fusion when patterns differ

    combinedFloat = combinedFloat * (1.0f - blendFactor) + maxPattern * blendFactor;
    combinedFloat.convertTo(combined, CV_8U);

    // Compute coverage metrics
    int nonZeroPixels1 = cv::countNonZero(pattern1 > config_.isolationParams.noiseThreshold);
    int nonZeroPixels2 = cv::countNonZero(pattern2 > config_.isolationParams.noiseThreshold);
    int nonZeroCombined = cv::countNonZero(combined > config_.isolationParams.noiseThreshold);
    int totalPixels = combined.rows * combined.cols;

    float coverage1 = static_cast<float>(nonZeroPixels1) / totalPixels;
    float coverage2 = static_cast<float>(nonZeroPixels2) / totalPixels;
    float coverageCombined = static_cast<float>(nonZeroCombined) / totalPixels;

    // Calculate coverage improvement
    float maxIndividualCoverage = std::max(coverage1, coverage2);
    float coverageImprovement = (coverageCombined - maxIndividualCoverage) / maxIndividualCoverage;

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern combination: VCSEL1=" << (coverage1 * 100) << "%, VCSEL2=" << (coverage2 * 100) << "%, combined=" << (coverageCombined * 100) << "%, improvement=" << (coverageImprovement * 100) << "%";

    return coverageCombined;
}

void TemporalStereoProcessor::enhancePatternContrast(cv::Mat& pattern) {
    // Apply contrast enhancement: output = alpha * input + beta
    pattern.convertTo(pattern, -1,
                      config_.isolationParams.contrastAlpha,
                      config_.isolationParams.contrastBeta);

    // Ensure values stay in valid range
    cv::threshold(pattern, pattern, 255, 255, cv::THRESH_TRUNC);
    cv::threshold(pattern, pattern, 0, 0, cv::THRESH_TOZERO);
}

void TemporalStereoProcessor::applyMorphologicalCleaning(cv::Mat& pattern) {
    // Create morphological kernel
    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(config_.isolationParams.morphKernelSize,
                 config_.isolationParams.morphKernelSize)
    );

    // Apply opening to remove small noise
    cv::morphologyEx(pattern, pattern, cv::MORPH_OPEN, kernel);

    // Apply closing to fill small gaps in patterns
    cv::morphologyEx(pattern, pattern, cv::MORPH_CLOSE, kernel);
}

void TemporalStereoProcessor::computePatternMetrics(
    const cv::Mat& pattern,
    float& variance,
    float& coverage) {

    // Compute variance
    cv::Scalar mean, stddev;
    cv::meanStdDev(pattern, mean, stddev);
    variance = static_cast<float>(stddev[0] * stddev[0]);

    // Compute coverage
    int nonZeroPixels = cv::countNonZero(pattern > config_.isolationParams.noiseThreshold);
    int totalPixels = pattern.rows * pattern.cols;
    coverage = static_cast<float>(nonZeroPixels) / totalPixels;
}

bool TemporalStereoProcessor::disparityToDepth(
    const cv::Mat& disparity,
    cv::Mat& depth) {

    if (disparity.empty()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Empty disparity map";
        return false;
    }

    // Get calibration data
    auto calibData = calibManager_->getCalibrationData();
    if (calibData.Q.empty()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Missing Q matrix in calibration";
        return false;
    }

    // Use StereoMatcher's static method for conversion
    return StereoMatcher::disparityToDepth(
        disparity, calibData.Q, depth, true);
}

void TemporalStereoProcessor::applyTemporalFilter(cv::Mat& depth) {
    std::lock_guard<std::mutex> lock(temporalMutex_);

    // Add current depth to temporal buffer
    temporalBuffer_.push_back(depth.clone());

    // Maintain window size
    while (temporalBuffer_.size() > static_cast<size_t>(config_.isolationParams.temporalWindowSize)) {
        temporalBuffer_.pop_front();
    }

    if (temporalBuffer_.size() < 2) {
        return; // Not enough frames for filtering
    }

    // Apply weighted average with decay
    cv::Mat filtered = cv::Mat::zeros(depth.size(), CV_32F);
    float totalWeight = 0.0f;
    float weight = 1.0f;

    for (auto it = temporalBuffer_.rbegin(); it != temporalBuffer_.rend(); ++it) {
        cv::Mat temp;
        it->convertTo(temp, CV_32F);
        filtered += temp * weight;
        totalWeight += weight;
        weight *= config_.isolationParams.temporalDecay;
    }

    filtered /= totalWeight;
    filtered.convertTo(depth, depth.type());
}

bool TemporalStereoProcessor::validateDepthTopology(
    const cv::Mat& depthMap,
    std::string& message) {

    if (depthMap.empty()) {
        message = "Empty depth map";
        return false;
    }

    // Sample depth at multiple points to verify topology
    // For a typical scene: background > mid-ground > foreground

    int h = depthMap.rows;
    int w = depthMap.cols;

    // Sample regions
    cv::Rect topRegion(w/4, 0, w/2, h/4);
    cv::Rect middleRegion(w/4, h/3, w/2, h/3);
    cv::Rect bottomRegion(w/4, 2*h/3, w/2, h/3);

    // Compute average depths
    double topDepth = cv::mean(depthMap(topRegion))[0];
    double middleDepth = cv::mean(depthMap(middleRegion))[0];
    double bottomDepth = cv::mean(depthMap(bottomRegion))[0];

    // Basic topological check: typically bottom (closer) < middle < top (farther)
    // This is a simplified check - adjust based on actual scene expectations
    bool topologyCorrect = true;
    std::stringstream ss;

    if (topDepth > 0 && middleDepth > 0 && bottomDepth > 0) {
        ss << "Depth topology: top=" << topDepth
           << "mm, middle=" << middleDepth
           << "mm, bottom=" << bottomDepth << "mm";

        // For handheld scanner, closer objects are typically at bottom of image
        if (bottomDepth > topDepth * 1.5) {
            ss << " [WARNING: Unexpected topology - bottom farther than top]";
            topologyCorrect = false;
        }
    } else {
        ss << "Insufficient valid depth data for topology validation";
        topologyCorrect = false;
    }

    message = ss.str();
    return topologyCorrect;
}

bool TemporalStereoProcessor::updateConfig(const TemporalStereoConfig& config) {
    if (!validateConfig(config)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid configuration";
        return false;
    }

    std::lock_guard<std::mutex> lock(configMutex_);
    config_ = config;

    // Update stereo matcher parameters if initialized
    if (stereoMatcher_) {
        stereoMatcher_->setParameters(config_.sgbmParams);
    }

    // Update debug settings
    if (config_.saveIntermediateImages) {
        std::filesystem::create_directories(config_.debugOutputPath);
        debugMode_ = true;
        debugOutputPath_ = config_.debugOutputPath;
    } else {
        debugMode_ = false;
    }

    // Clear temporal buffer on config change
    {
        std::lock_guard<std::mutex> tempLock(temporalMutex_);
        temporalBuffer_.clear();
    }

    UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Configuration updated";
    return true;
}

TemporalStereoProcessor::TemporalStereoConfig TemporalStereoProcessor::getConfig() const {
    std::lock_guard<std::mutex> lock(configMutex_);
    return config_;
}

StereoMatcher* TemporalStereoProcessor::getStereoMatcher() const {
    return stereoMatcher_.get();
}

void TemporalStereoProcessor::setDebugMode(bool enable, const std::string& outputPath) {
    debugMode_ = enable;
    debugOutputPath_ = outputPath;

    if (enable) {
        std::filesystem::create_directories(outputPath);
        UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Debug mode enabled, output path: " << outputPath;
    } else {
        UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Debug mode disabled";
    }
}

std::map<std::string, double> TemporalStereoProcessor::getStatistics() const {
    std::lock_guard<std::mutex> lock(statsMutex_);
    return statistics_;
}

void TemporalStereoProcessor::saveDebugImages(const TemporalStereoResult& result) {
    if (!debugMode_) return;

    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    std::string prefix = debugOutputPath_ + "/temporal_" + std::to_string(timestamp);

    try {
        // Save isolated patterns
        if (!result.vcsel1PatternLeft.empty()) {
            cv::imwrite(prefix + "_vcsel1_left.png", result.vcsel1PatternLeft);
        }
        if (!result.vcsel1PatternRight.empty()) {
            cv::imwrite(prefix + "_vcsel1_right.png", result.vcsel1PatternRight);
        }
        if (!result.vcsel2PatternLeft.empty()) {
            cv::imwrite(prefix + "_vcsel2_left.png", result.vcsel2PatternLeft);
        }
        if (!result.vcsel2PatternRight.empty()) {
            cv::imwrite(prefix + "_vcsel2_right.png", result.vcsel2PatternRight);
        }

        // Save combined patterns
        if (!result.isolatedPatternLeft.empty()) {
            cv::imwrite(prefix + "_combined_left.png", result.isolatedPatternLeft);
        }
        if (!result.isolatedPatternRight.empty()) {
            cv::imwrite(prefix + "_combined_right.png", result.isolatedPatternRight);
        }

        // Save disparity map (normalized for visualization)
        if (!result.disparityMap.empty()) {
            cv::Mat disparityVis;
            cv::normalize(result.disparityMap, disparityVis, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::imwrite(prefix + "_disparity.png", disparityVis);
        }

        // Save depth map (normalized for visualization)
        if (!result.depthMap.empty()) {
            cv::Mat depthVis;
            result.depthMap.convertTo(depthVis, CV_8U, 255.0 / 3000.0); // Scale to 3000mm max
            cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);
            cv::imwrite(prefix + "_depth.png", depthVis);
        }

        // Save confidence map
        if (!result.confidenceMap.empty()) {
            cv::Mat confVis;
            result.confidenceMap.convertTo(confVis, CV_8U, 255.0);
            cv::imwrite(prefix + "_confidence.png", confVis);
        }

        UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Saved debug images to " << prefix;

    } catch (const std::exception& e) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to save debug images: " << e.what();
    }
}

void TemporalStereoProcessor::updateStatistics(const TemporalStereoResult& result) {
    std::lock_guard<std::mutex> lock(statsMutex_);

    statistics_["pattern_variance"] = result.patternVariance;
    statistics_["coverage_ratio"] = result.coverageRatio;
    statistics_["avg_confidence"] = result.avgConfidence;
    statistics_["valid_pixel_count"] = result.validPixelCount;
    statistics_["isolation_time_ms"] = result.isolationTimeMs;
    statistics_["matching_time_ms"] = result.matchingTimeMs;
    statistics_["total_time_ms"] = result.totalTimeMs;
    statistics_["topology_correct"] = result.isTopologicallyCorrect ? 1.0 : 0.0;

    // Compute running averages
    static double avgTotalTime = 0.0;
    static int processCount = 0;
    processCount++;
    avgTotalTime = (avgTotalTime * (processCount - 1) + result.totalTimeMs) / processCount;
    statistics_["avg_total_time_ms"] = avgTotalTime;
    statistics_["process_count"] = processCount;
}

bool TemporalStereoProcessor::validateConfig(const TemporalStereoConfig& config) const {
    // Validate pattern isolation parameters
    if (config.isolationParams.noiseThreshold < 0 ||
        config.isolationParams.noiseThreshold > 255) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid noise threshold: " << config.isolationParams.noiseThreshold;
        return false;
    }

    if (config.isolationParams.vcsel1Weight < 0 ||
        config.isolationParams.vcsel1Weight > 1.0f ||
        config.isolationParams.vcsel2Weight < 0 ||
        config.isolationParams.vcsel2Weight > 1.0f) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid VCSEL weights";
        return false;
    }

    if (config.isolationParams.minCoverage < 0 ||
        config.isolationParams.minCoverage > 1.0f ||
        config.isolationParams.maxCoverage < config.isolationParams.minCoverage ||
        config.isolationParams.maxCoverage > 1.0f) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid coverage range";
        return false;
    }

    // Validate SGBM parameters
    if (!config.sgbmParams.validate()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid SGBM parameters";
        return false;
    }

    return true;
}

bool TemporalStereoProcessor::generateDualDepthMaps(
    const cv::Mat& pattern1Left, const cv::Mat& pattern1Right,
    const cv::Mat& pattern2Left, const cv::Mat& pattern2Right,
    cv::Mat& depth1, cv::Mat& depth2,
    cv::Mat& confidence1, cv::Mat& confidence2) {

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Generating dual depth maps from individual VCSEL patterns";

    // Rectify pattern pairs
    cv::Mat rect1Left, rect1Right, rect2Left, rect2Right;

    // Rectify VCSEL1 patterns
    calibManager_->rectifyImages(pattern1Left, pattern1Right, rect1Left, rect1Right);

    // Rectify VCSEL2 patterns
    calibManager_->rectifyImages(pattern2Left, pattern2Right, rect2Left, rect2Right);

    // Generate depth map from VCSEL1 pattern (left-side illumination)
    cv::Mat disparity1;
    if (!stereoMatcher_->computeDisparityWithConfidence(rect1Left, rect1Right,
                                                        disparity1, confidence1)) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Failed to compute disparity for VCSEL1 pattern";
        disparity1 = cv::Mat::zeros(rect1Left.size(), CV_16S);
        confidence1 = cv::Mat::zeros(rect1Left.size(), CV_32F);
    }

    // Apply WLS filtering to VCSEL1 disparity
    if (config_.useWLSFilter && !disparity1.empty()) {
        stereoMatcher_->applyPostProcessing(disparity1, rect1Left);
    }

    // Convert VCSEL1 disparity to depth
    if (!disparityToDepth(disparity1, depth1)) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Failed to convert VCSEL1 disparity to depth";
        depth1 = cv::Mat::zeros(disparity1.size(), CV_32F);
    }

    // Generate depth map from VCSEL2 pattern (right-side illumination)
    cv::Mat disparity2;
    if (!stereoMatcher_->computeDisparityWithConfidence(rect2Left, rect2Right,
                                                        disparity2, confidence2)) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Failed to compute disparity for VCSEL2 pattern";
        disparity2 = cv::Mat::zeros(rect2Left.size(), CV_16S);
        confidence2 = cv::Mat::zeros(rect2Left.size(), CV_32F);
    }

    // Apply WLS filtering to VCSEL2 disparity
    if (config_.useWLSFilter && !disparity2.empty()) {
        stereoMatcher_->applyPostProcessing(disparity2, rect2Left);
    }

    // Convert VCSEL2 disparity to depth
    if (!disparityToDepth(disparity2, depth2)) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Failed to convert VCSEL2 disparity to depth";
        depth2 = cv::Mat::zeros(disparity2.size(), CV_32F);
    }

    // Validate results
    int valid1 = cv::countNonZero(depth1 > 0);
    int valid2 = cv::countNonZero(depth2 > 0);
    int totalPixels = depth1.rows * depth1.cols;

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor")
        << "Dual depth generation: VCSEL1=" << (static_cast<float>(valid1) / totalPixels * 100)
        << "% (" << valid1 << "/" << totalPixels << "), VCSEL2="
        << (static_cast<float>(valid2) / totalPixels * 100)
        << "% (" << valid2 << "/" << totalPixels << ")";

    return (valid1 > 0 || valid2 > 0);
}

bool TemporalStereoProcessor::fuseDualDepthMaps(
    const cv::Mat& depth1, const cv::Mat& depth2,
    const cv::Mat& confidence1, const cv::Mat& confidence2,
    cv::Mat& fusedDepth, cv::Mat& fusedConfidence) {

    if (depth1.size() != depth2.size() || depth1.type() != depth2.type()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Depth map size or type mismatch";
        return false;
    }

    int width = depth1.cols;
    int height = depth1.rows;

    // Initialize output matrices
    fusedDepth = cv::Mat::zeros(height, width, CV_32F);
    fusedConfidence = cv::Mat::zeros(height, width, CV_32F);

    // Create geometric weight map based on VCSEL positions
    cv::Mat geometricWeight1(height, width, CV_32F);
    cv::Mat geometricWeight2(height, width, CV_32F);

    // VCSEL1 closer to left camera → better for left side
    // VCSEL2 closer to right camera → better for right side
    for (int x = 0; x < width; ++x) {
        float w1 = 1.0f - (static_cast<float>(x) / width);  // Higher on left
        float w2 = static_cast<float>(x) / width;           // Higher on right

        // Apply sigmoid smoothing for gradual transition
        w1 = 1.0f / (1.0f + std::exp(-10.0f * (w1 - 0.5f)));
        w2 = 1.0f / (1.0f + std::exp(-10.0f * (w2 - 0.5f)));

        geometricWeight1.col(x) = w1;
        geometricWeight2.col(x) = w2;
    }

    // Perform pixel-wise fusion
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float d1 = depth1.at<float>(y, x);
            float d2 = depth2.at<float>(y, x);
            float c1 = confidence1.empty() ? 1.0f : confidence1.at<float>(y, x);
            float c2 = confidence2.empty() ? 1.0f : confidence2.at<float>(y, x);
            float g1 = geometricWeight1.at<float>(y, x);
            float g2 = geometricWeight2.at<float>(y, x);

            // Compute combined weights
            float w1 = c1 * g1;
            float w2 = c2 * g2;

            // Handle cases where only one depth is valid
            if (d1 > 0 && d2 <= 0) {
                fusedDepth.at<float>(y, x) = d1;
                fusedConfidence.at<float>(y, x) = c1;
            } else if (d2 > 0 && d1 <= 0) {
                fusedDepth.at<float>(y, x) = d2;
                fusedConfidence.at<float>(y, x) = c2;
            } else if (d1 > 0 && d2 > 0) {
                // Both valid - use weighted average
                float totalWeight = w1 + w2;
                if (totalWeight > 0) {
                    fusedDepth.at<float>(y, x) = (d1 * w1 + d2 * w2) / totalWeight;
                    fusedConfidence.at<float>(y, x) = std::max(c1, c2);

                    // Check for consistency - if depths differ significantly, reduce confidence
                    float depthDiff = std::abs(d1 - d2);
                    float relDiff = depthDiff / std::min(d1, d2);
                    if (relDiff > 0.1f) {  // More than 10% difference
                        fusedConfidence.at<float>(y, x) *= (1.0f - relDiff);
                    }
                }
            }
        }
    }

    // Apply median filter to remove outliers
    cv::Mat fusedDepthFiltered;
    cv::medianBlur(fusedDepth, fusedDepthFiltered, 5);

    // Preserve edges while smoothing
    cv::Mat fusedDepthSmooth;
    cv::bilateralFilter(fusedDepthFiltered, fusedDepthSmooth, 9, 50, 50);
    fusedDepth = fusedDepthSmooth;

    // Compute fusion statistics
    int validFused = cv::countNonZero(fusedDepth > 0);
    int valid1 = cv::countNonZero(depth1 > 0);
    int valid2 = cv::countNonZero(depth2 > 0);
    int totalPixels = fusedDepth.rows * fusedDepth.cols;

    float fusionGain = static_cast<float>(validFused) / std::max(valid1, valid2);

    UNLOOK_LOG_INFO("TemporalStereoProcessor")
        << "Depth fusion complete: input1=" << (static_cast<float>(valid1) / totalPixels * 100)
        << "%, input2=" << (static_cast<float>(valid2) / totalPixels * 100)
        << "%, fused=" << (static_cast<float>(validFused) / totalPixels * 100)
        << "%, gain=" << fusionGain << "x";

    return validFused > 0;
}

} // namespace stereo
} // namespace unlook