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
#include <sstream>

namespace unlook {
namespace stereo {

using namespace unlook::core;

TemporalStereoProcessor::TemporalStereoProcessor() {
    UNLOOK_LOG_INFO("TemporalStereoProcessor")
        << "Creating temporal stereo processor for VCSEL-based depth sensing";
}

TemporalStereoProcessor::~TemporalStereoProcessor() {
    UNLOOK_LOG_INFO("TemporalStereoProcessor")
        << "Destroying temporal stereo processor";
}

bool TemporalStereoProcessor::initialize(
    std::shared_ptr<hardware::AS1170DualVCSELController> dualVCSEL,
    std::shared_ptr<calibration::CalibrationManager> calibManager,
    const TemporalStereoConfig& config) {

    if (!dualVCSEL || !calibManager) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Invalid dual VCSEL controller or calibration manager";
        return false;
    }

    if (!dualVCSEL->isInitialized()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Dual VCSEL controller not initialized";
        return false;
    }

    if (!calibManager->isCalibrationValid()) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Calibration manager not calibrated";
        return false;
    }

    if (!validateConfig(config)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Invalid configuration parameters";
        return false;
    }

    dualVCSEL_ = dualVCSEL;
    calibManager_ = calibManager;
    config_ = config;

    // Create stereo matcher with VCSEL-optimized parameters
    stereoMatcher_ = StereoMatcher::create(StereoAlgorithm::SGBM);
    if (!stereoMatcher_) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to create SGBM stereo matcher";
        return false;
    }

    // Set VCSEL-optimized parameters
    if (!stereoMatcher_->setParameters(config_.sgbmParams)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to set stereo matching parameters";
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
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Processor not initialized";
        return false;
    }

    if (!frames.is_valid) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Invalid triple frame capture";
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

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor")
        << "Pattern metrics: variance=" << std::fixed << std::setprecision(1) << result.patternVariance
        << ", coverage=" << std::fixed << std::setprecision(2) << (result.coverageRatio * 100) << "%";

    // Validate pattern quality
    if (result.patternVariance < config_.isolationParams.minPatternVariance) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor")
            << "Pattern variance too low: " << std::fixed << std::setprecision(1) << result.patternVariance
            << " < " << config_.isolationParams.minPatternVariance;
    }

    if (result.coverageRatio < config_.isolationParams.minCoverage ||
        result.coverageRatio > config_.isolationParams.maxCoverage) {
        UNLOOK_LOG_WARNING("TemporalStereoProcessor")
            << "Coverage ratio out of range: " << std::fixed << std::setprecision(2) << (result.coverageRatio * 100)
            << "% not in [" << (config_.isolationParams.minCoverage * 100)
            << "%, " << (config_.isolationParams.maxCoverage * 100) << "%]";
    }

    // Step 3: Generate dual depth maps from individual VCSEL patterns
    auto matchingStart = std::chrono::high_resolution_clock::now();

    cv::Mat depth1, depth2, confidence1, confidence2;

    // Generate depth maps from both VCSEL patterns independently
    if (!generateDualDepthMaps(
            result.vcsel1PatternLeft, result.vcsel1PatternRight,
            result.vcsel2PatternLeft, result.vcsel2PatternRight,
            depth1, depth2, confidence1, confidence2)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to generate dual depth maps";
        return false;
    }

    // Step 4: Intelligently fuse the dual depth maps
    cv::Mat fusedDepth, fusedConfidence;
    if (!fuseDualDepthMaps(depth1, depth2, confidence1, confidence2,
                           fusedDepth, fusedConfidence)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to fuse dual depth maps";
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
            UNLOOK_LOG_ERROR("TemporalStereoProcessor")
                << "Failed to compute disparity with confidence";
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
            UNLOOK_LOG_ERROR("TemporalStereoProcessor")
                << "Failed to compute disparity";
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
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to convert disparity to depth";
        return false;
    }

    // Choose the best depth map based on confidence and coverage
    int validFused = cv::countNonZero(fusedDepth > 0);
    int validCombined = cv::countNonZero(combinedDepth > 0);

    if (validFused > validCombined) {
        result.depthMap = fusedDepth;
        UNLOOK_LOG_DEBUG("TemporalStereoProcessor")
            << "Using fused dual depth (" << validFused << ">" << validCombined << " valid pixels)";
    } else {
        result.depthMap = combinedDepth;
        UNLOOK_LOG_DEBUG("TemporalStereoProcessor")
            << "Using combined pattern depth (" << validCombined << ">" << validFused << " valid pixels)";
    }

    // Count valid pixels
    result.validPixelCount = cv::countNonZero(result.depthMap > 0);

    // Step 6: Apply temporal filtering if enabled
    if (config_.isolationParams.enableTemporalFilter) {
        applyTemporalFilter(result.depthMap);
    }

    // Step 7: Validate depth topologically if enabled
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

    UNLOOK_LOG_INFO("TemporalStereoProcessor")
        << "Processing complete: " << std::fixed << std::setprecision(1) << result.totalTimeMs
        << "ms total (isolation: " << result.isolationTimeMs
        << "ms, matching: " << result.matchingTimeMs << "ms)";

    return result.isValid();
}

bool TemporalStereoProcessor::captureAndProcess(TemporalStereoResult& result) {
    if (!initialized_) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Processor not initialized";
        return false;
    }

    // Capture triple frame sequence
    hardware::AS1170DualVCSELController::TripleFrameCapture frames;
    if (!dualVCSEL_->captureTemporalSequence(frames)) {
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Failed to capture temporal sequence";
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
        UNLOOK_LOG_ERROR("TemporalStereoProcessor")
            << "Frame size or type mismatch";
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

    UNLOOK_LOG_DEBUG("TemporalStereoProcessor")
        << "Pattern isolation: variance=" << std::fixed << std::setprecision(1) << variance
        << ", mean=" << mean[0]
        << ", enhancement=" << std::setprecision(2) << enhancementFactor << "x"
        << ", adaptive_threshold=" << std::setprecision(1) << adaptiveThreshold;

    return variance;
}

// Continue with rest of the implementation...
// (I'll need to continue this in the next part due to length)