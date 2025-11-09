/**
 * @file test_offline_scan.cpp
 * @brief Offline scan test using pre-captured frames
 *
 * This test replicates the exact same processing pipeline as HandheldScanWidget
 * but uses pre-captured raw frames instead of live camera feed.
 *
 * Purpose: Fast iteration on stereo processing optimizations without cameras.
 */

#include <unlook/api/HandheldScanPipeline.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <unlook/stereo/RectificationEngine.hpp>
#include <unlook/stereo/DisparityComputer.hpp>
#include <unlook/stereo/DepthProcessor.hpp>
#include <unlook/stereo/DebugOutputManager.hpp>
#include <unlook/core/Logger.hpp>
#include <unlook/core/exception.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>
#include <chrono>
#include <vector>
#include <string>
#include <iomanip>
#include <filesystem>

namespace fs = std::filesystem;

// ============================================================================
// ███████╗ █████╗ ███████╗██╗   ██╗     ██████╗ ██████╗ ███╗   ██╗███████╗██╗ ██████╗
// ██╔════╝██╔══██╗██╔════╝╚██╗ ██╔╝    ██╔════╝██╔═══██╗████╗  ██║██╔════╝██║██╔════╝
// █████╗  ███████║███████╗ ╚████╔╝     ██║     ██║   ██║██╔██╗ ██║█████╗  ██║██║  ███╗
// ██╔══╝  ██╔══██║╚════██║  ╚██╔╝      ██║     ██║   ██║██║╚██╗██║██╔══╝  ██║██║   ██║
// ███████╗██║  ██║███████║   ██║       ╚██████╗╚██████╔╝██║ ╚████║██║     ██║╚██████╔╝
// ╚══════╝╚═╝  ╚═╝╚══════╝   ╚═╝        ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝╚═╝     ╚═╝ ╚═════╝
// ============================================================================
// MODIFY THESE PARAMETERS TO TEST DIFFERENT CONFIGURATIONS
// All stereo processing parameters are here for easy tuning
// ============================================================================

struct TestConfig {
    // === INPUT/OUTPUT ===
    std::string framesDir = "/home/alessandro/unlook_debug/scan_20251109_023628";
    std::string calibFile = "/unlook_calib/default.yaml";
    bool swapCameras = false;  // If true, swap left/right
    bool swapCalibrationLR = false;  // If true, swap calibration matrices L↔R

    // === DISPARITY PARAMETERS ===
    // Quick Win #1: Reduce disparity range for speed
    int numDisparities = 256;  // Try 128 for 2x speedup, 64 for 4x speedup
    int minDisparity = 0;
    int blockSize = 7;

    // SGM smoothness penalties (OpenCV formula: P1=8*ch*BS², P2=32*ch*BS²)
    // Quick Win: Use these values or calculate from blockSize
    int P1 = 392;   // 8 * 1 * 7²  (or set to 0 to auto-calculate from blockSize)
    int P2 = 1568;  // 32 * 1 * 7² (or set to 0 to auto-calculate from blockSize)

    // === SGM PATH CONTROL ===
    // Quick Win #2: Reduce paths for speed
    bool use_path_LR = true;   // Left→Right (RECOMMENDED: always enabled)
    bool use_path_RL = true;   // Right→Left (RECOMMENDED: always enabled)
    bool use_path_TB = true;   // Top→Bottom (disable for 2x speedup)
    bool use_path_BT = true;   // Bottom→Top (disable for 2x speedup)

    // === QUALITY PARAMETERS ===
    int uniquenessRatio = 10;      // 5-15, higher = stricter (10 = good default)
    int speckleWindowSize = 200;   // 50-200, larger = more filtering
    int speckleRange = 2;          // 1-32, smaller = stricter
    int disp12MaxDiff = 1;         // 1-2, left-right consistency check
    int preFilterCap = 63;         // 1-63, pre-filter intensity cap

    // === AD-CENSUS PARAMETERS ===
    // lambdaAD + lambdaCensus should = 1.0
    float lambdaAD = 0.25f;        // 0.0-1.0, weight for intensity difference
    float lambdaCensus = 0.75f;    // 0.0-1.0, weight for census transform
    int censusWindowSize = 9;      // 5, 7, 9 (odd numbers, larger = slower)
    int censusThreshold = 4;       // Census binary threshold

    // === POST-PROCESSING ===
    bool useSubpixel = true;       // Sub-pixel refinement
    bool useWLSFilter = true;      // WLS filter (if OpenCV ximgproc available)
    double wlsLambda = 8000.0;     // WLS smoothness (higher = smoother)
    double wlsSigma = 1.5;         // WLS color similarity

    // === RESOLUTION CONTROL ===
    // Quick Win #3: Process at lower resolution for speed
    bool downsampleToVGA = false;  // true = 3-4x speedup, slight quality loss
    int processingWidth = 1280;    // If downsampleToVGA=false, use these
    int processingHeight = 720;

    // === DEBUG OUTPUT ===
    bool saveAllDebugImages = true;
    bool saveRectified = true;
    bool saveDisparity = true;
    bool saveDepthMap = true;
    bool saveConfidenceMap = true;
    bool verbose = true;
};

// ============================================================================
// END OF EASY CONFIG - CODE BELOW
// ============================================================================

struct FramePair {
    cv::Mat leftImage;
    cv::Mat rightImage;
    int frameNumber;
};

struct ProcessingStats {
    int frameNumber;
    double rectificationMs;
    double disparityMs;
    double depthConversionMs;
    double totalMs;
    double validPixelPercent;
    double minDepth, maxDepth;
};

class OfflineScanTester {
public:
    OfflineScanTester(const TestConfig& config)
        : config_(config), logger_(unlook::core::Logger::getInstance()) {

        logger_.info("╔══════════════════════════════════════════════════════════════╗");
        logger_.info("║         OFFLINE SCAN TESTER - FAST OPTIMIZATION LOOP         ║");
        logger_.info("╚══════════════════════════════════════════════════════════════╝");
        logger_.info("");
        logger_.info("Frames directory: " + config_.framesDir);
        logger_.info("Calibration file: " + config_.calibFile);

        if (config_.swapCameras) logger_.info("⚠️  CAMERAS SWAPPED: Left↔Right");
        if (config_.swapCalibrationLR) logger_.info("⚠️  CALIBRATION SWAPPED: L↔R matrices");
        if (config_.downsampleToVGA) logger_.info("⚠️  DOWNSAMPLING: Processing at VGA (640x480)");

        // Create output directory
        outputDir_ = "/home/alessandro/unlook_debug/test_offline_scan_" + getCurrentTimestamp();
        fs::create_directories(outputDir_);
        logger_.info("Output directory: " + outputDir_);
        logger_.info("");
    }

    bool initialize() {
        logger_.info("--- INITIALIZATION ---\n");

        // 1. Load calibration
        logger_.info("Loading calibration...");
        calibManager_ = std::make_shared<unlook::calibration::CalibrationManager>();

        try {
            calibManager_->loadCalibration(config_.calibFile);
            auto calib = calibManager_->getCalibrationData();

            // Optionally swap calibration matrices
            if (config_.swapCalibrationLR) {
                logger_.warning("Swapping calibration L↔R matrices...");
                std::swap(calib.cameraMatrixLeft, calib.cameraMatrixRight);
                std::swap(calib.distCoeffsLeft, calib.distCoeffsRight);
                std::swap(calib.R1, calib.R2);
                std::swap(calib.P1, calib.P2);
                // Note: Q matrix might need adjustment too
            }

            logger_.info("✓ Calibration loaded:");
            logger_.info("  - Image size: " + std::to_string(calib.imageSize.width) + "x" +
                        std::to_string(calib.imageSize.height));
            logger_.info("  - Baseline: " + std::to_string(calib.baselineMm) + " mm");

            imageSize_ = calib.imageSize;
            Q_ = calib.Q;

        } catch (const std::exception& e) {
            logger_.error("Failed to load calibration: " + std::string(e.what()));
            return false;
        }

        // 2. Initialize RectificationEngine
        logger_.info("\nInitializing RectificationEngine...");
        rectificationEngine_ = std::make_unique<unlook::stereo::RectificationEngine>(
            calibManager_.get(), &logger_);

        unlook::stereo::RectificationEngine::Config rectConfig;
        rectConfig.validateSize = true;
        rectConfig.enableDebug = false;
        rectificationEngine_->setConfig(rectConfig);

        logger_.info("✓ RectificationEngine initialized");

        // 3. Initialize DisparityComputer
        logger_.info("\nInitializing DisparityComputer...");
        disparityComputer_ = std::make_unique<unlook::stereo::DisparityComputer>(&logger_);

        unlook::stereo::DisparityComputer::Config dispConfig;

        // Auto-calculate P1/P2 if set to 0
        int p1 = config_.P1;
        int p2 = config_.P2;
        if (p1 == 0) p1 = 8 * 1 * config_.blockSize * config_.blockSize;
        if (p2 == 0) p2 = 32 * 1 * config_.blockSize * config_.blockSize;

        dispConfig.method = unlook::stereo::DisparityMethod::AUTO;
        dispConfig.minDisparity = config_.minDisparity;
        dispConfig.numDisparities = config_.numDisparities;
        dispConfig.blockSize = config_.blockSize;
        dispConfig.P1 = p1;
        dispConfig.P2 = p2;
        dispConfig.uniquenessRatio = config_.uniquenessRatio;
        dispConfig.speckleWindowSize = config_.speckleWindowSize;
        dispConfig.speckleRange = config_.speckleRange;
        dispConfig.disp12MaxDiff = config_.disp12MaxDiff;
        dispConfig.preFilterCap = config_.preFilterCap;
        dispConfig.mode = cv::StereoSGBM::MODE_HH4;
        dispConfig.lambdaAD = config_.lambdaAD;
        dispConfig.lambdaCensus = config_.lambdaCensus;
        dispConfig.censusWindowSize = config_.censusWindowSize;
        dispConfig.censusThreshold = config_.censusThreshold;
        dispConfig.useSubpixel = config_.useSubpixel;
        dispConfig.useWLSFilter = config_.useWLSFilter;
        dispConfig.wlsLambda = config_.wlsLambda;
        dispConfig.wlsSigma = config_.wlsSigma;

        disparityComputer_->setConfig(dispConfig);

        logger_.info("✓ DisparityComputer configured:");
        logger_.info("  - Disparities: " + std::to_string(config_.numDisparities) +
                    " (range: " + std::to_string(config_.minDisparity) + "-" +
                    std::to_string(config_.minDisparity + config_.numDisparities) + ")");
        logger_.info("  - Block size: " + std::to_string(config_.blockSize) + "x" +
                    std::to_string(config_.blockSize));
        logger_.info("  - P1/P2: " + std::to_string(p1) + "/" + std::to_string(p2));
        logger_.info("  - AD-Census: λAD=" + std::to_string(config_.lambdaAD) +
                    ", λCensus=" + std::to_string(config_.lambdaCensus));
        logger_.info("  - SGM paths: " +
                    std::string(config_.use_path_LR ? "L→R " : "") +
                    std::string(config_.use_path_RL ? "R→L " : "") +
                    std::string(config_.use_path_TB ? "T→B " : "") +
                    std::string(config_.use_path_BT ? "B→T" : ""));

        // 4. Initialize DebugOutputManager
        debugManager_ = std::make_unique<unlook::stereo::DebugOutputManager>(&logger_);

        unlook::stereo::DebugOutputManager::Config debugConfig;
        debugConfig.enabled = config_.saveAllDebugImages;
        debugConfig.outputDir = outputDir_;
        debugConfig.saveInputImages = true;
        debugConfig.saveRectified = config_.saveRectified;
        debugConfig.saveRawDisparity = config_.saveDisparity;
        debugConfig.saveDepthMap = config_.saveDepthMap;
        debugConfig.savePointCloud = false;  // Point cloud not needed for quick tests

        debugManager_->setConfig(debugConfig);
        logger_.info("✓ DebugOutputManager initialized");

        return true;
    }

    bool loadFrames() {
        logger_.info("\n--- LOADING FRAMES ---\n");

        frames_.clear();

        for (int i = 0; i < 10; i++) {
            std::string leftPath = config_.framesDir + "/00_raw_frame" +
                                  (i < 10 ? "0" : "") + std::to_string(i) + "_left.png";
            std::string rightPath = config_.framesDir + "/00_raw_frame" +
                                   (i < 10 ? "0" : "") + std::to_string(i) + "_right.png";

            // Optionally swap camera files
            if (config_.swapCameras) {
                std::swap(leftPath, rightPath);
            }

            if (!fs::exists(leftPath) || !fs::exists(rightPath)) {
                logger_.warning("Frame " + std::to_string(i) + " not found, stopping at " +
                               std::to_string(i) + " frames");
                break;
            }

            FramePair pair;
            pair.leftImage = cv::imread(leftPath, cv::IMREAD_GRAYSCALE);
            pair.rightImage = cv::imread(rightPath, cv::IMREAD_GRAYSCALE);
            pair.frameNumber = i;

            if (pair.leftImage.empty() || pair.rightImage.empty()) {
                logger_.error("Failed to load frame " + std::to_string(i));
                continue;
            }

            // Optionally downsample for faster processing
            if (config_.downsampleToVGA) {
                cv::resize(pair.leftImage, pair.leftImage, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
                cv::resize(pair.rightImage, pair.rightImage, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
                logger_.info("✓ Frame " + std::to_string(i) + " downsampled to VGA (640x480)");
            }

            frames_.push_back(pair);
            if (config_.verbose) {
                logger_.info("✓ Loaded frame " + std::to_string(i) + " (" +
                            std::to_string(pair.leftImage.cols) + "x" +
                            std::to_string(pair.leftImage.rows) + ")");
            }
        }

        logger_.info("\nTotal frames loaded: " + std::to_string(frames_.size()));
        return !frames_.empty();
    }

    void processFrames() {
        logger_.info("\n--- PROCESSING FRAMES ---");
        logger_.info("Processing " + std::to_string(frames_.size()) + " frames with current config...\n");

        stats_.clear();

        for (size_t i = 0; i < frames_.size(); i++) {
            if (config_.verbose) {
                logger_.info("╔══════════════════════════════════════════════════╗");
                logger_.info("║  PROCESSING FRAME " + std::to_string(i) + "/" +
                            std::to_string(frames_.size()) + "                          ║");
                logger_.info("╚══════════════════════════════════════════════════╝");
            }

            ProcessingStats frameStats = processFrame(frames_[i]);
            stats_.push_back(frameStats);

            if (config_.verbose) {
                logger_.info("Frame " + std::to_string(i) + " complete:");
                logger_.info("  - Rectification: " + std::to_string(frameStats.rectificationMs) + " ms");
                logger_.info("  - Disparity: " + std::to_string(frameStats.disparityMs) + " ms");
                logger_.info("  - Depth: " + std::to_string(frameStats.depthConversionMs) + " ms");
                logger_.info("  - Total: " + std::to_string(frameStats.totalMs) + " ms (" +
                            std::to_string(1000.0/frameStats.totalMs) + " FPS)");
                logger_.info("  - Valid pixels: " + std::to_string(frameStats.validPixelPercent) + "%\n");
            }
        }
    }

    ProcessingStats processFrame(const FramePair& frame) {
        ProcessingStats stats;
        stats.frameNumber = frame.frameNumber;

        auto totalStart = std::chrono::high_resolution_clock::now();

        // STAGE 1: RECTIFICATION
        if (config_.verbose) logger_.info("Stage 1: Rectification...");
        auto rectStart = std::chrono::high_resolution_clock::now();

        auto rectResult = rectificationEngine_->rectify(frame.leftImage, frame.rightImage);

        if (!rectResult.success) {
            logger_.error("Rectification failed: " + rectResult.errorMessage);
            return stats;
        }

        auto rectEnd = std::chrono::high_resolution_clock::now();
        stats.rectificationMs = std::chrono::duration<double, std::milli>(rectEnd - rectStart).count();

        if (config_.verbose) {
            logger_.info("✓ Rectification complete (" + std::to_string(stats.rectificationMs) + " ms)");
        }

        // Save rectified
        if (config_.saveRectified) {
            debugManager_->saveRectifiedImages(rectResult.leftRectified, rectResult.rightRectified);
        }

        // STAGE 2: DISPARITY
        if (config_.verbose) logger_.info("Stage 2: Disparity computation...");
        auto dispStart = std::chrono::high_resolution_clock::now();

        auto dispResult = disparityComputer_->compute(rectResult.leftRectified,
                                                      rectResult.rightRectified);

        if (!dispResult.success) {
            logger_.error("Disparity failed: " + dispResult.errorMessage);
            return stats;
        }

        auto dispEnd = std::chrono::high_resolution_clock::now();
        stats.disparityMs = std::chrono::duration<double, std::milli>(dispEnd - dispStart).count();
        stats.validPixelPercent = dispResult.validPixelPercentage;

        if (config_.verbose) {
            logger_.info("✓ Disparity complete (" + std::to_string(stats.disparityMs) + " ms)");
        }

        // Save disparity
        if (config_.saveDisparity) {
            debugManager_->saveRawDisparity(dispResult.disparity);
        }

        // STAGE 3: DEPTH
        if (config_.verbose) logger_.info("Stage 3: Depth conversion...");
        auto depthStart = std::chrono::high_resolution_clock::now();

        cv::Mat depthMap;
        if (Q_.rows == 4 && Q_.cols == 4) {
            cv::Mat depth3D;
            cv::reprojectImageTo3D(dispResult.disparity, depth3D, Q_, true);

            std::vector<cv::Mat> channels;
            cv::split(depth3D, channels);
            depthMap = channels[2];

            cv::threshold(depthMap, depthMap, 0, 0, cv::THRESH_TOZERO);
            cv::threshold(depthMap, depthMap, 10000, 10000, cv::THRESH_TRUNC);

            // Calculate depth range
            double minVal, maxVal;
            cv::Mat validMask = depthMap > 0 & depthMap < 10000;
            if (cv::countNonZero(validMask) > 0) {
                cv::minMaxLoc(depthMap, &minVal, &maxVal, nullptr, nullptr, validMask);
                stats.minDepth = minVal;
                stats.maxDepth = maxVal;
            } else {
                stats.minDepth = stats.maxDepth = 0;
            }
        }

        auto depthEnd = std::chrono::high_resolution_clock::now();
        stats.depthConversionMs = std::chrono::duration<double, std::milli>(depthEnd - depthStart).count();

        // Save depth
        if (config_.saveDepthMap) {
            debugManager_->saveDepthMap(depthMap);
        }

        auto totalEnd = std::chrono::high_resolution_clock::now();
        stats.totalMs = std::chrono::duration<double, std::milli>(totalEnd - totalStart).count();

        return stats;
    }

    void printSummary() {
        logger_.info("\n╔══════════════════════════════════════════════════════════════╗");
        logger_.info("║                    PROCESSING SUMMARY                        ║");
        logger_.info("╚══════════════════════════════════════════════════════════════╝");

        if (stats_.empty()) {
            logger_.info("No frames processed!");
            return;
        }

        // Averages
        double avgRect = 0, avgDisp = 0, avgDepth = 0, avgTotal = 0, avgValid = 0;
        for (const auto& s : stats_) {
            avgRect += s.rectificationMs;
            avgDisp += s.disparityMs;
            avgDepth += s.depthConversionMs;
            avgTotal += s.totalMs;
            avgValid += s.validPixelPercent;
        }
        avgRect /= stats_.size();
        avgDisp /= stats_.size();
        avgDepth /= stats_.size();
        avgTotal /= stats_.size();
        avgValid /= stats_.size();

        logger_.info("\n📊 Frames processed: " + std::to_string(stats_.size()));
        logger_.info("\n⏱️  Average timing per frame:");
        logger_.info("  Rectification:   " + formatTime(avgRect) + "  ( " +
                    formatPercent(avgRect, avgTotal) + "%)");
        logger_.info("  Disparity:       " + formatTime(avgDisp) + "  ( " +
                    formatPercent(avgDisp, avgTotal) + "%) ← BOTTLENECK");
        logger_.info("  Depth:           " + formatTime(avgDepth) + "  ( " +
                    formatPercent(avgDepth, avgTotal) + "%)");
        logger_.info("  ────────────────────────────────────");
        logger_.info("  TOTAL:           " + formatTime(avgTotal));

        logger_.info("\n🚀 Performance:");
        logger_.info("  Average FPS:     " + std::to_string(1000.0 / avgTotal) + " fps");
        logger_.info("  Valid pixels:    " + std::to_string(avgValid) + "%");

        logger_.info("\n📁 Output: " + outputDir_);
        logger_.info("\n✅ Test complete!");

        // Print optimization suggestions
        if (avgDisp > 2000 && config_.numDisparities > 128) {
            logger_.info("\n💡 OPTIMIZATION SUGGESTION:");
            logger_.info("   Try reducing numDisparities from " +
                        std::to_string(config_.numDisparities) + " to 128 for 2x speedup");
        }
        if (config_.use_path_TB && config_.use_path_BT && avgDisp > 1000) {
            logger_.info("\n💡 OPTIMIZATION SUGGESTION:");
            logger_.info("   Disable vertical SGM paths (TB/BT) for 2x speedup");
        }
    }

private:
    std::string getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&time), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    std::string formatTime(double ms) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << std::setw(8) << ms << " ms";
        return ss.str();
    }

    std::string formatPercent(double part, double total) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(1) << std::setw(5) << (100.0 * part / total);
        return ss.str();
    }

    TestConfig config_;
    std::string outputDir_;
    cv::Size imageSize_;
    cv::Mat Q_;

    std::shared_ptr<unlook::calibration::CalibrationManager> calibManager_;
    std::unique_ptr<unlook::stereo::RectificationEngine> rectificationEngine_;
    std::unique_ptr<unlook::stereo::DisparityComputer> disparityComputer_;
    std::unique_ptr<unlook::stereo::DebugOutputManager> debugManager_;

    std::vector<FramePair> frames_;
    std::vector<ProcessingStats> stats_;

    unlook::core::Logger& logger_;
};

int main(int argc, char** argv) {
    // Create config with defaults (modifiable at top of file)
    TestConfig config;

    // Override from command line if provided
    if (argc > 1) config.framesDir = argv[1];
    if (argc > 2) config.calibFile = argv[2];

    try {
        OfflineScanTester tester(config);

        if (!tester.initialize()) {
            std::cerr << "❌ Failed to initialize" << std::endl;
            return 1;
        }

        if (!tester.loadFrames()) {
            std::cerr << "❌ Failed to load frames" << std::endl;
            return 1;
        }

        tester.processFrames();
        tester.printSummary();

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}
