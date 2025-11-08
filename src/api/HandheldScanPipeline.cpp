/**
 * @file HandheldScanPipeline.cpp
 * @brief COMPLETE REWRITE: Industrial-grade stereo processing pipeline
 *
 * Based on MEGA_PROMPT_BACKEND_REWRITE.md Phase 5 specification.
 * This is a complete rewrite using the new backend architecture.
 */

#include <unlook/api/HandheldScanPipeline.hpp>
#include <unlook/api/camera_system.h>
#include <unlook/core/Logger.hpp>
#include <unlook/core/exception.h>

// NEW BACKEND COMPONENTS (replacing VCSELStereoMatcher)
#include <unlook/calibration/CalibrationManager.hpp>
#include <unlook/calibration/CalibrationValidation.hpp>
#include <unlook/stereo/RectificationEngine.hpp>
#include <unlook/stereo/DisparityComputer.hpp>
#include <unlook/stereo/DepthProcessor.hpp>
#include <unlook/stereo/DebugOutputManager.hpp>

// Hardware components
#include <unlook/hardware/BMI270Driver.hpp>
#include <unlook/hardware/StabilityDetector.hpp>
#include <unlook/hardware/AS1170DualVCSELController.hpp>

// OpenCV and standard includes
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <numeric>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <omp.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace unlook {
namespace api {

/**
 * @brief Private implementation class - COMPLETELY REWRITTEN
 */
class HandheldScanPipeline::Impl {
public:
    // ========== CORE SYSTEMS ==========
    std::shared_ptr<camera::CameraSystem> cameraSystem_;
    core::Logger& logger_ = core::Logger::getInstance();

    // ========== NEW BACKEND COMPONENTS ==========
    std::shared_ptr<calibration::CalibrationManager> calibrationManager_;
    std::unique_ptr<stereo::RectificationEngine> rectificationEngine_;
    std::unique_ptr<stereo::DisparityComputer> disparityComputer_;
    std::unique_ptr<stereo::DepthProcessor> depthProcessor_;
    std::unique_ptr<stereo::DebugOutputManager> debugManager_;

    // ========== FILTERS (TO BE IMPLEMENTED) ==========
    // TODO: Implement these filter classes if not available
    // std::unique_ptr<stereo::MedianFilter> medianFilter_;
    // std::unique_ptr<stereo::SpatialFilter> spatialFilter_;
    // std::unique_ptr<stereo::TemporalFilter> temporalFilter_;
    // std::unique_ptr<stereo::HoleFiller> holeFiller_;

    // ========== HARDWARE COMPONENTS ==========
    std::unique_ptr<hardware::AS1170DualVCSELController> vcselController_;
    std::shared_ptr<hardware::BMI270Driver> bmi270Driver_;
    std::unique_ptr<hardware::StabilityDetector> stabilityDetector_;

    // ========== CONFIGURATION ==========
    stereo::DisparityComputer::Config disparityConfig_;
    stereo::DebugOutputManager::Config debugConfig_;
    bool vcselEnabled_ = true;

    // ========== STATE MANAGEMENT ==========
    std::atomic<bool> initialized_{false};
    std::atomic<bool> scanning_{false};
    std::atomic<int> frameCounter_{0};

    // ========== CALIBRATION DATA ==========
    cv::Size calibImageSize_;
    float baseline_mm_ = 70.017f;
    cv::Mat Q_;  // Disparity-to-depth matrix

    // ========== STATISTICS ==========
    mutable std::mutex statsMutex_;
    std::map<std::string, double> statistics_;

    // ========== PROGRESS REPORTING ==========
    ProgressCallback currentProgressCallback_ = nullptr;
    std::atomic<float> currentProgress_{0.0f};
    std::string currentStage_{"Idle"};  // Not atomic - protected by mutex
    mutable std::mutex stageMutex_;

    Impl(std::shared_ptr<camera::CameraSystem> cameraSystem)
        : cameraSystem_(cameraSystem) {

        logger_.info("=================================================");
        logger_.info("HandheldScanPipeline REWRITE - Initializing new backend");
        logger_.info("=================================================");

        // Initialize calibration manager
        calibrationManager_ = std::make_shared<calibration::CalibrationManager>();

        // Load calibration (try multiple paths)
        std::vector<std::string> calibPaths = {
            "/unlook_calib/default.yaml",
            "/unlook_calib/calib-20251108_171118.yaml",
            "/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml"
        };

        bool calibLoaded = false;
        for (const auto& path : calibPaths) {
            if (calibrationManager_->loadCalibration(path)) {
                logger_.info("✓ Loaded calibration from: " + path);
                calibLoaded = true;
                break;
            }
        }

        if (!calibLoaded) {
            logger_.error("✗ Failed to load any calibration file!");
            throw core::Exception(core::ResultCode::ERROR_CALIBRATION_INVALID,
                                "No calibration file found");
        }

        // Extract critical calibration data
        if (calibrationManager_->isCalibrationValid()) {
            const auto& calibData = calibrationManager_->getCalibrationData();
            calibImageSize_ = calibData.imageSize;
            baseline_mm_ = calibData.baselineMm;
            Q_ = calibData.Q.clone();

            logger_.info("Calibration info:");
            logger_.info("  - Image size: " + std::to_string(calibImageSize_.width) +
                        "x" + std::to_string(calibImageSize_.height));
            logger_.info("  - Baseline: " + std::to_string(baseline_mm_) + " mm");
            logger_.info("  - Q matrix: " + std::to_string(Q_.rows) + "x" +
                        std::to_string(Q_.cols));
        }

        // Initialize disparity computer configuration
        disparityConfig_.method = stereo::DisparityMethod::AUTO;  // Auto-select GPU if available
        disparityConfig_.minDisparity = 0;
        disparityConfig_.numDisparities = 256;  // Full range for close to far
        disparityConfig_.blockSize = 7;
        disparityConfig_.P1 = 8;
        disparityConfig_.P2 = 32;
        disparityConfig_.uniquenessRatio = 15;
        disparityConfig_.speckleWindowSize = 100;
        disparityConfig_.speckleRange = 16;
        disparityConfig_.disp12MaxDiff = 2;
        disparityConfig_.preFilterCap = 63;
        disparityConfig_.mode = cv::StereoSGBM::MODE_HH4;  // High quality mode

        // AD-Census parameters for VCSEL
        disparityConfig_.lambdaAD = 0.3f;
        disparityConfig_.lambdaCensus = 0.7f;
        disparityConfig_.censusWindowSize = 9;
        disparityConfig_.censusThreshold = 4;

        // Enable post-processing
        disparityConfig_.useSubpixel = true;
        disparityConfig_.useWLSFilter = true;
        disparityConfig_.wlsLambda = 8000.0;
        disparityConfig_.wlsSigma = 1.5;

        // Initialize debug configuration
        debugConfig_.enabled = false;  // Will be enabled when needed
        debugConfig_.saveInputImages = true;
        debugConfig_.saveRectified = true;
        debugConfig_.saveRawDisparity = true;
        debugConfig_.saveFilteredDisparity = true;
        debugConfig_.saveDepthMap = true;
        debugConfig_.savePointCloud = true;
        debugConfig_.saveProcessingReport = true;

        // Initialize IMU and stability detector
        initializeIMU();

        // Initialize VCSEL controller
        initializeVCSEL();
    }

    void initializeIMU() {
        logger_.info("Initializing BMI270 IMU driver...");
        bmi270Driver_ = hardware::BMI270Driver::getInstance();

        hardware::BMI270Driver::BMI270Config imuConfig;
        imuConfig.i2c_bus = 1;
        imuConfig.i2c_address = 0x69;
        imuConfig.gyro_range_dps = 500;
        imuConfig.accel_range_g = 2;
        imuConfig.sample_rate_hz = 100;

        if (!bmi270Driver_->initialize(imuConfig)) {
            logger_.warning("Failed to initialize BMI270, continuing without IMU");
            bmi270Driver_.reset();
        } else {
            logger_.info("✓ BMI270 initialized successfully");

            // Initialize stability detector
            stabilityDetector_ = std::make_unique<hardware::StabilityDetector>(bmi270Driver_);

            hardware::StabilityDetector::StabilityParams stabilityParams;
            stabilityParams.gyro_threshold_dps = 0.5f;
            stabilityParams.accel_variance_threshold = 0.1f;
            stabilityParams.stable_duration_ms = 500;
            stabilityParams.history_window_ms = 1000;

            if (!stabilityDetector_->initialize(stabilityParams)) {
                logger_.warning("Failed to initialize StabilityDetector");
                stabilityDetector_.reset();
            } else {
                logger_.info("✓ StabilityDetector initialized");
            }
        }
    }

    void initializeVCSEL() {
        try {
            vcselController_ = std::make_unique<hardware::AS1170DualVCSELController>();
            logger_.warning("VCSEL controller created but not initialized (API mismatch)");
            vcselController_.reset();
            vcselEnabled_ = false;
        } catch (const std::exception& e) {
            logger_.warning("VCSEL not available: " + std::string(e.what()));
            vcselEnabled_ = false;
        }
    }

    /**
     * @brief Initialize all processing components with validation
     */
    bool initializeProcessingPipeline() {
        logger_.info("Initializing processing pipeline...");

        // CRITICAL: Validate calibration first!
        cv::Size actualSize(1280, 720);  // Expected camera resolution
        auto validation = calibration::CalibrationValidation::validate(
            *calibrationManager_, actualSize);

        if (!validation.allChecksPass) {
            logger_.error("╔════════════════════════════════════════╗");
            logger_.error("║  CALIBRATION VALIDATION FAILED!        ║");
            logger_.error("╚════════════════════════════════════════╝");
            validation.printReport(logger_);

            // Report to GUI via progress callback
            if (currentProgressCallback_) {
                currentProgressCallback_(0.0f, "ERROR: Calibration validation failed!");
            }
            return false;
        }

        logger_.info("✓ Calibration validation PASSED");
        logger_.info("  - Image size match: " +
                    std::to_string(validation.calibImageSize.width) + "x" +
                    std::to_string(validation.calibImageSize.height));
        logger_.info("  - Baseline: " + std::to_string(validation.baselineMM) + " mm");
        logger_.info("  - Epipolar alignment: " + std::to_string(validation.cyDifference) + " px");

        // Initialize rectification engine
        rectificationEngine_ = std::make_unique<stereo::RectificationEngine>(
            calibrationManager_.get(), &logger_);

        stereo::RectificationEngine::Config rectConfig;
        rectConfig.interpolation = cv::INTER_LINEAR;
        rectConfig.validateSize = true;  // CRITICAL: Always validate!
        rectConfig.enableDebug = debugConfig_.enabled;
        rectConfig.debugDir = debugConfig_.outputDir;
        rectificationEngine_->setConfig(rectConfig);

        // Initialize disparity computer
        disparityComputer_ = std::make_unique<stereo::DisparityComputer>(&logger_);
        disparityComputer_->setConfig(disparityConfig_);

        // Try to initialize GPU acceleration
        if (disparityComputer_->initializeGPU()) {
            logger_.info("✓ GPU acceleration available (Vulkan/VideoCore VII)");
            updateStats("gpu_available", 1.0);
        } else {
            logger_.info("⚠ GPU not available, using CPU/NEON");
            updateStats("gpu_available", 0.0);
        }

        // Initialize depth processor
        depthProcessor_ = std::make_unique<stereo::DepthProcessor>();
        // Initialize it with calibration manager
        if (!depthProcessor_->initialize(calibrationManager_)) {
            logger_.error("Failed to initialize DepthProcessor!");
            return false;
        }

        // Initialize debug manager
        debugManager_ = std::make_unique<stereo::DebugOutputManager>(&logger_);
        debugManager_->setConfig(debugConfig_);

        // TODO: Initialize filters when available
        // medianFilter_ = std::make_unique<stereo::MedianFilter>(&logger_);
        // spatialFilter_ = std::make_unique<stereo::SpatialFilter>(&logger_);
        // temporalFilter_ = std::make_unique<stereo::TemporalFilter>(&logger_);
        // holeFiller_ = std::make_unique<stereo::HoleFiller>(&logger_);

        logger_.info("✓ Processing pipeline initialized successfully");
        return true;
    }

    /**
     * @brief Process single stereo frame through complete pipeline
     */
    cv::Mat processFrameWithNewBackend(const StereoFrame& frame) {
        auto startTime = std::chrono::steady_clock::now();

        int frameIdx = frameCounter_.fetch_add(1);
        logger_.info("Processing frame " + std::to_string(frameIdx) + " with new backend...");

        // Update progress
        reportProgress(0.3f, "Rectifying images...");

        // ========== STAGE 1: RECTIFICATION ==========
        auto rectStart = std::chrono::steady_clock::now();

        auto rectResult = rectificationEngine_->rectify(frame.leftImage, frame.rightImage);

        if (!rectResult.success) {
            logger_.error("Rectification failed: " + rectResult.errorMessage);
            return cv::Mat();
        }

        auto rectTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - rectStart);

        logger_.info("✓ Rectification complete (" + std::to_string(rectTime.count()) + " ms)");
        logger_.info("  - Valid pixels: " + std::to_string(rectResult.validPixelPercent) + "%");

        // Save rectified images to debug
        if (debugConfig_.enabled && debugManager_) {
            debugManager_->saveRectifiedImages(rectResult.leftRectified,
                                              rectResult.rightRectified);
        }

        // Update progress
        reportProgress(0.5f, "Computing disparity...");

        // ========== STAGE 2: DISPARITY COMPUTATION ==========
        auto dispStart = std::chrono::steady_clock::now();

        auto dispResult = disparityComputer_->compute(rectResult.leftRectified,
                                                      rectResult.rightRectified);

        if (!dispResult.success) {
            logger_.error("Disparity computation failed: " + dispResult.errorMessage);
            return cv::Mat();
        }

        auto dispTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - dispStart);

        logger_.info("✓ Disparity computed (" + std::to_string(dispTime.count()) + " ms)");
        logger_.info(std::string("  - Method: ") + (dispResult.gpuAccelerated ? "GPU" : "CPU"));
        logger_.info("  - Valid pixels: " + std::to_string(dispResult.validPixelPercentage) + "%");

        // Save disparity to debug
        if (debugConfig_.enabled && debugManager_) {
            debugManager_->saveRawDisparity(dispResult.disparity);  // raw
        }

        // Update progress
        reportProgress(0.7f, "Converting to depth...");

        // ========== STAGE 3: DEPTH CONVERSION ==========
        auto depthStart = std::chrono::steady_clock::now();

        cv::Mat depthMap;
        if (Q_.rows == 4 && Q_.cols == 4) {
            // Use Q matrix for reprojection
            cv::Mat depth3D;
            cv::reprojectImageTo3D(dispResult.disparity, depth3D, Q_, true);

            // Extract Z channel as depth
            std::vector<cv::Mat> channels;
            cv::split(depth3D, channels);
            depthMap = channels[2];  // Z channel

            // Clean up invalid depths
            cv::threshold(depthMap, depthMap, 0, 0, cv::THRESH_TOZERO);
            cv::threshold(depthMap, depthMap, 10000, 10000, cv::THRESH_TRUNC);
        } else {
            logger_.error("Q matrix invalid for depth conversion!");
            return cv::Mat();
        }

        auto depthTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - depthStart);

        logger_.info("✓ Depth conversion complete (" + std::to_string(depthTime.count()) + " ms)");

        // ========== STAGE 4: FILTERING (TODO: Implement when filters available) ==========
        reportProgress(0.8f, "Applying filters...");

        cv::Mat filteredDepth = depthMap.clone();

        // Apply median filter for speckle removal
        if (filteredDepth.type() == CV_32F) {
            cv::medianBlur(filteredDepth, filteredDepth, 5);
        }

        // TODO: Apply other filters when implemented
        // if (medianFilter_) {
        //     filteredDepth = medianFilter_->apply(filteredDepth);
        // }
        // if (spatialFilter_) {
        //     filteredDepth = spatialFilter_->apply(filteredDepth, frame.leftImage);
        // }

        // Save filtered depth to debug
        if (debugConfig_.enabled && debugManager_) {
            debugManager_->saveDepthMap(filteredDepth);
        }

        // Calculate total processing time
        auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime);

        logger_.info("Frame " + std::to_string(frameIdx) +
                    " processing complete (" + std::to_string(totalTime.count()) + " ms)");

        // Update statistics
        updateStats("last_frame_rect_time_ms", rectTime.count());
        updateStats("last_frame_disp_time_ms", dispTime.count());
        updateStats("last_frame_depth_time_ms", depthTime.count());
        updateStats("last_frame_total_time_ms", totalTime.count());
        updateStats("last_frame_valid_pixels_percent", dispResult.validPixelPercentage);

        return filteredDepth;
    }

    /**
     * @brief Multi-frame temporal fusion
     */
    cv::Mat fuseDepthMapsWithTemporalFilter(const std::vector<cv::Mat>& depthMaps,
                                           float outlierSigma = 2.5f) {
        if (depthMaps.empty()) {
            throw core::Exception(core::ResultCode::ERROR_INVALID_PARAMETER,
                                "No depth maps to fuse");
        }

        logger_.info("Fusing " + std::to_string(depthMaps.size()) +
                    " depth maps with temporal filtering...");

        cv::Mat fused = cv::Mat::zeros(depthMaps[0].size(), CV_32F);
        cv::Mat countMap = cv::Mat::zeros(depthMaps[0].size(), CV_32S);

        int height = fused.rows;
        int width = fused.cols;

        // Process each pixel with outlier rejection
        #pragma omp parallel for collapse(2)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                std::vector<float> values;
                values.reserve(depthMaps.size());

                // Collect valid depth values
                for (const auto& depth : depthMaps) {
                    float val = depth.at<float>(y, x);
                    if (val > 0 && val < 10000) {
                        values.push_back(val);
                    }
                }

                if (values.empty()) {
                    continue;
                }

                if (values.size() == 1) {
                    fused.at<float>(y, x) = values[0];
                    countMap.at<int>(y, x) = 1;
                    continue;
                }

                // Calculate statistics
                float mean = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
                float variance = 0.0f;
                for (float val : values) {
                    float diff = val - mean;
                    variance += diff * diff;
                }
                variance /= values.size();
                float stddev = std::sqrt(variance);

                // Outlier rejection
                std::vector<float> inliers;
                for (float val : values) {
                    if (std::abs(val - mean) <= outlierSigma * stddev) {
                        inliers.push_back(val);
                    }
                }

                if (!inliers.empty()) {
                    // Use median of inliers
                    std::sort(inliers.begin(), inliers.end());
                    float median = inliers[inliers.size() / 2];
                    fused.at<float>(y, x) = median;
                    countMap.at<int>(y, x) = inliers.size();
                }
            }
        }

        logger_.info("✓ Temporal fusion complete");
        return fused;
    }

    /**
     * @brief Report progress to GUI
     */
    void reportProgress(float progress, const std::string& message) {
        currentProgress_ = progress;
        {
            std::lock_guard<std::mutex> lock(stageMutex_);
            currentStage_ = message;
        }

        if (currentProgressCallback_) {
            currentProgressCallback_(progress, message);
        }

        logger_.debug("Progress: " + std::to_string(int(progress * 100)) +
                     "% - " + message);
    }

    /**
     * @brief Update statistics
     */
    void updateStats(const std::string& key, double value) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        statistics_[key] = value;
    }
};

// ========== PUBLIC INTERFACE IMPLEMENTATION ==========

HandheldScanPipeline::HandheldScanPipeline(std::shared_ptr<camera::CameraSystem> cameraSystem)
    : pImpl(std::make_unique<Impl>(cameraSystem)) {
}

HandheldScanPipeline::~HandheldScanPipeline() {
    shutdown();
}

bool HandheldScanPipeline::initialize() {
    if (pImpl->initialized_) {
        return true;
    }

    pImpl->logger_.info("Initializing HandheldScanPipeline (NEW BACKEND)...");

    // Initialize processing pipeline with validation
    if (!pImpl->initializeProcessingPipeline()) {
        pImpl->logger_.error("Failed to initialize processing pipeline!");
        return false;
    }

    // Camera system is optional (for pre-captured frames)
    if (!pImpl->cameraSystem_) {
        pImpl->logger_.warning("Camera system not provided - using pre-captured frames only");
    }

    pImpl->initialized_ = true;
    pImpl->logger_.info("✓ HandheldScanPipeline initialized successfully (NEW BACKEND)");

    return true;
}

void HandheldScanPipeline::shutdown() {
    if (!pImpl->initialized_) {
        return;
    }

    pImpl->logger_.info("Shutting down HandheldScanPipeline...");

    pImpl->scanning_ = false;

    if (pImpl->vcselController_) {
        pImpl->vcselController_->shutdown();
    }

    pImpl->initialized_ = false;
    pImpl->logger_.info("HandheldScanPipeline shutdown complete");
}

HandheldScanPipeline::ScanResult HandheldScanPipeline::scanWithStability(
    const ScanParams& params,
    ProgressCallback progressCallback) {

    auto startTime = std::chrono::steady_clock::now();
    ScanResult result;

    if (!pImpl->initialized_) {
        result.success = false;
        result.errorMessage = "Pipeline not initialized";
        return result;
    }

    if (pImpl->scanning_) {
        result.success = false;
        result.errorMessage = "Scan already in progress";
        return result;
    }

    pImpl->scanning_ = true;
    pImpl->currentProgressCallback_ = progressCallback;
    pImpl->frameCounter_ = 0;

    try {
        // ========== PHASE 1: STABILITY DETECTION ==========
        pImpl->logger_.info("PHASE 1: Waiting for stability...");
        pImpl->reportProgress(0.1f, "Waiting for device stability...");

        auto stabilityStart = std::chrono::steady_clock::now();

        bool stable = waitForStability(
            [this](float score) {
                pImpl->reportProgress(0.1f + score * 0.15f,
                    "Stability: " + std::to_string(static_cast<int>(score * 100)) + "%");
            },
            params.stabilityTimeoutMs
        );

        result.stabilityWaitTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - stabilityStart);

        if (!stable) {
            result.success = false;
            result.errorMessage = "Failed to achieve stability within timeout";
            pImpl->scanning_ = false;
            return result;
        }

        // ========== PHASE 2: MULTI-FRAME CAPTURE ==========
        pImpl->logger_.info("PHASE 2: Capturing " + std::to_string(params.numFrames) + " frames...");
        pImpl->reportProgress(0.25f, "Capturing frames...");

        auto captureStart = std::chrono::steady_clock::now();

        auto frames = captureMultiFrame(params.numFrames,
            [this, &params](float progress, const std::string& msg) {
                pImpl->reportProgress(0.25f + progress * 0.2f, msg);
            });

        result.captureTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - captureStart);

        if (frames.empty()) {
            result.success = false;
            result.errorMessage = "Failed to capture frames";
            pImpl->scanning_ = false;
            return result;
        }

        result.framesCaptures = frames.size();

        // ========== PHASE 3: PROCESS FRAMES WITH NEW BACKEND ==========
        pImpl->logger_.info("PHASE 3: Processing frames with new backend...");
        pImpl->reportProgress(0.45f, "Processing stereo depth...");

        auto processingStart = std::chrono::steady_clock::now();

        std::vector<cv::Mat> depthMaps;
        depthMaps.reserve(frames.size());

        for (size_t i = 0; i < frames.size(); ++i) {
            float frameProgress = static_cast<float>(i) / frames.size();
            pImpl->reportProgress(0.45f + frameProgress * 0.3f,
                "Processing frame " + std::to_string(i+1) + "/" + std::to_string(frames.size()));

            cv::Mat depth = pImpl->processFrameWithNewBackend(frames[i]);
            if (!depth.empty()) {
                depthMaps.push_back(depth);
            }
        }

        if (depthMaps.empty()) {
            result.success = false;
            result.errorMessage = "Failed to process any frames";
            pImpl->scanning_ = false;
            return result;
        }

        // ========== PHASE 4: TEMPORAL FUSION ==========
        pImpl->logger_.info("PHASE 4: Fusing " + std::to_string(depthMaps.size()) + " depth maps...");
        pImpl->reportProgress(0.75f, "Fusing depth maps...");

        result.depthMap = pImpl->fuseDepthMapsWithTemporalFilter(depthMaps, params.outlierSigma);

        if (result.depthMap.empty()) {
            result.success = false;
            result.errorMessage = "Failed to fuse depth maps";
            pImpl->scanning_ = false;
            return result;
        }

        // ========== PHASE 5: GENERATE POINT CLOUD ==========
        pImpl->logger_.info("PHASE 5: Generating point cloud...");
        pImpl->reportProgress(0.9f, "Generating point cloud...");

        result.pointCloud = generatePointCloud(result.depthMap, frames[0].leftImage);

        result.processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - processingStart);

        // ========== CALCULATE METRICS ==========
        result.achievedPrecisionMM = calculatePrecision(depthMaps);

        int validPixels = cv::countNonZero(result.depthMap > 0);
        int totalPixels = result.depthMap.rows * result.depthMap.cols;
        result.validPixelPercentage = (validPixels * 100) / totalPixels;

        result.framesUsed = depthMaps.size();

        result.scanDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime);

        // Update statistics
        pImpl->updateStats("last_scan_duration_ms", result.scanDuration.count());
        pImpl->updateStats("last_scan_frames", result.framesCaptures);
        pImpl->updateStats("last_scan_precision_mm", result.achievedPrecisionMM);
        pImpl->updateStats("last_scan_valid_pixels_percent", result.validPixelPercentage);

        result.success = true;

        pImpl->logger_.info("✓ Scan completed successfully in " +
                           std::to_string(result.scanDuration.count()) + "ms");

        // ========== SAVE DEBUG OUTPUT ==========
        if (pImpl->debugConfig_.enabled) {
            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            std::stringstream timestamp;
            timestamp << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");

            saveDebugOutput(pImpl->debugConfig_.outputDir + "/" + timestamp.str(),
                           frames, depthMaps, result.depthMap, result.pointCloud);
        }

        pImpl->reportProgress(1.0f, "Scan complete!");

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "Exception during scan: " + std::string(e.what());
        pImpl->logger_.error(result.errorMessage);
    }

    pImpl->scanning_ = false;
    pImpl->currentProgressCallback_ = nullptr;
    return result;
}

bool HandheldScanPipeline::waitForStability(StabilityCallback callback, int timeoutMs) {
    if (!pImpl->stabilityDetector_) {
        pImpl->logger_.warning("No stability detector available, proceeding without check");
        if (callback) {
            callback(1.0f);
        }
        return true;
    }

    auto startTime = std::chrono::steady_clock::now();
    pImpl->logger_.info("Waiting for IMU stability...");

    while (std::chrono::steady_clock::now() - startTime < std::chrono::milliseconds(timeoutMs)) {
        pImpl->stabilityDetector_->update();

        float stabilityScore = pImpl->stabilityDetector_->getStabilityScore();

        if (callback) {
            callback(stabilityScore);
        }

        if (pImpl->stabilityDetector_->isStable() &&
            pImpl->stabilityDetector_->getStableDuration() >= 500) {
            pImpl->logger_.info("Stability achieved (score: " + std::to_string(stabilityScore) + ")");
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    pImpl->logger_.warning("Stability timeout after " + std::to_string(timeoutMs) + "ms");
    return false;
}

std::vector<HandheldScanPipeline::StereoFrame> HandheldScanPipeline::captureMultiFrame(
    int numFrames,
    ProgressCallback progressCallback) {

    std::vector<StereoFrame> frames;
    frames.reserve(numFrames);

    pImpl->logger_.info("Capturing " + std::to_string(numFrames) + " frames...");

    if (!pImpl->cameraSystem_ || !pImpl->cameraSystem_->isInitialized()) {
        pImpl->logger_.error("Camera system not available");
        return frames;
    }

    for (int i = 0; i < numFrames; ++i) {
        if (progressCallback) {
            progressCallback(static_cast<float>(i) / numFrames,
                           "Capturing frame " + std::to_string(i + 1) + "/" + std::to_string(numFrames));
        }

        StereoFrame frame;
        camera::StereoFrame cameraFrame;
        bool success = pImpl->cameraSystem_->captureStereoFrame(cameraFrame, 2000);

        if (!success || cameraFrame.leftImage.empty() || cameraFrame.rightImage.empty()) {
            pImpl->logger_.warning("Frame capture failed at frame " + std::to_string(i));
            continue;
        }

        // Resize to calibration resolution if needed
        if (cameraFrame.leftImage.cols == 1456 && cameraFrame.leftImage.rows == 1088) {
            cv::resize(cameraFrame.leftImage, frame.leftImage, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
            cv::resize(cameraFrame.rightImage, frame.rightImage, cv::Size(1280, 720), 0, 0, cv::INTER_AREA);
            pImpl->logger_.debug("Resized from 1456x1088 to 1280x720");
        } else {
            frame.leftImage = cameraFrame.leftImage.clone();
            frame.rightImage = cameraFrame.rightImage.clone();
        }

        frame.timestampUs = cameraFrame.leftTimestampNs / 1000;
        frame.leftVCSEL = frame.leftImage;
        frame.rightVCSEL = frame.rightImage;

        if (pImpl->stabilityDetector_) {
            frame.stabilityScore = pImpl->stabilityDetector_->getStabilityScore();
        } else {
            frame.stabilityScore = 1.0f;
        }

        frames.push_back(frame);

        if (i < numFrames - 1) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    pImpl->logger_.info("Captured " + std::to_string(frames.size()) + "/" +
                       std::to_string(numFrames) + " frames successfully");

    return frames;
}

std::vector<cv::Mat> HandheldScanPipeline::processFrames(
    const std::vector<StereoFrame>& frames,
    const stereo::StereoMatchingParams& params,
    ProgressCallback progressCallback) {

    std::vector<cv::Mat> depthMaps;
    depthMaps.reserve(frames.size());

    pImpl->logger_.info("Processing " + std::to_string(frames.size()) + " frames...");

    for (size_t i = 0; i < frames.size(); ++i) {
        if (progressCallback) {
            float progress = static_cast<float>(i) / static_cast<float>(frames.size());
            progressCallback(progress, "Processing frame " + std::to_string(i+1) +
                           "/" + std::to_string(frames.size()));
        }

        cv::Mat depth = pImpl->processFrameWithNewBackend(frames[i]);
        if (!depth.empty()) {
            depthMaps.push_back(depth);
        }
    }

    pImpl->logger_.info("Processed " + std::to_string(depthMaps.size()) + " depth maps");
    return depthMaps;
}

cv::Mat HandheldScanPipeline::fuseDepthMaps(const std::vector<cv::Mat>& depthMaps,
                                            float outlierSigma) {
    return pImpl->fuseDepthMapsWithTemporalFilter(depthMaps, outlierSigma);
}

cv::Mat HandheldScanPipeline::wlsFilter(const cv::Mat& depthMap,
                                        const cv::Mat& guideImage,
                                        double lambda,
                                        double sigma) {
    if (depthMap.empty() || guideImage.empty()) {
        pImpl->logger_.warning("Empty input for WLS filter");
        return depthMap;
    }

    pImpl->logger_.info("Applying WLS filter...");

    cv::Mat filtered;
    try {
        cv::Mat disparity;
        depthMap.convertTo(disparity, CV_16S, 16.0);

        auto wlsFilter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
        wlsFilter->setLambda(lambda);
        wlsFilter->setSigmaColor(sigma);

        cv::Mat guide = guideImage;
        if (guide.channels() > 1) {
            cv::cvtColor(guide, guide, cv::COLOR_BGR2GRAY);
        }

        cv::Mat filteredDisparity;
        wlsFilter->filter(disparity, guide, filteredDisparity);

        filteredDisparity.convertTo(filtered, CV_32F, 1.0/16.0);
        cv::threshold(filtered, filtered, 0, 0, cv::THRESH_TOZERO);

    } catch (const cv::Exception& e) {
        pImpl->logger_.error("WLS filter failed: " + std::string(e.what()));
        return depthMap;
    }

    return filtered;
}

cv::Mat HandheldScanPipeline::generatePointCloud(const cv::Mat& depthMap,
                                                 const cv::Mat& colorImage) {
    if (depthMap.empty()) {
        pImpl->logger_.warning("Empty depth map for point cloud generation");
        return cv::Mat();
    }

    pImpl->logger_.info("Generating point cloud...");

    float fx = 1000.0f, fy = 1000.0f;
    float cx = depthMap.cols / 2.0f;
    float cy = depthMap.rows / 2.0f;

    if (pImpl->calibrationManager_ && pImpl->calibrationManager_->isCalibrationValid()) {
        const auto& calibData = pImpl->calibrationManager_->getCalibrationData();
        if (!calibData.cameraMatrixLeft.empty()) {
            fx = calibData.cameraMatrixLeft.at<double>(0, 0);
            fy = calibData.cameraMatrixLeft.at<double>(1, 1);
            cx = calibData.cameraMatrixLeft.at<double>(0, 2);
            cy = calibData.cameraMatrixLeft.at<double>(1, 2);
        }
    }

    std::vector<cv::Point3f> points;
    std::vector<cv::Vec3b> colors;

    bool hasColor = !colorImage.empty() &&
                   colorImage.rows == depthMap.rows &&
                   colorImage.cols == depthMap.cols;

    for (int y = 0; y < depthMap.rows; ++y) {
        for (int x = 0; x < depthMap.cols; ++x) {
            float depth = depthMap.at<float>(y, x);

            if (depth > 0 && depth < 10000) {
                float X = (x - cx) * depth / fx;
                float Y = (y - cy) * depth / fy;
                float Z = depth;

                points.push_back(cv::Point3f(X, Y, Z));

                if (hasColor) {
                    if (colorImage.channels() == 3) {
                        colors.push_back(colorImage.at<cv::Vec3b>(y, x));
                    } else {
                        uchar gray = colorImage.at<uchar>(y, x);
                        colors.push_back(cv::Vec3b(gray, gray, gray));
                    }
                }
            }
        }
    }

    pImpl->logger_.info("Generated point cloud with " + std::to_string(points.size()) + " points");

    cv::Mat pointCloud;
    if (hasColor) {
        const int CV_32FC6 = CV_MAKETYPE(CV_32F, 6);
        pointCloud = cv::Mat(points.size(), 1, CV_32FC6);
        for (size_t i = 0; i < points.size(); ++i) {
            float* ptr = pointCloud.ptr<float>(i);
            ptr[0] = points[i].x;
            ptr[1] = points[i].y;
            ptr[2] = points[i].z;
            ptr[3] = colors[i][2] / 255.0f;
            ptr[4] = colors[i][1] / 255.0f;
            ptr[5] = colors[i][0] / 255.0f;
        }
    } else {
        pointCloud = cv::Mat(points.size(), 1, CV_32FC3);
        for (size_t i = 0; i < points.size(); ++i) {
            float* ptr = pointCloud.ptr<float>(i);
            ptr[0] = points[i].x;
            ptr[1] = points[i].y;
            ptr[2] = points[i].z;
        }
    }

    return pointCloud;
}

float HandheldScanPipeline::calculatePrecision(const std::vector<cv::Mat>& depthMaps) {
    if (depthMaps.size() < 2) {
        return 0.0f;
    }

    pImpl->logger_.info("Calculating precision from " + std::to_string(depthMaps.size()) + " depth maps");

    int sampleStep = 10;
    std::vector<float> variances;

    for (int y = 0; y < depthMaps[0].rows; y += sampleStep) {
        for (int x = 0; x < depthMaps[0].cols; x += sampleStep) {
            std::vector<float> values;

            for (const auto& depth : depthMaps) {
                float val = depth.at<float>(y, x);
                if (val > 0 && val < 10000) {
                    values.push_back(val);
                }
            }

            if (values.size() >= 2) {
                float mean = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
                float variance = 0.0f;
                for (float val : values) {
                    float diff = val - mean;
                    variance += diff * diff;
                }
                variance /= values.size();
                variances.push_back(std::sqrt(variance));
            }
        }
    }

    if (variances.empty()) {
        return 0.0f;
    }

    std::sort(variances.begin(), variances.end());
    float medianStdDev = variances[variances.size() / 2];

    float improvementFactor = std::sqrt(depthMaps.size());
    float estimatedPrecision = medianStdDev / improvementFactor;

    pImpl->logger_.info("Estimated precision: " + std::to_string(estimatedPrecision) + "mm");

    return estimatedPrecision;
}

void HandheldScanPipeline::setStereoAlgorithm(stereo::StereoAlgorithm algorithm) {
    // Map to new DisparityMethod
    switch(algorithm) {
        case stereo::StereoAlgorithm::SGBM:
            pImpl->disparityConfig_.method = stereo::DisparityMethod::SGBM_OPENCV;
            break;
        case stereo::StereoAlgorithm::CENSUS:
            // Using CENSUS instead of AD_CENSUS which doesn't exist
            pImpl->disparityConfig_.method = stereo::DisparityMethod::AD_CENSUS_CPU;
            break;
        default:
            pImpl->disparityConfig_.method = stereo::DisparityMethod::AUTO;
    }

    if (pImpl->disparityComputer_) {
        pImpl->disparityComputer_->setConfig(pImpl->disparityConfig_);
    }
}

stereo::StereoMatchingParams HandheldScanPipeline::getStereoParams() const {
    // Convert from new config to old params for compatibility
    stereo::StereoMatchingParams params;
    params.minDisparity = pImpl->disparityConfig_.minDisparity;
    params.numDisparities = pImpl->disparityConfig_.numDisparities;
    params.blockSize = pImpl->disparityConfig_.blockSize;
    params.P1 = pImpl->disparityConfig_.P1;
    params.P2 = pImpl->disparityConfig_.P2;
    params.uniquenessRatio = pImpl->disparityConfig_.uniquenessRatio;
    params.speckleWindowSize = pImpl->disparityConfig_.speckleWindowSize;
    params.speckleRange = pImpl->disparityConfig_.speckleRange;
    params.disp12MaxDiff = pImpl->disparityConfig_.disp12MaxDiff;
    params.preFilterCap = pImpl->disparityConfig_.preFilterCap;
    params.mode = pImpl->disparityConfig_.mode;
    params.useParallel = true;
    params.numThreads = pImpl->disparityConfig_.numThreads;
    return params;
}

void HandheldScanPipeline::setStereoParams(const stereo::StereoMatchingParams& params) {
    // Convert from old params to new config
    pImpl->disparityConfig_.minDisparity = params.minDisparity;
    pImpl->disparityConfig_.numDisparities = params.numDisparities;
    pImpl->disparityConfig_.blockSize = params.blockSize;
    pImpl->disparityConfig_.P1 = params.P1;
    pImpl->disparityConfig_.P2 = params.P2;
    pImpl->disparityConfig_.uniquenessRatio = params.uniquenessRatio;
    pImpl->disparityConfig_.speckleWindowSize = params.speckleWindowSize;
    pImpl->disparityConfig_.speckleRange = params.speckleRange;
    pImpl->disparityConfig_.disp12MaxDiff = params.disp12MaxDiff;
    pImpl->disparityConfig_.preFilterCap = params.preFilterCap;
    pImpl->disparityConfig_.mode = params.mode;
    pImpl->disparityConfig_.numThreads = params.numThreads;

    if (pImpl->disparityComputer_) {
        pImpl->disparityComputer_->setConfig(pImpl->disparityConfig_);
    }
}

void HandheldScanPipeline::enableVCSEL(bool enable) {
    pImpl->vcselEnabled_ = enable;
    pImpl->logger_.info("VCSEL " + std::string(enable ? "enabled" : "disabled"));
}

void HandheldScanPipeline::setDebugOutput(bool enable, const std::string& debugDir) {
    pImpl->debugConfig_.enabled = enable;
    pImpl->debugConfig_.outputDir = debugDir;
    pImpl->frameCounter_ = 0;

    if (pImpl->debugManager_) {
        pImpl->debugManager_->setConfig(pImpl->debugConfig_);
    }

    if (enable) {
        // Create output directory
        std::filesystem::create_directories(debugDir);
        pImpl->logger_.info("Debug output enabled: " + debugDir);
    } else {
        pImpl->logger_.info("Debug output disabled");
    }
}

std::map<std::string, double> HandheldScanPipeline::getStatistics() const {
    std::lock_guard<std::mutex> lock(pImpl->statsMutex_);
    return pImpl->statistics_;
}

bool HandheldScanPipeline::saveDebugOutput(const std::string& debugDir,
                                           const std::vector<StereoFrame>& frames,
                                           const std::vector<cv::Mat>& depthMaps,
                                           const cv::Mat& fusedDepth,
                                           const cv::Mat& pointCloud) {
    pImpl->logger_.info("Saving debug output to: " + debugDir);

    try {
        std::filesystem::create_directories(debugDir);

        // Use debug manager if available
        if (pImpl->debugManager_) {
            stereo::DebugOutputManager::ProcessingMetrics metrics;
            metrics.inputSize = frames[0].leftImage.size();
            metrics.calibSize = pImpl->calibImageSize_;

            if (!fusedDepth.empty()) {
                int validPixels = cv::countNonZero(fusedDepth > 0);
                int totalPixels = fusedDepth.rows * fusedDepth.cols;
                metrics.validPixelPercentage = (validPixels * 100.0f) / totalPixels;

                cv::Scalar meanVal, stdVal;
                cv::meanStdDev(fusedDepth, meanVal, stdVal, fusedDepth > 0);
                metrics.meanDepthMM = meanVal[0];
                metrics.depthStdDevMM = stdVal[0];
            }

            metrics.gpuUsed = pImpl->disparityComputer_ && pImpl->disparityComputer_->isGPUAvailable();

            // Save report
            pImpl->debugManager_->saveProcessingReport(metrics);
        }

        // Save individual depth maps
        for (size_t i = 0; i < depthMaps.size(); ++i) {
            if (depthMaps[i].empty()) continue;

            cv::Mat depthVis;
            cv::normalize(depthMaps[i], depthVis, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);

            std::stringstream ss;
            ss << std::setw(2) << std::setfill('0') << i;
            std::string depthPath = debugDir + "/03_depth_frame" + ss.str() + ".png";
            cv::imwrite(depthPath, depthVis);
        }

        // Save fused depth
        if (!fusedDepth.empty()) {
            cv::Mat fusedVis;
            cv::normalize(fusedDepth, fusedVis, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(fusedVis, fusedVis, cv::COLORMAP_JET);

            cv::imwrite(debugDir + "/04_depth_fused.png", fusedVis);
            cv::imwrite(debugDir + "/04_depth_fused_raw.tiff", fusedDepth);
        }

        // Save point cloud as PLY
        if (!pointCloud.empty()) {
            std::string plyPath = debugDir + "/05_pointcloud.ply";
            std::ofstream ply(plyPath);

            if (ply.is_open()) {
                int numPoints = pointCloud.rows;
                bool hasColor = (pointCloud.channels() == 6);

                ply << "ply\n";
                ply << "format ascii 1.0\n";
                ply << "element vertex " << numPoints << "\n";
                ply << "property float x\n";
                ply << "property float y\n";
                ply << "property float z\n";
                if (hasColor) {
                    ply << "property uchar red\n";
                    ply << "property uchar green\n";
                    ply << "property uchar blue\n";
                }
                ply << "end_header\n";

                for (int i = 0; i < numPoints; ++i) {
                    const float* ptr = pointCloud.ptr<float>(i);
                    ply << ptr[0] << " " << ptr[1] << " " << ptr[2];

                    if (hasColor) {
                        int r = static_cast<int>(ptr[3] * 255.0f);
                        int g = static_cast<int>(ptr[4] * 255.0f);
                        int b = static_cast<int>(ptr[5] * 255.0f);
                        ply << " " << r << " " << g << " " << b;
                    }

                    ply << "\n";
                }

                ply.close();
                pImpl->logger_.info("Saved point cloud: " + plyPath);
            }
        }

        pImpl->logger_.info("Debug output saved successfully");
        return true;

    } catch (const std::exception& e) {
        pImpl->logger_.error("Failed to save debug output: " + std::string(e.what()));
        return false;
    }
}

} // namespace api
} // namespace unlook