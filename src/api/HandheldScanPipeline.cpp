#include <unlook/api/HandheldScanPipeline.hpp>
#include <unlook/api/camera_system.h>
#include <unlook/core/Logger.hpp>
#include <unlook/core/exception.h>
#include <unlook/stereo/SGBMStereoMatcher.hpp>
#include <unlook/stereo/TemporalStereoProcessor.hpp>
#include <unlook/calibration/CalibrationManager.hpp>
#include <unlook/hardware/AS1170DualVCSELController.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <numeric>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <omp.h>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace unlook {
namespace api {

/**
 * @brief Stub IMU stability detector (will be replaced by hardware-interface-controller)
 */
class StabilityDetector {
public:
    StabilityDetector() : stabilityScore_(0.0f), isStable_(false) {}

    /**
     * @brief Wait for stability with callback
     * @param callback Callback for stability updates
     * @param timeoutMs Timeout in milliseconds
     * @return true if stable achieved, false if timeout
     */
    bool waitForStability(std::function<void(float)> callback, int timeoutMs) {
        auto start = std::chrono::steady_clock::now();

        // Simulate gradual stability achievement
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(timeoutMs)) {
            // Simulate IMU readings gradually stabilizing
            stabilityScore_.store(stabilityScore_.load() + 0.1f);
            if (stabilityScore_ > 1.0f) stabilityScore_ = 1.0f;

            if (callback) {
                callback(stabilityScore_);
            }

            if (stabilityScore_ >= 0.95f) {
                isStable_ = true;
                return true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return false;
    }

    float getStabilityScore() const { return stabilityScore_; }
    bool isStable() const { return isStable_; }

private:
    std::atomic<float> stabilityScore_;
    std::atomic<bool> isStable_;
};

/**
 * @brief Private implementation class
 */
class HandheldScanPipeline::Impl {
public:
    // Camera system
    std::shared_ptr<camera::CameraSystem> cameraSystem_;

    // Stereo processor
    std::unique_ptr<stereo::TemporalStereoProcessor> temporalProcessor_;
    std::unique_ptr<stereo::SGBMStereoMatcher> sgbmMatcher_;

    // VCSEL controller
    std::unique_ptr<hardware::AS1170DualVCSELController> vcselController_;

    // IMU stability detector
    std::unique_ptr<StabilityDetector> stabilityDetector_;

    // Calibration
    std::shared_ptr<calibration::CalibrationManager> calibrationManager_;

    // Configuration
    stereo::StereoMatchingParams stereoParams_;
    stereo::StereoAlgorithm currentAlgorithm_ = stereo::StereoAlgorithm::SGBM;
    bool vcselEnabled_ = true;

    // Statistics
    mutable std::mutex statsMutex_;
    std::map<std::string, double> statistics_;

    // State
    std::atomic<bool> initialized_{false};
    std::atomic<bool> scanning_{false};

    // Camera parameters (from calibration)
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;
    cv::Mat R1_, R2_, P1_, P2_, Q_;
    float baseline_mm_ = 70.017f;  // From calibration

    // Logger
    core::Logger& logger_ = core::Logger::getInstance();

    Impl(std::shared_ptr<camera::CameraSystem> cameraSystem)
        : cameraSystem_(cameraSystem) {

        // Initialize stereo parameters for VCSEL
        stereoParams_.blockSize = 5;
        stereoParams_.numDisparities = 160;
        stereoParams_.minDisparity = 0;
        stereoParams_.P1 = 8 * 3 * 5 * 5;
        stereoParams_.P2 = 32 * 3 * 5 * 5;
        stereoParams_.uniquenessRatio = 15;
        stereoParams_.speckleWindowSize = 50;
        stereoParams_.speckleRange = 16;
        stereoParams_.disp12MaxDiff = 1;
        stereoParams_.preFilterCap = 31;
        stereoParams_.mode = 3;  // MODE_HH4

        // Initialize components
        stabilityDetector_ = std::make_unique<StabilityDetector>();
        temporalProcessor_ = std::make_unique<stereo::TemporalStereoProcessor>();
        sgbmMatcher_ = std::make_unique<stereo::SGBMStereoMatcher>();

        // Initialize VCSEL controller if available
        try {
            vcselController_ = std::make_unique<hardware::AS1170DualVCSELController>();
            // TODO: Fix type mismatch - AS1170DualVCSELController expects camera::CameraSystem
            // For now, skip VCSEL initialization in handheld mode
            logger_.warning("VCSEL controller initialization skipped (API mismatch), continuing without VCSEL");
            vcselController_.reset();
            vcselEnabled_ = false;
        } catch (const std::exception& e) {
            logger_.warning("VCSEL not available: " + std::string(e.what()));
            vcselEnabled_ = false;
        }

        // Load system default calibration
        calibrationManager_ = std::make_shared<calibration::CalibrationManager>();
        std::string defaultCalibPath = "/unlook_calib/default.yaml";
        if (!calibrationManager_->loadCalibration(defaultCalibPath)) {
            logger_.warning("Failed to load system default calibration from " + defaultCalibPath + ", using fallback");
            // Try fallback to old location
            if (!calibrationManager_->loadCalibration("/home/alessandro/unlook-standalone/calibration/calib_boofcv_test3.yaml")) {
                logger_.error("Failed to load any calibration - scan results will be incorrect!");
            }
        } else {
            logger_.info("Loaded system default calibration: " + defaultCalibPath);
        }
    }

    /**
     * @brief Calculate weighted median
     */
    float weightedMedian(std::vector<float>& values) {
        if (values.empty()) return 0.0f;

        // Sort values
        std::sort(values.begin(), values.end());

        // Return median
        size_t n = values.size();
        if (n % 2 == 0) {
            return (values[n/2 - 1] + values[n/2]) / 2.0f;
        } else {
            return values[n/2];
        }
    }

    /**
     * @brief Process single frame to depth map
     */
    cv::Mat processFrame(const StereoFrame& frame, const stereo::StereoMatchingParams& params) {
        cv::Mat disparity, depth;

        // For now, always use regular SGBM (temporal processor requires proper TripleFrameCapture)
        // TODO: Implement proper temporal processing with vcselController_->captureTemporalSequence()

        // Use regular SGBM
        sgbmMatcher_->setParameters(params);
        if (!sgbmMatcher_->computeDisparity(frame.leftImage, frame.rightImage, disparity)) {
            logger_.error("Failed to compute disparity");
            return cv::Mat();
        }

        // Convert disparity to depth
        if (!disparity.empty() && Q_.rows == 4 && Q_.cols == 4) {
            cv::reprojectImageTo3D(disparity, depth, Q_, true);

            // Extract Z channel as depth map
            std::vector<cv::Mat> channels;
            cv::split(depth, channels);
            depth = channels[2];  // Z channel
        }

        return depth;
    }

    /**
     * @brief Update statistics
     */
    void updateStats(const std::string& key, double value) {
        std::lock_guard<std::mutex> lock(statsMutex_);
        statistics_[key] = value;
    }
};

// Constructor
HandheldScanPipeline::HandheldScanPipeline(std::shared_ptr<camera::CameraSystem> cameraSystem)
    : pImpl(std::make_unique<Impl>(cameraSystem)) {
}

// Destructor
HandheldScanPipeline::~HandheldScanPipeline() {
    shutdown();
}

// Initialize
bool HandheldScanPipeline::initialize() {
    if (pImpl->initialized_) {
        return true;
    }

    pImpl->logger_.info("Initializing HandheldScanPipeline...");

    // Initialize camera system if not already initialized
    if (!pImpl->cameraSystem_) {
        pImpl->logger_.error("Camera system not provided");
        return false;
    }

    // Initialize stereo matcher
    if (pImpl->sgbmMatcher_) {
        pImpl->sgbmMatcher_->setParameters(pImpl->stereoParams_);
    }

    // Initialize temporal processor
    if (pImpl->temporalProcessor_) {
        auto config = stereo::TemporalStereoProcessor::TemporalStereoConfig::getVCSELOptimized();
        config.sgbmParams = pImpl->stereoParams_;
        // Temporal processor initialized with config
    }

    pImpl->initialized_ = true;
    pImpl->logger_.info("HandheldScanPipeline initialized successfully");

    return true;
}

// Shutdown
void HandheldScanPipeline::shutdown() {
    if (!pImpl->initialized_) {
        return;
    }

    pImpl->logger_.info("Shutting down HandheldScanPipeline...");

    // Stop any ongoing scans
    pImpl->scanning_ = false;

    // Shutdown VCSEL if active
    if (pImpl->vcselController_) {
        pImpl->vcselController_->shutdown();
    }

    pImpl->initialized_ = false;
    pImpl->logger_.info("HandheldScanPipeline shutdown complete");
}

// Main scanning function with stability detection
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

    try {
        // Phase 1: Wait for stability
        pImpl->logger_.info("Phase 1: Waiting for IMU stability...");
        if (progressCallback) {
            progressCallback(0.1f, "Waiting for device stability...");
        }

        auto stabilityStart = std::chrono::steady_clock::now();

        bool stable = waitForStability(
            [&progressCallback](float score) {
                if (progressCallback) {
                    progressCallback(0.1f + score * 0.2f,
                        "Stability: " + std::to_string(static_cast<int>(score * 100)) + "%");
                }
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

        // Phase 2: Multi-frame capture
        pImpl->logger_.info("Phase 2: Capturing " + std::to_string(params.numFrames) + " frames...");
        if (progressCallback) {
            progressCallback(0.3f, "Capturing frames...");
        }

        auto captureStart = std::chrono::steady_clock::now();

        auto frames = captureMultiFrame(params.numFrames,
            [&progressCallback, &params](float progress, const std::string& msg) {
                if (progressCallback) {
                    progressCallback(0.3f + progress * 0.2f, msg);
                }
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

        // Phase 3: Process frames to depth maps
        pImpl->logger_.info("Phase 3: Processing frames with stereo matching...");
        if (progressCallback) {
            progressCallback(0.5f, "Processing stereo depth...");
        }

        auto processingStart = std::chrono::steady_clock::now();

        auto depthMaps = processFrames(frames, pImpl->stereoParams_);

        if (depthMaps.empty()) {
            result.success = false;
            result.errorMessage = "Failed to process frames";
            pImpl->scanning_ = false;
            return result;
        }

        // Phase 4: Multi-frame fusion
        pImpl->logger_.info("Phase 4: Fusing " + std::to_string(depthMaps.size()) + " depth maps...");
        if (progressCallback) {
            progressCallback(0.7f, "Fusing depth maps...");
        }

        result.depthMap = fuseDepthMaps(depthMaps, params.outlierSigma);

        if (result.depthMap.empty()) {
            result.success = false;
            result.errorMessage = "Failed to fuse depth maps";
            pImpl->scanning_ = false;
            return result;
        }

        // Phase 5: WLS filtering
        if (params.useWLSFilter) {
            pImpl->logger_.info("Phase 5: Applying WLS filter...");
            if (progressCallback) {
                progressCallback(0.85f, "Applying edge-preserving filter...");
            }

            // Use first frame's left image as guide
            cv::Mat guideImage = frames[0].leftVCSEL.empty() ? frames[0].leftImage : frames[0].leftVCSEL;
            result.depthMap = wlsFilter(result.depthMap, guideImage, params.wlsLambda, params.wlsSigma);
        }

        // Phase 6: Generate point cloud
        pImpl->logger_.info("Phase 6: Generating point cloud...");
        if (progressCallback) {
            progressCallback(0.95f, "Generating point cloud...");
        }

        result.pointCloud = generatePointCloud(result.depthMap, frames[0].leftImage);

        result.processingTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - processingStart);

        // Calculate quality metrics
        result.achievedPrecisionMM = calculatePrecision(depthMaps);

        // Count valid pixels
        int validPixels = cv::countNonZero(result.depthMap > 0);
        int totalPixels = result.depthMap.rows * result.depthMap.cols;
        result.validPixelPercentage = (validPixels * 100) / totalPixels;

        // Calculate average confidence
        if (!result.confidenceMap.empty()) {
            cv::Scalar meanConfidence = cv::mean(result.confidenceMap, result.depthMap > 0);
            result.avgConfidence = meanConfidence[0];
        }

        result.framesUsed = depthMaps.size();

        // Total scan duration
        result.scanDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime);

        // Update statistics
        pImpl->updateStats("last_scan_duration_ms", result.scanDuration.count());
        pImpl->updateStats("last_scan_frames", result.framesCaptures);
        pImpl->updateStats("last_scan_precision_mm", result.achievedPrecisionMM);
        pImpl->updateStats("last_scan_valid_pixels_percent", result.validPixelPercentage);

        result.success = true;

        pImpl->logger_.info("Scan completed successfully in " +
                           std::to_string(result.scanDuration.count()) + "ms");

        // Save debug output with timestamp
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::stringstream timestamp;
        timestamp << std::put_time(std::localtime(&time_t_now), "%Y%m%d_%H%M%S");

        bool debugSaved = saveDebugOutput(timestamp.str(), frames, depthMaps,
                                          result.depthMap, result.pointCloud);

        if (debugSaved) {
            pImpl->logger_.info("Debug output saved to /unlook_debug/");
        } else {
            pImpl->logger_.warning("Failed to save debug output (non-critical)");
        }

        if (progressCallback) {
            progressCallback(1.0f, "Scan complete!");
        }

    } catch (const std::exception& e) {
        result.success = false;
        result.errorMessage = "Exception during scan: " + std::string(e.what());
        pImpl->logger_.error(result.errorMessage);
    }

    pImpl->scanning_ = false;
    return result;
}

// Wait for stability
bool HandheldScanPipeline::waitForStability(StabilityCallback callback, int timeoutMs) {
    if (!pImpl->stabilityDetector_) {
        pImpl->logger_.warning("No stability detector available, proceeding without stability check");
        return true;
    }

    return pImpl->stabilityDetector_->waitForStability(callback, timeoutMs);
}

// Capture multiple frames
std::vector<HandheldScanPipeline::StereoFrame> HandheldScanPipeline::captureMultiFrame(
    int numFrames,
    ProgressCallback progressCallback) {

    std::vector<StereoFrame> frames;
    frames.reserve(numFrames);

    pImpl->logger_.info("Capturing " + std::to_string(numFrames) + " frames using camera system...");

    // Ensure camera system is initialized
    if (!pImpl->cameraSystem_->isInitialized()) {
        pImpl->logger_.error("Camera system not initialized");
        return frames;
    }

    // Capture frames one by one
    for (int i = 0; i < numFrames; ++i) {
        if (progressCallback) {
            progressCallback(static_cast<float>(i) / numFrames,
                           "Capturing frame " + std::to_string(i + 1) + "/" + std::to_string(numFrames));
        }

        StereoFrame frame;

        // Capture single synchronized frame from camera system
        camera::StereoFrame cameraFrame;
        bool success = pImpl->cameraSystem_->captureStereoFrame(cameraFrame, 2000);

        if (!success) {
            pImpl->logger_.warning("Frame capture failed at frame " + std::to_string(i));
            continue;  // Skip this frame
        }

        // Verify frames are not empty
        if (cameraFrame.leftImage.empty() || cameraFrame.rightImage.empty()) {
            pImpl->logger_.warning("Empty frames received at frame " + std::to_string(i));
            continue;
        }

        // Copy frame data (convert camera::StereoFrame to api::StereoFrame)
        frame.leftImage = cameraFrame.leftImage.clone();
        frame.rightImage = cameraFrame.rightImage.clone();
        frame.timestampUs = cameraFrame.leftTimestampNs / 1000;  // ns to us

        // For now, VCSEL is same as regular image (VCSEL controller integration TODO)
        frame.leftVCSEL = frame.leftImage;
        frame.rightVCSEL = frame.rightImage;

        // Get stability score
        if (pImpl->stabilityDetector_) {
            frame.stabilityScore = pImpl->stabilityDetector_->getStabilityScore();
        } else {
            frame.stabilityScore = 1.0f;
        }

        frames.push_back(frame);

        pImpl->logger_.debug("Captured frame " + std::to_string(i + 1) + "/" + std::to_string(numFrames) +
                            " - Size: " + std::to_string(cameraFrame.leftImage.cols) + "x" + std::to_string(cameraFrame.leftImage.rows));

        // Small delay between frames for ~10 FPS
        if (i < numFrames - 1) {  // Don't delay after last frame
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    pImpl->logger_.info("Captured " + std::to_string(frames.size()) + "/" +
                       std::to_string(numFrames) + " frames successfully");

    return frames;
}

// Process frames to depth maps
std::vector<cv::Mat> HandheldScanPipeline::processFrames(
    const std::vector<StereoFrame>& frames,
    const stereo::StereoMatchingParams& params) {

    std::vector<cv::Mat> depthMaps;
    depthMaps.reserve(frames.size());

    pImpl->logger_.info("Processing " + std::to_string(frames.size()) + " frames...");

    // Process frames in parallel if enabled
    if (params.useParallel && frames.size() > 1) {
        // Use OpenMP for parallel processing
        std::vector<cv::Mat> tempMaps(frames.size());

        #pragma omp parallel for num_threads(params.numThreads)
        for (size_t i = 0; i < frames.size(); ++i) {
            tempMaps[i] = pImpl->processFrame(frames[i], params);
        }

        // Copy to output vector
        for (const auto& map : tempMaps) {
            if (!map.empty()) {
                depthMaps.push_back(map);
            }
        }
    } else {
        // Sequential processing
        for (const auto& frame : frames) {
            cv::Mat depth = pImpl->processFrame(frame, params);
            if (!depth.empty()) {
                depthMaps.push_back(depth);
            }
        }
    }

    pImpl->logger_.info("Processed " + std::to_string(depthMaps.size()) + " depth maps");

    return depthMaps;
}

// Fuse depth maps with outlier rejection
cv::Mat HandheldScanPipeline::fuseDepthMaps(const std::vector<cv::Mat>& depthMaps,
                                            float outlierSigma) {
    if (depthMaps.empty()) {
        throw core::Exception(core::ResultCode::ERROR_INVALID_PARAMETER, "No depth maps to fuse");
    }

    pImpl->logger_.info("Fusing " + std::to_string(depthMaps.size()) +
                       " depth maps with outlier sigma=" + std::to_string(outlierSigma));

    // Initialize fused map with same size as first depth map
    cv::Mat fused = cv::Mat::zeros(depthMaps[0].size(), CV_32F);
    cv::Mat countMap = cv::Mat::zeros(depthMaps[0].size(), CV_32S);

    int height = fused.rows;
    int width = fused.cols;

    // Statistics
    int totalPixels = 0;
    int rejectedPixels = 0;

    // Process each pixel with OpenMP parallelization
    #pragma omp parallel for collapse(2) reduction(+:totalPixels,rejectedPixels)
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::vector<float> values;
            values.reserve(depthMaps.size());

            // Collect all valid depth values for this pixel
            for (const auto& depth : depthMaps) {
                float val = depth.at<float>(y, x);
                if (val > 0 && val < 10000) {  // Valid depth range (0-10m)
                    values.push_back(val);
                }
            }

            if (values.empty()) {
                fused.at<float>(y, x) = 0.0f;
                continue;
            }

            totalPixels++;

            if (values.size() == 1) {
                // Only one value, use it directly
                fused.at<float>(y, x) = values[0];
                countMap.at<int>(y, x) = 1;
                continue;
            }

            // Calculate mean and standard deviation
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
                } else {
                    rejectedPixels++;
                }
            }

            if (inliers.empty()) {
                // All values were outliers
                fused.at<float>(y, x) = 0.0f;
                continue;
            }

            // Compute weighted median of inliers
            float medianValue = pImpl->weightedMedian(inliers);
            fused.at<float>(y, x) = medianValue;
            countMap.at<int>(y, x) = inliers.size();
        }
    }

    // Log fusion statistics
    float rejectionRate = (totalPixels > 0) ?
        (static_cast<float>(rejectedPixels) / (totalPixels * depthMaps.size())) * 100 : 0;

    pImpl->logger_.info("Fusion complete - Rejection rate: " +
                       std::to_string(rejectionRate) + "%");

    pImpl->updateStats("fusion_rejection_rate", rejectionRate);

    return fused;
}

// WLS filtering
cv::Mat HandheldScanPipeline::wlsFilter(const cv::Mat& depthMap,
                                        const cv::Mat& guideImage,
                                        double lambda,
                                        double sigma) {
    if (depthMap.empty() || guideImage.empty()) {
        pImpl->logger_.warning("Empty input for WLS filter, returning original");
        return depthMap;
    }

    pImpl->logger_.info("Applying WLS filter (lambda=" + std::to_string(lambda) +
                       ", sigma=" + std::to_string(sigma) + ")");

    cv::Mat filtered;

    try {
        // Convert depth to disparity-like format for WLS filter
        cv::Mat disparity;
        depthMap.convertTo(disparity, CV_16S, 16.0);

        // Create WLS filter
        auto wlsFilter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
        wlsFilter->setLambda(lambda);
        wlsFilter->setSigmaColor(sigma);

        // Ensure guide image is grayscale
        cv::Mat guide = guideImage;
        if (guide.channels() > 1) {
            cv::cvtColor(guide, guide, cv::COLOR_BGR2GRAY);
        }

        // Apply filter
        cv::Mat filteredDisparity;
        wlsFilter->filter(disparity, guide, filteredDisparity);

        // Convert back to depth
        filteredDisparity.convertTo(filtered, CV_32F, 1.0/16.0);

        // Remove negative values
        cv::threshold(filtered, filtered, 0, 0, cv::THRESH_TOZERO);

    } catch (const cv::Exception& e) {
        pImpl->logger_.error("WLS filter failed: " + std::string(e.what()));
        return depthMap;
    }

    return filtered;
}

// Generate point cloud
cv::Mat HandheldScanPipeline::generatePointCloud(const cv::Mat& depthMap,
                                                 const cv::Mat& colorImage) {
    if (depthMap.empty()) {
        pImpl->logger_.warning("Empty depth map for point cloud generation");
        return cv::Mat();
    }

    pImpl->logger_.info("Generating point cloud...");

    // Get camera parameters
    float fx = 1000.0f;  // Default focal length
    float fy = 1000.0f;
    float cx = depthMap.cols / 2.0f;
    float cy = depthMap.rows / 2.0f;

    // Use calibration if available
    if (!pImpl->cameraMatrix_.empty()) {
        fx = pImpl->cameraMatrix_.at<double>(0, 0);
        fy = pImpl->cameraMatrix_.at<double>(1, 1);
        cx = pImpl->cameraMatrix_.at<double>(0, 2);
        cy = pImpl->cameraMatrix_.at<double>(1, 2);
    }

    // Create point cloud
    std::vector<cv::Point3f> points;
    std::vector<cv::Vec3b> colors;

    bool hasColor = !colorImage.empty() &&
                   colorImage.rows == depthMap.rows &&
                   colorImage.cols == depthMap.cols;

    for (int y = 0; y < depthMap.rows; ++y) {
        for (int x = 0; x < depthMap.cols; ++x) {
            float depth = depthMap.at<float>(y, x);

            if (depth > 0 && depth < 10000) {  // Valid depth
                // Back-project to 3D
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

    // Create output matrix
    cv::Mat pointCloud;
    if (hasColor) {
        // Create 6-channel matrix (XYZ + RGB)
        // CV_32FC6 is not defined in OpenCV, use CV_MAKETYPE
        const int CV_32FC6 = CV_MAKETYPE(CV_32F, 6);
        pointCloud = cv::Mat(points.size(), 1, CV_32FC6);
        for (size_t i = 0; i < points.size(); ++i) {
            float* ptr = pointCloud.ptr<float>(i);
            ptr[0] = points[i].x;
            ptr[1] = points[i].y;
            ptr[2] = points[i].z;
            ptr[3] = colors[i][2] / 255.0f;  // R
            ptr[4] = colors[i][1] / 255.0f;  // G
            ptr[5] = colors[i][0] / 255.0f;  // B
        }
    } else {
        // Create 3-channel matrix (XYZ only)
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

// Calculate precision from depth variance
float HandheldScanPipeline::calculatePrecision(const std::vector<cv::Mat>& depthMaps) {
    if (depthMaps.size() < 2) {
        return 0.0f;
    }

    pImpl->logger_.info("Calculating precision from " + std::to_string(depthMaps.size()) + " depth maps");

    // Sample points across the image
    int sampleStep = 10;
    std::vector<float> variances;

    for (int y = 0; y < depthMaps[0].rows; y += sampleStep) {
        for (int x = 0; x < depthMaps[0].cols; x += sampleStep) {
            std::vector<float> values;

            // Collect depth values at this point from all maps
            for (const auto& depth : depthMaps) {
                float val = depth.at<float>(y, x);
                if (val > 0 && val < 10000) {
                    values.push_back(val);
                }
            }

            if (values.size() >= 2) {
                // Calculate variance
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

    // Calculate median standard deviation as precision estimate
    std::sort(variances.begin(), variances.end());
    float medianStdDev = variances[variances.size() / 2];

    // Multi-frame improvement factor
    float improvementFactor = std::sqrt(depthMaps.size());
    float estimatedPrecision = medianStdDev / improvementFactor;

    pImpl->logger_.info("Estimated precision: " + std::to_string(estimatedPrecision) + "mm");

    return estimatedPrecision;
}

// Set stereo algorithm
void HandheldScanPipeline::setStereoAlgorithm(stereo::StereoAlgorithm algorithm) {
    pImpl->currentAlgorithm_ = algorithm;
    pImpl->logger_.info("Stereo algorithm set to: " + std::to_string(static_cast<int>(algorithm)));
}

// Get stereo parameters
stereo::StereoMatchingParams HandheldScanPipeline::getStereoParams() const {
    return pImpl->stereoParams_;
}

// Set stereo parameters
void HandheldScanPipeline::setStereoParams(const stereo::StereoMatchingParams& params) {
    pImpl->stereoParams_ = params;

    // Update matcher parameters
    if (pImpl->sgbmMatcher_) {
        pImpl->sgbmMatcher_->setParameters(params);
    }

    pImpl->logger_.info("Stereo parameters updated");
}

// Enable/disable VCSEL
void HandheldScanPipeline::enableVCSEL(bool enable) {
    pImpl->vcselEnabled_ = enable;

    if (enable && !pImpl->vcselController_) {
        // Try to initialize VCSEL controller
        try {
            pImpl->vcselController_ = std::make_unique<hardware::AS1170DualVCSELController>();
            // TODO: Fix - AS1170DualVCSELController::initialize() requires camera system parameter
            pImpl->logger_.warning("VCSEL controller initialization skipped (API requires camera system)");
            pImpl->vcselController_.reset();
            pImpl->vcselEnabled_ = false;
        } catch (const std::exception& e) {
            pImpl->logger_.warning("VCSEL initialization failed: " + std::string(e.what()));
            pImpl->vcselEnabled_ = false;
        }
    }

    pImpl->logger_.info("VCSEL " + std::string(pImpl->vcselEnabled_ ? "enabled" : "disabled"));
}

// Get statistics
std::map<std::string, double> HandheldScanPipeline::getStatistics() const {
    std::lock_guard<std::mutex> lock(pImpl->statsMutex_);
    return pImpl->statistics_;
}

// Save debug output
bool HandheldScanPipeline::saveDebugOutput(const std::string& timestamp,
                                           const std::vector<StereoFrame>& frames,
                                           const std::vector<cv::Mat>& depthMaps,
                                           const cv::Mat& fusedDepth,
                                           const cv::Mat& pointCloud) {
    pImpl->logger_.info("Saving debug output to /unlook_debug/ with timestamp: " + timestamp);

    try {
        const std::string debugDir = "/unlook_debug";

        // Save captured frames (left/right images)
        for (size_t i = 0; i < frames.size(); ++i) {
            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << i;
            std::string frameNum = ss.str();

            // Save left image
            if (!frames[i].leftImage.empty()) {
                std::string leftPath = debugDir + "/frame_" + timestamp + "_" + frameNum + "_left.png";
                cv::imwrite(leftPath, frames[i].leftImage);
                pImpl->logger_.debug("Saved: " + leftPath);
            }

            // Save right image
            if (!frames[i].rightImage.empty()) {
                std::string rightPath = debugDir + "/frame_" + timestamp + "_" + frameNum + "_right.png";
                cv::imwrite(rightPath, frames[i].rightImage);
                pImpl->logger_.debug("Saved: " + rightPath);
            }

            // Save VCSEL images if available
            if (!frames[i].leftVCSEL.empty() &&
                !cv::countNonZero(frames[i].leftVCSEL == frames[i].leftImage)) {
                std::string vcselPath = debugDir + "/frame_" + timestamp + "_" + frameNum + "_vcsel_left.png";
                cv::imwrite(vcselPath, frames[i].leftVCSEL);
                pImpl->logger_.debug("Saved: " + vcselPath);
            }
        }

        // Save individual depth maps
        for (size_t i = 0; i < depthMaps.size(); ++i) {
            if (depthMaps[i].empty()) continue;

            std::stringstream ss;
            ss << std::setw(3) << std::setfill('0') << i;
            std::string depthNum = ss.str();

            // Normalize for visualization (0-255)
            cv::Mat depthVis;
            cv::normalize(depthMaps[i], depthVis, 0, 255, cv::NORM_MINMAX, CV_8U);

            std::string depthPath = debugDir + "/depth_" + timestamp + "_" + depthNum + ".png";
            cv::imwrite(depthPath, depthVis);
            pImpl->logger_.debug("Saved: " + depthPath);
        }

        // Save fused depth map
        if (!fusedDepth.empty()) {
            // Visualization (normalized)
            cv::Mat fusedVis;
            cv::normalize(fusedDepth, fusedVis, 0, 255, cv::NORM_MINMAX, CV_8U);
            cv::applyColorMap(fusedVis, fusedVis, cv::COLORMAP_JET);

            std::string fusedPath = debugDir + "/depth_fused_" + timestamp + ".png";
            cv::imwrite(fusedPath, fusedVis);
            pImpl->logger_.info("Saved fused depth visualization: " + fusedPath);

            // Save raw depth data as 32-bit float TIFF for analysis
            std::string rawDepthPath = debugDir + "/depth_fused_raw_" + timestamp + ".tiff";
            cv::imwrite(rawDepthPath, fusedDepth);
            pImpl->logger_.info("Saved raw depth data: " + rawDepthPath);
        }

        // Save point cloud as PLY
        if (!pointCloud.empty()) {
            std::string plyPath = debugDir + "/pointcloud_" + timestamp + ".ply";

            // Create PLY file
            std::ofstream ply(plyPath);
            if (!ply.is_open()) {
                pImpl->logger_.error("Failed to open PLY file: " + plyPath);
                return false;
            }

            int numPoints = pointCloud.rows;
            bool hasColor = (pointCloud.channels() == 6);

            // Write PLY header
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

            // Write points
            for (int i = 0; i < numPoints; ++i) {
                const float* ptr = pointCloud.ptr<float>(i);

                ply << ptr[0] << " " << ptr[1] << " " << ptr[2];

                if (hasColor) {
                    // RGB values (0-1) to (0-255)
                    int r = static_cast<int>(ptr[3] * 255.0f);
                    int g = static_cast<int>(ptr[4] * 255.0f);
                    int b = static_cast<int>(ptr[5] * 255.0f);
                    ply << " " << r << " " << g << " " << b;
                }

                ply << "\n";
            }

            ply.close();
            pImpl->logger_.info("Saved point cloud (" + std::to_string(numPoints) + " points): " + plyPath);
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